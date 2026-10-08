/*
 * Copyright (c) 2026 rainy75-zmk contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Rainy 75 flashing protocols, pure JavaScript (no DOM, no Node APIs), shared
 * by the browser flasher (index.html) and the node tests (test/). Ports of
 * reverse/tools/ota_flasher.py (stock Telink HID OTA), restore_original.py
 * (mcumgr SMP serial framing, CBOR, flash_mgmt group 64) and rainy75_dfu.py
 * (MCUboot image management). Classic script, so it loads from file:// too.
 */
(function (root, factory) {
    if (typeof module === 'object' && module.exports) { module.exports = factory(); }
    else { root.Rainy75Proto = factory(); }
}(typeof self !== 'undefined' ? self : this, function () {
    'use strict';

    /* ---------------------------------------------------------------- CRC */

    /* Telink OTA segment CRC: poly 0xA001 (reflected 0x8005), init 0xFFFF. */
    function crc16Telink(bytes) {
        let crc = 0xFFFF;
        for (let i = 0; i < bytes.length; i++) {
            let b = bytes[i];
            for (let k = 0; k < 8; k++) {
                crc = (crc >>> 1) ^ (((crc ^ b) & 1) ? 0xA001 : 0);
                b >>>= 1;
            }
        }
        return crc;
    }

    /* SMP serial CRC: CRC-16/XMODEM (poly 0x1021, init 0, no reflection). */
    function crc16Xmodem(bytes, init) {
        let crc = init || 0;
        for (let i = 0; i < bytes.length; i++) {
            crc ^= bytes[i] << 8;
            for (let k = 0; k < 8; k++) {
                crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
                crc &= 0xFFFF;
            }
        }
        return crc;
    }

    /* CRC-32 (zlib). The Telink OTA trailer is crc32(body) ^ 0xFFFFFFFF. */
    const CRC32_TABLE = (function () {
        const t = new Uint32Array(256);
        for (let n = 0; n < 256; n++) {
            let c = n;
            for (let k = 0; k < 8; k++) { c = (c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1); }
            t[n] = c >>> 0;
        }
        return t;
    }());
    function crc32(bytes) {
        let c = 0xFFFFFFFF;
        for (let i = 0; i < bytes.length; i++) { c = CRC32_TABLE[(c ^ bytes[i]) & 0xFF] ^ (c >>> 8); }
        return (c ^ 0xFFFFFFFF) >>> 0;
    }

    /* --------------------------------------------------------------- bytes */

    function concat(parts) {
        let n = 0;
        for (const p of parts) { n += p.length; }
        const out = new Uint8Array(n);
        let o = 0;
        for (const p of parts) { out.set(p, o); o += p.length; }
        return out;
    }
    function hex(bytes) {
        return Array.from(bytes, b => b.toString(16).padStart(2, '0')).join('');
    }
    function fromHex(s) {
        const out = new Uint8Array(s.length / 2);
        for (let i = 0; i < out.length; i++) { out[i] = parseInt(s.substr(2 * i, 2), 16); }
        return out;
    }
    function u16le(b, o) { return b[o] | (b[o + 1] << 8); }
    function u32le(b, o) { return (b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24)) >>> 0; }
    function bytesEqual(a, b) {
        if (a.length !== b.length) { return false; }
        for (let i = 0; i < a.length; i++) { if (a[i] !== b[i]) { return false; } }
        return true;
    }
    function b64encode(bytes) {
        let s = '';
        for (let i = 0; i < bytes.length; i++) { s += String.fromCharCode(bytes[i]); }
        return btoa(s);
    }
    function b64decode(str) {
        const s = atob(str);
        const out = new Uint8Array(s.length);
        for (let i = 0; i < s.length; i++) { out[i] = s.charCodeAt(i); }
        return out;
    }

    /* ---------------------------------------------------------------- CBOR */

    function cborHead(major, n) {
        const m = major << 5;
        if (n <= 23) { return [m | n]; }
        if (n <= 0xFF) { return [m | 24, n]; }
        if (n <= 0xFFFF) { return [m | 25, n >>> 8, n & 0xFF]; }
        return [m | 26, (n >>> 24) & 0xFF, (n >>> 16) & 0xFF, (n >>> 8) & 0xFF, n & 0xFF];
    }

    /* JS value -> CBOR (RFC 8949): integers, booleans, null, strings (text),
     * Uint8Array (bytes), arrays, plain objects (maps with text keys, in
     * insertion order, which is what the Python tools emit). */
    function cborEncode(v) {
        if (v === null || v === undefined) { return new Uint8Array([0xF6]); }
        if (v === true) { return new Uint8Array([0xF5]); }
        if (v === false) { return new Uint8Array([0xF4]); }
        if (typeof v === 'number') {
            if (!Number.isInteger(v)) { throw new Error('cbor: floats not supported'); }
            return new Uint8Array(v >= 0 ? cborHead(0, v) : cborHead(1, -1 - v));
        }
        if (typeof v === 'string') {
            const b = new TextEncoder().encode(v);
            return concat([new Uint8Array(cborHead(3, b.length)), b]);
        }
        if (v instanceof Uint8Array) {
            return concat([new Uint8Array(cborHead(2, v.length)), v]);
        }
        if (Array.isArray(v)) {
            return concat([new Uint8Array(cborHead(4, v.length))].concat(v.map(cborEncode)));
        }
        if (typeof v === 'object') {
            const keys = Object.keys(v);
            const parts = [new Uint8Array(cborHead(5, keys.length))];
            for (const k of keys) { parts.push(cborEncode(k), cborEncode(v[k])); }
            return concat(parts);
        }
        throw new Error('cbor: unsupported value ' + typeof v);
    }

    function cborDecode(bytes, offset) {
        let o = offset || 0;
        const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);

        function arg(minor) {
            if (minor <= 23) { return minor; }
            if (minor === 24) { return bytes[o++]; }
            if (minor === 25) { const v = dv.getUint16(o); o += 2; return v; }
            if (minor === 26) { const v = dv.getUint32(o); o += 4; return v; }
            if (minor === 27) {
                const hi = dv.getUint32(o), lo = dv.getUint32(o + 4); o += 8;
                return hi * 0x100000000 + lo;
            }
            if (minor === 31) { return -1; }   /* indefinite */
            throw new Error('cbor: bad minor ' + minor);
        }
        function item() {
            if (o >= bytes.length) { throw new Error('cbor: truncated'); }
            const ib = bytes[o++];
            const major = ib >> 5, minor = ib & 0x1F;
            switch (major) {
            case 0: return arg(minor);
            case 1: return -1 - arg(minor);
            case 2: { const n = arg(minor); const v = bytes.slice(o, o + n); o += n; return v; }
            case 3: { const n = arg(minor); const v = new TextDecoder().decode(bytes.subarray(o, o + n)); o += n; return v; }
            case 4: {
                const n = arg(minor); const a = [];
                if (n < 0) { while (o < bytes.length && bytes[o] !== 0xFF) { a.push(item()); } o++; }
                else { for (let i = 0; i < n; i++) { a.push(item()); } }
                return a;
            }
            case 5: {
                const n = arg(minor); const m = {};
                if (n < 0) { while (o < bytes.length && bytes[o] !== 0xFF) { const k = item(); m[k] = item(); } o++; }
                else { for (let i = 0; i < n; i++) { const k = item(); m[k] = item(); } }
                return m;
            }
            case 7:
                if (minor === 20) { return false; }
                if (minor === 21) { return true; }
                if (minor === 22 || minor === 23) { return null; }
                if (minor === 26) { const v = dv.getFloat32(o); o += 4; return v; }
                if (minor === 27) { const v = dv.getFloat64(o); o += 8; return v; }
                throw new Error('cbor: unsupported simple ' + minor);
            default:
                throw new Error('cbor: unsupported major ' + major);
            }
        }
        const value = item();
        return { value, offset: o };
    }

    /* ----------------------------------------------------------------- SMP */

    const SMP = {
        HDR_SIZE: 8,
        FRAME_FIRST: [0x06, 0x09],
        FRAME_CONT: [0x04, 0x14],
        FRAME_MAX: 127,
        OP_READ: 0, OP_READ_RSP: 1, OP_WRITE: 2, OP_WRITE_RSP: 3,
        GROUP_OS: 0, GROUP_IMG: 1, GROUP_FLASH: 64,
        OS_ECHO: 0, OS_RESET: 5,
        IMG_STATE: 0, IMG_UPLOAD: 1,
        FLASH_ERASE: 0, FLASH_WRITE: 1, FLASH_READ: 2, FLASH_COMMIT: 3,
    };

    function smpHeader(op, group, seq, id, len) {
        return new Uint8Array([op, 0, (len >>> 8) & 0xFF, len & 0xFF,
                               (group >>> 8) & 0xFF, group & 0xFF, seq & 0xFF, id]);
    }
    function smpParseHeader(pkt) {
        return { op: pkt[0], flags: pkt[1], len: (pkt[2] << 8) | pkt[3],
                 group: (pkt[4] << 8) | pkt[5], seq: pkt[6], id: pkt[7] };
    }
    /* One request: header + CBOR map. */
    function smpRequest(op, group, seq, id, payloadObj) {
        const body = cborEncode(payloadObj || {});
        return concat([smpHeader(op, group, seq, id, body.length), body]);
    }

    /* Raw SMP packet -> serial frames (each a Uint8Array ending in '\n'):
     * base64 of [len(2, big endian, packet+CRC) | packet | CRC16(2)], cut
     * into 124-char chunks, the first behind 06 09, the rest behind 04 14. */
    function smpSerialEncode(packet) {
        const crc = crc16Xmodem(packet);
        const total = packet.length + 2;
        const full = concat([new Uint8Array([total >>> 8, total & 0xFF]), packet,
                             new Uint8Array([crc >>> 8, crc & 0xFF])]);
        const b64 = b64encode(full);
        const per = SMP.FRAME_MAX - 3;
        const frames = [];
        for (let pos = 0; pos < b64.length; pos += per) {
            const chunk = b64.substr(pos, per);
            const head = pos === 0 ? SMP.FRAME_FIRST : SMP.FRAME_CONT;
            const f = new Uint8Array(2 + chunk.length + 1);
            f[0] = head[0]; f[1] = head[1];
            for (let i = 0; i < chunk.length; i++) { f[2 + i] = chunk.charCodeAt(i); }
            f[f.length - 1] = 0x0A;
            frames.push(f);
        }
        return frames;
    }

    /* Incremental decoder for the serial side: feed() raw bytes, get complete
     * SMP packets back. Log lines on the shared console port are ignored. */
    function SmpParser() {
        this.lineBuf = [];
        this.frames = [];
    }
    SmpParser.prototype.feed = function (bytes) {
        const packets = [];
        for (let i = 0; i < bytes.length; i++) {
            const b = bytes[i];
            if (b !== 0x0A) { this.lineBuf.push(b); if (this.lineBuf.length > 4096) { this.lineBuf = []; } continue; }
            const line = this.lineBuf; this.lineBuf = [];
            if (line.length < 2) { continue; }
            const isFirst = line[0] === 0x06 && line[1] === 0x09;
            const isCont = line[0] === 0x04 && line[1] === 0x14;
            if (isFirst) { this.frames = []; }
            else if (!isCont || this.frames.length === 0) { continue; }
            let text = '';
            for (let k = 2; k < line.length; k++) { if (line[k] !== 0x0D) { text += String.fromCharCode(line[k]); } }
            this.frames.push(text);
            let decoded;
            try { decoded = b64decode(this.frames.join('')); } catch (e) { continue; }
            if (decoded.length < 4) { continue; }
            const pktLen = (decoded[0] << 8) | decoded[1];
            if (decoded.length < 2 + pktLen) { continue; }
            const packet = decoded.slice(2, 2 + pktLen - 2);
            const crcRecv = (decoded[pktLen] << 8) | decoded[pktLen + 1];
            this.frames = [];
            if (crcRecv !== crc16Xmodem(packet)) { throw new Error('SMP CRC mismatch'); }
            packets.push(packet);
        }
        return packets;
    };

    /* Response packet -> { header, body } (CBOR decoded, {} when empty). */
    function smpParseResponse(pkt) {
        if (pkt.length < SMP.HDR_SIZE) { throw new Error('SMP response too short'); }
        const header = smpParseHeader(pkt);
        const bodyBytes = pkt.subarray(SMP.HDR_SIZE, SMP.HDR_SIZE + header.len);
        const body = bodyBytes.length ? cborDecode(bodyBytes).value : {};
        return { header, body };
    }

    /* ----------------------------------------------------- flash / images */

    const FLASH = {
        SECTOR: 4096,
        WRITE_CHUNK: 256,
        PROTECTED_START: 0xFE000,   /* RF/ADC calibration + MAC, never touched */
        MIN_STAGING: 0x80000,       /* above the running app's XIP code */
    };

    /* Staging plan for restore_original.py's stage+verify+commit flow. */
    function stagingPlan(fwSize) {
        const eraseEnd = Math.ceil(fwSize / FLASH.SECTOR) * FLASH.SECTOR;
        const stg = Math.max(eraseEnd, FLASH.MIN_STAGING);
        const ok = stg + fwSize <= FLASH.PROTECTED_START;
        return { eraseEnd, stg, stgEraseSize: eraseEnd, ok };
    }

    const IMG = { CHUNK: 420, MCUBOOT_MAGIC: 0x96F3B83D };

    /* The hash MCUboot reports: SHA-256 over header + image body, without
     * the trailing TLVs. Returns the byte range to hash. */
    function mcubootHashRange(data) {
        const hdrSize = u16le(data, 8);
        const imgSize = u32le(data, 12);
        return data.subarray(0, hdrSize + imgSize);
    }
    function isMcubootImage(data) {
        return data.length > 32 && u32le(data, 0) === IMG.MCUBOOT_MAGIC;
    }

    /* ------------------------------------------------------- Telink OTA */

    const OTA = {
        VID: 0x320F, PID: 0x5055, USAGE_PAGE: 0xFFEF, REPORT_ID: 0x05,
        REPORT_LEN: 63,     /* bytes after the report id */
        SEG_DATA: 16, SEG_LEN: 20, SEGS_PER_PACKET: 3,
        ERRORS: { 3: 'CRC error', 4: 'flash write error', 0x0B: 'size / alignment error', 0x0D: 'timeout' },
    };

    /* Pad a command to the 63-byte report body with 0xFF (0x00 padding makes
     * OTA END fail with 0x0B). */
    function otaReport(cmd) {
        const r = new Uint8Array(OTA.REPORT_LEN).fill(0xFF);
        r.set(cmd, 0);
        return r;
    }
    function otaVersionCmd() { return otaReport([0x01, 0x00, 0x00]); }
    function otaStartCmd() { return otaReport([0x02, 0x02, 0x00, 0x01, 0xFF]); }
    function otaEndCmd(lastIdx) {
        const comp = (0xFFFF - lastIdx + 1) & 0xFFFF;
        return otaReport([0x02, 0x06, 0x00, 0x02, 0xFF,
                          lastIdx & 0xFF, (lastIdx >>> 8) & 0xFF, comp & 0xFF, (comp >>> 8) & 0xFF]);
    }
    /* Data packet holding segments firstIdx .. firstIdx+count-1 of fw. */
    function otaDataCmd(fw, firstIdx, count) {
        const segs = new Uint8Array(count * OTA.SEG_LEN);
        for (let s = 0; s < count; s++) {
            const idx = firstIdx + s;
            const seg = segs.subarray(s * OTA.SEG_LEN, (s + 1) * OTA.SEG_LEN);
            seg[0] = idx & 0xFF; seg[1] = (idx >>> 8) & 0xFF;
            for (let k = 0; k < OTA.SEG_DATA; k++) {
                const off = idx * OTA.SEG_DATA + k;
                seg[2 + k] = off < fw.length ? fw[off] : 0xFF;
            }
            const crc = crc16Telink(seg.subarray(0, 18));
            seg[18] = crc & 0xFF; seg[19] = (crc >>> 8) & 0xFF;
        }
        return otaReport(concat([new Uint8Array([0x02, count * OTA.SEG_LEN, 0x00]), segs]));
    }
    function otaSegmentCount(fwSize) { return Math.ceil(fwSize / OTA.SEG_DATA); }

    /* Response body (without report id): [cmd, len, 0, ...]. */
    function otaParseResponse(body) {
        if (body.length >= 11 && body[0] === 0x01 && body[1] === 0x08) {
            return { kind: 'version', version: u32le(body, 3), crc: u32le(body, 7) };
        }
        if (body.length >= 6 && body[0] === 0x02 && body[1] === 0x03 && body[4] === 0xFF) {
            return { kind: 'ack', code: body[5], ok: body[5] === 0 };
        }
        return { kind: 'other' };
    }

    /* Telink OTA image header: TLNK magic at 0x20, size (incl. CRC32 trailer)
     * at 0x18, version at 0x02. */
    function parseTelinkImage(data) {
        const info = { size: data.length, hasTlnk: false };
        if (data.length > 0x24 && data[0x20] === 0x4B && data[0x21] === 0x4E && data[0x22] === 0x4C && data[0x23] === 0x54) {
            info.hasTlnk = true;
            info.otaSize = u32le(data, 0x18);
            info.version = u16le(data, 0x02);
            if (info.otaSize <= data.length && info.otaSize >= 4) {
                info.crcTrailer = u32le(data, info.otaSize - 4);
                info.crcOk = info.crcTrailer === ((crc32(data.subarray(0, info.otaSize - 4)) ^ 0xFFFFFFFF) >>> 0);
            }
        }
        return info;
    }

    return {
        crc16Telink, crc16Xmodem, crc32, concat, hex, fromHex, u16le, u32le, bytesEqual,
        b64encode, b64decode, cborEncode, cborDecode,
        SMP, smpHeader, smpParseHeader, smpRequest, smpSerialEncode, SmpParser, smpParseResponse,
        FLASH, stagingPlan, IMG, mcubootHashRange, isMcubootImage,
        OTA, otaReport, otaVersionCmd, otaStartCmd, otaEndCmd, otaDataCmd, otaSegmentCount,
        otaParseResponse, parseTelinkImage,
    };
}));
