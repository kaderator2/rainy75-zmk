/*
 * Copyright (c) 2026 rainy75-zmk contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Rainy 75 browser flasher (Chrome / Edge): WebHID for the stock firmware's
 * OTA channel, Web Serial for the mcumgr port of the bridge / ZMK. The
 * protocol code is proto.js (tested against the Python tools, see test/).
 */
'use strict';
/* global Rainy75Proto */
(function () {
    const P = Rainy75Proto;
    const S = P.SMP;
    const $ = id => document.getElementById(id);

    /* ------------------------------------------------------------ logging */
    const logEl = $('log');
    function log(msg, cls) {
        const t = new Date();
        const stamp = t.toTimeString().slice(0, 8);
        const line = document.createElement('div');
        if (cls) { line.className = cls; }
        line.textContent = stamp + '  ' + msg;
        logEl.appendChild(line);
        logEl.scrollTop = logEl.scrollHeight;
    }
    const info = m => log(m);
    const ok = m => log(m, 'ok');
    const warn = m => log(m, 'warn');
    const err = m => log(m, 'err');

    function progress(frac, text) {
        $('bar').style.width = Math.round(Math.max(0, Math.min(1, frac)) * 100) + '%';
        $('bartext').textContent = text || '';
    }
    const sleep = ms => new Promise(r => setTimeout(r, ms));
    function hexN(n, w) { return '0x' + n.toString(16).toUpperCase().padStart(w || 0, '0'); }
    function kb(n) { return (n / 1024).toFixed(1) + ' KB'; }

    async function readFile(input) {
        const f = input.files && input.files[0];
        if (!f) { throw new Error('choose a file first'); }
        return { name: f.name, data: new Uint8Array(await f.arrayBuffer()) };
    }
    async function sha256(bytes) {
        return new Uint8Array(await crypto.subtle.digest('SHA-256', bytes));
    }

    /* ---------------------------------------------------- busy / buttons */
    let busy = false;
    function setBusy(b) {
        busy = b;
        document.querySelectorAll('button[data-task]').forEach(el => { el.disabled = b; });
        document.body.classList.toggle('busy', b);
    }
    async function task(name, fn) {
        if (busy) { warn('another task is running'); return; }
        setBusy(true);
        try {
            await fn();
        } catch (e) {
            err(name + ' failed: ' + (e && e.message ? e.message : e));
            console.error(e);
        } finally {
            setBusy(false);
            refreshUi();
        }
    }

    /* ======================================================= stock OTA */
    /* Replies are queued as they arrive: the keyboard answers within a
     * millisecond, so a reply can land before the sendReport() promise
     * resolves. A waiter registered after the send would miss it. */
    const hid = { device: null, pending: null, rx: [], trace: false };

    function hidOnReport(ev) {
        const body = new Uint8Array(ev.data.buffer, ev.data.byteOffset, ev.data.byteLength);
        if (hid.trace) { info('HID in: report ' + ev.reportId + ' ' + P.hex(body.subarray(0, 16))); }
        if (ev.reportId !== P.OTA.REPORT_ID) { return; }
        if (hid.pending) { const p = hid.pending; hid.pending = null; p.resolve(body); }
        else { hid.rx.push(body); if (hid.rx.length > 64) { hid.rx.shift(); } }
    }
    function hidWait(timeoutMs) {
        if (hid.rx.length) { return Promise.resolve(hid.rx.shift()); }
        return new Promise(resolve => {
            const timer = setTimeout(() => { if (hid.pending && hid.pending.resolve === done) { hid.pending = null; } resolve(null); }, timeoutMs);
            const done = body => { clearTimeout(timer); resolve(body); };
            hid.pending = { resolve: done };
        });
    }
    /* Send one command and return its reply (or null on timeout). Stale
     * replies from before the send are discarded; the waiter is armed
     * before the send so a fast reply cannot be missed. */
    async function hidTransact(report63, timeoutMs) {
        hid.rx = [];
        const waiting = hidWait(timeoutMs);
        await hid.device.sendReport(P.OTA.REPORT_ID, report63);
        return waiting;
    }
    async function hidConnect() {
        if (!navigator.hid) { throw new Error('WebHID is not available (use Chrome or Edge, https:// or file://)'); }
        const devs = await navigator.hid.requestDevice({ filters: [{ vendorId: P.OTA.VID, productId: P.OTA.PID, usagePage: P.OTA.USAGE_PAGE }] });
        if (!devs.length) { throw new Error('no device chosen'); }
        const d = devs[0];
        if (!d.opened) { await d.open(); }
        d.addEventListener('inputreport', hidOnReport);
        hid.device = d;
        const cols = d.collections.map(c => hexN(c.usagePage, 4) + '/' + hexN(c.usage, 2) + ' out:[' +
            c.outputReports.map(r => r.reportId).join(',') + '] in:[' + c.inputReports.map(r => r.reportId).join(',') + ']');
        info('HID: ' + d.productName + ' collections ' + cols.join(' '));
        const hasOta = d.collections.some(c => c.usagePage === P.OTA.USAGE_PAGE && c.outputReports.some(r => r.reportId === P.OTA.REPORT_ID));
        if (!hasOta) { warn('no 0xFFEF collection with output report 5 visible: OTA will probably fail'); }
        ok('stock keyboard connected');
    }
    function hidDisconnect() {
        if (hid.device) { try { hid.device.close(); } catch (e) { /* ignore */ } }
        hid.device = null;
    }
    async function otaVersion() {
        hid.trace = true;
        let r;
        try { r = await hidTransact(P.otaVersionCmd(), 3000); } finally { hid.trace = false; }
        if (!r) { throw new Error('no version reply within 3 s. Is the keyboard in USB mode (Fn+Tab on stock cycles the mode) and nothing else (VIA, the Wobkey updater) holding it?'); }
        const v = P.otaParseResponse(r);
        if (v.kind !== 'version') { throw new Error('unexpected reply ' + P.hex(r.subarray(0, 12))); }
        ok('stock firmware version ' + hexN(v.version, 8) + ', CRC ' + hexN(v.crc, 8));
        return v;
    }
    /* Port of ota_flasher.flash_firmware(). */
    async function otaFlash(fw) {
        const nseg = P.otaSegmentCount(fw.length);
        info('OTA: ' + fw.length + ' bytes, ' + nseg + ' segments');
        const startAck = await hidTransact(P.otaStartCmd(), 3000);
        if (!startAck) { warn('no start acknowledgment, continuing'); }
        const t0 = performance.now();
        let missed = 0;
        for (let idx = 0; idx < nseg; idx += P.OTA.SEGS_PER_PACKET) {
            const count = Math.min(P.OTA.SEGS_PER_PACKET, nseg - idx);
            const r = await hidTransact(P.otaDataCmd(fw, idx, count), 2000);
            if (r) {
                const a = P.otaParseResponse(r);
                if (a.kind === 'ack' && !a.ok) {
                    throw new Error('OTA error at segment ' + idx + ': code ' + a.code + ' (' + (P.OTA.ERRORS[a.code] || '?') + ')');
                }
            } else { missed++; }
            if ((idx / P.OTA.SEGS_PER_PACKET) % 20 === 0) {
                const done = Math.min(1, (idx + count) / nseg);
                const el = (performance.now() - t0) / 1000;
                progress(done, 'OTA ' + Math.round(done * 100) + ' %  ' + el.toFixed(0) + ' s');
            }
        }
        if (missed) { warn(missed + ' data packets got no acknowledgment'); }
        const r = await hidTransact(P.otaEndCmd(nseg - 1), 10000);
        info('OTA end sent, waiting for the keyboard');
        progress(1, 'OTA done');
        if (r) {
            const a = P.otaParseResponse(r);
            if (a.kind === 'ack' && !a.ok) { throw new Error('OTA END refused: code ' + a.code + ' (' + (P.OTA.ERRORS[a.code] || '?') + ')'); }
            if (a.kind === 'ack') { ok('OTA accepted in ' + ((performance.now() - t0) / 1000).toFixed(1) + ' s, the keyboard reboots'); return; }
        }
        warn('no final reply (the keyboard may already be rebooting)');
    }

    /* ========================================================= SMP serial */
    const ser = { port: null, reader: null, writer: null, parser: null, seq: 0, pending: null, readLoop: null };
    /* Console echo: the mcumgr port is also ZMK's log console; with the box
     * ticked every non-SMP line is shown (boot log, USB / HID messages). */
    function consoleLine(text) {
        if ($('c-console').checked && text.trim()) { log('  | ' + text, 'con'); }
    }
    function newParser() { return new P.SmpParser(consoleLine); }

    async function serialConnect() {
        if (!navigator.serial) { throw new Error('Web Serial is not available (use Chrome or Edge, https:// or file://)'); }
        const port = await navigator.serial.requestPort({ filters: [{ usbVendorId: 0x1D50, usbProductId: 0x615E }] });
        await port.open({ baudRate: 115200, bufferSize: 16384 });
        try { await port.setSignals({ dataTerminalReady: true, requestToSend: true }); } catch (e) { /* optional */ }
        ser.port = port;
        ser.parser = newParser();
        ser.writer = port.writable.getWriter();
        ser.readLoop = (async () => {
            try {
                while (port.readable) {
                    ser.reader = port.readable.getReader();
                    try {
                        for (;;) {
                            const { value, done } = await ser.reader.read();
                            if (done) { break; }
                            let pkts;
                            try { pkts = ser.parser.feed(value); } catch (e) { warn('serial: ' + e.message); ser.parser = newParser(); continue; }
                            for (const pkt of pkts) {
                                if (ser.pending) { const p = ser.pending; ser.pending = null; p.resolve(pkt); }
                            }
                        }
                    } catch (e) {
                        warn('serial read: ' + e.message);
                    } finally {
                        try { ser.reader.releaseLock(); } catch (e) { /* ignore */ }
                    }
                    break;
                }
            } finally {
                if (ser.port === port) { await serialDrop('port closed'); }
            }
        })();
        ok('serial port opened');
    }
    async function serialDrop(why) {
        const port = ser.port;
        ser.port = null;
        if (ser.pending) { const p = ser.pending; ser.pending = null; p.reject(new Error('port closed')); }
        if (ser.writer) { try { ser.writer.releaseLock(); } catch (e) { /* ignore */ } ser.writer = null; }
        if (ser.reader) { try { await ser.reader.cancel(); } catch (e) { /* ignore */ } ser.reader = null; }
        if (port) { try { await port.close(); } catch (e) { /* ignore */ } }
        if (why) { info('serial: ' + why); }
        refreshUi();
    }
    async function serialDisconnect() { await serialDrop('disconnected'); }

    function smpWait(timeoutMs) {
        return new Promise((resolve, reject) => {
            const timer = setTimeout(() => { if (ser.pending && ser.pending.resolve === done) { ser.pending = null; } reject(new Error('timeout')); }, timeoutMs);
            const done = pkt => { clearTimeout(timer); resolve(pkt); };
            ser.pending = { resolve: done, reject: e => { clearTimeout(timer); reject(e); } };
        });
    }
    /* One SMP request (port of SMPClient.send / Dfu.request): frames go out
     * back to back (USB has its own flow control), retried on timeout. */
    async function smp(op, group, id, payload, opts) {
        opts = opts || {};
        if (!ser.port) { throw new Error('serial port not connected'); }
        const timeout = opts.timeout || 5000;
        const retries = opts.retries === undefined ? 2 : opts.retries;
        const seq = ser.seq; ser.seq = (ser.seq + 1) & 0xFF;
        const frames = P.smpSerialEncode(P.smpRequest(op, group, seq, id, payload || {}));
        for (let attempt = 0; ; attempt++) {
            ser.parser = newParser();
            const waiting = smpWait(timeout);
            for (const f of frames) { await ser.writer.write(f); }
            try {
                const pkt = await waiting;
                return P.smpParseResponse(pkt).body;
            } catch (e) {
                if (e.message !== 'timeout' || attempt >= retries) { throw e; }
                await sleep(100);
            }
        }
    }
    const rcOf = r => (r && r.rc !== undefined) ? r.rc : 0;

    async function smpProbe() {
        try {
            const r = await smp(S.OP_WRITE, S.GROUP_OS, S.OS_ECHO, { d: 'rainy75' }, { timeout: 1500, retries: 1 });
            if (r.r !== 'rainy75') { throw new Error('bad echo'); }
        } catch (e) {
            throw new Error('no mcumgr answer on this port. ZMK has two COM ports: this is probably the Studio one, disconnect and pick the other (usually the lower number).');
        }
        ok('mcumgr answers');
        try {
            const imgs = await imgList();
            printImages(imgs);
        } catch (e) {
            const r = await smp(S.OP_READ, S.GROUP_FLASH, S.FLASH_READ, { off: 0, len: 4 }, { timeout: 2000, retries: 0 });
            if (rcOf(r) === 0) { info('no image group: this is the OTA bridge (flash[0..4] = ' + P.hex(r.data) + ')'); }
        }
    }

    /* flash_mgmt group 64 */
    async function flashErase(off, len) {
        const t = Math.max(30000, Math.ceil(len / P.FLASH.SECTOR) * 150);
        const r = await smp(S.OP_WRITE, S.GROUP_FLASH, S.FLASH_ERASE, { off, len }, { timeout: t });
        if (rcOf(r) !== 0) { throw new Error('erase failed rc=' + rcOf(r)); }
    }
    async function flashWrite(off, data) {
        const r = await smp(S.OP_WRITE, S.GROUP_FLASH, S.FLASH_WRITE, { off, data });
        if (rcOf(r) !== 0) { throw new Error('write failed at ' + hexN(off) + ' rc=' + rcOf(r)); }
    }
    async function flashRead(off, len) {
        /* restore_original.py waits 10 s per read; the bridge's serial ring
         * is small and its mcumgr transport drops a reply it cannot queue. */
        const r = await smp(S.OP_READ, S.GROUP_FLASH, S.FLASH_READ, { off, len }, { timeout: 10000, retries: 3 });
        if (rcOf(r) !== 0) { throw new Error('read failed at ' + hexN(off) + ' rc=' + rcOf(r)); }
        return r.data;
    }
    async function flashCommit(stg, len) {
        const r = await smp(S.OP_WRITE, S.GROUP_FLASH, S.FLASH_COMMIT, { stg, len }, { timeout: 5000 });
        if (rcOf(r) !== 0) { throw new Error('commit failed rc=' + rcOf(r)); }
    }
    /* Port of restore_original.restore_firmware(): stage at >= 0x80000,
     * verify, commit (RAM trampoline erases 0x0, copies, resets). */
    /* What is in the staging area right now (this page session): lets a
     * second run after a USB stall skip the write and continue the verify.
     * Nothing is trusted from it: the verify reads every byte anyway. */
    const staged = { hash: null, verified: 0 };

    async function stageAndCommit(fw, verify) {
        const plan = P.stagingPlan(fw.length);
        if (!plan.ok) { throw new Error('image too large for the staging area'); }
        if (fw.length >= P.FLASH.PROTECTED_START) { throw new Error('image too large'); }
        const fwHash = P.hex(await sha256(fw));
        const chunks = Math.ceil(fw.length / P.FLASH.WRITE_CHUNK);
        info('image ' + fw.length + ' bytes, erase 0..' + hexN(plan.eraseEnd) + ', staging at ' + hexN(plan.stg));
        const probe = await flashRead(0, 4);
        info('connected, flash[0..4] = ' + P.hex(probe));
        const resume = staged.hash === fwHash;
        if (resume) {
            info('this image was already staged in this session: skipping the write, verifying from chunk ' + staged.verified + '/' + chunks);
        } else {
            staged.hash = null; staged.verified = 0;
            info('erasing staging area (' + (plan.stgEraseSize / 1024) + ' KB, ~' + Math.ceil(plan.stgEraseSize / P.FLASH.SECTOR * 0.015 + 1) + ' s)');
            progress(0, 'erasing');
            await flashErase(plan.stg, plan.stgEraseSize);
            const t0 = performance.now();
            for (let i = 0; i < chunks; i++) {
                const off = i * P.FLASH.WRITE_CHUNK;
                await flashWrite(plan.stg + off, fw.subarray(off, off + P.FLASH.WRITE_CHUNK));
                if (i % 8 === 0 || i === chunks - 1) {
                    const el = (performance.now() - t0) / 1000;
                    progress((i + 1) / chunks, 'writing ' + Math.round((i + 1) * 100 / chunks) + ' %  ' + el.toFixed(0) + ' s');
                }
            }
            ok('staged in ' + ((performance.now() - t0) / 1000).toFixed(1) + ' s');
            staged.hash = fwHash; staged.verified = 0;
        }
        if (verify) {
            const t1 = performance.now();
            for (let i = staged.verified; i < chunks; i++) {
                const off = i * P.FLASH.WRITE_CHUNK;
                const expect = fw.subarray(off, off + P.FLASH.WRITE_CHUNK);
                let got;
                try {
                    got = await flashRead(plan.stg + off, expect.length);
                } catch (e) {
                    staged.verified = i;
                    throw new Error(e.message + ' while verifying chunk ' + i + '/' + chunks + '. Nothing was committed and the staged data stays in flash: unplug the keyboard, plug it back in (the bridge boots again), Disconnect, Connect to its port and press Stage 2 again; the verify continues from here.');
                }
                if (!P.bytesEqual(got, expect)) { staged.hash = null; throw new Error('verify mismatch at staging ' + hexN(plan.stg + off) + ' (nothing committed, the running firmware is intact, run Stage 2 again to re-stage)'); }
                staged.verified = i + 1;
                if (i % 8 === 0 || i === chunks - 1) { progress((i + 1) / chunks, 'verifying ' + Math.round((i + 1) * 100 / chunks) + ' %'); }
            }
            ok('verified in ' + ((performance.now() - t1) / 1000).toFixed(1) + ' s');
        }
        staged.hash = null; staged.verified = 0;
        info('committing: erase 0..' + hexN(plan.eraseEnd) + ', copy ' + hexN(plan.stg) + ' -> 0, reset');
        await flashCommit(plan.stg, fw.length);
        ok('commit accepted: the keyboard erases, copies and resets itself in a few seconds. Do not unplug.');
        progress(1, 'committed');
    }

    /* image management (group 1) */
    async function imgList() {
        const r = await smp(S.OP_READ, S.GROUP_IMG, S.IMG_STATE, {}, { timeout: 3000, retries: 1 });
        if (r.rc !== undefined && r.rc !== 0) { throw new Error('image list rc=' + r.rc); }
        if (!r.images) { throw new Error('no images field'); }
        return r.images;
    }
    function printImages(images) {
        for (const i of images) {
            const flags = ['active', 'confirmed', 'pending', 'permanent'].filter(f => i[f]);
            info('slot ' + i.slot + ': ' + (i.version || '?') + '  ' + P.hex(i.hash || new Uint8Array()).slice(0, 16) + '  ' + flags.join(' '));
        }
    }
    async function imgUpload(data) {
        const t0 = performance.now();
        let off = 0;
        while (off < data.length) {
            const payload = off === 0 ? { image: 0, len: data.length } : {};
            payload.off = off;
            payload.data = data.subarray(off, off + P.IMG.CHUNK);
            const r = await smp(S.OP_WRITE, S.GROUP_IMG, S.IMG_UPLOAD, payload, { timeout: off === 0 ? 30000 : 5000 });
            if (rcOf(r) !== 0) { throw new Error('upload refused at ' + off + ': rc=' + rcOf(r)); }
            off = r.off !== undefined ? r.off : off + P.IMG.CHUNK;
            if (off % (P.IMG.CHUNK * 10) === 0 || off >= data.length) {
                const el = (performance.now() - t0) / 1000;
                progress(off / data.length, 'upload ' + Math.round(off * 100 / data.length) + ' %  ' + el.toFixed(0) + ' s');
            }
        }
        const dt = (performance.now() - t0) / 1000;
        ok('uploaded ' + data.length + ' bytes in ' + dt.toFixed(1) + ' s (' + (data.length / dt / 1024).toFixed(1) + ' KiB/s)');
    }
    async function imgSetState(hash, confirm) {
        const payload = {};
        if (hash) { payload.hash = hash; }
        payload.confirm = confirm;
        const r = await smp(S.OP_WRITE, S.GROUP_IMG, S.IMG_STATE, payload);
        if (rcOf(r) !== 0) { throw new Error('image state refused rc=' + rcOf(r)); }
        return r.images || [];
    }
    async function osReset() {
        try { await smp(S.OP_WRITE, S.GROUP_OS, S.OS_RESET, {}, { timeout: 2000, retries: 0 }); }
        catch (e) { /* the reset may beat the response */ }
        ok('reset sent');
    }

    /* ============================================================== flows */
    const flow = { combined: null };

    /* Known stock OTA images (SHA-256), so the restore tab can say which board
     * an image is for. Flashing the other layout's image is not a brick (the
     * keymap is wrong), but it is a mistake the page can catch. */
    const STOCK_IMAGES = {
        '1c50213601fb1520eead96d8d3a841fd6cfe006ef5d7da35463e5073ce9d7c41': 'Rainy 75 Standard/Pro RGB, ANSI, 2024-01-21',
        '37d2b846ef022721542cc795cc2258a3ec1652e0d21c79f6688642d09ec278bc': 'Rainy 75 Pro ISO, 2025-05-30',
    };
    /* combined.bin = MCUboot at 0, 0xFF padding, the signed app at 0x10000. */
    const COMBINED_APP_OFFSET = 0x10000;
    function checkCombined(file) {
        const d = file.data;
        if (P.isMcubootImage(d)) { throw new Error(file.name + ' is a signed app image, not combined.bin'); }
        if (d.length <= COMBINED_APP_OFFSET + 32 || P.u32le(d, COMBINED_APP_OFFSET) !== P.IMG.MCUBOOT_MAGIC) {
            throw new Error(file.name + ' has no MCUboot app image at 0x10000: not a combined.bin from ./build.sh -c / -a');
        }
        if (!P.parseTelinkImage(d).hasTlnk) { throw new Error(file.name + ' has no TLNK boot header at 0x20: the boot ROM would not start it'); }
        if (d.length >= P.FLASH.PROTECTED_START) { throw new Error('combined image too large'); }
    }

    async function installStage1() {
        const bridge = await readFile($('f-bridge'));
        const combined = await readFile($('f-combined'));
        const bi = P.parseTelinkImage(bridge.data);
        if (!bi.hasTlnk || !bi.crcOk) { throw new Error(bridge.name + ' is not a prepared OTA image (bridge_ota.bin from build-bridge/)'); }
        if (bridge.data.length > 256 * 1024) { throw new Error('bridge image too large for bank 1'); }
        checkCombined(combined);
        flow.combined = combined;
        info('bridge ' + bridge.name + ' (' + kb(bridge.data.length) + '), ZMK ' + combined.name + ' (' + kb(combined.data.length) + ')');
        if (!hid.device) { await hidConnect(); }
        try { await otaVersion(); } catch (e) { warn(e.message); }
        await otaFlash(bridge.data);
        hidDisconnect();
        ok('Stage 1 done. Wait ~5 s for the keyboard to come back as "Rainy 75 Bridge" (one COM port), then press "Stage 2".');
        $('btn-install-2').disabled = false;
    }
    async function installStage2() {
        if (!flow.combined) {
            const combined = await readFile($('f-combined'));
            checkCombined(combined);
            flow.combined = combined;
        }
        if (!ser.port) { await serialConnect(); }
        await stageAndCommit(flow.combined.data, $('c-verify-install').checked);
        ok('Stage 2 done. The keyboard reboots into ZMK (MCUboot swap, about 10 s). Done!');
        flow.combined = null;
    }
    async function updateZmk() {
        const img = await readFile($('f-signed'));
        if (!P.isMcubootImage(img.data)) { throw new Error(img.name + ' has no MCUboot header (use build/zephyr/zmk.signed.bin)'); }
        const want = await sha256(P.mcubootHashRange(img.data));
        info(img.name + ' (' + kb(img.data.length) + '), hash ' + P.hex(want).slice(0, 16));
        if (!ser.port) { await serialConnect(); }
        const before = await imgList();
        printImages(before);
        if (before.some(i => i.slot === 0 && !i.confirmed)) {
            throw new Error('the running image is not confirmed yet (a test image): confirm it first or reset to go back, never upload on top of it');
        }
        await imgUpload(img.data);
        const after = await imgList();
        const slot1 = after.find(i => i.slot === 1);
        if (!slot1 || !P.bytesEqual(slot1.hash, want)) { throw new Error('slot 1 does not hold the uploaded image (hash mismatch)'); }
        ok('slot 1 holds ' + P.hex(want).slice(0, 16));
        if ($('c-test').checked) {
            await imgSetState(want, false);
            ok('marked for a test boot: it reverts on the next reset unless confirmed');
        } else {
            await imgSetState(want, true);
            warn('marked permanent: no automatic fallback');
        }
        if ($('c-reset').checked) {
            await osReset();
            info('MCUboot swaps the images (about 10-20 s), the port disappears and comes back');
        }
    }
    async function restoreStock() {
        const fw = await readFile($('f-stock'));
        const fi = P.parseTelinkImage(fw.data);
        if (!fi.hasTlnk) { throw new Error(fw.name + ' has no TLNK header: not a Telink firmware image'); }
        if (fi.crcOk === false) { throw new Error(fw.name + ': CRC32 trailer does not match'); }
        if (fw.data.length >= P.FLASH.PROTECTED_START) { throw new Error('image too large'); }
        info(fw.name + ' (' + kb(fw.data.length) + '), version ' + hexN(fi.version || 0, 4) + ', CRC ok');
        const digest = P.hex(await sha256(fw.data));
        const known = STOCK_IMAGES[digest];
        if (known) { ok('recognized stock image: ' + known); }
        else { warn('unknown image (SHA-256 ' + digest.slice(0, 16) + '…): make sure it is the stock OTA image for YOUR board'); }
        if (known && /ISO/.test(known) && !confirm('This is the ISO stock image. On an ANSI board the keys around Enter will be wrong (not a brick). Continue?')) {
            throw new Error('cancelled');
        }
        if (!ser.port) { await serialConnect(); }
        await stageAndCommit(fw.data, $('c-verify-stock').checked);
        ok('The keyboard comes back as the stock firmware (320F:5055) in a few seconds.');
    }

    /* ================================================================ UI */
    function refreshUi() {
        $('hid-state').textContent = hid.device ? 'connected: ' + hid.device.productName : 'not connected';
        $('ser-state').textContent = ser.port ? 'connected' : 'not connected';
        $('btn-ser-disc').disabled = !ser.port;
        $('btn-hid-disc').disabled = !hid.device;
    }
    function bind(id, name, fn) { $(id).addEventListener('click', () => task(name, fn)); }

    bind('btn-hid-conn', 'connect', async () => { await hidConnect(); });
    bind('btn-hid-ver', 'version', async () => { if (!hid.device) { await hidConnect(); } await otaVersion(); });
    $('btn-hid-disc').addEventListener('click', () => { hidDisconnect(); refreshUi(); });
    bind('btn-ser-conn', 'connect', async () => { await serialConnect(); await smpProbe(); });
    $('btn-ser-disc').addEventListener('click', () => { serialDisconnect(); });
    bind('btn-install-1', 'install stage 1', installStage1);
    bind('btn-install-2', 'install stage 2', installStage2);
    bind('btn-update', 'update', updateZmk);
    bind('btn-restore', 'restore stock', restoreStock);
    bind('btn-list', 'list', async () => { if (!ser.port) { await serialConnect(); } printImages(await imgList()); });
    bind('btn-confirm', 'confirm', async () => { if (!ser.port) { await serialConnect(); } printImages(await imgSetState(null, true)); ok('running image confirmed'); });
    bind('btn-reset', 'reset', async () => { if (!ser.port) { await serialConnect(); } await osReset(); });
    bind('btn-test1', 'test slot 1', async () => {
        if (!ser.port) { await serialConnect(); }
        const imgs = await imgList();
        printImages(imgs);
        const slot1 = imgs.find(i => i.slot === 1);
        if (!slot1 || !slot1.hash) { throw new Error('no image in slot 1'); }
        if (imgs.some(i => i.slot === 0 && !i.confirmed)) { throw new Error('the running image is not confirmed; confirm it or reset first'); }
        await imgSetState(slot1.hash, false);
        ok('slot 1 (' + P.hex(slot1.hash).slice(0, 16) + ') marked for one test boot; resetting');
        await osReset();
        info('after the swap (15-20 s), Connect again with "console echo" on to read its boot log');
    });
    bind('btn-readflash', 'read flash', async () => {
        if (!ser.port) { await serialConnect(); }
        const off = parseInt($('i-readoff').value, 16) || 0;
        const d = await flashRead(off, 32);
        info('flash ' + hexN(off, 6) + ': ' + P.hex(d));
    });
    $('btn-clear').addEventListener('click', () => { logEl.textContent = ''; progress(0, ''); });

    document.querySelectorAll('.tabs button').forEach(b => b.addEventListener('click', () => {
        document.querySelectorAll('.tabs button').forEach(x => x.classList.toggle('on', x === b));
        document.querySelectorAll('.tab').forEach(t => t.classList.toggle('on', t.id === b.dataset.tab));
    }));

    /* environment check */
    const missing = [];
    if (!window.isSecureContext) { missing.push('not a secure context (open over https://, http://localhost or file://)'); }
    if (!navigator.hid) { missing.push('WebHID missing'); }
    if (!navigator.serial) { missing.push('Web Serial missing'); }
    if (missing.length) {
        $('env').textContent = 'This browser cannot flash: ' + missing.join(', ') + '. Use Chrome or Edge on Windows, macOS or Linux.';
        $('env').className = 'env bad';
    } else {
        $('env').textContent = 'WebHID and Web Serial available.';
        $('env').className = 'env good';
    }
    if (navigator.serial) {
        navigator.serial.addEventListener('disconnect', ev => { if (ser.port === ev.target) { serialDrop('the port went away (reboot)'); } });
    }
    refreshUi();
    info('ready');
}());
