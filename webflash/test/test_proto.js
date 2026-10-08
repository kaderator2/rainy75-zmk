'use strict';
// node webflash/test/test_proto.js  (vectors from gen_vectors.py)
const P = require('../proto.js');
const V = require('./vectors.json');
const crypto = require('crypto');
let failed = 0;
function check(name, cond) { if (!cond) { failed++; console.log('FAIL ' + name); } }
function framesHex(frames) { return frames.map(P.hex); }
function same(name, frames, expect) {
    const got = framesHex(frames);
    check(name, JSON.stringify(got) === JSON.stringify(expect));
}
const data256 = new Uint8Array(256).map((_, i) => (i * 7 + 3) & 0xFF);
const data420 = new Uint8Array(420).map((_, i) => (i * 13 + 1) & 0xFF);
const h = new Uint8Array(32).map((_, i) => i);
const S = P.SMP;

check('crc16 telink', P.crc16Telink(new TextEncoder().encode('123456789')) === V.crc16_telink_abc);
check('crc16 xmodem', P.crc16Xmodem(new TextEncoder().encode('123456789')) === V.crc16_xmodem_abc);
check('crc32', P.crc32(new TextEncoder().encode('123456789')) === 0xCBF43926);

same('flash_write', P.smpSerialEncode(P.smpRequest(S.OP_WRITE, S.GROUP_FLASH, 5, S.FLASH_WRITE, { off: 0x80100, data: data256 })), V.flash_write);
same('flash_erase', P.smpSerialEncode(P.smpRequest(S.OP_WRITE, S.GROUP_FLASH, 0, S.FLASH_ERASE, { off: 0x80000, len: 0x5D000 })), V.flash_erase);
same('flash_read', P.smpSerialEncode(P.smpRequest(S.OP_READ, S.GROUP_FLASH, 255, S.FLASH_READ, { off: 0, len: 4 })), V.flash_read);
same('flash_commit', P.smpSerialEncode(P.smpRequest(S.OP_WRITE, S.GROUP_FLASH, 7, S.FLASH_COMMIT, { stg: 0x80000, len: 383128 })), V.flash_commit);
same('img_upload_first', P.smpSerialEncode(P.smpRequest(S.OP_WRITE, S.GROUP_IMG, 1, S.IMG_UPLOAD, { image: 0, len: 313000, off: 0, data: data420 })), V.img_upload_first);
same('img_upload_next', P.smpSerialEncode(P.smpRequest(S.OP_WRITE, S.GROUP_IMG, 2, S.IMG_UPLOAD, { off: 420, data: data420 })), V.img_upload_next);
same('img_test', P.smpSerialEncode(P.smpRequest(S.OP_WRITE, S.GROUP_IMG, 3, S.IMG_STATE, { hash: h, confirm: false })), V.img_test);
same('img_confirm_running', P.smpSerialEncode(P.smpRequest(S.OP_WRITE, S.GROUP_IMG, 4, S.IMG_STATE, { confirm: true })), V.img_confirm_running);
same('os_reset', P.smpSerialEncode(P.smpRequest(S.OP_WRITE, S.GROUP_OS, 9, S.OS_RESET, {})), V.os_reset);
same('os_echo', P.smpSerialEncode(P.smpRequest(S.OP_WRITE, S.GROUP_OS, 10, S.OS_ECHO, { d: 'hi' })), V.os_echo);
same('img_list', P.smpSerialEncode(P.smpRequest(S.OP_READ, S.GROUP_IMG, 11, S.IMG_STATE, {})), V.img_list);

// Response stream: log line + multi-frame response with CR/LF, fed in odd pieces.
const stream = P.fromHex(V.rsp_stream);
const parser = new P.SmpParser();
let packets = [];
for (let i = 0; i < stream.length; i += 7) { packets = packets.concat(parser.feed(stream.subarray(i, i + 7))); }
check('one response', packets.length === 1);
if (packets.length === 1) {
    const r = P.smpParseResponse(packets[0]);
    check('rsp header', r.header.op === 1 && r.header.group === 1 && r.header.seq === 11 && r.header.id === 0);
    const img = r.body.images[0];
    check('rsp images', img.slot === 0 && img.version === '0.3.2' && img.active === true && img.pending === false);
    check('rsp hash', P.hex(img.hash) === V.rsp_expect.hash);
    check('rsp other', r.body.splitStatus === 0);
}
// CBOR round trip incl. indefinite map and negative ints
const rt = P.cborDecode(P.cborEncode({ a: -5, b: [1, 'x', true, null], c: new Uint8Array([1, 2]), d: 70000 })).value;
check('cbor rt', rt.a === -5 && rt.b[1] === 'x' && rt.b[2] === true && rt.b[3] === null && rt.c[1] === 2 && rt.d === 70000);
check('cbor indef', JSON.stringify(P.cborDecode(new Uint8Array([0xBF, 0x62, 0x72, 0x63, 0x00, 0xFF])).value) === '{"rc":0}');

// OTA packets vs ota_flasher capture (report id 5 in front there).
const fw = P.fromHex(V.ota_fw);
const withId = r => '05' + P.hex(r);
check('ota version', withId(P.otaVersionCmd()) === V.ota_version_pkt);
check('ota start', withId(P.otaStartCmd()) === V.ota_packets[0]);
const nseg = P.otaSegmentCount(fw.length);
check('ota nseg', nseg === 7);
let k = 1;
for (let idx = 0; idx < nseg; idx += 3, k++) {
    const count = Math.min(3, nseg - idx);
    check('ota data ' + idx, withId(P.otaDataCmd(fw, idx, count)) === V.ota_packets[k]);
}
check('ota end', withId(P.otaEndCmd(nseg - 1)) === V.ota_packets[k] && k + 1 === V.ota_packets.length);
check('ota parse ack', P.otaParseResponse(new Uint8Array([2, 3, 0, 6, 0xFF, 0])).ok === true);
check('ota parse err', P.otaParseResponse(new Uint8Array([2, 3, 0, 6, 0xFF, 0x0B])).code === 0x0B);
check('ota parse ver', P.otaParseResponse(new Uint8Array([1, 8, 0, 0x78, 0x56, 0x34, 0x12, 1, 2, 3, 4])).version === 0x12345678);

// Telink image header parse + CRC trailer on a synthetic image
const img = new Uint8Array(0x60).fill(0x11);
img[0x20] = 0x4B; img[0x21] = 0x4E; img[0x22] = 0x4C; img[0x23] = 0x54;
img[0x18] = 0x60; img[0x19] = 0; img[0x1A] = 0; img[0x1B] = 0;
const c = (P.crc32(img.subarray(0, 0x5C)) ^ 0xFFFFFFFF) >>> 0;
img[0x5C] = c & 0xFF; img[0x5D] = (c >>> 8) & 0xFF; img[0x5E] = (c >>> 16) & 0xFF; img[0x5F] = (c >>> 24) & 0xFF;
const info = P.parseTelinkImage(img);
check('tlnk parse', info.hasTlnk && info.otaSize === 0x60 && info.crcOk === true);
// staging plan like restore_original
const plan = P.stagingPlan(383128);
check('staging', plan.stg === 0x80000 && plan.eraseEnd === 0x5E000 && plan.ok);
check('staging small', P.stagingPlan(118004).stg === 0x80000);
// mcuboot hash range
const sb = new Uint8Array(64); sb.set([0x3D, 0xB8, 0xF3, 0x96, 0, 0, 0, 0, 0x20, 0, 0, 0, 0x10, 0, 0, 0]);
check('mcuboot magic', P.isMcubootImage(sb) && P.mcubootHashRange(sb).length === 0x30);

if (failed) { console.log(failed + ' FAILED'); process.exit(1); }
console.log('OK');
