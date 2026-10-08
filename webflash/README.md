# Rainy 75 Web Flasher

A browser version of the three flashing tools, for people who build in WSL (or anywhere)
and flash from Windows or macOS where the Python tools' `hidraw` / `termios` do not
exist. Chrome or Edge only (WebHID + Web Serial). No drivers, no Python on the flashing
machine.

| Task | Transport | Python equivalent |
|---|---|---|
| Install ZMK over the stock firmware (two stages) | WebHID (stock OTA, interface 2, usage page 0xFFEF) then Web Serial (bridge, flash_mgmt group 64) | `install_zmk.sh` = `ota_flasher.py` + `restore_original.py` |
| Update a running ZMK | Web Serial (mcumgr image upload, test/confirm, reset) | `rainy75_dfu.py` |
| Restore the stock firmware | Web Serial (flash_mgmt stage + verify + commit) | `restore_stock.sh` = `restore_original.py` |

## Open it

Any of these (WebHID/Web Serial need a secure context, which all three are):

- **Local file** from WSL in a Windows browser: `\\wsl.localhost\Ubuntu\home\<you>\rainy75-zmk\webflash\index.html`
  (or copy the `webflash/` folder anywhere). `file://` is a secure context in Chrome.
- **GitHub Pages** of your fork: Settings → Pages → "Deploy from a branch", branch `main`, folder `/ (root)`,
  then `https://<user>.github.io/rainy75-zmk/webflash/`.
- **Local server**: `python -m http.server 8000` in the repo, then `http://localhost:8000/webflash/`.

The firmware files come from the WSL build; pick them in the file dialogs via the same
`\\wsl.localhost\...` path (`build-bridge/bridge_ota.bin`, `build/combined.bin`,
`build/zephyr/zmk.signed.bin`).

## Windows notes

- The stock keyboard appears in the WebHID chooser as "Rainy 75"; the page filters on the
  OTA collection (0xFFEF), the same one the official updater uses.
- ZMK enumerates two COM ports (console/mcumgr on interface 0, ZMK Studio on interface 3).
  Pick the lower one; if the page reports "no mcumgr answer", disconnect and pick the other.
  The bridge has only one port.
- Nothing to install: Windows binds its own CDC ACM driver (`usbser`) to the ports.

## Safety

The flows are the same as the shell scripts: the bridge runs from flash bank 1, the ZMK
image is staged at 0x80000 and read back before the commit, and the calibration data at
0xFE000+ is never written. Prefer `--test-image` builds and the "test boot" option for
updates: the keyboard falls back to the previous image on the next reset unless the image
is confirmed. An interrupted stage 2 leaves the bridge running; run stage 2 again.

## Files

- `proto.js`: pure protocol code (CRCs, CBOR, SMP serial framing, flash_mgmt / image
  management requests, Telink OTA packets). `test/test_proto.js` checks it byte for byte
  against vectors produced by the Python tools (`test/gen_vectors.py`, run from the repo root
  with the repo venv): `node webflash/test/test_proto.js`.
- `app.js`: WebHID / Web Serial glue and the three flows.
- `index.html`: the page.

Not yet verified on hardware from a browser; the protocol bytes are identical to the
Linux tools that are.
