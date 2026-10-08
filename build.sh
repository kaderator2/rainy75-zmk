#!/bin/bash
# Build ZMK firmware for Rainy 75 Pro
# Usage: ./build.sh [-p] [-v] [-m] [-c] [-o] [-b] [-a] (--iso | --ansi) [--wired | --blob | --privacy] [--test-image]
#   -p  pristine build (clean rebuild)
#   -v  verbose output
#   -m  build MCUboot bootloader
#   -c  create combined flash image (MCUboot + app)
#   -o  create OTA-ready image (combined + Telink header + CRC32)
#   -b  build OTA bridge (monolithic, for stock-to-ZMK transition)
#   -a  all: MCUboot + app + combined + OTA + bridge
#   --iso / --ansi   physical layout, REQUIRED for any app build (no default)
#                    e.g. ./build.sh -pa --iso   or   ./build.sh -pa --ansi
#   (default) the open BLE link layer (issue #13), no binary blob is fetched or linked
#   --blob    opt in to the proprietary Telink BLE blob instead (fetched on demand)
#   --openll  accepted no-op alias (the open controller is the default)
#   --wired   USB-only gaming build: no Bluetooth, NKRO, eager press debounce, no deep
#             sleep, calm default lighting (conf/wired.conf + conf/wired.overlay, the
#             keymap sees RAINY75_WIRED). Refused with --blob / --privacy.
#   --privacy resolvable private address (BT_PRIVACY), open controller only. Every
#             host must be paired again; refused with --blob.
#   --test-image  risky test image: never confirms itself, so any reset or power
#             cycle returns to the previous image (conf/test-image.conf). Flash with
#             mcumgr image test, confirm a good one with mcumgr image confirm.
# Environment: BUILD_DIR=<dir> puts the app build (and the combined/OTA images)
#   there instead of build/.

set -e

cd "$(dirname "$0")"

source .venv/bin/activate
export ZEPHYR_SDK_INSTALL_DIR="$(pwd)/toolchain/zephyr-sdk-0.17.0"

PRISTINE=""
VERBOSE_CMAKE=""
BUILD_MCUBOOT=0
BUILD_COMBINED=0
BUILD_OTA=0
BUILD_BRIDGE=0
BUILD_APP=1
LAYOUT=""             # "iso" or "ansi" — REQUIRED for app builds, no default
ANSI_KCONFIG=""       # set when LAYOUT=ansi (the DTS symbol goes via DTS_CPPFLAGS)
APP_CONF="$(pwd)/conf/app.conf"
USE_BLOB=0            # set by --blob (default: open controller)
USE_PRIVACY=0         # set by --privacy (refused with --blob)
USE_TEST_IMAGE=0      # set by --test-image
USE_WIRED=0           # set by --wired (USB only, no BLE controller overlay)
BUILD_DIR="${BUILD_DIR:-build}"   # app build directory

# ── Apply upstream patches if needed ──────────────────────────
#
# The patches live in patches/<repo>/ and are listed, with their sha256 and
# target tree, in zmk/zephyr/patches.yml for Zephyr's `west patch` (manual
# flow: `west update && west patch -b ../patches apply`). `west patch apply`
# applies every listed patch and is not idempotent, so this script decides
# per tree which patches are missing and hands only those to `west patch`.
#
# A patch is "already applied" when its commit subject is in the tree's
# history: the apply-command is `git am`, which records the subject verbatim,
# so this holds for every patch in a stack. The obvious test, `git apply
# --reverse --check`, does NOT: once a later patch rewrites the same lines,
# the earlier one no longer reverses cleanly and gets re-applied on every
# build, failing loudly and harmlessly. That noise is indistinguishable from
# a patch that genuinely did not apply, which is how the bind-window grace
# (zmk-src 0003) went missing from every build unnoticed.
#
# An applied patch must also match its file (git patch-id), so a patch
# rewritten in place under the same subject is caught. Missing patches are
# applied only when they are the tail of the series (none applied yet after
# `west update`, or new patches appended). A gap (a later patch is in the
# tree, an earlier one is not or is outdated), a patch dropped from the
# series (a commit beyond manifest-rev that the list does not know) or a
# patch that did not end up in the tree stops the build: the tree carries
# an older series and needs `git -C <tree> checkout --detach manifest-rev`
# first. This script never
# resets or cleans a tree itself, so uncommitted work there is never lost.
#
# A patch that truly fails is fatal: building without it silently produces
# firmware that is not the tree anyone reviewed.
PATCH_YML=zmk/zephyr/patches.yml

# patches.yml queries (PyYAML comes with west).
patch_yml() {
    python3 - "$PATCH_YML" "$@" <<'PYEOF'
import sys, yaml
yml, cmd, args = sys.argv[1], sys.argv[2], sys.argv[3:]
data = yaml.safe_load(open(yml))
patches = data.get("patches") or []
if cmd == "modules":            # target trees, in order
    print("\n".join(dict.fromkeys(p["module"] for p in patches)))
elif cmd == "paths":            # patch files of one tree, in order
    print("\n".join(p["path"] for p in patches if p["module"] == args[0]))
elif cmd == "all-paths":
    print("\n".join(p["path"] for p in patches))
elif cmd == "subset":           # write a copy with only the given patches
    keep = set(args[1:])
    data["patches"] = [p for p in patches if p["path"] in keep]
    with open(args[0], "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
PYEOF
}

patch_subject() {
    git mailinfo /dev/null /dev/null < "$1" 2>/dev/null | sed -n 's/^Subject: //p'
}

# Newest commit above manifest-rev with this subject (empty if none).
patch_commit() {
    [ -n "$2" ] || return 0
    git -C "$1" log --format='%H %s' manifest-rev..HEAD 2>/dev/null |
        awk -v s="$2" 'substr($0, 42) == s { print $1; exit }'
}

patch_id() {
    git patch-id --stable | cut -d' ' -f1
}

check_patch_list() {
    local f listed
    listed=$(patch_yml all-paths) && [ -n "$listed" ] ||
        { echo "ERROR: cannot read $PATCH_YML" >&2; exit 1; }
    for f in patches/*/*.patch; do
        [ -f "$f" ] || continue
        if ! grep -Fxq "${f#patches/}" <<< "$listed"; then
            echo "ERROR: $f is not listed in $PATCH_YML (path, sha256sum, module, ...)." >&2
            exit 1
        fi
    done
}

apply_patches() {
    local dir="$1" path subject commit gap=0 tmp rebase
    local -a missing=() stale=() subjects=()
    # Neutral identity for `git am --abort`: these are throwaway commits in
    # fetched trees, and without an identity the abort fails and leaves a
    # half-finished rebase-apply behind, which breaks the *next* build with a
    # confusing "previous rebase directory still exists". The apply-command
    # in patches.yml sets the same identity for `git am`.
    local ident=(-c user.name="rainy75 build" -c user.email="build@localhost")
    local paths
    paths=$(patch_yml paths "$dir") || exit 1
    [ -n "$paths" ] || { echo "ERROR: no patches listed for $dir" >&2; exit 1; }
    while IFS= read -r path; do
        [ -n "$path" ] || continue
        subject=$(patch_subject "patches/$path")
        subjects+=("$subject")
        commit=$(patch_commit "$dir" "$subject")
        if [ -n "$commit" ]; then
            [ ${#missing[@]} -gt 0 ] && gap=1
            # Same subject, different content: the patch was rewritten in
            # place, and this tree still builds the old version.
            if [ "$(patch_id < "patches/$path")" != \
                 "$(git -C "$dir" show "$commit" | patch_id)" ]; then
                stale+=("$path")
            fi
        else
            missing+=("$path")
        fi
    done <<< "$paths"

    # A patch dropped from the series stays in a tree that has it until the
    # tree is reset, so commits beyond manifest-rev that are not in the list
    # stop the build. RAINY75_ALLOW_EXTRA_COMMITS=1 allows own work there.
    local extra
    extra=$(git -C "$dir" log --format=%s manifest-rev..HEAD 2>/dev/null |
            grep -Fxv -f <(printf '%s\n' "${subjects[@]}") || true)
    if [ -n "$extra" ]; then
        if [ "${RAINY75_ALLOW_EXTRA_COMMITS:-0}" = 1 ]; then
            echo "NOTE: $dir has commits beyond manifest-rev that are not in $PATCH_YML:" >&2
            sed 's/^/        /' <<< "$extra" >&2
        else
            echo "ERROR: $dir has commits beyond manifest-rev that are not in $PATCH_YML" >&2
            echo "       (for example a patch that was dropped from the series):" >&2
            sed 's/^/         /' <<< "$extra" >&2
            echo "       Reset the tree (git -C $dir checkout --detach manifest-rev)," >&2
            echo "       or set RAINY75_ALLOW_EXTRA_COMMITS=1 to build with own commits." >&2
            exit 1
        fi
    fi

    if [ ${#stale[@]} -gt 0 ]; then
        echo "ERROR: $dir carries an older version of these patches:" >&2
        printf '         %s\n' "${stale[@]}" >&2
        gap=1
    fi

    [ ${#missing[@]} -eq 0 ] && [ ${#stale[@]} -eq 0 ] && return 0

    if [ "$gap" -eq 1 ]; then
        echo "ERROR: $dir carries a different patch series than $PATCH_YML." >&2
        [ ${#missing[@]} -gt 0 ] && echo "       Missing, while later patches are applied:" >&2
        [ ${#missing[@]} -gt 0 ] && printf '         %s\n' "${missing[@]}" >&2
        echo "       Commit or save any own work in $dir, then:" >&2
        echo "         git -C $dir checkout --detach manifest-rev   # or: west update" >&2
        echo "       and build again (the patches are re-applied)." >&2
        exit 1
    fi

    echo "Applying ${#missing[@]} patch(es) to $dir with west patch"
    tmp=$(mktemp --suffix=.yml)
    patch_yml subset "$tmp" "${missing[@]}"
    if ! west patch -b ../patches -l "$tmp" apply; then
        rm -f "$tmp"
        git -C "$dir" "${ident[@]}" am --abort 2>/dev/null || true
        rebase=$(git -C "$dir" rev-parse --git-path rebase-apply)
        case "$rebase" in /*) ;; *) rebase="$dir/$rebase" ;; esac
        rm -rf "$rebase"
        echo "ERROR: a patch for $dir failed to apply (see above)." >&2
        echo "       Resolve it in $dir before building; a build without" >&2
        echo "       it is not the firmware this tree describes." >&2
        exit 1
    fi
    rm -f "$tmp"
    # west patch skips a patch whose module path does not resolve, and git am
    # --3way skips one that is already in the tree, both without failing.
    for path in "${missing[@]}"; do
        if [ -z "$(patch_commit "$dir" "$(patch_subject "patches/$path")")" ]; then
            echo "ERROR: $path was not applied to $dir (see above)." >&2
            exit 1
        fi
    done
}

check_patch_list
PATCH_TREE_LIST=$(patch_yml modules) && [ -n "$PATCH_TREE_LIST" ] ||
    { echo "ERROR: cannot read the trees from $PATCH_YML" >&2; exit 1; }
mapfile -t PATCH_TREES <<< "$PATCH_TREE_LIST"
for tree in "${PATCH_TREES[@]}"; do
    apply_patches "$tree"
done

# Allow long forms --iso / --ansi as aliases for -I / -A.
ARGS=(); for a in "$@"; do case "$a" in
    --iso)    ARGS+=("-I");;
    --ansi)   ARGS+=("-A");;
    --openll) ARGS+=("-O");;
    --blob)   ARGS+=("-B");;
    --privacy) ARGS+=("-Y");;
    --test-image) ARGS+=("-T");;
    --wired)  ARGS+=("-W");;
    *)        ARGS+=("$a");;
esac; done
set -- "${ARGS[@]}"

while getopts "pvmcobaIAOYBTW" opt; do
    case $opt in
        p) PRISTINE="-p" ;;
        v) VERBOSE_CMAKE="-DCMAKE_VERBOSE_MAKEFILE=ON" ;;
        m) BUILD_MCUBOOT=1; BUILD_APP=0 ;;
        c) BUILD_COMBINED=1 ;;
        o) BUILD_OTA=1 ;;
        b) BUILD_BRIDGE=1; BUILD_APP=0 ;;
        a) BUILD_MCUBOOT=1; BUILD_COMBINED=1; BUILD_OTA=1; BUILD_BRIDGE=1; BUILD_APP=1 ;;
        I) [ "$LAYOUT" = ansi ] && { echo "Error: --iso and --ansi are mutually exclusive" >&2; exit 1; }; LAYOUT="iso" ;;
        A) [ "$LAYOUT" = iso  ] && { echo "Error: --iso and --ansi are mutually exclusive" >&2; exit 1; }; LAYOUT="ansi"; ANSI_KCONFIG="-DCONFIG_RAINY_RGB_ANSI_LEDMAP=y" ;;
        O) echo "Note: --openll is a no-op, the open controller is the default." >&2 ;;
        B) USE_BLOB=1 ;;
        Y) USE_PRIVACY=1 ;;
        T) USE_TEST_IMAGE=1 ;;
        W) USE_WIRED=1 ;;
        *) echo "Usage: $0 [-p] [-v] [-m] [-c] [-o] [-b] [-a] (--iso | --ansi) [--wired | --blob | --privacy] [--test-image]"; exit 1 ;;
    esac
done

# The app build needs an explicit physical layout — no silent default (an ANSI
# owner must not get an ISO build by accident, and vice-versa).
if [ "$BUILD_APP" -eq 1 ] && [ -z "$LAYOUT" ]; then
    echo "Error: choose a layout for the app build: --iso or --ansi" >&2
    echo "  ./build.sh -pa --iso    # ISO DE  (the original board)" >&2
    echo "  ./build.sh -pa --ansi   # ANSI    (community-verified)" >&2
    exit 1
fi

# Bluetooth builds: conf/ble.conf (host, HCI driver, BLE SMP) plus the controller.
# The open controller is the default: conf/openll.conf (controller choice, power
# counters, long-PDU buffers) is part of its configuration. --blob keeps the
# blob build exactly as before (no openll.conf, blob selected explicitly).
# --wired drops Bluetooth entirely: no ble.conf, no controller overlay, conf/wired.conf
# on top of app.conf, conf/wired.overlay for the kscan, RAINY75_WIRED for the keymap.
APP_OVERLAYS="$(pwd)/zmk/boards/rainy75/rainy75.keymap;$(pwd)/conf/mcumgr.overlay;$(pwd)/conf/studio-usb.overlay"
DTS_CPPFLAGS=""
[ "$LAYOUT" = ansi ] && DTS_CPPFLAGS="-DRAINY75_ANSI"
if [ "$USE_WIRED" -eq 1 ]; then
    if [ "$USE_BLOB" -eq 1 ] || [ "$USE_PRIVACY" -eq 1 ]; then
        echo "Error: --wired has no Bluetooth; drop --blob / --privacy." >&2
        exit 1
    fi
    APP_CONF="$APP_CONF;$(pwd)/conf/wired.conf"
    APP_OVERLAYS="$APP_OVERLAYS;$(pwd)/conf/wired.overlay"
    DTS_CPPFLAGS="${DTS_CPPFLAGS:+$DTS_CPPFLAGS;}-DRAINY75_WIRED"
elif [ "$USE_BLOB" -eq 1 ]; then
    if [ "$USE_PRIVACY" -eq 1 ]; then
        echo "Error: --privacy cannot be combined with --blob (privacy is only tested with" >&2
        echo "       the open controller). Drop --blob to use the open controller." >&2
        exit 1
    fi
    APP_CONF="$APP_CONF;$(pwd)/conf/ble.conf;$(pwd)/conf/blob.conf"
else
    APP_CONF="$APP_CONF;$(pwd)/conf/ble.conf;$(pwd)/conf/openll.conf"
    [ "$USE_PRIVACY" -eq 1 ] && APP_CONF="$APP_CONF;$(pwd)/conf/privacy.conf"
fi
if [ "$USE_TEST_IMAGE" -eq 1 ]; then
    APP_CONF="$APP_CONF;$(pwd)/conf/test-image.conf"
    echo "Note: --test-image: the image will NOT confirm itself (mcumgr image confirm <hash>)." >&2
fi

# ── Fetch the (non-redistributable) Telink BLE blob, only for --blob ──
# MCUboot, the bridge (CONFIG_BT=n) and the default open app build do not link it.
if [ "$BUILD_APP" -eq 1 ] && [ "$USE_BLOB" -eq 1 ]; then
    ./fetch_ble_blob.sh
fi

# ── MCUboot build ──────────────────────────────────────────────
if [ "$BUILD_MCUBOOT" -eq 1 ]; then
    echo "=== Building MCUboot ==="
    west build $PRISTINE -b rainy75 -d build-mcuboot \
        bootloader/mcuboot/boot/zephyr -- \
        -DEXTRA_CONF_FILE="$(pwd)/conf/mcuboot.conf" \
        -DEXTRA_DTC_OVERLAY_FILE="$(pwd)/conf/mcuboot.overlay" \
        "-DDTS_ROOT=$(pwd)/zmk;$(pwd)/zmk-src/app" \
        "-DZMK_EXTRA_MODULES=$(pwd)/zmk;$(pwd)/zmk-src/app" \
        $VERBOSE_CMAKE

    SIZE=$(wc -c < build-mcuboot/zephyr/zephyr.bin | tr -d ' ')
    echo "MCUboot binary: $SIZE bytes (max 65536 for 64KB boot partition)"
    if [ "$SIZE" -gt 65536 ]; then
        echo "ERROR: MCUboot exceeds 64KB boot partition!" >&2
        exit 1
    fi
fi

# ── App build ──────────────────────────────────────────────────
if [ "$BUILD_APP" -eq 1 ]; then
    echo "=== Building ZMK app ($(echo "$LAYOUT" | tr a-z A-Z) layout$([ "$USE_WIRED" -eq 1 ] && echo ', wired')) ==="
    # DTS_EXTRA_CPPFLAGS is a CMake list (';'-separated), one -D per entry.
    west build $PRISTINE -b rainy75 -d "$BUILD_DIR" zmk-src/app -- \
        -DZMK_CONFIG="$(pwd)/zmk/boards/rainy75" \
        -DZMK_EXTRA_MODULES="$(pwd)/zmk" \
        -DEXTRA_CONF_FILE="$APP_CONF" \
        -DEXTRA_DTC_OVERLAY_FILE="$APP_OVERLAYS" \
        ${DTS_CPPFLAGS:+-DDTS_EXTRA_CPPFLAGS="$DTS_CPPFLAGS"} \
        $ANSI_KCONFIG \
        $VERBOSE_CMAKE
fi

# ── Combined image ─────────────────────────────────────────────
if [ "$BUILD_COMBINED" -eq 1 ]; then
    echo "=== Creating combined flash image ==="
    if [ ! -f build-mcuboot/zephyr/zephyr.bin ]; then
        echo "ERROR: MCUboot binary not found. Build with -m first." >&2
        exit 1
    fi
    if [ ! -f "$BUILD_DIR/zephyr/zmk.signed.bin" ]; then
        echo "ERROR: Signed app binary not found. Build app first." >&2
        exit 1
    fi

    python3 -c "
mcuboot = open('build-mcuboot/zephyr/zephyr.bin','rb').read()
app = open('$BUILD_DIR/zephyr/zmk.signed.bin','rb').read()
pad = 0x10000 - len(mcuboot)  # 64KB boot partition
assert pad > 0, f'MCUboot too large: {len(mcuboot)} bytes'
combined = mcuboot + (b'\xff' * pad) + app
open('$BUILD_DIR/combined.bin','wb').write(combined)
print(f'MCUboot:  {len(mcuboot):,} bytes')
print(f'App:      {len(app):,} bytes (at offset 0x10000)')
print(f'Combined: {len(combined):,} bytes')
"
    echo "Output: $BUILD_DIR/combined.bin"
fi

# ── OTA image ─────────────────────────────────────────────────
if [ "$BUILD_OTA" -eq 1 ]; then
    echo "=== Creating OTA-ready image ==="
    if [ ! -f "$BUILD_DIR/combined.bin" ]; then
        echo "ERROR: Combined image not found. Build with -c first." >&2
        exit 1
    fi

    python3 reverse/tools/prepare_ota.py "$BUILD_DIR/combined.bin" -o "$BUILD_DIR/combined_ota.bin"
fi

# ── Bridge build (monolithic, no MCUboot) ─────────────────────
if [ "$BUILD_BRIDGE" -eq 1 ]; then
    echo "=== Building OTA bridge ==="
    west build $PRISTINE -b rainy75 -d build-bridge zmk-src/app -- \
        -DZMK_CONFIG="$(pwd)/zmk/boards/rainy75" \
        -DZMK_EXTRA_MODULES="$(pwd)/zmk" \
        -DEXTRA_CONF_FILE="$(pwd)/conf/ota-bridge.conf" \
        -DEXTRA_DTC_OVERLAY_FILE="$(pwd)/zmk/boards/rainy75/rainy75.keymap;$(pwd)/conf/mcumgr.overlay" \
        $VERBOSE_CMAKE

    if [ ! -f build-bridge/zephyr/zmk.bin ]; then
        echo "ERROR: Bridge binary not found after build." >&2
        exit 1
    fi

    SIZE=$(wc -c < build-bridge/zephyr/zmk.bin | tr -d ' ')
    echo "Bridge binary: $SIZE bytes"

    # Safety: bridge + ZMK combined must both fit below calibration (0xFE000)
    if [ "$SIZE" -gt 262144 ]; then
        echo "ERROR: Bridge exceeds 256KB — too large for bank 1!" >&2
        exit 1
    fi

    echo "=== Creating bridge OTA image ==="
    python3 reverse/tools/prepare_ota.py build-bridge/zephyr/zmk.bin \
        -o build-bridge/bridge_ota.bin
    echo "Output: build-bridge/bridge_ota.bin"
fi
