#!/usr/bin/env bash
#
# Rebuild common/params_pyx.so on a Comma 4 (or any AGNOS / tici / mici)
# device so the compiled params allowlist picks up new keys added to
# common/params_keys.h (e.g. the Scintir* keys).
#
# Why this exists: sunnypilot release/staging branches ship prebuilt
# binaries (the `prebuilt` sentinel is committed) and strip SConstruct out
# of the tree, so `./system/manager/build.py` can't run. This script
# bypasses SCons entirely by invoking cython + clang++ directly using the
# flags captured in the repo's compile_commands.json.
#
# Usage (on the device, after SSH):
#   cd /data/openpilot
#   bash tools/scintir/rebuild_params_pyx.sh
#   sudo reboot
#
# Safe to run multiple times. Old .so is backed up to
# common/params_pyx.so.backup.<epoch>; restore by copying it back.

set -euo pipefail

cd "$(dirname "$0")/../.."
REPO=$PWD

if [[ ! -f common/params_pyx.pyx ]]; then
  echo "ERROR: run from the openpilot/sunnypilot repo root (couldn't find common/params_pyx.pyx)" >&2
  exit 1
fi

# --- detect build-environment paths (mirrors compile_commands.json) ---
PY_INC=$(python3 -c 'from sysconfig import get_paths; print(get_paths()["include"])')
NUMPY_INC=$(python3 -c 'import numpy; print(numpy.get_include())')
VENV_SITE=$(python3 -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')
ZMQ_INC=$VENV_SITE/zeromq/install/include
ZMQ_LIB_DIR=$VENV_SITE/zeromq/install/lib
CAPNP_INC=$VENV_SITE/capnproto/install/include
ZSTD_INC=$VENV_SITE/zstd/install/include
BZIP2_INC=$VENV_SITE/bzip2/install/include

for p in "$PY_INC" "$NUMPY_INC" "$ZMQ_INC" "$CAPNP_INC"; do
  [[ -d "$p" ]] || { echo "ERROR: missing include dir: $p" >&2; exit 2; }
done

ZMQ_LIB=
for cand in "$ZMQ_LIB_DIR/libzmq.a" "$ZMQ_LIB_DIR/libzmq.so"; do
  if [[ -f "$cand" ]]; then ZMQ_LIB=$cand; break; fi
done
[[ -n "$ZMQ_LIB" ]] || { echo "ERROR: no libzmq.{a,so} under $ZMQ_LIB_DIR" >&2; exit 2; }

CXX=${CXX:-clang++}
command -v "$CXX" >/dev/null || { echo "ERROR: $CXX not in PATH" >&2; exit 3; }
command -v cython >/dev/null || { echo "ERROR: cython not in PATH" >&2; exit 3; }

# --- back up old .so ---
TS=$(date +%s)
BACKUP=common/params_pyx.so.backup.$TS
cp -a common/params_pyx.so "$BACKUP"
echo "[+] backed up existing params_pyx.so -> $BACKUP"

# --- regenerate cython output ---
echo "[+] cython: common/params_pyx.pyx -> common/params_pyx.cpp"
cython -3 --cplus common/params_pyx.pyx -o common/params_pyx.cpp

# --- compile ---
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

COMMON_CFLAGS=(
  -std=c++1z -D__TICI__ -mcpu=cortex-a57 -DQCOM2
  -g -fPIC -O2
  -Wno-unknown-warning-option -Wno-inconsistent-missing-override
  -Wno-c99-designator -Wno-reorder-init-list -Wno-vla-cxx-extension
)
INCLUDES=(
  -I. -Imsgq
  -Ithird_party -Ithird_party/json11 -Ithird_party/linux/include
  -I"$CAPNP_INC" -I"$ZMQ_INC" -I"$ZSTD_INC" -I"$BZIP2_INC"
)
CYTHON_CFLAGS=(-Wno-cpp -Wno-deprecated-declarations -Wno-shadow -Wno-#warnings)
PY_INCS=(-I"$PY_INC" -I"$NUMPY_INC")

compile() {
  local src=$1 out=$2
  shift 2
  echo "[+] cc $src"
  "$CXX" "${COMMON_CFLAGS[@]}" "${INCLUDES[@]}" "$@" -c "$src" -o "$out"
}

compile common/params.cc                   "$TMP/params.o"
compile common/util.cc                     "$TMP/util.o"
compile common/swaglog.cc                  "$TMP/swaglog.o"
compile common/ratekeeper.cc               "$TMP/ratekeeper.o"
compile third_party/json11/json11.cpp      "$TMP/json11.o"
compile common/params_pyx.cpp              "$TMP/params_pyx.o" "${CYTHON_CFLAGS[@]}" "${PY_INCS[@]}"

# --- link ---
echo "[+] ld -> common/params_pyx.so"
"$CXX" -shared -fPIC -Wl,-soname,params_pyx.so \
  "$TMP/params_pyx.o" "$TMP/params.o" "$TMP/util.o" "$TMP/swaglog.o" \
  "$TMP/ratekeeper.o" "$TMP/json11.o" \
  "$ZMQ_LIB" -lpthread \
  -o common/params_pyx.so

ls -la common/params_pyx.so
echo

# --- smoke test ---
echo "[+] import + write/read test for ScintirEVLimiterEnabled"
if PYTHONPATH=. python3 -c "
import sys
from openpilot.common.params import Params
p = Params()
p.put_bool('ScintirEVLimiterEnabled', True)
v = p.get_bool('ScintirEVLimiterEnabled')
print('  ScintirEVLimiterEnabled =', v)
sys.exit(0 if v is True else 1)
"; then
  echo
  echo "[ok] rebuild succeeded. reboot to pick it up:  sudo reboot"
else
  echo
  echo "[FAIL] smoke test failed — restoring backup"
  cp -a "$BACKUP" common/params_pyx.so
  exit 4
fi
