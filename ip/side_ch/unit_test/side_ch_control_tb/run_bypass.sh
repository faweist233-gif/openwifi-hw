#!/bin/bash
# run_bypass.sh -- L1 bypass-equivalence regression
#
# Extracts the pre-P2 RTL (fa6c896) from git, renames its module to
# side_ch_control_ref, then runs side_ch_control_tb_bypass.v which drives the
# same stimulus into the old and new RTL with csi_iq_combined=0.
#
#   ./run_bypass.sh
#
# Requires iverilog (same as run.sh). The reference RTL is generated into a
# temp dir and is never committed.
set -e
cd "$(dirname "$0")"
SRC=../../src
REPO=$(cd ../../../.. && pwd)
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

git -C "$REPO" show fa6c896:ip/side_ch/src/side_ch_control.v > "$TMP/side_ch_control_ref.v"
sed -i 's/^\([[:space:]]*\)module side_ch_control #/\1module side_ch_control_ref #/' "$TMP/side_ch_control_ref.v"
if ! grep -q "module side_ch_control_ref" "$TMP/side_ch_control_ref.v"; then
  echo "FAIL: could not rename reference module" >&2
  exit 1
fi

iverilog -g2012 -o "$TMP/bypass.out" -I. \
  "$SRC/side_ch_control.v" "$TMP/side_ch_control_ref.v" \
  "$SRC/dpram.v" xpm_fifo_sync.v side_ch_control_tb_bypass.v
vvp "$TMP/bypass.out" | tee "$TMP/bypass.log"

if grep -q "^PASS" "$TMP/bypass.log"; then
  echo "--- bypass equivalence PASS ---"
else
  echo "--- bypass equivalence FAIL ---" >&2
  exit 1
fi
