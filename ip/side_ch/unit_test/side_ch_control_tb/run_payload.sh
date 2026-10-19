#!/bin/bash
# run_payload.sh -- P2 combined payload equivalence vs original CSI/IQ
#
# Extracts the pre-P2 RTL (fa6c896) as side_ch_control_ref, then runs
# side_ch_control_tb_payload.v with three DUTs on the same stimulus:
#   combined mode  vs  pure CSI  +  pure IQ
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

iverilog -g2012 -o "$TMP/payload.out" -I. \
  "$SRC/side_ch_control.v" "$TMP/side_ch_control_ref.v" \
  "$SRC/dpram.v" xpm_fifo_sync.v side_ch_control_tb_payload.v
vvp "$TMP/payload.out" | tee "$TMP/payload.log"

if grep -q "^PASS" "$TMP/payload.log"; then
  echo "--- payload equivalence PASS ---"
else
  echo "--- payload equivalence FAIL ---" >&2
  exit 1
fi
