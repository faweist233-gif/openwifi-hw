#!/bin/bash
# Run the side_ch_control combined-mode behavioral testbenches with iverilog.
#
#   ./run.sh            # matched (addr2 path) + unmatched FC + short frame
#   ./run.sh matched    # only the matched happy path
#
# The DUT (side_ch_control) is the same Verilog used in synthesis; the only
# sim-only pieces are xpm_fifo_sync.v (functional fwft model) and the empty
# side_ch_pre_def.v include. E316 macros are passed as parameters
# (MAX_NUM_DMA_SYMBOL=4096 -> SIDE_CH_LESS_BRAM behavior).
#
# To run with the real Xilinx xpm / xsim instead, create a Vivado project from
# the source (see ip/create_vivado_proj.sh) and add this dir as Simulation
# Sources (drop xpm_fifo_sync.v, keep the real xpm library).
set -e
cd "$(dirname "$0")"
SRC=../../src

run_one() {
  local tb="$1"
  iverilog -g2012 -o /tmp/"$tb".out -I. "$SRC/side_ch_control.v" "$SRC/dpram.v" \
    xpm_fifo_sync.v "$tb".v
  vvp /tmp/"$tb".out | grep -E "PASS|FAIL|RESULT|captured" || true
  echo "--- $tb done ---"
}

case "${1:-all}" in
  matched)  run_one side_ch_control_tb ;;
  unmatched) run_one side_ch_control_tb_unmatched ;;
  short)    run_one side_ch_control_tb_short ;;
  all)
    run_one side_ch_control_tb
    run_one side_ch_control_tb_unmatched
    run_one side_ch_control_tb_short
    ;;
  *) echo "usage: $0 [matched|unmatched|short|all]" >&2; exit 2 ;;
esac
