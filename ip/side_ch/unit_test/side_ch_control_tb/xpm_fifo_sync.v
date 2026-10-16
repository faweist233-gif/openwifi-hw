// Functional behavioral model of Xilinx xpm_fifo_sync, for simulation only.
// Mirrors the fwft / FIFO_READ_LATENCY=0 semantics used by side_ch_control.v
// (side_info_fifo). NOT for synthesis.
`timescale 1ns/1ps
module xpm_fifo_sync #(
  parameter DOUT_RESET_VALUE="0",
  parameter ECC_MODE="no_ecc",
  parameter FIFO_MEMORY_TYPE="auto",
  parameter FIFO_READ_LATENCY=0,
  parameter FIFO_WRITE_DEPTH=512,
  parameter FULL_RESET_VALUE=0,
  parameter PROG_EMPTY_THRESH=10,
  parameter PROG_FULL_THRESH=10,
  parameter RD_DATA_COUNT_WIDTH=10,
  parameter READ_DATA_WIDTH=32,
  parameter READ_MODE="fwft",
  parameter USE_ADV_FEATURES="0404",
  parameter WAKEUP_TIME=0,
  parameter WRITE_DATA_WIDTH=32,
  parameter WR_DATA_COUNT_WIDTH=10
)(
  output reg almost_empty, almost_full, data_valid, dbiterr,
  output reg [READ_DATA_WIDTH-1:0] dout,
  output reg empty, full, overflow, prog_empty, prog_full,
  output reg [RD_DATA_COUNT_WIDTH-1:0] rd_data_count,
  output reg rd_rst_busy, sbiterr, underflow, wr_ack,
  output reg [WR_DATA_COUNT_WIDTH-1:0] wr_data_count, wr_rst_busy,
  input [WRITE_DATA_WIDTH-1:0] din,
  input injectdbiterr, injectsbiterr, rd_en, rst, sleep, wr_clk, wr_en
);
  function integer clog2 (input integer n);
    begin
      clog2 = 0;
      while ((1 << clog2) < n) clog2 = clog2 + 1;
    end
  endfunction

  localparam AW = (FIFO_WRITE_DEPTH <= 1) ? 1 : clog2(FIFO_WRITE_DEPTH);
  reg [WRITE_DATA_WIDTH-1:0] mem [0:FIFO_WRITE_DEPTH-1];
  reg [AW-1:0] wr_ptr, rd_ptr;
  reg [AW:0] count;

  // fwft / latency 0: dout is combinational (first word available without rd_en)
  assign dout = (count != 0) ? mem[rd_ptr] : {READ_DATA_WIDTH{1'b0}};
  always @(posedge wr_clk) begin
    if (rst) begin
      wr_ptr <= 0; rd_ptr <= 0; count <= 0;
    end else begin
      if (wr_en && count < FIFO_WRITE_DEPTH) begin
        mem[wr_ptr] <= din; wr_ptr <= wr_ptr + 1; count <= count + 1;
      end
      if (rd_en && count > 0) begin
        rd_ptr <= rd_ptr + 1; count <= count - 1;
      end
    end
  end

  assign empty = (count == 0);
  assign full  = (count == FIFO_WRITE_DEPTH);
  assign rd_data_count = count;
  assign wr_data_count = count;

  initial begin
    almost_empty = 0; almost_full = 0; data_valid = 0; dbiterr = 0;
    overflow = 0; prog_empty = 0; prog_full = 0; rd_rst_busy = 0;
    sbiterr = 0; underflow = 0; wr_ack = 0; wr_rst_busy = 0;
  end
endmodule
