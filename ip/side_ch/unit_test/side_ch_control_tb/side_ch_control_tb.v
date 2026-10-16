// side_ch_control_tb.v -- behavioral testbench for P2 combined mode (two-stage trigger)
//
// Instantiates side_ch_control directly and drives the openofdm_rx / xpu
// handshake signals to model one matched 802.11 frame in combined mode
// (iq_capture=1, csi_iq_combined=1, reg3=3). A continuous 20MHz sample strobe
// keeps the IQ dpram / IQ streaming FSM live. Verifies:
//   1. record_len = (1+iq_len)+(2+56+num_eq*52) words emitted, IQ block first
//   2. IQ window locks at long_preamble_detected, pushes only after csi_commit
//   3. IQ TSF == trigger-time TSF, CSI TSF == header-time TSF
//   4. IQ block is a contiguous iq_len-sample slice from the pre-trigger window
//   5. CSI block passes through the 56 (mask-selected) estimator values
`timescale 1ns/1ps

module side_ch_control_tb;

  parameter IQ_LEN = 64;
  parameter PRE    = 32;

  reg clk = 0;
  always #5 clk = ~clk; // 100MHz

  // ---- DUT inputs ----
  reg rstn = 0;
  reg [7:0]  gpio_status = 0;
  reg signed [10:0] rssi_half_db = 0;
  reg [63:0] tsf_runtime_val = 0;
  reg [31:0] openofdm_tx_iq0 = 0, openofdm_tx_iq1 = 0;
  reg openofdm_tx_iq_valid = 0;
  reg [31:0] tx_intf_iq0 = 0, tx_intf_iq1 = 0;
  reg tx_intf_iq_valid = 0;
  reg [31:0] iq0 = 0, iq1 = 0;
  reg iq_strobe = 0;
  reg demod_is_ongoing = 0;
  reg ofdm_symbol_eq_out_pulse = 0;
  reg long_preamble_detected = 0;
  reg short_preamble_detected = 0;
  reg ht_unsupport = 0;
  reg [7:0]  pkt_rate = 0;
  reg [15:0] pkt_len = 100;
  reg [31:0] csi = 0;
  reg csi_valid = 0;
  reg signed [31:0] phase_offset_taken = 0;
  reg [31:0] equalizer = 0;
  reg equalizer_valid = 0;
  reg pkt_header_valid = 0;
  reg pkt_header_valid_strobe = 0;
  reg [1:0]  phy_type = 0;
  reg [3:0]  tx_control_state = 0;
  reg [31:0] FC_DI = 0;
  reg FC_DI_valid = 0;
  reg [47:0] addr1 = 0, addr2 = 0, addr3 = 0;
  reg addr1_valid = 0, addr2_valid = 0, addr3_valid = 0;
  reg fcs_in_strobe = 0, fcs_ok = 0;
  reg block_rx_dma_to_ps = 0, block_rx_dma_to_ps_valid = 0, ch_idle_final = 0;
  reg phy_tx_start = 0, tx_pkt_need_ack = 0, phy_tx_started = 0, phy_tx_done = 0;
  reg tx_bb_is_ongoing = 0, tx_rf_is_ongoing = 0;
  reg tx_pkt_iq_to_dac_ongoing = 0, retrans_in_progress = 0;
  reg slv_reg_wren_signal = 0;
  reg [4:0]  axi_awaddr_core = 0;
  reg iq_capture = 1;
  reg csi_iq_combined = 1;
  reg [1:0]  iq_capture_cfg = 0;
  reg [4:0]  iq_trigger_select = 0;
  reg iq_trigger_free_run_flag = 0;
  reg [1:0]  iq_source_select = 0;
  reg disable_tx_pkt_need_ack_check = 0;
  reg [2:0]  PPDU_FORMAT_target = 0;
  reg [15:0] rssi_or_iq_th = 0;
  reg [6:0]  gain_th = 0;
  reg [12:0] pre_trigger_len = PRE;
  reg [12:0] iq_len_target = IQ_LEN;
  reg [3:0]  tx_control_state_target = 0;
  reg [1:0]  phy_type_target = 0;
  reg [15:0] FC_target = 0;
  reg [31:0] addr1_target = 0, addr2_target = 0;
  reg [3:0]  match_cfg = 0;
  reg [3:0]  num_eq = 0;
  reg [1:0]  m_axis_start_mode = 1;
  reg m_axis_start_ext_trigger = 0;
  reg [63:0] data_to_pl = 0;
  reg [12:0] s_axis_data_count = 0;
  reg emptyn_to_pl = 0;
  reg S_AXIS_TVALID = 0, S_AXIS_TLAST = 0;
  reg [12:0] m_axis_data_count = 0;
  reg fulln_to_pl = 1;
  reg M_AXIS_TVALID = 0, M_AXIS_TLAST = 0;

  // ---- DUT outputs ----
  wire m_axis_start_1trans;
  wire pl_ask_data;
  wire [63:0] data_to_ps;
  wire data_to_ps_valid;
  wire [31:0] MAX_NUM_DMA_SYMBOL_UDP_debug, MAX_NUM_DMA_SYMBOL_debug;

  side_ch_control #(
    .TSF_TIMER_WIDTH(64), .GPIO_STATUS_WIDTH(8), .RSSI_HALF_DB_WIDTH(11),
    .C_S_AXI_DATA_WIDTH(32), .IQ_DATA_WIDTH(16), .C_S_AXIS_TDATA_WIDTH(64),
    .MAX_NUM_DMA_SYMBOL(4096), .MAX_BIT_NUM_DMA_SYMBOL(13)
  ) dut (
    .clk(clk), .rstn(rstn),
    .gpio_status(gpio_status), .rssi_half_db(rssi_half_db),
    .tsf_runtime_val(tsf_runtime_val),
    .openofdm_tx_iq0(openofdm_tx_iq0), .openofdm_tx_iq1(openofdm_tx_iq1),
    .openofdm_tx_iq_valid(openofdm_tx_iq_valid),
    .tx_intf_iq0(tx_intf_iq0), .tx_intf_iq1(tx_intf_iq1),
    .tx_intf_iq_valid(tx_intf_iq_valid),
    .iq0(iq0), .iq1(iq1), .iq_strobe(iq_strobe),
    .demod_is_ongoing(demod_is_ongoing),
    .ofdm_symbol_eq_out_pulse(ofdm_symbol_eq_out_pulse),
    .long_preamble_detected(long_preamble_detected),
    .short_preamble_detected(short_preamble_detected),
    .ht_unsupport(ht_unsupport), .pkt_rate(pkt_rate), .pkt_len(pkt_len),
    .csi(csi), .csi_valid(csi_valid),
    .phase_offset_taken(phase_offset_taken),
    .equalizer(equalizer), .equalizer_valid(equalizer_valid),
    .pkt_header_valid(pkt_header_valid), .pkt_header_valid_strobe(pkt_header_valid_strobe),
    .phy_type(phy_type),
    .tx_control_state(tx_control_state),
    .FC_DI(FC_DI), .FC_DI_valid(FC_DI_valid),
    .addr1(addr1), .addr1_valid(addr1_valid),
    .addr2(addr2), .addr2_valid(addr2_valid),
    .addr3(addr3), .addr3_valid(addr3_valid),
    .fcs_in_strobe(fcs_in_strobe), .fcs_ok(fcs_ok),
    .block_rx_dma_to_ps(block_rx_dma_to_ps),
    .block_rx_dma_to_ps_valid(block_rx_dma_to_ps_valid),
    .ch_idle_final(ch_idle_final),
    .phy_tx_start(phy_tx_start), .tx_pkt_need_ack(tx_pkt_need_ack),
    .phy_tx_started(phy_tx_started), .phy_tx_done(phy_tx_done),
    .tx_bb_is_ongoing(tx_bb_is_ongoing), .tx_rf_is_ongoing(tx_rf_is_ongoing),
    .tx_pkt_iq_to_dac_ongoing(tx_pkt_iq_to_dac_ongoing),
    .retrans_in_progress(retrans_in_progress),
    .slv_reg_wren_signal(slv_reg_wren_signal), .axi_awaddr_core(axi_awaddr_core),
    .iq_capture(iq_capture), .csi_iq_combined(csi_iq_combined),
    .iq_capture_cfg(iq_capture_cfg), .iq_trigger_select(iq_trigger_select),
    .iq_trigger_free_run_flag(iq_trigger_free_run_flag),
    .iq_source_select(iq_source_select),
    .disable_tx_pkt_need_ack_check(disable_tx_pkt_need_ack_check),
    .PPDU_FORMAT_target(PPDU_FORMAT_target),
    .rssi_or_iq_th(rssi_or_iq_th), .gain_th(gain_th),
    .pre_trigger_len(pre_trigger_len), .iq_len_target(iq_len_target),
    .tx_control_state_target(tx_control_state_target),
    .phy_type_target(phy_type_target),
    .FC_target(FC_target), .addr1_target(addr1_target), .addr2_target(addr2_target),
    .match_cfg(match_cfg), .num_eq(num_eq),
    .m_axis_start_mode(m_axis_start_mode),
    .m_axis_start_ext_trigger(m_axis_start_ext_trigger),
    .data_to_pl(data_to_pl), .pl_ask_data(pl_ask_data),
    .s_axis_data_count(s_axis_data_count), .emptyn_to_pl(emptyn_to_pl),
    .S_AXIS_TVALID(S_AXIS_TVALID), .S_AXIS_TLAST(S_AXIS_TLAST),
    .m_axis_start_1trans(m_axis_start_1trans),
    .data_to_ps(data_to_ps), .data_to_ps_valid(data_to_ps_valid),
    .m_axis_data_count(m_axis_data_count), .fulln_to_pl(fulln_to_pl),
    .MAX_NUM_DMA_SYMBOL_UDP_debug(MAX_NUM_DMA_SYMBOL_UDP_debug),
    .MAX_NUM_DMA_SYMBOL_debug(MAX_NUM_DMA_SYMBOL_debug),
    .M_AXIS_TVALID(M_AXIS_TVALID), .M_AXIS_TLAST(M_AXIS_TLAST)
  );

  // ---- continuous sample clock (20MHz strobe on a 100MHz clk) ----
  reg sample_en = 0;
  integer sample_idx = 0;
  always begin
    @(negedge clk);
    if (sample_en) begin
      iq_strobe = 1; iq0 = sample_idx; sample_idx = sample_idx + 1;
      @(negedge clk); iq_strobe = 0;
      repeat (3) @(negedge clk);
    end
  end

  // ---- monitor: capture m_axis write stream ----
  reg [63:0] cap [0:2047];
  integer wr_idx = 0;
  integer expected_total = (1+IQ_LEN) + (2+56+0); // num_eq=0
  always @(posedge clk) begin
    if (data_to_ps_valid) begin
      cap[wr_idx] <= data_to_ps;
      wr_idx <= wr_idx + 1;
    end
  end

  // ---- debug: state transition monitor ----
  reg [2:0] dbg_iq_state = 3'bxxx;
  reg [3:0] dbg_side_state = 4'bxxxx;
  always @(posedge clk) begin
    if (rstn) begin
      if (dut.iq_state !== dbg_iq_state || dut.side_ch_state !== dbg_side_state)
        $display("t=%0t iq_state %0d->%0d  side_ch_state %0d->%0d",
                 $time, dbg_iq_state, dut.iq_state, dbg_side_state, dut.side_ch_state);
      dbg_iq_state <= dut.iq_state;
      dbg_side_state <= dut.side_ch_state;
    end
  end

  integer failures = 0;
  task check_word(input integer idx, input reg [63:0] exp, input [8*32:1] name);
    begin
      if (cap[idx] !== exp) begin
        $display("FAIL %0s: word[%0d] = %h, expected %h", name, idx, cap[idx], exp);
        failures = failures + 1;
      end
    end
  endtask

  // HT20 subcarrier set-bit indices (order of CSI block output): 1..28, 36..63
  integer csi_idx [0:55];
  integer ci;

  initial begin : setup
    for (ci = 0; ci < 28; ci = ci + 1) csi_idx[ci] = ci + 1;
    for (ci = 0; ci < 28; ci = ci + 1) csi_idx[28 + ci] = ci + 36;
  end

  initial begin : test
    $dumpfile("side_ch_control_tb.vcd");
    $dumpvars(0, side_ch_control_tb);

    // reset
    rstn = 0;
    repeat (10) @(posedge clk);
    rstn = 1;
    @(posedge clk);

    // combined config
    iq_capture = 1; csi_iq_combined = 1;
    iq_capture_cfg = 0; iq_trigger_select = 0; iq_source_select = 0;
    pre_trigger_len = PRE; iq_len_target = IQ_LEN; num_eq = 0;
    match_cfg = 0; pkt_len = 100; pkt_rate = 0; // non-HT
    m_axis_start_mode = 1; m_axis_data_count = 0; fulln_to_pl = 1;
    phase_offset_taken = 32'h1234;

    // ---- frame begins; sample clock starts ----
    @(negedge clk);
    demod_is_ongoing = 1;
    sample_en = 1;

    // lock window after 100 samples (iq_waddr=100 -> iq_raddr=100-32=68)
    while (sample_idx < 100) @(posedge clk);
    @(negedge clk);
    tsf_runtime_val = 64'd1000;
    long_preamble_detected = 1;
    @(negedge clk);
    long_preamble_detected = 0;

    // channel-estimator output: 64 subcarriers as one continuous csi_valid burst
    @(negedge clk);
    csi_valid = 1;
    for (ci = 0; ci < 64; ci = ci + 1) begin
      csi = 100 + ci;
      @(negedge clk);
    end
    csi_valid = 0;

    // OFDM symbol pulse -> ofdm_rx FSM INIT->COUNT
    @(negedge clk); ofdm_symbol_eq_out_pulse = 1;
    @(negedge clk); ofdm_symbol_eq_out_pulse = 0;

    // FC / addr1 / addr2 filter (match_cfg=0 -> always pass)
    @(negedge clk); FC_DI_valid = 1;
    @(negedge clk); FC_DI_valid = 0;
    @(negedge clk); addr1_valid = 1;
    @(negedge clk); addr1_valid = 0;
    @(negedge clk); addr2_valid = 1;   // -> WAIT_FOR_CAPTURE_DONE = csi_commit
    @(negedge clk); addr2_valid = 0;

    // header parse -> OFDM_RX_END (last_ofdm_symbol_flag) + CSI TSF lock
    @(negedge clk);
    tsf_runtime_val = 64'd2000;
    pkt_header_valid = 0; pkt_header_valid_strobe = 1;
    @(negedge clk); pkt_header_valid_strobe = 0;

    // wait for the whole record to stream out
    repeat (1500) @(posedge clk);

    // ---- checks ----
    $display("total m_axis words captured = %0d (expected %0d)", wr_idx, expected_total);
    if (wr_idx !== expected_total) begin
      $display("FAIL record_len: got %0d words, expected %0d", wr_idx, expected_total);
      failures = failures + 1;
    end else begin
      check_word(0, 64'd1000, "iq_tsf");
      if (cap[1][31:0] !== 32'd68) begin
        $display("FAIL iq_first_sample: word[1][31:0] = %0d, expected 68", cap[1][31:0]);
        failures = failures + 1;
      end
      if (cap[IQ_LEN][31:0] !== 32'd131) begin
        $display("FAIL iq_last_sample: word[%0d][31:0] = %0d, expected 131", IQ_LEN, cap[IQ_LEN][31:0]);
        failures = failures + 1;
      end
      begin : contig
        integer kk;
        for (kk = 2; kk <= IQ_LEN; kk = kk + 1) begin
          if (cap[kk][31:0] !== cap[kk-1][31:0] + 1) begin
            $display("FAIL iq_contig: word[%0d]=%0d != word[%0d]+1", kk, cap[kk][31:0], kk-1);
            failures = failures + 1;
            disable contig;
          end
        end
      end
      check_word(1 + IQ_LEN, 64'd2000, "csi_tsf");
      check_word(2 + IQ_LEN, 64'h1234, "phase_offset");
      begin : csi_chk
        integer jj;
        integer expv;
        for (jj = 0; jj < 56; jj = jj + 1) begin
          expv = 100 + csi_idx[jj];
          if (cap[2 + IQ_LEN + 1 + jj][31:0] !== expv) begin
            $display("FAIL csi[%0d]: word[%0d][31:0] = %0d, expected %0d (subcarrier %0d)",
                     jj, 2 + IQ_LEN + 1 + jj, cap[2 + IQ_LEN + 1 + jj][31:0], expv, csi_idx[jj]);
            failures = failures + 1;
            disable csi_chk;
          end
        end
      end
    end

    if (failures == 0)
      $display("PASS: combined record length/order/TSF/IQ-window/CSI all correct");
    else
      $display("RESULT: %0d FAILURES", failures);

    $finish;
  end

endmodule
