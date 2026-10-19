// side_ch_control_tb_bypass.v -- L1 bypass-equivalence testbench
//
// Purpose: prove that with csi_iq_combined=0 (reg3[1]=0), the current RTL's
// pure-CSI (iq_capture=0) and pure-IQ (iq_capture=1) paths are functionally
// identical to the pre-P2 RTL (fa6c896, module renamed to side_ch_control_ref).
//
// Both DUTs get the exact same stimulus. Outputs are compared every clock:
//   side_info / side_info_valid (internal), data_to_ps / data_to_ps_valid,
//   pl_ask_data, m_axis_start_1trans.
//
// Phase A: pure CSI (reg3=0)  -> expect 2+56 = 58 words
// Phase B: pure IQ  (reg3=1)  -> expect 1+IQ_LEN = 65 words
//
// Run via run_bypass.sh (extracts fa6c896 RTL from git, renames the module).
`timescale 1ns/1ps

module side_ch_control_tb_bypass;

  parameter IQ_LEN = 64;
  parameter PRE    = 32;

  reg clk = 0;
  always #5 clk = ~clk;

  // ---- shared stimulus ----
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
  reg iq_capture = 0;
  reg csi_iq_combined = 0;   // fixed 0: bypass equivalence
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

  // ---- outputs: new ----
  wire m_axis_start_1trans_n, pl_ask_data_n, data_to_ps_valid_n;
  wire [63:0] data_to_ps_n;
  wire [31:0] dbg_udp_n, dbg_max_n;

  // ---- outputs: reference ----
  wire m_axis_start_1trans_r, pl_ask_data_r, data_to_ps_valid_r;
  wire [63:0] data_to_ps_r;
  wire [31:0] dbg_udp_r, dbg_max_r;

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
    .data_to_pl(data_to_pl), .pl_ask_data(pl_ask_data_n),
    .s_axis_data_count(s_axis_data_count), .emptyn_to_pl(emptyn_to_pl),
    .S_AXIS_TVALID(S_AXIS_TVALID), .S_AXIS_TLAST(S_AXIS_TLAST),
    .m_axis_start_1trans(m_axis_start_1trans_n),
    .data_to_ps(data_to_ps_n), .data_to_ps_valid(data_to_ps_valid_n),
    .m_axis_data_count(m_axis_data_count), .fulln_to_pl(fulln_to_pl),
    .MAX_NUM_DMA_SYMBOL_UDP_debug(dbg_udp_n),
    .MAX_NUM_DMA_SYMBOL_debug(dbg_max_n),
    .M_AXIS_TVALID(M_AXIS_TVALID), .M_AXIS_TLAST(M_AXIS_TLAST)
  );

  // Pre-P2 RTL (fa6c896) has no csi_iq_combined port.
  side_ch_control_ref #(
    .TSF_TIMER_WIDTH(64), .GPIO_STATUS_WIDTH(8), .RSSI_HALF_DB_WIDTH(11),
    .C_S_AXI_DATA_WIDTH(32), .IQ_DATA_WIDTH(16), .C_S_AXIS_TDATA_WIDTH(64),
    .MAX_NUM_DMA_SYMBOL(4096), .MAX_BIT_NUM_DMA_SYMBOL(13)
  ) ref_dut (
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
    .iq_capture(iq_capture),
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
    .data_to_pl(data_to_pl), .pl_ask_data(pl_ask_data_r),
    .s_axis_data_count(s_axis_data_count), .emptyn_to_pl(emptyn_to_pl),
    .S_AXIS_TVALID(S_AXIS_TVALID), .S_AXIS_TLAST(S_AXIS_TLAST),
    .m_axis_start_1trans(m_axis_start_1trans_r),
    .data_to_ps(data_to_ps_r), .data_to_ps_valid(data_to_ps_valid_r),
    .m_axis_data_count(m_axis_data_count), .fulln_to_pl(fulln_to_pl),
    .MAX_NUM_DMA_SYMBOL_UDP_debug(dbg_udp_r),
    .MAX_NUM_DMA_SYMBOL_debug(dbg_max_r),
    .M_AXIS_TVALID(M_AXIS_TVALID), .M_AXIS_TLAST(M_AXIS_TLAST)
  );

  // ---- continuous 20MHz sample strobe ----
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

  // ---- per-mode word counters ----
  integer csi_words = 0, iq_words = 0;
  always @(posedge clk) begin
    if (data_to_ps_valid_n) begin
      if (iq_capture) iq_words <= iq_words + 1;
      else            csi_words <= csi_words + 1;
    end
  end

  // ---- cycle-by-cycle equivalence checker (sample after posedge settle) ----
  integer mismatches = 0;
  reg compare_en = 0;
  always @(negedge clk) begin
    if (compare_en) begin
      if (data_to_ps_valid_n !== data_to_ps_valid_r) begin
        mismatches = mismatches + 1;
        if (mismatches <= 20)
          $display("MISMATCH t=%0t data_to_ps_valid new=%b ref=%b", $time, data_to_ps_valid_n, data_to_ps_valid_r);
      end else if (data_to_ps_valid_n && (data_to_ps_n !== data_to_ps_r)) begin
        mismatches = mismatches + 1;
        if (mismatches <= 20)
          $display("MISMATCH t=%0t data_to_ps new=%h ref=%h", $time, data_to_ps_n, data_to_ps_r);
      end

      if (pl_ask_data_n !== pl_ask_data_r) begin
        mismatches = mismatches + 1;
        if (mismatches <= 20)
          $display("MISMATCH t=%0t pl_ask_data new=%b ref=%b", $time, pl_ask_data_n, pl_ask_data_r);
      end
      if (m_axis_start_1trans_n !== m_axis_start_1trans_r) begin
        mismatches = mismatches + 1;
        if (mismatches <= 20)
          $display("MISMATCH t=%0t m_axis_start_1trans new=%b ref=%b", $time, m_axis_start_1trans_n, m_axis_start_1trans_r);
      end

      // side_info only matters when valid; stale data when valid=0 may differ.
      if (dut.side_info_valid !== ref_dut.side_info_valid) begin
        mismatches = mismatches + 1;
        if (mismatches <= 20)
          $display("MISMATCH t=%0t side_info_valid new=%b ref=%b", $time, dut.side_info_valid, ref_dut.side_info_valid);
      end else if (dut.side_info_valid && (dut.side_info !== ref_dut.side_info)) begin
        mismatches = mismatches + 1;
        if (mismatches <= 20)
          $display("MISMATCH t=%0t side_info new=%h ref=%h", $time, dut.side_info, ref_dut.side_info);
      end
    end
  end

  integer ci;
  integer phase_fail = 0;

  initial begin : test
    $dumpfile("side_ch_control_tb_bypass.vcd");
    $dumpvars(0, side_ch_control_tb_bypass);

    // common config
    iq_capture_cfg = 0; iq_trigger_select = 0; iq_source_select = 0;
    pre_trigger_len = PRE; iq_len_target = IQ_LEN; num_eq = 0;
    match_cfg = 0; pkt_len = 100; pkt_rate = 0;
    m_axis_start_mode = 1; m_axis_data_count = 0; fulln_to_pl = 1;
    phase_offset_taken = 32'h1234;

    // ================= Phase A: pure CSI (reg3=0) =================
    rstn = 0; compare_en = 0; iq_capture = 0; csi_iq_combined = 0;
    repeat (10) @(posedge clk);
    rstn = 1;
    repeat (3) @(posedge clk);
    compare_en = 1;

    @(negedge clk); demod_is_ongoing = 1;

    // 64-subcarrier channel estimate burst
    @(negedge clk); csi_valid = 1;
    for (ci = 0; ci < 64; ci = ci + 1) begin
      csi = 100 + ci;
      @(negedge clk);
    end
    csi_valid = 0;

    @(negedge clk); ofdm_symbol_eq_out_pulse = 1;
    @(negedge clk); ofdm_symbol_eq_out_pulse = 0;

    @(negedge clk); FC_DI_valid = 1;
    @(negedge clk); FC_DI_valid = 0;
    @(negedge clk); addr1_valid = 1;
    @(negedge clk); addr1_valid = 0;
    @(negedge clk); addr2_valid = 1;   // -> WAIT_FOR_CAPTURE_DONE
    @(negedge clk); addr2_valid = 0;

    @(negedge clk);
    tsf_runtime_val = 64'd2000;
    pkt_header_valid = 0; pkt_header_valid_strobe = 1;
    @(negedge clk); pkt_header_valid_strobe = 0;

    repeat (400) @(posedge clk);

    $display("PHASE A (pure CSI): words=%0d (expected 58)", csi_words);
    if (csi_words !== 58) begin
      $display("FAIL phase A word count: %0d != 58", csi_words);
      phase_fail = phase_fail + 1;
    end

    // ================= Phase B: pure IQ (reg3=1) =================
    compare_en = 0;
    rstn = 0;
    sample_en = 0; sample_idx = 0;
    iq_strobe = 0; iq0 = 0; iq1 = 0;
    demod_is_ongoing = 0;
    csi_valid = 0; ofdm_symbol_eq_out_pulse = 0;
    FC_DI_valid = 0; addr1_valid = 0; addr2_valid = 0;
    pkt_header_valid_strobe = 0; fcs_in_strobe = 0;
    repeat (5) @(posedge clk);

    rstn = 1;
    iq_capture = 1; csi_iq_combined = 0;
    iq_trigger_select = 0;
    repeat (3) @(posedge clk);
    compare_en = 1;

    @(negedge clk); demod_is_ongoing = 1;
    sample_en = 1;

    while (sample_idx < 100) @(posedge clk);
    @(negedge clk); tsf_runtime_val = 64'd3000;
    @(negedge clk); fcs_in_strobe = 1;   // iq_trigger_select=0 -> iq_trigger
    @(negedge clk); fcs_in_strobe = 0;

    repeat (400) @(posedge clk);

    $display("PHASE B (pure IQ): words=%0d (expected 65)", iq_words);
    if (iq_words !== 65) begin
      $display("FAIL phase B word count: %0d != 65", iq_words);
      phase_fail = phase_fail + 1;
    end

    // ================= verdict =================
    if (mismatches == 0 && phase_fail == 0)
      $display("PASS: bypass paths (reg3=0 / reg3=1) are cycle-identical to fa6c896 RTL");
    else
      $display("RESULT: %0d mismatches, %0d phase failures", mismatches, phase_fail);

    $finish;
  end

endmodule
