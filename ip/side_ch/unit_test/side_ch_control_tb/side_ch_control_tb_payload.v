// side_ch_control_tb_payload.v -- P2 combined payload equivalence
//
// One stimulus, three DUTs:
//   dut      : new RTL, combined mode (iq_capture=1, csi_iq_combined=1)
//   csi_ref  : pre-P2 RTL, pure CSI   (iq_capture=0)
//   iq_ref   : pre-P2 RTL, pure IQ    (iq_capture=1, iq_trigger_select=8)
//
// Checks:
//   combined[1+IQ_LEN ..]  == pure-CSI record   (2 header + 56 CSI words)
//   combined[0 .. IQ_LEN]  == pure-IQ record    (1 TSF + IQ_LEN samples)
//
// Run via run_payload.sh (extracts fa6c896 RTL as side_ch_control_ref).
`timescale 1ns/1ps

module side_ch_control_tb_payload;

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
  reg iq_capture = 1;          // -> dut (combined)
  reg csi_iq_combined = 1;     // -> dut
  reg csi_iq_capture = 0;      // -> csi_ref
  reg iq_iq_capture = 1;       // -> iq_ref
  reg [4:0] iq_trigger_select = 0;     // -> dut
  reg [4:0] iq_trigger_select_iq = 8;  // -> iq_ref: long_preamble_detected
  reg [1:0]  iq_capture_cfg = 0;
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

  wire m_axis_start_1trans, pl_ask_data, data_to_ps_valid;
  wire [63:0] data_to_ps;
  wire [31:0] dbg_udp, dbg_max;
  wire csi_valid_w, csi_data_valid, iq_data_valid;
  wire [63:0] csi_data, iq_data;
  wire csi_pl_ask, iq_pl_ask, csi_start, iq_start;
  wire [31:0] csi_udp, csi_max, iq_udp, iq_max;

  // ------------------------------------------------------------------
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
    .phy_type(phy_type), .tx_control_state(tx_control_state),
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
    .MAX_NUM_DMA_SYMBOL_UDP_debug(dbg_udp),
    .MAX_NUM_DMA_SYMBOL_debug(dbg_max),
    .M_AXIS_TVALID(M_AXIS_TVALID), .M_AXIS_TLAST(M_AXIS_TLAST)
  );

  // ------------------------------------------------------------------
  side_ch_control_ref #(
    .TSF_TIMER_WIDTH(64), .GPIO_STATUS_WIDTH(8), .RSSI_HALF_DB_WIDTH(11),
    .C_S_AXI_DATA_WIDTH(32), .IQ_DATA_WIDTH(16), .C_S_AXIS_TDATA_WIDTH(64),
    .MAX_NUM_DMA_SYMBOL(4096), .MAX_BIT_NUM_DMA_SYMBOL(13)
  ) csi_ref (
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
    .phy_type(phy_type), .tx_control_state(tx_control_state),
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
    .iq_capture(csi_iq_capture),
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
    .data_to_pl(data_to_pl), .pl_ask_data(csi_pl_ask),
    .s_axis_data_count(s_axis_data_count), .emptyn_to_pl(emptyn_to_pl),
    .S_AXIS_TVALID(S_AXIS_TVALID), .S_AXIS_TLAST(S_AXIS_TLAST),
    .m_axis_start_1trans(csi_start),
    .data_to_ps(csi_data), .data_to_ps_valid(csi_data_valid),
    .m_axis_data_count(m_axis_data_count), .fulln_to_pl(fulln_to_pl),
    .MAX_NUM_DMA_SYMBOL_UDP_debug(csi_udp),
    .MAX_NUM_DMA_SYMBOL_debug(csi_max),
    .M_AXIS_TVALID(M_AXIS_TVALID), .M_AXIS_TLAST(M_AXIS_TLAST)
  );

  // ------------------------------------------------------------------
  side_ch_control_ref #(
    .TSF_TIMER_WIDTH(64), .GPIO_STATUS_WIDTH(8), .RSSI_HALF_DB_WIDTH(11),
    .C_S_AXI_DATA_WIDTH(32), .IQ_DATA_WIDTH(16), .C_S_AXIS_TDATA_WIDTH(64),
    .MAX_NUM_DMA_SYMBOL(4096), .MAX_BIT_NUM_DMA_SYMBOL(13)
  ) iq_ref (
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
    .phy_type(phy_type), .tx_control_state(tx_control_state),
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
    .iq_capture(iq_iq_capture), .iq_trigger_select(iq_trigger_select_iq),
    .iq_capture_cfg(iq_capture_cfg),
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
    .data_to_pl(data_to_pl), .pl_ask_data(iq_pl_ask),
    .s_axis_data_count(s_axis_data_count), .emptyn_to_pl(emptyn_to_pl),
    .S_AXIS_TVALID(S_AXIS_TVALID), .S_AXIS_TLAST(S_AXIS_TLAST),
    .m_axis_start_1trans(iq_start),
    .data_to_ps(iq_data), .data_to_ps_valid(iq_data_valid),
    .m_axis_data_count(m_axis_data_count), .fulln_to_pl(fulln_to_pl),
    .MAX_NUM_DMA_SYMBOL_UDP_debug(iq_udp),
    .MAX_NUM_DMA_SYMBOL_debug(iq_max),
    .M_AXIS_TVALID(M_AXIS_TVALID), .M_AXIS_TLAST(M_AXIS_TLAST)
  );

  // ---- 20MHz sample strobe ----
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

  // ---- capture three streams ----
  reg [63:0] cap_comb [0:255];
  reg [63:0] cap_csi  [0:255];
  reg [63:0] cap_iq   [0:255];
  integer n_comb = 0, n_csi = 0, n_iq = 0;
  always @(posedge clk) begin
    if (data_to_ps_valid) begin cap_comb[n_comb] <= data_to_ps; n_comb <= n_comb + 1; end
    if (csi_data_valid)   begin cap_csi[n_csi]   <= csi_data; n_csi  <= n_csi + 1; end
    if (iq_data_valid)    begin cap_iq[n_iq]     <= iq_data;  n_iq   <= n_iq + 1; end
  end

  integer failures = 0;
  integer k;
  integer csi_expect;

  initial begin : test
    $dumpfile("side_ch_control_tb_payload.vcd");
    $dumpvars(0, side_ch_control_tb_payload);

    rstn = 0;
    repeat (10) @(posedge clk);
    rstn = 1;
    @(posedge clk);

    iq_capture = 1; csi_iq_combined = 1;
    csi_iq_capture = 0; iq_iq_capture = 1;
    iq_trigger_select = 0; iq_trigger_select_iq = 8;
    iq_capture_cfg = 0; iq_source_select = 0;
    pre_trigger_len = PRE; iq_len_target = IQ_LEN; num_eq = 0;
    match_cfg = 0; pkt_len = 100; pkt_rate = 0;
    FC_DI = 32'h00A0_0801;
    addr1 = 48'h0011_2233_4455;
    addr2 = 48'h6677_8899_AABB;
    addr3 = 48'hCCDD_EEFF_0011;
    m_axis_start_mode = 1; m_axis_data_count = 0; fulln_to_pl = 1;
    phase_offset_taken = 32'h1234;

    @(negedge clk); demod_is_ongoing = 1;
    sample_en = 1;

    while (sample_idx < 100) @(posedge clk);
    @(negedge clk);
    tsf_runtime_val = 64'd1000;
    long_preamble_detected = 1;
    @(negedge clk);
    long_preamble_detected = 0;

    @(negedge clk); csi_valid = 1;
    for (k = 0; k < 64; k = k + 1) begin
      csi = 100 + k;
      @(negedge clk);
    end
    csi_valid = 0;

    @(negedge clk); ofdm_symbol_eq_out_pulse = 1;
    @(negedge clk); ofdm_symbol_eq_out_pulse = 0;

    @(negedge clk); FC_DI_valid = 1;
    @(negedge clk); FC_DI_valid = 0;
    @(negedge clk); addr1_valid = 1;
    @(negedge clk); addr1_valid = 0;
    @(negedge clk); addr2_valid = 1;
    @(negedge clk); addr2_valid = 0;
    @(negedge clk); addr3_valid = 1;
    @(negedge clk); addr3_valid = 0;

    @(negedge clk);
    tsf_runtime_val = 64'd2000;
    pkt_header_valid = 0; pkt_header_valid_strobe = 1;
    @(negedge clk); pkt_header_valid_strobe = 0;

    repeat (1500) @(posedge clk);

    $display("words: combined=%0d (expect %0d), csi_ref=%0d (expect 58), iq_ref=%0d (expect %0d)",
             n_comb, 1 + IQ_LEN + 58 + 3, n_csi, n_iq, 1 + IQ_LEN);
    if (n_comb !== 1 + IQ_LEN + 58 + 3) failures = failures + 1;
    if (n_csi !== 58) failures = failures + 1;
    if (n_iq !== 1 + IQ_LEN) failures = failures + 1;

    // combined[1+IQ_LEN ...] == pure CSI record
    for (k = 0; k < 58; k = k + 1) begin
      if (cap_comb[1 + IQ_LEN + k] !== cap_csi[k]) begin
        if (failures < 8)
          $display("FAIL csi[%0d]: comb=%h csi_ref=%h", k,
                   cap_comb[1 + IQ_LEN + k], cap_csi[k]);
        failures = failures + 1;
      end
    end

    // combined[0 ...] == pure IQ record
    for (k = 0; k < 1 + IQ_LEN; k = k + 1) begin
      if (cap_comb[k] !== cap_iq[k]) begin
        if (failures < 8)
          $display("FAIL iq[%0d]: comb=%h iq_ref=%h", k, cap_comb[k], cap_iq[k]);
        failures = failures + 1;
      end
    end

    // P2.1 MAC header block appended after the 58-word CSI record
    if (cap_comb[1 + IQ_LEN + 58] !== 64'h22334455_00A00801) begin
      $display("FAIL mac_w0: %h", cap_comb[1 + IQ_LEN + 58]); failures = failures + 1;
    end
    if (cap_comb[1 + IQ_LEN + 59] !== 64'h66778899AABB_0011) begin
      $display("FAIL mac_w1: %h", cap_comb[1 + IQ_LEN + 59]); failures = failures + 1;
    end
    if (cap_comb[1 + IQ_LEN + 60] !== 64'h0064_CCDDEEFF0011) begin
      $display("FAIL mac_w2: %h", cap_comb[1 + IQ_LEN + 60]); failures = failures + 1;
    end

    if (failures == 0)
      $display("PASS: combined CSI block == pure-CSI record; combined IQ block == pure-IQ record");
    else
      $display("RESULT: %0d payload mismatches", failures);

    $finish;
  end

endmodule
