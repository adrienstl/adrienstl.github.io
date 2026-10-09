// Systolic accelerator: everything around the N x N array, as seen from the
// RV32I core's MMIO bus.
//
// Data:
//   CPU -> mmio_regs -> load_buffer (N x N regs) ---------------> array, all N^2 at once
//                    -> operand_buffer A, B (N banks each)
//                       -> dataflow_router -> west / north skew -> array edges
//   array south edge -> output_collector -> output_buffer -> mmio_regs -> CPU
// Control:
//   mmio_regs (config, start) -> controller -> stat_load / acc_clear / drain to
//   every PE, the read offset to every operand bank, collect to the collector
//
// A job: the CPU fills the load buffer and / or operand banks, writes CONFIG and
// LEN, writes CTRL.start, waits for STATUS.done (or irq), reads the output
// buffer. Data layout per dataflow is in docs/accel_overview.html.
//
// stall is an array-wide backpressure hook: while it is high, every datapath
// register and the controller hold. Tie it to 0 if nothing needs it.
module accel_top
    import pe_pkg::*, accel_pkg::*;
#(
    parameter int N = 4
)(
    input  logic               clk,
    input  logic               reset,          // synchronous, active high

    // MMIO slave port (RV32I core side)
    input  logic               mmio_we,
    input  logic               mmio_re,
    input  logic [MMIO_AW-1:0] mmio_addr,
    input  logic [31:0]        mmio_wdata,
    output logic [31:0]        mmio_rdata,
    output logic               mmio_rvalid,
    output logic               irq,

    input  logic               stall
);

    // Simulation-only check (Quartus 18.1 can't run $error at elaboration)
    // synthesis translate_off
    generate
        if (N > 2**BANK_W || N*N > 2**LOAD_IDX_W) begin : g_n_check
            $error("accel_top: N = %0d does not fit the bank / load index fields", N);
        end
    endgenerate
    // synthesis translate_on

    logic en;
    assign en = ~stall;

    // ------------------------------------------------------------ CSR / MMIO
    phase_e                phase;
    logic                  busy, job_done, start;
    dataflow_e             cfg_dataflow;
    logic                  cfg_dual;
    logic [LEN_W-1:0]      cfg_len;

    logic                  load_we, a_we, b_we;
    logic [LOAD_IDX_W-1:0] load_idx;
    logic [BANK_W-1:0]     buf_bank;
    logic [OFF_W-1:0]      buf_off;
    logic [DATA_W-1:0]     buf_wdata;

    logic                  out_re, out_word;
    logic [BANK_W-1:0]     out_bank;
    logic [OFF_W-1:0]      out_off;
    logic [31:0]           out_rdata;

    mmio_regs #(.N(N)) u_regs (
        .clk          (clk),
        .reset        (reset),
        .mmio_we      (mmio_we),
        .mmio_re      (mmio_re),
        .mmio_addr    (mmio_addr),
        .mmio_wdata   (mmio_wdata),
        .mmio_rdata   (mmio_rdata),
        .mmio_rvalid  (mmio_rvalid),
        .irq          (irq),
        .phase        (phase),
        .busy         (busy),
        .job_done     (job_done),
        .start        (start),
        .cfg_dataflow (cfg_dataflow),
        .cfg_dual     (cfg_dual),
        .cfg_len      (cfg_len),
        .load_we      (load_we),
        .load_idx     (load_idx),
        .a_we         (a_we),
        .b_we         (b_we),
        .buf_bank     (buf_bank),
        .buf_off      (buf_off),
        .buf_wdata    (buf_wdata),
        .out_re       (out_re),
        .out_bank     (out_bank),
        .out_off      (out_off),
        .out_word     (out_word),
        .out_rdata    (out_rdata)
    );

    // ------------------------------------------------------------ controller
    logic             stat_load, acc_clear, drain;
    logic             rd_en, rd_valid, collect;
    logic [OFF_W-1:0] rd_addr;

    controller #(.N(N)) u_ctrl (
        .clk          (clk),
        .reset        (reset),
        .en           (en),
        .start        (start),
        .cfg_dataflow (cfg_dataflow),
        .cfg_len      (cfg_len),
        .phase        (phase),
        .busy         (busy),
        .done         (job_done),
        .stat_load    (stat_load),
        .acc_clear    (acc_clear),
        .drain        (drain),
        .rd_en        (rd_en),
        .rd_addr      (rd_addr),
        .rd_valid     (rd_valid),
        .collect      (collect)
    );

    // ------------------------------------------------------------ storage in
    logic [N-1:0][N-1:0][DATA_W-1:0] load_q;

    load_buffer #(.N(N)) u_load (
        .clk   (clk),
        .reset (reset),
        .we    (load_we),
        .idx   (load_idx),
        .wdata (buf_wdata),
        .q     (load_q)
    );

    logic                     a_re, b_re;
    logic [N-1:0][DATA_W-1:0] a_data, b_data;

    operand_buffer #(.BANKS(N)) u_buf_a (
        .clk   (clk),
        .we    (a_we),
        .wbank (buf_bank),
        .woff  (buf_off),
        .wdata (buf_wdata),
        .re    (a_re),
        .roff  (rd_addr),
        .rdata (a_data)
    );

    operand_buffer #(.BANKS(N)) u_buf_b (
        .clk   (clk),
        .we    (b_we),
        .wbank (buf_bank),
        .woff  (buf_off),
        .wdata (buf_wdata),
        .re    (b_re),
        .roff  (rd_addr),
        .rdata (b_data)
    );

    // ------------------------------------------------------------ router + skews
    logic [N-1:0][DATA_W-1:0] west_data, north_data, west_skewed, north_skewed;
    logic [N-1:0]             west_valid, north_valid, west_skewed_v, north_skewed_v;

    dataflow_router #(.N(N)) u_router (
        .cfg_dataflow (cfg_dataflow),
        .rd_en        (rd_en),
        .rd_valid     (rd_valid),
        .a_re         (a_re),
        .b_re         (b_re),
        .a_data       (a_data),
        .b_data       (b_data),
        .west_data    (west_data),
        .west_valid   (west_valid),
        .north_data   (north_data),
        .north_valid  (north_valid)
    );

    skew #(.LANES(N), .W(DATA_W)) u_west_skew (
        .clk       (clk),
        .reset     (reset),
        .en        (en),
        .in_data   (west_data),
        .in_valid  (west_valid),
        .out_data  (west_skewed),
        .out_valid (west_skewed_v)
    );

    skew #(.LANES(N), .W(DATA_W)) u_north_skew (
        .clk       (clk),
        .reset     (reset),
        .en        (en),
        .in_data   (north_data),
        .in_valid  (north_valid),
        .out_data  (north_skewed),
        .out_valid (north_skewed_v)
    );

    // ------------------------------------------------------------ array
    logic [N-1:0][PSUM_W-1:0] south_psum;
    logic [N-1:0]             south_valid;

    systolic_array #(.ROWS(N), .COLS(N)) u_array (
        .clk              (clk),
        .reset            (reset),
        .cfg_dataflow     (cfg_dataflow),
        .cfg_dual         (cfg_dual),
        .en               (en),
        .stat_load        (stat_load),
        .acc_clear        (acc_clear),
        .drain            (drain),
        .load_in          (load_q),
        .west_h           (west_skewed),
        .west_h_valid     (west_skewed_v),
        .north_v          (north_skewed),
        .north_v_valid    (north_skewed_v),
        .south_psum       (south_psum),
        .south_psum_valid (south_valid)
    );

    // ------------------------------------------------------------ storage out
    logic [N-1:0]            wr_en;
    logic [N-1:0][OFF_W-1:0] wr_addr;
    logic [N-1:0][OUT_W-1:0] wr_data;

    output_collector #(.N(N)) u_collect (
        .clk          (clk),
        .reset        (reset),
        .job_start    (start),
        .en           (en),
        .collect      (collect),
        .cfg_dataflow (cfg_dataflow),
        .cfg_dual     (cfg_dual),
        .psum         (south_psum),
        .psum_valid   (south_valid),
        .wr_en        (wr_en),
        .wr_addr      (wr_addr),
        .wr_data      (wr_data)
    );

    output_buffer #(.N(N)) u_out (
        .clk     (clk),
        .wr_en   (wr_en),
        .wr_addr (wr_addr),
        .wr_data (wr_data),
        .rd_en   (out_re),
        .rd_bank (out_bank),
        .rd_off  (out_off),
        .rd_word (out_word),
        .rd_data (out_rdata)
    );

endmodule
