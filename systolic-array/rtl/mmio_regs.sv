// CSR / MMIO block: the CPU's only view of the accelerator.
//
// Decodes every bus access (map in accel_pkg), holds config / status, turns a
// CTRL write into a start pulse, and steers writes into the load buffer and
// operand banks and reads out of the output buffer.
//
// Bus: one-cycle mmio_we / mmio_re strobes, 32-bit words only. Read data comes
// back the next cycle with mmio_rvalid (the output buffer is a synchronous RAM,
// so CSR reads are registered too to keep one latency).
//
// While the accelerator is busy, writes to CONFIG, LEN and the buffers are
// ignored, so a job's inputs can't change under it.
module mmio_regs
    import pe_pkg::*, accel_pkg::*;
#(
    parameter int N = 4
)(
    input  logic                  clk,
    input  logic                  reset,

    // Bus (from the RV32I core)
    input  logic                  mmio_we,
    input  logic                  mmio_re,
    input  logic [MMIO_AW-1:0]    mmio_addr,
    input  logic [31:0]           mmio_wdata,
    output logic [31:0]           mmio_rdata,
    output logic                  mmio_rvalid,
    output logic                  irq,            // level: high while DONE is set

    // Controller
    input  phase_e                phase,
    input  logic                  busy,
    input  logic                  job_done,       // one-cycle pulse
    output logic                  start,          // one-cycle pulse, config already checked

    // Configuration, held while busy
    output dataflow_e             cfg_dataflow,
    output logic                  cfg_dual,
    output logic [LEN_W-1:0]      cfg_len,

    // Write ports into storage (data shared by all three)
    output logic                  load_we,
    output logic [LOAD_IDX_W-1:0] load_idx,
    output logic                  a_we,
    output logic                  b_we,
    output logic [BANK_W-1:0]     buf_bank,
    output logic [OFF_W-1:0]      buf_off,
    output logic [DATA_W-1:0]     buf_wdata,

    // Output buffer read port
    output logic                  out_re,
    output logic [BANK_W-1:0]     out_bank,
    output logic [OFF_W-1:0]      out_off,
    output logic                  out_word,
    input  logic [31:0]           out_rdata       // valid the cycle after out_re
);

    // ------------------------------------------------------------ decode
    region_e         region;
    logic            is_csr, is_load;
    logic [5:0]      idx;

    assign region  = region_e'(mmio_addr[RGN_LSB +: 2]);
    assign is_csr  = (region == RGN_CTRL) & ~mmio_addr[LOAD_SEL_BIT];
    assign is_load = (region == RGN_CTRL) &  mmio_addr[LOAD_SEL_BIT];
    assign idx     = mmio_addr[IDX_LSB +: 6];

    logic wr_ok;                         // storage / config writes: only while idle
    assign wr_ok = mmio_we & ~busy;

    assign load_we   = wr_ok & is_load;
    assign load_idx  = idx[LOAD_IDX_W-1:0];
    assign a_we      = wr_ok & (region == RGN_A);
    assign b_we      = wr_ok & (region == RGN_B);
    assign buf_bank  = mmio_addr[BUF_BANK_LSB +: BANK_W];
    assign buf_off   = mmio_addr[BUF_OFF_LSB +: OFF_W];
    assign buf_wdata = mmio_wdata[DATA_W-1:0];

    assign out_re    = mmio_re & (region == RGN_OUT);
    assign out_bank  = mmio_addr[OUT_BANK_LSB +: BANK_W];
    assign out_off   = mmio_addr[OUT_OFF_LSB +: OFF_W];
    assign out_word  = mmio_addr[OUT_WORD_BIT];

    // ------------------------------------------------------------ start / config check
    logic ctrl_wr, start_req, cfg_ok;
    logic [LEN_W-1:0] len_max;

    assign ctrl_wr   = mmio_we & is_csr & (idx == CSR_CTRL);
    assign start_req = ctrl_wr & mmio_wdata[CTRL_START] & ~busy;

    // OS sums over the whole stream, so LEN is capped by the psum guard bits
    assign len_max = (cfg_dataflow == DF_OS) ? LEN_W'(K_MAX) : LEN_W'(BUF_DEPTH);
    assign cfg_ok  = (cfg_dataflow == DF_WS || cfg_dataflow == DF_IS || cfg_dataflow == DF_OS) &&
                     (cfg_len != '0) && (cfg_len <= len_max);

    assign start = start_req & cfg_ok;

    // ------------------------------------------------------------ registers
    logic        done_q, error_q;
    logic [31:0] cycles_q;

    always_ff @(posedge clk) begin
        if (reset) begin
            cfg_dataflow <= DF_WS;
            cfg_dual     <= 1'b0;
            cfg_len      <= '0;
        end else if (wr_ok && is_csr) begin
            if (idx == CSR_CONFIG) begin
                cfg_dataflow <= dataflow_e'(mmio_wdata[1:0]);
                cfg_dual     <= mmio_wdata[2];
            end
            if (idx == CSR_LEN) cfg_len <= mmio_wdata[LEN_W-1:0];
        end
    end

    // done and error are sticky; a new start or a CLEAR write drops them
    always_ff @(posedge clk) begin
        if (reset) begin
            done_q  <= 1'b0;
            error_q <= 1'b0;
        end else begin
            if (job_done)                                          done_q <= 1'b1;
            else if (start_req || (ctrl_wr && mmio_wdata[CTRL_CLEAR])) done_q <= 1'b0;

            if (start_req)                                         error_q <= ~cfg_ok;
            else if (ctrl_wr && mmio_wdata[CTRL_CLEAR])            error_q <= 1'b0;
        end
    end

    // Cycle counter: every cycle from SETUP to DONE, stalls included
    always_ff @(posedge clk) begin
        if (reset)      cycles_q <= '0;
        else if (start) cycles_q <= '0;
        else if (busy)  cycles_q <= cycles_q + 1'b1;
    end

    assign irq = done_q;

    // ------------------------------------------------------------ read path
    logic [31:0] csr_rdata, csr_q;
    logic        from_out_q;

    always_comb begin
        csr_rdata = '0;
        if (is_csr) begin
            case (idx)
                CSR_STATUS: begin
                    csr_rdata[ST_BUSY]              = busy;
                    csr_rdata[ST_DONE]              = done_q;
                    csr_rdata[ST_ERROR]             = error_q;
                    csr_rdata[ST_PHASE_LSB +: 3]    = phase;
                end
                CSR_CONFIG: csr_rdata[2:0]          = {cfg_dual, cfg_dataflow};
                CSR_LEN:    csr_rdata[LEN_W-1:0]    = cfg_len;
                CSR_CYCLES: csr_rdata               = cycles_q;
                CSR_INFO:   csr_rdata               = {8'(K_MAX), 16'(BUF_DEPTH), 8'(N)};
                default:    csr_rdata               = '0;
            endcase
        end
    end

    always_ff @(posedge clk) begin
        if (reset) begin
            mmio_rvalid <= 1'b0;
            from_out_q  <= 1'b0;
            csr_q       <= '0;
        end else begin
            mmio_rvalid <= mmio_re;
            if (mmio_re) begin
                from_out_q <= (region == RGN_OUT);
                csr_q      <= csr_rdata;
            end
        end
    end

    assign mmio_rdata = from_out_q ? out_rdata : csr_q;

endmodule
