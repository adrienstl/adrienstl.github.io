// Controller FSM: sequences one job through its phases.
//
//   Phase    WS / IS                           OS
//   IDLE     wait for start                    wait for start
//   SETUP    stat_load: load buffer -> PEs     acc_clear
//   STREAM   read offset 0 .. LEN-1 from every operand bank, one per cycle
//   WAIT     last wavefront leaves the array   last PE finishes its MACs
//   DRAIN    -                                 drain for N cycles
//   DONE     one cycle, raises done for the CPU
//
// SETUP, STREAM, WAIT and DRAIN only advance on enabled cycles (en = 1), the
// same as every other register in the datapath. IDLE and DONE don't depend on
// en, so a start or done pulse is never lost to a stall.
module controller
    import pe_pkg::*, accel_pkg::*;
#(
    parameter int N = 4
)(
    input  logic             clk,
    input  logic             reset,
    input  logic             en,

    // From the CSR block (config is held while busy)
    input  logic             start,        // one-cycle pulse, config already checked
    input  dataflow_e        cfg_dataflow,
    input  logic [LEN_W-1:0] cfg_len,

    output phase_e           phase,
    output logic             busy,
    output logic             done,         // one-cycle pulse

    // Array control
    output logic             stat_load,
    output logic             acc_clear,
    output logic             drain,

    // Read address counter: same offset to every operand bank
    output logic             rd_en,        // read this cycle (en already folded in)
    output logic [OFF_W-1:0] rd_addr,
    output logic             rd_valid,     // bank outputs hold a real item (one cycle after rd_en)

    // Output collector: capture valid psums on the south edge
    output logic             collect
);

    // Wait lengths, counted from the last STREAM cycle (bank read latency 1,
    // skew r cycles, one register per PE hop):
    //   WS / IS: item m of column j is captured 2 + N + j cycles after its read,
    //            so the last one (column N-1) needs 2N + 1 cycles
    //   OS     : PE (i, j) does its last MAC 2 + i + j cycles after the last read,
    //            so PE (N-1, N-1) is done after 2N cycles and drain can start
    localparam int WAIT_WSIS = 2*N + 1;
    localparam int WAIT_OS   = 2*N;

    logic is_os;
    assign is_os = (cfg_dataflow == DF_OS);

    // ------------------------------------------------------------ phase counter
    // One counter, reused by every phase: counts enabled cycles, back to 0 on a phase change
    logic [LEN_W-1:0] cnt, phase_len;
    logic             last;

    always_comb begin
        case (phase)
            PH_STREAM: phase_len = cfg_len;
            PH_WAIT:   phase_len = is_os ? LEN_W'(WAIT_OS) : LEN_W'(WAIT_WSIS);
            PH_DRAIN:  phase_len = LEN_W'(N);
            default:   phase_len = LEN_W'(1);
        endcase
    end

    assign last = (cnt == phase_len - 1'b1);

    // ------------------------------------------------------------ next phase
    phase_e phase_next;

    always_comb begin
        phase_next = phase;
        case (phase)
            PH_IDLE:   if (start)       phase_next = PH_SETUP;
            PH_SETUP:  if (en && last)  phase_next = PH_STREAM;
            PH_STREAM: if (en && last)  phase_next = PH_WAIT;
            PH_WAIT:   if (en && last)  phase_next = is_os ? PH_DRAIN : PH_DONE;
            PH_DRAIN:  if (en && last)  phase_next = PH_DONE;
            PH_DONE:                    phase_next = PH_IDLE;
            default:                    phase_next = PH_IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (reset) begin
            phase <= PH_IDLE;
            cnt   <= '0;
        end else begin
            phase <= phase_next;
            if (phase_next != phase)  cnt <= '0;
            else if (en && busy)      cnt <= cnt + 1'b1;   // quiet while idle
        end
    end

    // ------------------------------------------------------------ outputs
    assign busy      = (phase != PH_IDLE);
    assign done      = (phase == PH_DONE);

    assign stat_load = (phase == PH_SETUP) & ~is_os;
    assign acc_clear = (phase == PH_SETUP) &  is_os;
    assign drain     = (phase == PH_DRAIN);

    assign rd_en     = en & (phase == PH_STREAM);
    assign rd_addr   = cnt[OFF_W-1:0];

    // Valid bit for the bank outputs: follows the read by one enabled cycle
    always_ff @(posedge clk) begin
        if (reset)   rd_valid <= 1'b0;
        else if (en) rd_valid <= (phase == PH_STREAM);
    end

    // WS / IS results fall out while streaming and waiting; OS only during drain
    assign collect = is_os ? (phase == PH_DRAIN)
                           : (phase == PH_STREAM) | (phase == PH_WAIT);

endmodule
