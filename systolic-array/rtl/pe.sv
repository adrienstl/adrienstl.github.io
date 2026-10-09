// Processing element for the reconfigurable systolic array.
//
// Dataflow (cfg_dataflow):
//   WS / IS : stat_reg holds the stationary operand. The horizontal stream is
//             multiplied by it, added to psum_in, and the sum moves down.
//             WS and IS are the same hardware inside the PE - only what the
//             buffers feed in (and how the output is read back) differs.
//   OS      : both operands stream (h west->east, v north->south) and sum_reg
//             accumulates in place. Results leave down the psum chain on drain.
//
// Precision (cfg_dual):
//   0 : one int16 per word, psum is one 36-bit value
//   1 : two int8 lanes per word ([15:8] lane 1, [7:0] lane 0),
//       psum is two 18-bit lanes ([35:18] lane 1, [17:0] lane 0)
//
// en = 0 freezes every register (array-wide stall). Each stream carries a
// valid bit, and data registers only load when their valid bit is set.
module pe
    import pe_pkg::*;
(
    input  logic              clk,
    input  logic              reset,          // synchronous, active high

    // Configuration: set before a job, held while it runs
    input  dataflow_e         cfg_dataflow,
    input  logic              cfg_dual,

    // Array-wide control, same wire to every PE
    input  logic              en,             // 0 = stall, every register holds
    input  logic              stat_load,      // copy load_in into stat_reg (WS/IS)
    input  logic              acc_clear,      // zero sum_reg and its valid flag
    input  logic              drain,          // OS: shift sum_reg down the psum chain

    // Stationary operand from the load buffer (all PEs load on the same cycle)
    input  logic [DATA_W-1:0] load_in,

    // Horizontal stream, west -> east (a in WS/OS, b in IS)
    input  logic [DATA_W-1:0] h_in,
    input  logic              h_valid_in,
    output logic [DATA_W-1:0] h_out,
    output logic              h_valid_out,

    // Vertical operand stream, north -> south (b in OS, idle otherwise)
    input  logic [DATA_W-1:0] v_in,
    input  logic              v_valid_in,
    output logic [DATA_W-1:0] v_out,
    output logic              v_valid_out,

    // Partial sum / drain chain, north -> south
    input  logic [PSUM_W-1:0] psum_in,
    input  logic              psum_valid_in,
    output logic [PSUM_W-1:0] psum_out,
    output logic              psum_valid_out
);

    localparam int PROD_W      = 2*DATA_W;    // single-mode product (32)
    localparam int LANE_PROD_W = 2*LANE_W;    // dual-mode lane product (16)

    // ------------------------------------------------------------ state
    logic [DATA_W-1:0] h_reg, v_reg, stat_reg;
    logic              h_valid_reg, v_valid_reg;
    logic [PSUM_W-1:0] sum_reg;
    logic              sum_valid_reg;

    // ------------------------------------------------------------ mode decode + enables
    // The PE only needs to know whether an operand is stationary or not
    logic is_os;
    assign is_os = (cfg_dataflow == DF_OS);

    logic h_en, v_en, stat_en, mac_valid, sum_en;

    assign h_en      = en & h_valid_in;
    assign v_en      = en & is_os & v_valid_in;
    assign stat_en   = en & stat_load & ~is_os;
    assign mac_valid = h_valid_reg & (is_os ? v_valid_reg : 1'b1);
    assign sum_en    = en & (acc_clear | (is_os & drain) | mac_valid);

    // ------------------------------------------------------------ operand registers
    // Horizontal pass register: data only moves when it is real, valid always moves
    always_ff @(posedge clk) begin
        if (reset)     h_reg <= '0;
        else if (h_en) h_reg <= h_in;
    end

    always_ff @(posedge clk) begin
        if (reset)   h_valid_reg <= 1'b0;
        else if (en) h_valid_reg <= h_valid_in;
    end

    // Vertical pass register: only used in OS, held quiet otherwise
    always_ff @(posedge clk) begin
        if (reset)     v_reg <= '0;
        else if (v_en) v_reg <= v_in;
    end

    always_ff @(posedge clk) begin
        if (reset)   v_valid_reg <= 1'b0;
        else if (en) v_valid_reg <= is_os & v_valid_in;
    end

    // Stationary register: loaded from the load buffer in one cycle, unused in OS
    always_ff @(posedge clk) begin
        if (reset)        stat_reg <= '0;
        else if (stat_en) stat_reg <= load_in;
    end

    // ------------------------------------------------------------ multiply
    // MUX_Y: second operand is the stationary value, or the v stream in OS
    logic [DATA_W-1:0] mul_y;
    assign mul_y = is_os ? v_reg : stat_reg;

    logic signed [LANE_PROD_W-1:0] prod_lane0, prod_lane1;
    logic signed [PROD_W-1:0]      prod_single;

    mac_multiplyer u_mul (
        .mode     (cfg_dual),
        .x        (h_reg),
        .y        (mul_y),
        .Z_lane0  (prod_lane0),
        .Z_lane1  (prod_lane1),
        .Z_single (prod_single)
    );

    // MUX_P: sign-extend the product into the psum layout
    logic [PSUM_W-1:0] product;
    assign product = cfg_dual
        ? { {(LANE_ACC_W-LANE_PROD_W){prod_lane1[LANE_PROD_W-1]}}, prod_lane1,
            {(LANE_ACC_W-LANE_PROD_W){prod_lane0[LANE_PROD_W-1]}}, prod_lane0 }
        : { {(PSUM_W-PROD_W){prod_single[PROD_W-1]}}, prod_single };

    // ------------------------------------------------------------ accumulate
    // MUX_ADD: add to the psum from above (WS/IS) or to our own sum (OS)
    logic [PSUM_W-1:0] addend, adder_out;
    assign addend = is_os ? sum_reg : psum_in;

    mac_split_adder #(.LANE_ACC_W(LANE_ACC_W)) u_add (
        .dual (cfg_dual),
        .x    (product),
        .y    (addend),
        .sum  (adder_out)
    );

    // MUX_SUM: what sum_reg loads. Priority clear > drain > MAC, hold otherwise.
    typedef enum logic [1:0] {
        SUM_MAC   = 2'd0,   // adder_out
        SUM_SHIFT = 2'd1,   // psum_in, shifts results one row down (OS drain)
        SUM_ZERO  = 2'd2    // acc_clear
    } sum_sel_e;

    sum_sel_e          sum_sel;
    logic [PSUM_W-1:0] sum_d;
    logic              sum_valid_d;

    always_comb begin
        if (acc_clear)           sum_sel = SUM_ZERO;
        else if (is_os && drain) sum_sel = SUM_SHIFT;
        else                     sum_sel = SUM_MAC;
    end

    always_comb begin
        case (sum_sel)
            SUM_ZERO:  begin sum_d = '0;        sum_valid_d = 1'b0;          end
            SUM_SHIFT: begin sum_d = psum_in;   sum_valid_d = psum_valid_in; end
            default:   begin sum_d = adder_out; sum_valid_d = 1'b1;          end
        endcase
    end

    always_ff @(posedge clk) begin
        if (reset) begin
            sum_reg       <= '0;
            sum_valid_reg <= 1'b0;
        end else if (sum_en) begin
            sum_reg       <= sum_d;
            sum_valid_reg <= sum_valid_d;
        end else if (en && !is_os) begin
            // WS/IS bubble: nothing new arrived, so mark psum_out stale
            sum_valid_reg <= 1'b0;
        end
    end

    // ------------------------------------------------------------ outputs
    assign h_out          = h_reg;
    assign h_valid_out    = h_valid_reg;
    assign v_out          = v_reg;
    assign v_valid_out    = v_valid_reg;
    assign psum_out       = sum_reg;
    assign psum_valid_out = sum_valid_reg;

endmodule
