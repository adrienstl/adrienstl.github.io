// ROWS x COLS systolic array of PEs.
//
// PE (r, c) sits in row r, column c; (0, 0) is the north-west corner.
//   h stream   : one per row, enters on the west edge, moves east
//   v stream   : one per column, enters on the north edge, moves south (OS only)
//   psum chain : one per column, starts at zero on the north edge, moves south,
//                leaves on the south edge into the output buffer
//   load_in    : one stationary operand per PE, all loaded on the same cycle
//   config / control : same wire to every PE
//
// The feeders skew the streams (row r / column c delayed r / c cycles); the
// array adds no skew of its own. Timing is in docs/pe_spec.md section 6.
//
// In WS / IS every column sum runs over all ROWS, so ROWS must not exceed
// K_MAX (the psum guard bits in pe_pkg are sized for it).
module systolic_array
    import pe_pkg::*;
#(
    parameter int ROWS = 4,
    parameter int COLS = 4
)(
    input  logic                                  clk,
    input  logic                                  reset,          // synchronous, active high

    // Configuration: set before a job, held while it runs
    input  dataflow_e                             cfg_dataflow,
    input  logic                                  cfg_dual,

    // Array-wide control, broadcast to every PE
    input  logic                                  en,             // 0 = stall, every register holds
    input  logic                                  stat_load,
    input  logic                                  acc_clear,
    input  logic                                  drain,

    // Stationary operands from the load buffer, [r][c] goes to PE (r, c)
    input  logic [ROWS-1:0][COLS-1:0][DATA_W-1:0] load_in,

    // West edge: horizontal stream into column 0 of each row
    input  logic [ROWS-1:0][DATA_W-1:0]           west_h,
    input  logic [ROWS-1:0]                       west_h_valid,

    // North edge: vertical stream into row 0 of each column
    input  logic [COLS-1:0][DATA_W-1:0]           north_v,
    input  logic [COLS-1:0]                       north_v_valid,

    // South edge: psum out of row ROWS-1 of each column, to the output buffer
    output logic [COLS-1:0][PSUM_W-1:0]           south_psum,
    output logic [COLS-1:0]                       south_psum_valid
);

    // Guard bits only cover K_MAX terms, so a taller array can overflow in dual mode.
    // Simulation-only check: Quartus 18.1 can't run $error at elaboration.
    // synthesis translate_off
    generate
        if (ROWS > K_MAX) begin : g_rows_check
            $error("systolic_array: ROWS = %0d exceeds K_MAX = %0d", ROWS, K_MAX);
        end
    endgenerate
    // synthesis translate_on

    // ------------------------------------------------------------ inter-PE wires
    // [r][c] is the wire going INTO PE (r, c): from the west for h, from the
    // north for v and psum. Each stream needs one more slot than there are PEs
    // along it, so the edge input and the edge output both have a place:
    //   h       [ROWS][COLS+1] : column 0 = west edge in, column COLS = east edge out
    //   v, psum [ROWS+1][COLS] : row 0 = north edge in,   row ROWS = south edge out
    wire [DATA_W-1:0] h_data     [ROWS][COLS+1];
    wire              h_valid    [ROWS][COLS+1];
    wire [DATA_W-1:0] v_data     [ROWS+1][COLS];
    wire              v_valid    [ROWS+1][COLS];
    wire [PSUM_W-1:0] psum_data  [ROWS+1][COLS];
    wire              psum_valid [ROWS+1][COLS];

    // Genvars declared up front and generate / endgenerate spelled out:
    // Quartus 18.1 rejects the shorter SystemVerilog forms
    genvar r, c;

    generate
        // -------------------------------------------------------- edges
        for (r = 0; r < ROWS; r++) begin : g_west
            assign h_data[r][0]  = west_h[r];
            assign h_valid[r][0] = west_h_valid[r];
        end

        for (c = 0; c < COLS; c++) begin : g_north
            assign v_data[0][c]     = north_v[c];
            assign v_valid[0][c]    = north_v_valid[c];
            assign psum_data[0][c]  = '0;       // every column sum starts from zero
            assign psum_valid[0][c] = 1'b0;
        end

        for (c = 0; c < COLS; c++) begin : g_south
            assign south_psum[c]       = psum_data[ROWS][c];
            assign south_psum_valid[c] = psum_valid[ROWS][c];
        end

        // h_data[r][COLS] (east edge) and v_data[ROWS][c] (south edge) are driven
        // but not read: nothing sits past the last column / row to take them.

        // -------------------------------------------------------- PE grid
        for (r = 0; r < ROWS; r++) begin : g_row
            for (c = 0; c < COLS; c++) begin : g_col
                pe u_pe (
                    .clk            (clk),
                    .reset          (reset),
                    .cfg_dataflow   (cfg_dataflow),
                    .cfg_dual       (cfg_dual),
                    .en             (en),
                    .stat_load      (stat_load),
                    .acc_clear      (acc_clear),
                    .drain          (drain),
                    .load_in        (load_in[r][c]),
                    .h_in           (h_data[r][c]),
                    .h_valid_in     (h_valid[r][c]),
                    .h_out          (h_data[r][c+1]),
                    .h_valid_out    (h_valid[r][c+1]),
                    .v_in           (v_data[r][c]),
                    .v_valid_in     (v_valid[r][c]),
                    .v_out          (v_data[r+1][c]),
                    .v_valid_out    (v_valid[r+1][c]),
                    .psum_in        (psum_data[r][c]),
                    .psum_valid_in  (psum_valid[r][c]),
                    .psum_out       (psum_data[r+1][c]),
                    .psum_valid_out (psum_valid[r+1][c])
                );
            end
        end
    endgenerate

endmodule
