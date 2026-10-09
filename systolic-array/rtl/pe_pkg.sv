// Shared widths and types for the systolic array PE.
package pe_pkg;

    // Every operand bus is one int16, or two packed int8 lanes ([15:8] lane 1, [7:0] lane 0)
    localparam int DATA_W = 16;
    localparam int LANE_W = DATA_W / 2;                // 8

    // Longest sum a PE must absorb without overflow (K = 4 for a 4x4 array)
    localparam int K_MAX   = 4;
    localparam int GUARD_W = $clog2(K_MAX);            // 2

    // Dual mode: each lane is a 16-bit product + guard bits
    localparam int LANE_ACC_W = 2*LANE_W + GUARD_W;    // 18
    // Psum bus holds both lanes side by side ([35:18] lane 1, [17:0] lane 0).
    // Single mode uses the whole bus: 32-bit product + 4 guard bits.
    localparam int PSUM_W = 2*LANE_ACC_W;              // 36

    typedef enum logic [1:0] {
        DF_WS = 2'b00,   // weight stationary
        DF_IS = 2'b01,   // input stationary
        DF_OS = 2'b10    // output stationary
    } dataflow_e;

endpackage
