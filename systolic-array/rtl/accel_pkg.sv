// Accelerator-level constants: buffer geometry, MMIO address map, register
// fields and the controller phases.
package accel_pkg;

    // ------------------------------------------------------------ buffers
    // Each operand / output bank is BUF_DEPTH deep (one 512 x 16 M10K for operands)
    localparam int BUF_DEPTH  = 512;
    localparam int OFF_W      = $clog2(BUF_DEPTH);   // 9, offset inside a bank
    localparam int BANK_W     = 3;                   // bank field: up to 8 banks
    localparam int LEN_W      = OFF_W + 1;           // job length, 1 .. BUF_DEPTH
    localparam int LOAD_IDX_W = 6;                   // load buffer index r*N + c: up to 8 x 8
    localparam int OUT_W      = 64;                  // one result, unpacked to two 32-bit words

    // ------------------------------------------------------------ MMIO address map
    // Byte addresses inside the accelerator window, 32-bit word accesses only.
    //   0x00000  CSRs                    (0x000 - 0x0FF)
    //   0x00100  load buffer             0x100 + 4*(r*N + c)
    //   0x08000  operand buffer A        0x08000 + 0x800*bank + 4*offset
    //   0x10000  operand buffer B        0x10000 + 0x800*bank + 4*offset
    //   0x18000  output buffer           0x18000 + 0x1000*bank + 8*offset + 4*word
    localparam int MMIO_AW = 17;                     // 128 KB window

    // addr[16:15] picks the region
    typedef enum logic [1:0] {
        RGN_CTRL = 2'd0,   // CSRs when addr[8] = 0, load buffer when addr[8] = 1
        RGN_A    = 2'd1,
        RGN_B    = 2'd2,
        RGN_OUT  = 2'd3
    } region_e;

    localparam int RGN_LSB      = 15;
    localparam int LOAD_SEL_BIT = 8;
    localparam int IDX_LSB      = 2;                           // CSR / load index = addr[7:2]

    // Operand buffers: address = bank | offset
    localparam int BUF_OFF_LSB  = 2;
    localparam int BUF_BANK_LSB = BUF_OFF_LSB + OFF_W;         // 11

    // Output buffer: address = bank | offset | word (word 0 = low half)
    localparam int OUT_WORD_BIT = 2;
    localparam int OUT_OFF_LSB  = 3;
    localparam int OUT_BANK_LSB = OUT_OFF_LSB + OFF_W;         // 12

    // ------------------------------------------------------------ CSRs (word index)
    localparam logic [5:0] CSR_CTRL   = 6'd0;   // W : [0] start, [1] clear done / error
    localparam logic [5:0] CSR_STATUS = 6'd1;   // R : [0] busy, [1] done, [2] error, [6:4] phase
    localparam logic [5:0] CSR_CONFIG = 6'd2;   // RW: [1:0] dataflow, [2] dual
    localparam logic [5:0] CSR_LEN    = 6'd3;   // RW: stream length (M in WS / IS, K in OS)
    localparam logic [5:0] CSR_CYCLES = 6'd4;   // R : clock cycles the last job took
    localparam logic [5:0] CSR_INFO   = 6'd5;   // R : [7:0] N, [23:8] BUF_DEPTH, [31:24] K_MAX

    localparam int CTRL_START   = 0;
    localparam int CTRL_CLEAR   = 1;
    localparam int ST_BUSY      = 0;
    localparam int ST_DONE      = 1;
    localparam int ST_ERROR     = 2;
    localparam int ST_PHASE_LSB = 4;

    // ------------------------------------------------------------ controller
    typedef enum logic [2:0] {
        PH_IDLE   = 3'd0,   // wait for start
        PH_SETUP  = 3'd1,   // WS / IS: load stationary values, OS: clear accumulators
        PH_STREAM = 3'd2,   // read the operand banks, one offset per cycle
        PH_WAIT   = 3'd3,   // let the last wavefront finish
        PH_DRAIN  = 3'd4,   // OS: shift results out
        PH_DONE   = 3'd5    // one cycle: raise done for the CPU
    } phase_e;

endpackage
