// System registers: the few things firmware needs besides memory and the
// accelerator. 32-bit word accesses, reads are combinational.
//
//   0x00  TOHOST  RW  result code; the testbench stops on it (1 = pass)
//   0x04  CYCLE   R   clock cycles since reset. RV32I only has counters
//                     through Zicsr, so they are memory-mapped instead
//   0x08  LED     RW  drives led[9:0]
//   0x0C  MARK    RW  scratch; firmware writes it after each job so the
//                     testbench can check that job's results in memory
module sys_regs (
    input  logic        clk,
    input  logic        reset,

    input  logic        we,
    input  logic [3:0]  addr,             // byte offset in the block
    input  logic [31:0] wdata,
    output logic [31:0] rdata,

    output logic [31:0] tohost,
    output logic [9:0]  led
);

    localparam logic [1:0] R_TOHOST = 2'd0, R_CYCLE = 2'd1, R_LED = 2'd2, R_MARK = 2'd3;

    logic [31:0] cycle, mark;

    always_ff @(posedge clk) begin
        if (reset) begin
            tohost <= '0;
            led    <= '0;
            mark   <= '0;
            cycle  <= '0;
        end else begin
            cycle <= cycle + 1'b1;
            if (we) begin
                case (addr[3:2])
                    R_TOHOST: tohost <= wdata;
                    R_LED:    led    <= wdata[9:0];
                    R_MARK:   mark   <= wdata;
                    default:  ;                        // CYCLE is read-only
                endcase
            end
        end
    end

    always_comb begin
        case (addr[3:2])
            R_TOHOST: rdata = tohost;
            R_CYCLE:  rdata = cycle;
            R_LED:    rdata = {22'd0, led};
            default:  rdata = mark;
        endcase
    end

endmodule
