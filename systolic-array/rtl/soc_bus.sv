// Data bus decoder: sends each core access to one slave by address[31:28],
// and adapts the accelerator's MMIO port to the core's request / ready bus.
//
//   0x1000_0000  data RAM            any size, done the same cycle
//   0x2000_0000  accelerator MMIO    32-bit only; stores done the same cycle,
//                                    loads one cycle later (its read data is
//                                    registered, the output buffer is block RAM)
//   0x3000_0000  system registers    32-bit only, done the same cycle
//   anything else, including instruction memory (it sits on its own port),
//   past the end of a slave, or a byte / half access where only words are
//   allowed: dbus_err, and the core stops with a fault
//
// The accelerator takes one-cycle mmio_re / mmio_we strobes. The core holds a
// load's request until ready, so the read strobe is sent only in the first
// cycle and the second cycle just waits for mmio_rvalid.
module soc_bus
    import accel_pkg::*;
#(
    parameter int DMEM_WORDS = 2048
)(
    input  logic        clk,
    input  logic        reset,

    // Core side
    input  logic        req,
    input  logic        we,
    input  logic [31:0] addr,
    input  logic [3:0]  be,
    output logic [31:0] rdata,
    output logic        ready,
    output logic        err,

    // Data RAM
    output logic        ram_we,
    input  logic [31:0] ram_rdata,

    // Accelerator
    output logic        mmio_we,
    output logic        mmio_re,
    input  logic [31:0] mmio_rdata,
    input  logic        mmio_rvalid,

    // System registers
    output logic        sys_we,
    input  logic [31:0] sys_rdata
);

    localparam logic [3:0] RGN_RAM = 4'h1, RGN_ACCEL = 4'h2, RGN_SYS = 4'h3;
    localparam int         RAM_AW  = $clog2(DMEM_WORDS) + 2;     // byte address bits in the RAM

    // ------------------------------------------------------------ decode
    logic [3:0] region;
    logic       word, hit_ram, hit_acc, hit_sys;

    assign region  = addr[31:28];
    assign word    = (be == 4'b1111);

    assign hit_ram = (region == RGN_RAM)   && (addr[27:RAM_AW]  == '0);
    assign hit_acc = (region == RGN_ACCEL) && (addr[27:MMIO_AW] == '0) && word;
    assign hit_sys = (region == RGN_SYS)   && (addr[27:4]       == '0) && word;

    assign err     = req & ~(hit_ram | hit_acc | hit_sys);

    // ------------------------------------------------------------ strobes
    // rd_wait: the read strobe went out last cycle, its data arrives this cycle
    logic rd_wait;

    assign ram_we  = req &  we & hit_ram;
    assign sys_we  = req &  we & hit_sys;
    assign mmio_we = req &  we & hit_acc;
    assign mmio_re = req & ~we & hit_acc & ~rd_wait;

    always_ff @(posedge clk) begin
        if (reset) rd_wait <= 1'b0;
        else       rd_wait <= mmio_re;
    end

    // ------------------------------------------------------------ response
    always_comb begin
        ready = 1'b1;
        if (hit_acc) begin
            rdata = mmio_rdata;
            ready = we | mmio_rvalid;
        end else if (hit_sys) begin
            rdata = sys_rdata;
        end else begin
            rdata = ram_rdata;
        end
    end

endmodule
