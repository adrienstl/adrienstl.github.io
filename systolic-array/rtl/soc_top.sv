// SoC: the single-cycle RV32I core as the systolic accelerator's host.
//
//   rv32i_core --imem port--> imem       0x0000_0000  program (ROM, 4 KB)
//              --dbus-------> soc_bus -+-> dmem       0x1000_0000  data RAM (8 KB)
//                                      +-> accel_top  0x2000_0000  accelerator MMIO (128 KB window)
//                                      +-> sys_regs   0x3000_0000  TOHOST, CYCLE, LED, MARK
//
// The accelerator has no port of its own for configuration: software sets the
// dataflow, precision and stream length with sw into its CSRs, fills its
// buffers with sw, polls STATUS with lw and reads results with lw. The map
// inside the window is in accel_pkg and docs/accel_overview.html.
module soc_top
    import accel_pkg::*;
#(
    parameter int N          = 4,           // accelerator array size
    parameter int IMEM_WORDS = 1024,
    parameter int DMEM_WORDS = 2048,
    parameter     IMEM_INIT  = "",          // program image ($readmemh)
    parameter     DMEM_INIT  = ""           // .data image ($readmemh)
)(
    input  logic        clk,
    input  logic        reset,              // synchronous, active high

    output logic        halted,             // core stopped (EBREAK / ECALL or fault)
    output logic        fault,
    output logic [31:0] tohost,             // result code from software
    output logic [9:0]  led,
    output logic        accel_irq           // accelerator done (polled; no interrupt controller)
);

    // ------------------------------------------------------------ core
    logic [31:0] imem_addr, imem_rdata;
    logic        dbus_req, dbus_we, dbus_ready, dbus_err;
    logic [31:0] dbus_addr, dbus_wdata, dbus_rdata;
    logic [3:0]  dbus_be;

    rv32i_core u_core (
        .clk         (clk),
        .reset       (reset),
        .imem_addr   (imem_addr),
        .imem_rdata  (imem_rdata),
        .dbus_req    (dbus_req),
        .dbus_we     (dbus_we),
        .dbus_addr   (dbus_addr),
        .dbus_be     (dbus_be),
        .dbus_wdata  (dbus_wdata),
        .dbus_rdata  (dbus_rdata),
        .dbus_ready  (dbus_ready),
        .dbus_err    (dbus_err),
        .halted      (halted),
        .fault       (fault),
        .tr_valid    (),
        .tr_trap     (),
        .tr_pc       (),
        .tr_insn     (),
        .tr_next_pc  (),
        .tr_rd       (),
        .tr_rd_wdata ()
    );

    imem #(.WORDS(IMEM_WORDS), .INIT_FILE(IMEM_INIT)) u_imem (
        .clk   (clk),
        .addr  (imem_addr),
        .rdata (imem_rdata)
    );

    // ------------------------------------------------------------ data bus
    logic        ram_we, mmio_we, mmio_re, mmio_rvalid, sys_we;
    logic [31:0] ram_rdata, mmio_rdata, sys_rdata;

    soc_bus #(.DMEM_WORDS(DMEM_WORDS)) u_bus (
        .clk         (clk),
        .reset       (reset),
        .req         (dbus_req),
        .we          (dbus_we),
        .addr        (dbus_addr),
        .be          (dbus_be),
        .rdata       (dbus_rdata),
        .ready       (dbus_ready),
        .err         (dbus_err),
        .ram_we      (ram_we),
        .ram_rdata   (ram_rdata),
        .mmio_we     (mmio_we),
        .mmio_re     (mmio_re),
        .mmio_rdata  (mmio_rdata),
        .mmio_rvalid (mmio_rvalid),
        .sys_we      (sys_we),
        .sys_rdata   (sys_rdata)
    );

    dmem #(.WORDS(DMEM_WORDS), .INIT_FILE(DMEM_INIT)) u_dmem (
        .clk   (clk),
        .we    (ram_we),
        .be    (dbus_be),
        .addr  (dbus_addr),
        .wdata (dbus_wdata),
        .rdata (ram_rdata)
    );

    accel_top #(.N(N)) u_accel (
        .clk         (clk),
        .reset       (reset),
        .mmio_we     (mmio_we),
        .mmio_re     (mmio_re),
        .mmio_addr   (dbus_addr[MMIO_AW-1:0]),
        .mmio_wdata  (dbus_wdata),
        .mmio_rdata  (mmio_rdata),
        .mmio_rvalid (mmio_rvalid),
        .irq         (accel_irq),
        .stall       (1'b0)
    );

    sys_regs u_sys (
        .clk    (clk),
        .reset  (reset),
        .we     (sys_we),
        .addr   (dbus_addr[3:0]),
        .wdata  (dbus_wdata),
        .rdata  (sys_rdata),
        .tohost (tohost),
        .led    (led)
    );

endmodule
