# Reconfigurable systolic array accelerator

A 4×4 systolic array for neural network matrix multiplication, written in SystemVerilog, with a single-cycle RV32I RISC-V core that drives it through memory-mapped registers. Software picks the dataflow (weight, input or output stationary) and the precision (one 16-bit or two 8-bit numbers per word) at run time.

- Project page: <https://adrienstl.github.io/systolic-array/>
- How it was verified: <https://adrienstl.github.io/systolic-array/verification/>

## Folders

| Folder | What's in it |
|--------|--------------|
| `rtl/` | The design: the accelerator, the RISC-V core and the system around them |
| `tb/` | The testbenches and reference models (run in ModelSim) |
| `docs/pe_spec.md` | Detailed spec of one processing element: ports, registers, multiplexers and timing |

## Design files (`rtl/`)

**The whole system**

| File | Role |
|------|------|
| `soc_top.sv` | Top level: the core, its two memories, the bus, the accelerator and the system registers |
| `soc_bus.sv` | Sends each load or store to the right place by address, and reports bad accesses |
| `imem.sv`, `dmem.sv` | Program memory and data memory, both in block RAM |
| `sys_regs.sv` | A result code, a cycle counter, the board's LEDs and a scratch register |

**The accelerator**

| File | Role |
|------|------|
| `accel_top.sv` | Top level of the accelerator: wires every block below together |
| `accel_pkg.sv` | Buffer sizes, the address map, register fields and controller phases |
| `mmio_regs.sv` | The registers the processor reads and writes, the start check and the write lock |
| `controller.sv` | State machine that takes each job through setup, stream, wait, drain and done |
| `load_buffer.sv` | The values each PE keeps in place during a job |
| `operand_buffer.sv` | Buffers A and B, one block RAM bank per row or column |
| `dataflow_router.sv` | Picks which buffer feeds which edge of the array |
| `skew.sv` | Delays row (or column) *i* by *i* cycles so values meet at the right time |
| `systolic_array.sv` | The grid of PEs and the wires between neighbours |
| `pe.sv`, `pe_pkg.sv` | One processing element and the shared widths and types |
| `mac_multiplyer.sv` | One 16-bit multiply, or two 8-bit multiplies |
| `mac_split_adder.sv` | One 36-bit add, or two 18-bit adds with the carry between them cut |
| `output_collector.sv` | Puts each column's results in order and widens them for the processor |
| `output_buffer.sv` | Holds the results until the processor reads them |

**The RISC-V core**

| File | Role |
|------|------|
| `rv32i_core.sv` | Top level of the single-cycle core |
| `rv32i_pkg.sv` | Opcodes and the control word |
| `rv32i_decoder.sv` | Turns each instruction into control signals, and flags anything that isn't RV32I |
| `rv32i_imm_gen.sv` | Rebuilds the constant packed into each instruction |
| `rv32i_regfile.sv` | The 32 registers |
| `rv32i_alu.sv` | The ten arithmetic and logic operations |
| `rv32i_branch.sv` | The comparisons for branches |
| `rv32i_lsu.sv` | Byte, halfword and word loads and stores |

## Testbenches (`tb/`)

| File | What it checks |
|------|----------------|
| `tb_ref_pkg.sv` | Reference arithmetic for the array and accelerator tests, written separately from the RTL |
| `tb_rv32i_iss_pkg.sv` | RV32I instruction-set model, used to check the core one instruction at a time |
| `tb_pe_array.sv` | The array of PEs in every dataflow and precision, with random pauses, at 4×4 and 3×5 |
| `tb_accel.sv` | The accelerator driven the way the processor would, compared with A × B |
| `tb_rv32i_core.sv` | The core: a self-checking instruction test, fault cases and random programs |
| `tb_soc.sv` | The whole system running the firmware, with every job's result checked |
