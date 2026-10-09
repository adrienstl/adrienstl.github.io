# PE specification

One processing element (PE) of the reconfigurable systolic array. It supports three
dataflows (weight stationary, input stationary, output stationary) and two precisions
(one int16 per word, or two packed int8 lanes per word).

Source: `pe.sv`, `pe_pkg.sv`, `mac_multiplyer.sv`, `mac_split_adder.sv`. The array that
wires PEs together is `systolic_array.sv` (`ROWS` x `COLS`, must have `ROWS <= K_MAX`).
Test: `tb/tb_pe_array.sv` drives `systolic_array` in all modes, with and without stalls, at
4x4 and 3x5. Run with `cd sim && vsim -c -do run_pe.do`.
The accelerator built around the array (buffers, skews, controller, MMIO) is described in
`docs/accel_overview.html`; its system test is `cd sim && vsim -c -do run_accel.do`.

---

## 1. Parameters (`pe_pkg.sv`)

| Name         | Value | Meaning                                              |
|--------------|-------|------------------------------------------------------|
| `DATA_W`     | 16    | Width of every operand bus                           |
| `LANE_W`     | 8     | One int8 lane in dual mode                           |
| `K_MAX`      | 4     | Longest sum a PE must hold without overflow          |
| `GUARD_W`    | 2     | `$clog2(K_MAX)` guard bits                           |
| `LANE_ACC_W` | 18    | Dual-mode lane sum: 16-bit product + 2 guard bits    |
| `PSUM_W`     | 36    | Psum bus: two 18-bit lanes, or one 36-bit value      |

Bit layout:

| Bus        | Single (`cfg_dual = 0`)            | Dual (`cfg_dual = 1`)                          |
|------------|------------------------------------|------------------------------------------------|
| 16-bit word| `[15:0]` one int16                 | `[15:8]` lane 1 int8, `[7:0]` lane 0 int8      |
| 36-bit psum| `[35:0]` one value (32 + 4 guard)  | `[35:18]` lane 1, `[17:0]` lane 0 (16 + 2 each)|

---

## 2. Ports

| Port             | Dir | Width | Side / source        | Meaning |
|------------------|-----|-------|----------------------|---------|
| `clk`            | in  | 1     | global               | Clock |
| `reset`          | in  | 1     | global               | Synchronous, active high. Clears every register. |
| `cfg_dataflow`   | in  | 2     | global (config)      | `00` WS, `01` IS, `10` OS. Held for a whole job. |
| `cfg_dual`       | in  | 1     | global (config)      | 0 = single int16, 1 = dual int8. Held for a whole job. |
| `en`             | in  | 1     | global (control)     | 0 = stall. Every register in the PE holds. |
| `stat_load`      | in  | 1     | global (control)     | Load `load_in` into `stat_reg` (every PE on the same cycle). |
| `acc_clear`      | in  | 1     | global (control)     | Zero `sum_reg` and its valid flag. |
| `drain`          | in  | 1     | global (control)     | OS only: shift `sum_reg` down the psum chain. |
| `load_in`        | in  | 16    | west, from load buffer | Stationary operand for this PE. |
| `h_in`           | in  | 16    | west, from PE (r, c-1) | Horizontal stream: `a` in WS/OS, `b` in IS. |
| `h_valid_in`     | in  | 1     | west                 | `h_in` holds real data. |
| `h_out`          | out | 16    | east, to PE (r, c+1)   | = `h_reg` |
| `h_valid_out`    | out | 1     | east                 | = `h_valid_reg` |
| `v_in`           | in  | 16    | north, from PE (r-1, c)| Vertical operand stream: `b` in OS, unused otherwise. |
| `v_valid_in`     | in  | 1     | north                | `v_in` holds real data. |
| `v_out`          | out | 16    | south, to PE (r+1, c)  | = `v_reg` |
| `v_valid_out`    | out | 1     | south                | = `v_valid_reg` |
| `psum_in`        | in  | 36    | north, from PE (r-1, c)| WS/IS: partial sum from above. OS: result from above during drain. |
| `psum_valid_in`  | in  | 1     | north                | Only read during OS drain. |
| `psum_out`       | out | 36    | south, to PE (r+1, c)  | = `sum_reg` |
| `psum_valid_out` | out | 1     | south                | = `sum_valid_reg` |

Array edge tie-offs: top row `psum_in = 0`, `psum_valid_in = 0`. Top row `v_in` and left
column `h_in` come from the stream feeders (skewed). The bottom row `psum_out` goes to
the output buffer.

---

## 3. Blocks

### Registers (all `always_ff @(posedge clk)`, all cleared by `reset`)

| Register        | Width | D input                         | Enable                       | Drives |
|-----------------|-------|---------------------------------|------------------------------|--------|
| `h_reg`         | 16    | `h_in`                          | `h_en`                       | `h_out`, multiplier `x` |
| `h_valid_reg`   | 1     | `h_valid_in`                    | `en`                         | `h_valid_out`, `mac_valid` |
| `v_reg`         | 16    | `v_in`                          | `v_en`                       | `v_out`, MUX_Y in 1 |
| `v_valid_reg`   | 1     | `is_os & v_valid_in`            | `en`                         | `v_valid_out`, `mac_valid` |
| `stat_reg`      | 16    | `load_in`                       | `stat_en`                    | MUX_Y in 0 |
| `sum_reg`       | 36    | MUX_SUM data out                | `sum_en`                     | `psum_out`, MUX_ADD in 1 |
| `sum_valid_reg` | 1     | MUX_SUM valid out               | `sum_en`; also cleared when `en & !is_os & !sum_en` (a WS/IS bubble) | `psum_valid_out` |

### Muxes

| Mux       | Inputs | Width | Select | Output goes to |
|-----------|--------|-------|--------|----------------|
| `MUX_Y`   | 0: `stat_reg`, 1: `v_reg` | 16 | `is_os` | multiplier `y` |
| `MUX_P`   | 0: `sext36(Z_single)`, 1: `{sext18(Z_lane1), sext18(Z_lane0)}` | 36 | `cfg_dual` | adder `x` (`product`) |
| `MUX_ADD` | 0: `psum_in`, 1: `sum_reg` | 36 | `is_os` | adder `y` (`addend`) |
| `MUX_SUM` | `SUM_MAC`: `adder_out` (valid 1), `SUM_SHIFT`: `psum_in` (valid `psum_valid_in`), `SUM_ZERO`: 0 (valid 0) | 36 + 1 | `sum_sel` | `sum_reg`, `sum_valid_reg` |

### Arithmetic

| Block             | Inputs | Outputs | Behaviour |
|-------------------|--------|---------|-----------|
| `mac_multiplyer`  | `x = h_reg`, `y = MUX_Y`, `mode = cfg_dual` | `Z_single[31:0]`, `Z_lane1[15:0]`, `Z_lane0[15:0]` | Single: `Z_single = x*y` (signed 16x16). Dual: `Z_lane1 = x[15:8]*y[15:8]`, `Z_lane0 = x[7:0]*y[7:0]` (signed 8x8). Unused outputs are 0. |
| `mac_split_adder` | `x = product`, `y = addend`, `dual = cfg_dual` | `adder_out[35:0]` | Single: one 36-bit add. Dual: the carry from bit 17 into bit 18 is cut, so it's two independent 18-bit adds. |

### Control and enable logic (combinational)

```
is_os     = (cfg_dataflow == DF_OS)
h_en      = en & h_valid_in
v_en      = en & is_os & v_valid_in
stat_en   = en & stat_load & !is_os
mac_valid = h_valid_reg & (is_os ? v_valid_reg : 1)
sum_en    = en & (acc_clear | (is_os & drain) | mac_valid)
sum_sel   = acc_clear       ? SUM_ZERO
          : (is_os & drain) ? SUM_SHIFT
          :                   SUM_MAC
```

Priority for `sum_reg`: clear > drain > MAC > hold.

---

## 4. Connection list (netlist)

Each line is one wire in the diagram: `source -> destination [width]`.

```
h_in                  -> h_reg.D                    [16]
h_valid_in            -> h_valid_reg.D              [1]
h_valid_in            -> control (h_en)             [1]
h_reg.Q               -> h_out                      [16]
h_reg.Q               -> mac_multiplyer.x           [16]
h_valid_reg.Q         -> h_valid_out                [1]
h_valid_reg.Q         -> control (mac_valid)        [1]

v_in                  -> v_reg.D                    [16]
v_valid_in            -> v_valid_reg.D (AND is_os)  [1]
v_valid_in            -> control (v_en)             [1]
v_reg.Q               -> v_out                      [16]
v_reg.Q               -> MUX_Y.in1                  [16]
v_valid_reg.Q         -> v_valid_out                [1]
v_valid_reg.Q         -> control (mac_valid)        [1]

load_in               -> stat_reg.D                 [16]
stat_reg.Q            -> MUX_Y.in0                  [16]

MUX_Y.out             -> mac_multiplyer.y           [16]
mac_multiplyer.Z_single            -> MUX_P.in0 (sign-extend 32->36)          [32]
mac_multiplyer.{Z_lane1, Z_lane0}  -> MUX_P.in1 (sign-extend each 16->18)     [2x16]
MUX_P.out (product)   -> mac_split_adder.x          [36]

psum_in               -> MUX_ADD.in0                [36]
sum_reg.Q             -> MUX_ADD.in1   (feedback)   [36]
MUX_ADD.out (addend)  -> mac_split_adder.y          [36]

mac_split_adder.sum   -> MUX_SUM.MAC                [36]
psum_in               -> MUX_SUM.SHIFT (drain)      [36]
constant 0            -> MUX_SUM.ZERO               [36]
psum_valid_in         -> MUX_SUM.SHIFT valid        [1]
MUX_SUM.out           -> sum_reg.D                  [36]
MUX_SUM.valid         -> sum_valid_reg.D            [1]
sum_reg.Q             -> psum_out                   [36]
sum_valid_reg.Q       -> psum_valid_out             [1]

Select / enable wires:
is_os     -> MUX_Y.sel, MUX_ADD.sel
cfg_dual  -> MUX_P.sel, mac_multiplyer.mode, mac_split_adder.dual
sum_sel   -> MUX_SUM.sel
h_en      -> h_reg.EN
v_en      -> v_reg.EN
stat_en   -> stat_reg.EN
sum_en    -> sum_reg.EN, sum_valid_reg.EN
en        -> h_valid_reg.EN, v_valid_reg.EN
```

---

## 5. What each mode uses

| Item                    | WS                    | IS                    | OS                          |
|-------------------------|-----------------------|-----------------------|-----------------------------|
| `h` stream carries      | `a` (activation)      | `b` (weight)          | `a` (activation)            |
| `stat_reg` holds        | `b` (weight)          | `a` (input)           | unused                      |
| `v` stream carries      | unused                | unused                | `b` (weight)                |
| `is_os`                 | 0                     | 0                     | 1                           |
| MUX_Y selects           | `stat_reg`            | `stat_reg`            | `v_reg`                     |
| MUX_ADD selects         | `psum_in`             | `psum_in`             | `sum_reg` (own sum)         |
| MUX_SUM uses            | MAC, ZERO             | MAC, ZERO             | MAC, SHIFT (drain), ZERO    |
| `sum_reg` role          | psum passed down each cycle | same as WS      | `c` accumulates in place    |
| `psum_out` carries      | partial sum + a*b     | partial sum + b*a     | `c`, only during drain      |

WS and IS are the same hardware configuration inside the PE. The difference is only in
what the buffers load and stream, and how the output is read back (IS gives C transposed
relative to WS).

---

## 6. How the controller drives it (N x N array)

Cycle numbers count enabled clock edges (`en = 1`); any cycle with `en = 0` is simply
inserted and changes nothing.

**WS / IS**
1. Fill the load buffer, then pulse `stat_load` for one cycle. Every PE loads at once.
2. Stream row `i` of the array with a delay of `i` cycles, with `h_valid_in` set on real data.
3. Item `m` of column `j` appears on the bottom row's `psum_out` after edge `m + N + j`
   (edge 0 = item 0 entering row 0). Capture whenever `psum_valid_out = 1`.
4. Before the next `stat_load`, wait for the last wavefront to leave. PE (N-1, N-1)
   uses its stationary value for the last time `2N - 1` cycles after the last item
   enters row 0. The swap is global, so it can't overlap with a tile still in flight.

**OS**
1. Pulse `acc_clear` for one cycle.
2. Stream row `i` (`h`) delayed `i` cycles and column `j` (`v`) delayed `j` cycles, valids set.
3. PE (N-1, N-1) finishes its last MAC `K + 2N - 2` cycles after the first input.
4. Hold `drain = 1` for `N` cycles. Column `j` emits `C[N-1][j], C[N-2][j], ..., C[0][j]`.
   Capture when `drain & psum_valid_out`. Draining also shifts in zeros from the top row.

---

## 7. Changes from the hand-drawn sketch

1. **Top input width.** The sketch has one 16-bit input from above, but in WS/IS the thing
   coming from above is a partial sum. The top side now mirrors the bottom:
   `psum_in` (36) + `v_in` (16).
2. **Psum is 36 bits, not 34.** Dual mode needs two 18-bit lane sums side by side (36).
   Single mode uses the same 36 bits (32 + 4 guard bits, so K up to 16 is safe there).
3. **One accumulator instead of two.** `MAC_Accumulator_0/1` had the feedback wired inside,
   so there was no way to add `psum_in` for WS/IS. They're replaced by one split adder,
   an addend mux (MUX_ADD), and a single 36-bit `sum_reg` (36 flops instead of 34 + 18).
   The old files are untouched but no longer used.
4. **Output reg = sum reg.** The sketch's "accumulator + output reg" is one register here.
   `psum_out` is already registered, so a second register would only add a cycle.
5. **OS drain path.** The OS reference shows "c, drained after" with no path. The drain
   reuses the psum chain: during `drain`, `sum_reg` loads `psum_in`, so results shift
   down one row per cycle and fall out the bottom.
6. **"Input logic" box.** In the RTL this is the two pass registers plus MUX_Y. It picks the
   multiplier's second operand (stationary value or the `v` stream).
7. **Valid bits and `en` everywhere.** Each stream carries a valid bit. Data registers only
   load on valid data, so they don't toggle when idle, and the MAC only fires when its
   operands are valid. `en` stalls the whole PE.

## 8. Possible later upgrades (not built)

- Double-buffer `sum_reg` in OS so draining one tile overlaps computing the next.
- Propagate `stat_load` diagonally with the data so WS/IS tiles can run back to back
  without the ~2N-cycle gap.
- A product pipeline register if the multiply + add path doesn't meet timing.
- Share one set of 9x9 multipliers between single and dual mode. The current multiplier
  builds a 16x16 and two 8x8 multipliers separately.
