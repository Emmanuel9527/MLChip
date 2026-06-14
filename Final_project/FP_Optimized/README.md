# Final Project Optimized

This folder contains the optimized implementation for the final project.

## Build and Run

```sh
make cat
make dog
```

Trace logs are disabled by default for submission. To rebuild and run with
progress logs on the screen, use:

```sh
make trace-cat
make trace-dog
```

The equivalent manual form is:

```sh
make clean
make TRACE=1 cat
```

To test only the final output write-back path without running the full CNN,
use:

```sh
make trace-test-writeback
```

The program reads `IMAGE_FILE_NAME` from the Makefile target. The DRAM model
searches for the data folder in this order:

1. `./data/`
2. `../data/`
3. `../../data/`
4. `../../hw4/data/`
5. `../../Final_report/data/`

## Optimized Architecture

- 4x4 mesh NoC is inherited from HW4.
- Router 0 is connected to the controller.
- Routers 0 through 15 are connected to worker cores and PEs.
- HW4 PE compute behavior is preserved as the baseline.
- ROM-based tensor loading is replaced by a DRAM model and an AXI4-like DMA.
- All modules use an active-low `reset_n` signal.
- A controller-side Global SRAM scratchpad is added after DMA.
- Fully connected layers are executed by the original PE0..PE15 as a
  cycle-aware 4x4 systolic-style array.
- FC input activations, weight tiles, bias tiles, partial sums, and output
  tiles are explicitly staged in Global SRAM.
- The optimized controller reports DRAM traffic, SRAM traffic, NoC payload
  words, peak SRAM usage, and modeled SRAM cycles.

## Optimization Method

The optimized design keeps the baseline 4x4 NoC, router, core, and PE compute
modules for convolution and pooling. The main change is the FC dataflow and
memory hierarchy:

```text
DRAM -> AXI DMA -> Global SRAM scratchpad -> NoC HOST port -> PE0..PE15 systolic array -> Global SRAM output buffer
```

For each FC layer, the controller first stages the input activation vector in
SRAM. It then processes up to four output neurons at a time, mapped to the four
columns of the original 4x4 PE mesh. For each input tile, the controller splits
the input dimension into four row segments. The input-vector segment `b(j)`
flows horizontally from left to right across one PE row. Each PE keeps its
matrix coefficient segment `a(i,j)` in local SRAM. The partial sum `c(i)` moves
vertically from bottom to top through one PE column, and the top-boundary PE
returns the finished tile partial sum to the controller. The controller writes
partial sums and final output tiles back to SRAM for observability and traffic
accounting. This follows the lecture MVM mapping for a PE-level 2D systolic
dataflow.

The SRAM model is defined in `global_sram.h`:

- Capacity: 2,097,152 32-bit words, or 8 MB
- Data width: 32 bits
- Banks: 1
- Read latency: 1 modeled cycle per word
- Write latency: 1 modeled cycle per word

To keep SystemC simulation practical, the full SRAM cycle cost is accumulated
in metrics while the simulator advances one synchronization cycle per SRAM
block access.

The systolic FC behavior is implemented in the original PE model:

- Array size: PE0..PE15 arranged as a physical 4x4 mesh
- Four output-neuron columns are computed in parallel
- Input-vector segments flow horizontally across each active PE row
- Partial sums flow vertically from bottom to top through each active PE column
- Each PE contains a `PeLocalSram` SystemC module for input, weight, and bias tiles
- Each PE contains a `NonlinearFunction` SystemC module for ReLU and max selection
- Input activation segments are read from Global SRAM and loaded into PE local buffers
- Weight segments are staged through SRAM before being loaded into PE local buffers
- Pipeline fill/drain cycles are included in the modeled cycle count

## PE Internal Blocks

Each original PE is modeled with three hardware-oriented blocks:

- Local SRAM: `PeLocalSram` stores input activation tiles, weight tiles, and
  bias tiles. Each PE models a 64 KB local SRAM with 16 input banks and 16
  weight banks for the FC systolic tile path.
- MAC array: each PE models a 4x4 internal MAC array with 16 MAC units, so
  `MACS_PER_CYCLE = 16`.
- Nonlinear function: `NonlinearFunction` performs ReLU for convolution/FC and
  max selection for pooling.

The 4x4 systolic-style array is formed by the sixteen original PEs at the NoC
mesh level. The MAC array is the compute datapath inside each PE.

## FC DG and DFG Mapping

The optimized FC layer is treated as matrix-vector multiplication:

```text
y[o] = bias[o] + sum_i input[i] * weight[o][i]
```

### Dependence Graph Design

The FC dependence graph has one MAC node for each pair `(o, i)`, where `o` is
the output-neuron index and `i` is the input-vector index.

Each node computes:

```text
psum[o, i + 1] = psum[o, i] + input[i] * weight[o][i]
```

with:

```text
psum[o, 0] = bias[o]
y[o] = nonlinear(psum[o, input_size])
```

The main dependences are:

- Partial-sum dependence along the input dimension:
  `psum[o, i] -> psum[o, i + 1]`
- Input activation reuse across output rows:
  `input[i]` is consumed by multiple output neurons
- Weight dependence is local to each output row:
  `weight[o][i]` is consumed by the PE assigned to output `o`

This graph exposes column-level parallelism across output neurons. The input
dimension is spatially partitioned across the four PE rows, input-vector
segments flow left to right, and the partial-sum dependence is mapped onto the
bottom-to-top PE links.

### DFG Mapping

The physical PE mesh is a 4x4 array with PE ids:

```text
 0   1   2   3
 4   5   6   7
 8   9  10  11
12  13  14  15
```

For each FC output tile, up to four output neurons are mapped to the four PE
columns:

```text
col 0: PE12 -> PE8  -> PE4  -> PE0  computes output out_base + 0
col 1: PE13 -> PE9  -> PE5  -> PE1  computes output out_base + 1
col 2: PE14 -> PE10 -> PE6  -> PE2  computes output out_base + 2
col 3: PE15 -> PE11 -> PE7  -> PE3  computes output out_base + 3
```

The mapping function is:

```text
col = o - out_base, for 0 <= col < 4
data_row = input segment id, for 0 <= data_row < 4
physical_row = 3 - data_row
PE id = physical_row * 4 + col
```

The input dimension is tiled by `SYSTOLIC_INPUT_TILE_WORDS`. For each input
tile:

1. The input activation tile is read from Global SRAM and split into four row
   segments.
2. The west PE of each active row receives one input segment and forwards it
   horizontally to the active columns, modeling the lecture's `b` flow.
3. Each PE receives the matching weight segment for its output column.
4. The bottom PE of each active column receives the incoming partial sum.
5. Each PE performs local MAC accumulation and forwards the updated partial sum
   to the next PE on the north side.
6. The top-boundary PE returns the column result to the controller.
7. The current partial sums are written back to the Global SRAM partial-sum
   region for observability and cycle-aware SRAM traffic accounting.

The schedule is:

```text
for out_base in output neurons step 4:
    initialize column partial sums with bias[out_base : out_base + 3]
    for input_base in input vector step SYSTOLIC_INPUT_TILE_WORDS:
        split input tile into four row segments
        send each segment into the west PE of one row
        input segments flow left to right across active columns
        load matching weight segments into PE local SRAM
        send each column partial sum into the bottom PE
        partial sums flow bottom to top through the column
        write partial sums to SRAM
    apply nonlinear function at the top-boundary PE on the final tile
    write output tile to SRAM
```

The modeled systolic cycle count for one input tile is:

```text
ceil(tile_words / (4 * 16)) + 4 + 4 - 2
```

where the first term models four PE rows per output column, each with a 16-lane
MAC array, and `4 + 4 - 2` is the 4x4 PE-array fill/drain overhead. The
arithmetic is still behavioral, but the data movement, buffering, horizontal
input forwarding, vertical partial-sum forwarding, and cycle metrics follow the
lecture MVM systolic dataflow.

The local SRAM bandwidth assumption for the FC systolic path is:

```text
input side:  16 banks x 32-bit word/cycle = 16 input words/cycle
weight side: 16 banks x 32-bit word/cycle = 16 weight words/cycle
partial sum: kept in a PE-local accumulator register
```

This bandwidth is sufficient to feed the 16-MAC internal array in each PE.
The Global SRAM is still useful because it stages reusable FC activations,
weight segments, bias values, partial sums, and output tiles between the AXI DMA
and the NoC/PE array instead of forcing every PE load to come directly from
DRAM.

## AXI4-like DMA Model

The DMA supports:

- Read address channel: `ARADDR`, `ARLEN`, `ARSIZE`, `ARVALID`, `ARREADY`
- Read data channel: `RDATA`, `RVALID`, `RREADY`, `RLAST`
- Write address channel: `AWADDR`, `AWLEN`, `AWSIZE`, `AWVALID`, `AWREADY`
- Write data channel: `WDATA`, `WVALID`, `WREADY`, `WLAST`
- Write response channel: `BVALID`, `BREADY`
- Burst transfers with up to 16 32-bit float beats per burst

The DRAM slave uses an idle-ready model for the address channels: `ARREADY`
and `AWREADY` may be asserted before the DMA asserts `ARVALID` or `AWVALID`.
A transfer is accepted only when the corresponding `VALID` and `READY` signals
are both high on the same clock cycle.

Outstanding transactions are not implemented in this optimized version.

## DRAM Memory Map

All addresses are byte addresses and each value is a 32-bit float.

| Region | Base Address |
| --- | --- |
| Input image | `0x00000000` |
| Conv1 weight | `0x00100000` |
| Conv1 bias | `0x00200000` |
| Conv2 weight | `0x00300000` |
| Conv2 bias | `0x00700000` |
| Conv3 weight | `0x00800000` |
| Conv3 bias | `0x01000000` |
| Conv4 weight | `0x01100000` |
| Conv4 bias | `0x01b00000` |
| Conv5 weight | `0x01c00000` |
| Conv5 bias | `0x02300000` |
| FC6 weight | `0x02400000` |
| FC6 bias | `0x1a000000` |
| FC7 weight | `0x1a100000` |
| FC7 bias | `0x24b00000` |
| FC8 weight | `0x24c00000` |
| FC8 bias | `0x27600000` |
| Output scores | `0x27700000` |
| Intermediate scratch | `0x27800000` |

The final FC8 score vector is written to `DRAM_OUTPUT_BASE` and then read back
through DMA before softmax and Top-100 printing.
