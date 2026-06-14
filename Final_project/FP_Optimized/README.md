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
SRAM. It then processes up to sixteen output neurons at a time, mapped to the
sixteen original PEs in the physical 4x4 mesh. For each input tile, the input
activation tile is broadcast through the NoC, the needed weight rows are fetched
from DRAM by AXI DMA, written into SRAM, read from SRAM, and sent to the
corresponding PE. Each PE accumulates partial sums in its local accumulator
register, and the controller writes partial sums and final output tiles back to
SRAM. This is a cycle-aware behavioral model of a scratchpad-fed PE-based 2D
systolic dataflow.

The SRAM model is defined in `global_sram.h`:

- Capacity: 10,485,760 32-bit words
- Data width: 32 bits
- Banks: 1
- Read latency: 1 modeled cycle per word
- Write latency: 1 modeled cycle per word

To keep SystemC simulation practical, the full SRAM cycle cost is accumulated
in metrics while the simulator advances one synchronization cycle per SRAM
block access.

The systolic FC behavior is implemented in the original PE model:

- Array size: PE0..PE15 arranged as a physical 4x4 mesh
- Sixteen output-neuron rows are computed in parallel
- Each PE keeps a local accumulator register for partial sums
- Each PE contains a `PeLocalSram` SystemC module for input, weight, and bias tiles
- Each PE contains a `NonlinearFunction` SystemC module for ReLU and max selection
- Input activation tiles are broadcast from the controller HOST port
- Weight tiles are staged through SRAM before being loaded into PE local buffers
- Pipeline fill/drain cycles are included in the modeled cycle count

## PE Internal Blocks

Each original PE is modeled with three hardware-oriented blocks:

- Local SRAM: `PeLocalSram` stores input activation tiles, weight tiles, and
  bias tiles. It also tracks local read/write traffic.
- MAC array: the PE datapath uses `MACS_PER_CYCLE` to model the internal MAC
  array throughput.
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

This graph exposes row-level parallelism across output neurons and sequential
accumulation along the input dimension.

### DFG Mapping

The physical PE mesh is a 4x4 array with PE ids:

```text
 0   1   2   3
 4   5   6   7
 8   9  10  11
12  13  14  15
```

For each FC output tile, up to sixteen output neurons are mapped to the sixteen
PEs:

```text
PE k computes output neuron out_base + k
```

The mapping function is:

```text
PE id = (o - out_base), for 0 <= PE id < 16
```

The input dimension is tiled by `SYSTOLIC_INPUT_TILE_WORDS`. For each input
tile:

1. The input activation tile is read from Global SRAM and broadcast through the
   NoC HOST port to PE0..PE15.
2. Each PE receives its own weight tile for its assigned output row.
3. Each PE performs MAC accumulation into its local accumulator register.
4. The current partial sums are written back to the Global SRAM partial-sum
   region for observability and cycle-aware SRAM traffic accounting.

The schedule is:

```text
for out_base in output neurons step 16:
    initialize PE accumulators with bias[out_base : out_base + 15]
    for input_base in input vector step SYSTOLIC_INPUT_TILE_WORDS:
        broadcast input[input_base tile] to PE0..PE15
        load weight row tile for each PE
        PE-local MAC array accumulates partial sums
        write partial sums to SRAM
    apply nonlinear function in each PE
    write output tile to SRAM
```

The modeled systolic cycle count for one input tile is:

```text
ceil(tile_words / 4) + 4 + 4 - 2
```

where the first term models four MAC columns per PE-level systolic step, and
`4 + 4 - 2` is the 4x4 array fill/drain overhead. The arithmetic is still
behavioral, but the data movement, buffering, accumulator state, and cycle
metrics follow a PE-level systolic dataflow.

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
