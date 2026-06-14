# Final Project Baseline

This folder contains the baseline implementation for the final project.

## Build and Run

```sh
make cat
make dog
```

The program reads `IMAGE_FILE_NAME` from the Makefile target. The DRAM model
searches for the data folder in this order:

1. `./data/`
2. `../data/`
3. `../../data/`
4. `../../hw4/data/`
5. `../../Final_report/data/`

## Baseline Architecture

- 4x4 mesh NoC is inherited from HW4.
- Router 0 is connected to the controller.
- Routers 1 through 15 are connected to worker cores and PEs.
- HW4 PE compute behavior is preserved as the baseline.
- ROM-based tensor loading is replaced by a DRAM model and an AXI4-like DMA.

## AXI4-like DMA Model

The DMA supports:

- Read address channel: `ARADDR`, `ARLEN`, `ARSIZE`, `ARVALID`, `ARREADY`
- Read data channel: `RDATA`, `RVALID`, `RREADY`, `RLAST`
- Write address channel: `AWADDR`, `AWLEN`, `AWSIZE`, `AWVALID`, `AWREADY`
- Write data channel: `WDATA`, `WVALID`, `WREADY`, `WLAST`
- Write response channel: `BVALID`, `BREADY`
- Burst transfers with up to 16 32-bit float beats per burst

Outstanding transactions are not implemented in this baseline.

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
