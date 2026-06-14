#ifndef AXI_DMA_H
#define AXI_DMA_H

#include "systemc.h"

SC_MODULE(AxiDma)
{
    sc_in<bool> clk;
    sc_in<bool> rst;

    // Simple command interface used by the Controller.
    // One command transfers cmd_len 32-bit float words starting at cmd_addr.
    sc_in<bool> cmd_valid;
    sc_out<bool> cmd_ready;
    sc_in<bool> cmd_write;
    sc_in<unsigned int> cmd_addr;
    sc_in<unsigned int> cmd_len;
    sc_out<bool> done;

    // Streaming data interface between Controller and DMA.
    // read_* is used for DRAM-to-Controller transfers.
    // write_* is used for Controller-to-DRAM transfers.
    sc_out<float> read_data;
    sc_out<bool> read_valid;
    sc_in<bool> read_ready;
    sc_in<float> write_data;
    sc_in<bool> write_valid;
    sc_out<bool> write_ready;

    // AXI4-like read address channel.
    sc_out<unsigned int> araddr;
    sc_out<unsigned int> arlen;
    sc_out<unsigned int> arsize;
    sc_out<bool> arvalid;
    sc_in<bool> arready;

    // AXI4-like read data channel.
    sc_in<float> rdata;
    sc_in<bool> rvalid;
    sc_out<bool> rready;
    sc_in<bool> rlast;

    // AXI4-like write address channel.
    sc_out<unsigned int> awaddr;
    sc_out<unsigned int> awlen;
    sc_out<unsigned int> awsize;
    sc_out<bool> awvalid;
    sc_in<bool> awready;

    // AXI4-like write data channel.
    sc_out<float> wdata;
    sc_out<bool> wvalid;
    sc_in<bool> wready;
    sc_out<bool> wlast;

    // AXI4-like write response channel.
    sc_in<bool> bvalid;
    sc_out<bool> bready;

    // Counters are used for trace logs and report metrics.
    unsigned long long read_words;
    unsigned long long write_words;
    unsigned long long read_bursts;
    unsigned long long write_bursts;

    void run();
    void read_burst(unsigned int addr, unsigned int beats);
    void write_burst(unsigned int addr, unsigned int beats);

    SC_CTOR(AxiDma)
    {
        read_words = 0;
        write_words = 0;
        read_bursts = 0;
        write_bursts = 0;
        SC_THREAD(run);
        sensitive << clk.pos();
    }
};

#endif
