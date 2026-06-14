#ifndef DRAM_H
#define DRAM_H

#include "systemc.h"
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

using namespace std;

SC_MODULE(DRAM)
{
    sc_in<bool> clk;
    sc_in<bool> rst;

    // DMA to DRAM read signals.
    sc_in<unsigned int> araddr;
    sc_in<unsigned int> arlen;
    sc_in<unsigned int> arsize;
    sc_in<bool> arvalid;
    sc_out<bool> arready;

    // DRAM to DMA read data signals.
    sc_out<float> rdata;
    sc_out<bool> rvalid;
    sc_in<bool> rready;
    sc_out<bool> rlast;

    // DMA to DRAM write address signals.
    sc_in<unsigned int> awaddr;
    sc_in<unsigned int> awlen;
    sc_in<unsigned int> awsize;
    sc_in<bool> awvalid;
    sc_out<bool> awready;

    // DMA to DRAM write data signals.
    sc_in<float> wdata;
    sc_in<bool> wvalid;
    sc_out<bool> wready;
    sc_in<bool> wlast;

    // DRAM to DMA write response signals.
    sc_out<bool> bvalid;
    sc_in<bool> bready;

    string DATA_PATH;
    string IMAGE_FILE_NAME;
    map<unsigned int, vector<float>> regions;

    void run();
    void initialize();
    string find_data_path();
    void load_region(unsigned int base, const string &filename);
    float read_word(unsigned int byte_addr);
    void write_word(unsigned int byte_addr, float value);

    SC_CTOR(DRAM)
    {
        const char *env_file = getenv("IMAGE_FILE_NAME");
        IMAGE_FILE_NAME = (env_file != NULL) ? env_file : "cat.txt";
        DATA_PATH = find_data_path();
        initialize();
        SC_THREAD(run);
        sensitive << clk.pos();
    }
};

#endif
