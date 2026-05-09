#ifndef FULLY_CONNECTED_H
#define FULLY_CONNECTED_H

#include <systemc.h>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <cstdlib>

using namespace std;

SC_MODULE(FullyConnected)
{
public:
    // ===== Layer parameters =====
    int FC_IN_SIZE;
    int FC_OUT_SIZE;

    // ===== File path =====
    string weight_file;
    string bias_file;

    // ===== Ports =====
    sc_in_clk clk;
    sc_in<bool> rst;
    sc_out<bool> out_done;

    // ===== FIFO 介面 =====
    sc_fifo_in<double> img_in;
    sc_fifo_out<double> img_out;

    // ===== Internal buffer =====
    // 把整條fmap存在模組內部的 SRAM 裡
    vector<double> input_buf;
    vector<double> weight_buf;
    vector<double> bias_buf;

    SC_HAS_PROCESS(FullyConnected);
    FullyConnected(sc_module_name name,
                   int in_size,
                   int out_size,
                   const string &w_file,
                   const string &b_file);

    void run();

private:
    void load_weights();
    void load_bias();

    inline int weight_index(int out_idx, int in_idx) const
    {
        return out_idx * FC_IN_SIZE + in_idx;
    }
};

#endif