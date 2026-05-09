#ifndef RELU_H
#define RELU_H

#include <systemc.h>
#include <vector>
#include <iostream>

using namespace std;

SC_MODULE(ReLU)
{
public:
    // ===== Layer parameters =====
    int RL_SIZE;

    // ===== Ports =====
    sc_in_clk clk;
    sc_in<bool> rst;
    sc_in<bool> in_valid;

    sc_vector<sc_in<double>> img_in{"img_in", 0};

    sc_out<bool> out_valid;
    sc_vector<sc_out<double>> img_out{"img_out", 0};

    // ===== Internal buffer =====
    vector<double> input_buf;
    vector<double> output_buf;

    SC_HAS_PROCESS(ReLU);
    ReLU(sc_module_name name, int size);

    void run();
};

#endif