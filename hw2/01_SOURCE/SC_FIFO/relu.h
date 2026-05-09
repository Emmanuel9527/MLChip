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

    sc_in_clk clk;
    sc_in<bool> rst;
    sc_out<bool> out_done;

    // ===== FIFOPorts =====
    sc_fifo_in<double> img_in;
    sc_fifo_out<double> img_out;

    SC_HAS_PROCESS(ReLU);
    ReLU(sc_module_name name, int size);

    void run();
};

#endif