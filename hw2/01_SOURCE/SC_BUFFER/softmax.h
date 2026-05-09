#ifndef SOFTMAX_H
#define SOFTMAX_H

#include <systemc.h>
#include <vector>
#include <iostream>
#include <cmath>

using namespace std;

SC_MODULE(Softmax)
{
public:
    int SM_SIZE;

    sc_in_clk clk;
    sc_in<bool> rst;
    sc_in<bool> in_valid;

    sc_vector<sc_in<double>> img_in{"img_in", 0};

    sc_out<bool> out_valid;
    sc_vector<sc_out<double>> img_out{"img_out", 0};

    SC_HAS_PROCESS(Softmax);
    Softmax(sc_module_name name, int size);

    void run();
};

#endif