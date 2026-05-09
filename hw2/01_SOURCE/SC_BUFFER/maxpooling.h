#ifndef MAX_POOLING_H
#define MAX_POOLING_H

#include <systemc.h>
#include <vector>
#include <iostream>

using namespace std;

SC_MODULE(MaxPooling)
{
public:
    // ===== Layer parameters =====
    int MP_IN_H;
    int MP_IN_W;
    int MP_CH;
    int MP_KERNEL;
    int MP_STRIDE;
    int MP_OUT_H;
    int MP_OUT_W;

    // ===== Ports =====
    sc_in_clk clk;
    sc_in<bool> rst;
    sc_in<bool> in_valid;

    sc_vector<sc_in<double>> img_in{"img_in", 0};

    sc_out<bool> out_valid;
    sc_vector<sc_out<double>> img_out{"img_out", 0};

    SC_HAS_PROCESS(MaxPooling);
    MaxPooling(sc_module_name name,
               int in_h,
               int in_w,
               int ch,
               int kernel,
               int stride);

    void run();

private:
    inline int input_index(int c, int h, int w) const
    {
        return c * (MP_IN_H * MP_IN_W) + h * MP_IN_W + w;
    }

    inline int output_index(int c, int h, int w) const
    {
        return c * (MP_OUT_H * MP_OUT_W) + h * MP_OUT_W + w;
    }
};

#endif