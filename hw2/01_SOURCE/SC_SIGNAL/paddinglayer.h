#ifndef PADDING_LAYER_H
#define PADDING_LAYER_H

#include <systemc.h>
#include <vector>
#include <iostream>

using namespace std;

SC_MODULE(PaddingLayer)
{
public:
    int PD_IN_H;
    int PD_IN_W;
    int PD_CH;
    int PD_PAD;
    int PD_OUT_H;
    int PD_OUT_W;

    sc_in_clk clk;
    sc_in<bool> rst;
    sc_in<bool> in_valid;

    sc_vector<sc_in<double>> img_in{"img_in", 0};

    sc_out<bool> out_valid;
    sc_vector<sc_out<double>> img_out{"img_out", 0};

    vector<double> input_buf;
    vector<double> output_buf;

    SC_HAS_PROCESS(PaddingLayer);
    PaddingLayer(sc_module_name name,
                 int in_h,
                 int in_w,
                 int ch,
                 int pad);

    void run();

private:
    inline int input_index(int c, int h, int w) const
    {
        return c * (PD_IN_H * PD_IN_W) + h * PD_IN_W + w;
    }

    inline int output_index(int c, int h, int w) const
    {
        return c * (PD_OUT_H * PD_OUT_W) + h * PD_OUT_W + w;
    }
};

#endif