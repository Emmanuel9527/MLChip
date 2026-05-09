#ifndef CONV_LAYER_H
#define CONV_LAYER_H

#include <systemc.h>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <cstdlib>

using namespace std;

SC_MODULE(ConvLayer)
{
public:
    // ===== Layer parameters =====
    int CL_IN_H;
    int CL_IN_W;
    int CL_IN_CH;
    int CL_OUT_CH;
    int CL_KERNEL;
    int CL_STRIDE;
    int CL_OUT_H;
    int CL_OUT_W;

    // ===== File path =====
    string weight_file;
    string bias_file;

    // ===== Ports =====
    sc_in_clk clk;
    sc_in<bool> rst;
    sc_in<bool> in_valid;

    sc_vector<sc_in<double>> img_in{"img_in", 0};
    sc_out<bool> out_valid;
    sc_vector<sc_out<double>> img_out{"img_out", 0};

    // ===== Internal buffer =====
    vector<double> input_buf;
    vector<double> weight_buf;
    vector<double> bias_buf;
    vector<double> output_buf;

    SC_HAS_PROCESS(ConvLayer);
    ConvLayer(sc_module_name name,
              int in_h,
              int in_w,
              int in_ch,
              int out_ch,
              int kernel,
              int stride,
              const string &w_file,
              const string &b_file);

    void run();

private:
    void load_weights();
    void load_bias();

    inline int input_index(int c, int h, int w) const
    {
        return c * (CL_IN_H * CL_IN_W) + h * CL_IN_W + w;
    }

    inline int weight_index(int oc, int ic, int kh, int kw) const
    {
        return oc * (CL_IN_CH * CL_KERNEL * CL_KERNEL) + ic * (CL_KERNEL * CL_KERNEL) + kh * CL_KERNEL + kw;
    }

    inline int output_index(int oc, int h, int w) const
    {
        return oc * (CL_OUT_H * CL_OUT_W) + h * CL_OUT_W + w;
    }
};

#endif