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

    sc_in_clk clk;
    sc_in<bool> rst;
    sc_out<bool> out_done;

    // ===== FIFO 介面 =====
    sc_fifo_in<double> img_in;
    sc_fifo_out<double> img_out;

    // ===== Internal buffer =====
    // 只存 1 個 Channel 的大小
    vector<double> input_buf;

    SC_HAS_PROCESS(MaxPooling);
    MaxPooling(sc_module_name name,
               int in_h,
               int in_w,
               int ch,
               int kernel,
               int stride);

    void run();

private:
    inline int input_index(int h, int w) const // 一個 channel 大小
    {
        return h * MP_IN_W + w;
    }
};

#endif