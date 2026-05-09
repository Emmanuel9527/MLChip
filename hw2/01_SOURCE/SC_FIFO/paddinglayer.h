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
    sc_out<bool> out_done;

    // ===== FIFO 介面 =====
    sc_fifo_in<double> img_in;
    sc_fifo_out<double> img_out;

    SC_HAS_PROCESS(PaddingLayer);
    PaddingLayer(sc_module_name name,
                 int in_h,
                 int in_w,
                 int ch,
                 int pad);

    void run();

private:
    /*
    在 FIFO 串流中，資料是依序流動的，我們不再需要去計算一維陣列的絕對位置
    inline int input_index(int c, int h, int w) const
        {
            return c * (PD_IN_H * PD_IN_W) + h * PD_IN_W + w;
        }

        inline int output_index(int c, int h, int w) const
        {
            return c * (PD_OUT_H * PD_OUT_W) + h * PD_OUT_W + w;
        }
    */
};

#endif