#ifndef ZERO_PADDING_H
#define ZERO_PADDING_H

#include <systemc.h>
#include <vector>

#define IN_H 224
#define IN_W 224
#define CHANNEL 3

#define OUT_H 227
#define OUT_W 227

SC_MODULE(ZeroPadding)
{
    // ===== Ports =====
    sc_in_clk clk;
    sc_in<bool> rst;
    sc_in<bool> in_valid;
    sc_out<bool> out_valid;

    // 恢復使用 sc_vector 來對接上層的 sc_buffer 陣列
    sc_vector<sc_in<double>> img_in{"img_in", IN_H * IN_W * CHANNEL};
    sc_vector<sc_out<double>> img_out{"img_out", OUT_H * OUT_W * CHANNEL};

    void run();

    SC_CTOR(ZeroPadding)
    {
        // 這裡保留 SC_THREAD。
        // 但要注意，改成 sc_in/sc_out 後，不再有 FIFO 那種會卡住等待(blocking)的特性了。
        SC_THREAD(run);
        sensitive << clk.pos();
        async_reset_signal_is(rst, true);
    }
};

#endif