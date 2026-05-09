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
    sc_out<bool> out_done;

    // 不要用vector
    // sc_vector<sc_in<double>> img_in{"img_in", IN_H * IN_W * CHANNEL};
    // sc_vector<sc_out<double>> img_out{"img_out", OUT_H * OUT_W * CHANNEL};

    // sc_vector<sc_in<double>> img_in{"img_in", IN_H * IN_W * CHANNEL};
    // sc_vector<sc_out<double>> img_out{"img_out", OUT_H * OUT_W * CHANNEL};

    sc_fifo_in<double> img_in; // port，給外層模組的 SC_FIFO channel 連接用
    sc_fifo_out<double> img_out;

    void run();

    SC_CTOR(ZeroPadding)
    {
        // SC_METHOD(run);
        // FIFO 需要用 thread，因為 method 不能有 wait()，而 FIFO 的讀寫需要 wait() 來等待資料
        SC_THREAD(run);
        sensitive << clk.pos();
        async_reset_signal_is(rst, true); // 非同步重製訊號設定:隨時隨地盯著這條訊號線，一旦它符合條件，立刻把這個 Thread 砍掉重練
    }
};

#endif