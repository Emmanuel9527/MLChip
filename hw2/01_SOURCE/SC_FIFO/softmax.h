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
    sc_out<bool> out_done;

    // sc_vector<sc_in<double>> img_in{"img_in", 0};
    // sc_vector<sc_out<double>> img_out{"img_out", 0};
    //  FIFO ports for input and output
    sc_fifo_in<double> img_in;
    sc_fifo_out<double> img_out;

    // softmax 需要全部讀完才能輸出，input 需暫存
    vector<double> input_buf;

    // 因為要額外參數，所以不能用 SC_CTOR 建構了，只能在 SC_CTOR 以外寫建構子(寫在.cpp是標準做法)，並用 SC_HAS_PROCESS 先在這邊宣告
    // 用SC_HAS_PROCESS 在此模組註冊 process
    SC_HAS_PROCESS(Softmax);
    Softmax(sc_module_name name, int size);

    void run();
};

#endif