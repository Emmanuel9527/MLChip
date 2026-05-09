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

    sc_vector<sc_in<double>> img_in{"img_in", IN_H * IN_W * CHANNEL};

    sc_out<bool> out_valid;
    sc_vector<sc_out<double>> img_out{"img_out", OUT_H * OUT_W * CHANNEL};

    // ===== Internal buffer =====
    std::vector<double> input_buf;
    std::vector<double> output_buf;

    void run();

    SC_CTOR(ZeroPadding)
    {
        SC_METHOD(run);
        sensitive << clk.pos();

        input_buf.resize(IN_H * IN_W * CHANNEL);
        output_buf.resize(OUT_H * OUT_W * CHANNEL);
    }
};

#endif