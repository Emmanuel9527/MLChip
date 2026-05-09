#include "relu.h"

ReLU::ReLU(sc_module_name name, int size)
    : sc_module(name),
      RL_SIZE(size)
{
    img_in.init(RL_SIZE);
    img_out.init(RL_SIZE);

    input_buf.resize(RL_SIZE);
    output_buf.resize(RL_SIZE);

    SC_METHOD(run);
    sensitive << clk.pos();
}

void ReLU::run()
{
    if (rst.read() == 1)
    {
        out_valid.write(0);
        return;
    }

    if (in_valid.read() == 1)
    {
        // ===== 1. read input =====
        for (int i = 0; i < RL_SIZE; i++)
        {
            input_buf[i] = img_in[i].read();
        }

        // ===== 2. ReLU =====
        for (int i = 0; i < RL_SIZE; i++)
        {
            if (input_buf[i] > 0.0)
            {
                output_buf[i] = input_buf[i];
            }
            else
            {
                output_buf[i] = 0.0;
            }
        }

        // ===== 3. write output =====
        for (int i = 0; i < RL_SIZE; i++)
        {
            img_out[i].write(output_buf[i]);
        }

        out_valid.write(1);
    }
    else
    {
        out_valid.write(0);
    }
}