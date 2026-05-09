#include "softmax.h"

Softmax::Softmax(sc_module_name name, int size)
    : sc_module(name),
      SM_SIZE(size)
{
    img_in.init(SM_SIZE);
    img_out.init(SM_SIZE);

    input_buf.resize(SM_SIZE);
    output_buf.resize(SM_SIZE);

    SC_METHOD(run);
    sensitive << clk.pos();
}

void Softmax::run()
{
    if (rst.read() == 1)
    {
        out_valid.write(0);
        return;
    }

    if (in_valid.read() == 1)
    {
        // ===== 1. read input =====
        for (int i = 0; i < SM_SIZE; i++)
        {
            input_buf[i] = img_in[i].read();
        }

        // ===== 2. numerically stable softmax =====
        double max_val = input_buf[0];
        for (int i = 1; i < SM_SIZE; i++)
        {
            if (input_buf[i] > max_val)
            {
                max_val = input_buf[i];
            }
        }

        double sum_exp = 0.0;
        for (int i = 0; i < SM_SIZE; i++)
        {
            output_buf[i] = exp(input_buf[i] - max_val);
            sum_exp += output_buf[i];
        }

        for (int i = 0; i < SM_SIZE; i++)
        {
            output_buf[i] /= sum_exp;
        }

        // ===== 3. write output =====
        for (int i = 0; i < SM_SIZE; i++)
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