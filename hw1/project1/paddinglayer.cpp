#include "paddinglayer.h"

PaddingLayer::PaddingLayer(sc_module_name name,
                           int in_h,
                           int in_w,
                           int ch,
                           int pad)
    : sc_module(name),
      PD_IN_H(in_h),
      PD_IN_W(in_w),
      PD_CH(ch),
      PD_PAD(pad),
      PD_OUT_H(in_h + 2 * pad),
      PD_OUT_W(in_w + 2 * pad)
{
    img_in.init(PD_IN_H * PD_IN_W * PD_CH);
    img_out.init(PD_OUT_H * PD_OUT_W * PD_CH);

    input_buf.resize(PD_IN_H * PD_IN_W * PD_CH);
    output_buf.resize(PD_OUT_H * PD_OUT_W * PD_CH);

    SC_METHOD(run);
    sensitive << clk.pos();
}

void PaddingLayer::run()
{
    if (rst.read() == 1)
    {
        out_valid.write(0);
        return;
    }

    if (in_valid.read() == 1)
    {
        for (int i = 0; i < PD_IN_H * PD_IN_W * PD_CH; i++)
        {
            input_buf[i] = img_in[i].read();
        }

        for (int i = 0; i < PD_OUT_H * PD_OUT_W * PD_CH; i++)
        {
            output_buf[i] = 0.0;
        }

        for (int c = 0; c < PD_CH; c++)
        {
            for (int h = 0; h < PD_IN_H; h++)
            {
                for (int w = 0; w < PD_IN_W; w++)
                {
                    int in_idx = input_index(c, h, w);
                    int out_idx = output_index(c, h + PD_PAD, w + PD_PAD);
                    output_buf[out_idx] = input_buf[in_idx];
                }
            }
        }

        for (int i = 0; i < PD_OUT_H * PD_OUT_W * PD_CH; i++)
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