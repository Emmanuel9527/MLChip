#include "maxpooling.h"

MaxPooling::MaxPooling(sc_module_name name,
                       int in_h,
                       int in_w,
                       int ch,
                       int kernel,
                       int stride)
    : sc_module(name),
      MP_IN_H(in_h),
      MP_IN_W(in_w),
      MP_CH(ch),
      MP_KERNEL(kernel),
      MP_STRIDE(stride),
      MP_OUT_H((in_h - kernel) / stride + 1),
      MP_OUT_W((in_w - kernel) / stride + 1)
{
    img_in.init(MP_IN_H * MP_IN_W * MP_CH);
    img_out.init(MP_OUT_H * MP_OUT_W * MP_CH);

    input_buf.resize(MP_IN_H * MP_IN_W * MP_CH);
    output_buf.resize(MP_OUT_H * MP_OUT_W * MP_CH);

    SC_METHOD(run);
    sensitive << clk.pos();
}

void MaxPooling::run()
{
    if (rst.read() == 1)
    {
        out_valid.write(0);
        return;
    }

    if (in_valid.read() == 1)
    {
        // ===== 1. read input =====
        for (int i = 0; i < MP_IN_H * MP_IN_W * MP_CH; i++)
        {
            input_buf[i] = img_in[i].read();
        }

        // ===== 2. max pooling =====
        for (int c = 0; c < MP_CH; c++)
        {
            for (int oh = 0; oh < MP_OUT_H; oh++)
            {
                for (int ow = 0; ow < MP_OUT_W; ow++)
                {
                    int start_h = oh * MP_STRIDE;
                    int start_w = ow * MP_STRIDE;

                    double max_val = input_buf[input_index(c, start_h, start_w)];

                    for (int kh = 0; kh < MP_KERNEL; kh++)
                    {
                        for (int kw = 0; kw < MP_KERNEL; kw++)
                        {
                            int ih = start_h + kh;
                            int iw = start_w + kw;

                            int in_idx = input_index(c, ih, iw);

                            if (input_buf[in_idx] > max_val)
                            {
                                max_val = input_buf[in_idx];
                            }
                        }
                    }

                    int out_idx = output_index(c, oh, ow);
                    output_buf[out_idx] = max_val;
                }
            }
        }

        // ===== 3. write output =====
        for (int i = 0; i < MP_OUT_H * MP_OUT_W * MP_CH; i++)
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