#include "convlayer.h"

ConvLayer::ConvLayer(sc_module_name name,
                     int in_h,
                     int in_w,
                     int in_ch,
                     int out_ch,
                     int kernel,
                     int stride,
                     const string &w_file,
                     const string &b_file)
    : sc_module(name),
      CL_IN_H(in_h),
      CL_IN_W(in_w),
      CL_IN_CH(in_ch),
      CL_OUT_CH(out_ch),
      CL_KERNEL(kernel),
      CL_STRIDE(stride),
      CL_OUT_H((in_h - kernel) / stride + 1),
      CL_OUT_W((in_w - kernel) / stride + 1),
      weight_file(w_file),
      bias_file(b_file)
{
    img_in.init(CL_IN_H * CL_IN_W * CL_IN_CH);
    img_out.init(CL_OUT_H * CL_OUT_W * CL_OUT_CH);

    input_buf.resize(CL_IN_H * CL_IN_W * CL_IN_CH);
    weight_buf.resize(CL_OUT_CH * CL_IN_CH * CL_KERNEL * CL_KERNEL);
    bias_buf.resize(CL_OUT_CH);
    output_buf.resize(CL_OUT_H * CL_OUT_W * CL_OUT_CH);

    load_weights();
    load_bias();

    SC_METHOD(run);
    sensitive << clk.pos();
}

void ConvLayer::load_weights()
{
    ifstream fin(weight_file);
    if (!fin.is_open())
    {
        cerr << "[ConvLayer] Cannot open weight file: " << weight_file << endl;
        exit(1);
    }

    for (int i = 0; i < (int)weight_buf.size(); i++)
    {
        if (!(fin >> weight_buf[i]))
        {
            cerr << "[ConvLayer] Weight format error at index " << i << endl;
            exit(1);
        }
    }

    fin.close();
}

void ConvLayer::load_bias()
{
    ifstream fin(bias_file);
    if (!fin.is_open())
    {
        cerr << "[ConvLayer] Cannot open bias file: " << bias_file << endl;
        exit(1);
    }

    for (int i = 0; i < (int)bias_buf.size(); i++)
    {
        if (!(fin >> bias_buf[i]))
        {
            cerr << "[ConvLayer] Bias format error at index " << i << endl;
            exit(1);
        }
    }

    fin.close();
}

void ConvLayer::run()
{
    if (rst.read() == 1)
    {
        out_valid.write(0);
        return;
    }

    if (in_valid.read() == 1)
    {
        // ===== 1. read input =====
        for (int i = 0; i < CL_IN_H * CL_IN_W * CL_IN_CH; i++)
        {
            input_buf[i] = img_in[i].read();
        }

        // ===== 2. convolution =====
        for (int oc = 0; oc < CL_OUT_CH; oc++)
        {
            for (int oh = 0; oh < CL_OUT_H; oh++)
            {
                for (int ow = 0; ow < CL_OUT_W; ow++)
                {
                    double sum = bias_buf[oc];

                    for (int ic = 0; ic < CL_IN_CH; ic++)
                    {
                        for (int kh = 0; kh < CL_KERNEL; kh++)
                        {
                            for (int kw = 0; kw < CL_KERNEL; kw++)
                            {
                                int ih = oh * CL_STRIDE + kh;
                                int iw = ow * CL_STRIDE + kw;

                                int in_idx = input_index(ic, ih, iw);
                                int wt_idx = weight_index(oc, ic, kh, kw);

                                sum += input_buf[in_idx] * weight_buf[wt_idx];
                            }
                        }
                    }

                    int out_idx = output_index(oc, oh, ow);
                    output_buf[out_idx] = sum;
                }
            }
        }

        // ===== 3. write output =====
        for (int i = 0; i < CL_OUT_H * CL_OUT_W * CL_OUT_CH; i++)
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