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
    // 動態初始化 sc_vector 大小
    img_in.init(CL_IN_H * CL_IN_W * CL_IN_CH);
    img_out.init(CL_OUT_H * CL_OUT_W * CL_OUT_CH);

    weight_buf.resize(CL_OUT_CH * CL_IN_CH * CL_KERNEL * CL_KERNEL);
    bias_buf.resize(CL_OUT_CH);

    load_weights();
    load_bias();

    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(rst, true); // 確保 rst 為 1 時能重置 Thread
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
    // ===== Reset 初始化區域 =====
    out_valid.write(false);
    wait(); // 等待 rst 結束

    // ===== SC_THREAD 的無窮迴圈 =====
    while (true)
    {
        out_valid.write(false);

        // 等待上一層的訊號：如果 in_valid 是 0，就在這裡等下一個 Clock
        while (in_valid.read() == false)
        {
            wait();
        }

        // 開始進行卷積運算 (當 in_valid == 1 時觸發)
        for (int oc = 0; oc < CL_OUT_CH; oc++)
        {
            // if (oc % 10 == 0)
            //     cout << "Processing Output Channel: " << oc << endl; // debug
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

                                // 直接從 sc_vector<sc_buffer> 讀取
                                sum += img_in[in_idx].read() * weight_buf[wt_idx];
                            }
                        }
                    }

                    int out_idx = output_index(oc, oh, ow);
                    img_out[out_idx].write(sum);

                    wait(); // 每算完一個輸出像素就等待一個 Clock，模擬硬體的流水線行為

                    // 如果你想模擬真實硬體「每個 pixel 算完花一個 cycle」
                    // 可以在這裡加上 wait();
                    // 但因為這張特徵圖太大了，建議不要加在最內層，否則模擬會跑超久！
                }
            }
        }

        // 3. 整張特徵圖算完，將 out_valid 拉高，通知下一層
        out_valid.write(true);
        wait(); // 維持一個 cycle，讓下一層抓到這個 true 訊號
    }
}
