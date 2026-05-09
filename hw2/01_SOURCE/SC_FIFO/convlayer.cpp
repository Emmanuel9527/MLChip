#include "convlayer.h"
// #include <iomanip> // for debug output formatting

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

    input_buf.resize(CL_IN_H * CL_IN_W * CL_IN_CH);
    weight_buf.resize(CL_OUT_CH * CL_IN_CH * CL_KERNEL * CL_KERNEL);
    bias_buf.resize(CL_OUT_CH);

    load_weights();
    load_bias();

    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(rst, true);
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
    // reset 初始化區域
    out_done.write(false);
    wait(); // 強制 Thread 在這裡掛起，直到 rst 變成 0 且下一個 Clock 來到

    while (true)
    {
        // 每張新圖開始前，確保 done 為低準位
        out_done.write(false);

        // 收集完整的 Input 特徵圖
        for (int i = 0; i < CL_IN_H * CL_IN_W * CL_IN_CH; i++)
        {
            input_buf[i] = img_in.read();
            wait(); // 讀取1個元素花費 1 clock cycle
        }

        // --- 2. 插入【時間切片】Debug 程式碼 ---
        // 當讀完一整張圖，準備算第一個 pixel 前，我們把 input_buf 當下的狀態印出來
        /*
        cout << fixed << setprecision(6);
        cout << "\n[TIME SLICE] Input Buffer State before First MAC:" << endl;
        cout << "----------------------------------------------------" << endl;
        for (int h = 0; h < 5; h++)
        {
            for (int w = 0; w < 5; w++)
            {
                // 這裡 index 算法：Channel 0, Height h, Width w
                int idx = 0 * (CL_IN_H * CL_IN_W) + h * CL_IN_W + w;
                cout << input_buf[idx] << "\t";
            }
            cout << endl;
        }
        cout << "----------------------------------------------------" << endl;
        */

        // MAC 運算與即時輸出
        // 外三層決定現在要算哪一個輸出的像素
        for (int oc = 0; oc < CL_OUT_CH; oc++)
        {
            for (int oh = 0; oh < CL_OUT_H; oh++)
            {
                for (int ow = 0; ow < CL_OUT_W; ow++)
                {
                    double sum = bias_buf[oc];

                    // 執行跨 Channel 與 Kernel 視窗的 MAC 累加
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

                    // 算完這個像素，立刻塞進 FIFO 丟給下一層
                    img_out.write(sum);
                    wait(); // 輸出花費 1 clock cycle
                }
            }
        }
        out_done.write(true);
        wait(); // 保持一個週期讓下一層看到 done 信號
    }
}