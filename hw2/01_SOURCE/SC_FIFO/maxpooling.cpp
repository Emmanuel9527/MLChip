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
    input_buf.resize(MP_IN_H * MP_IN_W);

    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(rst, true);
}

void MaxPooling::run()
{
    // reset 初始化區域
    out_done.write(false);
    wait(); // 強制 Thread 在這裡掛起，直到 rst 變成 0 且下一個 Clock 來到

    while (true)
    {
        // 開始處理新圖，拉低 Done
        out_done.write(false);

        // 逐個 Channel 進行處理
        for (int c = 0; c < MP_CH; c++)
        {
            // 讀取 1 個 Channel 的輸入到 input_buf 中
            for (int i = 0; i < MP_IN_H * MP_IN_W; i++)
            {
                input_buf[i] = img_in.read();
                wait(); // 每讀取一個像素花費 1 clock cycle
            }

            // 進行 Max Pooling 並直接輸出
            for (int oh = 0; oh < MP_OUT_H; oh++)
            {
                for (int ow = 0; ow < MP_OUT_W; ow++)
                {
                    int start_h = oh * MP_STRIDE;
                    int start_w = ow * MP_STRIDE;

                    double max_val = input_buf[input_index(start_h, start_w)];

                    for (int kh = 0; kh < MP_KERNEL; kh++)
                    {
                        for (int kw = 0; kw < MP_KERNEL; kw++)
                        {
                            int ih = start_h + kh;
                            int iw = start_w + kw;

                            int in_idx = input_index(ih, iw);

                            if (input_buf[in_idx] > max_val)
                            {
                                max_val = input_buf[in_idx];
                            }
                        }
                    }

                    // 算完一個 window 的最大值後，直接寫入 FIFO
                    img_out.write(max_val);
                    wait(); // 每寫入一個像素花費 1 clock cycle
                }
            }
        }
        out_done.write(true);
        wait(); // 維持一格讓下一層看到
    }
}