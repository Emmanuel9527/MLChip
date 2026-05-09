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

    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(rst, true);
}

void MaxPooling::run()
{
    // ===== Reset 初始化區域 =====
    out_valid.write(false);
    wait();

    // ===== SC_THREAD 無窮迴圈 =====
    while (true)
    {
        // 每輪開始前先確保完成訊號是低的
        out_valid.write(false);

        // 等待直到上一層的 done 訊號傳進這層的 in_valid
        while (in_valid.read() == false)
        {
            wait();
        }

        // 開始進行 Max Pooling 運算
        for (int c = 0; c < MP_CH; c++)
        {
            for (int oh = 0; oh < MP_OUT_H; oh++)
            {
                for (int ow = 0; ow < MP_OUT_W; ow++)
                {
                    int start_h = oh * MP_STRIDE;
                    int start_w = ow * MP_STRIDE;

                    // 先讀取 pooling window 左上角的第一個值當作初始 max_val
                    double max_val = img_in[input_index(c, start_h, start_w)].read();

                    for (int kh = 0; kh < MP_KERNEL; kh++)
                    {
                        for (int kw = 0; kw < MP_KERNEL; kw++)
                        {
                            int ih = start_h + kh;
                            int iw = start_w + kw;

                            int in_idx = input_index(c, ih, iw);

                            // 直接從 img_in 陣列讀取當下 pixel 的值來比較
                            double current_val = img_in[in_idx].read();

                            if (current_val > max_val)
                            {
                                max_val = current_val;
                            }
                        }
                    }

                    // 單一 Pooling 視窗找完最大值後，直接寫出，不透過 output_buf
                    int out_idx = output_index(c, oh, ow);
                    img_out[out_idx].write(max_val);
                }
            }
        }

        // 運算結束：拉高完成訊號通知下一層
        out_valid.write(true);
        wait();
    }
}