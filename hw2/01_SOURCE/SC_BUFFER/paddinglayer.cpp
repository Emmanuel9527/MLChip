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

    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(rst, true);
}

void PaddingLayer::run()
{
    // ===== Reset 初始化區域 =====
    out_valid.write(false);
    wait();

    // ===== SC_THREAD 無窮迴圈 =====
    while (true)
    {
        // 每輪開始前先確保完成訊號是低的
        out_valid.write(false);

        // 「卡位」等待 直到上一層的 done 訊號傳進這層的 in_valid
        while (in_valid.read() == false)
        {
            wait();
        }

        // 執行 Padding：直接遍歷輸出的長寬，判斷要在這格填 0 還是填原圖
        for (int c = 0; c < PD_CH; c++)
        {
            for (int h = 0; h < PD_OUT_H; h++)
            {
                for (int w = 0; w < PD_OUT_W; w++)
                {
                    int out_idx = output_index(c, h, w);

                    // 判斷目前座標是否落在邊緣的 padding 區域
                    if (h < PD_PAD || h >= PD_IN_H + PD_PAD ||
                        w < PD_PAD || w >= PD_IN_W + PD_PAD)
                    {
                        // 在 padding 區，直接寫入 0
                        img_out[out_idx].write(0.0);
                    }
                    else
                    {
                        // 在原圖區，反推回原圖的座標並從 img_in 讀取
                        int in_h = h - PD_PAD;
                        int in_w = w - PD_PAD;
                        int in_idx = input_index(c, in_h, in_w);

                        img_out[out_idx].write(img_in[in_idx].read());
                    }
                }
            }
        }

        // 運算結束，拉高完成訊號通知下一層
        out_valid.write(true);
        wait();
    }
}