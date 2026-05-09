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

    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(rst, true);
}

void PaddingLayer::run()
{
    // reset 初始化區域
    out_done.write(false);
    wait(); // 強制 Thread 在這裡掛起，直到 rst 變成 0 且下一個 Clock 來到

    while (true)
    {
        out_done.write(false);
        // 按照輸出的長寬大小來跑迴圈 (Channel -> Height -> Width)
        for (int c = 0; c < PD_CH; c++)
        {
            for (int h = 0; h < PD_OUT_H; h++)
            {
                for (int w = 0; w < PD_OUT_W; w++)
                {
                    // 判斷當前的輸出座標是否落在補零區，只要高度或寬度落在邊緣的 PD_PAD 範圍內，就是補零區
                    if (h < PD_PAD || h >= PD_IN_H + PD_PAD ||
                        w < PD_PAD || w >= PD_IN_W + PD_PAD)
                    {
                        img_out.write(0.0);
                    }
                    else
                    {
                        double pixel = img_in.read(); // img_in.read() 內建了等待功能。
                        img_out.write(pixel);
                    }

                    // 一個像素要一個 clock cycle
                    wait();
                }
            }
        }
        out_done.write(true);
        wait(); // 讓信號維持一個週期，方便下一層偵測
    }
}
