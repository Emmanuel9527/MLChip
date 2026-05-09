#include "ZeroPadding.h"

void ZeroPadding::run()
{
    if (rst.read() == 1)
    {
        out_valid.write(0);
        return;
    }

    if (in_valid.read() == 1)
    {
        // ===== 1. 讀 input =====
        for (int i = 0; i < IN_H * IN_W * CHANNEL; i++)
        {
            input_buf[i] = img_in[i].read();
        }

        // ===== 2. padding（全部先設 0）=====
        for (int i = 0; i < OUT_H * OUT_W * CHANNEL; i++)
        {
            output_buf[i] = 0.0;
        }

        // ===== 3. 填入原圖 =====
        for (int c = 0; c < CHANNEL; c++)
        {
            for (int h = 0; h < IN_H; h++)
            {
                for (int w = 0; w < IN_W; w++)
                {
                    int in_idx = c * (IN_H * IN_W) + h * IN_W + w;

                    int out_h = h + 2; // 上 padding
                    int out_w = w + 2; // 左 padding

                    int out_idx = c * (OUT_H * OUT_W) + out_h * OUT_W + out_w;

                    output_buf[out_idx] = input_buf[in_idx];
                }
            }
        }

        // ===== 4. 輸出 =====
        for (int i = 0; i < OUT_H * OUT_W * CHANNEL; i++)
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