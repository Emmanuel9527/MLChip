#include "ZeroPadding.h"

void ZeroPadding::run()
{
    // reset 初始化區域
    out_done.write(false);
    wait(); // 強制 Thread 在這裡掛起，直到 rst 變成 0 且下一個 Clock 來到

    // SC_THREAD 需要一個無窮迴圈來持續等待與處理資料
    while (true)
    {
        out_done.write(false);

        // 依照 Raster Scan 順序：Channel -> Height -> Width
        for (int c = 0; c < CHANNEL; c++)
        {
            for (int h = 0; h < OUT_H; h++)
            {
                for (int w = 0; w < OUT_W; w++)
                {
                    // 判斷目前的輸出座標是否落在「補零區」
                    // 上補 2 (h < 2), 下補 1 (h >= IN_H + 2)
                    // 左補 2 (w < 2), 右補 1 (w >= IN_W + 2)
                    bool is_padding = (h < 2 || h >= IN_H + 2 || w < 2 || w >= IN_W + 2);

                    /*
                    debug 區
                    if (c == 0 && h == 2 && w < 5)
                    {
                        cout << "[ZP DEBUG] c=" << c << " h=" << h << " w=" << w
                             << (is_padding ? " -> Action: [WRITE 0]" : " -> Action: [READ & WRITE DATA]")
                             << endl;
                    }
                    */
                    if (is_padding)
                    {
                        // 在 padding 區域，直接寫入 0
                        img_out.write(0.0);
                    }
                    else
                    {
                        // 在原圖區域，從 input FIFO 讀取一個像素，並寫入 output FIFO
                        double pixel = img_in.read();
                        img_out.write(pixel);
                    }
                    wait();
                }
            }
        }
        out_done.write(true);
        wait(); // 維持一個週期
    }
}