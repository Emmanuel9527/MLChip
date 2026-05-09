#include "ZeroPadding.h"

void ZeroPadding::run()
{
    // reset 初始化區域
    out_valid.write(false);
    wait(); // 強制 Thread 在這裡掛起，直到 rst 變成 0 且下一個 Clock 來到

    // SC_THREAD 需要一個無窮迴圈來持續等待與處理資料
    while (true)
    {
        out_valid.write(false);

        // 等待資料都就位
        while (in_valid.read() == false)
            wait();

        int in_idx = 0;
        int out_idx = 0;

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

                    if (is_padding)
                    {
                        // 在 padding 區域，對應的輸出位置寫入 0
                        img_out[out_idx].write(0.0);
                    }
                    else
                    {
                        // 在原圖區域，讀取當前 in_idx 的像素，並寫入 out_idx
                        double pixel = img_in[in_idx].read();
                        img_out[out_idx].write(pixel);

                        // 讀完一個原圖像素後，輸入的指標才往前推進 (等同於原本的 FIFO read)
                        in_idx++;
                    }

                    // 無論是補零還是原圖，輸出的指標永遠跟著迴圈一起推進
                    out_idx++;
                }
            }
        }

        // 整張圖片都寫入 Output Buffer 陣列後，發出完成訊號
        out_valid.write(true);
        wait(); // 維持一個週期，讓下一層偵測到 out_valid 為 true 並開始動作
    }
}