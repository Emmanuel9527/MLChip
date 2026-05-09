#include "relu.h"

ReLU::ReLU(sc_module_name name, int size)
    : sc_module(name),
      RL_SIZE(size)
{

    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(rst, true);
}

void ReLU::run()
{
    // reset 初始化區域
    out_done.write(false);
    wait(); // 強制 Thread 在這裡掛起，直到 rst 變成 0 且下一個 Clock 來到
    while (true)
    {
        out_done.write(false);

        for (int i = 0; i < RL_SIZE; i++)
        {
            // 讀取一個像素，若沒資料會自動 Block 等待
            double val = img_in.read();

            // 運算 ReLU
            if (val > 0.0)
            {
                img_out.write(val); // 大於 0，原值輸出
            }
            else
            {
                img_out.write(0.0); // 小於等於 0，輸出 0
            }

            // 等待一個 clock cycle
            wait();
        }
        out_done.write(true);
        wait(); // 保持一個週期讓下一層或 Testbench 偵測
    }
}