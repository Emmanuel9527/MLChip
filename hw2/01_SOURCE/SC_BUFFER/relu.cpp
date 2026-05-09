#include "relu.h"

ReLU::ReLU(sc_module_name name, int size)
    : sc_module(name),
      RL_SIZE(size)
{
    img_in.init(RL_SIZE);
    img_out.init(RL_SIZE);

    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(rst, true);
}

void ReLU::run()
{
    // ===== Reset 初始化區域 =====
    out_valid.write(false);
    wait();

    // ===== SC_THREAD 無窮迴圈 =====
    while (true)
    {
        // 每輪開始前先確保完成訊號為低電位
        out_valid.write(false);

        // 等待，直到上一層的 done 訊號傳進這層的 in_valid
        while (in_valid.read() == false)
        {
            wait();
        }

        // 執行 ReLU 運算：max(0, x)
        for (int i = 0; i < RL_SIZE; i++)
        {
            // 直接從 img_in 讀取並判斷輸出，不透過中間緩衝區
            double val = img_in[i].read();
            if (val > 0.0)
            {
                img_out[i].write(val);
            }
            else
            {
                img_out[i].write(0.0);
            }
        }

        // 運算結束：拉高完成訊號通知下一層
        out_valid.write(true);
        wait();
    }
}
