#include "softmax.h"

// constructor 寫在.cpp
Softmax::Softmax(sc_module_name name, int size)
    : sc_module(name),
      SM_SIZE(size)
{
    input_buf.resize(SM_SIZE);

    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(rst, true);
}

void Softmax::run()
{
    // reset 初始化區域
    out_done.write(false);
    wait(); // 強制 Thread 在這裡掛起，直到 rst 變成 0 且下一個 Clock 來到

    while (true)
    {
        out_done.write(false);

        // 從上一層逐一讀取 1000 個數值
        for (int i = 0; i < SM_SIZE; i++)
        {
            input_buf[i] = img_in.read(); // FIFO 空了會自動等待
            wait();                       // 讀取花費 1 clock cycle
        }

        // Softmax
        double max_val = input_buf[0];
        for (int i = 1; i < SM_SIZE; i++)
        {
            if (input_buf[i] > max_val)
            {
                max_val = input_buf[i];
            }
        }

        double sum_exp = 0.0;
        for (int i = 0; i < SM_SIZE; i++)
        {
            sum_exp += exp(input_buf[i] - max_val);
        }

        for (int i = 0; i < SM_SIZE; i++)
        {
            double result = exp(input_buf[i] - max_val) / sum_exp;
            img_out.write(result);
            wait(); // 寫入花費 1 clock cycle
        }
        out_done.write(true);
        wait(); // 讓信號維持一格時脈
    }
}