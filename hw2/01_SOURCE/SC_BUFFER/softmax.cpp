#include "softmax.h"

Softmax::Softmax(sc_module_name name, int size)
    : sc_module(name),
      SM_SIZE(size)
{
    img_in.init(SM_SIZE);
    img_out.init(SM_SIZE);

    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(rst, true);
}

void Softmax::run()
{
    // ===== Reset 初始化區域 =====
    out_valid.write(false);
    wait();

    // ===== SC_THREAD 無窮迴圈 =====
    while (true)
    {
        // 每輪開始前先確保完成訊號為低電位
        out_valid.write(false);

        // 等待，直到上一層的訊號傳進這層的 in_valid
        while (in_valid.read() == false)
        {
            wait();
        }

        // 執行數值穩定 Softmax 運算

        // 找出最大值
        double max_val = img_in[0].read();
        for (int i = 1; i < SM_SIZE; i++)
        {
            double val = img_in[i].read();
            if (val > max_val)
            {
                max_val = val;
            }
        }

        // 計算指數與總和，這裡使用局部 vector 來存放計算中的 e^x
        vector<double> exp_temp(SM_SIZE);
        double sum_exp = 0.0;
        for (int i = 0; i < SM_SIZE; i++)
        {
            exp_temp[i] = exp(img_in[i].read() - max_val);
            sum_exp += exp_temp[i];
        }

        // 歸一化並直接寫出到 img_out
        for (int i = 0; i < SM_SIZE; i++)
        {
            img_out[i].write(exp_temp[i] / sum_exp);
        }

        // 運算結束：拉高完成訊號通知下一層 (Pattern)
        out_valid.write(true);
        wait();
    }
}