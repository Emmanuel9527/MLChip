#include "fullyconneceted.h"

FullyConnected::FullyConnected(sc_module_name name,
                               int in_size,
                               int out_size,
                               const string &w_file,
                               const string &b_file)
    : sc_module(name),
      FC_IN_SIZE(in_size),
      FC_OUT_SIZE(out_size),
      weight_file(w_file),
      bias_file(b_file)
{

    input_buf.resize(FC_IN_SIZE);
    weight_buf.resize(FC_OUT_SIZE * FC_IN_SIZE);
    bias_buf.resize(FC_OUT_SIZE);

    load_weights();
    load_bias();

    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(rst, true);
}

void FullyConnected::load_weights()
{
    ifstream fin(weight_file);
    if (!fin.is_open())
    {
        cerr << "[FullyConnected] Cannot open weight file: " << weight_file << endl;
        exit(1);
    }

    for (int i = 0; i < (int)weight_buf.size(); i++)
    {
        if (!(fin >> weight_buf[i]))
        {
            cerr << "[FullyConnected] Weight format error at index " << i << endl;
            exit(1);
        }
    }

    fin.close();
}

void FullyConnected::load_bias()
{
    ifstream fin(bias_file);
    if (!fin.is_open())
    {
        cerr << "[FullyConnected] Cannot open bias file: " << bias_file << endl;
        exit(1);
    }

    for (int i = 0; i < (int)bias_buf.size(); i++)
    {
        if (!(fin >> bias_buf[i]))
        {
            cerr << "[FullyConnected] Bias format error at index " << i << endl;
            exit(1);
        }
    }

    fin.close();
}

void FullyConnected::run()
{
    // reset 初始化區域
    out_done.write(false);
    wait(); // 強制 Thread 在這裡掛起，直到 rst 變成 0 且下一個 Clock 來到

    while (true)
    {
        out_done.write(false); // 開始新的一張圖

        // 收集完整的 Input 特徵向量
        // 因為 FC 層的每個輸出都需要全部的輸入，所以必須先讀完存起來
        for (int i = 0; i < FC_IN_SIZE; i++)
        {
            input_buf[i] = img_in.read();
            wait(); // 每讀取一個數值，花費 1 clock cycle
        }

        // 執行計算並即時輸出
        for (int o = 0; o < FC_OUT_SIZE; o++)
        {
            double sum = bias_buf[o];

            // 執行 MAC 運算
            for (int i = 0; i < FC_IN_SIZE; i++)
            {
                sum += weight_buf[weight_index(o, i)] * input_buf[i];
            }

            // 算完後，立刻丟進 FIFO 給下一層 (ReLU 或 Softmax)
            img_out.write(sum);
            wait(); // 每輸出一個數值，花費 1 clock cycle
        }
        out_done.write(true);
        wait(); // 讓信號維持一個週期
    }
}