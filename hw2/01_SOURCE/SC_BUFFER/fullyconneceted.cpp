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
    // 動態初始化 sc_vector 大小
    img_in.init(FC_IN_SIZE);
    img_out.init(FC_OUT_SIZE);

    weight_buf.resize(FC_OUT_SIZE * FC_IN_SIZE);
    bias_buf.resize(FC_OUT_SIZE);

    load_weights();
    load_bias();

    // SC_THREAD，並加上非同步 Reset 設定
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
    // ===== Reset 初始化區域 =====
    out_valid.write(false);
    wait();
    if (rst.read() == 1)
    {
        out_valid.write(0);
        return;
    }

    // ===== SC_THREAD 無窮迴圈 =====
    while (true)
    {
        // 每輪開始前先確保完成訊號是低的
        out_valid.write(false);

        // 等待，直到上一層的 done 訊號傳進這層的 in_valid
        while (in_valid.read() == false)
        {
            wait();
        }

        // 開始進行 FC 運算
        for (int o = 0; o < FC_OUT_SIZE; o++)
        {
            double sum = bias_buf[o];

            for (int i = 0; i < FC_IN_SIZE; i++)
            {
                // 直接從 img_in 讀取，不透過 input_buf
                sum += weight_buf[weight_index(o, i)] * img_in[i].read();
            }

            // 算完單一 Neuron，直接寫出，不透過 output_buf
            img_out[o].write(sum);
        }

        // 運算結束：拉高完成訊號通知下一層
        out_valid.write(true);
        wait();
    }
}