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
    img_in.init(FC_IN_SIZE);
    img_out.init(FC_OUT_SIZE);

    input_buf.resize(FC_IN_SIZE);
    weight_buf.resize(FC_OUT_SIZE * FC_IN_SIZE);
    bias_buf.resize(FC_OUT_SIZE);
    output_buf.resize(FC_OUT_SIZE);

    load_weights();
    load_bias();

    SC_METHOD(run);
    sensitive << clk.pos();
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
    if (rst.read() == 1)
    {
        out_valid.write(0);
        return;
    }

    if (in_valid.read() == 1)
    {
        // ===== 1. read input =====
        for (int i = 0; i < FC_IN_SIZE; i++)
        {
            input_buf[i] = img_in[i].read();
        }

        // ===== 2. FC =====
        for (int o = 0; o < FC_OUT_SIZE; o++)
        {
            double sum = bias_buf[o];

            for (int i = 0; i < FC_IN_SIZE; i++)
            {
                sum += weight_buf[weight_index(o, i)] * input_buf[i];
            }

            output_buf[o] = sum;
        }

        // ===== 3. write output =====
        for (int i = 0; i < FC_OUT_SIZE; i++)
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