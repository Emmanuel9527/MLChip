#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "systemc.h"
#include "pe.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

using namespace std;

SC_MODULE( Controller ) {
    sc_in  < bool >  rst;
    sc_in  < bool >  clk;

    sc_out < int >   layer_id;
    sc_out < bool >  layer_id_type;
    sc_out < bool >  layer_id_valid;

    sc_in  < float > data;
    sc_in  < bool >  data_valid;

    sc_out < sc_lv<34> > flit_tx;
    sc_out < bool > req_tx;
    sc_in  < bool > ack_tx;

    sc_in  < sc_lv<34> > flit_rx;
    sc_in  < bool > req_rx;
    sc_out < bool > ack_rx;

    static const int IMG_H = 224;
    static const int IMG_W = 224;
    static const int IMG_C = 3;

    static const int ZP_H = 227;
    static const int ZP_W = 227;

    static int idx3(int c, int h, int w, int height, int width)
    {
        return c * height * width + h * width + w;
    }

    void request_rom(int id, bool type)
    {
        layer_id.write(id);
        layer_id_type.write(type);
        layer_id_valid.write(true);
        wait();
        layer_id_valid.write(false);
    }

    vector<double> read_rom_vector(int id, bool type, int expected)
    {
        vector<double> values;
        values.reserve(expected);
        request_rom(id, type);

        bool started = false;
        while (true)
        {
            wait();
            if (data_valid.read())
            {
                started = true;
                values.push_back((double)data.read());
            }
            else if (started)
            {
                break;
            }
        }

        if ((int)values.size() != expected)
        {
            cout << "ROM size mismatch at layer " << id << "." << endl;
            cout << "Expected " << expected << ", got " << values.size() << "." << endl;
            sc_stop();
        }
        return values;
    }

    vector<double> zero_pad_224_to_227(const vector<double> &input)
    {
        vector<double> output(ZP_H * ZP_W * IMG_C, 0.0);
        for (int c = 0; c < IMG_C; c++)
            for (int h = 0; h < IMG_H; h++)
                for (int w = 0; w < IMG_W; w++)
                    output[idx3(c, h + 2, w + 2, ZP_H, ZP_W)] =
                        input[idx3(c, h, w, IMG_H, IMG_W)];
        return output;
    }

    vector<double> pad_layer(const vector<double> &input, int in_h, int in_w, int ch, int pad)
    {
        int out_h = in_h + 2 * pad;
        int out_w = in_w + 2 * pad;
        vector<double> output(out_h * out_w * ch, 0.0);

        for (int c = 0; c < ch; c++)
            for (int h = 0; h < in_h; h++)
                for (int w = 0; w < in_w; w++)
                    output[idx3(c, h + pad, w + pad, out_h, out_w)] =
                        input[idx3(c, h, w, in_h, in_w)];
        return output;
    }

    vector<double> conv_layer(const vector<double> &input,
                              const vector<double> &weight,
                              const vector<double> &bias,
                              int in_h, int in_w, int in_ch,
                              int out_ch, int kernel, int stride)
    {
        int out_h = (in_h - kernel) / stride + 1;
        int out_w = (in_w - kernel) / stride + 1;
        vector<double> output(out_h * out_w * out_ch, 0.0);

        for (int oc = 0; oc < out_ch; oc++)
        {
            int out_base = oc * out_h * out_w;
            int wt_oc_base = oc * in_ch * kernel * kernel;
            for (int oh = 0; oh < out_h; oh++)
            {
                for (int ow = 0; ow < out_w; ow++)
                {
                    double sum = bias[oc];
                    for (int ic = 0; ic < in_ch; ic++)
                    {
                        int in_base = ic * in_h * in_w;
                        int wt_ic_base = wt_oc_base + ic * kernel * kernel;
                        int ih_base = oh * stride;
                        int iw_base = ow * stride;
                        for (int kh = 0; kh < kernel; kh++)
                        {
                            int in_row = in_base + (ih_base + kh) * in_w;
                            int wt_row = wt_ic_base + kh * kernel;
                            for (int kw = 0; kw < kernel; kw++)
                                sum += input[in_row + iw_base + kw] * weight[wt_row + kw];
                        }
                    }
                    output[out_base + oh * out_w + ow] = sum;
                }
            }
        }
        return output;
    }

    void relu_inplace(vector<double> &values)
    {
        for (size_t i = 0; i < values.size(); i++)
            if (values[i] < 0.0)
                values[i] = 0.0;
    }

    vector<double> max_pool(const vector<double> &input, int in_h, int in_w, int ch, int kernel, int stride)
    {
        int out_h = (in_h - kernel) / stride + 1;
        int out_w = (in_w - kernel) / stride + 1;
        vector<double> output(out_h * out_w * ch, 0.0);

        for (int c = 0; c < ch; c++)
            for (int oh = 0; oh < out_h; oh++)
                for (int ow = 0; ow < out_w; ow++)
                {
                    int start_h = oh * stride;
                    int start_w = ow * stride;
                    double best = input[idx3(c, start_h, start_w, in_h, in_w)];
                    for (int kh = 0; kh < kernel; kh++)
                        for (int kw = 0; kw < kernel; kw++)
                            best = max(best, input[idx3(c, start_h + kh, start_w + kw, in_h, in_w)]);
                    output[idx3(c, oh, ow, out_h, out_w)] = best;
                }
        return output;
    }

    vector<double> fc_layer(const vector<double> &input,
                            const vector<double> &weight,
                            const vector<double> &bias,
                            int out_size)
    {
        int in_size = (int)input.size();
        vector<double> output(out_size, 0.0);

        for (int o = 0; o < out_size; o++)
        {
            const double *w = &weight[o * in_size];
            double sum = bias[o];
            for (int i = 0; i < in_size; i++)
                sum += w[i] * input[i];
            output[o] = sum;
        }
        return output;
    }

    vector<double> softmax(const vector<double> &input)
    {
        vector<double> output(input.size(), 0.0);
        double max_val = *max_element(input.begin(), input.end());
        double sum = 0.0;

        for (size_t i = 0; i < input.size(); i++)
        {
            output[i] = exp(input[i] - max_val);
            sum += output[i];
        }
        for (size_t i = 0; i < output.size(); i++)
            output[i] /= sum;
        return output;
    }

    void load_and_run_conv(vector<double> &feature,
                           int layer,
                           int in_h, int in_w, int in_ch,
                           int out_ch, int kernel, int stride)
    {
        int weight_count = out_ch * in_ch * kernel * kernel;
        vector<double> weight = read_rom_vector(layer, false, weight_count);
        vector<double> bias = read_rom_vector(layer, true, out_ch);
        feature = conv_layer(feature, weight, bias, in_h, in_w, in_ch, out_ch, kernel, stride);
        relu_inplace(feature);
    }

    void load_and_run_fc(vector<double> &feature, int layer, int out_size, bool apply_relu)
    {
        int weight_count = out_size * (int)feature.size();
        vector<double> weight = read_rom_vector(layer, false, weight_count);
        vector<double> bias = read_rom_vector(layer, true, out_size);
        feature = fc_layer(feature, weight, bias, out_size);
        if (apply_relu)
            relu_inplace(feature);
    }

    vector<string> read_class_names()
    {
        vector<string> names;
        ifstream fin("data/imagenet_classes.txt");
        string line;
        while (getline(fin, line))
            names.push_back(line);
        return names;
    }

    static bool prob_greater(const pair<double, int> &a, const pair<double, int> &b)
    {
        return a.first > b.first;
    }

    void print_top100(const vector<double> &linear, const vector<double> &prob)
    {
        vector<pair<double, int> > order;
        for (int i = 0; i < 1000; i++)
            order.push_back(make_pair(prob[i], i));

        sort(order.begin(), order.end(), prob_greater);

        vector<string> names = read_class_names();

        cout << "Top 100 classes:" << endl;
        cout << "=================================================" << endl;
        cout << fixed << setprecision(2);
        cout << right << setw(5) << "idx"
             << " | " << setw(8) << "val"
             << " | " << setw(11) << "possibility"
             << " | " << "class name" << endl;
        cout << "-------------------------------------------------" << endl;

        for (int i = 0; i < 100; i++)
        {
            int index = order[i].second;
            string name = (index < (int)names.size()) ? names[index] : "";
            cout << right << setw(5) << index
                 << " | " << setw(8) << linear[index]
                 << " | " << setw(11) << (order[i].first * 100.0)
                 << " | " << name << endl;
        }
        cout << "=================================================" << endl;
    }

    void run()
    {
        layer_id.write(0);
        layer_id_type.write(false);
        layer_id_valid.write(false);
        req_tx.write(false);
        ack_rx.write(true);
        flit_tx.write(0);

        wait();
        while (rst.read())
            wait();
        wait();

        vector<double> feature = read_rom_vector(0, false, IMG_H * IMG_W * IMG_C);
        feature = zero_pad_224_to_227(feature);

        load_and_run_conv(feature, 1, 227, 227, 3, 64, 11, 4);
        feature = max_pool(feature, 55, 55, 64, 3, 2);

        feature = pad_layer(feature, 27, 27, 64, 2);
        load_and_run_conv(feature, 2, 31, 31, 64, 192, 5, 1);
        feature = max_pool(feature, 27, 27, 192, 3, 2);

        feature = pad_layer(feature, 13, 13, 192, 1);
        load_and_run_conv(feature, 3, 15, 15, 192, 384, 3, 1);

        feature = pad_layer(feature, 13, 13, 384, 1);
        load_and_run_conv(feature, 4, 15, 15, 384, 256, 3, 1);

        feature = pad_layer(feature, 13, 13, 256, 1);
        load_and_run_conv(feature, 5, 15, 15, 256, 256, 3, 1);
        feature = max_pool(feature, 13, 13, 256, 3, 2);

        load_and_run_fc(feature, 6, 4096, true);
        load_and_run_fc(feature, 7, 4096, true);
        load_and_run_fc(feature, 8, 1000, false);

        vector<double> linear = feature;
        vector<double> prob = softmax(linear);
        print_top100(linear, prob);
        sc_stop();
    }

    SC_CTOR( Controller )
    {
        SC_THREAD( run );
        sensitive << clk.pos();
    }
};

#endif
