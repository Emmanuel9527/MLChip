#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "pe.h"
#include "systemc.h"
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

SC_MODULE(Controller)
{
    // Global clock and reset from the HW4 top-level testbench.
    sc_in<bool> rst;
    sc_in<bool> clk;

    // ROM command interface. The ROM itself is kept unchanged.
    sc_out<int> layer_id;
    sc_out<bool> layer_id_type;
    sc_out<bool> layer_id_valid;

    // ROM streaming data interface.
    sc_in<float> data;
    sc_in<bool> data_valid;

    // Local NoC interface. The controller is connected to router 0 as the
    // master node; PE workers are connected to routers 1 through 15.
    sc_out<sc_lv<34>> flit_tx;
    sc_out<bool> req_tx;
    sc_in<bool> ack_tx;

    sc_in<sc_lv<34>> flit_rx;
    sc_in<bool> req_rx;
    sc_out<bool> ack_rx;

    static const int IMG_H = 224;
    static const int IMG_W = 224;
    static const int IMG_C = 3;

    // AlexNet conv1 uses a 227x227 zero-padded image.
    static const int ZP_H = 227;
    static const int ZP_W = 227;

    // Router/core id 0 is reserved for the Controller. Worker PEs use ids 1..15.
    static const int FIRST_WORKER = 1;
    static const int LAST_WORKER = 15;
    static const int WORKER_COUNT = 15;
    static const int BROADCAST_WORKERS = 65535;

    // Monotonic id used to label jobs sent to PEs.
    int next_job_id;

    // Channel-major tensor addressing helper: tensor[c][h][w].
    static int idx3(int c, int h, int w, int height, int width)
    {
        return c * height * width + h * width + w;
    }

    // Assert the ROM request interface for one cycle.
    void request_rom(int id, bool type)
    {
        layer_id.write(id);
        layer_id_type.write(type);
        layer_id_valid.write(true);
        wait();
        layer_id_valid.write(false);
    }

    // Read one complete tensor/vector from ROM.
    // The ROM streams values while data_valid is high.
    vector<float> read_rom_vector(int id, bool type, int expected)
    {
        vector<float> values;
        values.reserve(expected);
        request_rom(id, type);

        bool started = false;
        while (true)
        {
            wait();
            if (data_valid.read())
            {
                started = true;
                values.push_back(data.read());
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

    // Send one flit into router[0] using valid-ready handshake.
    void send_flit(const sc_lv<34> &flit)
    {
        while (true)
        {
            flit_tx.write(flit);
            req_tx.write(true);
            wait();

            if (ack_tx.read())
            {
                req_tx.write(false);
                return;
            }
        }
    }

    // Serialize a packet into one HEAD flit and payload BODY/TAIL flits.
    void send_packet(const Packet &packet)
    {
        // HEAD flit layout: type=2, destination id, source id.
        sc_lv<34> header;
        header.range(33, 32) = 2;
        header.range(31, 16) = packet.dest_id;
        header.range(15, 0) = packet.source_id;
        send_flit(header);

        for (size_t i = 0; i < packet.datas.size(); i++)
        {
            // Payload flit layout: type in [33:32], raw float bits in [31:0].
            sc_lv<34> flit;
            flit.range(33, 32) = (i == packet.datas.size() - 1) ? 1 : 0;

            union
            {
                float fval;
                unsigned int ival;
            } converter;

            converter.fval = packet.datas[i];
            flit.range(31, 0) = converter.ival;
            send_flit(flit);
        }
    }

    // Block until one complete packet returns from a worker PE.
    Packet receive_packet()
    {
        Packet packet;
        bool active = false;

        while (true)
        {
            ack_rx.write(true);
            wait();

            if (!req_rx.read())
                continue;

            sc_lv<34> flit = flit_rx.read();
            int type = flit.range(33, 32).to_uint();

            if (type == 2)
            {
                // HEAD starts a new return packet.
                packet = Packet();
                packet.dest_id = flit.range(31, 16).to_uint();
                packet.source_id = flit.range(15, 0).to_uint();
                active = true;
            }
            else if (active && (type == 0 || type == 1))
            {
                // BODY/TAIL payloads are reconstructed from raw float bits.
                union
                {
                    float fval;
                    unsigned int ival;
                } converter;

                converter.ival = flit.range(31, 0).to_uint();
                packet.datas.push_back(converter.fval);

                if (type == 1)
                {
                    return packet;
                }
            }
        }
    }

    // Initial image padding before conv1.
    vector<float> zero_pad_224_to_227(const vector<float> &input)
    {
        vector<float> output(ZP_H * ZP_W * IMG_C, 0.0f);
        for (int c = 0; c < IMG_C; c++)
            for (int h = 0; h < IMG_H; h++)
                for (int w = 0; w < IMG_W; w++)
                    output[idx3(c, h + 2, w + 2, ZP_H, ZP_W)] =
                        input[idx3(c, h, w, IMG_H, IMG_W)];
        return output;
    }

    // Generic symmetric padding used before conv2..conv5.
    vector<float> pad_layer(const vector<float> &input, int in_h, int in_w, int ch, int pad)
    {
        int out_h = in_h + 2 * pad;
        int out_w = in_w + 2 * pad;
        vector<float> output(out_h * out_w * ch, 0.0f);

        for (int c = 0; c < ch; c++)
            for (int h = 0; h < in_h; h++)
                for (int w = 0; w < in_w; w++)
                    output[idx3(c, h + pad, w + pad, out_h, out_w)] =
                        input[idx3(c, h, w, in_h, in_w)];
        return output;
    }

    // Extract the weight rows needed by one PE's output-channel slice.
    vector<float> slice_conv_weight(const vector<float> &weight,
                                    int oc_start, int oc_count,
                                    int in_ch, int kernel)
    {
        int per_oc = in_ch * kernel * kernel;
        vector<float> part(oc_count * per_oc);
        for (int oc = 0; oc < oc_count; oc++)
            for (int i = 0; i < per_oc; i++)
                part[oc * per_oc + i] = weight[(oc_start + oc) * per_oc + i];
        return part;
    }

    // Extract the FC matrix rows needed by one PE's output-neuron slice.
    vector<float> slice_fc_weight(const vector<float> &weight,
                                  int o_start, int o_count,
                                  int in_size)
    {
        vector<float> part(o_count * in_size);
        for (int o = 0; o < o_count; o++)
            for (int i = 0; i < in_size; i++)
                part[o * in_size + i] = weight[(o_start + o) * in_size + i];
        return part;
    }

    // Append a tensor/vector payload to a packet data vector.
    void append_vector(vector<float> & dst, const vector<float> &src)
    {
        dst.insert(dst.end(), src.begin(), src.end());
    }

    // Build a PE local-buffer load packet.
    // Layout: [op, layer_id, payload_size, payload...]
    Packet make_load_packet(int dest, int op, int layer, const vector<float> &payload)
    {
        Packet packet;
        packet.source_id = 0;
        packet.dest_id = dest;
        packet.datas.push_back((float)op);
        packet.datas.push_back((float)layer);
        packet.datas.push_back((float)payload.size());
        append_vector(packet.datas, payload);
        return packet;
    }

    // Broadcast one activation/input tensor to every worker PE.
    // The router implements this as a spanning-tree multicast, so Controller
    // injects one packet instead of unicasting the same tensor to each PE.
    void broadcast_input_to_workers(int layer, const vector<float> &payload)
    {
        send_packet(make_load_packet(BROADCAST_WORKERS, OP_LOAD_INPUT, layer, payload));

        // The broadcast has no response packet. Wait a small number of cycles
        // for the tail flit to drain through the 4x4 tree before compute starts.
        for (int i = 0; i < 64; i++)
            wait();
    }

    // Build a small convolution compute command.
    // Input/weight/bias data are reused from PE local buffers.
    Packet make_conv_compute(int dest, int job_id,
                             int in_h, int in_w, int in_ch,
                             int out_ch, int kernel, int stride,
                             int oc_start, int oc_count)
    {
        Packet packet;
        packet.source_id = 0;
        packet.dest_id = dest;
        packet.datas.push_back((float)OP_COMPUTE_CONV);
        packet.datas.push_back((float)job_id);
        packet.datas.push_back(1.0f);
        packet.datas.push_back((float)in_h);
        packet.datas.push_back((float)in_w);
        packet.datas.push_back((float)in_ch);
        packet.datas.push_back((float)out_ch);
        packet.datas.push_back((float)kernel);
        packet.datas.push_back((float)stride);
        packet.datas.push_back((float)oc_start);
        packet.datas.push_back((float)oc_count);
        return packet;
    }

    // Build a small pooling compute command.
    Packet make_pool_compute(int dest, int job_id,
                             int in_h, int in_w, int ch,
                             int kernel, int stride,
                             int c_start, int c_count)
    {
        Packet packet;
        packet.source_id = 0;
        packet.dest_id = dest;
        packet.datas.push_back((float)OP_COMPUTE_POOL);
        packet.datas.push_back((float)job_id);
        packet.datas.push_back((float)in_h);
        packet.datas.push_back((float)in_w);
        packet.datas.push_back((float)ch);
        packet.datas.push_back((float)kernel);
        packet.datas.push_back((float)stride);
        packet.datas.push_back((float)c_start);
        packet.datas.push_back((float)c_count);
        return packet;
    }

    // Build a small FC compute command.
    Packet make_fc_compute(int dest, int job_id,
                           int in_size, int out_size,
                           int o_start, int o_count,
                           bool apply_relu)
    {
        Packet packet;
        packet.source_id = 0;
        packet.dest_id = dest;
        packet.datas.push_back((float)OP_COMPUTE_FC);
        packet.datas.push_back((float)job_id);
        packet.datas.push_back(apply_relu ? 1.0f : 0.0f);
        packet.datas.push_back((float)in_size);
        packet.datas.push_back((float)out_size);
        packet.datas.push_back((float)o_start);
        packet.datas.push_back((float)o_count);
        return packet;
    }

    // Merge a PE result packet that contains one contiguous channel slice.
    void merge_channel_result(vector<float> & output, const Packet &result)
    {
        int p = 0;
        int job_id = (int)result.datas[p++];
        int start = (int)result.datas[p++];
        int count = (int)result.datas[p++];
        int out_h = (int)result.datas[p++];
        int out_w = (int)result.datas[p++];

        for (int local = 0; local < count; local++)
            for (int i = 0; i < out_h * out_w; i++)
                output[(start + local) * out_h * out_w + i] = result.datas[p++];

        (void)job_id;
    }

    // Run one convolution layer through worker PEs.
    // The Controller first preloads each PE's local input/weight/bias buffers,
    // then dispatches all compute commands before collecting output slices.
    vector<float> run_conv_on_pes(vector<float> & feature,
                                  int layer,
                                  int in_h, int in_w, int in_ch,
                                  int out_ch, int kernel, int stride)
    {
        int weight_count = out_ch * in_ch * kernel * kernel;
        vector<float> weight = read_rom_vector(layer, false, weight_count);
        vector<float> bias = read_rom_vector(layer, true, out_ch);
        int out_h = (in_h - kernel) / stride + 1;
        int out_w = (in_w - kernel) / stride + 1;
        vector<float> output(out_h * out_w * out_ch, 0.0f);

        vector<int> workers;
        vector<int> starts;
        vector<int> counts;

        broadcast_input_to_workers(layer, feature);

        int base = 0;
        for (int worker = FIRST_WORKER; worker <= LAST_WORKER && base < out_ch; worker++)
        {
            int remaining_workers = LAST_WORKER - worker + 1;
            int oc_count = (out_ch - base + remaining_workers - 1) / remaining_workers;
            vector<float> weight_part = slice_conv_weight(weight, base, oc_count, in_ch, kernel);
            vector<float> bias_part(bias.begin() + base, bias.begin() + base + oc_count);

            send_packet(make_load_packet(worker, OP_LOAD_WEIGHT, layer, weight_part));
            send_packet(make_load_packet(worker, OP_LOAD_BIAS, layer, bias_part));

            workers.push_back(worker);
            starts.push_back(base);
            counts.push_back(oc_count);
            base += oc_count;
        }

        for (size_t i = 0; i < workers.size(); i++)
        {
            int job_id = next_job_id++;
            Packet packet = make_conv_compute(workers[i], job_id,
                                              in_h, in_w, in_ch, out_ch, kernel, stride,
                                              starts[i], counts[i]);
            send_packet(packet);
        }

        for (size_t i = 0; i < workers.size(); i++)
        {
            Packet result = receive_packet();
            merge_channel_result(output, result);
        }

        return output;
    }

    // Run one max-pooling layer through worker PEs.
    // Pooling has no weight/bias, so only input is preloaded.
    vector<float> run_pool_on_pes(const vector<float> &feature,
                                  int in_h, int in_w, int ch,
                                  int kernel, int stride)
    {
        int out_h = (in_h - kernel) / stride + 1;
        int out_w = (in_w - kernel) / stride + 1;
        vector<float> output(out_h * out_w * ch, 0.0f);

        vector<int> workers;
        vector<int> starts;
        vector<int> counts;

        broadcast_input_to_workers(0, feature);

        int base = 0;
        for (int worker = FIRST_WORKER; worker <= LAST_WORKER && base < ch; worker++)
        {
            int remaining_workers = LAST_WORKER - worker + 1;
            int c_count = (ch - base + remaining_workers - 1) / remaining_workers;

            workers.push_back(worker);
            starts.push_back(base);
            counts.push_back(c_count);
            base += c_count;
        }

        for (size_t i = 0; i < workers.size(); i++)
        {
            int job_id = next_job_id++;
            Packet packet = make_pool_compute(workers[i], job_id, in_h, in_w, ch,
                                              kernel, stride, starts[i], counts[i]);
            send_packet(packet);
        }

        for (size_t i = 0; i < workers.size(); i++)
        {
            Packet result = receive_packet();
            merge_channel_result(output, result);
        }

        return output;
    }

    // Run one fully connected layer through worker PEs.
    vector<float> run_fc_on_pes(const vector<float> &feature,
                                int layer, int out_size, bool apply_relu)
    {
        int in_size = (int)feature.size();
        vector<float> weight = read_rom_vector(layer, false, out_size * in_size);
        vector<float> bias = read_rom_vector(layer, true, out_size);
        vector<float> output(out_size, 0.0f);

        vector<int> workers;
        vector<int> starts;
        vector<int> counts;

        broadcast_input_to_workers(layer, feature);

        int base = 0;
        for (int worker = FIRST_WORKER; worker <= LAST_WORKER && base < out_size; worker++)
        {
            int remaining_workers = LAST_WORKER - worker + 1;
            int o_count = (out_size - base + remaining_workers - 1) / remaining_workers;
            vector<float> weight_part = slice_fc_weight(weight, base, o_count, in_size);
            vector<float> bias_part(bias.begin() + base, bias.begin() + base + o_count);

            send_packet(make_load_packet(worker, OP_LOAD_WEIGHT, layer, weight_part));
            send_packet(make_load_packet(worker, OP_LOAD_BIAS, layer, bias_part));

            workers.push_back(worker);
            starts.push_back(base);
            counts.push_back(o_count);
            base += o_count;
        }

        for (size_t i = 0; i < workers.size(); i++)
        {
            int job_id = next_job_id++;
            Packet packet = make_fc_compute(workers[i], job_id, in_size, out_size,
                                            starts[i], counts[i], apply_relu);
            send_packet(packet);
        }

        for (size_t i = 0; i < workers.size(); i++)
        {
            Packet result = receive_packet();
            int p = 0;
            int result_job_id = (int)result.datas[p++];
            int start = (int)result.datas[p++];
            int count = (int)result.datas[p++];
            p += 2;
            for (int j = 0; j < count; j++)
                output[start + j] = result.datas[p++];
            (void)result_job_id;
        }

        return output;
    }

    // Final softmax is kept in Controller because it is a small output-format
    // step after all PE-computed fc8 scores have been collected.
    vector<double> softmax(const vector<float> &input)
    {
        vector<double> output(input.size(), 0.0);
        double max_val = input[0];
        for (size_t i = 1; i < input.size(); i++)
            max_val = max(max_val, (double)input[i]);

        double sum = 0.0;
        for (size_t i = 0; i < input.size(); i++)
        {
            output[i] = exp((double)input[i] - max_val);
            sum += output[i];
        }
        for (size_t i = 0; i < output.size(); i++)
            output[i] /= sum;
        return output;
    }

    // Load ImageNet class names for output formatting.
    vector<string> read_class_names()
    {
        vector<string> names;
        ifstream fin("data/imagenet_classes.txt");
        string line;
        while (getline(fin, line))
            names.push_back(line);
        return names;
    }

    // Sort helper for descending probability.
    static bool prob_greater(const pair<double, int> &a, const pair<double, int> &b)
    {
        return a.first > b.first;
    }

    // Print the final Top-100 table in the same format as the assignment output.
    void print_top100(const vector<float> &linear, const vector<double> &prob)
    {
        vector<pair<double, int>> order;
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

    // Main Controller schedule:
    // read ROM tensors, partition each layer into PE jobs, collect results,
    // and advance through the AlexNet layer order.
    void run()
    {
        next_job_id = 1;
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

        // Input image -> zero padding.
        vector<float> feature = read_rom_vector(0, false, IMG_H * IMG_W * IMG_C);
        feature = zero_pad_224_to_227(feature);

        // conv1 -> ReLU in PE -> pool1 in PE.
        feature = run_conv_on_pes(feature, 1, 227, 227, 3, 64, 11, 4);
        feature = run_pool_on_pes(feature, 55, 55, 64, 3, 2);

        // conv2 block: padding in Controller, conv/ReLU/pool in PEs.
        feature = pad_layer(feature, 27, 27, 64, 2);
        feature = run_conv_on_pes(feature, 2, 31, 31, 64, 192, 5, 1);
        feature = run_pool_on_pes(feature, 27, 27, 192, 3, 2);

        // conv3 block.
        feature = pad_layer(feature, 13, 13, 192, 1);
        feature = run_conv_on_pes(feature, 3, 15, 15, 192, 384, 3, 1);

        // conv4 block.
        feature = pad_layer(feature, 13, 13, 384, 1);
        feature = run_conv_on_pes(feature, 4, 15, 15, 384, 256, 3, 1);

        // conv5 block and final convolutional pooling.
        feature = pad_layer(feature, 13, 13, 256, 1);
        feature = run_conv_on_pes(feature, 5, 15, 15, 256, 256, 3, 1);
        feature = run_pool_on_pes(feature, 13, 13, 256, 3, 2);

        // Fully connected classifier.
        feature = run_fc_on_pes(feature, 6, 4096, true);
        feature = run_fc_on_pes(feature, 7, 4096, true);
        feature = run_fc_on_pes(feature, 8, 1000, false);

        // Output conversion and report.
        vector<double> prob = softmax(feature);
        print_top100(feature, prob);
        sc_stop();
    }

    // Register the Controller as one clocked SystemC thread.
    SC_CTOR(Controller)
    {
        SC_THREAD(run);
        sensitive << clk.pos();
    }
};

#endif
