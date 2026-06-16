#ifndef PE_H
#define PE_H

#include "systemc.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>
#include <vector>

using namespace std;

// Packet is the transaction format carried by the Core network interface.
// The router only uses source_id/dest_id in the HEAD flit; operation metadata
// is encoded at the front of datas by the Controller.
struct Packet
{
    int source_id;
    int dest_id;
    vector<float> datas;
};

// Operation codes used by the Controller to select PE actions.
enum PeOp
{
    OP_LOAD_INPUT = 1,
    OP_LOAD_WEIGHT = 2,
    OP_LOAD_BIAS = 3,
    OP_COMPUTE_CONV = 4,
    OP_COMPUTE_POOL = 5,
    OP_COMPUTE_FC = 6,
    OP_LOAD_ACK = 7
};

SC_MODULE(PE)
{
    static const int MACS_PER_CYCLE = 64;

    static unsigned long long &metric_mac_ops()
    {
        static unsigned long long value = 0;
        return value;
    }

    static unsigned long long &metric_compute_cycles()
    {
        static unsigned long long value = 0;
        return value;
    }

    static unsigned long long &metric_compute_jobs()
    {
        static unsigned long long value = 0;
        return value;
    }

    static unsigned long long &metric_fc_mac_ops()
    {
        static unsigned long long value = 0;
        return value;
    }

    static unsigned long long &metric_fc_compute_cycles()
    {
        static unsigned long long value = 0;
        return value;
    }

    static unsigned long long &metric_fc_compute_jobs()
    {
        static unsigned long long value = 0;
        return value;
    }

    static unsigned long long &metric_peak_input_words()
    {
        static unsigned long long value = 0;
        return value;
    }

    static unsigned long long &metric_peak_weight_words()
    {
        static unsigned long long value = 0;
        return value;
    }

    static unsigned long long &metric_peak_bias_words()
    {
        static unsigned long long value = 0;
        return value;
    }

    static void reset_metrics()
    {
        metric_mac_ops() = 0;
        metric_compute_cycles() = 0;
        metric_compute_jobs() = 0;
        metric_fc_mac_ops() = 0;
        metric_fc_compute_cycles() = 0;
        metric_fc_compute_jobs() = 0;
        metric_peak_input_words() = 0;
        metric_peak_weight_words() = 0;
        metric_peak_bias_words() = 0;
    }

    static void record_macs(long long mac_count, bool is_fc)
    {
        unsigned long long cycles =
            (unsigned long long)((mac_count + MACS_PER_CYCLE - 1) /
                                 MACS_PER_CYCLE);
        metric_mac_ops() += (unsigned long long)mac_count;
        metric_compute_cycles() += cycles;
        metric_compute_jobs()++;
        if (is_fc)
        {
            metric_fc_mac_ops() += (unsigned long long)mac_count;
            metric_fc_compute_cycles() += cycles;
            metric_fc_compute_jobs()++;
        }
    }

    static unsigned long long total_mac_ops()
    {
        return metric_mac_ops();
    }

    static unsigned long long total_compute_cycles()
    {
        return metric_compute_cycles();
    }

    static unsigned long long total_compute_jobs()
    {
        return metric_compute_jobs();
    }

    static unsigned long long total_fc_mac_ops()
    {
        return metric_fc_mac_ops();
    }

    static unsigned long long total_fc_compute_cycles()
    {
        return metric_fc_compute_cycles();
    }

    static unsigned long long total_fc_compute_jobs()
    {
        return metric_fc_compute_jobs();
    }

    static unsigned long long peak_input_words()
    {
        return metric_peak_input_words();
    }

    static unsigned long long peak_weight_words()
    {
        return metric_peak_weight_words();
    }

    static unsigned long long peak_bias_words()
    {
        return metric_peak_bias_words();
    }

    // PE id matches the local router/core id in the 4x4 mesh.
    int id;

    // Local buffers model the storage inside each PE. The Controller preloads
    // these buffers before sending a small compute command.
    vector<float> input_buf;
    vector<float> weight_buf;
    vector<float> bias_buf;

    // Channel-major tensor addressing helper: tensor[c][h][w].
    static int idx3(int c, int h, int w, int height, int width)
    {
        return c * height * width + h * width + w;
    }

    void wait_for_macs(long long mac_count, bool is_fc = false)
    {
        long long cycles = (mac_count + MACS_PER_CYCLE - 1) / MACS_PER_CYCLE;
        record_macs(mac_count, is_fc);
        for (long long i = 0; i < cycles; i++)
            wait();
    }

    // Called by main.cpp after construction so this PE knows its mesh id.
    void init(int pe_id)
    {
        id = pe_id;
    }

    // Top-level PE packet dispatcher.
    // LOAD packets update local buffers and do not generate a response.
    // COMPUTE packets consume local buffers and return an output slice packet.
    Packet *process_packet(const Packet &job)
    {
        if (job.datas.empty())
            return NULL;

        int op = (int)job.datas[0];
        if (op == OP_LOAD_INPUT)
        {
            load_buffer(input_buf, job);
            metric_peak_input_words() =
                max(metric_peak_input_words(),
                    (unsigned long long)input_buf.size());
            return make_load_ack(job);
        }
        if (op == OP_LOAD_WEIGHT)
        {
            load_buffer(weight_buf, job);
            metric_peak_weight_words() =
                max(metric_peak_weight_words(),
                    (unsigned long long)weight_buf.size());
            return make_load_ack(job);
        }
        if (op == OP_LOAD_BIAS)
        {
            load_buffer(bias_buf, job);
            metric_peak_bias_words() =
                max(metric_peak_bias_words(),
                    (unsigned long long)bias_buf.size());
            return make_load_ack(job);
        }
        if (op == OP_COMPUTE_CONV)
            return run_conv(job);
        if (op == OP_COMPUTE_POOL)
            return run_pool(job);
        if (op == OP_COMPUTE_FC)
            return run_fc(job);
        return NULL;
    }

    // Generic load packet:
    // [op, layer_id, payload_size, payload...]
    void load_buffer(vector<float> & buffer, const Packet &job)
    {
        int payload_size = (int)job.datas[2];
        buffer.assign(job.datas.begin() + 3, job.datas.begin() + 3 + payload_size);
    }

    Packet *make_load_ack(const Packet &job)
    {
        Packet *result = new Packet();
        result->source_id = id;
        result->dest_id = job.source_id;
        result->datas.push_back((float)OP_LOAD_ACK);
        result->datas.push_back(job.datas[0]);
        result->datas.push_back(job.datas[1]);
        return result;
    }

    // Create a result packet addressed back to the original sender.
    Packet *make_result(const Packet &job, int job_id)
    {
        Packet *result = new Packet();
        result->source_id = id;
        result->dest_id = job.source_id;
        result->datas.push_back((float)job_id);
        return result;
    }

    // Convolution compute command:
    // [op, job_id, apply_relu, in_h, in_w, in_ch, out_ch, kernel, stride,
    //  oc_start, oc_count]
    // input_buf contains the full input feature map; weight_buf/bias_buf contain
    // only this PE's assigned output-channel slice.
    Packet *run_conv(const Packet &job)
    {
        const vector<float> &d = job.datas;
        int p = 1;
        int job_id = (int)d[p++];
        bool apply_relu = ((int)d[p++] != 0);
        int in_h = (int)d[p++];
        int in_w = (int)d[p++];
        int in_ch = (int)d[p++];
        int out_ch = (int)d[p++];
        int kernel = (int)d[p++];
        int stride = (int)d[p++];
        int oc_start = (int)d[p++];
        int oc_count = (int)d[p++];

#ifdef DEBUG
        cout << "[PE_DEBUG] PE " << id
             << " start CONV job " << job_id
             << ", oc_start=" << oc_start
             << ", oc_count=" << oc_count
             << ", input_values=" << input_buf.size()
             << ", weight_values=" << weight_buf.size()
             << ", bias_values=" << bias_buf.size() << "." << endl;
#endif

        int out_h = (in_h - kernel) / stride + 1;
        int out_w = (in_w - kernel) / stride + 1;
        long long mac_count = (long long)oc_count * out_h * out_w *
                              in_ch * kernel * kernel;
#ifdef DEBUG
        cout << "[PE_DEBUG] PE " << id
             << " CONV job " << job_id
             << " compute latency cycles="
             << ((mac_count + MACS_PER_CYCLE - 1) / MACS_PER_CYCLE)
             << " for macs=" << mac_count << "." << endl;
#endif
        wait_for_macs(mac_count);

        Packet *result = make_result(job, job_id);
        result->datas.push_back((float)oc_start);
        result->datas.push_back((float)oc_count);
        result->datas.push_back((float)out_h);
        result->datas.push_back((float)out_w);

        // Each PE computes only its assigned output-channel range.
        for (int local_oc = 0; local_oc < oc_count; local_oc++)
        {
            int wt_oc_base = local_oc * in_ch * kernel * kernel;
            for (int oh = 0; oh < out_h; oh++)
            {
                for (int ow = 0; ow < out_w; ow++)
                {
                    double sum = bias_buf[local_oc];
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
                                sum += input_buf[in_row + iw_base + kw] * weight_buf[wt_row + kw];
                        }
                    }
                    if (apply_relu && sum < 0.0)
                        sum = 0.0;
                    result->datas.push_back((float)sum);
                }
            }
        }

        (void)out_ch;
#ifdef DEBUG
        cout << "[PE_DEBUG] PE " << id
             << " finish CONV job " << job_id
             << ", result_payload_size=" << result->datas.size()
             << "." << endl;
#endif
        return result;
    }

    // Max-pooling compute command:
    // [op, job_id, in_h, in_w, ch, kernel, stride, c_start, c_count]
    // input_buf contains the full input feature map.
    Packet *run_pool(const Packet &job)
    {
        const vector<float> &d = job.datas;
        int p = 1;
        int job_id = (int)d[p++];
        int in_h = (int)d[p++];
        int in_w = (int)d[p++];
        int ch = (int)d[p++];
        int kernel = (int)d[p++];
        int stride = (int)d[p++];
        int c_start = (int)d[p++];
        int c_count = (int)d[p++];

        int out_h = (in_h - kernel) / stride + 1;
        int out_w = (in_w - kernel) / stride + 1;
        Packet *result = make_result(job, job_id);
        result->datas.push_back((float)c_start);
        result->datas.push_back((float)c_count);
        result->datas.push_back((float)out_h);
        result->datas.push_back((float)out_w);

        // Pooling is independent per channel, so the Controller partitions
        // this job by channel range.
        for (int local_c = 0; local_c < c_count; local_c++)
        {
            int c = c_start + local_c;
            for (int oh = 0; oh < out_h; oh++)
            {
                for (int ow = 0; ow < out_w; ow++)
                {
                    int start_h = oh * stride;
                    int start_w = ow * stride;
                    float best = input_buf[idx3(c, start_h, start_w, in_h, in_w)];
                    for (int kh = 0; kh < kernel; kh++)
                        for (int kw = 0; kw < kernel; kw++)
                            best = max(best, input_buf[idx3(c, start_h + kh, start_w + kw, in_h, in_w)]);
                    result->datas.push_back(best);
                }
            }
        }

        (void)ch;
        return result;
    }

    // Fully connected compute command:
    // [op, job_id, apply_relu, in_size, out_size, o_start, o_count]
    // input_buf contains the full input vector; weight_buf/bias_buf contain only
    // this PE's assigned output-neuron rows.
    Packet *run_fc(const Packet &job)
    {
        const vector<float> &d = job.datas;
        int p = 1;
        int job_id = (int)d[p++];
        bool apply_relu = ((int)d[p++] != 0);
        int in_size = (int)d[p++];
        int out_size = (int)d[p++];
        int o_start = (int)d[p++];
        int o_count = (int)d[p++];

        long long mac_count = (long long)o_count * in_size;
#ifdef DEBUG
        cout << "[PE_DEBUG] PE " << id
             << " FC job " << job_id
             << " compute latency cycles="
             << ((mac_count + MACS_PER_CYCLE - 1) / MACS_PER_CYCLE)
             << " for macs=" << mac_count << "." << endl;
#endif
        wait_for_macs(mac_count, true);

        Packet *result = make_result(job, job_id);
        result->datas.push_back((float)o_start);
        result->datas.push_back((float)o_count);
        result->datas.push_back(1.0f);
        result->datas.push_back((float)out_size);

        // FC is partitioned by output neuron rows.
        for (int local_o = 0; local_o < o_count; local_o++)
        {
            const float *w = &weight_buf[local_o * in_size];
            double sum = bias_buf[local_o];
            for (int i = 0; i < in_size; i++)
                sum += w[i] * input_buf[i];
            if (apply_relu && sum < 0.0)
                sum = 0.0;
            result->datas.push_back((float)sum);
        }

        return result;
    }

    // PE is called by Core when a complete packet arrives; it has no clocked
    // process of its own.
    SC_HAS_PROCESS(PE);
    PE(sc_module_name name) : sc_module(name), id(0) {}
};

#endif
