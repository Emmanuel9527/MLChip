#ifndef PE_H
#define PE_H

#include "nonlinear_function.h"
#include "pe_local_sram.h"
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
    OP_LOAD_ACK = 7,
    OP_SYSTOLIC_FC_INIT = 8,
    OP_SYSTOLIC_FC_ACCUM = 9,
    OP_SYSTOLIC_FC_FINISH = 10,
    OP_SYSTOLIC_MVM_ACCUM = 11,
    OP_SYSTOLIC_BCAST_INPUT = 12
};

SC_MODULE(PE)
{
    static const int MACS_PER_CYCLE = 16;

    // PE id matches the local router/core id in the 4x4 mesh.
    int id;

    // Explicit PE internal hardware blocks.
    PeLocalSram local_sram;
    NonlinearFunction nonlinear;
    double systolic_acc;
    int systolic_out_index;

    // Channel-major tensor addressing helper: tensor[c][h][w].
    static int idx3(int c, int h, int w, int height, int width)
    {
        return c * height * width + h * width + w;
    }

    void wait_for_macs(long long mac_count)
    {
        long long cycles = (mac_count + MACS_PER_CYCLE - 1) / MACS_PER_CYCLE;
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
            local_sram.load_input(load_payload(job));
            return make_load_ack(job);
        }
        if (op == OP_LOAD_WEIGHT)
        {
            local_sram.load_weight(load_payload(job));
            return make_load_ack(job);
        }
        if (op == OP_LOAD_BIAS)
        {
            local_sram.load_bias(load_payload(job));
            return make_load_ack(job);
        }
        if (op == OP_SYSTOLIC_BCAST_INPUT)
            return run_systolic_bcast_input(job);
        if (op == OP_COMPUTE_CONV)
            return run_conv(job);
        if (op == OP_COMPUTE_POOL)
            return run_pool(job);
        if (op == OP_COMPUTE_FC)
            return run_fc(job);
        if (op == OP_SYSTOLIC_FC_INIT)
            return run_systolic_fc_init(job);
        if (op == OP_SYSTOLIC_FC_ACCUM)
            return run_systolic_fc_accum(job);
        if (op == OP_SYSTOLIC_FC_FINISH)
            return run_systolic_fc_finish(job);
        if (op == OP_SYSTOLIC_MVM_ACCUM)
            return run_systolic_mvm_accum(job);
        return NULL;
    }

    // Generic load packet:
    // [op, layer_id, payload_size, payload...]
    vector<float> load_payload(const Packet &job)
    {
        int payload_size = (int)job.datas[2];
        return vector<float>(job.datas.begin() + 3,
                             job.datas.begin() + 3 + payload_size);
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

    // Systolic input broadcast for the lecture MVM map:
    // [op, layer_id, active_cols, payload_size, payload...]
    // The input vector segment b(j) flows horizontally from left to right
    // across the active columns in one PE row.
    Packet *run_systolic_bcast_input(const Packet &job)
    {
        int active_cols = (int)job.datas[2];
        int payload_size = (int)job.datas[3];
        vector<float> payload(job.datas.begin() + 4,
                              job.datas.begin() + 4 + payload_size);
        local_sram.load_input(payload);

        int col = id % 4;
        if (col + 1 < active_cols)
        {
            Packet *forward = new Packet();
            forward->source_id = job.source_id;
            forward->dest_id = id + 1;
            forward->datas = job.datas;
            return forward;
        }

        return make_load_ack(job);
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
    // Local SRAM contains the full input feature map plus this PE's assigned
    // weight/bias output-channel slice.
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
             << ", input_values=" << local_sram.input_size()
             << ", weight_values=" << local_sram.weight_size()
             << ", bias_values=" << local_sram.bias_size() << "." << endl;
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
                    double sum = local_sram.read_bias(local_oc);
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
                                sum += local_sram.read_input(in_row + iw_base + kw) *
                                       local_sram.read_weight(wt_row + kw);
                        }
                    }
                    result->datas.push_back(nonlinear.relu((float)sum, apply_relu));
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
    // Local SRAM contains the full input feature map.
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
                    float best = local_sram.read_input(idx3(c, start_h, start_w, in_h, in_w));
                    for (int kh = 0; kh < kernel; kh++)
                        for (int kw = 0; kw < kernel; kw++)
                            best = nonlinear.max_select(
                                best,
                                local_sram.read_input(idx3(c, start_h + kh, start_w + kw, in_h, in_w)));
                    result->datas.push_back(best);
                }
            }
        }

        (void)ch;
        return result;
    }

    // Fully connected compute command:
    // [op, job_id, apply_relu, in_size, out_size, o_start, o_count]
    // Local SRAM contains the full input vector plus this PE's assigned
    // weight/bias output-neuron rows.
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
        wait_for_macs(mac_count);

        Packet *result = make_result(job, job_id);
        result->datas.push_back((float)o_start);
        result->datas.push_back((float)o_count);
        result->datas.push_back(1.0f);
        result->datas.push_back((float)out_size);

        // FC is partitioned by output neuron rows.
        for (int local_o = 0; local_o < o_count; local_o++)
        {
            int weight_base = local_o * in_size;
            double sum = local_sram.read_bias(local_o);
            for (int i = 0; i < in_size; i++)
                sum += local_sram.read_weight(weight_base + i) *
                       local_sram.read_input(i);
            result->datas.push_back(nonlinear.relu((float)sum, apply_relu));
        }

        return result;
    }

    // Systolic-style FC init:
    // [op, job_id, output_index, bias]
    Packet *run_systolic_fc_init(const Packet &job)
    {
        const vector<float> &d = job.datas;
        int job_id = (int)d[1];
        systolic_out_index = (int)d[2];
        systolic_acc = (double)d[3];

        Packet *result = make_result(job, job_id);
        result->datas.push_back((float)systolic_out_index);
        return result;
    }

    // Systolic-style FC accumulate:
    // [op, job_id, tile_words]
    // Local SRAM holds one activation tile and one weight tile.
    Packet *run_systolic_fc_accum(const Packet &job)
    {
        const vector<float> &d = job.datas;
        int job_id = (int)d[1];
        int tile_words = (int)d[2];
        if (tile_words > (int)local_sram.input_size())
            tile_words = (int)local_sram.input_size();
        if (tile_words > (int)local_sram.weight_size())
            tile_words = (int)local_sram.weight_size();

        long long mac_count = tile_words;
        wait_for_macs(mac_count);

        for (int i = 0; i < tile_words; i++)
            systolic_acc += (double)local_sram.read_input(i) *
                            (double)local_sram.read_weight(i);

        Packet *result = make_result(job, job_id);
        result->datas.push_back((float)systolic_out_index);
        result->datas.push_back((float)tile_words);
        result->datas.push_back((float)systolic_acc);
        return result;
    }

    // Systolic-style FC finish:
    // [op, job_id, apply_relu]
    Packet *run_systolic_fc_finish(const Packet &job)
    {
        const vector<float> &d = job.datas;
        int job_id = (int)d[1];
        bool apply_relu = ((int)d[2] != 0);
        double value = systolic_acc;

        Packet *result = make_result(job, job_id);
        result->datas.push_back((float)systolic_out_index);
        result->datas.push_back(nonlinear.relu((float)value, apply_relu));
        return result;
    }

    // Lecture-style systolic MVM accumulate:
    // [op, job_id, output_index, controller_id, apply_relu_on_top_edge,
    //  top_boundary_row, psum_in]
    // Each PE consumes its local input/weight segment, adds to psum_in, and
    // forwards the updated partial sum to the PE on its north side until it
    // reaches this sub-array's top boundary row. That boundary PE returns the
    // output value to the Controller.
    Packet *run_systolic_mvm_accum(const Packet &job)
    {
        const vector<float> &d = job.datas;
        int job_id = (int)d[1];
        int output_index = (int)d[2];
        int controller_id = (int)d[3];
        bool apply_relu = ((int)d[4] != 0);
        int top_boundary_row = (int)d[5];
        double psum = (double)d[6];

        unsigned int words = local_sram.input_size();
        if (words > local_sram.weight_size())
            words = local_sram.weight_size();
        wait_for_macs(words);

        for (unsigned int i = 0; i < words; i++)
            psum += (double)local_sram.read_input(i) *
                    (double)local_sram.read_weight(i);

        int row = id / 4;
        Packet *result = new Packet();
        result->source_id = id;

        if (row > top_boundary_row)
        {
            result->dest_id = id - 4;
            result->datas.push_back((float)OP_SYSTOLIC_MVM_ACCUM);
            result->datas.push_back((float)job_id);
            result->datas.push_back((float)output_index);
            result->datas.push_back((float)controller_id);
            result->datas.push_back(apply_relu ? 1.0f : 0.0f);
            result->datas.push_back((float)top_boundary_row);
            result->datas.push_back((float)psum);
        }
        else
        {
            result->dest_id = controller_id;
            result->datas.push_back((float)job_id);
            result->datas.push_back((float)output_index);
            result->datas.push_back(nonlinear.relu((float)psum, apply_relu));
        }

        return result;
    }

    // PE is called by Core when a complete packet arrives; it has no clocked
    // process of its own.
    SC_HAS_PROCESS(PE);
    PE(sc_module_name name) :
        sc_module(name),
        local_sram("local_sram"),
        nonlinear("nonlinear"),
        id(0),
        systolic_acc(0.0),
        systolic_out_index(0) {}
};

#endif
