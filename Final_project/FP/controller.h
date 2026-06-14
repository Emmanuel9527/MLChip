#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "memory_map.h"
#include "noc.h"
#include "pe.h"
#include "systemc.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace std;

extern "C" void deadlock_watchdog_report_unexpected_flit(int expected_node,
                                                         int flit_type,
                                                         unsigned int payload,
                                                         int active,
                                                         int packet_flits);

SC_MODULE(Controller)
{
    sc_in<bool> rst;
    sc_in<bool> clk;

    // Controller-facing DMA command and streaming interfaces.
    sc_out<bool> dma_cmd_valid;
    sc_in<bool> dma_cmd_ready;
    sc_out<bool> dma_cmd_write;
    sc_out<unsigned int> dma_cmd_addr;
    sc_out<unsigned int> dma_cmd_len;
    sc_in<bool> dma_done;

    sc_in<float> dma_read_data;
    sc_in<bool> dma_read_valid;
    sc_out<bool> dma_read_ready;
    sc_out<float> dma_write_data;
    sc_out<bool> dma_write_valid;
    sc_in<bool> dma_write_ready;

    // Local NoC interface. The controller is connected to router 0 as the
    // master node; PE workers are connected to routers 1 through 15.
    sc_out<Flit> flit_tx;
    sc_out<bool> req_tx;
    sc_in<bool> ack_tx;

    sc_in<Flit> flit_rx;
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
    static const int LOAD_ACK_WORD = 7;
    static const int DEADLOCK_WATCHDOG_CYCLES = 200000;
    static const int WATCHDOG_IDLE_LOG_INTERVALS = 10;
    static const int UNEXPECTED_FLIT_LOG_LIMIT = 5;
    static const int UNEXPECTED_FLIT_LOG_INTERVAL = 1000;
    static const int DMA_PROGRESS_INTERVAL = 1000000;
    static const int PACKET_PROGRESS_FLITS = 50000;

    // Monotonic id used to label jobs sent to PEs.
    int next_job_id;
    vector<Packet> pending_packets;
    string wait_context;

    void debug_log(const string &msg)
    {
#if defined(FP_TRACE)
        cout << "[TRACE] " << msg << endl;
#elif defined(DEBUG)
        cout << "[DEBUG] " << msg << endl;
#else
        (void)msg;
#endif
    }

    void set_wait_context(const string &context)
    {
        wait_context = context;
    }

    void set_rx_ready(bool ready)
    {
        ack_rx.write(ready);
    }

    template <typename T>
    string num_to_string(T value)
    {
        ostringstream oss;
        oss << value;
        return oss.str();
    }

    // Channel-major tensor addressing helper: tensor[c][h][w].
    static int idx3(int c, int h, int w, int height, int width)
    {
        return c * height * width + h * width + w;
    }

    void start_dma_command(bool is_write, unsigned int addr, unsigned int words)
    {
        while (!dma_cmd_ready.read())
            wait();
        dma_cmd_write.write(is_write);
        dma_cmd_addr.write(addr);
        dma_cmd_len.write(words);
        dma_cmd_valid.write(true);
        wait();
        dma_cmd_valid.write(false);
    }

    vector<float> dma_read_vector(unsigned int addr, int expected, const string &name)
    {
        vector<float> values;
        values.reserve(expected);
        debug_log(string("DMA read request: ") + name + ", words=" + num_to_string(expected) + ".");
        start_dma_command(false, addr, (unsigned int)expected);

        while ((int)values.size() < expected)
        {
            dma_read_ready.write(true);
            wait();
            if (dma_read_valid.read())
            {
                values.push_back(dma_read_data.read());
                if (expected >= DMA_PROGRESS_INTERVAL &&
                    (int)values.size() % DMA_PROGRESS_INTERVAL == 0)
                {
                    debug_log(string("DMA read progress: ") + name + ", " +
                              num_to_string(values.size()) + "/" +
                              num_to_string(expected) + " values.");
                }
            }
        }
        dma_read_ready.write(false);
        while (!dma_done.read())
            wait();

        debug_log(string("Finished DMA read: ") + name + ", " +
                  num_to_string(values.size()) + " values.");
        return values;
    }

    void dma_write_vector(unsigned int addr, const vector<float> &values, const string &name)
    {
        debug_log(string("DMA write request: ") + name + ", words=" +
                  num_to_string(values.size()) + ".");
        start_dma_command(true, addr, (unsigned int)values.size());

        for (size_t i = 0; i < values.size(); i++)
        {
            dma_write_data.write(values[i]);
            dma_write_valid.write(true);
            do { wait(); } while (!dma_write_ready.read());
            dma_write_valid.write(false);
        }
        while (!dma_done.read())
            wait();
        debug_log(string("Finished DMA write: ") + name + ".");
    }

    vector<float> read_dram_tensor(int id, bool type, int expected)
    {
        unsigned int base = dram_tensor_base(id, type);
        string name;
        if (id == 0)
            name = "input image";
        else
            name = string("layer ") + num_to_string(id) + (type ? " bias" : " weight");
        return dma_read_vector(base, expected, name);
    }

    // Send one flit into router[0] using valid-ready handshake.
    void send_flit(const Flit &flit)
    {
        while (true)
        {
            flit_tx.write(flit);
            req_tx.write(true);
            wait();

            if (ack_tx.read())
            {
                req_tx.write(false);
                wait();
                return;
            }
        }
    }

    // Serialize a packet into one HEAD flit and payload BODY/TAIL flits.
    void send_packet(const Packet &packet)
    {
        int payload_flits =
            (packet.datas.size() + PACKED_FLIT_WORDS - 1) / PACKED_FLIT_WORDS;
        bool report_progress = false;
#if defined(DEBUG) || defined(FP_TRACE)
        report_progress = payload_flits >= PACKET_PROGRESS_FLITS;
#endif
        int sent_payload_flits = 0;

        if (report_progress)
        {
            int op = packet.datas.empty() ? -1 : (int)packet.datas[0];
            debug_log(string("Sending large packet to PE ") +
                      num_to_string(packet.dest_id) +
                      ", op=" + num_to_string(op) +
                      ", payload_words=" + num_to_string(packet.datas.size()) +
                      ", packed_flits=" + num_to_string(payload_flits) + ".");
        }

        // HEAD flit layout: type=HEAD, destination id, source id.
        Flit header;
        header = 0;
        set_flit_type(header, NOC_HEAD_FLIT);
        set_flit_count(header, 0);
        set_header_fields(header, packet.dest_id, packet.source_id);
        send_flit(header);

        for (size_t i = 0; i < packet.datas.size();)
        {
            int count = (int)min((size_t)PACKED_FLIT_WORDS,
                                 packet.datas.size() - i);
            bool last = (i + count == packet.datas.size());

            Flit flit;
            flit = 0;
            set_flit_type(flit, last ? NOC_TAIL_FLIT : NOC_BODY_FLIT);
            set_flit_count(flit, count);

            for (int lane = 0; lane < count; lane++)
            {
                union
                {
                    float fval;
                    unsigned int ival;
                } converter;

                converter.fval = packet.datas[i + lane];
                set_flit_word(flit, lane, converter.ival);
            }
            send_flit(flit);
            i += count;
            sent_payload_flits++;

            if (report_progress &&
                (sent_payload_flits % PACKET_PROGRESS_FLITS == 0 ||
                 sent_payload_flits == payload_flits))
            {
                debug_log(string("Packet send progress to PE ") +
                          num_to_string(packet.dest_id) + ": " +
                          num_to_string(sent_payload_flits) + "/" +
                          num_to_string(payload_flits) +
                          " packed flits.");
            }
        }
    }

    // Block until one complete packet returns from a worker PE.
    Packet receive_packet()
    {
        Packet packet;
        bool active = false;
        int idle_cycles = 0;
        int idle_reports = 0;
        int packet_flits = 0;
        int unexpected_flits = 0;
        bool req_seen = false;

        while (true)
        {
            set_rx_ready(true);
            wait();

            if (!req_rx.read())
            {
                req_seen = false;
                idle_cycles++;
                if (idle_cycles == DEADLOCK_WATCHDOG_CYCLES)
                {
                    idle_reports++;
                    if (idle_reports == 1 ||
                        idle_reports % WATCHDOG_IDLE_LOG_INTERVALS == 0)
                    {
                        debug_log(string("Deadlock watchdog: no flit at Controller for ") +
                                  num_to_string(DEADLOCK_WATCHDOG_CYCLES * idle_reports) +
                                  " cycles while waiting for " + wait_context + ".");
                    }
                    idle_cycles = 0;
                }
                continue;
            }

            if (req_seen)
                continue;
            req_seen = true;
            idle_cycles = 0;
            idle_reports = 0;
            Flit flit = flit_rx.read();
            int type = get_flit_type(flit);

            if (type == NOC_HEAD_FLIT)
            {
                // HEAD starts a new return packet.
                packet = Packet();
                packet.dest_id = get_header_dest(flit);
                packet.source_id = get_header_source(flit);
                active = true;
                packet_flits = 1;
            }
            else if (active && (type == NOC_BODY_FLIT || type == NOC_TAIL_FLIT))
            {
                // BODY/TAIL payloads are reconstructed from raw float bits.
                int count = get_flit_count(flit);
                if (count <= 0 || count > PACKED_FLIT_WORDS)
                    count = PACKED_FLIT_WORDS;

                for (int lane = 0; lane < count; lane++)
                {
                    union
                    {
                        float fval;
                        unsigned int ival;
                    } converter;

                    converter.ival = get_flit_word(flit, lane);
                    packet.datas.push_back(converter.fval);
                }
                packet_flits++;

                if (type == NOC_TAIL_FLIT)
                {
                    set_rx_ready(false);
                    return packet;
                }
            }
            else
            {
                unexpected_flits++;
                if (unexpected_flits <= UNEXPECTED_FLIT_LOG_LIMIT ||
                    unexpected_flits % UNEXPECTED_FLIT_LOG_INTERVAL == 0)
                {
#if defined(DEBUG) || defined(FP_TRACE)
                    deadlock_watchdog_report_unexpected_flit(0,
                                                             type,
                                                             get_flit_word(flit, 0),
                                                             active ? 1 : 0,
                                                             packet_flits);
#endif
                    debug_log(string("Deadlock watchdog: unexpected flit type ") +
                              num_to_string(type) + " while waiting for " +
                              wait_context + ", active=" + num_to_string(active) +
                              ", packet flits=" + num_to_string(packet_flits) +
                              ", suppressed similar flits=" +
                              num_to_string(max(0, unexpected_flits - UNEXPECTED_FLIT_LOG_LIMIT)) +
                              ".");
                }
            }
        }
    }

    bool packet_first_word_is(const Packet &packet, int first_word)
    {
        return !packet.datas.empty() && (int)packet.datas[0] == first_word;
    }

    bool packet_is_load_ack(const Packet &packet, int expected_source)
    {
        if (packet.datas.size() != 3)
            return false;
        if ((int)packet.datas[0] != LOAD_ACK_WORD)
            return false;
        int ack_op = (int)packet.datas[1];
        if (ack_op != OP_LOAD_INPUT &&
            ack_op != OP_LOAD_WEIGHT &&
            ack_op != OP_LOAD_BIAS)
            return false;
        return expected_source < 0 || packet.source_id == expected_source;
    }

    bool packet_is_load_ack_op(const Packet &packet, int expected_source, int expected_op)
    {
        return packet.datas.size() == 3 &&
               (int)packet.datas[0] == LOAD_ACK_WORD &&
               (int)packet.datas[1] == expected_op &&
               (expected_source < 0 || packet.source_id == expected_source);
    }

    Packet take_pending_packet(size_t index)
    {
        Packet packet = pending_packets[index];
        pending_packets.erase(pending_packets.begin() + index);
        return packet;
    }

    Packet receive_specific_packet(int first_word, int expected_source)
    {
        while (true)
        {
            for (size_t i = 0; i < pending_packets.size(); i++)
            {
                if (first_word == LOAD_ACK_WORD &&
                    packet_is_load_ack(pending_packets[i], expected_source))
                    return take_pending_packet(i);

                if (first_word != LOAD_ACK_WORD &&
                    packet_first_word_is(pending_packets[i], first_word) &&
                    (expected_source < 0 || pending_packets[i].source_id == expected_source))
                    return take_pending_packet(i);
            }

            Packet packet = receive_packet();
            if (first_word == LOAD_ACK_WORD &&
                packet_is_load_ack(packet, expected_source))
                return packet;

            if (first_word != LOAD_ACK_WORD &&
                packet_first_word_is(packet, first_word) &&
                (expected_source < 0 || packet.source_id == expected_source))
                return packet;

            pending_packets.push_back(packet);
            debug_log(string("Skipping unexpected packet from PE ") +
                      num_to_string(packet.source_id) + " while waiting for packet type " +
                      num_to_string(first_word) + ".");
        }
    }

    Packet receive_compute_result()
    {
        while (true)
        {
            for (size_t i = 0; i < pending_packets.size(); i++)
            {
                if (!packet_is_load_ack(pending_packets[i], -1))
                    return take_pending_packet(i);
            }

            Packet packet = receive_packet();
            if (!packet_is_load_ack(packet, -1))
                return packet;

            debug_log(string("Dropping stale load ACK from PE ") +
                      num_to_string(packet.source_id) + ".");
        }
    }

    void wait_for_load_acks(int expected, const string &label, int expected_source)
    {
        for (int i = 0; i < expected; i++)
        {
            Packet ack = receive_specific_packet(LOAD_ACK_WORD, expected_source);
            int ack_op = ack.datas.size() > 1 ? (int)ack.datas[1] : -1;
            debug_log(string("Received ") + label + " ACK from PE " +
                      num_to_string(ack.source_id) +
                      ", original op " + num_to_string(ack_op) + ".");
        }
    }

    void wait_for_load_acks(int expected, const string &label)
    {
        wait_for_load_acks(expected, label, -1);
    }

    void wait_for_load_ack_op(int expected_source, int expected_op, const string &label)
    {
        set_wait_context(label + " ACK");
        while (true)
        {
            for (size_t i = 0; i < pending_packets.size(); i++)
            {
                if (packet_is_load_ack_op(pending_packets[i], expected_source, expected_op))
                {
                    Packet ack = take_pending_packet(i);
                    debug_log(string("Received ") + label + " ACK from PE " +
                              num_to_string(ack.source_id) +
                              ", original op " + num_to_string((int)ack.datas[1]) + ".");
                    return;
                }
            }

            Packet packet = receive_packet();
            if (packet_is_load_ack_op(packet, expected_source, expected_op))
            {
                debug_log(string("Received ") + label + " ACK from PE " +
                          num_to_string(packet.source_id) +
                          ", original op " + num_to_string((int)packet.datas[1]) + ".");
                return;
            }

            pending_packets.push_back(packet);
            debug_log(string("Buffering packet from PE ") + num_to_string(packet.source_id) +
                      " while waiting for load op " + num_to_string(expected_op) + ".");
        }
    }

    string missing_workers_string(const vector<int> &workers, const vector<bool> &received)
    {
        string text;
        for (size_t i = 0; i < workers.size(); i++)
        {
            if (received[i])
                continue;
            if (!text.empty())
                text += ",";
            text += num_to_string(workers[i]);
        }
        return text.empty() ? string("none") : text;
    }

    int worker_index(const vector<int> &workers, int source_id)
    {
        for (size_t i = 0; i < workers.size(); i++)
            if (workers[i] == source_id)
                return (int)i;
        return -1;
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
        debug_log(string("Broadcasting input buffer for layer ") + num_to_string(layer) +
                  " to worker PEs, values=" + num_to_string(payload.size()) + ".");
        send_packet(make_load_packet(BROADCAST_WORKERS, OP_LOAD_INPUT, layer, payload));
        wait_for_load_acks(WORKER_COUNT, "input broadcast");
        debug_log(string("Finished input broadcast for layer ") + num_to_string(layer) + ".");
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
        debug_log(string("Starting CONV layer ") + num_to_string(layer) + ".");
        int weight_count = out_ch * in_ch * kernel * kernel;
        vector<float> weight = read_dram_tensor(layer, false, weight_count);
        vector<float> bias = read_dram_tensor(layer, true, out_ch);
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

            debug_log(string("Preloading CONV layer ") + num_to_string(layer) +
                      " worker PE " + num_to_string(worker) +
                      ", output channels [" + num_to_string(base) + ", " +
                      num_to_string(base + oc_count - 1) + "].");
            send_packet(make_load_packet(worker, OP_LOAD_WEIGHT, layer, weight_part));
            wait_for_load_ack_op(worker, OP_LOAD_WEIGHT,
                                 string("CONV layer ") + num_to_string(layer) +
                                 " weight for PE " + num_to_string(worker));
            send_packet(make_load_packet(worker, OP_LOAD_BIAS, layer, bias_part));
            wait_for_load_ack_op(worker, OP_LOAD_BIAS,
                                 string("CONV layer ") + num_to_string(layer) +
                                 " bias for PE " + num_to_string(worker));

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
            debug_log(string("Dispatching CONV job ") + num_to_string(job_id) +
                      " to PE " + num_to_string(workers[i]) + ".");
            send_packet(packet);
        }

        vector<bool> received(workers.size(), false);
        int received_count = 0;
        while (received_count < (int)workers.size())
        {
            set_wait_context(string("CONV layer ") + num_to_string(layer) +
                             " result, missing PEs [" +
                             missing_workers_string(workers, received) + "]");
            Packet result = receive_compute_result();
            debug_log(string("Received CONV result from PE ") + num_to_string(result.source_id) + ".");
            int index = worker_index(workers, result.source_id);
            if (index < 0)
            {
                debug_log(string("Deadlock watchdog: unexpected CONV result source PE ") +
                          num_to_string(result.source_id) + ".");
                continue;
            }
            if (received[index])
            {
                debug_log(string("Deadlock watchdog: duplicate CONV result from PE ") +
                          num_to_string(result.source_id) + ".");
                continue;
            }
            received[index] = true;
            received_count++;
            merge_channel_result(output, result);
        }

        debug_log(string("Finished CONV layer ") + num_to_string(layer) + ".");
        return output;
    }

    // Run one max-pooling layer through worker PEs.
    // Pooling has no weight/bias, so only input is preloaded.
    vector<float> run_pool_on_pes(const vector<float> &feature,
                                  int in_h, int in_w, int ch,
                                  int kernel, int stride)
    {
        debug_log("Starting POOL layer.");
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

            debug_log(string("Preparing POOL worker PE ") + num_to_string(worker) +
                      ", channels [" + num_to_string(base) + ", " +
                      num_to_string(base + c_count - 1) + "].");
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
            debug_log(string("Dispatching POOL job ") + num_to_string(job_id) +
                      " to PE " + num_to_string(workers[i]) + ".");
            send_packet(packet);
        }

        vector<bool> received(workers.size(), false);
        int received_count = 0;
        while (received_count < (int)workers.size())
        {
            set_wait_context(string("POOL result, missing PEs [") +
                             missing_workers_string(workers, received) + "]");
            Packet result = receive_compute_result();
            debug_log(string("Received POOL result from PE ") + num_to_string(result.source_id) + ".");
            int index = worker_index(workers, result.source_id);
            if (index < 0)
            {
                debug_log(string("Deadlock watchdog: unexpected POOL result source PE ") +
                          num_to_string(result.source_id) + ".");
                continue;
            }
            if (received[index])
            {
                debug_log(string("Deadlock watchdog: duplicate POOL result from PE ") +
                          num_to_string(result.source_id) + ".");
                continue;
            }
            received[index] = true;
            received_count++;
            merge_channel_result(output, result);
        }

        debug_log("Finished POOL layer.");
        return output;
    }

    // Run one fully connected layer through worker PEs.
    vector<float> run_fc_on_pes(const vector<float> &feature,
                                int layer, int out_size, bool apply_relu)
    {
        debug_log(string("Starting FC layer ") + num_to_string(layer) + ".");
        int in_size = (int)feature.size();
        vector<float> weight = read_dram_tensor(layer, false, out_size * in_size);
        vector<float> bias = read_dram_tensor(layer, true, out_size);
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

            debug_log(string("Preloading FC layer ") + num_to_string(layer) +
                      " worker PE " + num_to_string(worker) +
                      ", output neurons [" + num_to_string(base) + ", " +
                      num_to_string(base + o_count - 1) + "].");
            send_packet(make_load_packet(worker, OP_LOAD_WEIGHT, layer, weight_part));
            wait_for_load_ack_op(worker, OP_LOAD_WEIGHT,
                                 string("FC layer ") + num_to_string(layer) +
                                 " weight for PE " + num_to_string(worker));
            send_packet(make_load_packet(worker, OP_LOAD_BIAS, layer, bias_part));
            wait_for_load_ack_op(worker, OP_LOAD_BIAS,
                                 string("FC layer ") + num_to_string(layer) +
                                 " bias for PE " + num_to_string(worker));

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
            debug_log(string("Dispatching FC job ") + num_to_string(job_id) +
                      " to PE " + num_to_string(workers[i]) + ".");
            send_packet(packet);
        }

        vector<bool> received(workers.size(), false);
        int received_count = 0;
        while (received_count < (int)workers.size())
        {
            set_wait_context(string("FC layer ") + num_to_string(layer) +
                             " result, missing PEs [" +
                             missing_workers_string(workers, received) + "]");
            Packet result = receive_compute_result();
            debug_log(string("Received FC result from PE ") + num_to_string(result.source_id) + ".");
            int index = worker_index(workers, result.source_id);
            if (index < 0)
            {
                debug_log(string("Deadlock watchdog: unexpected FC result source PE ") +
                          num_to_string(result.source_id) + ".");
                continue;
            }
            if (received[index])
            {
                debug_log(string("Deadlock watchdog: duplicate FC result from PE ") +
                          num_to_string(result.source_id) + ".");
                continue;
            }
            received[index] = true;
            received_count++;
            int p = 0;
            int result_job_id = (int)result.datas[p++];
            int start = (int)result.datas[p++];
            int count = (int)result.datas[p++];
            p += 2;
            for (int j = 0; j < count; j++)
                output[start + j] = result.datas[p++];
            (void)result_job_id;
        }

        debug_log(string("Finished FC layer ") + num_to_string(layer) + ".");
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
        const char *paths[] = {
            "data/imagenet_classes.txt",
            "../data/imagenet_classes.txt",
            "../../data/imagenet_classes.txt",
            "../../hw4/data/imagenet_classes.txt",
            "../../Final_report/data/imagenet_classes.txt"
        };
        ifstream fin;
        for (int i = 0; i < 5; i++)
        {
            fin.open(paths[i]);
            if (fin.is_open())
                break;
            fin.clear();
        }
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

    void run_output_writeback_test()
    {
        cout << "[TEST] Running output write-back DMA test." << endl;
        vector<float> test_scores(1000, 0.0f);
        for (int i = 0; i < 1000; i++)
            test_scores[i] = (float)(1000 - i) / 100.0f;

        dma_write_vector(DRAM_OUTPUT_BASE, test_scores, "test output scores");
        vector<float> readback = dma_read_vector(DRAM_OUTPUT_BASE, 1000, "test output scores");

        cout << "[TEST] First 20 read-back scores:" << endl;
        cout << fixed << setprecision(2);
        for (int i = 0; i < 20; i++)
            cout << "idx " << setw(3) << i << " = " << setw(8) << readback[i] << endl;

        cout << "[TEST] Printing Top-100 from read-back DRAM output." << endl;
        vector<double> prob = softmax(readback);
        print_top100(readback, prob);
    }

    // Main Controller schedule:
    // read DRAM tensors through AXI DMA, partition each layer into PE jobs, collect results,
    // and advance through the AlexNet layer order.
    void run()
    {
        debug_log("Controller reset initialization.");
        next_job_id = 1;
        dma_cmd_valid.write(false);
        dma_cmd_write.write(false);
        dma_cmd_addr.write(0);
        dma_cmd_len.write(0);
        dma_read_ready.write(false);
        dma_write_data.write(0.0f);
        dma_write_valid.write(false);
        req_tx.write(false);
        set_rx_ready(false);
        flit_tx.write(0);

        wait();
        while (rst.read())
            wait();
        wait();
        debug_log("Reset deasserted. Starting AlexNet schedule.");

        const char *writeback_test = getenv("OUTPUT_WRITEBACK_TEST");
        if (writeback_test != NULL && string(writeback_test) == "1")
        {
            run_output_writeback_test();
            sc_stop();
            return;
        }

        // Input image -> zero padding.
        vector<float> feature = read_dram_tensor(0, false, IMG_H * IMG_W * IMG_C);
        feature = zero_pad_224_to_227(feature);
        debug_log("Finished input zero padding.");

        // conv1 -> ReLU in PE -> pool1 in PE.
        debug_log("Entering conv1/pool1 block.");
        feature = run_conv_on_pes(feature, 1, 227, 227, 3, 64, 11, 4);
        feature = run_pool_on_pes(feature, 55, 55, 64, 3, 2);

        // conv2 block: padding in Controller, conv/ReLU/pool in PEs.
        debug_log("Entering conv2/pool2 block.");
        feature = pad_layer(feature, 27, 27, 64, 2);
        feature = run_conv_on_pes(feature, 2, 31, 31, 64, 192, 5, 1);
        feature = run_pool_on_pes(feature, 27, 27, 192, 3, 2);

        // conv3 block.
        debug_log("Entering conv3 block.");
        feature = pad_layer(feature, 13, 13, 192, 1);
        feature = run_conv_on_pes(feature, 3, 15, 15, 192, 384, 3, 1);

        // conv4 block.
        debug_log("Entering conv4 block.");
        feature = pad_layer(feature, 13, 13, 384, 1);
        feature = run_conv_on_pes(feature, 4, 15, 15, 384, 256, 3, 1);

        // conv5 block and final convolutional pooling.
        debug_log("Entering conv5/pool5 block.");
        feature = pad_layer(feature, 13, 13, 256, 1);
        feature = run_conv_on_pes(feature, 5, 15, 15, 256, 256, 3, 1);
        feature = run_pool_on_pes(feature, 13, 13, 256, 3, 2);

        // Fully connected classifier.
        debug_log("Entering fully connected classifier.");
        feature = run_fc_on_pes(feature, 6, 4096, true);
        feature = run_fc_on_pes(feature, 7, 4096, true);
        feature = run_fc_on_pes(feature, 8, 1000, false);

        // The final score vector is committed to DRAM before being checked or printed.
        dma_write_vector(DRAM_OUTPUT_BASE, feature, "fc8 output scores");
        vector<float> dram_output = dma_read_vector(DRAM_OUTPUT_BASE, 1000, "fc8 output scores");

        // Output conversion and report.
        debug_log("Computing softmax and printing Top-100 output.");
        vector<double> prob = softmax(dram_output);
        print_top100(dram_output, prob);
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
