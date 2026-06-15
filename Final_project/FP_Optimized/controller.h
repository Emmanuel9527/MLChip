#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "global_sram.h"
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
#include <map>
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
    sc_in<bool> reset_n;
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

    // Local NoC interface. The controller is connected to router 0 through
    // the HOST port; PE workers use router local ports 0 through 15.
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

    // Controller uses HOST_ID. Worker PEs use mesh ids 0..15.
    static const int CONTROLLER_ID = 16;
    static const int FIRST_WORKER = 0;
    static const int LAST_WORKER = 15;
    static const int WORKER_COUNT = 16;
    static const int BROADCAST_WORKERS = 65535;
    static const int LOAD_ACK_WORD = 7;
    static const int DEADLOCK_WATCHDOG_CYCLES = 200000;
    static const int WATCHDOG_IDLE_LOG_INTERVALS = 10;
    static const int UNEXPECTED_FLIT_LOG_LIMIT = 5;
    static const int UNEXPECTED_FLIT_LOG_INTERVAL = 1000;
    static const int DMA_PROGRESS_INTERVAL = 65536;
    static const int PACKET_PROGRESS_FLITS = 50000;
    static const int DMA_SERVICE_READ_VECTOR = 0;
    static const int DMA_SERVICE_WRITE_VECTOR = 1;
    static const int DMA_SERVICE_READ_TO_SRAM = 2;
    static const int DMA_SERVICE_WRITE_FROM_SRAM = 3;

    // Monotonic id used to label jobs sent to PEs.
    int next_job_id;
    vector<Packet> pending_packets;
    string wait_context;
    GlobalSram global_sram;
    unsigned long long optimized_dram_read_words;
    unsigned long long optimized_dram_write_words;
    unsigned long long optimized_sram_wait_cycles;
    unsigned long long optimized_noc_payload_words;
    unsigned long long optimized_systolic_wait_cycles;
    unsigned long long optimized_systolic_mac_ops;
    unsigned long long optimized_systolic_tiles;
    bool fc_prefetch_request;
    bool fc_prefetch_busy;
    bool fc_prefetch_done;
    int fc_prefetch_layer;
    int fc_prefetch_in_size;
    int fc_prefetch_out_base;
    int fc_prefetch_active_outputs;
    int fc_prefetch_input_base;
    int fc_prefetch_tile_words;
    int fc_prefetch_buffer_id;
    // DMA descriptor/status register model. The Controller writes these
    // registers to request one DMA transaction. The dma_service_thread is the
    // only process that drives the actual DMA pins.
    sc_event dma_service_request_event;
    sc_event dma_service_done_event;
    bool dma_service_request;
    bool dma_service_busy;
    int dma_service_mode;
    bool dma_service_write;
    bool dma_service_quiet;
    unsigned int dma_service_addr;
    unsigned int dma_service_sram_addr;
    unsigned int dma_service_words;
    string dma_service_name;
    vector<float> dma_service_write_values;
    vector<float> dma_service_read_values;

    struct SramTag
    {
        bool valid;
        int layer;
        int output_index;
        int input_base;
        int words;
        int pe;
        int buffer_id;

        SramTag() :
            valid(false),
            layer(-1),
            output_index(-1),
            input_base(-1),
            words(0),
            pe(-1),
            buffer_id(-1) {}
    };

    map<unsigned int, SramTag> fc_weight_tags;

    static const unsigned int SRAM_CAPACITY_WORDS = 2097152;
    static const unsigned int SRAM_FC_INPUT_BASE = 0;
    static const unsigned int SRAM_FC_WEIGHT_PING_BASE = 65536;
    static const unsigned int SRAM_FC_WEIGHT_PONG_BASE = 131072;
    static const unsigned int SRAM_FC_BIAS_TILE_BASE = 196608;
    static const unsigned int SRAM_FC_PSUM_BASE = 200704;
    static const unsigned int SRAM_FC_OUTPUT_BASE = 204800;
    static const int SYSTOLIC_ARRAY_ROWS = 4;
    static const int SYSTOLIC_ARRAY_COLS = 4;
    static const int SYSTOLIC_SUBARRAY_ROWS = 2;
    static const int SYSTOLIC_SUBARRAYS = 2;
    static const int SYSTOLIC_OUTPUTS_PER_TILE = 8;
    static const int PE_MACS_PER_CYCLE = 64;
    static const int SYSTOLIC_INPUT_TILE_WORDS = 4096;
    static const int CONTROLLER_FC_TILE_REGISTER_WORDS = SYSTOLIC_OUTPUTS_PER_TILE;

    vector<float> make_fc_tile_register(int words,
                                        const string &name,
                                        float init_value = 0.0f)
    {
        if (words > CONTROLLER_FC_TILE_REGISTER_WORDS)
        {
            cout << "[ERROR] Controller FC tile register overflow for "
                 << name << ": words=" << words
                 << ", capacity=" << CONTROLLER_FC_TILE_REGISTER_WORDS << endl;
            sc_stop();
            return vector<float>();
        }
        return vector<float>(words, init_value);
    }

    unsigned int fc_weight_buffer_base(int buffer_id) const
    {
        return (buffer_id == 0) ? SRAM_FC_WEIGHT_PING_BASE
                                : SRAM_FC_WEIGHT_PONG_BASE;
    }

    void tag_fc_weight_sram(unsigned int addr,
                            int layer,
                            int output_index,
                            int input_base,
                            int words,
                            int pe,
                            int buffer_id)
    {
        SramTag tag;
        tag.valid = true;
        tag.layer = layer;
        tag.output_index = output_index;
        tag.input_base = input_base;
        tag.words = words;
        tag.pe = pe;
        tag.buffer_id = buffer_id;
        fc_weight_tags[addr] = tag;
    }

    void check_fc_weight_sram(unsigned int addr,
                              int layer,
                              int output_index,
                              int input_base,
                              int words,
                              int pe,
                              int buffer_id)
    {
        map<unsigned int, SramTag>::iterator it = fc_weight_tags.find(addr);
        bool ok = it != fc_weight_tags.end() &&
                  it->second.valid &&
                  it->second.layer == layer &&
                  it->second.output_index == output_index &&
                  it->second.input_base == input_base &&
                  it->second.words == words &&
                  it->second.pe == pe &&
                  it->second.buffer_id == buffer_id;
        if (ok)
            return;

        cout << "[ERROR] FC weight SRAM tag mismatch at addr=" << addr
             << ", expected layer=" << layer
             << ", output=" << output_index
             << ", input_base=" << input_base
             << ", words=" << words
             << ", PE=" << pe
             << ", buffer=" << buffer_id << endl;
        if (it != fc_weight_tags.end())
        {
            cout << "[ERROR] Found tag layer=" << it->second.layer
                 << ", output=" << it->second.output_index
                 << ", input_base=" << it->second.input_base
                 << ", words=" << it->second.words
                 << ", PE=" << it->second.pe
                 << ", buffer=" << it->second.buffer_id << endl;
        }
        else
        {
            cout << "[ERROR] No tag found for this SRAM address." << endl;
        }
        sc_stop();
    }

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

    void progress_log(const string &msg)
    {
#if defined(FP_TRACE)
        cout << "[TRACE] " << msg << endl;
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

    vector<float> dma_read_vector_direct(unsigned int addr,
                                         int expected,
                                         const string &name,
                                         bool quiet_detail)
    {
        vector<float> values;
        values.reserve(expected);
        optimized_dram_read_words += expected;
        if (!quiet_detail)
            debug_log(string("DMA read request: ") + name +
                      ", words=" + num_to_string(expected) + ".");
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

        if (!quiet_detail)
            debug_log(string("Finished DMA read: ") + name + ", " +
                      num_to_string(values.size()) + " values.");
        return values;
    }

    void dma_write_vector_direct(unsigned int addr,
                                 const vector<float> &values,
                                 const string &name,
                                 bool quiet_detail)
    {
        optimized_dram_write_words += values.size();
        if (!quiet_detail)
            debug_log(string("DMA write request: ") + name + ", words=" +
                      num_to_string(values.size()) + ".");
        start_dma_command(true, addr, (unsigned int)values.size());

        for (size_t i = 0; i < values.size(); i++)
        {
            dma_write_data.write(values[i]);
            dma_write_valid.write(true);
            do
            {
                wait();
            } while (!dma_write_ready.read());
            dma_write_valid.write(false);
        }
        while (!dma_done.read())
            wait();
        if (!quiet_detail)
            debug_log(string("Finished DMA write: ") + name + ".");
    }

    void dma_read_to_sram_direct(unsigned int dram_addr,
                                 unsigned int sram_addr,
                                 unsigned int words,
                                 const string &name,
                                 bool quiet_detail)
    {
        if (!global_sram.can_hold(sram_addr, words))
        {
            cout << "[ERROR] DMA read-to-SRAM range overflow for " << name
                 << ": sram_addr=" << sram_addr
                 << ", words=" << words << endl;
            sc_stop();
            return;
        }
        optimized_dram_read_words += words;
        if (!quiet_detail)
            debug_log(string("DMA read-to-SRAM request: ") + name +
                      ", words=" + num_to_string(words) + ".");
        start_dma_command(false, dram_addr, words);

        unsigned int received = 0;
        unsigned int accumulated_sram_cycles = 0;
        while (received < words)
        {
            dma_read_ready.write(true);
            wait();
            if (dma_read_valid.read())
            {
                accumulated_sram_cycles +=
                    global_sram.write_word(sram_addr + received,
                                           dma_read_data.read());
                received++;
                if (words >= DMA_PROGRESS_INTERVAL &&
                    received % DMA_PROGRESS_INTERVAL == 0)
                {
                    debug_log(string("DMA read-to-SRAM progress: ") + name +
                              ", " + num_to_string(received) + "/" +
                              num_to_string(words) + " values.");
                }
            }
        }
        dma_read_ready.write(false);
        wait_sram_cycles(accumulated_sram_cycles);
        while (!dma_done.read())
            wait();

        if (!quiet_detail)
            debug_log(string("Finished DMA read-to-SRAM: ") + name + ".");
    }

    void dma_write_from_sram_direct(unsigned int dram_addr,
                                    unsigned int sram_addr,
                                    unsigned int words,
                                    const string &name,
                                    bool quiet_detail)
    {
        if (!global_sram.can_hold(sram_addr, words))
        {
            cout << "[ERROR] DMA write-from-SRAM range overflow for " << name
                 << ": sram_addr=" << sram_addr
                 << ", words=" << words << endl;
            sc_stop();
            return;
        }
        optimized_dram_write_words += words;
        if (!quiet_detail)
            debug_log(string("DMA write-from-SRAM request: ") + name +
                      ", words=" + num_to_string(words) + ".");
        start_dma_command(true, dram_addr, words);

        unsigned int accumulated_sram_cycles = 0;
        for (unsigned int i = 0; i < words; i++)
        {
            float value = 0.0f;
            accumulated_sram_cycles +=
                global_sram.read_word(sram_addr + i, value);
            dma_write_data.write(value);
            dma_write_valid.write(true);
            do
            {
                wait();
            } while (!dma_write_ready.read());
            dma_write_valid.write(false);
        }
        wait_sram_cycles(accumulated_sram_cycles);
        while (!dma_done.read())
            wait();
        if (!quiet_detail)
            debug_log(string("Finished DMA write-from-SRAM: ") + name + ".");
    }

    vector<float> dma_read_vector(unsigned int addr,
                                  int expected,
                                  const string &name,
                                  bool quiet_detail = false)
    {
        while (dma_service_busy || dma_service_request)
            wait();
        dma_service_mode = DMA_SERVICE_READ_VECTOR;
        dma_service_write = false;
        dma_service_quiet = quiet_detail;
        dma_service_addr = addr;
        dma_service_sram_addr = 0;
        dma_service_words = (unsigned int)expected;
        dma_service_name = name;
        dma_service_write_values.clear();
        dma_service_read_values.clear();
        dma_service_request = true;
        dma_service_request_event.notify(SC_ZERO_TIME);
        wait(dma_service_done_event);
        return dma_service_read_values;
    }

    void dma_write_vector(unsigned int addr,
                          const vector<float> &values,
                          const string &name,
                          bool quiet_detail = false)
    {
        while (dma_service_busy || dma_service_request)
            wait();
        dma_service_mode = DMA_SERVICE_WRITE_VECTOR;
        dma_service_write = true;
        dma_service_quiet = quiet_detail;
        dma_service_addr = addr;
        dma_service_sram_addr = 0;
        dma_service_words = (unsigned int)values.size();
        dma_service_name = name;
        dma_service_write_values = values;
        dma_service_read_values.clear();
        dma_service_request = true;
        dma_service_request_event.notify(SC_ZERO_TIME);
        wait(dma_service_done_event);
    }

    void dma_read_to_sram(unsigned int dram_addr,
                          unsigned int sram_addr,
                          unsigned int words,
                          const string &name,
                          bool quiet_detail = false)
    {
        while (dma_service_busy || dma_service_request)
            wait();
        dma_service_mode = DMA_SERVICE_READ_TO_SRAM;
        dma_service_write = false;
        dma_service_quiet = quiet_detail;
        dma_service_addr = dram_addr;
        dma_service_sram_addr = sram_addr;
        dma_service_words = words;
        dma_service_name = name;
        dma_service_write_values.clear();
        dma_service_read_values.clear();
        dma_service_request = true;
        dma_service_request_event.notify(SC_ZERO_TIME);
        wait(dma_service_done_event);
    }

    void dma_write_from_sram(unsigned int dram_addr,
                             unsigned int sram_addr,
                             unsigned int words,
                             const string &name,
                             bool quiet_detail = false)
    {
        while (dma_service_busy || dma_service_request)
            wait();
        dma_service_mode = DMA_SERVICE_WRITE_FROM_SRAM;
        dma_service_write = true;
        dma_service_quiet = quiet_detail;
        dma_service_addr = dram_addr;
        dma_service_sram_addr = sram_addr;
        dma_service_words = words;
        dma_service_name = name;
        dma_service_write_values.clear();
        dma_service_read_values.clear();
        dma_service_request = true;
        dma_service_request_event.notify(SC_ZERO_TIME);
        wait(dma_service_done_event);
    }

    void dma_service_thread()
    {
        dma_cmd_valid.write(false);
        dma_cmd_write.write(false);
        dma_cmd_addr.write(0);
        dma_cmd_len.write(0);
        dma_read_ready.write(false);
        dma_write_data.write(0.0f);
        dma_write_valid.write(false);

        while (!reset_n.read())
            wait();

        while (true)
        {
            if (!dma_service_request)
                wait(dma_service_request_event);

            dma_service_request = false;
            dma_service_busy = true;
            if (dma_service_mode == DMA_SERVICE_WRITE_VECTOR)
            {
                dma_write_vector_direct(dma_service_addr,
                                        dma_service_write_values,
                                        dma_service_name,
                                        dma_service_quiet);
            }
            else if (dma_service_mode == DMA_SERVICE_READ_VECTOR)
            {
                dma_service_read_values =
                    dma_read_vector_direct(dma_service_addr,
                                           (int)dma_service_words,
                                           dma_service_name,
                                           dma_service_quiet);
            }
            else if (dma_service_mode == DMA_SERVICE_READ_TO_SRAM)
            {
                dma_read_to_sram_direct(dma_service_addr,
                                        dma_service_sram_addr,
                                        dma_service_words,
                                        dma_service_name,
                                        dma_service_quiet);
            }
            else
            {
                dma_write_from_sram_direct(dma_service_addr,
                                           dma_service_sram_addr,
                                           dma_service_words,
                                           dma_service_name,
                                           dma_service_quiet);
            }
            dma_service_busy = false;
            dma_service_done_event.notify(SC_ZERO_TIME);
        }
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

    void wait_sram_cycles(unsigned int cycles)
    {
        optimized_sram_wait_cycles += cycles;
        // The full cycle cost is counted in metrics. The simulator advances
        // one synchronization cycle per SRAM block access to keep long FC
        // layers practical at behavioral simulation speed.
        if (cycles > 0)
            wait();
    }

    vector<float> sram_stage_block(unsigned int sram_addr,
                                   const vector<float> &data,
                                   const string &name,
                                   bool quiet_detail = false)
    {
        if (!quiet_detail)
            debug_log(string("SRAM stage write: ") + name +
                      ", words=" + num_to_string(data.size()) + ".");
        wait_sram_cycles(global_sram.write_block(sram_addr, data));

        vector<float> out;
        wait_sram_cycles(global_sram.read_block(sram_addr,
                                                (unsigned int)data.size(),
                                                out));
        if (!quiet_detail)
            debug_log(string("SRAM stage read: ") + name +
                      ", words=" + num_to_string(out.size()) + ".");
        return out;
    }

    void sram_write_only(unsigned int sram_addr,
                         const vector<float> &data,
                         const string &name,
                         bool quiet_detail = false)
    {
        if (!quiet_detail)
            debug_log(string("SRAM write: ") + name +
                      ", words=" + num_to_string(data.size()) + ".");
        wait_sram_cycles(global_sram.write_block(sram_addr, data));
    }

    vector<float> sram_read_only(unsigned int sram_addr,
                                 unsigned int words,
                                 const string &name,
                                 bool quiet_detail = false)
    {
        vector<float> out;
        wait_sram_cycles(global_sram.read_block(sram_addr, words, out));
        if (!quiet_detail)
            debug_log(string("SRAM read: ") + name +
                      ", words=" + num_to_string(out.size()) + ".");
        return out;
    }

    void wait_systolic_cycles(unsigned int cycles)
    {
        optimized_systolic_wait_cycles += cycles;
        if (cycles > 0)
            wait();
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
        optimized_noc_payload_words += packet.datas.size();
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
            ack_op != OP_LOAD_BIAS &&
            ack_op != OP_SYSTOLIC_BCAST_INPUT)
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

    void wait_for_load_ack_op(int expected_source,
                              int expected_op,
                              const string &label,
                              bool quiet_detail = false)
    {
        set_wait_context(label + " ACK");
        while (true)
        {
            for (size_t i = 0; i < pending_packets.size(); i++)
            {
                if (packet_is_load_ack_op(pending_packets[i], expected_source, expected_op))
                {
                    Packet ack = take_pending_packet(i);
                    if (!quiet_detail)
                        debug_log(string("Received ") + label + " ACK from PE " +
                                  num_to_string(ack.source_id) +
                                  ", original op " +
                                  num_to_string((int)ack.datas[1]) + ".");
                    return;
                }
            }

            Packet packet = receive_packet();
            if (packet_is_load_ack_op(packet, expected_source, expected_op))
            {
                if (!quiet_detail)
                    debug_log(string("Received ") + label + " ACK from PE " +
                              num_to_string(packet.source_id) +
                              ", original op " +
                              num_to_string((int)packet.datas[1]) + ".");
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

    int systolic_result_index(const Packet &packet,
                              const vector<int> &job_ids,
                              const vector<int> &sources,
                              const vector<bool> &received)
    {
        if (packet.datas.empty())
            return -1;

        int job_id = (int)packet.datas[0];
        for (size_t i = 0; i < job_ids.size(); i++)
        {
            if (received[i])
                continue;
            if (job_ids[i] == job_id && sources[i] == packet.source_id)
                return (int)i;
        }
        return -1;
    }

    void collect_systolic_results(const vector<int> &job_ids,
                                  const vector<int> &sources,
                                  vector<float> &psum_tile)
    {
        vector<bool> received(job_ids.size(), false);
        int received_count = 0;

        while (received_count < (int)job_ids.size())
        {
            bool found_pending = false;
            for (size_t i = 0; i < pending_packets.size(); i++)
            {
                int index =
                    systolic_result_index(pending_packets[i],
                                          job_ids,
                                          sources,
                                          received);
                if (index < 0)
                    continue;

                Packet result = take_pending_packet(i);
                if (result.datas.size() > 2)
                    psum_tile[index] = result.datas[2];
                received[index] = true;
                received_count++;
                found_pending = true;
                break;
            }
            if (found_pending)
                continue;

            Packet packet = receive_packet();
            int index = systolic_result_index(packet, job_ids, sources, received);
            if (index >= 0)
            {
                if (packet.datas.size() > 2)
                    psum_tile[index] = packet.datas[2];
                received[index] = true;
                received_count++;
            }
            else
            {
                pending_packets.push_back(packet);
                debug_log(string("Buffering packet from PE ") +
                          num_to_string(packet.source_id) +
                          " while collecting FC systolic results.");
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
        packet.source_id = CONTROLLER_ID;
        packet.dest_id = dest;
        packet.datas.push_back((float)op);
        packet.datas.push_back((float)layer);
        packet.datas.push_back((float)payload.size());
        append_vector(packet.datas, payload);
        return packet;
    }

    // Build a lecture-style MVM input packet. The west PE in one row loads
    // this b-vector segment and forwards it horizontally to the active columns.
    // Layout: [op, layer_id, active_cols, payload_size, payload...]
    Packet make_systolic_bcast_input(int dest,
                                     int layer,
                                     int active_cols,
                                     const vector<float> &payload)
    {
        Packet packet;
        packet.source_id = CONTROLLER_ID;
        packet.dest_id = dest;
        packet.datas.push_back((float)OP_SYSTOLIC_BCAST_INPUT);
        packet.datas.push_back((float)layer);
        packet.datas.push_back((float)active_cols);
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
        packet.source_id = CONTROLLER_ID;
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
        packet.source_id = CONTROLLER_ID;
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
        packet.source_id = CONTROLLER_ID;
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

    Packet make_systolic_fc_init(int dest, int job_id, int output_index, float bias)
    {
        Packet packet;
        packet.source_id = CONTROLLER_ID;
        packet.dest_id = dest;
        packet.datas.push_back((float)OP_SYSTOLIC_FC_INIT);
        packet.datas.push_back((float)job_id);
        packet.datas.push_back((float)output_index);
        packet.datas.push_back(bias);
        return packet;
    }

    Packet make_systolic_fc_accum(int dest, int job_id, int tile_words)
    {
        Packet packet;
        packet.source_id = CONTROLLER_ID;
        packet.dest_id = dest;
        packet.datas.push_back((float)OP_SYSTOLIC_FC_ACCUM);
        packet.datas.push_back((float)job_id);
        packet.datas.push_back((float)tile_words);
        return packet;
    }

    Packet make_systolic_fc_finish(int dest, int job_id, bool apply_relu)
    {
        Packet packet;
        packet.source_id = CONTROLLER_ID;
        packet.dest_id = dest;
        packet.datas.push_back((float)OP_SYSTOLIC_FC_FINISH);
        packet.datas.push_back((float)job_id);
        packet.datas.push_back(apply_relu ? 1.0f : 0.0f);
        return packet;
    }

    Packet make_systolic_mvm_accum(int dest,
                                   int job_id,
                                   int output_index,
                                   bool apply_relu,
                                   int top_boundary_row,
                                   float psum_in)
    {
        Packet packet;
        packet.source_id = CONTROLLER_ID;
        packet.dest_id = dest;
        packet.datas.push_back((float)OP_SYSTOLIC_MVM_ACCUM);
        packet.datas.push_back((float)job_id);
        packet.datas.push_back((float)output_index);
        packet.datas.push_back((float)CONTROLLER_ID);
        packet.datas.push_back(apply_relu ? 1.0f : 0.0f);
        packet.datas.push_back((float)top_boundary_row);
        packet.datas.push_back(psum_in);
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

    void prefetch_fc_weight_tile_now(int layer,
                                     int in_size,
                                     int out_base,
                                     int active_outputs,
                                     int input_base,
                                     int tile_words,
                                     int buffer_id)
    {
        unsigned int weight_buffer_base = fc_weight_buffer_base(buffer_id);

        for (int sub = 0; sub < SYSTOLIC_SUBARRAYS; sub++)
        {
            int group_base = sub * SYSTOLIC_ARRAY_COLS;
            int remaining_group_outputs = active_outputs - group_base;
            if (remaining_group_outputs <= 0)
                continue;
            int active_cols = (remaining_group_outputs < SYSTOLIC_ARRAY_COLS)
                                  ? remaining_group_outputs
                                  : SYSTOLIC_ARRAY_COLS;
            int top_boundary_row = sub * SYSTOLIC_SUBARRAY_ROWS;
            int bottom_row = top_boundary_row + SYSTOLIC_SUBARRAY_ROWS - 1;

            for (int data_row = 0; data_row < SYSTOLIC_SUBARRAY_ROWS; data_row++)
            {
                int seg_start = (tile_words * data_row) / SYSTOLIC_SUBARRAY_ROWS;
                int seg_end = (tile_words * (data_row + 1)) / SYSTOLIC_SUBARRAY_ROWS;
                int seg_words = seg_end - seg_start;
                int global_input_start = input_base + seg_start;
                int physical_row = bottom_row - data_row;

                for (int col = 0; col < active_cols; col++)
                {
                    int local_out = group_base + col;
                    int output_index = out_base + local_out;
                    int pe = physical_row * SYSTOLIC_ARRAY_COLS + col;
                    unsigned int weight_addr =
                        dram_weight_base(layer) +
                        ((unsigned int)output_index * (unsigned int)in_size +
                         (unsigned int)global_input_start) *
                            4u;
                    unsigned int sram_addr =
                        weight_buffer_base +
                        (unsigned int)pe * SYSTOLIC_INPUT_TILE_WORDS;
                    dma_read_to_sram(weight_addr,
                                     sram_addr,
                                     (unsigned int)seg_words,
                                     string("FC layer ") +
                                         num_to_string(layer) +
                                         " direct weight output " +
                                         num_to_string(output_index) +
                                         " PE " + num_to_string(pe),
                                     true);
                    tag_fc_weight_sram(sram_addr,
                                       layer,
                                       output_index,
                                       global_input_start,
                                       seg_words,
                                       pe,
                                       buffer_id);
                }
            }
        }
    }

    void start_fc_weight_prefetch(int layer,
                                  int in_size,
                                  int out_base,
                                  int active_outputs,
                                  int input_base,
                                  int tile_words,
                                  int buffer_id)
    {
        while (fc_prefetch_busy || fc_prefetch_request)
            wait();
        fc_prefetch_layer = layer;
        fc_prefetch_in_size = in_size;
        fc_prefetch_out_base = out_base;
        fc_prefetch_active_outputs = active_outputs;
        fc_prefetch_input_base = input_base;
        fc_prefetch_tile_words = tile_words;
        fc_prefetch_buffer_id = buffer_id;
        fc_prefetch_done = false;
        fc_prefetch_request = true;
    }

    void wait_fc_weight_prefetch()
    {
        while (!fc_prefetch_done)
            wait();
        fc_prefetch_done = false;
    }

    void fc_weight_prefetch_thread()
    {
        while (!reset_n.read())
            wait();

        while (true)
        {
            if (!fc_prefetch_request)
            {
                wait();
                continue;
            }

            fc_prefetch_request = false;
            fc_prefetch_busy = true;
            prefetch_fc_weight_tile_now(fc_prefetch_layer,
                                        fc_prefetch_in_size,
                                        fc_prefetch_out_base,
                                        fc_prefetch_active_outputs,
                                        fc_prefetch_input_base,
                                        fc_prefetch_tile_words,
                                        fc_prefetch_buffer_id);
            fc_prefetch_busy = false;
            fc_prefetch_done = true;
            wait();
        }
    }

    // Run one fully connected layer through the lecture-style MVM systolic map.
    // The bulk activation tensors live in Global SRAM. Controller-side state
    // for this path is limited to descriptor registers, loop counters, and
    // small tile registers such as bias/partial-sum/output tiles.
    void run_fc_on_pes_sram(unsigned int input_sram_base,
                            int in_size,
                            unsigned int output_sram_base,
                            int layer,
                            int out_size,
                            bool apply_relu)
    {
        debug_log(string("Starting PE-based systolic FC layer ") + num_to_string(layer) + ".");
        fc_weight_tags.clear();

        if (!global_sram.can_hold(input_sram_base, (unsigned int)in_size) ||
            !global_sram.can_hold(output_sram_base, (unsigned int)out_size))
        {
            cout << "[ERROR] FC SRAM activation range overflow for layer "
                 << layer << endl;
            sc_stop();
            return;
        }

        for (int out_base = 0; out_base < out_size; out_base += SYSTOLIC_OUTPUTS_PER_TILE)
        {
            int remaining_outputs = out_size - out_base;
            int active_outputs = (remaining_outputs < SYSTOLIC_OUTPUTS_PER_TILE)
                                     ? remaining_outputs
                                     : SYSTOLIC_OUTPUTS_PER_TILE;

            unsigned int bias_addr = dram_bias_base(layer) + (unsigned int)out_base * 4u;
            dma_read_to_sram(bias_addr,
                             SRAM_FC_BIAS_TILE_BASE,
                             (unsigned int)active_outputs,
                             string("FC layer ") + num_to_string(layer) +
                                 " direct bias tile [" +
                                 num_to_string(out_base) + ", " +
                                 num_to_string(out_base + active_outputs - 1) + "]",
                             true);
            vector<float> bias_tile =
                sram_read_only(SRAM_FC_BIAS_TILE_BASE,
                               (unsigned int)active_outputs,
                               string("FC layer ") + num_to_string(layer) +
                                   " bias tile",
                               true);

            vector<float> psum_tile =
                make_fc_tile_register(active_outputs,
                                      string("FC layer ") +
                                          num_to_string(layer) +
                                          " psum register");
            for (int local_out = 0; local_out < active_outputs; local_out++)
                psum_tile[local_out] = bias_tile[local_out];
            sram_write_only(SRAM_FC_PSUM_BASE, psum_tile,
                            string("FC layer ") + num_to_string(layer) +
                                " initial partial sums",
                            true);

            if (in_size > 0)
            {
                int first_tile_words = (in_size < SYSTOLIC_INPUT_TILE_WORDS)
                                           ? in_size
                                           : SYSTOLIC_INPUT_TILE_WORDS;
                start_fc_weight_prefetch(layer,
                                         in_size,
                                         out_base,
                                         active_outputs,
                                         0,
                                         first_tile_words,
                                         0);
            }

            for (int input_base = 0; input_base < in_size;
                 input_base += SYSTOLIC_INPUT_TILE_WORDS)
            {
                wait_fc_weight_prefetch();
                int input_tile_id = input_base / SYSTOLIC_INPUT_TILE_WORDS;
                int weight_buffer_id = input_tile_id & 1;
                unsigned int weight_buffer_base =
                    fc_weight_buffer_base(weight_buffer_id);
                int remaining_inputs = in_size - input_base;
                int tile_words = (remaining_inputs < SYSTOLIC_INPUT_TILE_WORDS)
                                     ? remaining_inputs
                                     : SYSTOLIC_INPUT_TILE_WORDS;

                for (int sub = 0; sub < SYSTOLIC_SUBARRAYS; sub++)
                {
                    int group_base = sub * SYSTOLIC_ARRAY_COLS;
                    int remaining_group_outputs = active_outputs - group_base;
                    if (remaining_group_outputs <= 0)
                        continue;
                    int active_cols = (remaining_group_outputs < SYSTOLIC_ARRAY_COLS)
                                          ? remaining_group_outputs
                                          : SYSTOLIC_ARRAY_COLS;
                    int top_boundary_row = sub * SYSTOLIC_SUBARRAY_ROWS;
                    int bottom_row = top_boundary_row + SYSTOLIC_SUBARRAY_ROWS - 1;

                    for (int data_row = 0; data_row < SYSTOLIC_SUBARRAY_ROWS; data_row++)
                    {
                        int seg_start = (tile_words * data_row) / SYSTOLIC_SUBARRAY_ROWS;
                        int seg_end = (tile_words * (data_row + 1)) / SYSTOLIC_SUBARRAY_ROWS;
                        int seg_words = seg_end - seg_start;
                        int global_input_start = input_base + seg_start;
                        int physical_row = bottom_row - data_row;
                        int west_pe = physical_row * SYSTOLIC_ARRAY_COLS;
                        int last_active_pe = west_pe + active_cols - 1;

                        vector<float> input_segment =
                            sram_read_only(input_sram_base +
                                               (unsigned int)global_input_start,
                                           seg_words,
                                           string("FC layer ") + num_to_string(layer) +
                                               " sub-array " + num_to_string(sub) +
                                               " data row " +
                                               num_to_string(data_row) +
                                               " horizontal input segment",
                                           true);
                        send_packet(make_systolic_bcast_input(west_pe, layer,
                                                              active_cols,
                                                              input_segment));
                        wait_for_load_ack_op(last_active_pe, OP_SYSTOLIC_BCAST_INPUT,
                                             string("FC layer ") +
                                                 num_to_string(layer) +
                                                 " horizontal input sub-array " +
                                                 num_to_string(sub) +
                                                 " row " +
                                                 num_to_string(data_row),
                                             true);

                        for (int col = 0; col < active_cols; col++)
                        {
                            int local_out = group_base + col;
                            int output_index = out_base + local_out;
                            int pe = physical_row * SYSTOLIC_ARRAY_COLS + col;
                            unsigned int weight_sram_addr =
                                weight_buffer_base +
                                (unsigned int)pe *
                                    SYSTOLIC_INPUT_TILE_WORDS;
                            check_fc_weight_sram(weight_sram_addr,
                                                 layer,
                                                 output_index,
                                                 global_input_start,
                                                 seg_words,
                                                 pe,
                                                 weight_buffer_id);

                            vector<float> staged_weight =
                                sram_read_only(weight_sram_addr,
                                               seg_words,
                                               string("FC layer ") +
                                                   num_to_string(layer) +
                                                   " PE " + num_to_string(pe) +
                                                   " prefetched weight segment",
                                               true);
                            send_packet(make_load_packet(pe, OP_LOAD_WEIGHT, layer,
                                                         staged_weight));
                            wait_for_load_ack_op(pe, OP_LOAD_WEIGHT,
                                                 string("FC layer ") +
                                                     num_to_string(layer) +
                                                     " systolic weight for PE " +
                                                     num_to_string(pe),
                                                 true);
                        }
                    }
                }

                int next_input_base = input_base + SYSTOLIC_INPUT_TILE_WORDS;
                if (next_input_base < in_size)
                {
                    int remaining_next_inputs = in_size - next_input_base;
                    int next_tile_words =
                        (remaining_next_inputs < SYSTOLIC_INPUT_TILE_WORDS)
                            ? remaining_next_inputs
                            : SYSTOLIC_INPUT_TILE_WORDS;
                    start_fc_weight_prefetch(layer,
                                             in_size,
                                             out_base,
                                             active_outputs,
                                             next_input_base,
                                             next_tile_words,
                                             (input_tile_id + 1) & 1);
                }

                vector<int> job_ids(active_outputs, 0);
                vector<int> result_sources(active_outputs, 0);
                bool apply_relu_this_tile =
                    apply_relu && (input_base + tile_words >= in_size);
                for (int local_out = 0; local_out < active_outputs; local_out++)
                {
                    int sub = local_out / SYSTOLIC_ARRAY_COLS;
                    int col = local_out % SYSTOLIC_ARRAY_COLS;
                    int top_boundary_row = sub * SYSTOLIC_SUBARRAY_ROWS;
                    int bottom_row = top_boundary_row + SYSTOLIC_SUBARRAY_ROWS - 1;
                    int job_id = next_job_id++;
                    job_ids[local_out] = job_id;
                    result_sources[local_out] =
                        top_boundary_row * SYSTOLIC_ARRAY_COLS + col;
                    int bottom_pe = bottom_row * SYSTOLIC_ARRAY_COLS + col;
                    send_packet(make_systolic_mvm_accum(bottom_pe, job_id,
                                                        out_base + local_out,
                                                        apply_relu_this_tile,
                                                        top_boundary_row,
                                                        psum_tile[local_out]));
                }
                collect_systolic_results(job_ids, result_sources, psum_tile);
                optimized_systolic_mac_ops +=
                    (unsigned long long)active_outputs *
                    (unsigned long long)tile_words;

                unsigned int systolic_cycles =
                    (tile_words +
                     PE_MACS_PER_CYCLE * SYSTOLIC_SUBARRAY_ROWS - 1) /
                        (PE_MACS_PER_CYCLE * SYSTOLIC_SUBARRAY_ROWS) +
                    SYSTOLIC_SUBARRAY_ROWS + SYSTOLIC_ARRAY_COLS - 2;
                optimized_systolic_tiles++;
                wait_systolic_cycles(systolic_cycles);

                sram_write_only(SRAM_FC_PSUM_BASE, psum_tile,
                                string("FC layer ") + num_to_string(layer) +
                                    " updated partial sums",
                                true);
            }

            vector<float> output_tile =
                make_fc_tile_register(active_outputs,
                                      string("FC layer ") +
                                          num_to_string(layer) +
                                          " output register");
            for (int local_out = 0; local_out < active_outputs; local_out++)
                output_tile[local_out] = psum_tile[local_out];

            sram_write_only(output_sram_base + (unsigned int)out_base,
                            output_tile,
                            string("FC layer ") + num_to_string(layer) +
                                " output tile",
                            true);

            int done_outputs = out_base + active_outputs;
            unsigned long long done_macs =
                (unsigned long long)done_outputs *
                (unsigned long long)in_size;
            unsigned long long total_macs =
                (unsigned long long)out_size *
                (unsigned long long)in_size;
            progress_log(string("FC layer ") + num_to_string(layer) +
                         " progress: outputs " +
                         num_to_string(done_outputs) + "/" +
                         num_to_string(out_size) +
                         ", MACs " +
                         num_to_string(done_macs) + "/" +
                         num_to_string(total_macs) +
                         ", weight ping-pong buffers enabled" +
                         ".");
        }

        debug_log(string("Finished PE-based systolic FC layer ") + num_to_string(layer) + ".");
    }

    // Compatibility helper for experiments. The top-level optimized schedule
    // uses run_fc_on_pes_sram() so FC layer outputs stay in Global SRAM instead
    // of returning as large Controller buffers.
    vector<float> run_fc_on_pes(const vector<float> &feature,
                                int layer, int out_size, bool apply_relu)
    {
        sram_write_only(SRAM_FC_INPUT_BASE, feature,
                        string("FC layer ") + num_to_string(layer) +
                            " input activation");
        run_fc_on_pes_sram(SRAM_FC_INPUT_BASE,
                           (int)feature.size(),
                           SRAM_FC_OUTPUT_BASE,
                           layer,
                           out_size,
                           apply_relu);
        vector<float> output =
            sram_read_only(SRAM_FC_OUTPUT_BASE,
                           (unsigned int)out_size,
                           string("FC layer ") + num_to_string(layer) +
                               " output activation");
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
            "../../Final_report/data/imagenet_classes.txt"};
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

    void print_optimized_metrics()
    {
        cout << "========== Optimized Design Metrics ==========" << endl;
        cout << "DRAM read words: " << optimized_dram_read_words << endl;
        cout << "DRAM write words: " << optimized_dram_write_words << endl;
        cout << "NoC payload words: " << optimized_noc_payload_words << endl;
        cout << "Modeled SRAM wait cycles: " << optimized_sram_wait_cycles << endl;
        cout << "Modeled systolic wait cycles: " << optimized_systolic_wait_cycles << endl;
        cout << "Systolic MAC ops: " << optimized_systolic_mac_ops << endl;
        cout << "Systolic input tiles: " << optimized_systolic_tiles << endl;
        unsigned long long max_macs =
            optimized_systolic_wait_cycles *
            (unsigned long long)SYSTOLIC_ARRAY_ROWS *
            (unsigned long long)SYSTOLIC_ARRAY_COLS *
            (unsigned long long)PE_MACS_PER_CYCLE;
        double utilization = (max_macs == 0)
                                 ? 0.0
                                 : (double)optimized_systolic_mac_ops * 100.0 /
                                       (double)max_macs;
        cout << "Estimated systolic utilization (%): " << utilization << endl;
        cout << "FC optimization: original PE0..PE15 form a 4x4 systolic-style array with SRAM input, weight, bias, psum, and output buffers" << endl;
        cout << "==============================================" << endl;
        global_sram.print_metrics("Global Scratchpad");
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
        optimized_dram_read_words = 0;
        optimized_dram_write_words = 0;
        optimized_sram_wait_cycles = 0;
        optimized_noc_payload_words = 0;
        optimized_systolic_wait_cycles = 0;
        optimized_systolic_mac_ops = 0;
        optimized_systolic_tiles = 0;
        fc_prefetch_request = false;
        fc_prefetch_busy = false;
        fc_prefetch_done = false;
        dma_service_request = false;
        dma_service_busy = false;
        dma_service_mode = DMA_SERVICE_READ_VECTOR;
        global_sram.configure(SRAM_CAPACITY_WORDS, 32, 1, 1, SYSTOLIC_SUBARRAYS);
        req_tx.write(false);
        set_rx_ready(false);
        flit_tx.write(0);

        wait();
        while (!reset_n.read())
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
        sram_write_only(SRAM_FC_INPUT_BASE,
                        feature,
                        "FC layer 6 input activation");
        run_fc_on_pes_sram(SRAM_FC_INPUT_BASE,
                           (int)feature.size(),
                           SRAM_FC_OUTPUT_BASE,
                           6,
                           4096,
                           true);
        run_fc_on_pes_sram(SRAM_FC_OUTPUT_BASE,
                           4096,
                           SRAM_FC_INPUT_BASE,
                           7,
                           4096,
                           true);
        run_fc_on_pes_sram(SRAM_FC_INPUT_BASE,
                           4096,
                           SRAM_FC_OUTPUT_BASE,
                           8,
                           1000,
                           false);

        // The final score buffer is committed from Global SRAM to DRAM before
        // being read back and printed.
        dma_write_from_sram(DRAM_OUTPUT_BASE,
                            SRAM_FC_OUTPUT_BASE,
                            1000,
                            "fc8 output scores");
        dma_read_to_sram(DRAM_OUTPUT_BASE,
                         SRAM_FC_OUTPUT_BASE,
                         1000,
                         "fc8 output scores readback");
        vector<float> dram_output =
            sram_read_only(SRAM_FC_OUTPUT_BASE,
                           1000,
                           "fc8 output scores readback");

        // Output conversion and report.
        debug_log("Computing softmax and printing Top-100 output.");
        vector<double> prob = softmax(dram_output);
        print_top100(dram_output, prob);
        print_optimized_metrics();
        sc_stop();
    }

    // Register the Controller as one clocked SystemC thread.
    SC_CTOR(Controller)
    {
        fc_prefetch_request = false;
        fc_prefetch_busy = false;
        fc_prefetch_done = false;
        dma_service_request = false;
        dma_service_busy = false;
        dma_service_mode = DMA_SERVICE_READ_VECTOR;
        SC_THREAD(run);
        sensitive << clk.pos();
        SC_THREAD(dma_service_thread);
        sensitive << clk.pos();
        SC_THREAD(fc_weight_prefetch_thread);
        sensitive << clk.pos();
    }
};

#endif
