#ifndef CORE_H
#define CORE_H

#include "noc.h"
#include "pe.h"
#include "systemc.h"
#include <algorithm>
#include <queue>

using namespace std;

SC_MODULE(Core)
{
    // Clock/reset and local router handshake interface.
    sc_in<bool> reset_n;
    sc_in<bool> clk;

    // Router -> Core link. Incoming flits are deserialized into packets.
    sc_in<Flit> flit_rx;
    sc_in<bool> req_rx;
    sc_out<bool> ack_rx;

    // Core -> Router link. Completed PE result packets are serialized here.
    sc_out<Flit> flit_tx;
    sc_out<bool> req_tx;
    sc_in<bool> ack_tx;

    // One PE sits behind each worker Core.
    PE pe;
    int id;

    // Result packets waiting to be sent back to the Controller.
    queue<Packet *> tx_packets;

    static const int PACKET_PROGRESS_FLITS = 50000;

    // Bind Core and PE id to the local router id.
    void init(int core_id)
    {
        id = core_id;
        pe.init(core_id);
    }

    int packet_first_word(Packet * packet)
    {
        if (packet == NULL || packet->datas.empty())
            return -1;
        return (int)packet->datas[0];
    }

    // Send one flit using the valid-ready protocol.
    void send_flit(const Flit &flit)
    {
        while (true)
        {
            if (!reset_n.read())
            {
                req_tx.write(false);
                wait();
                continue;
            }

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

    // TX network-interface thread.
    // Packet format on the link: HEAD, then BODY payload flits, then TAIL.
    void tx_thread()
    {
        while (true)
        {
            if (!reset_n.read())
            {
                Flit zero_flit;
                zero_flit = 0;
                req_tx.write(false);
                flit_tx.write(zero_flit);
                wait();
                continue;
            }

            if (tx_packets.empty())
            {
                wait();
                continue;
            }

            Packet *packet = tx_packets.front();
            tx_packets.pop();
#ifdef DEBUG
            cout << "[CORE_TX] PE " << id
                 << " start send packet to " << packet->dest_id
                 << ", first_word=" << packet_first_word(packet)
                 << ", payload_size=" << packet->datas.size() << "." << endl;
#endif
            int payload_flits =
                (packet->datas.size() + PACKED_FLIT_WORDS - 1) / PACKED_FLIT_WORDS;
            bool report_progress = false;
#ifdef DEBUG
            report_progress = payload_flits >= PACKET_PROGRESS_FLITS;
#endif
            int sent_payload_flits = 0;

            // HEAD flit layout: type, destination id, source id.
            Flit header;
            header = 0;
            set_flit_type(header, NOC_HEAD_FLIT);
            set_flit_count(header, 0);
            set_header_fields(header, packet->dest_id, packet->source_id);
            send_flit(header);

            for (size_t i = 0; i < packet->datas.size();)
            {
                // Payload flits carry multiple raw float words.
                int count = (int)min((size_t)PACKED_FLIT_WORDS,
                                     packet->datas.size() - i);
                bool last = (i + count == packet->datas.size());

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

                    converter.fval = packet->datas[i + lane];
                    set_flit_word(flit, lane, converter.ival);
                }
                send_flit(flit);
                i += count;
                sent_payload_flits++;

                if (report_progress &&
                    (sent_payload_flits % PACKET_PROGRESS_FLITS == 0 ||
                     sent_payload_flits == payload_flits))
                {
#ifdef DEBUG
                    cout << "[CORE_TX] PE " << id
                         << " send progress: "
                         << sent_payload_flits << "/" << payload_flits
                         << " packed flits." << endl;
#endif
                }
            }

            delete packet;
#ifdef DEBUG
            cout << "[CORE_TX] PE " << id
                 << " finished sending packet." << endl;
#endif
        }
    }

    // Decode an incoming flit and update the packet being reconstructed.
    void receive_flit(const Flit &flit, Packet &packet, bool &packet_active)
    {
        unsigned int type = get_flit_type(flit);

        if (type == NOC_HEAD_FLIT)
        {
            // HEAD starts a new packet and carries routing metadata.
            packet = Packet();
            packet.dest_id = get_header_dest(flit);
            packet.source_id = get_header_source(flit);
            packet_active = true;
            return;
        }

        if (!packet_active || (type != NOC_BODY_FLIT && type != NOC_TAIL_FLIT))
            return;

        // BODY/TAIL payloads are converted from raw bits back to float values.
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

        if (type == NOC_TAIL_FLIT)
        {
            // TAIL completes the packet. Run PE computation and enqueue result.
            if (!packet.datas.empty())
            {
#ifdef DEBUG
                int op = (int)packet.datas[0];
                cout << "[CORE_RX] PE " << id
                     << " received packet from " << packet.source_id
                     << ", op=" << op;
                if ((op == OP_COMPUTE_CONV ||
                     op == OP_COMPUTE_POOL ||
                     op == OP_COMPUTE_FC) &&
                    packet.datas.size() > 1)
                    cout << ", job=" << (int)packet.datas[1];
                cout << ", payload_size=" << packet.datas.size() << "." << endl;
#endif
            }

            Packet *result = pe.process_packet(packet);
            if (result != NULL)
            {
#ifdef DEBUG
                cout << "[CORE_RX] PE " << id
                     << " queued result packet, first_word="
                     << packet_first_word(result)
                     << ", payload_size=" << result->datas.size() << "." << endl;
#endif
                tx_packets.push(result);
            }
            packet = Packet();
            packet_active = false;
        }
    }

    // RX network-interface thread. It keeps ack_rx high when ready to accept
    // flits from the local router.
    void rx_thread()
    {
        Packet packet;
        bool packet_active = false;
        bool req_seen = false;

        while (true)
        {
            if (!reset_n.read())
            {
                packet = Packet();
                packet_active = false;
                req_seen = false;
                ack_rx.write(false);
                wait();
                continue;
            }

            ack_rx.write(true);

            if (!req_rx.read())
            {
                req_seen = false;
            }
            else if (!req_seen)
            {
                req_seen = true;
                receive_flit(flit_rx.read(), packet, packet_active);
            }

            wait();
        }
    }

    // Register independent RX and TX threads for the worker network interface.
    SC_HAS_PROCESS(Core);
    Core(sc_module_name name) : sc_module(name), pe("pe"), id(0)
    {
        Flit zero_flit;
        zero_flit = 0;
        req_tx.initialize(false);
        ack_rx.initialize(true);
        flit_tx.initialize(zero_flit);

        SC_THREAD(tx_thread);
        sensitive << clk.pos();
        SC_THREAD(rx_thread);
        sensitive << clk.pos();
    }
};

#endif
