#ifndef CORE_H
#define CORE_H

#include "pe.h"
#include "systemc.h"
#include <queue>

using namespace std;

SC_MODULE( Core ) {
    // Clock/reset and local router handshake interface.
    sc_in  < bool >  rst;
    sc_in  < bool >  clk;

    // Router -> Core link. Incoming flits are deserialized into packets.
    sc_in  < sc_lv<34> > flit_rx;
    sc_in  < bool > req_rx;
    sc_out < bool > ack_rx;

    // Core -> Router link. Completed PE result packets are serialized here.
    sc_out < sc_lv<34> > flit_tx;
    sc_out < bool > req_tx;
    sc_in  < bool > ack_tx;

    // One PE sits behind each worker Core.
    PE pe;
    int id;

    // Result packets waiting to be sent back to the Controller.
    queue<Packet *> tx_packets;

    // Bind Core and PE id to the local router id.
    void init(int core_id)
    {
        id = core_id;
        pe.init(core_id);
    }

    // Send one flit using the valid-ready protocol.
    void send_flit(const sc_lv<34> &flit)
    {
        while (true)
        {
            if (rst.read())
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
            if (rst.read())
            {
                sc_lv<34> zero_flit;
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

            // HEAD flit layout: type, destination id, source id.
            sc_lv<34> header;
            header.range(33, 32) = 2;
            header.range(31, 16) = packet->dest_id;
            header.range(15, 0) = packet->source_id;
            send_flit(header);

            for (size_t i = 0; i < packet->datas.size(); i++)
            {
                // Payload flits carry raw float bits in [31:0].
                sc_lv<34> flit;
                flit.range(33, 32) = (i == packet->datas.size() - 1) ? 1 : 0;

                union {
                    float fval;
                    unsigned int ival;
                } converter;

                converter.fval = packet->datas[i];
                flit.range(31, 0) = converter.ival;
                send_flit(flit);
            }

            delete packet;
        }
    }

    // Decode an incoming flit and update the packet being reconstructed.
    void receive_flit(const sc_lv<34> &flit, Packet &packet, bool &packet_active)
    {
        unsigned int type = flit.range(33, 32).to_uint();

        if (type == 2)
        {
            // HEAD starts a new packet and carries routing metadata.
            packet = Packet();
            packet.dest_id = flit.range(31, 16).to_uint();
            packet.source_id = flit.range(15, 0).to_uint();
            packet_active = true;
            return;
        }

        if (!packet_active || (type != 0 && type != 1))
            return;

        // BODY/TAIL payloads are converted from raw bits back to float values.
        union {
            float fval;
            unsigned int ival;
        } converter;

        converter.ival = flit.range(31, 0).to_uint();
        packet.datas.push_back(converter.fval);

        if (type == 1)
        {
            // TAIL completes the packet. Run PE computation and enqueue result.
            Packet *result = pe.process_packet(packet);
            if (result != NULL)
                tx_packets.push(result);
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

        while (true)
        {
            if (rst.read())
            {
                packet = Packet();
                packet_active = false;
                ack_rx.write(false);
                wait();
                continue;
            }

            ack_rx.write(true);

            if (req_rx.read())
                receive_flit(flit_rx.read(), packet, packet_active);

            wait();
        }
    }

    // Register independent RX and TX threads for the worker network interface.
    SC_HAS_PROCESS(Core);
    Core(sc_module_name name) : sc_module(name), pe("pe"), id(0)
    {
        sc_lv<34> zero_flit;
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
