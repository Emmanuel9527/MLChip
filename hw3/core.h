#ifndef CORE_H
#define CORE_H

#include "systemc.h"
#include "pe.h"

SC_MODULE(Core)
{
    sc_in<bool> rst;
    sc_in<bool> clk;

    // Router-to-NI link.
    sc_in<sc_lv<34> > flit_rx;
    sc_in<bool> req_rx;
    sc_out<bool> ack_rx;

    // NI-to-router link.
    sc_out<sc_lv<34> > flit_tx;
    sc_out<bool> req_tx;
    sc_in<bool> ack_tx;

    PE pe;
    int core_id;

    void init(int id)
    {
        core_id = id;
        pe.init(id);
    }

    // Drive one flit through the four-phase link handshake.
    void send_flit(const sc_lv<34> &flit)
    {
        flit_tx.write(flit);
        req_tx.write(1);

        while (ack_tx.read() == 0)
            wait();

        req_tx.write(0);

        while (ack_tx.read() == 1)
            wait();
    }

    // Packet injection stage: serialize one PE packet into header/body/tail flits.
    void tx_thread()
    {
        while (true)
        {
            Packet *packet = pe.get_packet();
            if (packet == NULL)
            {
                wait();
                continue;
            }

            sc_lv<34> header;
            header.range(33, 32) = 2;
            header.range(31, 16) = packet->dest_id;
            header.range(15, 0) = packet->source_id;
            send_flit(header);

            for (size_t i = 0; i < packet->datas.size(); i++)
            {
                sc_lv<34> flit;
                if (i == packet->datas.size() - 1)
                    flit.range(33, 32) = 1;
                else
                    flit.range(33, 32) = 0;

                union
                {
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

    // Packet ejection stage: deserialize incoming flits and deliver completed packets to PE.
    void rx_thread()
    {
        Packet packet;
        bool packet_active = false;

        while (true)
        {
            if (req_rx.read() == 0)
            {
                wait();
                continue;
            }

            sc_lv<34> flit = flit_rx.read();

            ack_rx.write(1);
            while (req_rx.read() == 1)
                wait();
            ack_rx.write(0);

            unsigned int type = flit.range(33, 32).to_uint();

            if (type == 2)
            {
                packet = Packet();
                packet.dest_id = flit.range(31, 16).to_uint();
                packet.source_id = flit.range(15, 0).to_uint();
                packet_active = true;
            }
            else if (packet_active && (type == 0 || type == 1))
            {
                union
                {
                    float fval;
                    unsigned int ival;
                } converter;

                converter.ival = flit.range(31, 0).to_uint();
                packet.datas.push_back(converter.fval);

                if (type == 1)
                {
                    pe.check_packet(&packet);
                    packet_active = false;
                    packet = Packet();
                }
            }
        }
    }

    SC_HAS_PROCESS(Core);
    Core(sc_module_name name) : sc_module(name)
    {
        sc_lv<34> zero_flit;
        zero_flit = 0;

        req_tx.initialize(false);
        ack_rx.initialize(false);
        flit_tx.initialize(zero_flit);

        SC_THREAD(tx_thread);
        sensitive << clk.pos();
        SC_THREAD(rx_thread);
        sensitive << clk.pos();
    }
};

#endif
