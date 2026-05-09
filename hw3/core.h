#ifndef CORE_H
#define CORE_H

#include "systemc.h"
#include "pe.h"

SC_MODULE( Core ) {
    sc_in  < bool >  rst;
    sc_in  < bool >  clk;
    // receive
    sc_in  < sc_lv<34> > flit_rx;
    sc_in  < bool > req_rx;      
    sc_out < bool > ack_rx;      
    // transmit
    sc_out < sc_lv<34> > flit_tx;
    sc_out < bool > req_tx;      
    sc_in  < bool > ack_tx;      

    PE pe;
    int core_id;

    void init(int id) {
        core_id = id;
        pe.init(id);
    }

    void tx_thread() {
        while(true) {
            Packet* p = pe.get_packet();
            if (p != NULL) {
                // 傳送 Header flit
                sc_lv<34> h;
                h.range(33, 32) = 2; // Type: 10 (Header)
                h.range(31, 16) = p->dest_id;
                h.range(15, 0) = p->source_id;
                
                req_tx.write(1);
                flit_tx.write(h);
                while(ack_tx.read() == 0) wait();
                req_tx.write(0);
                while(ack_tx.read() == 1) wait();

                // 傳送 Body 與 Tail flit
                for (size_t i = 0; i < p->datas.size(); i++) {
                    sc_lv<34> f;
                    if (i == p->datas.size() - 1) {
                        f.range(33, 32) = 1; // Type: 01 (Tail)
                    } else {
                        f.range(33, 32) = 0; // Type: 00 (Body)
                    }
                    union { float fval; unsigned int ival; } converter;
                    converter.fval = p->datas[i];
                    f.range(31, 0) = converter.ival;

                    req_tx.write(1);
                    flit_tx.write(f);
                    while(ack_tx.read() == 0) wait();
                    req_tx.write(0);
                    while(ack_tx.read() == 1) wait();
                }
            }
            wait();
        }
    }

    void rx_thread() {
        Packet* p = NULL;
        while(true) {
            while(req_rx.read() == 0) wait();
            sc_lv<34> f = flit_rx.read();
            
            ack_rx.write(1);
            while(req_rx.read() == 1) wait();
            ack_rx.write(0);

            unsigned int type = f.range(33, 32).to_uint();
            if (type == 2) { // Header
                p = new Packet();
                p->dest_id = f.range(31, 16).to_uint();
                p->source_id = f.range(15, 0).to_uint();
            } else if (type == 0 || type == 1) { // Body 或 Tail
                union { float fval; unsigned int ival; } converter;
                converter.ival = f.range(31, 0).to_uint();
                if (p != NULL) {
                    p->datas.push_back(converter.fval);
                    if (type == 1) { // Tail
                        pe.check_packet(p);
                        delete p; // 避免指標殘留導致長時間執行的記憶體崩潰
                        p = NULL; 
                    }
                }
            }
            wait();
        }
    }

    SC_HAS_PROCESS(Core);
    Core(sc_module_name name) : sc_module(name) {
        SC_THREAD(tx_thread);
        sensitive << clk.pos();
        SC_THREAD(rx_thread);
        sensitive << clk.pos();
    }
};

#endif