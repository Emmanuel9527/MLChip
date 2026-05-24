#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "systemc.h"
#include <vector>
#include "pe.h"
#include <queue>

SC_MODULE( Controller ) {
    sc_in  < bool >  rst;
    sc_in  < bool >  clk;
    
    // to ROM
    sc_out < int >   layer_id;       // '0' means input data
    sc_out < bool >  layer_id_type;  // '0' means weight, '1' means bias (for layer_id == 0, we don't care this signal)
    sc_out < bool >  layer_id_valid;

    // from ROM
    sc_in  < float > data;
    sc_in  < bool >  data_valid;
    
    // to router0
    sc_out < sc_lv<34> > flit_tx;
    sc_out < bool > req_tx;
    sc_in  < bool > ack_tx;

    // from router0
    sc_in  < sc_lv<34> > flit_rx;
    sc_in  < bool > req_rx;
    sc_out < bool > ack_rx;

    int id;
    int tx_cnt;//tx_cnt for number of flits in one packet
    
    int tx_state;//1 2 3 4 5 for idle wait head body tail 

    Packet* rx_packet;
    Packet* tx_packet;

    void tx_process();
    void rx_process();
    void ROM_request();
    void ROM_receive();

    SC_CTOR( Controller )
    {
    
        SC_METHOD( tx_process );
        sensitive << clk.pos() << rst.pos();

        SC_METHOD( rx_process );
        sensitive << clk.pos() << rst.pos();

        SC_METHOD( ROM_request );
        sensitive << clk.pos() << rst.pos();
        dont_initialize();

        SC_METHOD( ROM_receive );
        sensitive << clk.pos();
        dont_initialize();

    }
};

#endif