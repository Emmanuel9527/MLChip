#ifndef CORE_H
#define CORE_H

#include "systemc.h"
#include <vector>
#include "pe.h"
#include <queue>


#include <string>
#include <algorithm>
#include <cmath>
#include <iomanip>

using namespace std;

SC_MODULE( Core ) {
    sc_in  < bool >  rst;
    sc_in  < bool >  clk;
    
    sc_in  < sc_lv<34> > flit_rx;	// The input channel
    sc_in  < bool > req_rx;	        // The request associated with the input channel
    sc_out < bool > ack_rx;	        // The outgoing ack signal associated with the input channel
    
    sc_out < sc_lv<34> > flit_tx;	// The output channel
    sc_out < bool > req_tx;	        // The request associated with the output channel
    sc_in  < bool > ack_tx;	        // The outgoing ack signal associated with the output channel

    int id;
    int tx_cnt;//tx_cnt for number of flits in one packet
    bool rx_cnt;//rx_cnt for number of packets received
    
    int tx_state;//1 2 3 4 5 for idle wait head body tail 

    Packet* tx_packet;
    Packet* rx_packet;

    
    void tx_process();
    void rx_process();
    void CAL_process();

    

    //SC_CTOR( Core ) : id(core_id)
    SC_HAS_PROCESS(Core);
    Core(sc_module_name name, int core_id) : sc_module(name), id(core_id)
    {
        SC_METHOD( tx_process );
        sensitive << clk.pos() << rst.pos();

        SC_METHOD( rx_process );
        sensitive << clk.pos() << rst.pos();

        SC_METHOD( CAL_process );
        sensitive << clk.pos() << rst.pos();
    }

    
};

#endif