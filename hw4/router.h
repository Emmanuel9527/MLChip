#ifndef ROUTER_H
#define ROUTER_H

#include "systemc.h"
#include <queue>

#define EAST  0
#define SOUTH 1
#define WEST  2
#define NORTH 3
#define LOCAL 4
#define IDLE  5

using namespace std;

SC_MODULE( Router ) {
    sc_in  < bool >  rst;
    sc_in  < bool >  clk;

    sc_out < sc_lv<34> >  out_flit[5];
    sc_out < bool >  out_req[5];
    sc_in  < bool >  in_ack[5];

    sc_in  < sc_lv<34> >  in_flit[5];
    sc_in  < bool >  in_req[5];
    sc_out < bool >  out_ack[5];



    void tx_process();
    void rx_process();

    SC_CTOR( Router )
    {    
        SC_METHOD( rx_process );
        sensitive << clk.pos() << rst.pos();

        SC_METHOD( tx_process );
        sensitive << clk.pos() << rst.pos();
    }
};

#endif