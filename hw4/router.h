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

void Router::rx_process()
{
    for (int i = 0; i < 5; i++)
        out_ack[i].write(!rst.read());
}

void Router::tx_process()
{
    sc_lv<34> zero_flit;
    zero_flit = 0;

    for (int i = 0; i < 5; i++)
    {
        out_flit[i].write(zero_flit);
        out_req[i].write(false);
    }

    if (rst.read())
        return;

    if (in_req[LOCAL].read() && in_ack[LOCAL].read())
    {
        out_flit[LOCAL].write(in_flit[LOCAL].read());
        out_req[LOCAL].write(true);
    }
}

#endif
