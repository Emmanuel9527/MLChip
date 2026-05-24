#ifndef ROUTER_H
#define ROUTER_H

#include "systemc.h"
#include <queue>

#define NORTH 0
#define SOUTH 1
#define EAST  2
#define WEST  3
#define LOCAL 4

using namespace std;

SC_MODULE( Router ) {
    // Five-port router: NORTH, SOUTH, EAST, WEST, and LOCAL.
    sc_in  < bool >  rst;
    sc_in  < bool >  clk;

    // Output-side valid-ready links.
    sc_out < sc_lv<34> > out_flit[5];
    sc_out < bool > out_req[5];
    sc_in  < bool > in_ack[5];

    // Input-side valid-ready links.
    sc_in  < sc_lv<34> > in_flit[5];
    sc_in  < bool > in_req[5];
    sc_out < bool > out_ack[5];

    // Router id is the node id in the 4x4 mesh.
    int router_id;

    // Per-input packet state. A HEAD flit computes the route, and BODY/TAIL
    // flits reuse that route until the packet finishes.
    int current_route[5];
    bool packet_active[5];

    // Per-output buffering and handshake state.
    bool tx_active[5];
    sc_lv<34> tx_buffer[5];
    queue<sc_lv<34> > out_q[5];

    // main.cpp assigns this router's mesh node id after construction.
    void init(int id)
    {
        router_id = id;
    }

    // Decode flit type from bits [33:32]: 2=HEAD, 0=BODY, 1=TAIL.
    int flit_type(const sc_lv<34> &flit)
    {
        return flit.range(33, 32).to_uint();
    }

    // Destination id is stored in bits [31:16] of the HEAD flit.
    int flit_dest_id(const sc_lv<34> &flit)
    {
        return flit.range(31, 16).to_uint();
    }

    // Deterministic XY routing for a 4x4 mesh.
    // Move in X first; once x matches, move in Y; LOCAL means destination hit.
    int routing_computation(int current_id, int dest_id)
    {
        int cx = current_id % 4;
        int cy = current_id / 4;
        int dx = dest_id % 4;
        int dy = dest_id / 4;

        if (dx > cx)
            return EAST;
        if (dx < cx)
            return WEST;
        if (dy > cy)
            return SOUTH;
        if (dy < cy)
            return NORTH;
        return LOCAL;
    }

    // Utility used by reset to drop any buffered flits.
    void clear_queue(queue<sc_lv<34> > &q)
    {
        while (!q.empty())
            q.pop();
    }

    // Reset internal routing state and drive all links idle.
    void reset_state()
    {
        sc_lv<34> zero_flit;
        zero_flit = 0;
        for (int i = 0; i < 5; i++)
        {
            current_route[i] = LOCAL;
            packet_active[i] = false;
            tx_active[i] = false;
            tx_buffer[i] = zero_flit;
            clear_queue(out_q[i]);
            out_flit[i].write(zero_flit);
            out_req[i].write(false);
            out_ack[i].write(false);
        }
    }

    // Input stage. Accept valid flits when the selected output queue has space.
    // HEAD decides the output port; BODY/TAIL follow current_route[p].
    void accept_inputs()
    {
        for (int p = 0; p < 5; p++)
        {
            bool can_accept = out_q[current_route[p]].size() < 32;
            out_ack[p].write(can_accept);

            if (!can_accept || !in_req[p].read())
                continue;

            sc_lv<34> flit = in_flit[p].read();
            int type = flit_type(flit);

            if (type == 2)
            {
                current_route[p] = routing_computation(router_id, flit_dest_id(flit));
                packet_active[p] = true;
            }

            int out = packet_active[p] ? current_route[p] : routing_computation(router_id, flit_dest_id(flit));
            out_q[out].push(flit);

            if (type == 1)
            {
                packet_active[p] = false;
                current_route[p] = LOCAL;
            }
        }
    }

    // Output stage. Hold one flit stable until the downstream receiver acks it.
    void drive_outputs()
    {
        for (int p = 0; p < 5; p++)
        {
            if (tx_active[p] && in_ack[p].read())
            {
                if (!out_q[p].empty())
                    out_q[p].pop();
                tx_active[p] = false;
            }

            if (!tx_active[p] && !out_q[p].empty())
            {
                tx_buffer[p] = out_q[p].front();
                tx_active[p] = true;
            }

            if (tx_active[p])
            {
                out_flit[p].write(tx_buffer[p]);
                out_req[p].write(true);
            }
            else
            {
                sc_lv<34> zero_flit;
                zero_flit = 0;
                out_flit[p].write(zero_flit);
                out_req[p].write(false);
            }
        }
    }

    // Single-cycle router pipeline model with active-high reset.
    void tick()
    {
        if (rst.read())
        {
            reset_state();
            return;
        }

        accept_inputs();
        drive_outputs();
    }

    // Initialize visible outputs and register the clocked tick method.
    SC_CTOR( Router )
    {
        router_id = 0;
        sc_lv<34> zero_flit;
        zero_flit = 0;
        for (int i = 0; i < 5; i++)
        {
            current_route[i] = LOCAL;
            packet_active[i] = false;
            tx_active[i] = false;
            tx_buffer[i] = zero_flit;
            out_req[i].initialize(false);
            out_ack[i].initialize(false);
            out_flit[i].initialize(zero_flit);
        }

        SC_METHOD(tick);
        sensitive << clk.pos();
    }
};

#endif
