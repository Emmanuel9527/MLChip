#ifndef ROUTER_H
#define ROUTER_H

#include "systemc.h"
#include <queue>

SC_MODULE(Router)
{
    static const int PORT_NUM = 5;
    static const int VC_NUM = 2;
    static const int VC_DEPTH = 16;
    static const int OUT_DEPTH = 8;

    // Router ports.
    sc_in<bool> rst;
    sc_in<bool> clk;

    sc_out<sc_lv<34> > out_flit[PORT_NUM];
    sc_out<bool> out_req[PORT_NUM];
    sc_in<bool> in_ack[PORT_NUM];

    sc_in<sc_lv<34> > in_flit[PORT_NUM];
    sc_in<bool> in_req[PORT_NUM];
    sc_out<bool> out_ack[PORT_NUM];

    // Finite logical FIFOs for virtual-channel and output buffering.
    std::queue<sc_lv<34> > in_q[PORT_NUM][VC_NUM];
    std::queue<sc_lv<34> > out_q[PORT_NUM];

    // Allocation state for packet-level output locking.
    int vc_state[PORT_NUM][VC_NUM];
    int out_port_lock[PORT_NUM];
    int output_rr_start[PORT_NUM];
    int rx_current_vc[PORT_NUM];
    bool tx_active[PORT_NUM];
    sc_lv<34> tx_buffer[PORT_NUM];
    int router_id;

    void init(int id)
    {
        router_id = id;
    }

    // Deterministic XY routing for a 4x4 mesh.
    int get_xy_route(int current_id, int dest_id)
    {
        int cx = current_id % 4;
        int cy = current_id / 4;
        int dx = dest_id % 4;
        int dy = dest_id / 4;

        if (dx > cx)
            return 2;
        if (dx < cx)
            return 3;
        if (dy > cy)
            return 1;
        if (dy < cy)
            return 0;
        return 4;
    }

    // Input VC FIFOs apply backpressure when the selected depth is full.
    bool vc_has_space(int p, int vc)
    {
        return in_q[p][vc].size() < VC_DEPTH;
    }

    // Output FIFOs model finite switch-to-link buffers.
    bool out_has_space(int p)
    {
        return out_q[p].size() < OUT_DEPTH;
    }

    // Spread traffic directions over two VCs to reduce head-of-line blocking.
    int preferred_vc_for_output(int target_out)
    {
        if (target_out == 2 || target_out == 3)
            return 0;
        return 1;
    }

    // Header flits choose a VC using both output direction and free space.
    int choose_vc(int p, int target_out)
    {
        int preferred = preferred_vc_for_output(target_out);
        int other = 1 - preferred;

        if (vc_has_space(p, preferred) && in_q[p][preferred].size() <= in_q[p][other].size() + 1)
            return preferred;
        if (vc_has_space(p, other))
            return other;
        if (vc_has_space(p, preferred))
            return preferred;
        return -1;
    }

    // The output lock stores one input-port and VC pair.
    int encode_lock(int input_port, int vc)
    {
        return input_port * VC_NUM + vc;
    }

    bool any_vc_has_space(int p)
    {
        for (int v = 0; v < VC_NUM; v++)
            if (vc_has_space(p, v))
                return true;
        return false;
    }

    void rx_thread_0() { rx_logic(0); }
    void rx_thread_1() { rx_logic(1); }
    void rx_thread_2() { rx_logic(2); }
    void rx_thread_3() { rx_logic(3); }
    void rx_thread_4() { rx_logic(4); }

    // RX stage: req is valid and ack is ready.
    void rx_logic(int p)
    {
        while (true)
        {
            bool current_ready = out_ack[p].read();
            bool accepted = false;
            int accepted_type = -1;

            if (in_req[p].read() == 1 && current_ready)
            {
                sc_lv<34> f = in_flit[p].read();
                int type = f.range(33, 32).to_uint();
                int vc = rx_current_vc[p];

                if (type == 2)
                {
                    int dest_id = f.range(31, 16).to_uint();
                    int target_out = get_xy_route(router_id, dest_id);
                    vc = choose_vc(p, target_out);
                }

                if (vc != -1 && vc_has_space(p, vc))
                {
                    if (type == 2)
                        rx_current_vc[p] = vc;

                    in_q[p][vc].push(f);
                    accepted = true;
                    accepted_type = type;

                    if (type == 1)
                        rx_current_vc[p] = -1;
                }
            }

            bool next_ready = false;
            if (accepted)
            {
                if (accepted_type == 1)
                    next_ready = any_vc_has_space(p);
                else if (rx_current_vc[p] != -1)
                    next_ready = vc_has_space(p, rx_current_vc[p]);
                else
                    next_ready = any_vc_has_space(p);
            }
            else if (in_req[p].read() == 1)
            {
                sc_lv<34> f = in_flit[p].read();
                int type = f.range(33, 32).to_uint();

                if (type == 2)
                {
                    int dest_id = f.range(31, 16).to_uint();
                    int target_out = get_xy_route(router_id, dest_id);
                    next_ready = (choose_vc(p, target_out) != -1);
                }
                else if (rx_current_vc[p] != -1)
                {
                    next_ready = vc_has_space(p, rx_current_vc[p]);
                }
                else
                {
                    next_ready = false;
                }
            }
            else
            {
                next_ready = any_vc_has_space(p);
            }

            out_ack[p].write(next_ready);
            wait();
        }
    }

    // Route and switch-allocation stage with one round-robin arbiter per output.
    void route_thread()
    {
        while (true)
        {
            bool input_used[PORT_NUM];
            for (int i = 0; i < PORT_NUM; i++)
                input_used[i] = false;

            for (int out = 0; out < PORT_NUM; out++)
            {
                if (!out_has_space(out))
                    continue;

                int grant_input = -1;
                int grant_vc = -1;

                for (int k = 0; k < PORT_NUM; k++)
                {
                    int i = (output_rr_start[out] + k) % PORT_NUM;
                    if (input_used[i])
                        continue;

                    for (int v = 0; v < VC_NUM; v++)
                    {
                        if (in_q[i][v].empty())
                            continue;

                        sc_lv<34> f = in_q[i][v].front();
                        int type = f.range(33, 32).to_uint();

                        if (vc_state[i][v] == -1)
                        {
                            if (type != 2)
                            {
                                in_q[i][v].pop();
                                continue;
                            }

                            int dest_id = f.range(31, 16).to_uint();
                            int target_out = get_xy_route(router_id, dest_id);

                            if (target_out != out)
                                continue;

                            if (out_port_lock[out] == -1)
                            {
                                out_port_lock[out] = encode_lock(i, v);
                                vc_state[i][v] = out;
                            }
                            else
                            {
                                continue;
                            }
                        }

                        if (vc_state[i][v] == out && out_port_lock[out] == encode_lock(i, v))
                        {
                            grant_input = i;
                            grant_vc = v;
                            break;
                        }
                    }

                    if (grant_input != -1)
                        break;
                }

                if (grant_input != -1)
                {
                    sc_lv<34> f = in_q[grant_input][grant_vc].front();
                    int type = f.range(33, 32).to_uint();

                    out_q[out].push(f);
                    in_q[grant_input][grant_vc].pop();
                    input_used[grant_input] = true;
                    output_rr_start[out] = (grant_input + 1) % PORT_NUM;

                    if (type == 1)
                    {
                        out_port_lock[out] = -1;
                        vc_state[grant_input][grant_vc] = -1;
                    }
                }
            }

            wait();
        }
    }

    void tx_thread_0() { tx_logic(0); }
    void tx_thread_1() { tx_logic(1); }
    void tx_thread_2() { tx_logic(2); }
    void tx_thread_3() { tx_logic(3); }
    void tx_thread_4() { tx_logic(4); }

    // TX stage: req is valid and ack is ready.
    void tx_logic(int p)
    {
        while (true)
        {
            if (tx_active[p] && in_ack[p].read() == 1)
            {
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
                out_req[p].write(1);
            }
            else
            {
                out_req[p].write(0);
            }

            wait();
        }
    }

    SC_HAS_PROCESS(Router);
    Router(sc_module_name name) : sc_module(name)
    {
        router_id = 0;
        sc_lv<34> zero_flit;
        zero_flit = 0;

        for (int i = 0; i < PORT_NUM; i++)
        {
            out_port_lock[i] = -1;
            output_rr_start[i] = 0;
            rx_current_vc[i] = -1;
            tx_active[i] = false;
            tx_buffer[i] = zero_flit;
            out_req[i].initialize(false);
            out_ack[i].initialize(true);
            out_flit[i].initialize(zero_flit);

            for (int v = 0; v < VC_NUM; v++)
                vc_state[i][v] = -1;
        }

        SC_THREAD(rx_thread_0);
        sensitive << clk.pos();
        SC_THREAD(rx_thread_1);
        sensitive << clk.pos();
        SC_THREAD(rx_thread_2);
        sensitive << clk.pos();
        SC_THREAD(rx_thread_3);
        sensitive << clk.pos();
        SC_THREAD(rx_thread_4);
        sensitive << clk.pos();

        SC_THREAD(route_thread);
        sensitive << clk.pos();

        SC_THREAD(tx_thread_0);
        sensitive << clk.pos();
        SC_THREAD(tx_thread_1);
        sensitive << clk.pos();
        SC_THREAD(tx_thread_2);
        sensitive << clk.pos();
        SC_THREAD(tx_thread_3);
        sensitive << clk.pos();
        SC_THREAD(tx_thread_4);
        sensitive << clk.pos();
    }
};

#endif
