#ifndef ROUTER_H
#define ROUTER_H

#include "systemc.h"
#include <queue>

SC_MODULE(Router)
{
    static const int PORT_NUM = 5;
    static const int VC_NUM = 2;

    sc_in<bool> rst;
    sc_in<bool> clk;

    sc_out<sc_lv<34> > out_flit[PORT_NUM];
    sc_out<bool> out_req[PORT_NUM];
    sc_in<bool> in_ack[PORT_NUM];

    sc_in<sc_lv<34> > in_flit[PORT_NUM];
    sc_in<bool> in_req[PORT_NUM];
    sc_out<bool> out_ack[PORT_NUM];

    std::queue<sc_lv<34> > in_q[PORT_NUM][VC_NUM];
    std::queue<sc_lv<34> > out_q[PORT_NUM];

    int vc_state[PORT_NUM][VC_NUM];
    int out_port_lock[PORT_NUM];
    int rx_current_vc[PORT_NUM];
    int router_id;

    void init(int id)
    {
        router_id = id;
    }

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

    int choose_vc(int p)
    {
        if (in_q[p][0].size() <= in_q[p][1].size())
            return 0;
        return 1;
    }

    int encode_lock(int input_port, int vc)
    {
        return input_port * VC_NUM + vc;
    }

    void rx_thread_0() { rx_logic(0); }
    void rx_thread_1() { rx_logic(1); }
    void rx_thread_2() { rx_logic(2); }
    void rx_thread_3() { rx_logic(3); }
    void rx_thread_4() { rx_logic(4); }

    void rx_logic(int p)
    {
        while (true)
        {
            if (in_req[p].read() == 0)
            {
                wait();
                continue;
            }

            sc_lv<34> f = in_flit[p].read();
            int type = f.range(33, 32).to_uint();

            int vc = rx_current_vc[p];
            if (type == 2 || vc == -1)
            {
                vc = choose_vc(p);
                rx_current_vc[p] = vc;
            }

            in_q[p][vc].push(f);

            out_ack[p].write(1);
            while (in_req[p].read() == 1)
                wait();
            out_ack[p].write(0);

            if (type == 1)
                rx_current_vc[p] = -1;
        }
    }

    void route_thread()
    {
        int rr_start = 0;

        while (true)
        {
            for (int k = 0; k < PORT_NUM; k++)
            {
                int i = (rr_start + k) % PORT_NUM;
                bool input_used = false;

                for (int v = 0; v < VC_NUM; v++)
                {
                    if (input_used)
                        break;

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

                        if (out_port_lock[target_out] == -1)
                        {
                            out_port_lock[target_out] = encode_lock(i, v);
                            vc_state[i][v] = target_out;
                        }
                        else
                        {
                            continue;
                        }
                    }

                    int target_out = vc_state[i][v];
                    if (out_port_lock[target_out] == encode_lock(i, v))
                    {
                        out_q[target_out].push(f);
                        in_q[i][v].pop();
                        input_used = true;

                        if (type == 1)
                        {
                            out_port_lock[target_out] = -1;
                            vc_state[i][v] = -1;
                        }
                    }
                }
            }

            rr_start = (rr_start + 1) % PORT_NUM;
            wait();
        }
    }

    void tx_thread_0() { tx_logic(0); }
    void tx_thread_1() { tx_logic(1); }
    void tx_thread_2() { tx_logic(2); }
    void tx_thread_3() { tx_logic(3); }
    void tx_thread_4() { tx_logic(4); }

    void tx_logic(int p)
    {
        while (true)
        {
            if (out_q[p].empty())
            {
                wait();
                continue;
            }

            sc_lv<34> f = out_q[p].front();
            out_flit[p].write(f);
            out_req[p].write(1);

            while (in_ack[p].read() == 0)
                wait();

            out_q[p].pop();
            out_req[p].write(0);

            while (in_ack[p].read() == 1)
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
            rx_current_vc[i] = -1;
            out_req[i].initialize(false);
            out_ack[i].initialize(false);
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
