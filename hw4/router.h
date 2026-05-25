#ifndef ROUTER_H
#define ROUTER_H

#include "systemc.h"
#include <queue>

#define NORTH 0
#define SOUTH 1
#define EAST  2
#define WEST  3
#define LOCAL 4
#define BROADCAST_ID 65535

using namespace std;

SC_MODULE( Router ) {
    /*
    HW4 NoC router model.

    The link protocol is VALID/READY style:
      in_req/out_req  = VALID
      out_ack/in_ack  = READY
      flit transfer   = VALID && READY

    VALID is generated from buffered data and is held until READY arrives. This
    follows the AXI-style rule that VALID must not wait for READY before rising.

    Internal pipeline:
      Input Sync -> Virtual-Channel Buffer -> XY Routing
                 -> Round-Robin Switch Allocation -> Output Queue -> Output Sync
    */

    static const int PORT_NUM = 5;
    static const int VC_NUM = 2;
    static const int VC_DEPTH = 16;
    static const int OUT_DEPTH = 8;

    enum FlitType {
        BODY_FLIT = 0,
        TAIL_FLIT = 1,
        HEAD_FLIT = 2
    };

    struct InputResult {
        bool accepted;
        int flit_type;

        InputResult() : accepted(false), flit_type(-1) {}
    };

    struct Grant {
        int input_port;
        int vc;

        Grant() : input_port(-1), vc(-1) {}
    };

    // Five-port router: NORTH, SOUTH, EAST, WEST, and LOCAL.
    sc_in  < bool >  rst;
    sc_in  < bool >  clk;

    // Output-side valid-ready links.
    sc_out < sc_lv<34> > out_flit[PORT_NUM];
    sc_out < bool > out_req[PORT_NUM];
    sc_in  < bool > in_ack[PORT_NUM];

    // Input-side valid-ready links.
    sc_in  < sc_lv<34> > in_flit[PORT_NUM];
    sc_in  < bool > in_req[PORT_NUM];
    sc_out < bool > out_ack[PORT_NUM];

    int router_id;

    // Input virtual-channel buffers.
    queue<sc_lv<34> > in_q[PORT_NUM][VC_NUM];

    // Output queues and output-sync holding registers.
    queue<sc_lv<34> > out_q[PORT_NUM];
    bool tx_active[PORT_NUM];
    sc_lv<34> tx_buffer[PORT_NUM];

    // Packet-level allocation state.
    int vc_state[PORT_NUM][VC_NUM];
    int out_port_lock[PORT_NUM];
    int output_rr_start[PORT_NUM];
    int rx_current_vc[PORT_NUM];
    bool broadcast_active[PORT_NUM];
    bool broadcast_mask[PORT_NUM][PORT_NUM];

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

    bool is_broadcast_flit(const sc_lv<34> &flit)
    {
        return flit_dest_id(flit) == BROADCAST_ID;
    }

    // Deterministic XY routing for a 4x4 mesh.
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

    void clear_queue(queue<sc_lv<34> > &q)
    {
        while (!q.empty())
            q.pop();
    }

    bool eb_has_space(int port, int vc)
    {
        return (int)in_q[port][vc].size() < VC_DEPTH;
    }

    bool any_vc_has_space(int port)
    {
        for (int vc = 0; vc < VC_NUM; vc++)
            if (eb_has_space(port, vc))
                return true;
        return false;
    }

    bool output_has_space(int port)
    {
        return (int)out_q[port].size() < OUT_DEPTH;
    }

    /*
    VC mapping:
      VC0 is the deterministic XY escape channel.
      VC1 is the adaptive minimal-routing channel.

    This follows the lecture idea of using virtual channels to reduce blocking
    while keeping one deadlock-free escape path. Adaptive routing is only used
    on VC1; VC0 always follows XY.
    */
    int preferred_vc_for_header(const sc_lv<34> &flit)
    {
        (void)flit;
        return 1;
    }

    int choose_input_vc(int input_port, const sc_lv<34> &flit)
    {
        int preferred = preferred_vc_for_header(flit);
        int other = 1 - preferred;

        if (eb_has_space(input_port, preferred))
            return preferred;
        if (eb_has_space(input_port, other))
            return other;
        return -1;
    }

    int encode_lock(int input_port, int vc)
    {
        return input_port * VC_NUM + vc;
    }

    int encode_broadcast_lock(int input_port)
    {
        return PORT_NUM * VC_NUM + input_port;
    }

    void clear_broadcast_mask(int input_port)
    {
        for (int out = 0; out < PORT_NUM; out++)
            broadcast_mask[input_port][out] = false;
    }

    // Build a spanning-tree multicast mask for activation broadcast.
    // Node 0 injects to EAST and SOUTH. The top row forwards EAST and SOUTH;
    // each column then forwards SOUTH. Every worker receives exactly one copy.
    void build_broadcast_mask(int input_port)
    {
        clear_broadcast_mask(input_port);

        int x = router_id % 4;
        int y = router_id / 4;

        if (router_id != 0)
            broadcast_mask[input_port][LOCAL] = true;

        if (input_port == LOCAL)
        {
            if (x < 3)
                broadcast_mask[input_port][EAST] = true;
            if (y < 3)
                broadcast_mask[input_port][SOUTH] = true;
            return;
        }

        if (input_port == WEST && y == 0)
        {
            if (x < 3)
                broadcast_mask[input_port][EAST] = true;
            if (y < 3)
                broadcast_mask[input_port][SOUTH] = true;
            return;
        }

        if (input_port == NORTH && y < 3)
            broadcast_mask[input_port][SOUTH] = true;
    }

    bool broadcast_outputs_have_space(int input_port)
    {
        int lock_id = encode_broadcast_lock(input_port);
        for (int out = 0; out < PORT_NUM; out++)
        {
            if (!broadcast_mask[input_port][out])
                continue;
            if (!output_has_space(out))
                return false;
            if (out_port_lock[out] != lock_id)
                return false;
        }
        return true;
    }

    bool broadcast_outputs_can_allocate(int input_port)
    {
        for (int out = 0; out < PORT_NUM; out++)
        {
            if (!broadcast_mask[input_port][out])
                continue;
            if (!output_has_space(out) || out_port_lock[out] != -1)
                return false;
        }
        return true;
    }

    void allocate_broadcast_outputs(int input_port)
    {
        int lock_id = encode_broadcast_lock(input_port);
        for (int out = 0; out < PORT_NUM; out++)
            if (broadcast_mask[input_port][out])
                out_port_lock[out] = lock_id;
    }

    void release_broadcast_outputs(int input_port)
    {
        int lock_id = encode_broadcast_lock(input_port);
        for (int out = 0; out < PORT_NUM; out++)
            if (out_port_lock[out] == lock_id)
                out_port_lock[out] = -1;
    }

    bool ready_for_broadcast_input(int input_port)
    {
        if (!in_req[input_port].read())
            return true;

        sc_lv<34> incoming = in_flit[input_port].read();
        if (!broadcast_active[input_port])
        {
            if (flit_type(incoming) != HEAD_FLIT || !is_broadcast_flit(incoming))
                return false;
            build_broadcast_mask(input_port);
            return broadcast_outputs_can_allocate(input_port);
        }

        return broadcast_outputs_have_space(input_port);
    }

    bool accept_broadcast_input(int input_port)
    {
        if (!in_req[input_port].read())
            return false;

        sc_lv<34> incoming = in_flit[input_port].read();
        int type = flit_type(incoming);

        if (!broadcast_active[input_port])
        {
            if (type != HEAD_FLIT || !is_broadcast_flit(incoming))
                return false;
            build_broadcast_mask(input_port);
            if (!broadcast_outputs_can_allocate(input_port))
                return false;
            allocate_broadcast_outputs(input_port);
            broadcast_active[input_port] = true;
        }

        if (!broadcast_outputs_have_space(input_port))
            return false;

        for (int out = 0; out < PORT_NUM; out++)
            if (broadcast_mask[input_port][out])
                out_q[out].push(incoming);

        if (type == TAIL_FLIT)
        {
            release_broadcast_outputs(input_port);
            broadcast_active[input_port] = false;
            clear_broadcast_mask(input_port);
        }

        return true;
    }

    int xy_route(int current_id, int dest_id)
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

    int output_waiting_time(int out)
    {
        int buffer_len = (int)out_q[out].size() + (tx_active[out] ? 1 : 0);
        int service_time = in_ack[out].read() ? 1 : 4;
        return service_time * (buffer_len + 1);
    }

    int adaptive_minimal_route(int current_id, int dest_id)
    {
        int cx = current_id % 4;
        int cy = current_id / 4;
        int dx = dest_id % 4;
        int dy = dest_id / 4;

        int x_dir = -1;
        int y_dir = -1;

        if (dx > cx)
            x_dir = EAST;
        else if (dx < cx)
            x_dir = WEST;

        if (dy > cy)
            y_dir = SOUTH;
        else if (dy < cy)
            y_dir = NORTH;

        if (x_dir == -1 && y_dir == -1)
            return LOCAL;
        if (x_dir == -1)
            return y_dir;
        if (y_dir == -1)
            return x_dir;

        int x_wait = output_waiting_time(x_dir);
        int y_wait = output_waiting_time(y_dir);

        if (!output_has_space(x_dir) && output_has_space(y_dir))
            return y_dir;
        if (!output_has_space(y_dir) && output_has_space(x_dir))
            return x_dir;

        return (y_wait < x_wait) ? y_dir : x_dir;
    }

    int route_for_vc(const sc_lv<34> &header, int vc)
    {
        int dest_id = flit_dest_id(header);
        if (vc == 0)
            return xy_route(router_id, dest_id);
        return adaptive_minimal_route(router_id, dest_id);
    }

    // READY generation for one input port. READY depends on buffer capacity,
    // not on the sender's VALID policy.
    bool ready_for_input(int input_port)
    {
        if (broadcast_active[input_port])
            return ready_for_broadcast_input(input_port);

        if (!in_req[input_port].read())
            return any_vc_has_space(input_port);

        sc_lv<34> incoming = in_flit[input_port].read();
        int type = flit_type(incoming);

        if (type == HEAD_FLIT && is_broadcast_flit(incoming))
            return ready_for_broadcast_input(input_port);

        if (type == HEAD_FLIT)
            return choose_input_vc(input_port, incoming) != -1;

        if (rx_current_vc[input_port] != -1)
            return eb_has_space(input_port, rx_current_vc[input_port]);

        return false;
    }

    // Input Sync + VC buffer write.
    InputResult accept_one_input(int input_port)
    {
        InputResult result;

        if (broadcast_active[input_port] ||
            (in_req[input_port].read() &&
             flit_type(in_flit[input_port].read()) == HEAD_FLIT &&
             is_broadcast_flit(in_flit[input_port].read())))
        {
            bool accepted = accept_broadcast_input(input_port);
            result.accepted = accepted;
            if (accepted)
                result.flit_type = flit_type(in_flit[input_port].read());
            return result;
        }

        if (!in_req[input_port].read())
            return result;

        sc_lv<34> incoming = in_flit[input_port].read();
        int type = flit_type(incoming);
        int vc = rx_current_vc[input_port];

        if (type == HEAD_FLIT)
            vc = choose_input_vc(input_port, incoming);

        if (vc == -1 || !eb_has_space(input_port, vc))
            return result;

        if (type == HEAD_FLIT)
            rx_current_vc[input_port] = vc;

        in_q[input_port][vc].push(incoming);
        result.accepted = true;
        result.flit_type = type;

        if (type == TAIL_FLIT)
            rx_current_vc[input_port] = -1;

        return result;
    }

    void input_port_stage(int input_port)
    {
        while (true)
        {
            if (rst.read())
            {
                rx_current_vc[input_port] = -1;
                release_broadcast_outputs(input_port);
                broadcast_active[input_port] = false;
                clear_broadcast_mask(input_port);
                out_ack[input_port].write(false);
                wait();
                continue;
            }

            // A transfer is accepted only when VALID is high and this router
            // had already advertised READY to the sender in the previous cycle.
            // This avoids consuming the same stable flit twice in SystemC's
            // posedge/delta-cycle scheduling model.
            bool ready_seen_by_sender = out_ack[input_port].read();
            if (in_req[input_port].read() && ready_seen_by_sender)
                accept_one_input(input_port);

            out_ack[input_port].write(ready_for_input(input_port));
            wait();
        }
    }

    bool can_allocate_output(int out, int input_port, int vc)
    {
        if (out_port_lock[out] == -1)
        {
            out_port_lock[out] = encode_lock(input_port, vc);
            vc_state[input_port][vc] = out;
            return true;
        }

        return false;
    }

    // One round-robin arbiter per output port.
    Grant arbitrate_output(int out, bool input_used[PORT_NUM])
    {
        Grant grant;

        if (!output_has_space(out))
            return grant;

        for (int step = 0; step < PORT_NUM; step++)
        {
            int input_port = (output_rr_start[out] + step) % PORT_NUM;
            if (input_used[input_port])
                continue;

            for (int vc = 0; vc < VC_NUM; vc++)
            {
                if (in_q[input_port][vc].empty())
                    continue;

                sc_lv<34> candidate = in_q[input_port][vc].front();
                int type = flit_type(candidate);

                if (vc_state[input_port][vc] == -1)
                {
                    if (type != HEAD_FLIT)
                    {
                        in_q[input_port][vc].pop();
                        continue;
                    }

                    int target_out = route_for_vc(candidate, vc);
                    if (target_out != out)
                        continue;

                    if (!can_allocate_output(out, input_port, vc))
                        continue;
                }

                if (vc_state[input_port][vc] == out &&
                    out_port_lock[out] == encode_lock(input_port, vc))
                {
                    grant.input_port = input_port;
                    grant.vc = vc;
                    return grant;
                }
            }
        }

        return grant;
    }

    // Crossbar movement from granted input VC to output queue.
    void transfer_grant(int out, const Grant &grant, bool input_used[PORT_NUM])
    {
        if (grant.input_port == -1)
            return;

        sc_lv<34> flit = in_q[grant.input_port][grant.vc].front();
        int type = flit_type(flit);

        out_q[out].push(flit);
        in_q[grant.input_port][grant.vc].pop();
        input_used[grant.input_port] = true;
        output_rr_start[out] = (grant.input_port + 1) % PORT_NUM;

        if (type == TAIL_FLIT)
        {
            out_port_lock[out] = -1;
            vc_state[grant.input_port][grant.vc] = -1;
        }
    }

    void reset_allocation_state()
    {
        for (int p = 0; p < PORT_NUM; p++)
        {
            out_port_lock[p] = -1;
            output_rr_start[p] = 0;
            broadcast_active[p] = false;
            clear_broadcast_mask(p);
            clear_queue(out_q[p]);

            for (int vc = 0; vc < VC_NUM; vc++)
            {
                vc_state[p][vc] = -1;
                clear_queue(in_q[p][vc]);
            }
        }
    }

    void route_arbiter_crossbar_stage()
    {
        while (true)
        {
            if (rst.read())
            {
                reset_allocation_state();
                wait();
                continue;
            }

            bool input_used[PORT_NUM];
            for (int p = 0; p < PORT_NUM; p++)
                input_used[p] = false;

            for (int out = 0; out < PORT_NUM; out++)
            {
                Grant grant = arbitrate_output(out, input_used);
                transfer_grant(out, grant, input_used);
            }

            wait();
        }
    }

    // Output Sync: keep VALID high and flit stable until downstream READY.
    void output_stage(int output_port)
    {
        while (true)
        {
            if (rst.read())
            {
                sc_lv<34> zero_flit;
                zero_flit = 0;
                tx_active[output_port] = false;
                tx_buffer[output_port] = zero_flit;
                out_req[output_port].write(false);
                out_flit[output_port].write(zero_flit);
                wait();
                continue;
            }

            if (tx_active[output_port] && in_ack[output_port].read())
            {
                if (!out_q[output_port].empty())
                    out_q[output_port].pop();
                tx_active[output_port] = false;
                out_req[output_port].write(false);
                wait();
                continue;
            }

            if (!tx_active[output_port] && !out_q[output_port].empty())
            {
                tx_buffer[output_port] = out_q[output_port].front();
                tx_active[output_port] = true;
            }

            if (tx_active[output_port])
            {
                out_flit[output_port].write(tx_buffer[output_port]);
                out_req[output_port].write(true);
            }
            else
            {
                out_req[output_port].write(false);
            }

            wait();
        }
    }

    void input_thread_0() { input_port_stage(0); }
    void input_thread_1() { input_port_stage(1); }
    void input_thread_2() { input_port_stage(2); }
    void input_thread_3() { input_port_stage(3); }
    void input_thread_4() { input_port_stage(4); }

    void route_thread() { route_arbiter_crossbar_stage(); }

    void output_thread_0() { output_stage(0); }
    void output_thread_1() { output_stage(1); }
    void output_thread_2() { output_stage(2); }
    void output_thread_3() { output_stage(3); }
    void output_thread_4() { output_stage(4); }

    SC_CTOR( Router )
    {
        router_id = 0;
        sc_lv<34> zero_flit;
        zero_flit = 0;

        for (int p = 0; p < PORT_NUM; p++)
        {
            out_port_lock[p] = -1;
            output_rr_start[p] = 0;
            rx_current_vc[p] = -1;
            broadcast_active[p] = false;
            tx_active[p] = false;
            tx_buffer[p] = zero_flit;
            out_req[p].initialize(false);
            out_ack[p].initialize(false);
            out_flit[p].initialize(zero_flit);

            for (int vc = 0; vc < VC_NUM; vc++)
                vc_state[p][vc] = -1;
            clear_broadcast_mask(p);
        }

        SC_THREAD(input_thread_0);
        sensitive << clk.pos();
        SC_THREAD(input_thread_1);
        sensitive << clk.pos();
        SC_THREAD(input_thread_2);
        sensitive << clk.pos();
        SC_THREAD(input_thread_3);
        sensitive << clk.pos();
        SC_THREAD(input_thread_4);
        sensitive << clk.pos();

        SC_THREAD(route_thread);
        sensitive << clk.pos();

        SC_THREAD(output_thread_0);
        sensitive << clk.pos();
        SC_THREAD(output_thread_1);
        sensitive << clk.pos();
        SC_THREAD(output_thread_2);
        sensitive << clk.pos();
        SC_THREAD(output_thread_3);
        sensitive << clk.pos();
        SC_THREAD(output_thread_4);
        sensitive << clk.pos();
    }
};

#endif
