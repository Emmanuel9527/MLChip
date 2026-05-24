#ifndef ROUTER_H
#define ROUTER_H

#include "systemc.h"
#include <queue>

SC_MODULE(Router)
{
    /*
    Router block map:

    Input Sync -> ElasticBuffer -> Routing Computation
               -> Stall Control / Arbiter -> Crossbar Switch -> Output Sync

    Backward Blocking Selector is modeled by out_ack generation on each input
    port. Packet Lock is modeled by out_port_lock[] and vc_state[][].
    */

    static const int PORT_NUM = 5;  // Number of router ports: North, South, East, West, and Local.
    static const int VC_NUM = 2;    // Number of virtual channels per input port.
    static const int VC_DEPTH = 16; // Maximum number of flits stored in each input virtual channel FIFO.
    static const int OUT_DEPTH = 8; // Maximum number of flits stored in each output-side FIFO.

    // Logical port encoding used by routing computation and switch allocation.
    enum PortId
    {
        NORTH = 0, // Port connected to the router above this router in the mesh.
        SOUTH = 1, // Port connected to the router below this router in the mesh.
        EAST = 2,  // Port connected to the router on the right side of this router.
        WEST = 3,  // Port connected to the router on the left side of this router.
        LOCAL = 4  // Port connected to the local processing element / core.
    };

    // Two-bit flit type encoding stored in flit[33:32].
    enum FlitType
    {
        BODY_FLIT = 0, // Middle payload flit of a packet.
        TAIL_FLIT = 1, // Last payload flit; releases the packet-level output lock.
        HEAD_FLIT = 2  // Header flit; carries destination/source IDs and starts routing allocation.
    };

    // Result produced by the Input Sync + ElasticBuffer write stage for one cycle.
    struct InputSyncResult
    {
        bool accepted; // True when the input handshake succeeded and the flit was written into an input VC.
        int flit_type; // Type of the accepted flit; -1 means no flit was accepted this cycle.

        InputSyncResult()
        {
            accepted = false;
            flit_type = -1;
        }
    };

    // Arbiter decision for one output port in one cycle.
    struct SwitchGrant
    {
        int input_port; // Input port selected to drive the requested output port; -1 means no grant.
        int vc;         // Virtual channel selected from the granted input port.

        SwitchGrant()
        {
            input_port = -1;
            vc = -1;
        }
    };

    // Router ports.
    sc_in<bool> rst; // Active-low reset shared by all router pipeline stages.
    sc_in<bool> clk; // Clock used by all sequential router stages.

    sc_out<sc_lv<34>> out_flit[PORT_NUM]; // Output flit data driven to the next router or local core.
    sc_out<bool> out_req[PORT_NUM];       // Output valid signal; 1 when out_flit carries a valid flit.
    sc_in<bool> in_ack[PORT_NUM];         // Input ready signal from the downstream receiver.

    sc_in<sc_lv<34>> in_flit[PORT_NUM]; // Input flit data from the previous router or local core.
    sc_in<bool> in_req[PORT_NUM];       // Input valid signal; 1 when in_flit carries a valid flit.
    sc_out<bool> out_ack[PORT_NUM];     // Output ready signal; 1 when this router can accept a flit.

    // ElasticBuffer: input-side virtual-channel FIFOs.
    std::queue<sc_lv<34>> in_q[PORT_NUM][VC_NUM]; // Buffered flits for each input port and virtual channel.

    // Output Sync: switch-to-link FIFOs and held flits for valid-ready output.
    std::queue<sc_lv<34>> out_q[PORT_NUM]; // Per-output FIFO after crossbar transfer, before link handshake.
    bool tx_active[PORT_NUM];              // True when an output port is currently holding a flit until ack.
    sc_lv<34> tx_buffer[PORT_NUM];         // Stable output flit register used while waiting for downstream ack.

    // Stall Control + Arbiter: allocation state for packet-level output locking.
    int vc_state[PORT_NUM][VC_NUM]; // Allocated output port for each input VC; -1 means unallocated.
    int out_port_lock[PORT_NUM];    // Encoded input-port/VC pair that owns each output until packet tail.
    int output_rr_start[PORT_NUM];  // Round-robin starting input port for each output arbiter.

    // Input Sync state: body/tail flits follow the VC chosen by the header.
    int rx_current_vc[PORT_NUM]; // Current packet VC for each input port; -1 means waiting for a header.
    int router_id;               // Router coordinate ID in the 4x4 mesh, used by XY routing.

    void init(int id)
    {
        router_id = id;
    }

    int flit_type(const sc_lv<34> &flit)
    {
        return flit.range(33, 32).to_uint();
    }

    int flit_dest_id(const sc_lv<34> &flit)
    {
        return flit.range(31, 16).to_uint();
    }

    // Routing Computation: deterministic XY routing for a 4x4 mesh.
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

    // Kept as a compatibility name for older code/comments.
    int get_xy_route(int current_id, int dest_id)
    {
        return routing_computation(current_id, dest_id);
    }

    // ElasticBuffer capacity check.
    bool eb_has_space(int p, int vc)
    {
        return in_q[p][vc].size() < VC_DEPTH;
    }

    bool vc_has_space(int p, int vc)
    {
        return eb_has_space(p, vc);
    }

    // Output Sync buffer capacity check. When this is full, backward blocking is asserted.
    bool output_buffer_has_space(int p)
    {
        return out_q[p].size() < OUT_DEPTH;
    }

    bool out_has_space(int p)
    {
        return output_buffer_has_space(p);
    }

    bool any_vc_has_space(int p)
    {
        for (int v = 0; v < VC_NUM; v++)
            if (eb_has_space(p, v))
                return true;
        return false;
    }

    void clear_flit_queue(std::queue<sc_lv<34>> & q)
    {
        while (!q.empty())
            q.pop();
    }

    // One-Hot to 2D-Map idea: spread directions over two VCs to reduce head-of-line blocking.
    int preferred_vc_for_output(int target_out)
    {
        if (target_out == EAST || target_out == WEST)
            return 0;
        return 1;
    }

    // ElasticBuffer write selector: a header chooses a VC, body/tail reuse that VC.
    int select_input_vc(int input_port, int target_out)
    {
        int preferred = preferred_vc_for_output(target_out);
        int other = 1 - preferred;

        if (eb_has_space(input_port, preferred) &&
            in_q[input_port][preferred].size() <= in_q[input_port][other].size() + 1)
            return preferred;
        if (eb_has_space(input_port, other))
            return other;
        if (eb_has_space(input_port, preferred))
            return preferred;
        return -1;
    }

    int choose_vc(int p, int target_out)
    {
        return select_input_vc(p, target_out);
    }

    int encode_lock(int input_port, int vc)
    {
        return input_port * VC_NUM + vc;
    }

    // Backward Blocking Selector: compute the ready signal returned to the upstream link.
    bool backward_blocking_selector(int input_port, bool accepted, int accepted_type)
    {
        if (accepted)
        {
            if (accepted_type == TAIL_FLIT)
                return any_vc_has_space(input_port);
            if (rx_current_vc[input_port] != -1)
                return eb_has_space(input_port, rx_current_vc[input_port]);
            return any_vc_has_space(input_port);
        }

        if (in_req[input_port].read() != 1)
            return any_vc_has_space(input_port);

        sc_lv<34> incoming = in_flit[input_port].read();
        int type = flit_type(incoming);

        if (type == HEAD_FLIT)
        {
            int target_out = routing_computation(router_id, flit_dest_id(incoming));
            return select_input_vc(input_port, target_out) != -1;
        }

        if (rx_current_vc[input_port] != -1)
            return eb_has_space(input_port, rx_current_vc[input_port]);

        return false;
    }

    // Input Sync + ElasticBuffer: accept one flit from a valid-ready input link.
    InputSyncResult input_sync_and_eb_write(int input_port)
    {
        InputSyncResult result;
        bool current_ready = out_ack[input_port].read();

        if (in_req[input_port].read() != 1 || !current_ready)
            return result;

        sc_lv<34> incoming = in_flit[input_port].read();
        int type = flit_type(incoming);
        int vc = rx_current_vc[input_port];

        if (type == HEAD_FLIT)
        {
            int target_out = routing_computation(router_id, flit_dest_id(incoming));
            vc = select_input_vc(input_port, target_out);
        }

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

    // Input side stage in the slide: Input Sync -> ElasticBuffer -> backward ack.
    void input_port_stage(int input_port)
    {
        while (true)
        {
            if (rst.read() == 0)
            {
                rx_current_vc[input_port] = -1;
                out_ack[input_port].write(false);
                wait();
                continue;
            }

            InputSyncResult result = input_sync_and_eb_write(input_port);
            bool next_ready = backward_blocking_selector(input_port, result.accepted, result.flit_type);
            out_ack[input_port].write(next_ready);
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

    // Stall Control Arbiter: one round-robin arbiter per output port.
    SwitchGrant stall_control_arbiter(int out, bool input_used[PORT_NUM])
    {
        SwitchGrant grant;

        if (!output_buffer_has_space(out))
            return grant;

        for (int k = 0; k < PORT_NUM; k++)
        {
            int input_port = (output_rr_start[out] + k) % PORT_NUM;
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

                    int target_out = routing_computation(router_id, flit_dest_id(candidate));
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

    // Crossbar Switch: move the granted flit from an input VC to the selected output buffer.
    void crossbar_switch_transfer(int out, const SwitchGrant &grant, bool input_used[PORT_NUM])
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

    void reset_switch_allocation_state()
    {
        for (int i = 0; i < PORT_NUM; i++)
        {
            out_port_lock[i] = -1;
            output_rr_start[i] = 0;
            clear_flit_queue(out_q[i]);

            for (int v = 0; v < VC_NUM; v++)
            {
                vc_state[i][v] = -1;
                clear_flit_queue(in_q[i][v]);
            }
        }
    }

    // Middle stages : Routing Computation -> Stall Control/Arbiter -> Crossbar Switch.
    void route_compute_arbiter_crossbar_stage()
    {
        while (true)
        {
            if (rst.read() == 0)
            {
                reset_switch_allocation_state();
                wait();
                continue;
            }

            bool input_used[PORT_NUM];
            for (int i = 0; i < PORT_NUM; i++)
                input_used[i] = false;

            for (int out = 0; out < PORT_NUM; out++)
            {
                SwitchGrant grant = stall_control_arbiter(out, input_used);
                crossbar_switch_transfer(out, grant, input_used);
            }

            wait();
        }
    }

    // Output Sync: hold a flit stable until the downstream ack completes the handshake.
    void output_sync_stage(int output_port)
    {
        while (true)
        {
            if (rst.read() == 0)
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

            if (tx_active[output_port] && in_ack[output_port].read() == 1)
            {
                out_q[output_port].pop();
                tx_active[output_port] = false;
            }

            if (!tx_active[output_port] && !out_q[output_port].empty())
            {
                tx_buffer[output_port] = out_q[output_port].front();
                tx_active[output_port] = true;
            }

            if (tx_active[output_port])
            {
                out_flit[output_port].write(tx_buffer[output_port]);
                out_req[output_port].write(1);
            }
            else
            {
                out_req[output_port].write(0);
            }

            wait();
        }
    }

    void rx_thread_0() { input_port_stage(0); }
    void rx_thread_1() { input_port_stage(1); }
    void rx_thread_2() { input_port_stage(2); }
    void rx_thread_3() { input_port_stage(3); }
    void rx_thread_4() { input_port_stage(4); }

    void route_thread() { route_compute_arbiter_crossbar_stage(); }

    void tx_thread_0() { output_sync_stage(0); }
    void tx_thread_1() { output_sync_stage(1); }
    void tx_thread_2() { output_sync_stage(2); }
    void tx_thread_3() { output_sync_stage(3); }
    void tx_thread_4() { output_sync_stage(4); }

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