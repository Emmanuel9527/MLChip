#ifndef ROUTER_H
#define ROUTER_H

#include "noc.h"
#include "systemc.h"
#include <queue>

#define NORTH 0
#define SOUTH 1
#define EAST  2
#define WEST  3
#define LOCAL 4
#define BROADCAST_ID 65535

using namespace std;

extern "C" void deadlock_watchdog_wait_edge(int waiter_core,
                                            int owner_core,
                                            int router_id,
                                            int out_port,
                                            int input_port,
                                            int vc);
extern "C" void deadlock_watchdog_clear_waiter(int core_id);
extern "C" void deadlock_watchdog_note_flit_transfer(int router_id,
                                                     int out_port,
                                                     int flit_type,
                                                     unsigned int payload,
                                                     int dest_id,
                                                     int source_id);

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

    enum RoutingMode {
        ROUTING_FULL_XY = 0,
        ROUTING_ESCAPE_ADAPTIVE = 1
    };

    // Switch routing policy here.
    // ROUTING_FULL_XY: every unicast packet uses deterministic XY routing.
    // ROUTING_ESCAPE_ADAPTIVE: VC0 uses XY as escape VC, VC1 uses adaptive minimal routing.
    static const int ROUTING_MODE = ROUTING_ESCAPE_ADAPTIVE;

    enum FlitType {
        BODY_FLIT = NOC_BODY_FLIT,
        TAIL_FLIT = NOC_TAIL_FLIT,
        HEAD_FLIT = NOC_HEAD_FLIT
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
    sc_in  < bool >  reset_n;
    sc_in  < bool >  clk;

    // Output-side valid-ready links.
    sc_out < Flit > out_flit[PORT_NUM];
    sc_out < bool > out_req[PORT_NUM];
    sc_in  < bool > in_ack[PORT_NUM];

    // Input-side valid-ready links.
    sc_in  < Flit > in_flit[PORT_NUM];
    sc_in  < bool > in_req[PORT_NUM];
    sc_out < bool > out_ack[PORT_NUM];

    int router_id;

    // Input virtual-channel buffers.
    queue<Flit > in_q[PORT_NUM][VC_NUM];

    // Output queues and output-sync holding registers.
    queue<Flit > out_q[PORT_NUM];
    bool tx_active[PORT_NUM];
    Flit tx_buffer[PORT_NUM];

    // Packet-level allocation state.
    int vc_state[PORT_NUM][VC_NUM];
    int out_port_lock[PORT_NUM];
    int out_lock_source[PORT_NUM];
    int output_rr_start[PORT_NUM];
    int rx_current_vc[PORT_NUM];
    bool broadcast_active[PORT_NUM];
    bool broadcast_mask[PORT_NUM][PORT_NUM];
    bool dbg_in_packet_active[PORT_NUM];
    int dbg_in_packet_source[PORT_NUM];
    int dbg_in_packet_dest[PORT_NUM];
    int dbg_in_packet_flits[PORT_NUM];
    bool dbg_out_packet_active[PORT_NUM];
    int dbg_out_packet_source[PORT_NUM];
    int dbg_out_packet_dest[PORT_NUM];
    int dbg_out_packet_flits[PORT_NUM];
    int dbg_out_packet_idle_cycles[PORT_NUM];

    // main.cpp assigns this router's mesh node id after construction.
    void init(int id)
    {
        router_id = id;
    }

#ifdef DEBUG
    static const bool ROUTER_VERBOSE_DEBUG = true;
#else
    static const bool ROUTER_VERBOSE_DEBUG = false;
#endif

    // Decode flit type: 2=HEAD, 0=BODY, 1=TAIL.
    int flit_type(const Flit &flit)
    {
        return get_flit_type(flit);
    }

    // Destination id is stored in payload lane 0 of the HEAD flit.
    int flit_dest_id(const Flit &flit)
    {
        return get_header_dest(flit);
    }

    int flit_source_id(const Flit &flit)
    {
        return get_header_source(flit);
    }

    const char *port_name(int port)
    {
        switch (port)
        {
        case NORTH:
            return "NORTH";
        case SOUTH:
            return "SOUTH";
        case EAST:
            return "EAST";
        case WEST:
            return "WEST";
        case LOCAL:
            return "LOCAL";
        default:
            return "UNKNOWN";
        }
    }

    bool is_broadcast_flit(const Flit &flit)
    {
        return flit_dest_id(flit) == BROADCAST_ID;
    }

    void debug_note_input_accept(int input_port, const Flit &flit)
    {
        if (!ROUTER_VERBOSE_DEBUG)
            return;

        int type = flit_type(flit);

        if (type == HEAD_FLIT)
        {
            dbg_in_packet_active[input_port] = true;
            dbg_in_packet_source[input_port] = flit_source_id(flit);
            dbg_in_packet_dest[input_port] = flit_dest_id(flit);
            dbg_in_packet_flits[input_port] = 1;
            return;
        }

        if (!dbg_in_packet_active[input_port])
            return;

        dbg_in_packet_flits[input_port]++;
        if (type == TAIL_FLIT)
        {
            cout << "[ROUTER_IN] router " << router_id
                 << " input " << port_name(input_port)
                 << " accepted packet dest=" << dbg_in_packet_dest[input_port]
                 << " source=" << dbg_in_packet_source[input_port]
                 << ", flits=" << dbg_in_packet_flits[input_port]
                 << "." << endl;
            dbg_in_packet_active[input_port] = false;
        }
    }

    void debug_note_output_transfer(int output_port, const Flit &flit)
    {
        if (!ROUTER_VERBOSE_DEBUG)
            return;

        int type = flit_type(flit);

        if (type == HEAD_FLIT)
        {
            dbg_out_packet_active[output_port] = true;
            dbg_out_packet_source[output_port] = flit_source_id(flit);
            dbg_out_packet_dest[output_port] = flit_dest_id(flit);
            dbg_out_packet_flits[output_port] = 1;
            dbg_out_packet_idle_cycles[output_port] = 0;
            return;
        }

        if (!dbg_out_packet_active[output_port])
            return;

        dbg_out_packet_flits[output_port]++;
        dbg_out_packet_idle_cycles[output_port] = 0;
        if (type == TAIL_FLIT)
        {
            cout << "[ROUTER_OUT] router " << router_id
                 << " output " << port_name(output_port)
                 << " sent packet dest=" << dbg_out_packet_dest[output_port]
                 << " source=" << dbg_out_packet_source[output_port]
                 << ", flits=" << dbg_out_packet_flits[output_port]
                 << "." << endl;
            dbg_out_packet_active[output_port] = false;
        }
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

    void clear_queue(queue<Flit > &q)
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
    int preferred_vc_for_header(const Flit &flit)
    {
        (void)flit;
        if (ROUTING_MODE == ROUTING_FULL_XY)
            return 0;
        return 1;
    }

    int choose_input_vc(int input_port, const Flit &flit)
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
        int source_id = flit_source_id(in_flit[input_port].read());
        for (int out = 0; out < PORT_NUM; out++)
        {
            if (broadcast_mask[input_port][out])
            {
                out_port_lock[out] = lock_id;
                out_lock_source[out] = source_id;
            }
        }
    }

    void release_broadcast_outputs(int input_port)
    {
        int lock_id = encode_broadcast_lock(input_port);
        for (int out = 0; out < PORT_NUM; out++)
        {
            if (out_port_lock[out] == lock_id)
            {
                deadlock_watchdog_clear_waiter(out_lock_source[out]);
                out_port_lock[out] = -1;
                out_lock_source[out] = -1;
            }
        }
    }

    bool ready_for_broadcast_input(int input_port)
    {
        if (broadcast_active[input_port])
            return broadcast_outputs_have_space(input_port);

        if (!in_req[input_port].read())
            return true;

        Flit incoming = in_flit[input_port].read();
        if (flit_type(incoming) != HEAD_FLIT || !is_broadcast_flit(incoming))
            return false;
        build_broadcast_mask(input_port);
        return broadcast_outputs_can_allocate(input_port);
    }

    bool accept_broadcast_input(int input_port)
    {
        if (!in_req[input_port].read())
            return false;

        Flit incoming = in_flit[input_port].read();
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

    int route_for_vc(const Flit &header, int vc)
    {
        int dest_id = flit_dest_id(header);
        if (ROUTING_MODE == ROUTING_FULL_XY)
            return xy_route(router_id, dest_id);

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

        if (rx_current_vc[input_port] != -1)
            return eb_has_space(input_port, rx_current_vc[input_port]);

        if (!in_req[input_port].read())
            return any_vc_has_space(input_port);

        Flit incoming = in_flit[input_port].read();
        int type = flit_type(incoming);

        if (type == HEAD_FLIT && is_broadcast_flit(incoming))
            return ready_for_broadcast_input(input_port);

        if (type == HEAD_FLIT)
            return choose_input_vc(input_port, incoming) != -1;

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

        Flit incoming = in_flit[input_port].read();
        int type = flit_type(incoming);
        int vc = rx_current_vc[input_port];

        if (type == HEAD_FLIT)
            vc = choose_input_vc(input_port, incoming);

        if (vc == -1 || !eb_has_space(input_port, vc))
            return result;

        if (type == HEAD_FLIT)
            rx_current_vc[input_port] = vc;

        in_q[input_port][vc].push(incoming);
        debug_note_input_accept(input_port, incoming);
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
            if (!reset_n.read())
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

    bool can_allocate_output(int out, int input_port, int vc, int source_id)
    {
        if (out_port_lock[out] == -1)
        {
            out_port_lock[out] = encode_lock(input_port, vc);
            out_lock_source[out] = source_id;
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

                Flit candidate = in_q[input_port][vc].front();
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

                    int waiter_source = flit_source_id(candidate);
                    if (!can_allocate_output(out, input_port, vc, waiter_source))
                    {
                        deadlock_watchdog_wait_edge(waiter_source,
                                                    out_lock_source[out],
                                                    router_id,
                                                    out,
                                                    input_port,
                                                    vc);
                        continue;
                    }
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

        Flit flit = in_q[grant.input_port][grant.vc].front();
        int type = flit_type(flit);
        int owner_source = out_lock_source[out];
        deadlock_watchdog_clear_waiter(owner_source);

        out_q[out].push(flit);
        in_q[grant.input_port][grant.vc].pop();
        input_used[grant.input_port] = true;
        output_rr_start[out] = (grant.input_port + 1) % PORT_NUM;

        if (type == TAIL_FLIT)
        {
            out_port_lock[out] = -1;
            out_lock_source[out] = -1;
            vc_state[grant.input_port][grant.vc] = -1;
        }
    }

    void reset_allocation_state()
    {
        for (int p = 0; p < PORT_NUM; p++)
        {
            out_port_lock[p] = -1;
            out_lock_source[p] = -1;
            output_rr_start[p] = 0;
            broadcast_active[p] = false;
            dbg_in_packet_active[p] = false;
            dbg_in_packet_source[p] = -1;
            dbg_in_packet_dest[p] = -1;
            dbg_in_packet_flits[p] = 0;
            dbg_out_packet_active[p] = false;
            dbg_out_packet_source[p] = -1;
            dbg_out_packet_dest[p] = -1;
            dbg_out_packet_flits[p] = 0;
            dbg_out_packet_idle_cycles[p] = 0;
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
            if (!reset_n.read())
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
            if (!reset_n.read())
            {
                Flit zero_flit;
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
                int type = flit_type(tx_buffer[output_port]);
                int dest_id = (type == HEAD_FLIT) ? flit_dest_id(tx_buffer[output_port]) : -1;
                int source_id = (type == HEAD_FLIT) ? flit_source_id(tx_buffer[output_port]) : -1;
                if (ROUTER_VERBOSE_DEBUG && type == HEAD_FLIT)
                {
                    cout << "[ROUTER_PKT] router " << router_id
                         << " output " << port_name(output_port)
                         << " HEAD dest=" << dest_id
                         << " source=" << source_id << "." << endl;
                }
                else if (ROUTER_VERBOSE_DEBUG && type == TAIL_FLIT)
                {
                    cout << "[ROUTER_PKT] router " << router_id
                         << " output " << port_name(output_port)
                         << " TAIL." << endl;
                }
                debug_note_output_transfer(output_port, tx_buffer[output_port]);
                if (ROUTER_VERBOSE_DEBUG)
                {
                    deadlock_watchdog_note_flit_transfer(router_id,
                                                         output_port,
                                                         type,
                                                         get_flit_word(tx_buffer[output_port], 0),
                                                         dest_id,
                                                         source_id);
                }
                if (!out_q[output_port].empty())
                    out_q[output_port].pop();
                tx_active[output_port] = false;
                out_req[output_port].write(false);
                wait();
                continue;
            }

            if (ROUTER_VERBOSE_DEBUG &&
                dbg_out_packet_active[output_port] &&
                !tx_active[output_port] &&
                out_q[output_port].empty())
            {
                dbg_out_packet_idle_cycles[output_port]++;
                if (dbg_out_packet_idle_cycles[output_port] == 50000 ||
                    dbg_out_packet_idle_cycles[output_port] % 200000 == 0)
                {
                    cout << "[ROUTER_STUCK] router " << router_id
                         << " output " << port_name(output_port)
                         << " waiting for rest of packet dest="
                         << dbg_out_packet_dest[output_port]
                         << " source=" << dbg_out_packet_source[output_port]
                         << ", flits_sent=" << dbg_out_packet_flits[output_port]
                         << ", idle_cycles=" << dbg_out_packet_idle_cycles[output_port]
                         << "." << endl;
                }
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
        Flit zero_flit;
        zero_flit = 0;

        for (int p = 0; p < PORT_NUM; p++)
        {
            out_port_lock[p] = -1;
            out_lock_source[p] = -1;
            output_rr_start[p] = 0;
            rx_current_vc[p] = -1;
            broadcast_active[p] = false;
            dbg_in_packet_active[p] = false;
            dbg_in_packet_source[p] = -1;
            dbg_in_packet_dest[p] = -1;
            dbg_in_packet_flits[p] = 0;
            dbg_out_packet_active[p] = false;
            dbg_out_packet_source[p] = -1;
            dbg_out_packet_dest[p] = -1;
            dbg_out_packet_flits[p] = 0;
            dbg_out_packet_idle_cycles[p] = 0;
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
