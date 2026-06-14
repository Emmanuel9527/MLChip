#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

namespace
{
    const int NODE_COUNT = 16;
    const int PORT_COUNT = 5;
    const int RECENT_FLIT_COUNT = 32;

    bool wait_edge[NODE_COUNT][NODE_COUNT];
    string wait_detail[NODE_COUNT][NODE_COUNT];
    bool reported_cycle[NODE_COUNT][NODE_COUNT];

    struct RecentFlit
    {
        bool valid;
        int router_id;
        int out_port;
        int flit_type;
        unsigned int payload;
        int dest_id;
        int source_id;
        unsigned int sequence;
    };

    RecentFlit recent_flits[RECENT_FLIT_COUNT];
    unsigned int flit_sequence;

    string node_name(int node)
    {
        if (node == 0)
            return "Controller";
        ostringstream oss;
        oss << "PE" << node;
        return oss.str();
    }

    string cycle_string(const vector<int> &path, int start_index, int closing_node)
    {
        ostringstream oss;
        for (size_t i = start_index; i < path.size(); i++)
            oss << node_name(path[i]) << " -> ";
        oss << node_name(closing_node);
        return oss.str();
    }

    string port_name(int port)
    {
        switch (port)
        {
        case 0:
            return "NORTH";
        case 1:
            return "SOUTH";
        case 2:
            return "EAST";
        case 3:
            return "WEST";
        case 4:
            return "LOCAL";
        default:
            return "UNKNOWN";
        }
    }

    string flit_type_name(int type)
    {
        switch (type)
        {
        case 0:
            return "BODY";
        case 1:
            return "TAIL";
        case 2:
            return "HEAD";
        default:
            return "INVALID";
        }
    }

    void print_flit_payload(unsigned int payload)
    {
#ifdef DEBUG
        union
        {
            float fval;
            unsigned int ival;
        } converter;

        converter.ival = payload;
        cout << "payload_raw=0x" << hex << payload << dec
             << ", payload_float=" << converter.fval;
#else
        (void)payload;
#endif
    }

    int find_recent_matching_flit(int flit_type, unsigned int payload)
    {
        int best = -1;
        unsigned int best_sequence = 0;

        for (int i = 0; i < RECENT_FLIT_COUNT; i++)
        {
            if (!recent_flits[i].valid)
                continue;
            if (recent_flits[i].flit_type != flit_type ||
                recent_flits[i].payload != payload)
                continue;
            if (best == -1 || recent_flits[i].sequence > best_sequence)
            {
                best = i;
                best_sequence = recent_flits[i].sequence;
            }
        }

        return best;
    }

    void print_recent_link_transfers(int router_id, int out_port, int limit)
    {
#ifdef DEBUG
        int printed = 0;
        unsigned int next_sequence = flit_sequence;

        while (printed < limit && next_sequence > 0)
        {
            next_sequence--;
            for (int i = 0; i < RECENT_FLIT_COUNT; i++)
            {
                if (!recent_flits[i].valid ||
                    recent_flits[i].sequence != next_sequence ||
                    recent_flits[i].router_id != router_id ||
                    recent_flits[i].out_port != out_port)
                    continue;

                const RecentFlit &flit = recent_flits[i];
                cout << "[FLIT_DEBUG]     seq " << flit.sequence
                     << ": type=" << flit.flit_type
                     << "(" << flit_type_name(flit.flit_type) << "), ";
                print_flit_payload(flit.payload);
                if (flit.flit_type == 2)
                {
                    cout << ", dest=" << flit.dest_id
                         << ", source=" << flit.source_id;
                }
                cout << endl;
                printed++;
                break;
            }
        }

        if (printed == 0)
        {
            cout << "[FLIT_DEBUG]     No recent transfers recorded on router "
                 << router_id << " output " << port_name(out_port) << "." << endl;
        }
#else
        (void)router_id;
        (void)out_port;
        (void)limit;
#endif
    }

    bool dfs_cycle(int current, int target, bool visited[], vector<int> &path)
    {
        visited[current] = true;
        path.push_back(current);

        for (int next = 0; next < NODE_COUNT; next++)
        {
            if (!wait_edge[current][next])
                continue;
            if (next == target)
                return true;
            if (!visited[next] && dfs_cycle(next, target, visited, path))
                return true;
        }

        path.pop_back();
        return false;
    }

    void print_wait_edge_details(const vector<int> &path, int closing_node)
    {
#ifdef DEBUG
        for (size_t i = 0; i + 1 < path.size(); i++)
        {
            int from = path[i];
            int to = path[i + 1];
            if (!wait_detail[from][to].empty())
                cout << "[DEADLOCK]   " << wait_detail[from][to] << endl;
        }

        int from = path.back();
        int to = closing_node;
        if (!wait_detail[from][to].empty())
            cout << "[DEADLOCK]   " << wait_detail[from][to] << endl;
#else
        (void)path;
        (void)closing_node;
#endif
    }
}

extern "C" void deadlock_watchdog_clear()
{
    for (int i = 0; i < NODE_COUNT; i++)
    {
        for (int j = 0; j < NODE_COUNT; j++)
        {
            wait_edge[i][j] = false;
            wait_detail[i][j] = "";
            reported_cycle[i][j] = false;
        }
    }

    for (int i = 0; i < RECENT_FLIT_COUNT; i++)
        recent_flits[i].valid = false;
    flit_sequence = 0;
}

extern "C" void deadlock_watchdog_note_flit_transfer(int router_id,
                                                     int out_port,
                                                     int flit_type,
                                                     unsigned int payload,
                                                     int dest_id,
                                                     int source_id)
{
    int index = flit_sequence % RECENT_FLIT_COUNT;
    recent_flits[index].valid = true;
    recent_flits[index].router_id = router_id;
    recent_flits[index].out_port = out_port;
    recent_flits[index].flit_type = flit_type;
    recent_flits[index].payload = payload;
    recent_flits[index].dest_id = dest_id;
    recent_flits[index].source_id = source_id;
    recent_flits[index].sequence = flit_sequence;
    flit_sequence++;
}

extern "C" void deadlock_watchdog_report_unexpected_flit(int expected_node,
                                                         int flit_type,
                                                         unsigned int payload,
                                                         int active,
                                                         int packet_flits)
{
#ifdef DEBUG
    cout << "[FLIT_DEBUG] Unexpected flit at " << node_name(expected_node)
         << ": type=" << flit_type << "(" << flit_type_name(flit_type) << "), ";
    print_flit_payload(payload);
    cout << ", active=" << active
         << ", packet_flits=" << packet_flits << "." << endl;

    int recent = find_recent_matching_flit(flit_type, payload);
    if (recent == -1)
    {
        cout << "[FLIT_DEBUG]   No matching recent router transfer was recorded." << endl;
        return;
    }

    const RecentFlit &flit = recent_flits[recent];
    cout << "[FLIT_DEBUG]   Last matching transfer: router " << flit.router_id
         << " output " << port_name(flit.out_port)
         << ", sequence " << flit.sequence << ".";

    if (flit.flit_type == 2)
    {
        cout << " dest=" << flit.dest_id
             << ", source=" << flit.source_id;
    }
    cout << endl;

    if (expected_node == 0)
    {
        cout << "[FLIT_DEBUG]   Recent router 0 LOCAL transfers to Controller:" << endl;
        print_recent_link_transfers(0, 4, 8);
    }
#else
    (void)expected_node;
    (void)flit_type;
    (void)payload;
    (void)active;
    (void)packet_flits;
#endif
}

extern "C" void deadlock_watchdog_wait_edge(int waiter_core,
                                            int owner_core,
                                            int router_id,
                                            int out_port,
                                            int input_port,
                                            int vc)
{
    if (waiter_core < 0 || waiter_core >= NODE_COUNT ||
        owner_core < 0 || owner_core >= NODE_COUNT ||
        waiter_core == owner_core)
        return;

    wait_edge[waiter_core][owner_core] = true;

    ostringstream detail;
    detail << node_name(waiter_core) << " waits for " << node_name(owner_core)
           << " at router " << router_id
           << ", output port " << out_port
           << ", input port " << input_port
           << ", VC " << vc << ".";
    wait_detail[waiter_core][owner_core] = detail.str();

    if (reported_cycle[waiter_core][owner_core])
        return;

    bool visited[NODE_COUNT];
    for (int i = 0; i < NODE_COUNT; i++)
        visited[i] = false;

    vector<int> path;
    if (dfs_cycle(owner_core, waiter_core, visited, path))
    {
        reported_cycle[waiter_core][owner_core] = true;
#ifdef DEBUG
        cout << "[DEADLOCK] Circular waiting detected: "
             << node_name(waiter_core) << " -> "
             << cycle_string(path, 0, waiter_core) << endl;
        cout << "[DEADLOCK] Wait-for edges:" << endl;
        cout << "[DEADLOCK]   " << wait_detail[waiter_core][owner_core] << endl;
        print_wait_edge_details(path, waiter_core);
#endif
    }
}

extern "C" void deadlock_watchdog_release_core(int core_id)
{
    if (core_id < 0 || core_id >= NODE_COUNT)
        return;

    for (int i = 0; i < NODE_COUNT; i++)
    {
        wait_edge[core_id][i] = false;
        wait_detail[core_id][i] = "";
        wait_edge[i][core_id] = false;
        wait_detail[i][core_id] = "";
    }
}

extern "C" void deadlock_watchdog_clear_waiter(int core_id)
{
    if (core_id < 0 || core_id >= NODE_COUNT)
        return;

    for (int i = 0; i < NODE_COUNT; i++)
    {
        wait_edge[core_id][i] = false;
        wait_detail[core_id][i] = "";
    }
}
