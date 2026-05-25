#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

namespace
{
    const int NODE_COUNT = 16;

    bool wait_edge[NODE_COUNT][NODE_COUNT];
    string wait_detail[NODE_COUNT][NODE_COUNT];
    bool reported_cycle[NODE_COUNT][NODE_COUNT];

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
        cout << "[DEADLOCK] Circular waiting detected: "
             << node_name(waiter_core) << " -> "
             << cycle_string(path, 0, waiter_core) << endl;
        cout << "[DEADLOCK] Wait-for edges:" << endl;
        cout << "[DEADLOCK]   " << wait_detail[waiter_core][owner_core] << endl;
        print_wait_edge_details(path, waiter_core);
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
