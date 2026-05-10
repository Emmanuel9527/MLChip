#ifndef ROUTER_H
#define ROUTER_H

#include "systemc.h"
#include <queue>

SC_MODULE(Router)
{
    sc_in<bool> rst;
    sc_in<bool> clk;

    sc_out<sc_lv<34>> out_flit[5]; // lv = logic vector，4值邏輯
    sc_out<bool> out_req[5];       // 0: 沒有資料要送, 1: 有資料要送
    sc_in<bool> in_ack[5];         // 0: 對方告訴我他還沒收到資料, 1: 對方已經收到資料。用來握手用。

    sc_in<sc_lv<34>> in_flit[5]; // 輸入資料 (1個flit)
    sc_in<bool> in_req[5];       // 0: 無資料, 1: 有資料
    sc_out<bool> out_ack[5];     // 0: 告訴對方我還沒收到資料, 1: 我已經收到資料。用來握手用。

    std::queue<sc_lv<34>> in_q[5];  // 輸入緩衝區
    std::queue<sc_lv<34>> out_q[5]; // 輸出緩衝區

    int in_port_state[5]; // 記錄「第 i 號輸入門進來的資料，應該往哪個後門送？」(-1 代表閒置)
    int out_port_lock[5]; // 記錄「第 j 號輸出門，現在被哪個輸入門佔用了？」(-1 代表閒置)
    int router_id;

    void init(int id)
    {
        router_id = id;
    }

    // XY Routing 策略
    int get_xy_route(int current_id, int dest_id)
    {
        int cx = current_id % 4;
        int cy = current_id / 4;
        int dx = dest_id % 4;
        int dy = dest_id / 4;

        if (dx > cx)
            return 2; // East
        if (dx < cx)
            return 3; // West
        if (dy > cy)
            return 1; // South
        if (dy < cy)
            return 0; // North
        return 4;     // Local (Core)
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
            while (in_req[p].read() == 0)
                wait();
            sc_lv<34> f = in_flit[p].read();
            in_q[p].push(f);

            out_ack[p].write(1);
            while (in_req[p].read() == 1) // 等 sender 把 in_req[p] 拉回 0。這是 handshake 的另一半，避免同一筆資料被重複收。
                wait();
            out_ack[p].write(0);
            wait();
        }
    }

    // ==============================================================================
    // // 將 in_q 的資料根據規則丟進 out_q (包含 Lock 分配機制)
    // ==============================================================================
    void route_thread()
    {
        while (true)
        {
            for (int i = 0; i < 5; i++) // 每個 clock cycle 檢查 5 個 input port
            {
                if (!in_q[i].empty())
                {
                    sc_lv<34> f = in_q[i].front();
                    int type = f.range(33, 32).to_uint(); // 2: Header flit, 1: Tail flit, other: Body flit

                    // 如果該輸入埠還沒綁定輸出埠
                    if (in_port_state[i] == -1)
                    {
                        if (type == 2)
                        { // 遇到 Header 就從 flit 裡面取出目的地 router id
                            int dest_id = f.range(31, 16).to_uint();
                            int target_out = get_xy_route(router_id, dest_id);

                            // 檢查目標輸出埠是否被別的輸入埠佔用了
                            if (out_port_lock[target_out] == -1)
                            {
                                out_port_lock[target_out] = i; // 搶下鎖定權
                                in_port_state[i] = target_out; // 記錄自己的去向
                            }
                            else
                            {
                                // 輸出埠正在忙碌中。這個 flit 會繼續卡在 in_q[i].front()，等下一個 clock 再試一次。
                                continue;
                            }
                        }
                        else
                        {
                            // 防呆：如果狀態是閒置卻收到 Body/Tail (錯誤封包)，直接丟棄
                            in_q[i].pop();
                            continue;
                        }
                    }

                    // 如果該輸入埠已經有專屬的輸出埠鎖定了
                    if (in_port_state[i] != -1)
                    {
                        int target_out = in_port_state[i];
                        if (out_port_lock[target_out] == i) // 確認自己真的持有鎖
                        {
                            out_q[target_out].push(f); // 放心把資料丟進去
                            in_q[i].pop();

                            if (type == 1)
                            { // 遇到 Tail flit，傳輸結束，解除鎖定
                                out_port_lock[target_out] = -1;
                                in_port_state[i] = -1;
                            }
                        }
                    }
                }
            }
            wait(); // 一定要等下一個 clock cycle 才符合設計
        }
    }

    // ==============================================================================
    // 負責把 out_q[p] 裡的 flit 送到下一個 router / core，並用 req / ack 做 handshake
    // ==============================================================================

    void tx_thread_0() { tx_logic(0); }
    void tx_thread_1() { tx_logic(1); }
    void tx_thread_2() { tx_logic(2); }
    void tx_thread_3() { tx_logic(3); }
    void tx_thread_4() { tx_logic(4); }

    void tx_logic(int p)
    {
        while (true)
        {
            if (!out_q[p].empty())
            {
                sc_lv<34> f = out_q[p].front();
                out_req[p].write(1);
                out_flit[p].write(f);

                while (in_ack[p].read() == 0) // 對方還沒收到資料，持續等待
                    wait();
                out_q[p].pop();
                out_req[p].write(0);
                while (in_ack[p].read() == 1) // 等對方把 in_ack[p] 拉回 0。
                    wait();
            }
            wait();
        }
    }

    SC_HAS_PROCESS(Router);
    Router(sc_module_name name) : sc_module(name)
    {
        for (int i = 0; i < 5; i++)
        {
            in_port_state[i] = -1;
            out_port_lock[i] = -1;
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