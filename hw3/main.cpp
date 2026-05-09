#include "systemc.h"
#include "clockreset.h"
#include "core.h"
#include "router.h"
#include <cstdio>

int sc_main(int argc, char* argv[])
{
    // =======================
    //   signals declaration
    // =======================
    sc_signal < bool > clk;
    sc_signal < bool > rst;

    // Core <-> Router (Port 4) 專用訊號
    sc_signal< sc_lv<34> > c2r_flit[16];
    sc_signal< bool >      c2r_req[16];
    sc_signal< bool >      c2r_ack[16];

    sc_signal< sc_lv<34> > r2c_flit[16];
    sc_signal< bool >      r2c_req[16];
    sc_signal< bool >      r2c_ack[16];

    // Router <-> Router (四個方向) 的實體訊號
    sc_signal< sc_lv<34> > r2r_flit[16][4];
    sc_signal< bool >      r2r_req[16][4];
    sc_signal< bool >      r2r_ack[16][4];

    // 邊界路由器的 Dummy Signals (虛擬訊號，專門用來塞住沒有連線的邊界 Port，避免 E109 錯誤)
    sc_signal< sc_lv<34> > dummy_out_flit[16][4];
    sc_signal< bool >      dummy_out_req[16][4];
    sc_signal< bool >      dummy_in_ack[16][4];
    sc_signal< sc_lv<34> > dummy_in_flit[16][4];
    sc_signal< bool >      dummy_in_req[16][4];
    sc_signal< bool >      dummy_out_ack[16][4];

    // =======================
    //   modules declaration
    // =======================
    Clock m_clock("m_clock", 10);
    Reset m_reset("m_reset", 15);

    Core* cores[16];
    Router* routers[16];

    for(int i=0; i<16; i++) {
        char cname[20], rname[20];
        sprintf(cname, "core_%d", i);
        sprintf(rname, "router_%d", i);

        cores[i] = new Core(cname);
        routers[i] = new Router(rname);

        cores[i]->init(i);
        routers[i]->init(i);
    }

    // =======================
    //   modules connection
    // =======================
    m_clock( clk );
    m_reset( rst );

    for(int i=0; i<16; i++) {
        cores[i]->clk(clk);
        cores[i]->rst(rst);
        routers[i]->clk(clk);
        routers[i]->rst(rst);

        // ---------------------------------------------------
        //  綁定 Local Port (Port 4): Core <-> Router
        // ---------------------------------------------------
        // Core (Tx) -> Router (Rx Port 4)
        cores[i]->flit_tx(c2r_flit[i]);
        cores[i]->req_tx(c2r_req[i]);
        cores[i]->ack_tx(c2r_ack[i]);

        routers[i]->in_flit[4](c2r_flit[i]);
        routers[i]->in_req[4](c2r_req[i]);
        routers[i]->out_ack[4](c2r_ack[i]);

        // Router (Tx Port 4) -> Core (Rx)
        routers[i]->out_flit[4](r2c_flit[i]);
        routers[i]->out_req[4](r2c_req[i]);
        routers[i]->in_ack[4](r2c_ack[i]);

        cores[i]->flit_rx(r2c_flit[i]);
        cores[i]->req_rx(r2c_req[i]);
        cores[i]->ack_rx(r2c_ack[i]);

        // ---------------------------------------------------
        //  綁定 網格網路 (Mesh Network) 實體連線
        // ---------------------------------------------------
        int x = i % 4;
        int y = i / 4;

        // 【Port 0: North 往北】
        if (y > 0) { // 如果上面有鄰居
            int north = i - 4;
            // 自己的輸出 接到 往北的專屬線路
            routers[i]->out_flit[0](r2r_flit[i][0]);
            routers[i]->out_req[0](r2r_req[i][0]);
            routers[i]->in_ack[0](r2r_ack[i][0]);
            // 北邊鄰居的輸入 接到 同一條專屬線路 (鄰居的 Port 1 是南向輸入)
            routers[north]->in_flit[1](r2r_flit[i][0]);
            routers[north]->in_req[1](r2r_req[i][0]);
            routers[north]->out_ack[1](r2r_ack[i][0]);
        } else { // 邊界：塞入虛擬訊號
            routers[i]->out_flit[0](dummy_out_flit[i][0]);
            routers[i]->out_req[0](dummy_out_req[i][0]);
            routers[i]->in_ack[0](dummy_in_ack[i][0]);
            routers[i]->in_flit[0](dummy_in_flit[i][0]);
            routers[i]->in_req[0](dummy_in_req[i][0]);
            routers[i]->out_ack[0](dummy_out_ack[i][0]);
        }

        // 【Port 1: South 往南】
        if (y < 3) { // 如果下面有鄰居
            int south = i + 4;
            routers[i]->out_flit[1](r2r_flit[i][1]);
            routers[i]->out_req[1](r2r_req[i][1]);
            routers[i]->in_ack[1](r2r_ack[i][1]);
            // 南邊鄰居的輸入 接到 同一條專屬線路 (鄰居的 Port 0 是北向輸入)
            routers[south]->in_flit[0](r2r_flit[i][1]);
            routers[south]->in_req[0](r2r_req[i][1]);
            routers[south]->out_ack[0](r2r_ack[i][1]);
        } else {
            routers[i]->out_flit[1](dummy_out_flit[i][1]);
            routers[i]->out_req[1](dummy_out_req[i][1]);
            routers[i]->in_ack[1](dummy_in_ack[i][1]);
            routers[i]->in_flit[1](dummy_in_flit[i][1]);
            routers[i]->in_req[1](dummy_in_req[i][1]);
            routers[i]->out_ack[1](dummy_out_ack[i][1]);
        }

        // 【Port 2: East 往東】
        if (x < 3) { // 如果右邊有鄰居
            int east = i + 1;
            routers[i]->out_flit[2](r2r_flit[i][2]);
            routers[i]->out_req[2](r2r_req[i][2]);
            routers[i]->in_ack[2](r2r_ack[i][2]);
            // 東邊鄰居的輸入 接到 同一條專屬線路 (鄰居的 Port 3 是西向輸入)
            routers[east]->in_flit[3](r2r_flit[i][2]);
            routers[east]->in_req[3](r2r_req[i][2]);
            routers[east]->out_ack[3](r2r_ack[i][2]);
        } else {
            routers[i]->out_flit[2](dummy_out_flit[i][2]);
            routers[i]->out_req[2](dummy_out_req[i][2]);
            routers[i]->in_ack[2](dummy_in_ack[i][2]);
            routers[i]->in_flit[2](dummy_in_flit[i][2]);
            routers[i]->in_req[2](dummy_in_req[i][2]);
            routers[i]->out_ack[2](dummy_out_ack[i][2]);
        }

        // 【Port 3: West 往西】
        if (x > 0) { // 如果左邊有鄰居
            int west = i - 1;
            routers[i]->out_flit[3](r2r_flit[i][3]);
            routers[i]->out_req[3](r2r_req[i][3]);
            routers[i]->in_ack[3](r2r_ack[i][3]);
            // 西邊鄰居的輸入 接到 同一條專屬線路 (鄰居的 Port 2 是東向輸入)
            routers[west]->in_flit[2](r2r_flit[i][3]);
            routers[west]->in_req[2](r2r_req[i][3]);
            routers[west]->out_ack[2](r2r_ack[i][3]);
        } else {
            routers[i]->out_flit[3](dummy_out_flit[i][3]);
            routers[i]->out_req[3](dummy_out_req[i][3]);
            routers[i]->in_ack[3](dummy_in_ack[i][3]);
            routers[i]->in_flit[3](dummy_in_flit[i][3]);
            routers[i]->in_req[3](dummy_in_req[i][3]);
            routers[i]->out_ack[3](dummy_out_ack[i][3]);
        }
    }

    sc_start();
    return 0;
}