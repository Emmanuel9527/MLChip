#include "clockreset.h"
#include "controller.h"
#include "core.h"
#include "router.h"
#include "ROM.h"
#include "systemc.h"
#include <cstdio>

int sc_main(int argc, char* argv[])
{
    // Global clock and active-high reset used by all modules.
    sc_signal < bool > clk;
    sc_signal < bool > rst;

    // Controller <-> ROM command and data signals.
    sc_signal < int > layer_id;
    sc_signal < bool > layer_id_type;
    sc_signal < bool > layer_id_valid;

    sc_signal < float > data;
    sc_signal < bool > data_valid;

    // Controller <-> router[0] local port.
    // The Controller acts as the master NoC endpoint at mesh node 0.
    sc_signal < sc_lv<34> > ctrl2r_flit;
    sc_signal < bool > ctrl2r_req;
    sc_signal < bool > ctrl2r_ack;
    sc_signal < sc_lv<34> > r2ctrl_flit;
    sc_signal < bool > r2ctrl_req;
    sc_signal < bool > r2ctrl_ack;

    // Worker core <-> local router ports. Core 0 is reserved for the controller.
    // Worker Core/PE nodes are placed at mesh ids 1 through 15.
    sc_signal < sc_lv<34> > c2r_flit[16];
    sc_signal < bool > c2r_req[16];
    sc_signal < bool > c2r_ack[16];
    sc_signal < sc_lv<34> > r2c_flit[16];
    sc_signal < bool > r2c_req[16];
    sc_signal < bool > r2c_ack[16];

    // Router-to-router mesh links: NORTH, SOUTH, EAST, WEST.
    sc_signal < sc_lv<34> > r2r_flit[16][4];
    sc_signal < bool > r2r_req[16][4];
    sc_signal < bool > r2r_ack[16][4];

    // Boundary dummy links.
    // Edge routers still have all five ports connected, but off-chip mesh
    // directions are tied to idle dummy signals.
    sc_signal < sc_lv<34> > dummy_out_flit[16][4];
    sc_signal < bool > dummy_out_req[16][4];
    sc_signal < bool > dummy_in_ack[16][4];
    sc_signal < sc_lv<34> > dummy_in_flit[16][4];
    sc_signal < bool > dummy_in_req[16][4];
    sc_signal < bool > dummy_out_ack[16][4];

    // Fixed assignment-provided modules plus the new mesh controller.
    Clock m_clock("m_clock", 10);
    Reset m_reset("m_reset", 15);
    ROM m_rom("m_rom");
    Controller m_controller("m_controller");

    // 4x4 mesh routers and worker cores.
    Router* routers[16];
    Core* cores[16];

    // Instantiate all routers first so neighbor connections can reference any id.
    for (int i = 0; i < 16; i++)
    {
        char rname[20];
        sprintf(rname, "router_%d", i);
        routers[i] = new Router(rname);
        routers[i]->init(i);
        cores[i] = NULL;
    }

    // Instantiate worker cores for nodes 1..15. Node 0 local port is Controller.
    for (int i = 1; i < 16; i++)
    {
        char cname[20];
        sprintf(cname, "core_%d", i);
        cores[i] = new Core(cname);
        cores[i]->init(i);
    }

    // Bind clock/reset generators.
    m_clock( clk );
    m_reset( rst );

    // Bind ROM to Controller-facing command/data wires.
    m_rom.clk( clk );
    m_rom.rst( rst );
    m_rom.layer_id( layer_id );
    m_rom.layer_id_type( layer_id_type );
    m_rom.layer_id_valid( layer_id_valid );
    m_rom.data( data );
    m_rom.data_valid( data_valid );

    // Bind Controller to ROM and router[0] local port signals.
    m_controller.clk( clk );
    m_controller.rst( rst );
    m_controller.layer_id( layer_id );
    m_controller.layer_id_type( layer_id_type );
    m_controller.layer_id_valid( layer_id_valid );
    m_controller.data( data );
    m_controller.data_valid( data_valid );
    m_controller.flit_tx( ctrl2r_flit );
    m_controller.req_tx( ctrl2r_req );
    m_controller.ack_tx( ctrl2r_ack );
    m_controller.flit_rx( r2ctrl_flit );
    m_controller.req_rx( r2ctrl_req );
    m_controller.ack_rx( r2ctrl_ack );

    // Connect each router local port and its four mesh directions.
    for (int i = 0; i < 16; i++)
    {
        routers[i]->clk( clk );
        routers[i]->rst( rst );

        if (i == 0)
        {
            // Node 0 local port is the Controller network interface.
            routers[i]->in_flit[LOCAL]( ctrl2r_flit );
            routers[i]->in_req[LOCAL]( ctrl2r_req );
            routers[i]->out_ack[LOCAL]( ctrl2r_ack );
            routers[i]->out_flit[LOCAL]( r2ctrl_flit );
            routers[i]->out_req[LOCAL]( r2ctrl_req );
            routers[i]->in_ack[LOCAL]( r2ctrl_ack );
        }
        else
        {
            // Nodes 1..15 local ports are worker Core/PE network interfaces.
            cores[i]->clk( clk );
            cores[i]->rst( rst );
            cores[i]->flit_tx( c2r_flit[i] );
            cores[i]->req_tx( c2r_req[i] );
            cores[i]->ack_tx( c2r_ack[i] );
            cores[i]->flit_rx( r2c_flit[i] );
            cores[i]->req_rx( r2c_req[i] );
            cores[i]->ack_rx( r2c_ack[i] );

            routers[i]->in_flit[LOCAL]( c2r_flit[i] );
            routers[i]->in_req[LOCAL]( c2r_req[i] );
            routers[i]->out_ack[LOCAL]( c2r_ack[i] );
            routers[i]->out_flit[LOCAL]( r2c_flit[i] );
            routers[i]->out_req[LOCAL]( r2c_req[i] );
            routers[i]->in_ack[LOCAL]( r2c_ack[i] );
        }

        int x = i % 4;
        int y = i / 4;

        // NORTH output of this router connects to SOUTH input of the router above.
        if (y > 0)
        {
            int north = i - 4;
            routers[i]->out_flit[NORTH]( r2r_flit[i][NORTH] );
            routers[i]->out_req[NORTH]( r2r_req[i][NORTH] );
            routers[i]->in_ack[NORTH]( r2r_ack[i][NORTH] );
            routers[north]->in_flit[SOUTH]( r2r_flit[i][NORTH] );
            routers[north]->in_req[SOUTH]( r2r_req[i][NORTH] );
            routers[north]->out_ack[SOUTH]( r2r_ack[i][NORTH] );
        }
        else
        {
            // Top boundary: no northern neighbor.
            routers[i]->out_flit[NORTH]( dummy_out_flit[i][NORTH] );
            routers[i]->out_req[NORTH]( dummy_out_req[i][NORTH] );
            routers[i]->in_ack[NORTH]( dummy_in_ack[i][NORTH] );
            routers[i]->in_flit[NORTH]( dummy_in_flit[i][NORTH] );
            routers[i]->in_req[NORTH]( dummy_in_req[i][NORTH] );
            routers[i]->out_ack[NORTH]( dummy_out_ack[i][NORTH] );
        }

        // SOUTH output of this router connects to NORTH input of the router below.
        if (y < 3)
        {
            int south = i + 4;
            routers[i]->out_flit[SOUTH]( r2r_flit[i][SOUTH] );
            routers[i]->out_req[SOUTH]( r2r_req[i][SOUTH] );
            routers[i]->in_ack[SOUTH]( r2r_ack[i][SOUTH] );
            routers[south]->in_flit[NORTH]( r2r_flit[i][SOUTH] );
            routers[south]->in_req[NORTH]( r2r_req[i][SOUTH] );
            routers[south]->out_ack[NORTH]( r2r_ack[i][SOUTH] );
        }
        else
        {
            // Bottom boundary: no southern neighbor.
            routers[i]->out_flit[SOUTH]( dummy_out_flit[i][SOUTH] );
            routers[i]->out_req[SOUTH]( dummy_out_req[i][SOUTH] );
            routers[i]->in_ack[SOUTH]( dummy_in_ack[i][SOUTH] );
            routers[i]->in_flit[SOUTH]( dummy_in_flit[i][SOUTH] );
            routers[i]->in_req[SOUTH]( dummy_in_req[i][SOUTH] );
            routers[i]->out_ack[SOUTH]( dummy_out_ack[i][SOUTH] );
        }

        // EAST output of this router connects to WEST input of the right neighbor.
        if (x < 3)
        {
            int east = i + 1;
            routers[i]->out_flit[EAST]( r2r_flit[i][EAST] );
            routers[i]->out_req[EAST]( r2r_req[i][EAST] );
            routers[i]->in_ack[EAST]( r2r_ack[i][EAST] );
            routers[east]->in_flit[WEST]( r2r_flit[i][EAST] );
            routers[east]->in_req[WEST]( r2r_req[i][EAST] );
            routers[east]->out_ack[WEST]( r2r_ack[i][EAST] );
        }
        else
        {
            // Right boundary: no eastern neighbor.
            routers[i]->out_flit[EAST]( dummy_out_flit[i][EAST] );
            routers[i]->out_req[EAST]( dummy_out_req[i][EAST] );
            routers[i]->in_ack[EAST]( dummy_in_ack[i][EAST] );
            routers[i]->in_flit[EAST]( dummy_in_flit[i][EAST] );
            routers[i]->in_req[EAST]( dummy_in_req[i][EAST] );
            routers[i]->out_ack[EAST]( dummy_out_ack[i][EAST] );
        }

        // WEST output of this router connects to EAST input of the left neighbor.
        if (x > 0)
        {
            int west = i - 1;
            routers[i]->out_flit[WEST]( r2r_flit[i][WEST] );
            routers[i]->out_req[WEST]( r2r_req[i][WEST] );
            routers[i]->in_ack[WEST]( r2r_ack[i][WEST] );
            routers[west]->in_flit[EAST]( r2r_flit[i][WEST] );
            routers[west]->in_req[EAST]( r2r_req[i][WEST] );
            routers[west]->out_ack[EAST]( r2r_ack[i][WEST] );
        }
        else
        {
            // Left boundary: no western neighbor.
            routers[i]->out_flit[WEST]( dummy_out_flit[i][WEST] );
            routers[i]->out_req[WEST]( dummy_out_req[i][WEST] );
            routers[i]->in_ack[WEST]( dummy_in_ack[i][WEST] );
            routers[i]->in_flit[WEST]( dummy_in_flit[i][WEST] );
            routers[i]->in_req[WEST]( dummy_in_req[i][WEST] );
            routers[i]->out_ack[WEST]( dummy_out_ack[i][WEST] );
        }
    }

    // Initialize dummy boundary sources to idle and their sink acks to ready.
    for (int i = 0; i < 16; i++)
    {
        for (int p = 0; p < 4; p++)
        {
            dummy_in_flit[i][p].write(0);
            dummy_in_req[i][p].write(false);
            dummy_in_ack[i][p].write(true);
        }
    }

    // Start SystemC simulation.
    sc_start();
    return 0;
}
