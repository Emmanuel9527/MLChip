#include "axi_dma.h"
#include "clockreset.h"
#include "controller.h"
#include "core.h"
#include "dram.h"
#include "noc.h"
#include "router.h"
#include "systemc.h"
#include <cstdio>

extern "C" void deadlock_watchdog_clear();

int sc_main(int argc, char* argv[])
{
    deadlock_watchdog_clear();

    // Global clock and active-low reset used by all modules.
    sc_signal < bool > clk;
    sc_signal < bool > reset_n;

    // Controller <-> AXI DMA command and data-stream signals.
    sc_signal < bool > dma_cmd_valid;
    sc_signal < bool > dma_cmd_ready;
    sc_signal < bool > dma_cmd_write;
    sc_signal < unsigned int > dma_cmd_addr;
    sc_signal < unsigned int > dma_cmd_len;
    sc_signal < bool > dma_done;
    sc_signal < float > dma_read_data;
    sc_signal < bool > dma_read_valid;
    sc_signal < bool > dma_read_ready;
    sc_signal < float > dma_write_data;
    sc_signal < bool > dma_write_valid;
    sc_signal < bool > dma_write_ready;

    // AXI4-like DMA <-> DRAM channels.
    sc_signal < unsigned int > araddr;
    sc_signal < unsigned int > arlen;
    sc_signal < unsigned int > arsize;
    sc_signal < bool > arvalid;
    sc_signal < bool > arready;
    sc_signal < sc_uint<32> > rdata;
    sc_signal < bool > rvalid;
    sc_signal < bool > rready;
    sc_signal < bool > rlast;

    sc_signal < unsigned int > awaddr;
    sc_signal < unsigned int > awlen;
    sc_signal < unsigned int > awsize;
    sc_signal < bool > awvalid;
    sc_signal < bool > awready;
    sc_signal < sc_uint<32> > wdata;
    sc_signal < bool > wvalid;
    sc_signal < bool > wready;
    sc_signal < bool > wlast;
    sc_signal < unsigned int > bresp;
    sc_signal < bool > bvalid;
    sc_signal < bool > bready;

    // Controller <-> router[0] HOST port.
    // The Controller acts as an external host and does not occupy PE0's LOCAL port.
    sc_signal < Flit > ctrl2r_flit;
    sc_signal < bool > ctrl2r_req;
    sc_signal < bool > ctrl2r_ack;
    sc_signal < Flit > r2ctrl_flit;
    sc_signal < bool > r2ctrl_req;
    sc_signal < bool > r2ctrl_ack;

    // Worker core <-> local router ports. Core/PE nodes occupy mesh ids 0..15.
    sc_signal < Flit > c2r_flit[16];
    sc_signal < bool > c2r_req[16];
    sc_signal < bool > c2r_ack[16];
    sc_signal < Flit > r2c_flit[16];
    sc_signal < bool > r2c_req[16];
    sc_signal < bool > r2c_ack[16];

    // Router-to-router mesh links: NORTH, SOUTH, EAST, WEST.
    sc_signal < Flit > r2r_flit[16][4];
    sc_signal < bool > r2r_req[16][4];
    sc_signal < bool > r2r_ack[16][4];

    // Boundary dummy links.
    // Edge routers still have all five ports connected, but off-chip mesh
    // directions are tied to idle dummy signals.
    sc_signal < Flit > dummy_out_flit[16][4];
    sc_signal < bool > dummy_out_req[16][4];
    sc_signal < bool > dummy_in_ack[16][4];
    sc_signal < Flit > dummy_in_flit[16][4];
    sc_signal < bool > dummy_in_req[16][4];
    sc_signal < bool > dummy_out_ack[16][4];

    // HOST ports exist on every router; only router 0 is connected to Controller.
    sc_signal < Flit > dummy_host_out_flit[16];
    sc_signal < bool > dummy_host_out_req[16];
    sc_signal < bool > dummy_host_in_ack[16];
    sc_signal < Flit > dummy_host_in_flit[16];
    sc_signal < bool > dummy_host_in_req[16];
    sc_signal < bool > dummy_host_out_ack[16];

    // Fixed assignment-provided modules plus DRAM, AXI DMA, and mesh controller.
    Clock m_clock("m_clock", 10);
    Reset m_reset("m_reset", 15);
    DRAM m_dram("m_dram");
    AxiDma m_dma("m_dma");
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

    // Instantiate worker cores for all 16 mesh nodes.
    for (int i = 0; i < 16; i++)
    {
        char cname[20];
        sprintf(cname, "core_%d", i);
        cores[i] = new Core(cname);
        cores[i]->init(i);
    }

    // Bind clock/reset generators.
    m_clock( clk );
    m_reset( reset_n );

    // Bind DMA to DRAM through the AXI4-like read/write channels.
    m_dram.clk( clk );
    m_dram.reset_n( reset_n );
    m_dram.araddr( araddr );
    m_dram.arlen( arlen );
    m_dram.arsize( arsize );
    m_dram.arvalid( arvalid );
    m_dram.arready( arready );
    m_dram.rdata( rdata );
    m_dram.rvalid( rvalid );
    m_dram.rready( rready );
    m_dram.rlast( rlast );
    m_dram.awaddr( awaddr );
    m_dram.awlen( awlen );
    m_dram.awsize( awsize );
    m_dram.awvalid( awvalid );
    m_dram.awready( awready );
    m_dram.wdata( wdata );
    m_dram.wvalid( wvalid );
    m_dram.wready( wready );
    m_dram.wlast( wlast );
    m_dram.bresp( bresp );
    m_dram.bvalid( bvalid );
    m_dram.bready( bready );

    m_dma.clk( clk );
    m_dma.reset_n( reset_n );
    m_dma.cmd_valid( dma_cmd_valid );
    m_dma.cmd_ready( dma_cmd_ready );
    m_dma.cmd_write( dma_cmd_write );
    m_dma.cmd_addr( dma_cmd_addr );
    m_dma.cmd_len( dma_cmd_len );
    m_dma.done( dma_done );
    m_dma.read_data( dma_read_data );
    m_dma.read_valid( dma_read_valid );
    m_dma.read_ready( dma_read_ready );
    m_dma.write_data( dma_write_data );
    m_dma.write_valid( dma_write_valid );
    m_dma.write_ready( dma_write_ready );
    m_dma.araddr( araddr );
    m_dma.arlen( arlen );
    m_dma.arsize( arsize );
    m_dma.arvalid( arvalid );
    m_dma.arready( arready );
    m_dma.rdata( rdata );
    m_dma.rvalid( rvalid );
    m_dma.rready( rready );
    m_dma.rlast( rlast );
    m_dma.awaddr( awaddr );
    m_dma.awlen( awlen );
    m_dma.awsize( awsize );
    m_dma.awvalid( awvalid );
    m_dma.awready( awready );
    m_dma.wdata( wdata );
    m_dma.wvalid( wvalid );
    m_dma.wready( wready );
    m_dma.wlast( wlast );
    m_dma.bresp( bresp );
    m_dma.bvalid( bvalid );
    m_dma.bready( bready );

    // Bind Controller to DMA and router[0] HOST port signals.
    m_controller.clk( clk );
    m_controller.reset_n( reset_n );
    m_controller.dma_cmd_valid( dma_cmd_valid );
    m_controller.dma_cmd_ready( dma_cmd_ready );
    m_controller.dma_cmd_write( dma_cmd_write );
    m_controller.dma_cmd_addr( dma_cmd_addr );
    m_controller.dma_cmd_len( dma_cmd_len );
    m_controller.dma_done( dma_done );
    m_controller.dma_read_data( dma_read_data );
    m_controller.dma_read_valid( dma_read_valid );
    m_controller.dma_read_ready( dma_read_ready );
    m_controller.dma_write_data( dma_write_data );
    m_controller.dma_write_valid( dma_write_valid );
    m_controller.dma_write_ready( dma_write_ready );
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
        routers[i]->reset_n( reset_n );

        // All LOCAL ports are worker Core/PE network interfaces.
        cores[i]->clk( clk );
        cores[i]->reset_n( reset_n );
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

        if (i == 0)
        {
            routers[i]->in_flit[HOST]( ctrl2r_flit );
            routers[i]->in_req[HOST]( ctrl2r_req );
            routers[i]->out_ack[HOST]( ctrl2r_ack );
            routers[i]->out_flit[HOST]( r2ctrl_flit );
            routers[i]->out_req[HOST]( r2ctrl_req );
            routers[i]->in_ack[HOST]( r2ctrl_ack );
        }
        else
        {
            routers[i]->out_flit[HOST]( dummy_host_out_flit[i] );
            routers[i]->out_req[HOST]( dummy_host_out_req[i] );
            routers[i]->in_ack[HOST]( dummy_host_in_ack[i] );
            routers[i]->in_flit[HOST]( dummy_host_in_flit[i] );
            routers[i]->in_req[HOST]( dummy_host_in_req[i] );
            routers[i]->out_ack[HOST]( dummy_host_out_ack[i] );
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
        dummy_host_in_flit[i].write(0);
        dummy_host_in_req[i].write(false);
        dummy_host_in_ack[i].write(true);
    }

    // Start SystemC simulation.
    sc_start();
    return 0;
}
