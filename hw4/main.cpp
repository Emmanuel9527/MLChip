#include "clockreset.h"
#include "core.h"
#include "router.h"
#include "systemc.h"
#include "controller.h"
#include "ROM.h"

int sc_main(int argc, char* argv[])
{
    sc_signal < bool > clk;
    sc_signal < bool > rst;

    sc_signal < int > layer_id;
    sc_signal < bool > layer_id_type;
    sc_signal < bool > layer_id_valid;

    sc_signal < float > data;
    sc_signal < bool > data_valid;

    sc_signal < sc_lv<34> > ctrl_flit_tx;
    sc_signal < bool > ctrl_req_tx;
    sc_signal < bool > ctrl_ack_tx;
    sc_signal < sc_lv<34> > ctrl_flit_rx;
    sc_signal < bool > ctrl_req_rx;
    sc_signal < bool > ctrl_ack_rx;
    sc_signal < sc_lv<34> > dummy_in_flit[4];
    sc_signal < bool > dummy_in_req[4];
    sc_signal < bool > dummy_out_ack[4];
    sc_signal < sc_lv<34> > dummy_out_flit[4];
    sc_signal < bool > dummy_out_req[4];
    sc_signal < bool > dummy_in_ack[4];

    Clock m_clock("m_clock", 10);
    Reset m_reset("m_reset", 15);
    ROM m_rom("m_rom");
    Controller m_controller("m_controller");
    Router m_router("m_router");

    m_clock( clk );
    m_reset( rst );

    m_rom.clk( clk );
    m_rom.rst( rst );
    m_rom.layer_id( layer_id );
    m_rom.layer_id_type( layer_id_type );
    m_rom.layer_id_valid( layer_id_valid );
    m_rom.data( data );
    m_rom.data_valid( data_valid );

    m_controller.clk( clk );
    m_controller.rst( rst );
    m_controller.layer_id( layer_id );
    m_controller.layer_id_type( layer_id_type );
    m_controller.layer_id_valid( layer_id_valid );
    m_controller.data( data );
    m_controller.data_valid( data_valid );
    m_controller.flit_tx( ctrl_flit_tx );
    m_controller.req_tx( ctrl_req_tx );
    m_controller.ack_tx( ctrl_ack_tx );
    m_controller.flit_rx( ctrl_flit_rx );
    m_controller.req_rx( ctrl_req_rx );
    m_controller.ack_rx( ctrl_ack_rx );

    m_router.clk( clk );
    m_router.rst( rst );
    m_router.in_flit[LOCAL]( ctrl_flit_tx );
    m_router.in_req[LOCAL]( ctrl_req_tx );
    m_router.out_ack[LOCAL]( ctrl_ack_tx );
    m_router.out_flit[LOCAL]( ctrl_flit_rx );
    m_router.out_req[LOCAL]( ctrl_req_rx );
    m_router.in_ack[LOCAL]( ctrl_ack_rx );

    for (int i = 0; i < 4; i++)
    {
        m_router.in_flit[i]( dummy_in_flit[i] );
        m_router.in_req[i]( dummy_in_req[i] );
        m_router.out_ack[i]( dummy_out_ack[i] );
        m_router.out_flit[i]( dummy_out_flit[i] );
        m_router.out_req[i]( dummy_out_req[i] );
        m_router.in_ack[i]( dummy_in_ack[i] );
        dummy_in_flit[i].write(0);
        dummy_in_req[i].write(false);
        dummy_in_ack[i].write(true);
    }

    sc_start();
    return 0;
}
