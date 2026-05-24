#include "clockreset.h"
#include "core.h"
#include "router.h"
#include "systemc.h"
#include "controller.h"
#include "ROM.h"
#include <sstream>

int sc_main(int argc, char* argv[])
{
    // =======================
    //   signals declaration
    // =======================
    sc_signal < bool > clk;
    sc_signal < bool > rst;

    
    sc_signal < int > layer_id;
    sc_signal < bool > layer_id_type;
    sc_signal < bool > layer_id_valid;

    sc_signal < float > data;
    sc_signal < bool > data_valid;


    // =======================
    //   modules declaration
    // =======================
    sc_trace_file *tf = NULL;
    // tf = sc_create_vcd_trace_file("wave");
    sc_trace(tf, clk, "clk");
    sc_trace(tf, rst, "rst");



    Clock m_clock("m_clock", 10);
    Reset m_reset("m_reset", 15);
    ROM   m_rom("m_rom");


    // =======================
    //   modules connection
    // =======================
    m_clock( clk );
    m_reset( rst );

    m_rom.clk( clk );
    m_rom.rst( rst );
    m_rom.layer_id( layer_id );
    m_rom.layer_id_type( layer_id_type );
    m_rom.layer_id_valid( layer_id_valid );
    m_rom.data( data );
    m_rom.data_valid( data_valid );

  









    sc_start(10, SC_NS);
    // sc_start();
    return 0;
}