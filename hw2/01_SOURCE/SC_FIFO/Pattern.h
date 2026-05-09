#ifndef PATTERN_H
#define PATTERN_H

#include <systemc.h>
#include <string>

#define IN_H 224
#define IN_W 224
#define CHANNEL 3
#define NUM_CLASSES 1000

SC_MODULE(Pattern)
{
	sc_in_clk clock;
	sc_in<bool> rst;
	sc_fifo_out<double> img_out;
	sc_fifo_in<double> softmax_in;
	sc_fifo_in<double> linear_in;

	std::string img_name;

	void feed_data();
	void check_result();

	SC_HAS_PROCESS(Pattern);
	Pattern(sc_module_name name, std::string img_file);
};

#endif