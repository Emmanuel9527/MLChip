#ifndef PATTERN_H
#define PATTERN_H
#include <systemc.h>
#include <iostream>

#define CYCLE 40

#define IMG_CHANNEL 3
#define IMG_WEIGHT 224
#define IMG_HEIGHT 224

#include <time.h>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <iomanip>

using namespace std;

SC_MODULE(Pattern)
{
	sc_in_clk clock;
	sc_in<bool> out_valid;

	sc_vector<sc_in<double>> output_softmax{"output_softmax", 1000};
	sc_vector<sc_in<double>> output_linear{"output_linear", 1000};

	sc_out<bool> rst, in_valid;
	sc_vector<sc_out<double>> img{"img", 150528};

	string img_name;
	bool printed;

	uint cycle;

	void run();

	SC_HAS_PROCESS(Pattern);
	Pattern(sc_module_name name, string img_name) : sc_module(name), img_name(img_name)
	{
		SC_METHOD(run);
		printed = false;
		// in_valid= 0;
		cycle = 0;
		sensitive << clock.neg();
	}
};
#endif