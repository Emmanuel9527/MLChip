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

	sc_vector<sc_in<double>> output_softmax{"output_softmax", 1000}; // 前面的 output_softmax 是 C++的變數名，後面的 "output_softmax" 是systemC裡用來稱呼此元件的名字
	sc_vector<sc_in<double>> output_linear{"output_linear", 1000};

	sc_out<bool> rst, in_valid;
	sc_vector<sc_out<double>> img{"img", 150528};

	string img_name;
	bool printed;

	uint cycle;

	void run();

	SC_HAS_PROCESS(Pattern);															// 如果 constructor 不是標準寫法，就需要用 SC_HAS_PROCESS 告知在這個 module 裡有 process
	Pattern(sc_module_name name, string img_name) : sc_module(name), img_name(img_name) // 建立 Pattern 時把 name 傳給 SystemC（命名 module）和把圖片檔名存到 member 變數
	{
		SC_METHOD(run);
		printed = false;
		// in_valid= 0;
		cycle = 0;
		sensitive << clock.neg();
	}
};
#endif

// #ifndef PATTERN_H
// #define PATTERN_H
//...
// #endif