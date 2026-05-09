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
	// ===== Ports =====
	sc_in_clk clock;
	sc_out<bool> rst;
	sc_out<bool> in_valid; // 通知下一層 (ZeroPadding) 圖片已準備好

	sc_vector<sc_out<double>> img{"img", 150528};

	sc_in<bool> out_valid; // 接收最後一層 (Softmax) 的完成訊號
	// 對接 sc_main 中的 buf_softmax_out
	sc_vector<sc_in<double>> output_softmax{"output_softmax", 1000};
	sc_vector<sc_in<double>> output_linear{"output_linear", 1000};

	string img_name;
	bool printed;

	uint cycle;

	void run();

	SC_HAS_PROCESS(Pattern);
	Pattern(sc_module_name name, string img_name)
		: sc_module(name), img_name(img_name)
	{
		SC_THREAD(run); // 改用 THREAD 以便精確控制 Reset 與 wait()
		sensitive << clock.pos();
	}
};
#endif