#include "Pattern.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iomanip>

using namespace std;

struct Result
{
	int id;
	double softmax_val;
	double linear_val;
};

// 排序規則：機率 (softmax_val) 大的排前面
bool compare(const Result &a, const Result &b)
{
	return a.softmax_val > b.softmax_val;
}

Pattern::Pattern(sc_module_name name, string img_file)
	: sc_module(name), img_name(img_file)
{
	SC_THREAD(feed_data);
	sensitive << clock.pos();
	async_reset_signal_is(rst, true);

	SC_THREAD(check_result);
	sensitive << clock.pos();
	async_reset_signal_is(rst, true);
}

void Pattern::feed_data()
{
	// [🛡️ Reset 幽靈防護盾]
	wait();

	while (true)
	{
		string path = "../../00_TESTBED/data/" + img_name;
		ifstream inputFile(path.c_str());

		if (!inputFile.is_open())
		{
			cerr << "[Pattern] Error: Cannot open " << path << endl;
			sc_stop();
			wait();
		}

		// cout << "\n[Pattern] Start feeding " << img_name << " into FIFO..." << endl;

		double val;
		int count = 0;

		// 直接把資料狂塞進水管裡
		while (inputFile >> val)
		{
			img_out.write(val);
			count++;
			wait();
		}
		inputFile.close();

		// cout << "[Pattern] Successfully fed " << count << " pixels." << endl;

		// 餵完一張圖就進入無限休眠
		while (true)
			wait();
	}
}

void Pattern::check_result()
{
	wait();

	while (true)
	{
		// cout << "[Pattern] Waiting for Neural Network to process..." << endl;
		vector<Result> results(NUM_CLASSES);

		// 預防死結的關鍵：先收 FC8，再收 Softmax
		// 先從 FC8 旁通水管接收 1000 個原始數值 (val)
		for (int i = 0; i < NUM_CLASSES; i++)
		{
			results[i].id = i;
			results[i].linear_val = linear_in.read();
			wait();
		}

		// 接著等待 Softmax 算完，接收 1000 個機率
		for (int i = 0; i < NUM_CLASSES; i++)
		{
			results[i].softmax_val = softmax_in.read();
			wait();
		}

		// ==========================================
		// 資料收完，開始讀取類別名稱與排版列印
		// ==========================================
		ifstream class_file("../../00_TESTBED/data/imagenet_classes.txt");
		if (!class_file.is_open())
		{
			cerr << "Error opening file: imagenet_classes.txt" << endl;
			sc_stop();
			wait();
		}

		vector<string> class_name;
		string class_name_element;
		while (getline(class_file, class_name_element))
		{
			class_name.push_back(class_name_element);
		}
		class_file.close();

		// 根據 Softmax 機率由大到小排序
		sort(results.begin(), results.end(), compare);

		// 列印 Top 100
		cout << "\nTop 100 classes:" << endl;
		cout << "=================================================" << endl;
		cout << fixed << setprecision(2);
		cout << right << setw(5) << "idx"
			 << " | " << setw(8) << "val"
			 << " | " << setw(11) << "possibility"
			 << " | " << "class name" << endl;
		cout << "-------------------------------------------------" << endl;

		for (int i = 0; i < 100; ++i)
		{
			cout << right << setw(5) << results[i].id
				 << " | " << setw(8) << results[i].linear_val			  // 原生 FIFO 沒有接 val 訊號，顯示 N/A
				 << " | " << setw(11) << (results[i].softmax_val * 100.0) // 轉為 %
				 << " | " << class_name[results[i].id] << endl;
		}

		cout << "=================================================\n"
			 << endl;

		// 任務完成，停止模擬
		sc_stop();
		wait();
	}
}