#include "Pattern.h"

bool compare(const pair<double, int> &a, const pair<double, int> &b)
{
	return a.first > b.first;
}

void Pattern::run()
{
	// ===== Step 1: Reset 階段 =====
	rst.write(1);
	in_valid.write(0);
	wait(); // 等待一個週期

	rst.write(0);
	wait(); // 再等一個週期確保 Reset 訊號被所有模組抓到

	ifstream inputFile(("../../00_TESTBED/data/" + img_name).c_str());
	if (!inputFile.is_open())
	{
		cerr << "Error opening file: " << img_name << endl;
		sc_stop();
		return;
	}

	double img_element;
	int cnt = 0;
	// cout << "[" << sc_time_stamp() << "] Pattern: 開始讀取圖片檔案..." << endl; // debug

	while (inputFile >> img_element && cnt < 150528)
	{
		// sc_vector 必須使用 .write()
		img[cnt].write(img_element);
		cnt++;
	}
	inputFile.close();

	// cout << "[" << sc_time_stamp() << "] Pattern: 圖片讀取完畢，發出 in_valid 啟動脈衝！" << endl; // debug

	// 資料擺好了，拉高訊號啟動 AlexNet
	in_valid.write(1);
	wait();			   // 維持一個週期，讓 ZeroPadding 確實抓到 1
	in_valid.write(0); // 立刻收回訊號，避免 ZeroPadding 算完後重複啟動

	// cout << "[" << sc_time_stamp() << "] Pattern: 進入休眠，等待 Softmax 傳回完成訊號..." << endl; // debug

	// ===== Step 3: 等待最後一層 Softmax 完成 =====
	// 這裡會卡住直到 Softmax 的 out_valid 傳回來
	while (out_valid.read() == false)
	{
		wait();
	}

	// cout << "[Pattern] Received completion signal from Softmax!" << endl; // debug

	// 既然收到了，就可以把啟動訊號放掉
	in_valid.write(0);

	// ===== Step 4: 顯示結果 (與你原本邏輯相同) =====
	ifstream class_file("../../00_TESTBED/data/imagenet_classes.txt");
	if (!class_file.is_open())
	{
		cerr << "Error opening file: imagenet_classes.txt" << endl;
		return;
	}

	vector<string> class_name;
	string class_name_element;
	while (getline(class_file, class_name_element))
	{
		class_name.push_back(class_name_element);
	}

	vector<pair<double, int>> indexed_values;
	for (int i = 0; i < 1000; ++i)
	{
		indexed_values.push_back(make_pair(output_softmax[i].read(), i));
	}

	sort(indexed_values.begin(), indexed_values.end(), compare);

	cout << "Top 100 classes:" << endl;
	cout << "=================================================" << endl;
	cout << fixed << setprecision(2);
	cout << right << setw(5) << "idx"
		 << " | " << setw(8) << "val"
		 << " | " << setw(11) << "possibility"
		 << " | " << "class name" << endl;
	cout << "-------------------------------------------------" << endl;

	for (int i = 0; i < 100; ++i)
	{
		int idx = indexed_values[i].second;
		cout << right << setw(5) << idx
			 << " | " << setw(8) << output_linear[idx].read() // 注意這裡也需要 read()
			 << " | " << setw(11) << (indexed_values[i].first * 100.0)
			 << " | " << class_name[idx] << endl;
	}

	cout << "=================================================" << endl;

	// 完成所有動作，停止模擬
	sc_stop();
}