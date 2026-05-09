#include "Pattern.h"

bool compare(const pair<double, int> &a, const pair<double, int> &b)
{
	return a.first > b.first; // 這樣可以從大到小排序
}

/*
這裡的 pair<double, int> 代表：第一項：softmax 機率，第二項：類別 index

傳參考呼叫，對x取alias (& 在datatype旁邊)
int& x;
pair<double,int>& a;

取址運算子(& 在變數旁邊)
&x;
*/

void Pattern::run()
{
	if (cycle == 0)
	{
		// cout << "Pattern reset" << endl;

		rst.write(1);
		in_valid.write(0);
	}
	else if (cycle == 1)
	{
		rst.write(0);
		ifstream inputFile(("data/" + img_name).c_str()); // 開檔案，例如 img_name = "dog.txt" 那它就會開 data/dog.txt

		double img_element;
		int cnt{0};
		vector<double> numbers;

		while (inputFile >> img_element) // 從檔案讀一個double，放進 img_element。inputFile >> img_element 會回傳 inputFile，然後 C++ 會判斷 stream 是否還是「有效狀態」
		{
			img[cnt] = (double)(img_element); // 在做型別轉換（type cast），(目標型別)(值)
			cnt++;
		}

		in_valid.write(1); // 輸入資料現在有效，可以讀了
	}
	else if (out_valid.read() == 1 && !printed)
	{
		ifstream class_file("data/imagenet_classes.txt"); // 讀類別名稱
		vector<string> class_name;
		string class_name_element;
		while (getline(class_file, class_name_element)) // class_name_element 一開始是空字串 ""，getline() 會把一整行讀進去
		{
			class_name.push_back(class_name_element); // v.push_back(x); 把 x 加到 v 的尾端
		}

		vector<pair<double, int>> indexed_values;

		for (int i = 0; i < 1000; ++i) // 收集 1000 維 softmax 結果，它把每個類別的資訊存成：(機率, 類別編號)
		{
			indexed_values.push_back(make_pair(output_softmax[i].read(), i));
		}

		sort(indexed_values.begin(), indexed_values.end(), compare); // 用剛剛那個 compare() 函式做排序

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
			cout << right << setw(5) << indexed_values[i].second
				 << " | " << setw(8) << double(output_linear[indexed_values[i].second].read())
				 << " | " << setw(11) << (indexed_values[i].first * 100)
				 << " | " << class_name[indexed_values[i].second] << endl;
		}

		cout << "=================================================" << endl;
		printed = true;
	}
	else
	{
		in_valid.write(0);
	}

	// cout << "Pattern cycle: " << cycle << endl;
	cycle++;
	if (cycle == CYCLE)
		exit(0);
	// sc_stop();
}
