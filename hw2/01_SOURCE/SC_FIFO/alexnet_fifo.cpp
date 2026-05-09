/*#include <systemc.h>

int sc_main(int argc, char *argv[])
{
    string img_name = argv[1];
    sc_clock clk("clk", 1, SC_NS);
    sc_signal<bool> reset;

    // You can add your code here

    return 0;
}
*/

#include <systemc.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "ZeroPadding.h"
#include "paddinglayer.h"
#include "convlayer.h"
#include "relu.h"
#include "maxpooling.h"
#include "fullyconneceted.h"
#include "softmax.h"
#include "Pattern.h"

using namespace std;

// ===== Conv1 spec =====
#define CONV1_IN_H 227
#define CONV1_IN_W 227
#define CONV1_IN_CH 3
#define CONV1_OUT_CH 64
#define CONV1_KERNEL 11
#define CONV1_STRIDE 4

#define CONV1_OUT_H ((CONV1_IN_H - CONV1_KERNEL) / CONV1_STRIDE + 1) // 55
#define CONV1_OUT_W ((CONV1_IN_W - CONV1_KERNEL) / CONV1_STRIDE + 1) // 55

// ===== ReLU1 spec =====
#define RELU1_SIZE (CONV1_OUT_H * CONV1_OUT_W * CONV1_OUT_CH)

// ===== Pool1 spec =====
#define POOL1_IN_H CONV1_OUT_H
#define POOL1_IN_W CONV1_OUT_W
#define POOL1_CH CONV1_OUT_CH
#define POOL1_KERNEL 3
#define POOL1_STRIDE 2

#define POOL1_OUT_H ((POOL1_IN_H - POOL1_KERNEL) / POOL1_STRIDE + 1) // 27
#define POOL1_OUT_W ((POOL1_IN_W - POOL1_KERNEL) / POOL1_STRIDE + 1) // 27

// ===== Pad2 / Conv2 / ReLU2 / Pool2 =====
#define PAD2_IN_H 27
#define PAD2_IN_W 27
#define PAD2_CH 64
#define PAD2_PAD 2
#define PAD2_OUT_H 31
#define PAD2_OUT_W 31

#define CONV2_IN_H PAD2_OUT_H
#define CONV2_IN_W PAD2_OUT_W
#define CONV2_IN_CH 64
#define CONV2_OUT_CH 192
#define CONV2_KERNEL 5
#define CONV2_STRIDE 1
#define CONV2_OUT_H ((CONV2_IN_H - CONV2_KERNEL) / CONV2_STRIDE + 1) // 27
#define CONV2_OUT_W ((CONV2_IN_W - CONV2_KERNEL) / CONV2_STRIDE + 1) // 27

#define RELU2_SIZE (CONV2_OUT_H * CONV2_OUT_W * CONV2_OUT_CH)

#define POOL2_IN_H CONV2_OUT_H
#define POOL2_IN_W CONV2_OUT_W
#define POOL2_CH CONV2_OUT_CH
#define POOL2_KERNEL 3
#define POOL2_STRIDE 2
#define POOL2_OUT_H ((POOL2_IN_H - POOL2_KERNEL) / POOL2_STRIDE + 1) // 13
#define POOL2_OUT_W ((POOL2_IN_W - POOL2_KERNEL) / POOL2_STRIDE + 1) // 13

// ===== Pad3 / Conv3 / ReLU3 =====
#define PAD3_IN_H POOL2_OUT_H
#define PAD3_IN_W POOL2_OUT_W
#define PAD3_CH POOL2_CH
#define PAD3_PAD 1
#define PAD3_OUT_H 15
#define PAD3_OUT_W 15

#define CONV3_IN_H PAD3_OUT_H
#define CONV3_IN_W PAD3_OUT_W
#define CONV3_IN_CH 192
#define CONV3_OUT_CH 384
#define CONV3_KERNEL 3
#define CONV3_STRIDE 1
#define CONV3_OUT_H 13
#define CONV3_OUT_W 13

#define RELU3_SIZE (CONV3_OUT_H * CONV3_OUT_W * CONV3_OUT_CH)

// ===== Pad4 / Conv4 / ReLU4 =====
#define PAD4_IN_H CONV3_OUT_H
#define PAD4_IN_W CONV3_OUT_W
#define PAD4_CH CONV3_OUT_CH
#define PAD4_PAD 1
#define PAD4_OUT_H 15
#define PAD4_OUT_W 15

#define CONV4_IN_H PAD4_OUT_H
#define CONV4_IN_W PAD4_OUT_W
#define CONV4_IN_CH 384
#define CONV4_OUT_CH 256
#define CONV4_KERNEL 3
#define CONV4_STRIDE 1
#define CONV4_OUT_H 13
#define CONV4_OUT_W 13

#define RELU4_SIZE (CONV4_OUT_H * CONV4_OUT_W * CONV4_OUT_CH)

// ===== Pad5 / Conv5 / ReLU5 / Pool5 =====
#define PAD5_IN_H CONV4_OUT_H
#define PAD5_IN_W CONV4_OUT_W
#define PAD5_CH CONV4_OUT_CH
#define PAD5_PAD 1
#define PAD5_OUT_H 15
#define PAD5_OUT_W 15

#define CONV5_IN_H PAD5_OUT_H
#define CONV5_IN_W PAD5_OUT_W
#define CONV5_IN_CH 256
#define CONV5_OUT_CH 256
#define CONV5_KERNEL 3
#define CONV5_STRIDE 1
#define CONV5_OUT_H 13
#define CONV5_OUT_W 13

#define RELU5_SIZE (CONV5_OUT_H * CONV5_OUT_W * CONV5_OUT_CH)

#define POOL5_IN_H CONV5_OUT_H
#define POOL5_IN_W CONV5_OUT_W
#define POOL5_CH CONV5_OUT_CH
#define POOL5_KERNEL 3
#define POOL5_STRIDE 2
#define POOL5_OUT_H 6
#define POOL5_OUT_W 6

// ===== FC6 / ReLU6 =====
#define FC6_IN_SIZE (POOL5_OUT_H * POOL5_OUT_W * POOL5_CH) // 6*6*256 = 9216
#define FC6_OUT_SIZE 4096
#define RELU6_SIZE FC6_OUT_SIZE

// ===== FC7 / ReLU7 =====
#define FC7_IN_SIZE FC6_OUT_SIZE
#define FC7_OUT_SIZE 4096
#define RELU7_SIZE FC7_OUT_SIZE

// ===== FC8 =====
#define FC8_IN_SIZE FC7_OUT_SIZE
#define FC8_OUT_SIZE 1000

// ===== Softmax =====
#define SOFTMAX_SIZE FC8_OUT_SIZE

// 硬體分接器，用在傳遞F8訊號給 Pattern 和 Softmax，避免死結
SC_MODULE(Splitter)
{
    sc_in_clk clk;
    sc_in<bool> rst;
    sc_fifo_in<double> in_data;
    sc_fifo_out<double> out_softmax;
    sc_fifo_out<double> out_pattern;

    void run()
    {
        wait();
        while (true)
        {
            double val = in_data.read();
            out_softmax.write(val);
            out_pattern.write(val);
            wait();
        }
    }

    SC_CTOR(Splitter)
    {
        SC_THREAD(run);
        sensitive << clk.pos();
        async_reset_signal_is(rst, true);
    }
};

int sc_main(int argc, char *argv[])
{
    if (argc < 2)
    {
        cout << "Usage: ./run dog.txt" << endl;
        return 1;
    }

    string img_name = argv[1];

    sc_clock clk("clk", 1, SC_NS);
    sc_signal<bool> reset;

    // ==========================================
    // 宣告 FIFO Channels
    // 預設深度加大至 1024，確保神經網路水流暢通
    // ==========================================
    sc_fifo<double> fifo_img_in(1024);
    sc_fifo<double> fifo_zp_out(1024);

    sc_fifo<double> fifo_conv1_out(1024);
    sc_fifo<double> fifo_relu1_out(1024);
    sc_fifo<double> fifo_pool1_out(1024);

    sc_fifo<double> fifo_pad2_out(1024);
    sc_fifo<double> fifo_conv2_out(1024);
    sc_fifo<double> fifo_relu2_out(1024);
    sc_fifo<double> fifo_pool2_out(1024);

    sc_fifo<double> fifo_pad3_out(1024);
    sc_fifo<double> fifo_conv3_out(1024);
    sc_fifo<double> fifo_relu3_out(1024);

    sc_fifo<double> fifo_pad4_out(1024);
    sc_fifo<double> fifo_conv4_out(1024);
    sc_fifo<double> fifo_relu4_out(1024);

    sc_fifo<double> fifo_pad5_out(1024);
    sc_fifo<double> fifo_conv5_out(1024);
    sc_fifo<double> fifo_relu5_out(1024);
    sc_fifo<double> fifo_pool5_out(1024);

    sc_fifo<double> fifo_fc6_out(1024);
    sc_fifo<double> fifo_relu6_out(1024);

    sc_fifo<double> fifo_fc7_out(1024);
    sc_fifo<double> fifo_relu7_out(1024);

    sc_fifo<double> fifo_fc8_out(1024);
    sc_fifo<double> fifo_softmax_out(1024);

    sc_fifo<double> fifo_linear_out(1024); // 硬體分接器
    sc_fifo<double> fifo_fc8_out_split(1024);

    // ==========================================
    // 這裡宣告一些 signal 來追蹤每層的完成狀態
    // ==========================================
    sc_signal<bool> zp_done, conv1_done, relu1_done, pool1_done;
    sc_signal<bool> pad2_done, conv2_done, relu2_done, pool2_done;
    sc_signal<bool> pad3_done, conv3_done, relu3_done;
    sc_signal<bool> pad4_done, conv4_done, relu4_done;
    sc_signal<bool> pad5_done, conv5_done, relu5_done, pool5_done;
    sc_signal<bool> fc6_done, relu6_done_sig; // 改名避免跟 relu6 模組撞名
    sc_signal<bool> fc7_done, relu7_done_sig;
    sc_signal<bool> fc8_done, softmax_done;

    // ==========================================
    // 實例化模組並綁定 FIFO
    // ==========================================

    // --- Input & Pattern ---
    Pattern pattern("pattern", img_name);
    pattern.clock(clk);
    pattern.rst(reset);
    pattern.img_out(fifo_img_in);         // Pattern 吐出圖片
    pattern.softmax_in(fifo_softmax_out); // Pattern 接收最終機率
    pattern.linear_in(fifo_linear_out);   // Pattern 接收 FC8 原生數值

    // --- ZeroPadding ---
    ZeroPadding zp("zp");
    zp.clk(clk);
    zp.rst(reset);
    zp.out_done(zp_done);
    zp.img_in(fifo_img_in);
    zp.img_out(fifo_zp_out);

    // --- Layer 1 ---
    ConvLayer conv1("conv1", CONV1_IN_H, CONV1_IN_W, CONV1_IN_CH, CONV1_OUT_CH, CONV1_KERNEL, CONV1_STRIDE, "../../00_TESTBED/data/conv1_weight.txt", "../../00_TESTBED/data/conv1_bias.txt");
    conv1.clk(clk);
    conv1.rst(reset);
    conv1.out_done(conv1_done);
    conv1.img_in(fifo_zp_out);
    conv1.img_out(fifo_conv1_out);

    ReLU relu1("relu1", RELU1_SIZE);
    relu1.clk(clk);
    relu1.rst(reset);
    relu1.out_done(relu1_done);
    relu1.img_in(fifo_conv1_out);
    relu1.img_out(fifo_relu1_out);

    MaxPooling pool1("pool1", POOL1_IN_H, POOL1_IN_W, POOL1_CH, POOL1_KERNEL, POOL1_STRIDE);
    pool1.clk(clk);
    pool1.rst(reset);
    pool1.out_done(pool1_done);
    pool1.img_in(fifo_relu1_out);
    pool1.img_out(fifo_pool1_out);

    // --- Layer 2 ---
    PaddingLayer pad2("pad2", PAD2_IN_H, PAD2_IN_W, PAD2_CH, PAD2_PAD);
    pad2.clk(clk);
    pad2.rst(reset);
    pad2.out_done(pad2_done);
    pad2.img_in(fifo_pool1_out);
    pad2.img_out(fifo_pad2_out);

    ConvLayer conv2("conv2", CONV2_IN_H, CONV2_IN_W, CONV2_IN_CH, CONV2_OUT_CH, CONV2_KERNEL, CONV2_STRIDE, "../../00_TESTBED/data/conv2_weight.txt", "../../00_TESTBED/data/conv2_bias.txt");
    conv2.clk(clk);
    conv2.rst(reset);
    conv2.out_done(conv2_done);
    conv2.img_in(fifo_pad2_out);
    conv2.img_out(fifo_conv2_out);

    ReLU relu2("relu2", RELU2_SIZE);
    relu2.clk(clk);
    relu2.rst(reset);
    relu2.out_done(relu2_done);
    relu2.img_in(fifo_conv2_out);
    relu2.img_out(fifo_relu2_out);

    MaxPooling pool2("pool2", POOL2_IN_H, POOL2_IN_W, POOL2_CH, POOL2_KERNEL, POOL2_STRIDE);
    pool2.clk(clk);
    pool2.rst(reset);
    pool2.out_done(pool2_done);
    pool2.img_in(fifo_relu2_out);
    pool2.img_out(fifo_pool2_out);

    // --- Layer 3 ---
    PaddingLayer pad3("pad3", PAD3_IN_H, PAD3_IN_W, PAD3_CH, PAD3_PAD);
    pad3.clk(clk);
    pad3.rst(reset);
    pad3.out_done(pad3_done);
    pad3.img_in(fifo_pool2_out);
    pad3.img_out(fifo_pad3_out);

    ConvLayer conv3("conv3", CONV3_IN_H, CONV3_IN_W, CONV3_IN_CH, CONV3_OUT_CH, CONV3_KERNEL, CONV3_STRIDE, "../../00_TESTBED/data/conv3_weight.txt", "../../00_TESTBED/data/conv3_bias.txt");
    conv3.clk(clk);
    conv3.rst(reset);
    conv3.out_done(conv3_done);
    conv3.img_in(fifo_pad3_out);
    conv3.img_out(fifo_conv3_out);

    ReLU relu3("relu3", RELU3_SIZE);
    relu3.clk(clk);
    relu3.rst(reset);
    relu3.out_done(relu3_done);
    relu3.img_in(fifo_conv3_out);
    relu3.img_out(fifo_relu3_out);

    // --- Layer 4 ---
    PaddingLayer pad4("pad4", PAD4_IN_H, PAD4_IN_W, PAD4_CH, PAD4_PAD);
    pad4.clk(clk);
    pad4.rst(reset);
    pad4.out_done(pad4_done);
    pad4.img_in(fifo_relu3_out);
    pad4.img_out(fifo_pad4_out);

    ConvLayer conv4("conv4", CONV4_IN_H, CONV4_IN_W, CONV4_IN_CH, CONV4_OUT_CH, CONV4_KERNEL, CONV4_STRIDE, "../../00_TESTBED/data/conv4_weight.txt", "../../00_TESTBED/data/conv4_bias.txt");
    conv4.clk(clk);
    conv4.rst(reset);
    conv4.out_done(conv4_done);
    conv4.img_in(fifo_pad4_out);
    conv4.img_out(fifo_conv4_out);

    ReLU relu4("relu4", RELU4_SIZE);
    relu4.clk(clk);
    relu4.rst(reset);
    relu4.out_done(relu4_done);
    relu4.img_in(fifo_conv4_out);
    relu4.img_out(fifo_relu4_out);

    // --- Layer 5 ---
    PaddingLayer pad5("pad5", PAD5_IN_H, PAD5_IN_W, PAD5_CH, PAD5_PAD);
    pad5.clk(clk);
    pad5.rst(reset);
    pad5.out_done(pad5_done);
    pad5.img_in(fifo_relu4_out);
    pad5.img_out(fifo_pad5_out);

    ConvLayer conv5("conv5", CONV5_IN_H, CONV5_IN_W, CONV5_IN_CH, CONV5_OUT_CH, CONV5_KERNEL, CONV5_STRIDE, "../../00_TESTBED/data/conv5_weight.txt", "../../00_TESTBED/data/conv5_bias.txt");
    conv5.clk(clk);
    conv5.rst(reset);
    conv5.out_done(conv5_done);
    conv5.img_in(fifo_pad5_out);
    conv5.img_out(fifo_conv5_out);

    ReLU relu5("relu5", RELU5_SIZE);
    relu5.clk(clk);
    relu5.rst(reset);
    relu5.out_done(relu5_done);
    relu5.img_in(fifo_conv5_out);
    relu5.img_out(fifo_relu5_out);

    MaxPooling pool5("pool5", POOL5_IN_H, POOL5_IN_W, POOL5_CH, POOL5_KERNEL, POOL5_STRIDE);
    pool5.clk(clk);
    pool5.rst(reset);
    pool5.out_done(pool5_done);
    pool5.img_in(fifo_relu5_out);
    pool5.img_out(fifo_pool5_out);

    // --- FC6 ---
    FullyConnected fc6("fc6", FC6_IN_SIZE, FC6_OUT_SIZE, "../../00_TESTBED/data/fc6_weight.txt", "../../00_TESTBED/data/fc6_bias.txt");
    fc6.clk(clk);
    fc6.rst(reset);
    fc6.out_done(fc6_done);
    fc6.img_in(fifo_pool5_out);
    fc6.img_out(fifo_fc6_out);

    ReLU relu6("relu6", RELU6_SIZE);
    relu6.clk(clk);
    relu6.rst(reset);
    relu6.out_done(relu6_done_sig);
    relu6.img_in(fifo_fc6_out);
    relu6.img_out(fifo_relu6_out);

    // --- FC7 ---
    FullyConnected fc7("fc7", FC7_IN_SIZE, FC7_OUT_SIZE, "../../00_TESTBED/data/fc7_weight.txt", "../../00_TESTBED/data/fc7_bias.txt");
    fc7.clk(clk);
    fc7.rst(reset);
    fc7.out_done(fc7_done);
    fc7.img_in(fifo_relu6_out);
    fc7.img_out(fifo_fc7_out);

    ReLU relu7("relu7", RELU7_SIZE);
    relu7.clk(clk);
    relu7.rst(reset);
    relu7.out_done(relu7_done_sig);
    relu7.img_in(fifo_fc7_out);
    relu7.img_out(fifo_relu7_out);

    // --- FC8 & Softmax ---
    FullyConnected fc8("fc8", FC8_IN_SIZE, FC8_OUT_SIZE, "../../00_TESTBED/data/fc8_weight.txt", "../../00_TESTBED/data/fc8_bias.txt");
    fc8.clk(clk);
    fc8.rst(reset);
    fc8.out_done(fc8_done);
    fc8.img_in(fifo_relu7_out);
    fc8.img_out(fifo_fc8_out);

    Splitter splitter("splitter");
    splitter.clk(clk);
    splitter.rst(reset);
    splitter.in_data(fifo_fc8_out); // 吃 FC8 吐出來的東西
    splitter.out_softmax(fifo_fc8_out_split);
    splitter.out_pattern(fifo_linear_out); // 抄一份給 Pattern

    Softmax softmax("softmax", SOFTMAX_SIZE);
    softmax.clk(clk);
    softmax.rst(reset);
    softmax.out_done(softmax_done);
    softmax.img_in(fifo_fc8_out_split);
    softmax.img_out(fifo_softmax_out);

    sc_start();
    return 0;

    // ===== Read input image (leave it to pattern.h) =====
    /*
    ifstream inputFile(("data/" + img_name).c_str());
    if (!inputFile.is_open())
    {
        cout << "Cannot open file: data/" << img_name << endl;
        return 1;
    }

    double val;
    int cnt = 0;
    while (inputFile >> val)
    {
        if (cnt >= IN_H * IN_W * CHANNEL)
        {
            cout << "Input data too large." << endl;
            return 1;
        }
        img_in_sig[cnt].write(val);
        cnt++;
    }
    inputFile.close();


    if (cnt != IN_H * IN_W * CHANNEL)
    {
        cout << "Input data size mismatch. Read " << cnt
             << " elements, expected " << IN_H * IN_W * CHANNEL << endl;
        return 1;
    }
    */

    // ===== helper index =====
    auto zp_idx = [](int c, int h, int w)
    {
        return c * (OUT_H * OUT_W) + h * OUT_W + w;
    };

    auto conv1_idx = [](int c, int h, int w)
    {
        return c * (CONV1_OUT_H * CONV1_OUT_W) + h * CONV1_OUT_W + w;
    };

    auto pool1_idx = [](int c, int h, int w)
    {
        return c * (POOL1_OUT_H * POOL1_OUT_W) + h * POOL1_OUT_W + w;
    };

    // =========================================================
    // Simulation sequence (till pool1)
    // =========================================================

    // cycle 1: reset
    /*
    reset.write(1);
    zp_in_valid.write(0);
    sc_start(1, SC_NS);

    // cycle 2: send image into ZeroPadding
    reset.write(0);
    zp_in_valid.write(1);
    sc_start(1, SC_NS);

    cout << "After ZeroPadding cycle:" << endl;
    cout << "zp_out_valid    = " << zp_out_valid.read() << endl;
    cout << "conv1_out_valid = " << conv1_out_valid.read() << endl;
    cout << "relu1_out_valid = " << relu1_out_valid.read() << endl;
    cout << "pool1_out_valid = " << pool1_out_valid.read() << endl;

    // cycle 3: ZeroPadding output becomes Conv1 input
    zp_in_valid.write(0);
    sc_start(1, SC_NS);

    cout << "After Conv1 cycle:" << endl;
    cout << "zp_out_valid    = " << zp_out_valid.read() << endl;
    cout << "conv1_out_valid = " << conv1_out_valid.read() << endl;
    cout << "relu1_out_valid = " << relu1_out_valid.read() << endl;
    cout << "pool1_out_valid = " << pool1_out_valid.read() << endl;

    // cycle 4: Conv1 -> ReLU1
    sc_start(1, SC_NS);

    cout << "After ReLU1 cycle:" << endl;
    cout << "zp_out_valid    = " << zp_out_valid.read() << endl;
    cout << "conv1_out_valid = " << conv1_out_valid.read() << endl;
    cout << "relu1_out_valid = " << relu1_out_valid.read() << endl;
    cout << "pool1_out_valid = " << pool1_out_valid.read() << endl;

    // cycle 5: ReLU1 -> Pool1
    sc_start(1, SC_NS);

    cout << "After Pool1 cycle:" << endl;
    cout << "zp_out_valid    = " << zp_out_valid.read() << endl;
    cout << "conv1_out_valid = " << conv1_out_valid.read() << endl;
    cout << "relu1_out_valid = " << relu1_out_valid.read() << endl;
    cout << "pool1_out_valid = " << pool1_out_valid.read() << endl;
    */

    // simulation

    /*
    while (softmax_out_valid.read() == 0 && cycle < MAX_CYCLE)
    {
        sc_start(1, SC_NS);
        cycle++;

        cout << "[Cycle " << cycle << "] "
             << "pool5=" << pool5_out_valid.read() << " "
             << "fc6=" << fc6_out_valid.read() << " "
             << "relu6=" << relu6_out_valid.read() << " "
             << "fc7=" << fc7_out_valid.read() << " "
             << "relu7=" << relu7_out_valid.read() << " "
             << "fc8=" << fc8_out_valid.read() << " "
             << "softmax=" << softmax_out_valid.read()
             << endl;
    }

    if (softmax_out_valid.read())
    {
        cout << "\nSoftmax output is ready at cycle " << cycle << endl;
    }
    else
    {
        cout << "\nTimeout: softmax_out_valid never became 1." << endl;
    }

    // ===== Check ZeroPadding result =====
    cout << "\n=== ZeroPadding check: channel 0, top-left 5x5 ===" << endl;
    for (int h = 0; h < 5; h++)
    {
        for (int w = 0; w < 5; w++)
        {
            cout << zp_to_conv1_sig[zp_idx(0, h, w)].read() << "\t";
        }
        cout << endl;
    }

    // ===== Check Conv1 result =====
    if (conv1_out_valid.read())
    {
        cout << "\nConv1 output is valid." << endl;
    }
    else
    {
        cout << "\nConv1 output is NOT valid." << endl;
    }

    cout << "\n=== Conv1 check: channel 0, top-left 5x5 ===" << endl;
    for (int h = 0; h < 5; h++)
    {
        for (int w = 0; w < 5; w++)
        {
            cout << conv1_out_sig[conv1_idx(0, h, w)].read() << "\t";
        }
        cout << endl;
    }

    if (softmax_out_valid.read())
    {
        cout << "\n=== Softmax first 10 values ===" << endl;
        for (int i = 0; i < 10; i++)
        {
            cout << "softmax[" << i << "] = " << softmax_out_sig[i].read() << endl;
        }
    }


    // ===== Print a few Conv1 outputs =====
    cout << "\n=== Conv1 key values ===" << endl;
    cout << "conv1[0][0][0] = " << conv1_out_sig[conv1_idx(0, 0, 0)].read() << endl;
    cout << "conv1[0][0][1] = " << conv1_out_sig[conv1_idx(0, 0, 1)].read() << endl;
    cout << "conv1[0][1][0] = " << conv1_out_sig[conv1_idx(0, 1, 0)].read() << endl;
    cout << "conv1[0][1][1] = " << conv1_out_sig[conv1_idx(0, 1, 1)].read() << endl;

    if (pool1_out_valid.read())
    {
        cout << "\nPool1 output is valid." << endl;
    }
    else
    {
        cout << "\nPool1 output is NOT valid." << endl;
    }

    if (relu1_out_valid.read())
    {
        cout << "\nReLU1 output is valid." << endl;
    }
    else
    {
        cout << "\nReLU1 output is NOT valid." << endl;
    }

    cout << "\n=== ReLU1 check: channel 0, top-left 5x5 ===" << endl;
    for (int h = 0; h < 5; h++)
    {
        for (int w = 0; w < 5; w++)
        {
            cout << relu1_out_sig[conv1_idx(0, h, w)].read() << "\t";
        }
        cout << endl;
    }

    cout << "\n=== Pool1 check: channel 0, top-left 5x5 ===" << endl;
    for (int h = 0; h < 5; h++)
    {
        for (int w = 0; w < 5; w++)
        {
            cout << pool1_out_sig[pool1_idx(0, h, w)].read() << "\t";
        }
        cout << endl;
    }

    cout << "\n=== Pool1 key values ===" << endl;
    cout << "pool1[0][0][0] = " << pool1_out_sig[pool1_idx(0, 0, 0)].read() << endl;
    cout << "pool1[0][0][1] = " << pool1_out_sig[pool1_idx(0, 0, 1)].read() << endl;
    cout << "pool1[0][1][0] = " << pool1_out_sig[pool1_idx(0, 1, 0)].read() << endl;
    cout << "pool1[0][1][1] = " << pool1_out_sig[pool1_idx(0, 1, 1)].read() << endl;
    */

    return 0;
}

// ===============================================================
//------------------------OUTPUT EXAMPLE--------------------------
// ===============================================================
/*
    cout << fixed << setprecision(2);
    cout << "Top 100 classes:" << endl;
    cout << "=================================================" << endl;
    cout << right << setw(5) << "idx"
         << " | " << setw(8) << "val"
         << " | " << setw(11) << "possibility"
         << " | " << "class name" << endl;
    cout << "-------------------------------------------------" << endl;

    for (int i = 0; i < 100; i++) {
        file.clear(); // Clear any potential error flags
        file.seekg(0, ios::beg); // Seek back to the beginning of the file
        int index = top_5_val[i].second;
        string line;
        for (int j = 0; j <= index; j++) {
            getline(file, line);
        }
        cout << right << setw(5) << index
             << " | " << setw(8) << top_5_val[i].first
             << " | " << setw(11) << (top_5_pos[i].first) // Assuming softmax outputs probabilities
             << " | " << line << endl;

    }
    cout << "=================================================" << endl;
*/
