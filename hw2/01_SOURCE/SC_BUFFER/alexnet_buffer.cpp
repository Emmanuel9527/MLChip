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

// ==========================================
// 定義傳輸所需的 Data Sizes 巨集 (給 sc_vector 使用)
// ==========================================
#define IMG_IN_SIZE (224 * 224 * 3)                               // 150528
#define ZP_OUT_SIZE (CONV1_IN_H * CONV1_IN_W * CONV1_IN_CH)       // 154587
#define CONV1_OUT_SIZE (CONV1_OUT_H * CONV1_OUT_W * CONV1_OUT_CH) // 193600
#define POOL1_OUT_SIZE (POOL1_OUT_H * POOL1_OUT_W * POOL1_CH)     // 46656
#define PAD2_OUT_SIZE (PAD2_OUT_H * PAD2_OUT_W * PAD2_CH)         // 61504
#define CONV2_OUT_SIZE (CONV2_OUT_H * CONV2_OUT_W * CONV2_OUT_CH) // 139968
#define POOL2_OUT_SIZE (POOL2_OUT_H * POOL2_OUT_W * POOL2_CH)     // 32448
#define PAD3_OUT_SIZE (PAD3_OUT_H * PAD3_OUT_W * PAD3_CH)         // 43200
#define CONV3_OUT_SIZE (CONV3_OUT_H * CONV3_OUT_W * CONV3_OUT_CH) // 64896
#define PAD4_OUT_SIZE (PAD4_OUT_H * PAD4_OUT_W * PAD4_CH)         // 86400
#define CONV4_OUT_SIZE (CONV4_OUT_H * CONV4_OUT_W * CONV4_OUT_CH) // 43264
#define PAD5_OUT_SIZE (PAD5_OUT_H * PAD5_OUT_W * PAD5_CH)         // 57600
#define CONV5_OUT_SIZE (CONV5_OUT_H * CONV5_OUT_W * CONV5_OUT_CH) // 43264
#define POOL5_OUT_SIZE (POOL5_OUT_H * POOL5_OUT_W * POOL5_CH)     // 9216

int sc_main(int argc, char *argv[])
{
    // debug
    // cout << "\n[System] =========================================" << endl;
    // cout << "[System] 1. 進入 sc_main，準備開始建構 AlexNet 硬體架構..." << endl;

    if (argc < 2)
    {
        cout << "Usage: ./run dog.txt" << endl;
        return 1;
    }

    string img_name = argv[1];

    sc_clock clk("clk", 1, SC_NS);
    sc_signal<bool> reset;

    // ==========================================
    // 宣告 sc_vector<sc_buffer<double>> 陣列
    // ==========================================
    // debug
    // cout << "[System] 2. 開始宣告海量的 sc_vector (配置記憶體)..." << endl;

    // ==========================================
    // 宣告 sc_vector<sc_buffer<double>> 陣列 (全部改為 new 動態配置)
    // ==========================================
    // debug
    // cout << "[System] 2. 開始宣告海量的 sc_vector (配置記憶體在 Heap)..." << endl;

    sc_vector<sc_buffer<double>> *buf_img_in = new sc_vector<sc_buffer<double>>("buf_img_in", IMG_IN_SIZE);
    sc_vector<sc_buffer<double>> *buf_zp_out = new sc_vector<sc_buffer<double>>("buf_zp_out", ZP_OUT_SIZE);

    sc_vector<sc_buffer<double>> *buf_conv1_out = new sc_vector<sc_buffer<double>>("buf_conv1_out", CONV1_OUT_SIZE);
    sc_vector<sc_buffer<double>> *buf_relu1_out = new sc_vector<sc_buffer<double>>("buf_relu1_out", RELU1_SIZE);
    sc_vector<sc_buffer<double>> *buf_pool1_out = new sc_vector<sc_buffer<double>>("buf_pool1_out", POOL1_OUT_SIZE);

    sc_vector<sc_buffer<double>> *buf_pad2_out = new sc_vector<sc_buffer<double>>("buf_pad2_out", PAD2_OUT_SIZE);
    sc_vector<sc_buffer<double>> *buf_conv2_out = new sc_vector<sc_buffer<double>>("buf_conv2_out", CONV2_OUT_SIZE);
    sc_vector<sc_buffer<double>> *buf_relu2_out = new sc_vector<sc_buffer<double>>("buf_relu2_out", RELU2_SIZE);
    sc_vector<sc_buffer<double>> *buf_pool2_out = new sc_vector<sc_buffer<double>>("buf_pool2_out", POOL2_OUT_SIZE);

    sc_vector<sc_buffer<double>> *buf_pad3_out = new sc_vector<sc_buffer<double>>("buf_pad3_out", PAD3_OUT_SIZE);
    sc_vector<sc_buffer<double>> *buf_conv3_out = new sc_vector<sc_buffer<double>>("buf_conv3_out", CONV3_OUT_SIZE);
    sc_vector<sc_buffer<double>> *buf_relu3_out = new sc_vector<sc_buffer<double>>("buf_relu3_out", RELU3_SIZE);

    sc_vector<sc_buffer<double>> *buf_pad4_out = new sc_vector<sc_buffer<double>>("buf_pad4_out", PAD4_OUT_SIZE);
    sc_vector<sc_buffer<double>> *buf_conv4_out = new sc_vector<sc_buffer<double>>("buf_conv4_out", CONV4_OUT_SIZE);
    sc_vector<sc_buffer<double>> *buf_relu4_out = new sc_vector<sc_buffer<double>>("buf_relu4_out", RELU4_SIZE);

    sc_vector<sc_buffer<double>> *buf_pad5_out = new sc_vector<sc_buffer<double>>("buf_pad5_out", PAD5_OUT_SIZE);
    sc_vector<sc_buffer<double>> *buf_conv5_out = new sc_vector<sc_buffer<double>>("buf_conv5_out", CONV5_OUT_SIZE);
    sc_vector<sc_buffer<double>> *buf_relu5_out = new sc_vector<sc_buffer<double>>("buf_relu5_out", RELU5_SIZE);
    sc_vector<sc_buffer<double>> *buf_pool5_out = new sc_vector<sc_buffer<double>>("buf_pool5_out", POOL5_OUT_SIZE);

    sc_vector<sc_buffer<double>> *buf_fc6_out = new sc_vector<sc_buffer<double>>("buf_fc6_out", FC6_OUT_SIZE);
    sc_vector<sc_buffer<double>> *buf_relu6_out = new sc_vector<sc_buffer<double>>("buf_relu6_out", RELU6_SIZE);

    sc_vector<sc_buffer<double>> *buf_fc7_out = new sc_vector<sc_buffer<double>>("buf_fc7_out", FC7_OUT_SIZE);
    sc_vector<sc_buffer<double>> *buf_relu7_out = new sc_vector<sc_buffer<double>>("buf_relu7_out", RELU7_SIZE);

    sc_vector<sc_buffer<double>> *buf_fc8_out = new sc_vector<sc_buffer<double>>("buf_fc8_out", FC8_OUT_SIZE);
    sc_vector<sc_buffer<double>> *buf_softmax_out = new sc_vector<sc_buffer<double>>("buf_softmax_out", SOFTMAX_SIZE);
    // ==========================================
    // 追蹤每層的完成狀態 Signals
    // ==========================================
    sc_signal<bool> zp_done, conv1_done, relu1_done, pool1_done;
    sc_signal<bool> pad2_done, conv2_done, relu2_done, pool2_done;
    sc_signal<bool> pad3_done, conv3_done, relu3_done;
    sc_signal<bool> pad4_done, conv4_done, relu4_done;
    sc_signal<bool> pad5_done, conv5_done, relu5_done, pool5_done;
    sc_signal<bool> fc6_done, relu6_done_sig;
    sc_signal<bool> fc7_done, relu7_done_sig;
    sc_signal<bool> fc8_done, softmax_done;
    sc_signal<bool> pattern_done;

    // ==========================================
    // 實例化模組並透過迴圈綁定 sc_vector
    // ==========================================
    // debug
    // cout << "[System] 3. 開始實例化各個神經網路層 (注意：此時會開始讀取 Weight 檔案)..." << endl;

    // --- Input & Pattern ---
    Pattern *pattern = new Pattern("pattern", img_name);
    pattern->clock(clk);
    pattern->rst(reset);

    pattern->in_valid(pattern_done);
    pattern->out_valid(softmax_done);
    for (int i = 0; i < IMG_IN_SIZE; i++)
        pattern->img[i]((*buf_img_in)[i]);
    for (int i = 0; i < SOFTMAX_SIZE; i++)
        pattern->output_softmax[i]((*buf_softmax_out)[i]);
    for (int i = 0; i < 1000; i++)
        pattern->output_linear[i]((*buf_fc8_out)[i]);

    // --- ZeroPadding ---
    ZeroPadding *zp = new ZeroPadding("zp");
    zp->clk(clk);
    zp->rst(reset);
    zp->in_valid(pattern_done);
    zp->out_valid(zp_done);
    for (int i = 0; i < IMG_IN_SIZE; i++)
        zp->img_in[i]((*buf_img_in)[i]);
    for (int i = 0; i < ZP_OUT_SIZE; i++)
        zp->img_out[i]((*buf_zp_out)[i]);

    // --- Layer 1 ---
    ConvLayer *conv1 = new ConvLayer("conv1", CONV1_IN_H, CONV1_IN_W, CONV1_IN_CH, CONV1_OUT_CH, CONV1_KERNEL, CONV1_STRIDE, "../../00_TESTBED/data/conv1_weight.txt", "../../00_TESTBED/data/conv1_bias.txt");
    conv1->clk(clk);
    conv1->rst(reset);
    conv1->out_valid(conv1_done);
    conv1->in_valid(zp_done);
    for (int i = 0; i < ZP_OUT_SIZE; i++)
        conv1->img_in[i]((*buf_zp_out)[i]);
    for (int i = 0; i < CONV1_OUT_SIZE; i++)
        conv1->img_out[i]((*buf_conv1_out)[i]);

    ReLU *relu1 = new ReLU("relu1", RELU1_SIZE);
    relu1->clk(clk);
    relu1->rst(reset);
    relu1->in_valid(conv1_done);
    relu1->out_valid(relu1_done);
    for (int i = 0; i < CONV1_OUT_SIZE; i++)
        relu1->img_in[i]((*buf_conv1_out)[i]);
    for (int i = 0; i < RELU1_SIZE; i++)
        relu1->img_out[i]((*buf_relu1_out)[i]);

    MaxPooling *pool1 = new MaxPooling("pool1", POOL1_IN_H, POOL1_IN_W, POOL1_CH, POOL1_KERNEL, POOL1_STRIDE);
    pool1->clk(clk);
    pool1->rst(reset);
    pool1->in_valid(relu1_done);
    pool1->out_valid(pool1_done);
    for (int i = 0; i < RELU1_SIZE; i++)
        pool1->img_in[i]((*buf_relu1_out)[i]);
    for (int i = 0; i < POOL1_OUT_SIZE; i++)
        pool1->img_out[i]((*buf_pool1_out)[i]);

    // --- Layer 2 ---
    PaddingLayer *pad2 = new PaddingLayer("pad2", PAD2_IN_H, PAD2_IN_W, PAD2_CH, PAD2_PAD);
    pad2->clk(clk);
    pad2->rst(reset);
    pad2->in_valid(pool1_done);
    pad2->out_valid(pad2_done);
    for (int i = 0; i < POOL1_OUT_SIZE; i++)
        pad2->img_in[i]((*buf_pool1_out)[i]);
    for (int i = 0; i < PAD2_OUT_SIZE; i++)
        pad2->img_out[i]((*buf_pad2_out)[i]);

    ConvLayer *conv2 = new ConvLayer("conv2", CONV2_IN_H, CONV2_IN_W, CONV2_IN_CH, CONV2_OUT_CH, CONV2_KERNEL, CONV2_STRIDE, "../../00_TESTBED/data/conv2_weight.txt", "../../00_TESTBED/data/conv2_bias.txt");
    conv2->clk(clk);
    conv2->rst(reset);
    conv2->in_valid(pad2_done);
    conv2->out_valid(conv2_done);
    for (int i = 0; i < PAD2_OUT_SIZE; i++)
        conv2->img_in[i]((*buf_pad2_out)[i]);
    for (int i = 0; i < CONV2_OUT_SIZE; i++)
        conv2->img_out[i]((*buf_conv2_out)[i]);

    ReLU *relu2 = new ReLU("relu2", RELU2_SIZE);
    relu2->clk(clk);
    relu2->rst(reset);
    relu2->in_valid(conv2_done);
    relu2->out_valid(relu2_done);
    for (int i = 0; i < CONV2_OUT_SIZE; i++)
        relu2->img_in[i]((*buf_conv2_out)[i]);
    for (int i = 0; i < RELU2_SIZE; i++)
        relu2->img_out[i]((*buf_relu2_out)[i]);

    MaxPooling *pool2 = new MaxPooling("pool2", POOL2_IN_H, POOL2_IN_W, POOL2_CH, POOL2_KERNEL, POOL2_STRIDE);
    pool2->clk(clk);
    pool2->rst(reset);
    pool2->in_valid(relu2_done);
    pool2->out_valid(pool2_done);
    for (int i = 0; i < RELU2_SIZE; i++)
        pool2->img_in[i]((*buf_relu2_out)[i]);
    for (int i = 0; i < POOL2_OUT_SIZE; i++)
        pool2->img_out[i]((*buf_pool2_out)[i]);

    // --- Layer 3 ---
    PaddingLayer *pad3 = new PaddingLayer("pad3", PAD3_IN_H, PAD3_IN_W, PAD3_CH, PAD3_PAD);
    pad3->clk(clk);
    pad3->rst(reset);
    pad3->in_valid(pool2_done);
    pad3->out_valid(pad3_done);
    for (int i = 0; i < POOL2_OUT_SIZE; i++)
        pad3->img_in[i]((*buf_pool2_out)[i]);
    for (int i = 0; i < PAD3_OUT_SIZE; i++)
        pad3->img_out[i]((*buf_pad3_out)[i]);

    ConvLayer *conv3 = new ConvLayer("conv3", CONV3_IN_H, CONV3_IN_W, CONV3_IN_CH, CONV3_OUT_CH, CONV3_KERNEL, CONV3_STRIDE, "../../00_TESTBED/data/conv3_weight.txt", "../../00_TESTBED/data/conv3_bias.txt");
    conv3->clk(clk);
    conv3->rst(reset);
    conv3->in_valid(pad3_done);
    conv3->out_valid(conv3_done);
    for (int i = 0; i < PAD3_OUT_SIZE; i++)
        conv3->img_in[i]((*buf_pad3_out)[i]);
    for (int i = 0; i < CONV3_OUT_SIZE; i++)
        conv3->img_out[i]((*buf_conv3_out)[i]);

    ReLU *relu3 = new ReLU("relu3", RELU3_SIZE);
    relu3->clk(clk);
    relu3->rst(reset);
    relu3->in_valid(conv3_done);
    relu3->out_valid(relu3_done);
    for (int i = 0; i < CONV3_OUT_SIZE; i++)
        relu3->img_in[i]((*buf_conv3_out)[i]);
    for (int i = 0; i < RELU3_SIZE; i++)
        relu3->img_out[i]((*buf_relu3_out)[i]);

    // --- Layer 4 ---
    PaddingLayer *pad4 = new PaddingLayer("pad4", PAD4_IN_H, PAD4_IN_W, PAD4_CH, PAD4_PAD);
    pad4->clk(clk);
    pad4->rst(reset);
    pad4->in_valid(relu3_done);
    pad4->out_valid(pad4_done);
    for (int i = 0; i < RELU3_SIZE; i++)
        pad4->img_in[i]((*buf_relu3_out)[i]);
    for (int i = 0; i < PAD4_OUT_SIZE; i++)
        pad4->img_out[i]((*buf_pad4_out)[i]);

    ConvLayer *conv4 = new ConvLayer("conv4", CONV4_IN_H, CONV4_IN_W, CONV4_IN_CH, CONV4_OUT_CH, CONV4_KERNEL, CONV4_STRIDE, "../../00_TESTBED/data/conv4_weight.txt", "../../00_TESTBED/data/conv4_bias.txt");
    conv4->clk(clk);
    conv4->rst(reset);
    conv4->in_valid(pad4_done);
    conv4->out_valid(conv4_done);
    for (int i = 0; i < PAD4_OUT_SIZE; i++)
        conv4->img_in[i]((*buf_pad4_out)[i]);
    for (int i = 0; i < CONV4_OUT_SIZE; i++)
        conv4->img_out[i]((*buf_conv4_out)[i]);

    ReLU *relu4 = new ReLU("relu4", RELU4_SIZE);
    relu4->clk(clk);
    relu4->rst(reset);
    relu4->in_valid(conv4_done);
    relu4->out_valid(relu4_done);
    for (int i = 0; i < CONV4_OUT_SIZE; i++)
        relu4->img_in[i]((*buf_conv4_out)[i]);
    for (int i = 0; i < RELU4_SIZE; i++)
        relu4->img_out[i]((*buf_relu4_out)[i]);

    // --- Layer 5 ---
    PaddingLayer *pad5 = new PaddingLayer("pad5", PAD5_IN_H, PAD5_IN_W, PAD5_CH, PAD5_PAD);
    pad5->clk(clk);
    pad5->rst(reset);
    pad5->in_valid(relu4_done);
    pad5->out_valid(pad5_done);
    for (int i = 0; i < RELU4_SIZE; i++)
        pad5->img_in[i]((*buf_relu4_out)[i]);
    for (int i = 0; i < PAD5_OUT_SIZE; i++)
        pad5->img_out[i]((*buf_pad5_out)[i]);

    ConvLayer *conv5 = new ConvLayer("conv5", CONV5_IN_H, CONV5_IN_W, CONV5_IN_CH, CONV5_OUT_CH, CONV5_KERNEL, CONV5_STRIDE, "../../00_TESTBED/data/conv5_weight.txt", "../../00_TESTBED/data/conv5_bias.txt");
    conv5->clk(clk);
    conv5->rst(reset);
    conv5->in_valid(pad5_done);
    conv5->out_valid(conv5_done);
    for (int i = 0; i < PAD5_OUT_SIZE; i++)
        conv5->img_in[i]((*buf_pad5_out)[i]);
    for (int i = 0; i < CONV5_OUT_SIZE; i++)
        conv5->img_out[i]((*buf_conv5_out)[i]);

    ReLU *relu5 = new ReLU("relu5", RELU5_SIZE);
    relu5->clk(clk);
    relu5->rst(reset);
    relu5->in_valid(conv5_done);
    relu5->out_valid(relu5_done);
    for (int i = 0; i < CONV5_OUT_SIZE; i++)
        relu5->img_in[i]((*buf_conv5_out)[i]);
    for (int i = 0; i < RELU5_SIZE; i++)
        relu5->img_out[i]((*buf_relu5_out)[i]);

    MaxPooling *pool5 = new MaxPooling("pool5", POOL5_IN_H, POOL5_IN_W, POOL5_CH, POOL5_KERNEL, POOL5_STRIDE);
    pool5->clk(clk);
    pool5->rst(reset);
    pool5->in_valid(relu5_done);
    pool5->out_valid(pool5_done);
    for (int i = 0; i < RELU5_SIZE; i++)
        pool5->img_in[i]((*buf_relu5_out)[i]);
    for (int i = 0; i < POOL5_OUT_SIZE; i++)
        pool5->img_out[i]((*buf_pool5_out)[i]);

    // debug
    // cout << "[System]   -> 準備實例化 FC6 (開始讀取 3700 萬個權重)..." << endl;

    // --- FC6 ---
    FullyConnected *fc6 = new FullyConnected("fc6", FC6_IN_SIZE, FC6_OUT_SIZE, "../../00_TESTBED/data/fc6_weight.txt", "../../00_TESTBED/data/fc6_bias.txt");
    fc6->clk(clk);
    fc6->rst(reset);
    fc6->in_valid(pool5_done);
    fc6->out_valid(fc6_done);
    for (int i = 0; i < POOL5_OUT_SIZE; i++)
        fc6->img_in[i]((*buf_pool5_out)[i]);
    for (int i = 0; i < FC6_OUT_SIZE; i++)
        fc6->img_out[i]((*buf_fc6_out)[i]);

    // debug
    // cout << "[System]   -> FC6 實例化完成！" << endl;

    ReLU *relu6 = new ReLU("relu6", RELU6_SIZE);
    relu6->clk(clk);
    relu6->rst(reset);
    relu6->in_valid(fc6_done);
    relu6->out_valid(relu6_done_sig);
    for (int i = 0; i < FC6_OUT_SIZE; i++)
        relu6->img_in[i]((*buf_fc6_out)[i]);
    for (int i = 0; i < RELU6_SIZE; i++)
        relu6->img_out[i]((*buf_relu6_out)[i]);

    // --- FC7 ---
    FullyConnected *fc7 = new FullyConnected("fc7", FC7_IN_SIZE, FC7_OUT_SIZE, "../../00_TESTBED/data/fc7_weight.txt", "../../00_TESTBED/data/fc7_bias.txt");
    fc7->clk(clk);
    fc7->rst(reset);
    fc7->in_valid(relu6_done_sig);
    fc7->out_valid(fc7_done);
    for (int i = 0; i < RELU6_SIZE; i++)
        fc7->img_in[i]((*buf_relu6_out)[i]);
    for (int i = 0; i < FC7_OUT_SIZE; i++)
        fc7->img_out[i]((*buf_fc7_out)[i]);

    ReLU *relu7 = new ReLU("relu7", RELU7_SIZE);
    relu7->clk(clk);
    relu7->rst(reset);
    relu7->in_valid(fc7_done);
    relu7->out_valid(relu7_done_sig);
    for (int i = 0; i < FC7_OUT_SIZE; i++)
        relu7->img_in[i]((*buf_fc7_out)[i]);
    for (int i = 0; i < RELU7_SIZE; i++)
        relu7->img_out[i]((*buf_relu7_out)[i]);

    // --- FC8 & Softmax ---
    FullyConnected *fc8 = new FullyConnected("fc8", FC8_IN_SIZE, FC8_OUT_SIZE, "../../00_TESTBED/data/fc8_weight.txt", "../../00_TESTBED/data/fc8_bias.txt");
    fc8->clk(clk);
    fc8->rst(reset);
    fc8->in_valid(relu7_done_sig);
    fc8->out_valid(fc8_done);
    for (int i = 0; i < RELU7_SIZE; i++)
        fc8->img_in[i]((*buf_relu7_out)[i]);
    for (int i = 0; i < FC8_OUT_SIZE; i++)
        fc8->img_out[i]((*buf_fc8_out)[i]);

    Softmax *softmax = new Softmax("softmax", SOFTMAX_SIZE);
    softmax->clk(clk);
    softmax->rst(reset);
    softmax->in_valid(fc8_done);
    softmax->out_valid(softmax_done);
    for (int i = 0; i < FC8_OUT_SIZE; i++)
        softmax->img_in[i]((*buf_fc8_out)[i]);
    for (int i = 0; i < SOFTMAX_SIZE; i++)
        softmax->img_out[i]((*buf_softmax_out)[i]);

    // --- Input & Pattern ---
    /*Pattern pattern("pattern", img_name);
    pattern.clock(clk);
    pattern.rst(reset);

    pattern.in_valid(pattern_done);  // Pattern 準備好圖片後，輸出訊號給 ZeroPadding
    pattern.out_valid(softmax_done); // Pattern 睡醒的鬧鐘，接收來自 Softmax 的完成訊號
    for (int i = 0; i < IMG_IN_SIZE; i++)
        pattern.img[i](buf_img_in[i]);
    for (int i = 0; i < SOFTMAX_SIZE; i++)
        pattern.output_softmax[i](buf_softmax_out[i]);

    for (int i = 0; i < 1000; i++) // FC8 的數值輸出
        pattern.output_linear[i](buf_fc8_out[i]);

    // --- ZeroPadding ---
    ZeroPadding zp("zp");
    zp.clk(clk);
    zp.rst(reset);
    zp.in_valid(pattern_done);
    zp.out_valid(zp_done);
    for (int i = 0; i < IMG_IN_SIZE; i++)
        zp.img_in[i](buf_img_in[i]);
    for (int i = 0; i < ZP_OUT_SIZE; i++)
        zp.img_out[i](buf_zp_out[i]);

    // --- Layer 1 ---
    ConvLayer conv1("conv1", CONV1_IN_H, CONV1_IN_W, CONV1_IN_CH, CONV1_OUT_CH, CONV1_KERNEL, CONV1_STRIDE, "../../00_TESTBED/data/conv1_weight.txt", "../../00_TESTBED/data/conv1_bias.txt");
    conv1.clk(clk);
    conv1.rst(reset);
    conv1.out_valid(conv1_done);
    conv1.in_valid(zp_done);
    for (int i = 0; i < ZP_OUT_SIZE; i++)
        conv1.img_in[i](buf_zp_out[i]);
    for (int i = 0; i < CONV1_OUT_SIZE; i++)
        conv1.img_out[i](buf_conv1_out[i]);

    ReLU relu1("relu1", RELU1_SIZE);
    relu1.clk(clk);
    relu1.rst(reset);
    relu1.in_valid(conv1_done);
    relu1.out_valid(relu1_done);
    for (int i = 0; i < CONV1_OUT_SIZE; i++)
        relu1.img_in[i](buf_conv1_out[i]);
    for (int i = 0; i < RELU1_SIZE; i++)
        relu1.img_out[i](buf_relu1_out[i]);

    MaxPooling pool1("pool1", POOL1_IN_H, POOL1_IN_W, POOL1_CH, POOL1_KERNEL, POOL1_STRIDE);
    pool1.clk(clk);
    pool1.rst(reset);
    pool1.in_valid(relu1_done);
    pool1.out_valid(pool1_done);
    for (int i = 0; i < RELU1_SIZE; i++)
        pool1.img_in[i](buf_relu1_out[i]);
    for (int i = 0; i < POOL1_OUT_SIZE; i++)
        pool1.img_out[i](buf_pool1_out[i]);

    // --- Layer 2 ---
    PaddingLayer pad2("pad2", PAD2_IN_H, PAD2_IN_W, PAD2_CH, PAD2_PAD);
    pad2.clk(clk);
    pad2.rst(reset);
    pad2.in_valid(pool1_done);
    pad2.out_valid(pad2_done);
    for (int i = 0; i < POOL1_OUT_SIZE; i++)
        pad2.img_in[i](buf_pool1_out[i]);
    for (int i = 0; i < PAD2_OUT_SIZE; i++)
        pad2.img_out[i](buf_pad2_out[i]);

    ConvLayer conv2("conv2", CONV2_IN_H, CONV2_IN_W, CONV2_IN_CH, CONV2_OUT_CH, CONV2_KERNEL, CONV2_STRIDE, "../../00_TESTBED/data/conv2_weight.txt", "../../00_TESTBED/data/conv2_bias.txt");
    conv2.clk(clk);
    conv2.rst(reset);
    conv2.in_valid(pad2_done);
    conv2.out_valid(conv2_done);
    for (int i = 0; i < PAD2_OUT_SIZE; i++)
        conv2.img_in[i](buf_pad2_out[i]);
    for (int i = 0; i < CONV2_OUT_SIZE; i++)
        conv2.img_out[i](buf_conv2_out[i]);

    ReLU relu2("relu2", RELU2_SIZE);
    relu2.clk(clk);
    relu2.rst(reset);
    relu2.in_valid(conv2_done);
    relu2.out_valid(relu2_done);
    for (int i = 0; i < CONV2_OUT_SIZE; i++)
        relu2.img_in[i](buf_conv2_out[i]);
    for (int i = 0; i < RELU2_SIZE; i++)
        relu2.img_out[i](buf_relu2_out[i]);

    MaxPooling pool2("pool2", POOL2_IN_H, POOL2_IN_W, POOL2_CH, POOL2_KERNEL, POOL2_STRIDE);
    pool2.clk(clk);
    pool2.rst(reset);
    pool2.in_valid(relu2_done);
    pool2.out_valid(pool2_done);
    for (int i = 0; i < RELU2_SIZE; i++)
        pool2.img_in[i](buf_relu2_out[i]);
    for (int i = 0; i < POOL2_OUT_SIZE; i++)
        pool2.img_out[i](buf_pool2_out[i]);

    // --- Layer 3 ---
    PaddingLayer pad3("pad3", PAD3_IN_H, PAD3_IN_W, PAD3_CH, PAD3_PAD);
    pad3.clk(clk);
    pad3.rst(reset);
    pad3.in_valid(pool2_done);
    pad3.out_valid(pad3_done);
    for (int i = 0; i < POOL2_OUT_SIZE; i++)
        pad3.img_in[i](buf_pool2_out[i]);
    for (int i = 0; i < PAD3_OUT_SIZE; i++)
        pad3.img_out[i](buf_pad3_out[i]);

    ConvLayer conv3("conv3", CONV3_IN_H, CONV3_IN_W, CONV3_IN_CH, CONV3_OUT_CH, CONV3_KERNEL, CONV3_STRIDE, "../../00_TESTBED/data/conv3_weight.txt", "../../00_TESTBED/data/conv3_bias.txt");
    conv3.clk(clk);
    conv3.rst(reset);
    conv3.in_valid(pad3_done);
    conv3.out_valid(conv3_done);
    for (int i = 0; i < PAD3_OUT_SIZE; i++)
        conv3.img_in[i](buf_pad3_out[i]);
    for (int i = 0; i < CONV3_OUT_SIZE; i++)
        conv3.img_out[i](buf_conv3_out[i]);

    ReLU relu3("relu3", RELU3_SIZE);
    relu3.clk(clk);
    relu3.rst(reset);
    relu3.in_valid(conv3_done);
    relu3.out_valid(relu3_done);
    for (int i = 0; i < CONV3_OUT_SIZE; i++)
        relu3.img_in[i](buf_conv3_out[i]);
    for (int i = 0; i < RELU3_SIZE; i++)
        relu3.img_out[i](buf_relu3_out[i]);

    // --- Layer 4 ---
    PaddingLayer pad4("pad4", PAD4_IN_H, PAD4_IN_W, PAD4_CH, PAD4_PAD);
    pad4.clk(clk);
    pad4.rst(reset);
    pad4.in_valid(relu3_done);
    pad4.out_valid(pad4_done);
    for (int i = 0; i < RELU3_SIZE; i++)
        pad4.img_in[i](buf_relu3_out[i]);
    for (int i = 0; i < PAD4_OUT_SIZE; i++)
        pad4.img_out[i](buf_pad4_out[i]);

    ConvLayer conv4("conv4", CONV4_IN_H, CONV4_IN_W, CONV4_IN_CH, CONV4_OUT_CH, CONV4_KERNEL, CONV4_STRIDE, "../../00_TESTBED/data/conv4_weight.txt", "../../00_TESTBED/data/conv4_bias.txt");
    conv4.clk(clk);
    conv4.rst(reset);
    conv4.in_valid(pad4_done);
    conv4.out_valid(conv4_done);
    for (int i = 0; i < PAD4_OUT_SIZE; i++)
        conv4.img_in[i](buf_pad4_out[i]);
    for (int i = 0; i < CONV4_OUT_SIZE; i++)
        conv4.img_out[i](buf_conv4_out[i]);

    ReLU relu4("relu4", RELU4_SIZE);
    relu4.clk(clk);
    relu4.rst(reset);
    relu4.in_valid(conv4_done);
    relu4.out_valid(relu4_done);
    for (int i = 0; i < CONV4_OUT_SIZE; i++)
        relu4.img_in[i](buf_conv4_out[i]);
    for (int i = 0; i < RELU4_SIZE; i++)
        relu4.img_out[i](buf_relu4_out[i]);

    // --- Layer 5 ---
    PaddingLayer pad5("pad5", PAD5_IN_H, PAD5_IN_W, PAD5_CH, PAD5_PAD);
    pad5.clk(clk);
    pad5.rst(reset);
    pad5.in_valid(relu4_done);
    pad5.out_valid(pad5_done);
    for (int i = 0; i < RELU4_SIZE; i++)
        pad5.img_in[i](buf_relu4_out[i]);
    for (int i = 0; i < PAD5_OUT_SIZE; i++)
        pad5.img_out[i](buf_pad5_out[i]);

    ConvLayer conv5("conv5", CONV5_IN_H, CONV5_IN_W, CONV5_IN_CH, CONV5_OUT_CH, CONV5_KERNEL, CONV5_STRIDE, "../../00_TESTBED/data/conv5_weight.txt", "../../00_TESTBED/data/conv5_bias.txt");
    conv5.clk(clk);
    conv5.rst(reset);
    conv5.in_valid(pad5_done);
    conv5.out_valid(conv5_done);
    for (int i = 0; i < PAD5_OUT_SIZE; i++)
        conv5.img_in[i](buf_pad5_out[i]);
    for (int i = 0; i < CONV5_OUT_SIZE; i++)
        conv5.img_out[i](buf_conv5_out[i]);

    ReLU relu5("relu5", RELU5_SIZE);
    relu5.clk(clk);
    relu5.rst(reset);
    relu5.in_valid(conv5_done);
    relu5.out_valid(relu5_done);
    for (int i = 0; i < CONV5_OUT_SIZE; i++)
        relu5.img_in[i](buf_conv5_out[i]);
    for (int i = 0; i < RELU5_SIZE; i++)
        relu5.img_out[i](buf_relu5_out[i]);

    MaxPooling pool5("pool5", POOL5_IN_H, POOL5_IN_W, POOL5_CH, POOL5_KERNEL, POOL5_STRIDE);
    pool5.clk(clk);
    pool5.rst(reset);
    pool5.in_valid(relu5_done);
    pool5.out_valid(pool5_done);
    for (int i = 0; i < RELU5_SIZE; i++)
        pool5.img_in[i](buf_relu5_out[i]);
    for (int i = 0; i < POOL5_OUT_SIZE; i++)
        pool5.img_out[i](buf_pool5_out[i]);

    // debug
    cout << "[System]   -> 準備實例化 FC6 (開始讀取 3700 萬個權重)..." << endl;
    // --- FC6 ---
    FullyConnected fc6("fc6", FC6_IN_SIZE, FC6_OUT_SIZE, "../../00_TESTBED/data/fc6_weight.txt", "../../00_TESTBED/data/fc6_bias.txt");
    fc6.clk(clk);
    fc6.rst(reset);
    fc6.in_valid(pool5_done);
    fc6.out_valid(fc6_done);
    for (int i = 0; i < POOL5_OUT_SIZE; i++)
        fc6.img_in[i](buf_pool5_out[i]);
    for (int i = 0; i < FC6_OUT_SIZE; i++)
        fc6.img_out[i](buf_fc6_out[i]);
    // debug
    cout << "[System]   -> FC6 實例化完成！" << endl;

    ReLU relu6("relu6", RELU6_SIZE);
    relu6.clk(clk);
    relu6.rst(reset);
    relu6.in_valid(fc6_done);
    relu6.out_valid(relu6_done_sig);
    for (int i = 0; i < FC6_OUT_SIZE; i++)
        relu6.img_in[i](buf_fc6_out[i]);
    for (int i = 0; i < RELU6_SIZE; i++)
        relu6.img_out[i](buf_relu6_out[i]);

    // --- FC7 ---
    FullyConnected fc7("fc7", FC7_IN_SIZE, FC7_OUT_SIZE, "../../00_TESTBED/data/fc7_weight.txt", "../../00_TESTBED/data/fc7_bias.txt");
    fc7.clk(clk);
    fc7.rst(reset);
    fc7.in_valid(relu6_done_sig);
    fc7.out_valid(fc7_done);
    for (int i = 0; i < RELU6_SIZE; i++)
        fc7.img_in[i](buf_relu6_out[i]);
    for (int i = 0; i < FC7_OUT_SIZE; i++)
        fc7.img_out[i](buf_fc7_out[i]);

    ReLU relu7("relu7", RELU7_SIZE);
    relu7.clk(clk);
    relu7.rst(reset);
    relu7.in_valid(fc7_done);
    relu7.out_valid(relu7_done_sig);
    for (int i = 0; i < FC7_OUT_SIZE; i++)
        relu7.img_in[i](buf_fc7_out[i]);
    for (int i = 0; i < RELU7_SIZE; i++)
        relu7.img_out[i](buf_relu7_out[i]);

    // --- FC8 & Softmax ---
    FullyConnected fc8("fc8", FC8_IN_SIZE, FC8_OUT_SIZE, "../../00_TESTBED/data/fc8_weight.txt", "../../00_TESTBED/data/fc8_bias.txt");
    fc8.clk(clk);
    fc8.rst(reset);
    fc8.in_valid(relu7_done_sig);
    fc8.out_valid(fc8_done);
    for (int i = 0; i < RELU7_SIZE; i++)
        fc8.img_in[i](buf_relu7_out[i]);
    for (int i = 0; i < FC8_OUT_SIZE; i++)
        fc8.img_out[i](buf_fc8_out[i]);

    Softmax softmax("softmax", SOFTMAX_SIZE);
    softmax.clk(clk);
    softmax.rst(reset);
    softmax.in_valid(fc8_done);
    softmax.out_valid(softmax_done);
    for (int i = 0; i < FC8_OUT_SIZE; i++)
        softmax.img_in[i](buf_fc8_out[i]);
    for (int i = 0; i < SOFTMAX_SIZE; i++)
        softmax.img_out[i](buf_softmax_out[i]);
        */

    // 在 return 0; 之前加上這些來觸發 reset
    reset.write(1);
    sc_start(1, SC_NS);
    reset.write(0);

    // 然後才開始完整的模擬
    sc_start();

    return 0;
}