#ifndef PE_LOCAL_SRAM_H
#define PE_LOCAL_SRAM_H

#include "systemc.h"
#include <vector>

using namespace std;

// PE-local SRAM behavior model.
// It stores the input activation tile, weight tile, bias tile, and exposes
// simple block load/read methods used by the PE datapath.
SC_MODULE(PeLocalSram)
{
    static const unsigned int DATA_WIDTH_BITS = 32;
    static const unsigned int READ_LATENCY_CYCLES = 1;
    static const unsigned int WRITE_LATENCY_CYCLES = 1;

    vector<float> input_buf;
    vector<float> weight_buf;
    vector<float> bias_buf;

    unsigned long long read_words;
    unsigned long long write_words;

    void reset_stats()
    {
        read_words = 0;
        write_words = 0;
    }

    void load_input(const vector<float> &data)
    {
        input_buf = data;
        write_words += data.size();
    }

    void load_weight(const vector<float> &data)
    {
        weight_buf = data;
        write_words += data.size();
    }

    void load_bias(const vector<float> &data)
    {
        bias_buf = data;
        write_words += data.size();
    }

    float read_input(unsigned int index)
    {
        read_words++;
        return input_buf[index];
    }

    float read_weight(unsigned int index)
    {
        read_words++;
        return weight_buf[index];
    }

    float read_bias(unsigned int index)
    {
        read_words++;
        return bias_buf[index];
    }

    unsigned int input_size() const
    {
        return (unsigned int)input_buf.size();
    }

    unsigned int weight_size() const
    {
        return (unsigned int)weight_buf.size();
    }

    unsigned int bias_size() const
    {
        return (unsigned int)bias_buf.size();
    }

    SC_CTOR(PeLocalSram)
    {
        reset_stats();
    }
};

#endif
