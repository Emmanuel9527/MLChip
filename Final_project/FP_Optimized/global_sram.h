#ifndef GLOBAL_SRAM_H
#define GLOBAL_SRAM_H

#include <iostream>
#include <string>
#include <vector>

using namespace std;

// Cycle-aware scratchpad SRAM model used by the optimized design.
// The model stores 32-bit float words, tracks traffic, and reports the
// latency that a controller scheduler should account for.
class GlobalSram
{
public:
    static const unsigned int WORD_BYTES = 4;

    unsigned int capacity_words;
    unsigned int data_width_bits;
    unsigned int read_latency_cycles;
    unsigned int write_latency_cycles;
    unsigned int bank_count;

    unsigned long long read_words;
    unsigned long long write_words;
    unsigned long long read_ops;
    unsigned long long write_ops;
    unsigned long long modeled_cycles;
    unsigned int peak_used_words;

    vector<float> storage;

    GlobalSram()
    {
        capacity_words = 0;
        data_width_bits = 32;
        read_latency_cycles = 1;
        write_latency_cycles = 1;
        bank_count = 1;
        reset_stats();
    }

    void configure(unsigned int words,
                   unsigned int width_bits,
                   unsigned int read_latency,
                   unsigned int write_latency,
                   unsigned int banks)
    {
        capacity_words = words;
        data_width_bits = width_bits;
        read_latency_cycles = read_latency;
        write_latency_cycles = write_latency;
        bank_count = banks;
        storage.assign(capacity_words, 0.0f);
        reset_stats();
    }

    void reset_stats()
    {
        read_words = 0;
        write_words = 0;
        read_ops = 0;
        write_ops = 0;
        modeled_cycles = 0;
        peak_used_words = 0;
    }

    bool can_hold(unsigned int addr, unsigned int words) const
    {
        return addr <= capacity_words && words <= capacity_words - addr;
    }

    unsigned int write_block(unsigned int addr, const vector<float> &data)
    {
        if (!can_hold(addr, (unsigned int)data.size()))
        {
            cout << "Global SRAM overflow on write_block." << endl;
            return 0;
        }
        for (size_t i = 0; i < data.size(); i++)
            storage[addr + (unsigned int)i] = data[i];
        write_words += data.size();
        write_ops++;
        if (addr + data.size() > peak_used_words)
            peak_used_words = addr + (unsigned int)data.size();
        unsigned int cycles = write_latency_cycles * (unsigned int)data.size();
        modeled_cycles += cycles;
        return cycles;
    }

    unsigned int read_block(unsigned int addr, unsigned int words, vector<float> &out)
    {
        if (!can_hold(addr, words))
        {
            cout << "Global SRAM overflow on read_block." << endl;
            out.clear();
            return 0;
        }
        out.assign(storage.begin() + addr, storage.begin() + addr + words);
        read_words += words;
        read_ops++;
        unsigned int cycles = read_latency_cycles * words;
        modeled_cycles += cycles;
        return cycles;
    }

    void print_metrics(const string &label) const
    {
        cout << "========== " << label << " SRAM Metrics ==========" << endl;
        cout << "Capacity words: " << capacity_words << endl;
        cout << "Capacity bytes: " << (unsigned long long)capacity_words * WORD_BYTES << endl;
        cout << "Data width bits: " << data_width_bits << endl;
        cout << "Banks: " << bank_count << endl;
        cout << "Read latency cycles/word: " << read_latency_cycles << endl;
        cout << "Write latency cycles/word: " << write_latency_cycles << endl;
        cout << "Read ops: " << read_ops << ", read words: " << read_words << endl;
        cout << "Write ops: " << write_ops << ", write words: " << write_words << endl;
        cout << "Peak used words: " << peak_used_words << endl;
        cout << "Modeled SRAM cycles: " << modeled_cycles << endl;
        cout << "=================================================" << endl;
    }
};

#endif
