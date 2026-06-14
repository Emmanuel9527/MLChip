#include "dram.h"
#include "memory_map.h"
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>

string DRAM::find_data_path()
{
    const char *env_path = getenv("DATA_PATH");
    if (env_path != NULL)
        return string(env_path);

    const char *candidates[] = {
        "./data/",
        "../data/",
        "../../data/",
        "../../hw4/data/",
        "../../Final_report/data/"};

    for (int i = 0; i < 5; i++)
    {
        ifstream fin((string(candidates[i]) + "conv1_bias.txt").c_str());
        if (fin.good())
            return candidates[i];
    }
    return "./data/";
}

void DRAM::load_region(unsigned int base, const string &filename)
{
    ifstream fin((DATA_PATH + filename).c_str());
    if (!fin.is_open())
    {
        cout << "DRAM init failed: cannot open " << DATA_PATH + filename << endl;
        sc_stop();
        return;
    }

    vector<float> values;
    float value;
    while (fin >> value)
        values.push_back(value);

    // use swap to interchange the pointers and avoid copying the whole vector.
    regions[base].swap(values);
}

void DRAM::initialize()
{
#if defined(FP_TRACE)
    cout << "[TRACE] DRAM data path: " << DATA_PATH
         << ", image: " << IMAGE_FILE_NAME << endl;
#endif
    load_region(DRAM_INPUT_BASE, IMAGE_FILE_NAME);
    for (int layer = 1; layer <= 5; layer++)
    {
        stringstream ss;
        ss << layer;
        load_region(dram_weight_base(layer), "conv" + ss.str() + "_weight.txt");
        load_region(dram_bias_base(layer), "conv" + ss.str() + "_bias.txt");
    }
    for (int layer = 6; layer <= 8; layer++)
    {
        stringstream ss;
        ss << layer;
        load_region(dram_weight_base(layer), "fc" + ss.str() + "_weight.txt");
        load_region(dram_bias_base(layer), "fc" + ss.str() + "_bias.txt");
    }
    regions[DRAM_OUTPUT_BASE] = vector<float>(1000, 0.0f);
    regions[DRAM_INTER_BASE] = vector<float>();
}

float DRAM::read_word(unsigned int byte_addr)
{
    // map<> is a template class in the namespace std, iterator is a nested type defined in map<> to traverse the key-value pairs.
    map<unsigned int, vector<float>>::iterator best = regions.end();
    for (map<unsigned int, vector<float>>::iterator it = regions.begin(); it != regions.end(); ++it)
    {
        if (it->first <= byte_addr)
            best = it;
        else
            break;
    }
    if (best == regions.end())
        return 0.0f;

    unsigned int index = (byte_addr - best->first) / 4u;
    if (index >= best->second.size())
        return 0.0f;
    return best->second[index];
}

void DRAM::write_word(unsigned int byte_addr, float value)
{
    map<unsigned int, vector<float>>::iterator best = regions.end();
    for (map<unsigned int, vector<float>>::iterator it = regions.begin(); it != regions.end(); ++it)
    {
        if (it->first <= byte_addr)
            best = it;
        else
            break;
    }
    if (best == regions.end())
    {
        regions[byte_addr] = vector<float>(1, value);
        return;
    }

    unsigned int index = (byte_addr - best->first) / 4u;
    if (index >= best->second.size())
        best->second.resize(index + 1, 0.0f);
    best->second[index] = value;
}

void DRAM::run()
{
    arready.write(false);
    rvalid.write(false);
    rlast.write(false);
    awready.write(false);
    wready.write(false);
    bvalid.write(false);
    rdata.write(0.0f);

    while (!reset_n.read())
        wait();

    while (true)
    {
        // The DRAM slave is idle here, so it can advertise readiness before
        // the DMA master asserts VALID. A transfer is still accepted only on
        // a cycle where VALID and READY are both high.
        arready.write(true);
        awready.write(true);
        wait();

        if (arvalid.read() && arready.read())
        {
            unsigned int addr = araddr.read();
            unsigned int beats = arlen.read() + 1u;
            unsigned int step = 1u << arsize.read();
#if defined(FP_TRACE)
            if (addr < 0x100u)
            {
                cout << "[TRACE] DRAM accepted AR addr=0x"
                     << hex << addr << dec
                     << ", beats=" << beats << endl;
            }
#endif
            arready.write(false);
            awready.write(false);

            for (unsigned int beat = 0; beat < beats; beat++)
            {
                rdata.write(read_word(addr + beat * step));
                rlast.write(beat + 1u == beats);
                rvalid.write(true);
                unsigned int wait_cycles = 0;
                do
                {
                    wait();
#if defined(FP_TRACE)
                    wait_cycles++;
                    if (wait_cycles % 100000 == 0)
                    {
                        cout << "[TRACE] DRAM waiting RREADY, addr=0x"
                             << hex << (addr + beat * step) << dec
                             << ", beat=" << beat
                             << ", rvalid=" << rvalid.read()
                             << ", rready=" << rready.read() << endl;
                    }
#endif
                } while (!rready.read());
                rvalid.write(false);
                rlast.write(false);
            }
            continue;
        }

        if (awvalid.read() && awready.read())
        {
            unsigned int addr = awaddr.read();
            unsigned int beats = awlen.read() + 1u;
            unsigned int step = 1u << awsize.read();
            arready.write(false);
            awready.write(false);

            for (unsigned int beat = 0; beat < beats; beat++)
            {
                // WREADY is asserted when DRAM can accept a write-data beat.
                // The beat is captured only when WVALID is also high.
                wready.write(true);
                do
                {
                    wait();
                } while (!wvalid.read());
                write_word(addr + beat * step, wdata.read());
                bool last = wlast.read();
                wready.write(false);
                wait();
                (void)last;
            }

            bvalid.write(true);
            do
            {
                wait();
            } while (!bready.read());
            bvalid.write(false);
            continue;
        }

        wait();
    }
}
