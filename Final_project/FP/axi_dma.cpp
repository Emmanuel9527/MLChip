#include "axi_dma.h"
#include <algorithm>
#include <iostream>

using namespace std;

static const unsigned int AXI_BURST_MAX_BEATS = 16;
static const unsigned int AXI_FLOAT_SIZE_LOG2 = 2;

void AxiDma::read_burst(unsigned int addr, unsigned int beats)
{
#if defined(FP_TRACE)
    if (read_bursts < 4 || ((read_bursts + 1) % 100000 == 0))
        cout << "[TRACE] DMA read burst #" << (read_bursts + 1)
             << " addr=0x" << hex << addr << dec
             << ", beats=" << beats
             << ", total_read_words=" << read_words << endl;
#endif
    // Send the AXI read address phase. ARLEN stores beats - 1, following AXI.
    araddr.write(addr);
    arlen.write(beats - 1u);
    arsize.write(AXI_FLOAT_SIZE_LOG2);
    arvalid.write(true);
    do
    {
        wait();
    } while (!arready.read());
    arvalid.write(false);

    // Accept each read beat from DRAM, then forward it to the Controller.
    // RREADY is held until DRAM presents RVALID. read_valid is held until the
    // Controller accepts the streamed word with read_ready.
    for (unsigned int i = 0; i < beats; i++)
    {
        rready.write(true);
        do
        {
            wait();
        } while (!rvalid.read());
        float value = rdata.read();
        (void)rlast.read();
        rready.write(false);
        read_data.write(value);
        read_valid.write(true);
        do
        {
            wait();
        } while (!read_ready.read());
        read_valid.write(false);
        read_words++;
    }
    read_bursts++;
}

void AxiDma::write_burst(unsigned int addr, unsigned int beats)
{
#if defined(FP_TRACE)
    cout << "[TRACE] DMA write burst #" << (write_bursts + 1)
         << " addr=0x" << hex << addr << dec
         << ", beats=" << beats
         << ", total_write_words=" << write_words << endl;
#endif
    // Send the AXI write address phase before any write-data beats.
    awaddr.write(addr);
    awlen.write(beats - 1u);
    awsize.write(AXI_FLOAT_SIZE_LOG2);
    awvalid.write(true);
    do
    {
        wait();
    } while (!awready.read());
    awvalid.write(false);

    // Pull one word at a time from the Controller and push it to DRAM.
    // Both sides use valid/ready handshakes, so VALID is kept asserted until
    // the receiver raises READY.
    for (unsigned int i = 0; i < beats; i++)
    {
        // Wait for the Controller to present the next word to DMA write.
        write_ready.write(true);
        do
        {
            wait();
        } while (!write_valid.read());
        float value = write_data.read();
        write_ready.write(false);

        // DMA accepts the word and sends it to DRAM.
        wdata.write(value);
        wlast.write(i + 1u == beats);
        wvalid.write(true);
        do
        {
            wait();
        } while (!wready.read());
        wvalid.write(false);
        wlast.write(false);
        write_words++;
    }

    // Wait for DRAM's write response before completing this burst.
    bready.write(true);
    do
    {
        wait();
    } while (!bvalid.read());
    bready.write(false);
    write_bursts++;
}

void AxiDma::run()
{
    // Drive all outputs to idle values before reset is released.
    cmd_ready.write(false);
    done.write(false);
    read_valid.write(false);
    write_ready.write(false);
    arvalid.write(false);
    rready.write(false);
    awvalid.write(false);
    wvalid.write(false);
    wlast.write(false);
    bready.write(false);
    read_data.write(0.0f);
    wdata.write(0.0f);

    while (!reset_n.read())
        wait();

    while (true)
    {
        // Wait for one Controller command. The DMA handles one command at a
        // time in this baseline, so outstanding transactions are not modeled.
        cmd_ready.write(true);
        done.write(false);
        wait();

        if (!cmd_valid.read())
            continue;

        bool is_write = cmd_write.read();
        unsigned int base = cmd_addr.read();
        unsigned int remaining = cmd_len.read();
        cmd_ready.write(false);

        // Split long commands into fixed-size AXI bursts. The byte address is
        // advanced by four bytes per float word.
        unsigned int offset = 0;
        while (remaining > 0)
        {
            unsigned int beats = min(remaining, AXI_BURST_MAX_BEATS);
            if (is_write)
                write_burst(base + offset * 4u, beats);
            else
                read_burst(base + offset * 4u, beats);
            offset += beats;
            remaining -= beats;
        }

#if defined(FP_TRACE)
        if (is_write)
            cout << "[TRACE] DMA write command done, total_write_words="
                 << write_words << ", total_write_bursts=" << write_bursts << endl;
#endif
        done.write(true);
        wait();
        done.write(false);
    }
}
