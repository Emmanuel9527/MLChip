#ifndef MEMORY_MAP_H
#define MEMORY_MAP_H

// Byte-addressed DRAM layout used by the baseline final project.
// Every tensor is stored as contiguous 32-bit floating-point words.
static const unsigned int DRAM_INPUT_BASE      = 0x00000000u;
static const unsigned int DRAM_CONV1_W_BASE    = 0x00100000u;
static const unsigned int DRAM_CONV1_B_BASE    = 0x00200000u;
static const unsigned int DRAM_CONV2_W_BASE    = 0x00300000u;
static const unsigned int DRAM_CONV2_B_BASE    = 0x00700000u;
static const unsigned int DRAM_CONV3_W_BASE    = 0x00800000u;
static const unsigned int DRAM_CONV3_B_BASE    = 0x01000000u;
static const unsigned int DRAM_CONV4_W_BASE    = 0x01100000u;
static const unsigned int DRAM_CONV4_B_BASE    = 0x01b00000u;
static const unsigned int DRAM_CONV5_W_BASE    = 0x01c00000u;
static const unsigned int DRAM_CONV5_B_BASE    = 0x02300000u;
static const unsigned int DRAM_FC6_W_BASE      = 0x02400000u;
static const unsigned int DRAM_FC6_B_BASE      = 0x1a000000u;
static const unsigned int DRAM_FC7_W_BASE      = 0x1a100000u;
static const unsigned int DRAM_FC7_B_BASE      = 0x24b00000u;
static const unsigned int DRAM_FC8_W_BASE      = 0x24c00000u;
static const unsigned int DRAM_FC8_B_BASE      = 0x27600000u;
static const unsigned int DRAM_OUTPUT_BASE     = 0x27700000u;
static const unsigned int DRAM_INTER_BASE      = 0x27800000u;

static inline unsigned int dram_weight_base(int layer)
{
    switch (layer)
    {
    case 1: return DRAM_CONV1_W_BASE;
    case 2: return DRAM_CONV2_W_BASE;
    case 3: return DRAM_CONV3_W_BASE;
    case 4: return DRAM_CONV4_W_BASE;
    case 5: return DRAM_CONV5_W_BASE;
    case 6: return DRAM_FC6_W_BASE;
    case 7: return DRAM_FC7_W_BASE;
    case 8: return DRAM_FC8_W_BASE;
    default: return 0;
    }
}

static inline unsigned int dram_bias_base(int layer)
{
    switch (layer)
    {
    case 1: return DRAM_CONV1_B_BASE;
    case 2: return DRAM_CONV2_B_BASE;
    case 3: return DRAM_CONV3_B_BASE;
    case 4: return DRAM_CONV4_B_BASE;
    case 5: return DRAM_CONV5_B_BASE;
    case 6: return DRAM_FC6_B_BASE;
    case 7: return DRAM_FC7_B_BASE;
    case 8: return DRAM_FC8_B_BASE;
    default: return 0;
    }
}

static inline unsigned int dram_tensor_base(int layer, bool bias)
{
    if (layer == 0)
        return DRAM_INPUT_BASE;
    return bias ? dram_bias_base(layer) : dram_weight_base(layer);
}

#endif
