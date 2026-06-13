#ifndef NOC_H
#define NOC_H

#include "systemc.h"

// One NoC flit carries a header/control field plus multiple 32-bit payload
// lanes. Increasing PACKED_FLIT_WORDS widens the datapath without changing the
// packet-level protocol.
static const int PACKED_FLIT_WORDS = 16;
static const int FLIT_WORD_BITS = 32;
static const int FLIT_PAYLOAD_BITS = PACKED_FLIT_WORDS * FLIT_WORD_BITS;
static const int FLIT_COUNT_BITS = 5;
static const int FLIT_TYPE_BITS = 2;
static const int FLIT_WIDTH = FLIT_PAYLOAD_BITS + FLIT_COUNT_BITS + FLIT_TYPE_BITS;

static const int FLIT_TYPE_MSB = FLIT_WIDTH - 1;
static const int FLIT_TYPE_LSB = FLIT_WIDTH - FLIT_TYPE_BITS;
static const int FLIT_COUNT_MSB = FLIT_TYPE_LSB - 1;
static const int FLIT_COUNT_LSB = FLIT_PAYLOAD_BITS;

static const int NOC_BODY_FLIT = 0;
static const int NOC_TAIL_FLIT = 1;
static const int NOC_HEAD_FLIT = 2;

typedef sc_lv<FLIT_WIDTH> Flit;

inline int flit_word_lsb(int lane)
{
    return lane * FLIT_WORD_BITS;
}

inline int flit_word_msb(int lane)
{
    return flit_word_lsb(lane) + FLIT_WORD_BITS - 1;
}

inline void set_flit_type(Flit &flit, int type)
{
    flit.range(FLIT_TYPE_MSB, FLIT_TYPE_LSB) = type;
}

inline int get_flit_type(const Flit &flit)
{
    return flit.range(FLIT_TYPE_MSB, FLIT_TYPE_LSB).to_uint();
}

inline void set_flit_count(Flit &flit, int count)
{
    flit.range(FLIT_COUNT_MSB, FLIT_COUNT_LSB) = count;
}

inline int get_flit_count(const Flit &flit)
{
    return flit.range(FLIT_COUNT_MSB, FLIT_COUNT_LSB).to_uint();
}

inline void set_flit_word(Flit &flit, int lane, unsigned int word)
{
    flit.range(flit_word_msb(lane), flit_word_lsb(lane)) = word;
}

inline unsigned int get_flit_word(const Flit &flit, int lane)
{
    return flit.range(flit_word_msb(lane), flit_word_lsb(lane)).to_uint();
}

inline void set_header_fields(Flit &flit, int dest_id, int source_id)
{
    unsigned int header =
        ((unsigned int)(dest_id & 0xffff) << 16) |
        (unsigned int)(source_id & 0xffff);
    set_flit_word(flit, 0, header);
}

inline int get_header_dest(const Flit &flit)
{
    return (get_flit_word(flit, 0) >> 16) & 0xffff;
}

inline int get_header_source(const Flit &flit)
{
    return get_flit_word(flit, 0) & 0xffff;
}

#endif
