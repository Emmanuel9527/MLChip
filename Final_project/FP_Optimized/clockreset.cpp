#include "clockreset.h"

void Clock::do_it() {
  clk = clk_intern;
  count++;
}

void Reset::do_it() {
  reset_n = 0;
  wait( ticks, SC_NS );
  reset_n = 1;
}
