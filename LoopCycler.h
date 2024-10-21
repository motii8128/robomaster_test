#ifndef LOOP_CYCLER
#define LOOP_CYCLER

#include "Arduino.h"

namespace robomaster
{
  static void (*timerCallback)() = nullptr;

  class LoopCycler
  {
    public:
    LoopCycler(long delta_ms);
    void setFunc(void (*f)());
    void cycle();

    private:
    long delta_ms_;
    long prev_time_;
  };

}

#endif