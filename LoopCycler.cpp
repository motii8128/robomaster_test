#include "LoopCycler.h"

namespace robomaster
{
  LoopCycler::LoopCycler(long delta_ms): delta_ms_(delta_ms),prev_time_(0)
  {
  
  }

  void LoopCycler::setFunc(void (*f)())
  {
    noInterrupts();

    timerCallback = f;

    TCCR1A = 0;
    TCCR1B = 0;
    auto max_count = (delta_ms_ * 10e-6) * 16 * 10e16;
    OCR1A = max_count - 1;

    TCCR1B |= (1<<WGM12) |(1<<CS10);
    TIMSK1 |= (1 << OCIE1A);
    interrupts();
  }

  void LoopCycler::cycle()
  {
    while(millis() - prev_time_ < delta_ms_)
    {

    }

    prev_time_ = millis();
  }

  ISR(TIMER1_COMPA_vect) {
    if(timerCallback != nullptr)
    {
      timerCallback();
    }
  }

}