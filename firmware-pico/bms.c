#include "cfg.h"
#include "application.h"
#include "pico/stdlib.h"


// PIO and state machine selection
#define SM_SQ 0

volatile int Flag_10ms = 0;

void timerHandle_10ms()
{
  Flag_10ms = 1;
}

int main() {
  APPLICATION_init();

  int Counter10ms  = 0;
  int Counter100ms  = 0;
  int Counter500ms  = 0;

  struct repeating_timer timer;
  add_repeating_timer_ms(10, timerHandle_10ms, NULL, &timer);

  while (1) {
    if(Flag_10ms == 1)
    {
        Flag_10ms = 0;
        Counter10ms++;
        // 10ms
        APPLICATION_10ms();
    }
    if(Counter10ms >= 10)
    {
        Counter10ms = 0;
        Counter100ms++;
        // 100ms
        APPLICATION_100ms();
    }
    if(Counter100ms >= 5)
    {
      Counter100ms = 0;
      Counter500ms++;
      // 500ms
      APPLICATION_500ms();
    }
    if(Counter500ms >= 2)
    {
      Counter500ms = 0;
      // 1s
      APPLICATION_1s();
    }
  }
}
