#include "hardware/clocks.h"

#include "cfg.h"
#include "can.h"
#include "slave.h"


// PIO and state machine selection
#define SM_SQ 0


void reconfigure_clocks() {
  // Clock the peripherals, ref clk, and rtc from the 12MHz crystal oscillator
  clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 12000000, 12000000);
  clock_configure(clk_ref, CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC, 0, 12000000, 12000000);
  clock_configure(clk_rtc, 0, CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 12000000, 46875);
}

int main() {
  // Set system clock to 80MHz, this seems like a reasonable value for the 4MHz
  // data
  set_sys_clock_khz(80000, true);
  stdio_init_all();
  reconfigure_clocks();

  //  Used to keep track of battery voltage data stream
  uint8_t can_chain = 0, can_slave = 0, can_cell = 0;

  SLAVE_init();
  CAN_init();

  // Main loop.
  while (1) {
    SLAVE_handle();

    float overall_voltage_mV = 0;
    for (int chain = 0; chain < CHAIN_COUNT; chain++) {
        for (int slave = 0; slave < SLAVES_PER_CHAIN; slave++) {
            for (int cell = 0; cell < CELLS_PER_SLAVE; cell++) {
              overall_voltage_mV += SLAVE_GetVoltage_mV(chain, slave, cell);
            }
        }
    }
    CAN_send_pack_voltage(overall_voltage_mV);

    float min_voltage_mV = SLAVE_Get_min_voltage_mV();
    float max_voltage_mV = SLAVE_Get_max_voltage_mV();
    float min_temperature_C = SLAVE_Get_min_temperature_C();
    float max_temperature_C = SLAVE_Get_max_temperature_C();
    CAN_send_min_max_values(min_voltage_mV, max_voltage_mV, min_temperature_C, max_temperature_C);

    // Send out individual cell voltages one at a time
    uint16_t voltage_mV = SLAVE_GetVoltage_mV(can_chain, can_slave, can_cell);
    CAN_send_individual_cell_voltage(can_chain, can_slave, can_cell, voltage_mV);

    can_cell++;
    if (can_cell == CELLS_PER_SLAVE) {
      can_cell = 0;
      can_slave++;
      if (can_slave == SLAVES_PER_CHAIN) {
        can_slave = 0;
        can_chain++;
        if (can_chain == CHAIN_COUNT) {
          can_chain = 0;
        }
      }
    }

    sleep_ms(500);
  }
}
