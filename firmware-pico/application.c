#include "application.h"
#include "hardware/clocks.h"
#include "can.h"
#include "slave.h"
#include "error.h"
#include "currentMeasurement.h"

void handle_can_sending_voltages(){
  static uint8_t can_chain = 0, can_slave = 0, can_cell = 0;

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
}

void handle_can_sending_temperatures_battery(){
  static uint8_t can_chain = 0, can_slave = 0, sensor_number = 0;
  float temeprature_C = SLAVE_GetBatteryTemperature_C(can_chain, can_slave, sensor_number);
  CAN_send_individual_temperature_battery(can_chain, can_slave, sensor_number, temeprature_C);

  sensor_number++;
  if (sensor_number == AUX_CHANNEL_TEMPERATURE_SENSOR_BATTERY_COUNT) {
    sensor_number = 0;
    can_slave++;
    if (can_slave == SLAVES_PER_CHAIN) {
      can_slave = 0;
      can_chain++;
      if (can_chain == CHAIN_COUNT) {
        can_chain = 0;
      }
    }
  }
}

void handle_can_sending_temperatures_pcb(){
  static uint8_t can_chain = 0, can_slave = 0;
  float temeprature_C = SLAVE_GetPCBTemperature_C(can_chain, can_slave);
  CAN_send_individual_temperature_pcb(can_chain, can_slave, 0, temeprature_C);

  can_slave++;
  if (can_slave == SLAVES_PER_CHAIN) {
    can_slave = 0;
    can_chain++;
    if (can_chain == CHAIN_COUNT) {
      can_chain = 0;
    }
  }
}

void handle_can_sending()
{
  handle_can_sending_voltages();
  handle_can_sending_temperatures_battery();
  handle_can_sending_temperatures_pcb();
}


void reconfigure_clocks() {
  // Clock the peripherals, ref clk, and rtc from the 12MHz crystal oscillator
  clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 12000000, 12000000);
  clock_configure(clk_ref, CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC, 0, 12000000, 12000000);
  clock_configure(clk_rtc, 0, CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 12000000, 46875);
}

void APPLICATION_init(void)
{
  // Set system clock to 80MHz, this seems like a reasonable value for the 4MHz
  // data
  set_sys_clock_khz(80000, true);
  reconfigure_clocks();
  // stdio_init_all must be called after clock change, because otherwise UART baudrate will be wrong
  stdio_init_all();

  SLAVE_init();
  CAN_init();
  CURRENTMEASURMENT_init();
  ERROR_init();
}

void APPLICATION_10ms(void)
{
    handle_can_sending();
}

void APPLICATION_100ms(void)
{
    CURRENTMEASURMENT_Handle_100ms();
}

void APPLICATION_500ms(void)
{
    float overall_voltage_mV = 0;
    for (int chain = 0; chain < CHAIN_COUNT; chain++) {
        for (int slave = 0; slave < SLAVES_PER_CHAIN; slave++) {
            for (int cell = 0; cell < CELLS_PER_SLAVE; cell++) {
              overall_voltage_mV += SLAVE_GetVoltage_mV(chain, slave, cell);
            }
        }
    }

    float current = 0;
    CURRENTMEASURMENT_GetCurrent_A(&current);

    CAN_send_pack_voltage(overall_voltage_mV, current);

    float min_voltage_mV = SLAVE_Get_min_voltage_mV();
    float max_voltage_mV = SLAVE_Get_max_voltage_mV();
    float min_temperature_C = SLAVE_Get_min_temperature_C();
    float max_temperature_C = SLAVE_Get_max_temperature_C();
    CAN_send_min_max_values(min_voltage_mV, max_voltage_mV, min_temperature_C, max_temperature_C);

    printf("Min voltage[mV]: %f\n", min_voltage_mV);
    printf("Max voltage[mV]: %f\n", max_voltage_mV);
    printf("Min temperature[C]: %f\n", min_temperature_C);
    printf("Max temperature[C]: %f\n", max_temperature_C);
    printf("\n\n");

    SLAVE_handle();
}

void APPLICATION_1s(void)
{
  ERROR_Handle_1s();
  t_error_flags flags = ERROR_getErrorState();

  CAN_send_errors(flags);
}
