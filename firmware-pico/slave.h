#ifndef SLAVE_H
#define SLAVE_H
#include "pico/stdlib.h"
#include "cfg.h"
#include "bms.pio.h"
#include "stdio.h"
#include "math.h"

// Struct for RS485 transceiver info
struct battery_interface {
  uint16_t serial_out;
  uint16_t serial_master;
  uint16_t serial_enable;
  uint16_t serial_in;
  uint16_t sm;
};

extern float target_balance_voltage_mV;

extern void SLAVE_init(void);
extern void SLAVE_handle(void);

/*
 * returns the cell voltage of the given cell in mV. returns 5000 on error.
 */
extern uint16_t SLAVE_GetVoltage_mV(uint8_t chainNumber, uint8_t slaveNumber, uint8_t cellNumber);

/*
 * returns the temperature of the given cell in °C. returns 100 on error.
 */
extern uint8_t SLAVE_GetCellTemperature_C(uint8_t numberOfSlave, uint8_t numberOfSensor);

extern float SLAVE_Get_min_voltage_mV(void);
extern float SLAVE_Get_max_voltage_mV(void);
extern float SLAVE_Get_min_temperature_C(void);
extern float SLAVE_Get_max_temperature_C(void);

#endif
