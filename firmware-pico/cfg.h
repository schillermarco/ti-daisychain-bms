#ifndef CFG_H
#define CFG_H

// It should not be necessary for most users to change anything below this
// point.

// The number of battery interfaces on the board
#define MAX_MODULES (16 * CHAIN_COUNT)

#define CHAIN_COUNT 1
#define SLAVES_PER_CHAIN 16
#define CELLS_PER_SLAVE 13 // not all channels of the slave are used
#define AUX_CHANNELS_PER_SLAVE 8
#define AUX_CHANNEL_TEMPERATURE_SENSOR_BATTERY_START 1
#define AUX_CHANNEL_TEMPERATURE_SENSOR_BATTERY_COUNT 6 // There are 6 temperature sensors for the battery.
#define AUX_CHANNEL_TEMPERATURE_SENSOR_PCB           7 // The last AUX channel is used for the PCB sensor.

// Min absolute voltage to enable balancing.
#define BALANCE_MIN_mV 4000
// Min difference to enable balancing. 131 = 10mV
#define BALANCE_DIFF_mV 10
// Number of cells per module to bleed simultaneously
#define MAX_BALANCE_CELLS 1
// max temperature of the pcb during balancing
#define BALANCE_MAX_PCB_TEMP_C 50.0

#endif
