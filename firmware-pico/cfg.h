#ifndef CFG_H
#define CFG_H

// It should not be necessary for most users to change anything below this
// point.

// The number of battery interfaces on the board
#define MAX_MODULES (16 * CHAIN_COUNT)

#define CHAIN_COUNT 1
#define SLAVES_PER_CHAIN 6
#define CELLS_PER_SLAVE 13 // not all channels of the slave are used
#define AUX_CHANNELS_PER_SLAVE 8
#define AUX_CHANNEL_TEMPERATURE_SENSOR_BATTERY_START 1
#define AUX_CHANNEL_TEMPERATURE_SENSOR_BATTERY_COUNT 3 // There are 6 temperature sensors for the slave, but only 3 are connected.
#define AUX_CHANNEL_TEMPERATURE_SENSOR_PCB           7 // The last AUX channel is used for the PCB sensor.

// Min absolute voltage to enable balancing.
#define BALANCE_MIN_mV 4000
// Min difference to enable balancing. 131 = 10mV
#define BALANCE_DIFF_mV 10
// Number of cells per module to bleed simultaneously
#define MAX_BALANCE_CELLS 1
// max temperature of the pcb during balancing
#define BALANCE_MAX_PCB_TEMP_C 50.0


// error and warning configuratuon
#define WARNING_LIMIT_SlaveTimeout_s 5
#define WARNING_UV_mV              3500
#define WARNING_OV_mV              4150
#define WARNING_UT_C               -200 // TODO -> adjust
#define WARNING_OT_C               45
#define WARNING_OC_A               100
#define ERROR_LIMIT_SlaveTimeout_s 10
#define ERROR_UV_mV                3300
#define ERROR_OV_mV                4180
#define ERROR_UT_C                 -200 // TODO -> adjust
#define ERROR_OT_C                 50
#define ERROR_OC_A                 150

// pin config
#define GPIO_RELAIS 15
#define GPIO_LEM_REF_PIN_NUMBER 26
#define GPIO_LEM_REF_ADC   0
#define GPIO_LEM_VALUE_PIN_NUMBER 27
#define GPIO_LEM_VALUE_ADC 1

#endif
