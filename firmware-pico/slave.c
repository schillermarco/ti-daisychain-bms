#include "slave.h"
#include <string.h>

#define MAX_VOLTAGE_mV 5000
#define MIN_VOLTAGE_mV 0

#define MAX_TEMPERATURE_C 100
#define MIN_TEMPERATURE_C 0

#define NUMBER_OF_TRANSACTION_ERROR_HISTORY 10

float min_voltage_mV = MIN_VOLTAGE_mV;
float max_voltage_mV = MAX_VOLTAGE_mV;
float min_temperature_C = MIN_TEMPERATURE_C;
float max_temperature_C = MAX_TEMPERATURE_C;

float target_balance_voltage_mV = 0;

#define TRANSACTION_RESULT_OK             0x00
#define TRANSACTION_RESULT_LENGTH_ERROR   0x01
#define TRANSACTION_RESULT_CRC_ERROR      0x02
uint8_t transaction_error_history[NUMBER_OF_TRANSACTION_ERROR_HISTORY];

#define SLAVE_STATE_WAKEUP 0x00
#define SLAVE_STATE_NORMAL 0x01
uint8_t slave_state = SLAVE_STATE_WAKEUP;

// Used for program loading
int offset;

// contains the raw ADC values for the cell voltages from the slave chip
uint16_t raw_cell_voltages[CHAIN_COUNT][SLAVES_PER_CHAIN][CELLS_PER_SLAVE];
// contains the raw ADC values for the auxilary inputs from the slave chip
uint16_t raw_aux_voltage[CHAIN_COUNT][SLAVES_PER_CHAIN][AUX_CHANNELS_PER_SLAVE];

// contains the raw ADC values for the cell voltages from the slave chip
uint16_t cell_voltages_mV[CHAIN_COUNT][SLAVES_PER_CHAIN][CELLS_PER_SLAVE];
// contains the temperature values for the battery pack
uint16_t temperatures_battery_C[CHAIN_COUNT][SLAVES_PER_CHAIN][AUX_CHANNEL_TEMPERATURE_SENSOR_BATTERY_COUNT];
// contains the temperature values for the PCB sensor. One temperature sensor per PCB
uint16_t temperatures_pcb_C[CHAIN_COUNT][SLAVES_PER_CHAIN];

// contains a bitmap, which cells should be balanced
uint16_t balance_bitmap[CHAIN_COUNT][SLAVES_PER_CHAIN];

#define RX_BUFFER_SIZE 128
// Buffers for received data
uint8_t rx_data_buffer[RX_BUFFER_SIZE];


// Define pins for RS485 transceivers
struct battery_interface battery_interfaces[CHAIN_COUNT] = {{
                                                      .serial_out = 2,
                                                      .serial_master = 3,
                                                      .serial_enable = 4,
                                                      .serial_in = 5,
                                                      .sm = 0,
                                                  },
                                                  {
                                                      .serial_out = 10,
                                                      .serial_master = 11,
                                                      .serial_enable = 12,
                                                      .serial_in = 13,
                                                      .sm = 1,
                                                  }};

// Calculate message CRC.
uint16_t crc16(uint8_t* message, uint8_t length) {
  uint16_t crc = 0;
  uint16_t j;
  while (length--) {
    crc ^= *message++;
    for (j = 0; j < 8; j++) {
      crc = (crc >> 1) ^ ((crc & 1) ? 0xa001 : 0);
    }
  }
  return crc;
}

void WriteTransactionResultInBuffer(uint8_t result){
    static uint8_t transaction_error_counter_position = 0;

    if(transaction_error_counter_position < NUMBER_OF_TRANSACTION_ERROR_HISTORY)
    {
        transaction_error_history[transaction_error_counter_position] = result;
    }

    transaction_error_counter_position++;

    if(transaction_error_counter_position >= NUMBER_OF_TRANSACTION_ERROR_HISTORY)
    {
        transaction_error_counter_position = 0;
    }
}

// Return 1 if all PCB temperature sensors on a module are above 1.0v
uint8_t pcb_below_temp(uint8_t chain, uint8_t slave) {
  return temperatures_pcb_C[chain][slave] < BALANCE_MAX_PCB_TEMP_C;
}

float voltage_mV(uint16_t adc) {
  return (float)(adc / 13107.0);
}

float get_C_from_K(float kelvin){
  return kelvin - 273.15;
}

float get_K_from_C(float celsius){
  return celsius + 273.15;
}

float get_temperature_C(uint8_t chain, uint8_t slave, uint16_t adc) {
  float R_pullup = 10000.0;
  float NTC_B = 3960.0;
  float NTC_RT0 = 10000.0;
  float T0 = 25.0;
  float T0_K = get_K_from_C(T0);

  // The first aux channel is connected to the Reference voltage for the temperature sensors
  int adc_ref_value = raw_aux_voltage[chain][slave][0];
  int quotient = adc_ref_value-adc;
  if(quotient == 0)
  {
	  return MAX_TEMPERATURE_C;
  }
  float r = (adc*R_pullup)/(quotient);
  float r_K = get_K_from_C(r);
  float numerus = r_K/NTC_RT0;
  if(numerus <= 0)
  {
	  return MAX_TEMPERATURE_C;
  }
  float invTemp = (1/T0_K) + (1/NTC_B)*log(numerus);
  if(invTemp == 0)
  {
	  return MAX_TEMPERATURE_C;
  }
  float temperature_K = (1/invTemp);
  float t_C = get_C_from_K(temperature_K);

  return t_C;
}


// Send a command string
void send_command(struct battery_interface* battery_interface, uint8_t* command, uint8_t length) {
  uint16_t crc = crc16(command, length);
  // Append framing but to the first byte and send it
  pio_sm_put_blocking(pio0, battery_interface->sm, command[0] | 0x100);
  // Send remaining bytes
  for (int n = 1; n < length; n++) pio_sm_put_blocking(pio0, battery_interface->sm, command[n]);
  // Send CRC16
  pio_sm_put_blocking(pio0, battery_interface->sm, crc & 0xFF);
  pio_sm_put_blocking(pio0, battery_interface->sm, crc >> 8);
  busy_wait_us(20);  // Always insert a short pause after sending commands
}


// Configure all daisychained packs with sequential addresses
void configure(struct battery_interface* battery_interface) {
  // Fully Enable Differential Interfaces and Select Auto-Addressing Mode
  send_command(battery_interface, (uint8_t[]){0xF2, 0x10, 0x10, 0xE0}, 4);
  // Configure the bq76PL455A-Q1 device to use auto-addressing to select address
  send_command(battery_interface, (uint8_t[]){0xF1, 0x0E, 0x10}, 3);
  // Configure the bq76PL455A-Q1 device to enter auto-address mode
  send_command(battery_interface, (uint8_t[]){0xF1, 0x0C, 0x08}, 3);
  // Configure devices with sequential addresses
  for (int n = 0; n < SLAVES_PER_CHAIN; n++) {
    send_command(battery_interface, (uint8_t[]){0xF1, 0x0A, n}, 3);
  }
}

// Put all modules to sleep
void SLAVE_sleep_modules() {
  // Broadcast sleep command to each chain
  for (int n = 0; n < CHAIN_COUNT; n++) send_command(battery_interfaces + n, (uint8_t[]){0xF1, 0x0C, 0x48}, 3);
  // Wait a little for safety
  busy_wait_ms(1);
  // Disable line drivers
  for (int n = 0; n < CHAIN_COUNT; n++) gpio_put(battery_interfaces[n].serial_enable, 1);
  // Ensure modules are woken when needed again
}

// Receive data from PIO into a local buffer, size limit and timeout in
// microseconds specified It's probably unnecessary to do this with an interrupt
// because we know when we expect to receive data
uint16_t SLAVE_receive_data(struct battery_interface* battery_interface, uint8_t* buffer, uint16_t size, uint32_t timeout) {
  uint16_t rx_data_offset = 0;
  // Return immediately if size is zero
  if (size == 0) return 0;
  // Loop until timeout expires
  for (int n = 0; n < timeout; n++) {
    // Check for data in input FIFO
    while (!pio_sm_is_rx_fifo_empty(pio1, battery_interface->sm)) {
      // Receive one byte
      buffer[rx_data_offset++] = pio_sm_get_blocking(pio1, battery_interface->sm);
      // Return full size if we've filled the string
      if (rx_data_offset == size) return size;
    }
    // Sleep 1 microsecond each loop
    busy_wait_us(1);
  }
  // Return partial length received
  return rx_data_offset;
}

// Deactivate the TX PIO and send a square wave to wake up the device
void wakeup(struct battery_interface* battery_interface) {
  // Enable the line driver
  gpio_put(battery_interface->serial_enable, 0);
  // Disable TX PIO
  pio_sm_set_enabled(pio0, battery_interface->sm, false);
  // Wait for it to be disabled
  busy_wait_ms(1);
  // Loop for 100 x 10us
  for (int n = 0; n < 100; n++) {
    pio_sm_set_pins(pio0, battery_interface->sm, (1 << battery_interface->serial_master) | (1 << battery_interface->serial_out));  // Drive DO high (DE enabled)
    busy_wait_us(2);
    pio_sm_set_pins(pio0, battery_interface->sm, (1 << battery_interface->serial_master) | (0 << battery_interface->serial_out));  // Drive DO low (DE enabled)
    busy_wait_us(2);
  }
  pio_sm_set_pins(pio0, battery_interface->sm, (1 << battery_interface->serial_master) | (1 << battery_interface->serial_out));  // Drive DO high (DE enabled)
  busy_wait_us(2);
  // Disable DE, stop driving bus
  pio_sm_set_pins(pio0, battery_interface->sm, 0);
  // Re-enable TX PIO
  pio_sm_set_enabled(pio0, battery_interface->sm, true);
}

void rewake() {
  // Wake and reset up the modules
  for (int n = 0; n < CHAIN_COUNT; n++) wakeup(battery_interfaces + n);

  // Give the modules time to reset
  busy_wait_ms(10);

  // Configure the module addresses
  for (int n = 0; n < CHAIN_COUNT; n++) configure(battery_interfaces + n);
}

// Request that all packs simultaneously sample voltage
void sample_all() {
  // 0xFF 0xFF 0xFF - these 24 bits enable sampling of 16 cell voltages and
  //                  8 AUX channels. Some will contain temperature data.
  //                  16x oversampling.
  for (int n = 0; n < CHAIN_COUNT; n++) send_command(battery_interfaces + n, (uint8_t[]){0xF6, 0x02, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x04}, 8);
  // Wait for sampling to complete
  busy_wait_ms(10);
}

void read_all_adc_values() {
    // Collect voltages and temperature data for all modules
    // We want to complete this loop as fast as possible because balancing must
    // be disabled during measurement

    for (int chain = 0; chain < CHAIN_COUNT; chain++) {
        for (int slave = 0; slave < SLAVES_PER_CHAIN; slave++) {
            // Clear the input FIFO just in case
            pio_sm_clear_fifos(pio1, battery_interfaces[chain].sm);
            // Request sampled voltage data from module
            send_command(battery_interfaces + chain, (uint8_t[]){0x81, slave, 0x02, 0x20}, 4);
            // Receive response data from PIO FIFO into CPU buffer - 51 bytes of data
            // with 10ms timeout 24 values * 2 bytes + length + 2 byte checksum = 51
            uint16_t received = SLAVE_receive_data(battery_interfaces + chain, rx_data_buffer, 51, 10000);
            // Check RX CRC
            uint16_t rx_crc = crc16(rx_data_buffer, 51);
            if (received == 51 && rx_crc == 0) {
                for (int cell = 0; cell < CELLS_PER_SLAVE; cell++) {
                // nb. Cells are in reverse, cell 16 is reported first
                    raw_cell_voltages[chain][slave][cell] = rx_data_buffer[(15 - cell) * 2 + 1] << 8 | rx_data_buffer[(15 - cell) * 2 + 2];
                }
                for (int aux = 0; aux < AUX_CHANNELS_PER_SLAVE; aux++) {
                    // aux values are in reverse order
                    raw_aux_voltage[chain][slave][aux] = rx_data_buffer[(23 - aux) * 2 + 1] << 8 | rx_data_buffer[(23 - aux) * 2 + 2];
                }
                WriteTransactionResultInBuffer(TRANSACTION_RESULT_OK);
            } else {
                if (received == 51)
                {
                    WriteTransactionResultInBuffer(TRANSACTION_RESULT_LENGTH_ERROR);
                    printf("CRC Error %i %i: %04x\n", chain, slave, rx_crc);
                }
                else
                {
                    WriteTransactionResultInBuffer(TRANSACTION_RESULT_CRC_ERROR);
                    printf("RX Error %i %i: %i\n", chain, slave, received);
                }

                slave_state = SLAVE_STATE_WAKEUP;
                target_balance_voltage_mV = 0;
            }
        }
    }


    // reset max and min values
    min_voltage_mV = MAX_VOLTAGE_mV;
    max_voltage_mV = MIN_VOLTAGE_mV;
    min_temperature_C = MAX_TEMPERATURE_C;
    max_temperature_C = MIN_TEMPERATURE_C;

    for (int chain = 0; chain < CHAIN_COUNT; chain++) {
        for (int slave = 0; slave < SLAVES_PER_CHAIN; slave++) {
            for (int cell = 0; cell < CELLS_PER_SLAVE; cell++) {
                float cell_voltage_mV = voltage_mV(raw_cell_voltages[chain][slave][cell]);
                cell_voltages_mV[chain][slave][cell] = cell_voltage_mV;

                if(cell_voltage_mV > max_voltage_mV){
                    max_voltage_mV = cell_voltage_mV;
                }

                if(cell_voltage_mV < min_voltage_mV){
                    min_voltage_mV = cell_voltage_mV;
                }
            }
            for (int aux = AUX_CHANNEL_TEMPERATURE_SENSOR_BATTERY_START; aux < (AUX_CHANNEL_TEMPERATURE_SENSOR_BATTERY_START+AUX_CHANNEL_TEMPERATURE_SENSOR_BATTERY_COUNT); aux++) {
                float temperature_C = get_temperature_C(chain, slave, raw_aux_voltage[chain][slave][aux]);

                // The number of temperature sensor differs from the AUX channel, because the first AUX channel is used for a diffrent purpose
                uint8_t number_of_temperature_sensor  = aux - AUX_CHANNEL_TEMPERATURE_SENSOR_BATTERY_START;
                temperatures_battery_C[chain][slave][number_of_temperature_sensor] = temperature_C;

                if(temperature_C > max_temperature_C){
                    max_temperature_C = temperature_C;
                }

                if(temperature_C < min_temperature_C){
                    min_temperature_C = temperature_C;
                }
            }

            float temperature_C = get_temperature_C(chain, slave, raw_aux_voltage[chain][slave][AUX_CHANNEL_TEMPERATURE_SENSOR_PCB]);
            temperatures_pcb_C[chain][slave] = temperature_C;
        }
    }
}

void handle_balancing(){
    // Work out if balancing is required
    if (max_voltage_mV > BALANCE_MIN_mV) {
      if (max_voltage_mV > min_voltage_mV + BALANCE_DIFF_mV) {
        // At least one cell is overcharged, lets balance!
        target_balance_voltage_mV = min_voltage_mV + BALANCE_DIFF_mV;
        if (target_balance_voltage_mV < BALANCE_MIN_mV) target_balance_voltage_mV = BALANCE_MIN_mV;  // No less than BALANCE_MIN
      } else {
        // Cells are balanced
        target_balance_voltage_mV = 0;
      }
    } else {
      // Under balancing threshold
      target_balance_voltage_mV = 0;
    }

    // Balancing
    for (int chain = 0; chain < CHAIN_COUNT; chain++) {
        for (int slave = 0; slave < SLAVES_PER_CHAIN; slave++) {
            balance_bitmap[chain][slave] = 0;
            // Only balance if conditions are appropriate
            if (pcb_below_temp(chain, slave) && target_balance_voltage_mV > 0){
                // Add up to MAX_BALANCE_CELLS to balancing bitmap
                for (int n = 0; n < MAX_BALANCE_CELLS; n++) {
                    uint16_t max_v_mV = 0;
                    int8_t selected_cell = 0;
                    // Loop over each cell
                    for (int cell = 0; cell < CELLS_PER_SLAVE; cell++) {
                        // Ignore cell if already balancing
                        if (balance_bitmap[chain][slave] & (1 << cell)) continue;
                        // If cell is highest cell so far, provisionally select it
                        if (cell_voltages_mV[chain][slave][cell] > target_balance_voltage_mV)
                        if (cell_voltages_mV[chain][slave][cell] > max_v_mV) {
                            selected_cell = cell + 1;
                            max_v_mV = cell_voltages_mV[chain][slave][cell];
                        }
                    }
                    // Add selected candidate to final bitmap
                    if (selected_cell) balance_bitmap[chain][slave] |= (1 << (selected_cell - 1));
                }
                send_command(battery_interfaces + chain, (uint8_t[]){0x92, slave, 0x14, balance_bitmap[chain][slave] >> 8, balance_bitmap[chain][slave]}, 5);
            }
        }
    }
}

void SLAVE_init(){
  // Load and initialize the TX PIO program
  offset = pio_add_program(pio0, &daisychain_tx_program);
  for (int n = 0; n < CHAIN_COUNT; n++) daisychain_tx_program_init(pio0, battery_interfaces[n].sm, offset, battery_interfaces[n].serial_out, battery_interfaces[n].serial_master);

  // Load and initialize the RX PIO program
  offset = pio_add_program(pio1, &daisychain_rx_program);
  for (int n = 0; n < CHAIN_COUNT; n++) daisychain_rx_program_init(pio1, battery_interfaces[n].sm, offset, battery_interfaces[n].serial_in, battery_interfaces[n].serial_master);

  // Configure serial enable pins
  for (int chain = 0; chain < CHAIN_COUNT; chain++) {
    gpio_init(battery_interfaces[chain].serial_enable);
    gpio_set_dir(battery_interfaces[chain].serial_enable, GPIO_OUT);
  }

  slave_state = SLAVE_STATE_WAKEUP;
}

void clearAllVariables(){
    memset(raw_cell_voltages, 0, sizeof(raw_cell_voltages));
    memset(cell_voltages_mV, 0, sizeof(cell_voltages_mV));
    memset(raw_aux_voltage, 0, sizeof(raw_aux_voltage));
    memset(temperatures_battery_C, 0, sizeof(temperatures_battery_C));
    memset(temperatures_pcb_C, 0, sizeof(temperatures_pcb_C));
    memset(rx_data_buffer, 0, sizeof(rx_data_buffer));

    memset(transaction_error_history, 0, sizeof(transaction_error_history));
    min_voltage_mV = MIN_VOLTAGE_mV;
    max_voltage_mV = MAX_VOLTAGE_mV;
    min_temperature_C = MIN_TEMPERATURE_C;
    max_temperature_C = MAX_TEMPERATURE_C;

    target_balance_voltage_mV = 0;

    uint8_t slave_state = SLAVE_STATE_WAKEUP;
}

void handleStateWakeup(){
    clearAllVariables();
    rewake();

    slave_state = SLAVE_STATE_NORMAL;
}

void handleStateNormal(){
    // configure timeout and disable balancing
    for (int chain = 0; chain < CHAIN_COUNT; chain++) {
      // Set discharge timeout (1min, all modules)
      send_command(battery_interfaces + chain, (uint8_t[]){0xF1, 0x13, (2 << 4) | (1 << 3)}, 3);
      // Disable discharge (all modules)
      send_command(battery_interfaces + chain, (uint8_t[]){0xF2, 0x14, 0, 0}, 4);
      // Set communication timeout (10s) (all modules)
      send_command(battery_interfaces + chain, (uint8_t[]){0xF1, 0x28, (6 << 4)}, 3);
    }

    sample_all();
    read_all_adc_values();
    handle_balancing();
}

void SLAVE_handle(void){
    switch (slave_state)
    {
    case SLAVE_STATE_WAKEUP:
        handleStateWakeup();
        break;
    case SLAVE_STATE_NORMAL:
        handleStateNormal();
        break;
    default:
        break;
    }
}



uint16_t SLAVE_GetVoltage_mV(uint8_t chainNumber, uint8_t slaveNumber, uint8_t cellNumber){
    return (uint16_t)(cell_voltages_mV[chainNumber][slaveNumber][cellNumber]);
}

float SLAVE_Get_min_voltage_mV(void){
    return min_voltage_mV;
}

float SLAVE_Get_max_voltage_mV(void){
    return max_voltage_mV;
}

float SLAVE_Get_min_temperature_C(void){
    return min_temperature_C;
}

float SLAVE_Get_max_temperature_C(void){
    return max_temperature_C;
}
