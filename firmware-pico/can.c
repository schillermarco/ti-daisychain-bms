#include "can.h"


#define DBC_TEMPERATURE_FACTOR 10

void SPI_configure() {
  spi_init(SPI_PORT, 1000000);
  spi_set_format(SPI_PORT, 8, 0, 0, SPI_MSB_FIRST);
  gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);
  gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(SPI_CLK, GPIO_FUNC_SPI);
  gpio_init(SPI_CS);
  gpio_set_dir(SPI_CS, GPIO_OUT);
  gpio_put(SPI_CS, 1);
}

void CAN_init() {
    SPI_configure();

    // Configure CAN transceiver sleep line
    gpio_init(CAN_SLEEP);
    gpio_set_dir(CAN_SLEEP, GPIO_OUT);
    gpio_put(CAN_SLEEP, 0);  // Logic low to wake transceiver

    // Output 8MHz square wave on CAN_CLK pin
    clock_gpio_init(CAN_CLK, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 10);

    // Configure SPI to communicate with CAN
    SPI_configure();
    // Set up CAN to receive messages
    CAN_reset();
    CAN_configure(0x4F8);
}

void CAN_reset() {
  gpio_put(SPI_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_RESET}, 1);
  gpio_put(SPI_CS, 1);
  busy_wait_us(100);
}

uint8_t CAN_reg_read(uint8_t reg) {
  uint8_t data;
  gpio_put(SPI_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_READ, reg}, 2);
  spi_read_blocking(SPI_PORT, 0, &data, 1);
  gpio_put(SPI_CS, 1);
  return (data);
}

void CAN_reg_write(uint8_t reg, uint8_t val) {
  gpio_put(SPI_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_WRITE, reg, val}, 3);
  gpio_put(SPI_CS, 1);
}

void CAN_reg_modify(uint8_t reg, uint8_t mask, uint8_t val) {
  gpio_put(SPI_CS, 0);
  busy_wait_us(2);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_MODIFY, reg, mask, val}, 4);
  busy_wait_us(2);
  gpio_put(SPI_CS, 1);
}

void CAN_configure(uint16_t id) {
  // Configure speed to 500kbps based on 8MHz Crystal
  // Magic constants from
  // https://github.com/sandeepmistry/arduino-CAN/blob/master/src/MCP2515.cpp
  CAN_reg_write(REG_CNF1, 0x00);
  CAN_reg_write(REG_CNF2, 0x90);
  CAN_reg_write(REG_CNF3, 0x02);

  // Enable Filters
  CAN_reg_write(REG_RXBnCTRL(0), 1 << 2);  // Enable rollover from BUF0 to BUF1
  CAN_reg_write(REG_RXBnCTRL(1), 0);
  // Set masks for RXB0 and RXB1 the same
  for (int n = 0; n < 2; n++) {
    uint16_t mask = 0x7ff;
    CAN_reg_write(REG_RXMnSIDH(n), mask >> 3);
    CAN_reg_write(REG_RXMnSIDL(n), mask << 5);
    CAN_reg_write(REG_RXMnEID8(n), 0);
    CAN_reg_write(REG_RXMnEID0(n), 0);
  }
  // Set match ID for all filters the same
  for (int n = 0; n < 6; n++) {
    CAN_reg_write(REG_RXFnSIDH(n), id >> 3);
    CAN_reg_write(REG_RXFnSIDL(n), id << 5);
    CAN_reg_write(REG_RXFnEID8(n), 0);
    CAN_reg_write(REG_RXFnEID0(n), 0);
  }

  // Enable receive interrupts
  CAN_reg_write(REG_CANINTE, 3);

  // Set normal operation mode
  CAN_reg_write(REG_CANCTRL, MODE_NORMAL);
}

void transmit(uint16_t id, uint8_t* data, uint8_t length) {
  CAN_reg_write(REG_TXBnSIDH(0), id >> 3);  // Set CAN ID
  CAN_reg_write(REG_TXBnSIDL(0), id << 5);  // Set CAN ID
  CAN_reg_write(REG_TXBnEID8(0), 0x00);     // Extended ID
  CAN_reg_write(REG_TXBnEID0(0), 0x00);     // Extended ID

  CAN_reg_write(REG_TXBnDLC(0), length);  // Frame length

  for (int i = 0; i < length; i++) {  // Write the frame data
    CAN_reg_write(REG_TXBnD0(0) + i, data[i]);
  }

  CAN_reg_write(REG_TXBnCTRL(0), 0x08);              // Start sending
  busy_wait_us(1000);                                // Allow up to 1ms to transmit
  CAN_reg_write(REG_TXBnCTRL(0), 0);                 // Stop sending
  CAN_reg_modify(REG_CANINTF, FLAG_TXnIF(0), 0x00);  // Clear interrupt flag
}


void CAN_send_errors(t_error_flags error_flags){
  // combining flags
  uint8_t first_byte = 0x00;
  first_byte |= error_flags.warning_AnyWarning << 0;
  first_byte |= error_flags.error_AnyError << 1;
  // warnings flags
  uint8_t second_byte = 0x00;
  second_byte |= error_flags.warning_SlaveTimeout << 0;
  second_byte |= error_flags.warning_UV << 1;
  second_byte |= error_flags.warning_OV << 2;
  second_byte |= error_flags.warning_UT << 3;
  second_byte |= error_flags.warning_OT << 4;
  second_byte |= error_flags.warning_OC << 5;
  // error flags
  uint8_t third_byte = 0x00;
  third_byte |= error_flags.error_SlaveTimeout << 0;
  third_byte |= error_flags.error_UV << 1;
  third_byte |= error_flags.error_OV << 2;
  third_byte |= error_flags.error_UT << 3;
  third_byte |= error_flags.error_OT << 4;
  third_byte |= error_flags.error_OC << 5;

  transmit(0x4ef, (uint8_t[]){first_byte, second_byte, third_byte}, 3);
}

void CAN_send_pack_voltage(float overall_voltage_mV, float current_A){
    uint32_t uint32_overall_voltage_mV = (uint32_t)overall_voltage_mV;
    int32_t int32_overall_current_mA = (int32_t)(current_A * 1000);
    transmit(0x4f0, (uint8_t[]){
      uint32_overall_voltage_mV >> 24,
      uint32_overall_voltage_mV >> 16,
      uint32_overall_voltage_mV >> 8,
      uint32_overall_voltage_mV,
      int32_overall_current_mA >> 24,
      int32_overall_current_mA >> 16,
      int32_overall_current_mA >> 8,
      int32_overall_current_mA},
       8);
}

void CAN_send_min_max_values(float min_voltage_mV, float max_voltage_mV, float min_temperature_C, float max_temperature_C){
    uint16_t uint16_min_voltage_mV = (uint16_t)min_voltage_mV;
    uint16_t uint16_max_voltage_mV = (uint16_t)max_voltage_mV;
    uint16_t uint16_min_temperature_C = (uint16_t)min_temperature_C * DBC_TEMPERATURE_FACTOR;
    uint16_t uint16_max_temperature_C = (uint16_t)max_temperature_C * DBC_TEMPERATURE_FACTOR;
    transmit(0x4f1, (uint8_t[]){
        uint16_max_voltage_mV >> 8, uint16_max_voltage_mV,
        uint16_min_voltage_mV >> 8, uint16_min_voltage_mV,
        uint16_max_temperature_C >> 8, uint16_max_temperature_C,
        uint16_min_temperature_C >> 8, uint16_min_temperature_C},
       8);
}

void CAN_send_individual_cell_voltage(uint8_t chain, uint8_t slave, uint8_t cell, float voltage_mV){
    uint16_t uint16_voltage_mV = (uint16_t)voltage_mV;
    transmit(0x4f2, (uint8_t[]){chain, slave, cell, uint16_voltage_mV >> 8, uint16_voltage_mV}, 5);
}

void CAN_send_individual_temperature_battery(uint8_t chain, uint8_t slave, uint8_t sensor_number, float temperature_C){
    uint16_t uint16_temperature_C = (uint16_t)temperature_C * DBC_TEMPERATURE_FACTOR;
    transmit(0x4f3, (uint8_t[]){chain, slave, sensor_number, uint16_temperature_C >> 8, uint16_temperature_C}, 5);
}

void CAN_send_individual_temperature_pcb(uint8_t chain, uint8_t slave, uint8_t sensor_number, float temperature_C){
    uint16_t uint16_temperature_C = (uint16_t)temperature_C * DBC_TEMPERATURE_FACTOR;
    transmit(0x4f4, (uint8_t[]){chain, slave, sensor_number, uint16_temperature_C >> 8, uint16_temperature_C}, 5);
}
