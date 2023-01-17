#ifndef CAN_H
#define CAN_H

#include "hardware/clocks.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

// Define pins for SPI (to CAN)
#define SPI_PORT spi0
#define SPI_MISO 16
#define SPI_CS 17
#define SPI_CLK 18
#define SPI_MOSI 19
#define CAN_INT 20    // Interrupt from CAN controller
#define CAN_CLK 21    // 8MHz clock for CAN
#define CAN_SLEEP 22  // Shut down CAN transceiver

#define REG_BFPCTRL                0x0c
#define REG_TXRTSCTRL              0x0d

#define REG_CANCTRL                0x0f

#define REG_CNF3                   0x28
#define REG_CNF2                   0x29
#define REG_CNF1                   0x2a

#define REG_CANINTE                0x2b
#define REG_CANINTF                0x2c

#define FLAG_RXnIE(n)              (0x01 << n)
#define FLAG_RXnIF(n)              (0x01 << n)
#define FLAG_TXnIF(n)              (0x04 << n)

#define REG_RXFnSIDH(n)            (0x00 + (n * 4))
#define REG_RXFnSIDL(n)            (0x01 + (n * 4))
#define REG_RXFnEID8(n)            (0x02 + (n * 4))
#define REG_RXFnEID0(n)            (0x03 + (n * 4))

#define REG_RXMnSIDH(n)            (0x20 + (n * 0x04))
#define REG_RXMnSIDL(n)            (0x21 + (n * 0x04))
#define REG_RXMnEID8(n)            (0x22 + (n * 0x04))
#define REG_RXMnEID0(n)            (0x23 + (n * 0x04))

#define REG_TXBnCTRL(n)            (0x30 + (n * 0x10))
#define REG_TXBnSIDH(n)            (0x31 + (n * 0x10))
#define REG_TXBnSIDL(n)            (0x32 + (n * 0x10))
#define REG_TXBnEID8(n)            (0x33 + (n * 0x10))
#define REG_TXBnEID0(n)            (0x34 + (n * 0x10))
#define REG_TXBnDLC(n)             (0x35 + (n * 0x10))
#define REG_TXBnD0(n)              (0x36 + (n * 0x10))

#define REG_RXBnCTRL(n)            (0x60 + (n * 0x10))
#define REG_RXBnSIDH(n)            (0x61 + (n * 0x10))
#define REG_RXBnSIDL(n)            (0x62 + (n * 0x10))
#define REG_RXBnEID8(n)            (0x63 + (n * 0x10))
#define REG_RXBnEID0(n)            (0x64 + (n * 0x10))
#define REG_RXBnDLC(n)             (0x65 + (n * 0x10))
#define REG_RXBnD0(n)              (0x66 + (n * 0x10))

#define FLAG_IDE                   0x08
#define FLAG_SRR                   0x10
#define FLAG_RTR                   0x40
#define FLAG_EXIDE                 0x08

#define FLAG_RXM0                  0x20
#define FLAG_RXM1                  0x40

#define CMD_RESET                  0xC0
#define CMD_WRITE                  0x02
#define CMD_READ                   0x03
#define CMD_MODIFY                 0x05

#define MODE_NORMAL                0x00
#define MODE_SLEEP                 0x01

void CAN_init();

void CAN_reset();

uint8_t CAN_reg_read(uint8_t reg);

void CAN_reg_write(uint8_t reg, uint8_t val);

void CAN_reg_modify(uint8_t reg, uint8_t mask, uint8_t val);

void CAN_configure(uint16_t id);

void CAN_send_pack_voltage(float pack_voltage_mV);

void CAN_send_min_max_values(float min_voltage_mV, float max_voltage_mV, float min_temperature_C, float max_temperature_C);

void CAN_send_individual_cell_voltage(uint8_t chain, uint8_t slave, uint8_t cell, float voltage_mV);

#endif
