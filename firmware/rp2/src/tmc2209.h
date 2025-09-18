#ifndef TMC2209_H
#define TMC2209_H

#include <stdint.h>
#include "hardware/uart.h"

#define MOTOR_RA 0
#define MOTOR_DEC 1

typedef struct {
    uart_inst_t *uart;
    uint tx_pin;
    uint rx_pin;
    uint baudrate;
} tmc2209_config_t;

int tmc2209_init(tmc2209_config_t *config);
int tmc2209_write_register(uart_inst_t *uart, uint8_t node_addr, uint8_t reg_addr, uint32_t data);
int tmc2209_read_register(uart_inst_t *uart, uint8_t node_addr, uint8_t reg_addr, uint32_t *data);

#endif // TMC2209_H