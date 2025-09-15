/*
 * xm125.h
 *
 *  Created on: 30 de jun. de 2025
 *      Author: gabri
 */

#ifndef MAIN_XM125_H_
#define MAIN_XM125_H_

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c.h"
#include <stdio.h>
#include "esp_log.h"
#include "distance_reg_protocol.h"
#include "presence_reg_protocol.h"
#define NMAX_MED 10
#define DIST_MIN 200
#define DIST_MAX 1000
#define LED_RADAR_D 46
#define LED_RADAR_P 14
// Use 1000ms timeout
#define I2C_TIMEOUT_MS 1000
// O STM32 usa o endereço i2c deslocado uma posição.
// à esquerda (0x52 se torna 0xa4)
#define I2C_ADDR_D 0x52
#define I2C_ADDR_P 0x51
// O comprimento do endereço do registro é de dois bytes
#define REG_ADDRESS_LENGTH 2
// O comprimento dos dados do registro é de quatro bytes
#define REG_DATA_LENGTH 4


bool detectar_obj(uint32_t value);
uint32_t media_dist(uint32_t value);
uint32_t media_peak(uint32_t value);
uint32_t lineariza(uint32_t value);
uint32_t lineariza_peak(uint32_t value);

bool read_register(int led,int addr_modo ,uint16_t reg_addr, uint32_t *reg_data);
bool write_register(int led,int addr_modo ,uint16_t reg_addr, uint32_t reg_data);
bool write_register1(uint16_t reg_addr, uint32_t reg_data);
bool configuration_ok(void);
bool configuration_ok_p(void);
bool wait_not_busy(void);
bool wait_not_busy_p(void);
bool example_setup_and_measure(void);
bool example_setup_and_start(void);
#endif /* MAIN_XM125_H_ */
