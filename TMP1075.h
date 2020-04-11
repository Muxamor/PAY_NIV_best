/*
 * TMP1075.h
 *
 *  Created on: Apr 7, 2020
 *      Author: ftor
 */

#ifndef INC_TMP1075_H_
#define INC_TMP1075_H_


#include "main.h"

#define SHUTDOWN_MODE            1
#define CONTINUOS_CONVERSION     0

#define VERY_FAST      		0  // 27.5 ms
#define FAST           		1  // 55 ms
#define MEDIUM         		2  // 110 ms
#define SLOW           		3  // 220 ms

#define ACTIVE_LOW 			0
#define ACTIVE_HIGH 		1

#define ONE_FAULT   		0
#define TWO_FAULT   		1
#define THREE_FAULT 		3
#define FOUR_FAULT  		4

int8_t I2C_Read_word_with_error(I2C_TypeDef *I2Cx, uint8_t reg_id, uint8_t SlaveAddr, uint16_t *data);
int8_t I2C_Write_word(I2C_TypeDef *I2Cx, uint8_t reg_id, uint8_t SlaveAddr, uint16_t data);

int8_t TMP1075_read_id(uint8_t tmp1075_addr, uint16_t *read_data);
int8_t TMP1075_read_raw_temperature(uint8_t tmp1075_addr, uint16_t *read_data);
float binary_to_float_temperature(uint16_t ADC_CODE);
int8_t binary_to_int_temperature(uint16_t ADC_CODE);
int16_t float_to_binary(float val);
int8_t TMP1075_read_configuration(uint8_t tmp1075_addr, uint16_t *read_data);
int8_t TMP1075_set_mode(uint8_t tmp1075_addr, uint8_t mode);
int8_t TMP1075_set_time_conversation(uint8_t tmp1075_addr, uint8_t time);
int8_t TMP1075_temperature_request(uint8_t tmp1075_addr);
int8_t TMP1075_set_function_of_the_ALERT_pin(uint8_t tmp1075_addr, uint8_t mode);
int8_t TMP1075_ALERT_active_level(uint8_t tmp1075_addr, uint8_t mode);
int8_t TMP1075_ALERT_sensitivity(uint8_t tmp1075_addr, uint8_t mode);
int8_t TMP1075_set_low_limit(uint8_t tmp1075_addr, float low_limit);
int8_t TMP1075_set_high_limit(uint8_t tmp1075_addr, float high_limit);
int8_t TMP1075_get_high_limit(uint8_t tmp1075_addr, uint16_t *read_data);
int8_t TMP1075_get_low_limit(uint8_t tmp1075_addr, uint16_t *read_data);


#endif /* INC_TMP1075_H_ */
