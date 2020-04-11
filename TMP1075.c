/*
 * TMP1075.c
 *
 *  Created on: Apr 7, 2020
 *      Author: ftor
 */

#include "TMP1075.h"


/*
 *  @brief	Reading 2 byte register.
 *
	@param 	*I2Cx - pointer to I2C controller, where x is a number (e.x., I2C1, I2C2 etc.).
	@param 	reg_id - register's address.
	@param 	SlaveAddr - 7-bit device address.

	@retval error status
 */
int8_t I2C_Read_word_with_error(I2C_TypeDef *I2Cx, uint8_t reg_id, uint8_t SlaveAddr, uint16_t *data){
    uint8_t big, little;
    int8_t error_status = 0;
    SlaveAddr = SlaveAddr << 1;

    while(LL_I2C_IsActiveFlag_BUSY(I2Cx) == SET);

    LL_I2C_HandleTransfer(I2Cx, SlaveAddr,LL_I2C_ADDRSLAVE_7BIT, 1,
                          LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);
    while(LL_I2C_IsActiveFlag_TXE(I2Cx) == RESET);

    LL_I2C_TransmitData8(I2Cx, reg_id);
    while(LL_I2C_IsActiveFlag_TXE(I2Cx) == RESET);
    while(LL_I2C_IsActiveFlag_TC(I2Cx) == RESET);

    LL_I2C_HandleTransfer(I2Cx, SlaveAddr,LL_I2C_ADDRSLAVE_7BIT, 2,
                          LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_RESTART_7BIT_READ);

    while(LL_I2C_IsActiveFlag_RXNE(I2Cx) == RESET);

    LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);
    big = LL_I2C_ReceiveData8(I2Cx);
    while(LL_I2C_IsActiveFlag_RXNE(I2Cx) == RESET);
    LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);
    little = LL_I2C_ReceiveData8(I2Cx);

    LL_I2C_GenerateStopCondition(I2Cx);
    while(LL_I2C_IsActiveFlag_STOP(I2Cx) == RESET);

    LL_I2C_ClearFlag_STOP(I2Cx);

    *data = big << 8 | little;
    return error_status;
}


/*
 *  @brief	Writing word to register.
 *
	@param *I2Cx - pointer to I2C controller, where x is a number (e.x., I2C1, I2C2 etc.).
	@param reg_id - register address.
	@param SlaveAddr - 7-bit device address.
	@param data - 2-byte data to write

	@retval	error status
 */
int8_t I2C_Write_word(I2C_TypeDef *I2Cx, uint8_t reg_id, uint8_t SlaveAddr, uint16_t data){
    uint8_t data_low = (data & 0xFF);
    uint8_t data_high = data >> 8;
    int8_t error_status = 0;
    SlaveAddr = SlaveAddr << 1;

    while(LL_I2C_IsActiveFlag_BUSY(I2Cx) == SET);
    LL_I2C_HandleTransfer(I2Cx, SlaveAddr,LL_I2C_ADDRSLAVE_7BIT, 3,
                          LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);

    while(LL_I2C_IsActiveFlag_TXE(I2Cx) == RESET);

    LL_I2C_TransmitData8(I2Cx, reg_id);
    while(LL_I2C_IsActiveFlag_TXE(I2Cx) == RESET);

    LL_I2C_TransmitData8(I2Cx, data_high);
    while(LL_I2C_IsActiveFlag_TXE(I2Cx) == RESET);

    LL_I2C_TransmitData8(I2Cx, data_low);
    while(LL_I2C_IsActiveFlag_TXE(I2Cx) == RESET);
    while(LL_I2C_IsActiveFlag_TC(I2Cx) == RESET);

    LL_I2C_GenerateStopCondition(I2Cx);
    while(LL_I2C_IsActiveFlag_STOP(I2Cx) == RESET);

    LL_I2C_ClearFlag_STOP(I2Cx);
    return error_status;
}


/*
    @brief	Reading device id.

	@param tmp1075_addr - 7-bit device address.

	@retval	error status
 */
int8_t TMP1075_read_id(uint8_t tmp1075_addr, uint16_t *read_data){
	int8_t err_status = I2C_Read_word_with_error(I2C3, 0x0F, tmp1075_addr, read_data);
	return err_status;
}


/*
    @brief	Reading raw data from temperature register.

	@param tmp1075_addr - 7-bit device address.

	@retval	error status
 */
int8_t TMP1075_read_raw_temperature(uint8_t tmp1075_addr, uint16_t *read_data){
	int8_t err_status = I2C_Read_word_with_error(I2C3, 0x00, tmp1075_addr, read_data);
	return err_status;
}


/*
    @brief	Converter from raw data (temperature register)  to float.

	@param tmp1075_addr - 7-bit device address.

	@retval	error status
 */
float binary_to_float_temperature(uint16_t ADC_CODE){
	if(ADC_CODE & (1 << 15)){
        return -(((~ADC_CODE) + 1) >> 8);
	}
    else{
    	ADC_CODE = ADC_CODE >> 4;
        return ADC_CODE * 0.0625;
    }
}


/*
    @brief	Converter from raw data (temperature register)  to int.

	@param tmp1075_addr - 7-bit device address.

	@retval	error status.
 */
int8_t binary_to_int_temperature(uint16_t ADC_CODE){
	ADC_CODE = ADC_CODE >> 4;
    if(ADC_CODE & (1 << 11))
        return -((((~ADC_CODE)+1) >> 1) + (((~ADC_CODE)+1) >> 3)) / 10;
    else
        return ((ADC_CODE >> 1) + (ADC_CODE >> 3)) / 10;
}


/*
    @brief	Converter from float to raw data temperature register.

	@param tmp1075_addr - 7-bit device address.

	@retval	error status.
 */
int16_t float_to_binary(float val){
    return val * 256;
}


/*
    @brief	Reading configuration register.

	@param tmp1075_addr - 7-bit device address.

	@retval	error status.
 */
int8_t TMP1075_read_configuration(uint8_t tmp1075_addr, uint16_t *read_data){
    int8_t err_status = I2C_Read_word_with_error(I2C3, 0x01, tmp1075_addr, read_data);
    return err_status;
}


/*
    @brief	Sets the device in SHUTDOWN_MODE to conserve power or in CONTINUOS_CONVERSION

	@param tmp1075_addr - 7-bit device address.
	@param mode - SHUTDOWN_MODE or CONTINUOS_CONVERSION mode of TMP1075

	@retval	error status
 */
int8_t TMP1075_set_mode(uint8_t tmp1075_addr, uint8_t mode){
    uint16_t last_state;
    TMP1075_read_configuration(tmp1075_addr, &last_state);
    I2C_Write_word(I2C3, 0x01, tmp1075_addr, (last_state & (~(1 << 8)))  | (mode << 8));

    uint16_t current_state;
    TMP1075_read_configuration(tmp1075_addr, &current_state);
    if((current_state & (~(1 << 8)))  | (mode << 8))
        return 0;
    else
        return -1;
}


/*
    @brief	Conversion rate setting when device is in continuos conversion mode

	@param tmp1075_addr - 7-bit device address.
	@param time - conversion rate
			#define VERY_FAST      0  // 27.5 ms
			#define FAST           1  // 55 ms
			#define MEDIUM         2  // 110 ms
			#define SLOW           3  // 220 ms
	@retval	error status
 */
int8_t TMP1075_set_time_conversation(uint8_t tmp1075_addr, uint8_t time){
    uint16_t last_state;
    TMP1075_read_configuration(tmp1075_addr, &last_state);
    I2C_Write_word(I2C3, 0x01, tmp1075_addr, (last_state & (~(3 << 13))) | (time << 13));

    uint16_t current_state;
	TMP1075_read_configuration(tmp1075_addr, &current_state);
    if(((last_state & (~(3 << 13))) | (time << 13)) == current_state)
        return 0;
    else
        return -1;
}


/*
    @brief	One-shot conversion mode (for SHUTDOWN_MODE). Requests single measurement of temperature.

	@param tmp1075_addr - 7-bit device address.

	@retval	error status
 */
int8_t TMP1075_temperature_request(uint8_t tmp1075_addr){
    uint16_t last_state;
    TMP1075_read_configuration(tmp1075_addr, &last_state);
    I2C_Write_word(I2C3, 0x01, tmp1075_addr, (last_state & (~(1 << 15))) | (1 << 15));

    uint16_t current_state;
	TMP1075_read_configuration(tmp1075_addr, &current_state);
    if(((last_state & (~(1 << 15))) | (1 << 15)) == current_state)
        return 0;
    else
        return -1;
}


/*
    @brief	Selects the function of the ALERT pin.

	@param tmp1075_addr - 7-bit device address.
	@param mode - ALERT mode.
			#define COMPARATOR_MODE 0
			#define INTERRUPT_MODE 1
	@retval	error status
 */
int8_t TMP1075_set_function_of_the_ALERT_pin(uint8_t tmp1075_addr, uint8_t mode){
    uint16_t last_state;
    TMP1075_read_configuration(tmp1075_addr, &last_state);
    I2C_Write_word(I2C3, 0x01, tmp1075_addr, (last_state & (~(1 << 9))) | (mode << 9));

    uint16_t current_state;
    TMP1075_read_configuration(tmp1075_addr, &current_state);
    if(((last_state & (~(1 << 9))) | (mode << 9)) == current_state)
        return 0;
    else
        return -1;
}


/*
    @brief	Polarity of the output ALERT pin.

	@param tmp1075_addr - 7-bit device address.
	@param mode - ALERT polarity.
			#define ACTIVE_LOW 0
			#define ACTIVE_HIGH 1
	@retval	error status.
 */
int8_t TMP1075_ALERT_active_level(uint8_t tmp1075_addr, uint8_t mode){
    uint16_t last_state;
    TMP1075_read_configuration(tmp1075_addr, &last_state);
    I2C_Write_word(I2C3, 0x01, tmp1075_addr, (last_state & (~(1 << 10))) | (mode << 10));

    uint16_t current_state;
    TMP1075_read_configuration(tmp1075_addr, &current_state);
    if(((last_state & (~(1 << 10))) | (mode << 10)) == current_state)
        return 0;
    else
        return -1;
}


/*
    @brief	Consecutive fault measurements to trigger the alert function.

	@param tmp1075_addr - 7-bit device address.
	@param mode - amount of faults
			#define ONE_FAULT   0
			#define TWO_FAULT   1
			#define THREE_FAULT 3
			#define FOUR_FAULT  4
	@retval	error status
 */
int8_t TMP1075_ALERT_sensitivity(uint8_t tmp1075_addr, uint8_t mode){
    uint16_t last_state;
    TMP1075_read_configuration(tmp1075_addr, &last_state);
    I2C_Write_word(I2C3, 0x01, tmp1075_addr, (last_state & (~(3 << 11))) | (mode << 11));
    uint16_t current_state;
    TMP1075_read_configuration(tmp1075_addr, &current_state);
    if(((last_state & (~(3 << 11))) | (mode << 11)) == current_state)
        return 0;
    else
        return -1;
}


/*
    @brief	Set low limit for comparison with temperature results.

	@param tmp1075_addr - 7-bit device address.
	@param low_limit -  low limit for comparison with temperature results.

	@retval	error status.
 */
int8_t TMP1075_set_low_limit(uint8_t tmp1075_addr, float low_limit){
    I2C_Write_word(I2C3, 0x02, tmp1075_addr, low_limit * 256);
    uint16_t read_data;
    TMP1075_get_low_limit(tmp1075_addr, &read_data);
    float current_low_lim = binary_to_float_temperature(read_data);
    if(abs(current_low_lim - low_limit) <= 0.1)
        return 0;
    else
        return -1;
}


/*
    @brief	 Set high limit for comparison with temperature results

	@param tmp1075_addr - 7-bit device address.
	@param low_limit - high limit for comparison with temperature results.

	@retval	error status.
 */
int8_t TMP1075_set_high_limit(uint8_t tmp1075_addr, float high_limit){
    I2C_Write_word(I2C3, 0x03, tmp1075_addr, high_limit * 256);
    uint16_t read_data;
    TMP1075_get_high_limit(tmp1075_addr, &read_data);
    float current_high_lim = binary_to_float_temperature(read_data);
    if(abs(current_high_lim - high_limit) <= 0.1)
        return 0;
    else
        return -1;
}


/*
    @brief	Reading current value of high limit register.

	@param tmp1075_addr - 7-bit device address.

	@retval	error status.
 */
int8_t TMP1075_get_high_limit(uint8_t tmp1075_addr, uint16_t *read_data){
    int8_t err_status = I2C_Read_word_with_error(I2C3, 0x03, tmp1075_addr, read_data);
    return err_status;
}


/*
    @brief	Reading current value of low limit register.

	@param tmp1075_addr - 7-bit device address.

	@retval	error status.
 */
int8_t TMP1075_get_low_limit(uint8_t tmp1075_addr, uint16_t *read_data){
	int8_t err_status =  I2C_Read_word_with_error(I2C3, 0x02, tmp1075_addr, read_data);
	return err_status;
}







