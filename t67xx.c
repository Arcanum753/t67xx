/*
 * t67xx.c
 *
 *  Created on: 20 сент. 2021 г.
 *      Author: sam
 */
#include "stdio.h"
#include "main.h"
#include "usart.h"
#include "usart_user.h"
#include "i2c.h"
#include "t67xx.h"
#include "EERTOS.h"
#include "main_sensors.h"
//I2C_HandleTypeDef 		i2c_instance ;
uint16_t 				data_ppm = 0;
uint16_t 				T6703_GetPPM(void)	{	return data_ppm;	}


I2C_HandleTypeDef* hi2c_t6703;
uint8_t t6703_addr = 0;
void T6703_i2cInit (I2C_HandleTypeDef* _hi2c_t6703, uint8_t _addr)	{
	t6703_addr = _addr;
	hi2c_t6703 = _hi2c_t6703;
	T6703_i2cReset();
	T6703_ABCEnable();
}

//uint16_t 				t6703_addr = t6703_addr;
HAL_StatusTypeDef T6703_i2cReset (void){
	HAL_StatusTypeDef status = HAL_TIMEOUT;
	uint8_t data[5] = T67XX_REG_RESET;
	status = HAL_I2C_Master_Transmit(hi2c_t6703, t6703_addr, data, sizeof(data), I2C_TIMEOUT);
	return status;
}

HAL_StatusTypeDef T6703_ABCEnable (void){
	HAL_StatusTypeDef status = HAL_TIMEOUT;
	uint8_t data[5] = T67XX_REG_ABCLOGIC_EN;
	status = HAL_I2C_Master_Transmit(hi2c_t6703, t6703_addr, data, sizeof(data), I2C_TIMEOUT);
	return status;
}

HAL_StatusTypeDef T6703_ABCDisable (void){
	HAL_StatusTypeDef status = HAL_TIMEOUT;
	uint8_t data[5] = T67XX_REG_ABCLOGIC_DI;
	status = HAL_I2C_Master_Transmit(hi2c_t6703, t6703_addr, data, sizeof(data), I2C_TIMEOUT);
	return status;
}


HAL_StatusTypeDef T6703_i2cGetFirmVer_req (void) {
	HAL_StatusTypeDef status = HAL_TIMEOUT;
	uint8_t data[5] = T67XX_REG_FWREV;
	status = HAL_I2C_Master_Transmit(hi2c_t6703, t6703_addr, data, sizeof(data), I2C_TIMEOUT);
//	HAL_Delay(T67XX_READ_DELAY);
	uint8_t t6703Data[4];
	status = HAL_I2C_Master_Receive(hi2c_t6703, t6703_addr, (uint8_t*) &t6703Data, sizeof(t6703Data), I2C_TIMEOUT);
//	PRINTF_UART("f %X b %X m %X l %X ", t6703Data[0], t6703Data[1], t6703Data[2], t6703Data[3]);
	return status;
}


int8_t T6703_i2cGetStatus (void) {
	HAL_StatusTypeDef status = HAL_TIMEOUT;
	uint8_t data[4] = T67XX_REG_STATUS;
	status = HAL_I2C_Master_Transmit(hi2c_t6703, t6703_addr, data, sizeof(data), I2C_TIMEOUT);
	HAL_Delay(T67XX_READ_DELAY);
	uint8_t t6703Data[4];
	status = HAL_I2C_Master_Receive(hi2c_t6703, t6703_addr, (uint8_t*) &t6703Data, sizeof(t6703Data), I2C_TIMEOUT);
//	PRINTF_UART("f %X b %X m %X l %X ", t6703Data[0], t6703Data[1], t6703Data[2], t6703Data[3]);
	if (t6703Data[2] != 0) { return -1; }
	if (t6703Data[3] != 0) { return -1; }
	return status;
}


void T6703_StartMeas(void){
	HAL_StatusTypeDef status = HAL_TIMEOUT;
	uint8_t data[5] = T67XX_REG_GASPPM;
	status = HAL_I2C_Master_Transmit(hi2c_t6703, t6703_addr, data, sizeof(data), I2C_TIMEOUT);
	HAL_Delay(T67XX_MEASURE_DELAY);
	T6703_StopMeas();
//	SetTimerTask(T6703_StopMeas, T67XX_MEASURE_DELAY);

}

void T6703_StopMeas(void){
	uint8_t t6703Data[4];
	HAL_StatusTypeDef status = HAL_TIMEOUT;
	status = HAL_I2C_Master_Receive(hi2c_t6703, t6703_addr, (uint8_t*) &t6703Data, sizeof(t6703Data), I2C_TIMEOUT);
	data_ppm = t6703Data[2] * 256 + t6703Data[3];
	T6703_i2cGetPPM();
}


uint16_t T6703_i2cGetPPM (void) {
	SensDataRaw.t6703_CO2 = data_ppm;
	DisplayOutput.eCO2_t6703 = SensDataRaw.t6703_CO2;
	return data_ppm;
}



