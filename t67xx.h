/*
 * t67xx.h
 *
 *  Created on: 20 сент. 2021 г.
 *      Author: sam
 */

#ifndef T67XX_H_
#define T67XX_H_

// S.A.
#define I2C_INSTANCE 			I2C_5V
#define T67XX_I2C_ADDR 			(0x15 << 1)

/* Modbus registers */
#define T67XX_REG_FWREV           {0x04, 0x13, 0x89, 0x00, 0x01}//0x1389  /* Firmware revision */
#define T67XX_REG_STATUS          {0x04, 0x13, 0x8a, 0x00}//0x138a  /* Status */
#define T67XX_REG_GASPPM          {0x04, 0x13, 0x8b, 0x00, 0x01}//0x138b  /* Gas parts per million */
#define T67XX_REG_RESET           {0x05, 0x03, 0xe8, 0xFF, 0x00}//0x03e8  /* Reset device */
#define T67XX_REG_SPCAL           0x03ec  /* Single point calibration */
#define T67XX_REG_SLAVEADDR       0x0fa5  /* Slave address */
#define T67XX_REG_ABCLOGIC_EN     {0x05, 0x03, 0xee, 0xFF, 0x00}//  0x03ee  /* ABC Logic enable/disable */
#define T67XX_REG_ABCLOGIC_DI     {0x05, 0x03, 0xee, 0x00, 0x00}//0x03ee  /* ABC Logic enable/disable */
/* Status register bits */
#define T67XX_STATUS_ERROR        (1 << 0)  /* Error condition */
#define T67XX_STATUS_FLASH_ERROR  (1 << 1)  /* Flash error */
#define T67XX_STATUS_CALIB_ERROR  (1 << 2)  /* Calibration error */
#define T67XX_STATUS_RS232        (1 << 8)  /* RS-232 error */
#define T67XX_STATUS_RS485        (1 << 9)  /* RS-485 error */
#define T67XX_STATUS_I2C          (1 << 10) /* I2C error */
#define T67XX_STATUS_WARMUP       (1 << 11) /* Warm-up mode */
#define T67XX_STATUS_SPCAL        (1 << 15) /* Single point calibration */

/* Command bits */
#define RESET_SENSOR              0xff00    /* Reset sensor */
#define SPCAL_START               0xff00    /* Start SP calibration */
#define SPCAL_STOP                0x0000    /* Stop SP calibration */
#define ABCLOGIC_ENABLE           0xff00    /* Enable ABC Logic */
#define ABCLOGIC_DISABLE          0x0000    /* Disable ABC Logic */

#define T67XX_READ_DELAY 		10
#define T67XX_MEASURE_DELAY 	2250 // Recommended value



//typedef struct	T6703Data{
//	uint8_t		fCode;
//	uint8_t		bCount;
//	uint8_t		msb;
//	uint8_t		lsb;
//} sT6703Data;


void T6703_i2cInit (I2C_HandleTypeDef* _hi2c_t6703, uint8_t _addr);

HAL_StatusTypeDef T6703_i2cReset (void)				;
HAL_StatusTypeDef T6703_ABCEnable (void)			;
HAL_StatusTypeDef T6703_ABCDisable (void)			;
HAL_StatusTypeDef T6703_i2cGetFirmVer_req (void)	;

int8_t T6703_i2cGetStatus (void) ;

uint16_t T6703_i2cGetPPM (void)	;
HAL_StatusTypeDef T6703_i2cGetPPM_rsp (void)	;
uint16_t T6703_GetPPM(void);
void T6703_StartMeas(void);
void T6703_StopMeas(void);

#endif /* T67XX_H_ */
