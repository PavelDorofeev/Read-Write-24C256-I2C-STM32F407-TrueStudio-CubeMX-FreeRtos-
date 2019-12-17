#ifndef __IIC_FUNC_H
#define __IIC_FUNC_H

#include "main.h"

#define  PIN_RS   (1 << 0)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)

#define LCD_DELAY_MS 5

#define I2C_SLAVE_ADDR   0x00 // 0x7C //0x3E

#define TEMP_OUT_INT_REGISTER   0x0
#define TEMP_OUT_FRAC_REGISTER  0x1
#define WHO_AM_I_REGISTER       0xF
#define WHO_AM_I_VALUE          0xBC
#define TRANSFER_DIR_WRITE      0x1
#define TRANSFER_DIR_READ       0x0

//#define I2C_PORT		hi2c3

void writeDR(uint8_t val);
void writeIR(uint8_t val);
void I2C1_DataTransfer(uint8_t *aTxBuffer,int TXBUFFERSIZE);
uint8_t readI2C();
void rst_LCD();
uint8_t ReadBusyFlagAndAddress();
void search_I2C_devices();

void I2C_Scan(char *str,I2C_HandleTypeDef *pH);
void LCD_Init(I2C_HandleTypeDef *h,uint8_t contrast);
HAL_StatusTypeDef LCD_SendInternal(uint8_t data,uint8_t flags) ;
HAL_StatusTypeDef LCD_SendCommand(uint8_t cmd);
void LCD_SendString(char *str);
void LCD_SendData(uint8_t data);
uint8_t revert(uint8_t data);
char * error_HAL(HAL_StatusTypeDef res);
void Display_ON(uint8_t On);
void Function_Set_On(uint8_t On,uint8_t ExtMode);
HAL_StatusTypeDef LCDI2C_write4bits(uint8_t cmd,uint8_t data);
void Contrast_set(uint8_t val); // 1..16
void Internal_OSC_Frequency(uint8_t bias,uint8_t freq); //1..8
void Power_ICON_Contrast();
void Follower_control();
void Return_Home();
/*void SCL(uint8_t vv);
void SDA(uint8_t vv);
void send(uint8_t val);
void body(uint8_t val);
void start();
void stop();*/
#define ST7032_INST_DDRAM (uint8_t)(1 << 7)

#define BIAS1_4 (uint8_t)(1 << 3)
#define BIAS1_5 (uint8_t)(0 << 3)

#endif

