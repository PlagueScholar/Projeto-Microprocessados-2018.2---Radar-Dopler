#include "stm32f0xx_hal.h"

#ifndef LCD_h
#define LCD_h

void lcd_send_cmd (char cmd, I2C_HandleTypeDef hi2c1);
void lcd_send_data (char data, I2C_HandleTypeDef hi2c1);
void lcd_init (I2C_HandleTypeDef hi2c1);
void lcd_send_string (char *str, I2C_HandleTypeDef hi2c1);
void lcd_print_data(uint8_t data, I2C_HandleTypeDef hi2c1);

#endif

