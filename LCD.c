#include "stm32f0xx_hal.h"

void lcd_send_cmd (char cmd, I2C_HandleTypeDef hi2c1)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = cmd&0xf0;
	data_l = (cmd<<4)&0xf0;
	data_t[0] = data_u|0x04;  //en=1, rs=0
	data_t[1] = data_u;  //en=0, rs=0
	data_t[2] = data_l|0x04;  //en=1, rs=0
	data_t[3] = data_l;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, 0x4E,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data, I2C_HandleTypeDef hi2c1)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = data&0xf0;
	data_l = (data<<4)&0xf0;
	data_t[0] = data_u|0x05;  //en=1, rs=0
	data_t[1] = data_u|0x01;  //en=0, rs=0
	data_t[2] = data_l|0x05;  //en=1, rs=0
	data_t[3] = data_l|0x01;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, 0x4E,(uint8_t *) data_t, 4, 100);
}

void lcd_init (I2C_HandleTypeDef hi2c1)
{
	lcd_send_cmd (0x02, hi2c1);
	lcd_send_cmd (0x28, hi2c1);
	lcd_send_cmd (0x0c, hi2c1);
	lcd_send_cmd (0x80, hi2c1);
}

void lcd_send_string (char *str, I2C_HandleTypeDef hi2c1)
{
	while (*str) lcd_send_data (*str++, hi2c1);
}

void lcd_print_data(char data, I2C_HandleTypeDef hi2c1){
	uint8_t data_t[1];
	char data_u;
	data_u = data&0xf0;
	data_t[0] = data_u|0x05;
	HAL_I2C_Master_Transmit (&hi2c1, 0x4E,(uint8_t *) data_t, 1, 100);
}
