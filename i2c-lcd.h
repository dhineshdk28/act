/*
 * i2c-lcd.h
 *
 *  Created on: Apr 17, 2023
 *      Author: dhine
 */

#ifndef INC_I2C_LCD_H_
#define INC_I2C_LCD_H_

#include "stm32g4xx_hal.h"


void lcd_init (void);   // initialize lcd

void lcd_send_cmd (char cmd);  // send command to the lcd

void lcd_send_data (char data);  // send data to the lcd

void lcd_send_string (char *str);  // send string to the lcd

void lcd_clear (void); // Clear LCD

void lcd_SetCursor(uint8_t row, uint8_t column);  // To set the cursor in the required position

void lcd_WriteString(uint8_t row, uint8_t column, char* str);  // To write the string in the LCD


#endif /* INC_I2C_LCD_H_ */
