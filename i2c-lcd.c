/*
 * i2c-lcd.c
 *
 *  Created on: Apr 17, 2023
 *      Author: dhine
 */
//#include "main.h"
#include "i2c-lcd.h"

#define SLAVE_ADDRESS_LCD 0x4E // change this according to your setup
extern I2C_HandleTypeDef hi2c1;     // I2C handler declaration

void lcd_send_cmd(char cmd) {
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xf0);
    data_l = ((cmd << 4) & 0xf0);
    data_t[0] = data_u | 0x0C; // en=1, rs=0
    data_t[1] = data_u | 0x08; // en=0, rs=0
    data_t[2] = data_l | 0x0C; // en=1, rs=0
    data_t[3] = data_l | 0x08; // en=0, rs=0
    HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, (uint8_t*) data_t, 4, 100);
}

void lcd_send_data(char data) {
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xf0);
    data_l = ((data << 4) & 0xf0);
    data_t[0] = data_u | 0x0D; // en=1, rs=0
    data_t[1] = data_u | 0x09; // en=0, rs=0
    data_t[2] = data_l | 0x0D; // en=1, rs=0
    data_t[3] = data_l | 0x09; // en=0, rs=0
    HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, (uint8_t*) data_t, 4, 100);
}

void lcd_clear(void) {
    lcd_send_cmd(0x01);
    HAL_Delay(2); // Wait for 2 ms for the LCD to clear
}

void lcd_init(void) {
    lcd_send_cmd(0x02);
    lcd_send_cmd(0x28);
    lcd_send_cmd(0x0c);
    lcd_send_cmd(0x80);
}

void lcd_send_string(char* str) {
    while (*str) lcd_send_data(*str++);
}

void lcd_SetCursor(uint8_t row, uint8_t column){
    uint8_t address;
    switch (row) {
        case 0: address = 0x80 + column; break;
        case 1: address = 0xc0 + column; break;
        case 2: address = 0x94 + column; break;
        case 3: address = 0xd4 + column; break;
        default: address = 0x80 + column;
    }
    lcd_send_cmd(address);
}

void lcd_WriteString(uint8_t row, uint8_t column, char* str){
    lcd_SetCursor(row, column);
    lcd_send_string(str);
}

//int main(void) {
    /* MCU Configuration */
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  //  HAL_Init();

    /* Initialize all configured peripherals */
    //MX_I2C1_Init();

    //lcd_init(); // initialize the LCD

    /* Infinite loop */
   // while (1) {
     //   lcd_send_string("Hello, World!"); // print the message
     //   HAL_Delay(1000); // delay for 1 second
     //   lcd_clear(); // clear the LCD
     //   HAL_Delay(1000); // delay for 1 second
    //}
//}



