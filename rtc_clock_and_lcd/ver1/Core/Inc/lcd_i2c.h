/**********************************************************

Author: Logan Ngo
Modify: Logan

***********************************************************/

#include "stm32f1xx_hal.h"

void lcd_i2c_send_cmd(uint8_t data);
void lcd_init(void);
void lcd_i2c_send_data(uint8_t data);
void lcd_i2c_clear(void);
void lcd_i2c_set_cs(int x, int y);
void lcd_i2c_send_string(char *string);
void lcd_i2c_send_num(int num);
void lcd_i2c_backlight_on(void);
void lcd_i2c_backlight_off(void);
