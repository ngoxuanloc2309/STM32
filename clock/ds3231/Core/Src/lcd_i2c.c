
/**********************************************************

Author: Logan Ngo
Modify: Logan

***********************************************************/

#include "lcd_i2c.h"
#include "string.h"
#include "stdio.h"
extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly

#define device_id 0x27<<1

void lcd_i2c_send_cmd(uint8_t data){
	uint8_t buf[4] = {(data&0xf0)|0x0c , (data&0xf0)|0x08, (data<<4)|0x0c, (data<<4)|0x08};
	HAL_I2C_Master_Transmit(&hi2c1,device_id,buf,sizeof(buf), 10);
}
void lcd_init(){
	lcd_i2c_send_cmd(0x33);
	lcd_i2c_send_cmd(0x32);
	lcd_i2c_send_cmd(0x28);
	lcd_i2c_send_cmd(0x0c);
	lcd_i2c_send_cmd(0x06);
	lcd_i2c_send_cmd(0x01);
}
void lcd_i2c_send_data(uint8_t data){
	uint8_t buf[4] = {(data & 0xF0) | 0x0D, (data & 0xF0) | 0x09, (data << 4) | 0x0D, (data << 4) | 0x09};
	HAL_I2C_Master_Transmit(&hi2c1, device_id, buf, sizeof(buf), 10);
	HAL_Delay(2);
}
void lcd_i2c_clear(){
	lcd_i2c_send_cmd(0x01);
	HAL_Delay(2);
}
void lcd_i2c_set_cs(int x, int y){
	if(x==0){
		lcd_i2c_send_cmd(0x80 + y);
	}else
	if(x==1){
		lcd_i2c_send_cmd(0xC0 + y);
	}
}
void lcd_i2c_send_string(char *string){
	for(int i=0; i<strlen(string); i++){
		lcd_i2c_send_data(string[i]);
	}
}
void lcd_i2c_send_num(int num){
	char buff[8];
	sprintf(buff, "%d", num);
	lcd_i2c_send_string(buff);
}
void lcd_i2c_backlight_on() {
    uint8_t data = 0x08; // B?t backlight
    HAL_I2C_Master_Transmit(&hi2c1, device_id, &data, 1, 10);
}

void lcd_i2c_backlight_off() {
    uint8_t data = 0x00; // B?t backlight
    HAL_I2C_Master_Transmit(&hi2c1, device_id, &data, 1, 10);
}
