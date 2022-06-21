 #ifndef __LCD_H__
#define __LCD_H__

#include <rtthread.h>
#include <rtdevice.h>

#define LCD_DC_PIN 6//A6
#define LCD_RES_PIN 16 //B0
#define LCD_BLK_PIN 28

#define LCD_W 240
#define LCD_H 240

//画笔颜色
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE           	 0x001F  
#define BRED             0XF81F
#define GRED 			       0XFFE0
#define GBLUE			       0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			     0XBC40 //棕色
#define BRRED 			     0XFC07 //棕红色
#define GRAY  			     0X8430 //灰色
//GUI颜色

#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色  
#define GRAYBLUE       	 0X5458 //灰蓝色
//以上三色为PANEL的颜色 
 
#define LIGHTGREEN     	 0X841F //浅绿色
#define LGRAY 			     0XC618 //浅灰色(PANNEL),窗体背景色

#define LGRAYBLUE        0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE           0X2B12 //浅棕蓝色(选择条目的反色)
#define u16 rt_uint16_t

extern rt_uint16_t BACK_COLOR;   //背景色

#define USE_HORIZONTAL 0 //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏

void LCD_Clear(rt_uint16_t Color);
int rt_hw_lcd_init(void);
void LCD_Address_Set(rt_uint16_t x1,rt_uint16_t y1,rt_uint16_t x2,rt_uint16_t y2);
void write_picture(void);
void LCD_DrawPoint_big(rt_uint16_t x,rt_uint16_t y,rt_uint16_t color);
#endif
