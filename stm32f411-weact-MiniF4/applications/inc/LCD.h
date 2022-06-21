 #ifndef __LCD_H__
#define __LCD_H__

#include <rtthread.h>
#include <rtdevice.h>

#define LCD_DC_PIN 6//A6
#define LCD_RES_PIN 16 //B0
#define LCD_BLK_PIN 28

#define LCD_W 240
#define LCD_H 240

//������ɫ
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
#define BROWN 			     0XBC40 //��ɫ
#define BRRED 			     0XFC07 //�غ�ɫ
#define GRAY  			     0X8430 //��ɫ
//GUI��ɫ

#define DARKBLUE      	 0X01CF	//����ɫ
#define LIGHTBLUE      	 0X7D7C	//ǳ��ɫ  
#define GRAYBLUE       	 0X5458 //����ɫ
//������ɫΪPANEL����ɫ 
 
#define LIGHTGREEN     	 0X841F //ǳ��ɫ
#define LGRAY 			     0XC618 //ǳ��ɫ(PANNEL),���屳��ɫ

#define LGRAYBLUE        0XA651 //ǳ����ɫ(�м����ɫ)
#define LBBLUE           0X2B12 //ǳ����ɫ(ѡ����Ŀ�ķ�ɫ)
#define u16 rt_uint16_t

extern rt_uint16_t BACK_COLOR;   //����ɫ

#define USE_HORIZONTAL 0 //���ú�������������ʾ 0��1Ϊ���� 2��3Ϊ����

void LCD_Clear(rt_uint16_t Color);
int rt_hw_lcd_init(void);
void LCD_Address_Set(rt_uint16_t x1,rt_uint16_t y1,rt_uint16_t x2,rt_uint16_t y2);
void write_picture(void);
void LCD_DrawPoint_big(rt_uint16_t x,rt_uint16_t y,rt_uint16_t color);
#endif
