#include <board.h>
#define DBG_SECTION_NAME    "LCD"
#define DBG_COLOR
#define DBG_LEVEL           DBG_LOG
#include "oledfont.h"
#include <rtdbg.h>
#include "drv_spi.h"
#include "lcd.h"
static struct rt_spi_device *spi_dev_lcd;
/*****************************
* LCD SPI configuration 
*******************************/
static int rt_hw_lcd_config(void)
{
	spi_dev_lcd = (struct rt_spi_device *)rt_device_find("spi10");
	 /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode =/*RT_SPI_CPOL |*/ RT_SPI_MASTER | RT_SPI_MODE_2 | RT_SPI_MSB | RT_SPI_NO_CS;
        cfg.max_hz = 42 * 1000 * 1000; /* 42M,SPI max 42MHz,lcd 4-wire spi */

        rt_spi_configure(spi_dev_lcd, &cfg);
    }

    return RT_EOK;
}

/*****************************
* LCD IO pin  initialize
*******************************/
static void lcd_gpio_init(void)
{
//    rt_hw_lcd_config();

	rt_pin_mode(LCD_DC_PIN, PIN_MODE_OUTPUT);
	rt_pin_mode(LCD_RES_PIN, PIN_MODE_OUTPUT);
	rt_pin_mode(LCD_BLK_PIN, PIN_MODE_OUTPUT);
	
	rt_pin_write(LCD_RES_PIN, PIN_HIGH);
	rt_pin_write(LCD_DC_PIN, PIN_HIGH);
	rt_pin_write(LCD_BLK_PIN, PIN_HIGH);
	

//    rt_pin_mode(LCD_PWR_PIN, PIN_MODE_OUTPUT);
//    rt_pin_write(LCD_PWR_PIN, PIN_LOW);

    rt_pin_write(LCD_RES_PIN, PIN_LOW);
    //wait at least 100ms for reset
    rt_thread_delay(100);
    rt_pin_write(LCD_RES_PIN, PIN_HIGH);
		rt_thread_delay(100);
		rt_pin_write(LCD_BLK_PIN, PIN_HIGH);
}
/*****************************
*write  command to the LCD 
*******************************/
static rt_err_t lcd_write_cmd(const rt_uint8_t cmd)
{
    rt_size_t len;

    rt_pin_write(LCD_DC_PIN, PIN_LOW);

    len = rt_spi_send(spi_dev_lcd, &cmd, 1);

    if (len != 1)
    {
        LOG_I("lcd_write_cmd error. %d", len);
        return -RT_ERROR;
    }
    else
    {
        return RT_EOK;
    }
}
/*****************************
*write  data to the LCD 
*******************************/
static rt_err_t lcd_write_data8(const rt_uint8_t data)
{
    rt_size_t len;

    rt_pin_write(LCD_DC_PIN, PIN_HIGH);

    len = rt_spi_send(spi_dev_lcd, &data, 1);

    if (len != 1)
    {
        LOG_I("lcd_write_data error. %d", len);
        return -RT_ERROR;
    }
    else
    {
        return RT_EOK;
    }
}


/*****************************
*write  data to the LCD 
*******************************/
static rt_err_t lcd_write_data16(const rt_uint16_t data)
{
    rt_size_t len;
		rt_uint8_t	highData;
		highData=data>>8;
    rt_pin_write(LCD_DC_PIN, PIN_HIGH);
	
		len = rt_spi_send(spi_dev_lcd, &highData, 1);
    len = rt_spi_send(spi_dev_lcd, &data, 1);

    if (len != 1)
    {
        LOG_I("lcd_write_data error. %d", len);
        return -RT_ERROR;
    }
    else
    {
        return RT_EOK;
    }
}
/*****************************
* LCD  initialize
*******************************/
int rt_hw_lcd_init(void)
{
	/*creat a spi drvice and attach to the bus*/
	rt_hw_spi_device_attach("spi1", "spi10", GPIOB, GPIO_PIN_6);
	/*spi drvice configuration*/
	rt_hw_lcd_config();
	/*lcd GPIO initialize*/
	lcd_gpio_init();
	
    /* Memory Data Access Control */
    lcd_write_cmd(0x36);
    if(USE_HORIZONTAL==0)lcd_write_data8(0x00);
		else if(USE_HORIZONTAL==1)lcd_write_data8(0xC0);
		else if(USE_HORIZONTAL==2)lcd_write_data8(0x70);
		else lcd_write_data8(0xA0);
    /* RGB 5-6-5-bit  */
    lcd_write_cmd(0x3A);
    lcd_write_data8(0x05);
    /* Porch Setting */
    lcd_write_cmd(0xB2);
    lcd_write_data8(0x0C);
    lcd_write_data8(0x0C);
    lcd_write_data8(0x00);
    lcd_write_data8(0x33);
    lcd_write_data8(0x33);
    /*  Gate Control */
    lcd_write_cmd(0xB7);
    lcd_write_data8(0x35);
    /* VCOM Setting */
    lcd_write_cmd(0xBB);
    lcd_write_data8(0x19);
    /* LCM Control */
    lcd_write_cmd(0xC0);
    lcd_write_data8(0x2C);
    /* VDV and VRH Command Enable */
    lcd_write_cmd(0xC2);
    lcd_write_data8(0x01);
    /* VRH Set */
    lcd_write_cmd(0xC3);
    lcd_write_data8(0x12);
    /* VDV Set */
    lcd_write_cmd(0xC4);
    lcd_write_data8(0x20);
    /* Frame Rate Control in Normal Mode */
    lcd_write_cmd(0xC6);
    lcd_write_data8(0x0F);
    /* Power Control 1 */
    lcd_write_cmd(0xD0);
    lcd_write_data8(0xA4);
    lcd_write_data8(0xA1);
    /* Positive Voltage Gamma Control */
    lcd_write_cmd(0xE0);
    lcd_write_data8(0xD0);
    lcd_write_data8(0x04);
    lcd_write_data8(0x0D);
    lcd_write_data8(0x11);
    lcd_write_data8(0x13);
    lcd_write_data8(0x2B);
    lcd_write_data8(0x3F);
    lcd_write_data8(0x54);
    lcd_write_data8(0x4C);
    lcd_write_data8(0x18);
    lcd_write_data8(0x0D);
    lcd_write_data8(0x0B);
    lcd_write_data8(0x1F);
    lcd_write_data8(0x23);
    /* Negative Voltage Gamma Control */
    lcd_write_cmd(0xE1);
    lcd_write_data8(0xD0);
    lcd_write_data8(0x04);
    lcd_write_data8(0x0C);
    lcd_write_data8(0x11);
    lcd_write_data8(0x13);
    lcd_write_data8(0x2C);
    lcd_write_data8(0x3F);
    lcd_write_data8(0x44);
    lcd_write_data8(0x51);
    lcd_write_data8(0x2F);
    lcd_write_data8(0x1F);
    lcd_write_data8(0x1F);
    lcd_write_data8(0x20);
    lcd_write_data8(0x23);
    /* Display Inversion On */
    lcd_write_cmd(0x21);
    /* Sleep Out */
    lcd_write_cmd(0x11);
    /* wait for power stability */
//    rt_thread_mdelay(100);

//    lcd_clear(WHITE);

    /* display on */
//    rt_pin_write(LCD_PWR_PIN, PIN_HIGH);
    lcd_write_cmd(0x29);
	
	
	return RT_EOK;
	
}
//INIT_DEVICE_EXPORT(rt_hw_lcd_init);
/*****************************
* Set the display range
*******************************/
void LCD_Address_Set(rt_uint16_t x1,rt_uint16_t y1,rt_uint16_t x2,rt_uint16_t y2)
{
	if(USE_HORIZONTAL==0)
	{
		lcd_write_cmd(0x2a);//列地址设置
		lcd_write_data16(x1);
		lcd_write_data16(x2);
		lcd_write_cmd(0x2b);//行地址设置
		lcd_write_data16(y1);
		lcd_write_data16(y2);
		lcd_write_cmd(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==1)
	{
		lcd_write_cmd(0x2a);//列地址设置
		lcd_write_data16(x1);
		lcd_write_data16(x2);
		lcd_write_cmd(0x2b);//行地址设置
		lcd_write_data16(y1+80);
		lcd_write_data16(y2+80);
		lcd_write_cmd(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==2)
	{
		lcd_write_cmd(0x2a);//列地址设置
		lcd_write_data16(x1);
		lcd_write_data16(x2);
		lcd_write_cmd(0x2b);//行地址设置
		lcd_write_data16(y1);
		lcd_write_data16(y2);
		lcd_write_cmd(0x2c);//储存器写
	}
	else
	{
		lcd_write_cmd(0x2a);//列地址设置
		lcd_write_data16(x1+80);
		lcd_write_data16(x2+80);
		lcd_write_cmd(0x2b);//行地址设置
		lcd_write_data16(y1);
		lcd_write_data16(y2);
		lcd_write_cmd(0x2c);//储存器写
	}
}
/*****************************
* clear the screen
*******************************/
void LCD_Clear(rt_uint16_t Color)
{
	rt_uint16_t i,j;  	
	LCD_Address_Set(0,0,LCD_W-1,LCD_H-1);
    for(i=0;i<LCD_W;i++)
	 {
	  for (j=0;j<LCD_H;j++)
	   	{
        	lcd_write_data16(Color);	 			 
	    }
	  }
}
/*****************************
* display a picture
*******************************/
extern const unsigned char gImage_picture[59858];
void write_picture(void)
{
	rt_size_t leng=0;
	rt_uint16_t pic;

	rt_pin_write(LCD_DC_PIN,PIN_HIGH);//写数据
	for(pic=0;pic<59858;pic++)
	{
		leng = rt_spi_send(spi_dev_lcd, &gImage_picture[pic], 1);
	}
}

/******************************************************************************
      函数说明：在指定区域填充颜色
      入口数据：xsta,ysta   起始坐标
                xend,yend   终止坐标
      返回值：  无
******************************************************************************/
void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color)
{          
	u16 i,j; 
	LCD_Address_Set(xsta,ysta,xend,yend);      //设置光标位置 
	for(i=ysta;i<=yend;i++)
	{													   	 	
		for(j=xsta;j<=xend;j++)lcd_write_data16(color);//设置光标位置 	    
	} 					  	    
}

/******************************************************************************
      函数说明：LCD画一个大的点
      入口数据：x,y   起始坐标
      返回值：  无
******************************************************************************/
void LCD_DrawPoint_big(rt_uint16_t x,rt_uint16_t y,rt_uint16_t color)
{
	LCD_Fill(x-1,y-1,x+1,y+1,color);
} 



