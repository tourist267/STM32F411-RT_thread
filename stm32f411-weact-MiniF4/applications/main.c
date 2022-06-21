/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "led.h"
#include "lcd.h"

/* defined the LED0 pin: PC13 */
#define LED0_PIN               GET_PIN(C, 13)

int main(void)
{

	led_thread();
	rt_hw_lcd_init();
	while (1)
	{
		//		LCD_DrawPoint_big(50,50,RED);
		LCD_Clear(BLACK);
		LCD_Address_Set(33,33,205,205);
		write_picture();
		rt_thread_mdelay(2000);
		LCD_Clear(BLACK);
		LCD_Address_Set(33,33,205,205);
		write_picture();
		rt_thread_mdelay(2000);
	
	}
}
