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

/* defined the LED0 pin: PC13 */
#define LED0_PIN               GET_PIN(C, 13)

int main(void)
{

led_thread();
    while (1)
    {
  
    }
}
