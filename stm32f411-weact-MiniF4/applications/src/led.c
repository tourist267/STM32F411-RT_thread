#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "led.h"


static rt_timer_t timer1;

static void led(void *parameter)
{
	rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);
	rt_pin_write(LED_PIN, !rt_pin_read(LED_PIN));
	rt_kprintf("test\n");
}
int led_thread(void)
{
	/*creat a timer thread */
	 timer1 = rt_timer_create("ledChange", led,
                             RT_NULL, 1000,
                             RT_TIMER_FLAG_PERIODIC);

    /* start timer thread */
    if (timer1 != RT_NULL) rt_timer_start(timer1);

		return 0;
}
MSH_CMD_EXPORT(led_thread, led thread);