avrlib
======

avrlib is a library to make my projects easier. The main goal is to support all ATmega328P peripherals. With a single configuration file, the designer is free to configure the peripherals of the microcontroller. If it is not used, then it is not initialized (to save power).

This library is **not** recommended for beginners.

## Why use it? ##
When building complex or professional projects, it is necessary to have control of all microcontroller hardware: from TIMERs to I/O pins. This is the main goal of this library: to provide easy low level functions to configure peripherals.

Suppose that you wanna build a solar tracker. You probably will need to configure:

 - A/D converter
 - TIMERs (as clock source and maybe PWM)
 - SPI or UART for communication

With avrlib, all you need is to configure config.h. Set A/D mode, A/D clock, TIMER0 as CTC and so on. Obviously you need to know about your microcontroller. But, isn't you a professional?

How is this library organized?
------------------------------

With the purpose of trying to be the most generic possible, the code was divided in two parts:

> - A HAL layer (i.e., low level drivers for gpio, TIMER, A/D, etc)
> - A high level API (there is support for PWM, A/D and gpio pins)

The low level drivers have the job of initialize the hardware. The high level API has the purpose of making programming very fun.

The key idea here is do not get in your way. You are free to configure the hardware as you require. You have access to all peripherals. There is nothing going on automatically. If you do not configure, then it does not work. There is no hidden TIMER working against you. The only purpose of this library is to allow faster development, by configuring the hardware automatically.

Example: blink a LED connected to pin PD5
-----------------------------------------
```
#include <avrlib.h>

gpio led(&PORTD, PD5, OUTPUT, LOW);

int main(void)
{
	while (1)
	{
		led.toggle();
		_delay_ms(1000);
	}
	return 0;
}

```
