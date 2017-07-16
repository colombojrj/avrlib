# avrlib

What this library is not?
 - A new innovative language.

What is there in this library?
 - Helper functions to work with ATmega micro-controllers.

With the purpose of trying to be the most generic possible, the code was divided in two parts:
 - low level drivers (timers.h, adc.h, etc)
 - high level API (gpio.h)

The low level drivers have the job of initialize the hardware. For example, if you want use TIMER1 as a PWM generator, you must configure TIMER1 configuration definitions (TIMER1_CONFIG as PWM, TIMER1_CLOCK, and so on). Only then you will be able to use the gpio library to instantiate a gpio PWM pin.

The key idea here is do not get in your way. You are free to configure the hardware as you require. You have access to all peripherals. There is nothing going on automatically. If you do not configure, then it does not work. There is no hidden TIMER working against you. The only purpose of this library is to allow faster development, by configuring the hardware automatically.

I recommend you include these libraries in your project as a submodule.
