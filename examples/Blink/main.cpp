#include <avrlib.h>

DigitalPin led(&PORTB, PB5, OUTPUT);

int main(void)
{
    initHAL();

	while(true)
	{
		led.toggle();
		_delay_ms(1000);
	}

	return 0;
}




