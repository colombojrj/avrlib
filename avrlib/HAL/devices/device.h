#ifndef AVRLIB_AVRLIB_HAL_DEVICES_DEVICE_H_
#define AVRLIB_AVRLIB_HAL_DEVICES_DEVICE_H_

#if defined (__AVR_ATmega88__)   || (__AVR_ATmega88P__)   || \
            (__AVR_ATmega168__)  || (__AVR_ATmega168P__)  || \
            (__AVR_ATmega328__)  || (__AVR_ATmega328P__)
    #include "atmega328p.h"

#elif defined (__AVR_ATtiny13__)   || (__AVR_ATtiny13A__)
#elif defined (__AVR_ATtiny24__)   || (__AVR_ATtiny24A__)  || \
              (__AVR_ATtiny44__)   || (__AVR_ATtiny44A__)  || \
              (__AVR_ATtiny84__)   || (__AVR_ATtiny84A__)
#endif

#endif /* AVRLIB_AVRLIB_HAL_DEVICES_DEVICE_H_ */
