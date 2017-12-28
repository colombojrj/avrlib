#ifndef AVRLIB_DRIVERS_CIS_SHIFTREGISTERS_H_
#define AVRLIB_DRIVERS_CIS_SHIFTREGISTERS_H_

#include "../avrlib/avrlib.h"

class ShiftRegister74HC595
{
private:
    DigitalPin *sdi, *shcp, *stcp;

    // Optional pins
    DigitalPin *oe, *mr;

    // Value stored in the 74LS595
    uint8_t actualValue = 0;

    // Available pins
    uint8_t hasOE = 0, hasMR = 0;

public:
    ShiftRegister74HC595(DigitalPin *serialDataInput,
                         DigitalPin *shiftRegisterClock,
                         DigitalPin *storageRegisterClock,
                         DigitalPin *outputEnable = nullptr,
                         DigitalPin *masterReset = nullptr);

    // Write functions
    void write(uint8_t value);
    void setPin(uint8_t pin);
    void clearPin(uint8_t pin);
    void write(uint8_t pin, uint8_t value);

    // Other general tasks
    void enableOutput();
    void disableOutput();
    void enableHighImpedance();
    void disableHighImpedance();

    void reset();
};

#endif /* AVRLIB_DRIVERS_CIS_SHIFTREGISTERS_H_ */
