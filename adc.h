/*
  adc.h - A/D converter library

  Copyright (c) 2012 - José Roberto Colombo Junior (colombojrj [at] gmail [dot] com)

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

#ifndef ADC_H_
#define ADC_H_


/*
 * Choose the A/D operation mode listed from below:
 *
 * SINGLE
 * NOISE_REDUCTION
 * FREE_RUNNING
 */
#define OPERATION_MODE SINGLE


/*
 * Defina qual como será gerada a tensão de
 * referência do circuito. Os possíveis valores são:
 *
 * EXTERNA
 * INTERNA
 * VCC
 */
#define ADC_REFERENCE REF_INTERN

/*
 * Define the reference voltage and the analog circuit vcc supply voltage
 */
#define ADC_REFERENCE_VOLTAGE	1.1
#define ADC_SUPPLY_VOLTAGE		5.0


/* É também preciso informar qual o fator de
 * divisão da frequência da CPU. Os possíveis
 * valores são:
 *
 * DIV_BY_2
 * DIV_BY_4
 * DIV_BY_8
 * DIV_BY_16
 * DIV_BY_32
 * DIV_BY_64
 * DIV_BY_128
 */
#define PREESCALE DIV_BY_32

/* Data alignment:
 *
 * LEFT
 * RIGHT
 */
#define DATA_ALIGN RIGHT

//
// Internal defines (do not change them!)
//
#define SINGLE				1
#define NOISE_REDUCTION		2
#define FREE_RUNNING		3
#define REF_EXTERN			0
#define REF_INTERN			((1 << REFS1) | (1 << REFS0))
#define REF_VCC				(1 << REFS0)
#define DIV_BY_2			1
#define DIV_BY_4			2
#define DIV_BY_8			3
#define DIV_BY_16			4
#define DIV_BY_32			5
#define DIV_BY_64			6
#define DIV_BY_128			7
#define LEFT				(1 << ADLAR)
#define RIGHT				0

// Available functions
extern void INIT_ADC(uint8_t pin);
extern uint16_t ANALOG_READ(uint8_t pin);
extern void CHANGE_ADMUX(uint8_t pin);
extern void SET_REFERENCE_VOLTAGE(uint8_t source);
#if defined (__AVR_ATmega328P__)
	extern int16_t READ_TEMPERATURE();
#endif

#endif /* ADC_H_ */
