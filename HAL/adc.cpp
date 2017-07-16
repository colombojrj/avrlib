/*
  adc.c - Biblioteca para o controle do conversor A/D

  Copyright (c) 2013 - José Roberto Colombo Junior (tickbrown@gmail.com)

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

#include "adc.h"

/*
 * Adicione as seguintes linhas de código ao
 * seu arquivo principal main.c caso o modo
 * de operação seja o FREE_RUNNING

	volatile uint16_t valor_adc;
	ISR(ADC_vect) {
		valor_adc = ADC;
	}

 * No caso de utilizar o modo com redução de
 * ruído, simplesmente adicione o vetor de
 * interrupção na main, somente para que o
 * programa não faça nada e não bugue. O código
 * a ser inserido na main é o mostrado abaixo:

   ISR(ADC_vect) {}

 */

/*************************************************************************
Função:     init_adc ()
Propósito:  inicia o conversor A/D
Retorna:    nada
**************************************************************************/
void INIT_ADC(uint8_t pin) {
	// Enable the A/D converter
	ADCSRA = (1 << ADEN) | PREESCALE;
	ADMUX = ADC_REFERENCE | DATA_ALIGN;

	// Disable input digital buffer (save power)
	#if defined (__AVR_ATmega328P__)
		DIDR0 = (1 << pin);
	#endif

	// TODO: add support to the free running mode
}

/**************************************************************************
Function:     ANALOG_READ(uint8_t pin)
Purpose:      Perform an analog to digital conversion. This function blocks
              the CPU
Return:       Converted value (int16_t)
**************************************************************************/
uint16_t ANALOG_READ(uint8_t pin) {
	uint8_t backup_ADMUX = ADMUX;

	// Select the pin
	ADMUX = ADMUX | pin;

	#if OPERATION_MODE == SINGLE
		ADCSRA |= (1 << ADSC);
		while (ADCSRA & (1 << ADSC)) {};
		ADMUX = backup_ADMUX;
		return ADC;

	#elif OPERATION_MODE == NOISE_REDUCTION
		// Gera uma interrupção quando a conversão termina
		ADCSRA |= _BV(ADIE);
		set_sleep_mode(SLEEP_MODE_ADC);
		sleep_enable();

		do {
			// Garante que as interrupções estão ativadas
			sei();
			// Põe a CPU em modo de economia de energia
			sleep_cpu();
			// Desativa as interrupções
			cli();
		}
		while (ADCSRA & (1<<ADSC));

		// Desativa o sleep
		sleep_disable();

		// Liga as interrupções
		sei();

		// Desativa interrupção por ADC
		ADCSRA &= ~ _BV(ADIE);
		return(ADC);

	#else
		return 0;
	#endif
}

/*************************************************************************
Função:     CHANGE_ADMUX ()
Propósito:  Trocar o ADMUX quando o conversor opera em modo FREE RUNNING
Retorna:    Nada
**************************************************************************/
void CHANGE_ADMUX (uint8_t pino) {
	ADMUX &= 0b11110000; // Zera os 4 registradores MUX
	if (pino != 0) ADMUX |= (pino & 0b00001111); // Apenas os 4 bits menos significativos (0 ~ 15)
}

void SET_REFERENCE_VOLTAGE(uint8_t source)
{
	ADMUX = source;
}
