/*
 * logic_operations.h
 *
 *  Created on: 21/06/2017
 *      Author: junior
 */

#ifndef AVRLIB_LOGIC_OPERATIONS_H_
#define AVRLIB_LOGIC_OPERATIONS_H_

#define	AND(__IN1, __IN2)	(__IN1 & __IN2)
#define	OR(__IN1, __IN2)	(__IN1 | __IN2)
#define	XOR(__IN1, __IN2)	((__IN1) ^ (__IN2))
#define	XNOR(__IN1, __IN2)	(~(__IN1 ^ __IN2))
#define NOT(__IN1)			(~__IN1)



#endif /* AVRLIB_LOGIC_OPERATIONS_H_ */
