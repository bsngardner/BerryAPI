/*
 * pins.h
 *
 *  Created on: Jun 20, 2016
 *      Author: Broderick
 */

#ifndef PINS_H_
#define PINS_H_

/*
 * Berry27a-vibrerry
 * Rev E:
 *
 * 	OUT		| P1.0	AVCC | -
 * 	-		| P1.1	AVSS | -
 * 	-		| P1.2	PJ.5 | -
 * 	-		| P1.3	PJ.4 | -
 * 	-		| P1.4	DVCC | -
 * 	-		| P1.5	DVSS | -
 * 	-		| PJ.0	VCORE| -
 * 	-		| PJ.1	P1.7 | BSCL
 * 	-		| PJ.2	P1.6 | BSDA
 * 	-		| PJ.3	P2.2 | BINT
 * 	-		| TEST	P2.1 | -
 * 	-		| RESET	P2.0 | -
 *
 */

//Port 1
#define BSDA BIT6
#define BSCL BIT7

//Port 2
#define BINT BIT2

#endif /* PINS_H_ */
