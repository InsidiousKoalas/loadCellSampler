/*
 * thFunks.h - Temperature and Humidity sensor Functions
 *
 *  Created on: May 29, 2019
 *      Author: bhunt
 *
 */

#ifndef THFUNKS_H_
#define THFUNKS_H_

#define	 DATA	0x80


extern volatile char thBuffer[5], restFlag, sampNdx;

void thInit();
void thStart();
int thRead();


#endif /* THFUNKS_H_ */
