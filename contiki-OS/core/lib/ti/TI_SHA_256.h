/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

/*
 * sha.h
 *
 *  Created on: Mar 13, 2012
 *      Author: a0273957
 */

#ifndef TI_SHA_256_H_
#define TI_SHA_256_H_

#include <stdint.h>

#define lastchunk 16*Nblocks

/*** SHA Function Macros ***/
#define SR(x,a)          (x >> a)                         /* Shift  right Function - Shift  x by a  */
#define ROTR(x,n)        (( x >> n ) | ( x << (32 - n ))) /* Rotate right Function - rotate x by n */
#define ROTR25(x)		 (((x >> 9) >> 16) | ( x << 7 ))

/*** Algorithm defined logical functions ***/
#define Ch(x , y, z)    ((x & y) ^ (~x & z))
#define Maj(x, y, z)    ((x & y) ^ (x & z) ^ ( y & z))

/*Alternate Ch and MAJ functions that could be faster (With IAR optimizations, this method is not faster)*/
#define SIGZ(x)         (ROTR(x,2) ^ ROTR(x,13) ^ ROTR(x,22))
#define SIG1(x)         (ROTR(x,6) ^ ROTR(x,11) ^ ROTR(x,25))   //(ROTR(x,6)  ^ ROTR(x,11) ^ ROTR25(x))
#define sigmaZ(x)       (ROTR(x,7) ^ ROTR(x,18) ^ SR(x,3))
#define sigma1(x)       (ROTR(x,17) ^ ROTR(x,19) ^ SR(x,10))

extern void SHA_256( uint32_t *Message, uint64_t Mbit_Length, uint32_t *Hash, short mode);

#endif /* SHA_H_ */
