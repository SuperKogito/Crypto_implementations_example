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
 * TI_SHA_256.c
 *
 *  Created on: March 13, 2012
 *      Author: Jace Hall
 *
 *  Description: Implementation of the SHA-256 as defined by the FIPS PUB 180-3: 
 *  the official SHA-256 standard
 */

/*===================================================================
// NAME:  void SHA_256 ( uint32_t *Message, uint64_t Mbit_Length, uint32_t *Hash);
//
// BRIEF: Is used to execute SHA-256 hashing algorithm on the data referenced by Message.
//        *Hash will contain the final hashing at completion of function.
//
// INPUTS:       uint32_t *Message -- Pointer to array of 32-bit long to be hashed.
// 						   Size of array must be a multiple of a hashing block.
// 						   ( I.e. 512 bits or sixteen 32-bit longs)
//		         uint64_t Mbit_Length --  64-bit value containing the precise number of
//					      bits to be hashed within Message[].
//
// Note: If Mbit_Length %(mod) 512 is >= 448 bits, then an additional hashing block is needed.
// User must allocate the additional 512 bits.
//
//		uint32_t *Hash	--	pointer to hashing array. Final hash will be stored here.
//							size of array should equal 8 32-bit longs
//
//		short  mode	--	If Mode =='0', SHA-224 is used, all else SHA-256
//
// OUTPUTS:      results stored at given pointers. Final Hash stored at Hash pointer.
//
// CHANGE:
// DATE 		WHO	Detail
// 13March2012    JH	Original Code
// 26March2012    JH	Comments added. 
//						Pre-processing halfway working
// 13April2012	  JH	Pre-processing working
// 08May2012	  JH	Mode added for SHA-224 along with intial hash values for SHA-224
// 11June2012	  JH	SHA algorithm tessted against NIST test vectors. Pass.
// 09July2012	  JH	Copyright added along with additional comments. Changed file names.
//==================================================================*/


/* This code being developed to implement SHA-244/256 on the MSP430.
* This code is by no means  optimized as of this moment.
* The object is to develop an understandable implementation of SHA-2 on the MSP430
* The algorithm will be used as a function call with inputs being a pointer to the message
* needing encryption, the length of the message in longs and a pointer to the 
* Hash (size of 8 longs) array in which will contain the answer after the function is done.
*/

//#include <msp430.h>
#include "TI_SHA_256.h"

/***************** SHA-XYZ INITIAL HASH VALUES AND CONSTANTS **********************/
/* Hash constant words K for SHA-256: */
static const uint32_t K256[64] = { 0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
								   0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
								   0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
								   0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
								   0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
								   0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
								   0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
								   0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
								   0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
								   0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
								   0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
								   0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
								   0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
								   0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
								   0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
								   0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2 };

/* Initial hash value H for SHA-256: */
static const uint32_t Initial_Hash[8] = { 0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
										  0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19 };

/*Function*/
void SHA_256 ( uint32_t *Message, uint64_t Mbit_Length, uint32_t *Hash, short mode){

/*Variable Declarations go here*/
unsigned int leftoverlong = 0;
unsigned int leftoverbits = 0;
unsigned char ChunkOffset = 0;
 uint32_t a=0;
 uint32_t b=0;
 uint32_t c=0;
 uint32_t d=0;
 uint32_t e=0;
 uint32_t f=0;
 uint32_t g=0;
 uint32_t h=0;
 uint64_t Nblocks = 0;
 uint32_t temp1=0;
 uint32_t temp2=0;
 uint32_t W[64] = {0};  //for Speed opt.
unsigned int i= 0;
unsigned int p =0;
unsigned int v =0;
unsigned int t =0;
uint64_t M_Length;
uint32_t mask = 0;
 //uint32_t W[16] = {0};   // For Code Size opt.  */ //With IAR Optimizations, this method does not improve codesize


/* Pre-processing: */
/* Initialize Hash Values 2. Parse the Message block 3. Padd the message block */
 for (i=0;i<=7; i++){
	 Hash[i] = Initial_Hash[i];     // Initialize Hash for SHA-256
 }
 i=0;  								//clear i
 
 /* Message Parsing */
 M_Length = Mbit_Length >> 5;       // Converting Bit length of message to How many longs in a message

 Nblocks = M_Length >> 4;           // Number of whole buckets (512 bits or 16 32-bit buckets)
 leftoverlong = M_Length % 16;      // leftover longs not in a full bucket
 leftoverbits = Mbit_Length % 32;   // leftover bits in last long

 /* Message Padding */
 /*The next set of statements finds the end of a message, appends a 1, then adds 0's
 * to pad the message to a 512bit chunk. The length of the original message is parsed
 * into the last 2 bytes */
 
// Check to see if last 32bit long is full of data
 if (leftoverbits == 0){
	 Message[lastchunk + leftoverlong ] = 0x80000000; // Message[lastchunk + leftoverlong + 1] = 0x80000000;
 }
 else{
   	 // Last long is not full
	 mask = (0xFFFFFFFF >> (leftoverbits));
 	 if(leftoverlong == 0){
 		 Message[lastchunk ] = (Message[lastchunk ] | mask) & (~mask >> 1); // append one to last bit
 	 }
 	 else{
 		 Message[lastchunk + leftoverlong] = (Message[lastchunk + leftoverlong ] | mask) & (~mask >>1); // append one to last bit
 	 }
 }

// Check to see if a new block (chunk) is needed
if ((Mbit_Length % 512) < 448){
	// No new chunk needed
	for(v=1; v < (14-leftoverlong); v++){
			Message[lastchunk + leftoverlong + v] &= 0x00000000; // zero pad
		}
		Message[lastchunk + 14]= Mbit_Length >> 32;   //append bit length to end of chunk
			Message[lastchunk + 15] = Mbit_Length & 0x00000000ffffFFFF;
			ChunkOffset =0; 
}
else{
  	// New chunk needed
	for (p=1; p < (16-leftoverlong); p++){ 
		Message[lastchunk +leftoverlong +p] = 0x00000000; // zero out remaining bits in chunk
	}
	for (p=0; p <14; p++){
		Message[lastchunk + 16 + p] = 0x00000000;         //zero out next chunk
	}
	Message[lastchunk + 30]= Mbit_Length >> 32;           //append bit length to end of chunk
	Message[lastchunk + 31] = Mbit_Length & 0x0000FFFF;
	ChunkOffset = 1; 
}
/** End Pre-Processing  **/
 
/** Main algorithm  **/
/* Chunk control. Process 512 bits at a time*/
 do{
	/*Place i-1 Hash into letters. Initialize with initial hash values.*/
	a = Hash[0];
	b = Hash[1];
	c = Hash[2];
	d = Hash[3];
	e = Hash[4];
	f = Hash[5];
	g = Hash[6];
	h = Hash[7];

	for (t=0; t < 64; t++){
		// W calculation. Can optimize for speed or code size (speed opt. Assume W[64])
		if(t >= 16) {
			W[t] = sigma1(W[(t-2)]) + W[(t-7)] + sigmaZ(W[(t-15)]) + W[(t-16)]; //000000000000000000000000000000
		}
		else {
			W[t] = Message[ 16*i + t ];
		}
		// Algorithm Proper
		temp1 = h + SIG1(e) + Ch(e, f, g) + K256[t] + W[t];     // for speed opt.
		temp2 = Maj(a, b, c) +SIGZ(a) ;
		
		h=g;
		g=f;
		f=e;
		e=d +temp1;
		d=c;
		c=b;
		b=a;
		a= temp1 + temp2;
	}
	Hash[0] = Hash[0] + a;
	Hash[1] = Hash[1] + b;
	Hash[2] = Hash[2] + c;
	Hash[3] = Hash[3] + d;
	Hash[4] = Hash[4] + e;
	Hash[5] = Hash[5] + f;
	Hash[6] = Hash[6] + g;
	Hash[7] = Hash[7] + h;

	i++;
 }while( i <= (Nblocks + ChunkOffset) ); 
}
