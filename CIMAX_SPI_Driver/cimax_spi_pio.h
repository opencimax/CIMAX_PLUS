/*
Copyright (c) <2012>, <TiViSu>
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted 
provided that the following conditions are met:

	- Redistributions of source code must retain the above copyright notice, this list of conditions 
	  and the following disclaimer.
	- Redistributions in binary form must reproduce the above copyright notice, this list of conditions 
	  and the following disclaimer in the documentation and/or other materials provided with the 
	  distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __CIMAX_SPI_PIO_H__
#define __CIMAX_SPI_PIO_H__

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "os.h"

/******************************************************************************
 * Defines
 *****************************************************************************/
#define CIMAX_SPI_MAX_BUFF_LEN      512

/* Supported SPI modes	*/
#define CIMAX_SPI_MODE_0            0
#define CIMAX_SPI_MODE_1            1
#define CIMAX_SPI_MODE_2            2
#define CIMAX_SPI_MODE_3            3

/* Supported SPI clocks	*/
#define CIMAX_SPI_CLOCK_1MHZ        1*1000000
#define CIMAX_SPI_CLOCK_2MHZ        2*CIMAX_SPI_CLOCK_1MHZ
#define CIMAX_SPI_CLOCK_3MHZ        3*CIMAX_SPI_CLOCK_1MHZ
#define CIMAX_SPI_CLOCK_4MHZ        4*CIMAX_SPI_CLOCK_1MHZ
#define CIMAX_SPI_CLOCK_5MHZ        5*CIMAX_SPI_CLOCK_1MHZ
#define CIMAX_SPI_CLOCK_6MHZ        6*CIMAX_SPI_CLOCK_1MHZ
#define CIMAX_SPI_CLOCK_7MHZ        7*CIMAX_SPI_CLOCK_1MHZ
#define CIMAX_SPI_CLOCK_8MHZ        8*CIMAX_SPI_CLOCK_1MHZ
#define CIMAX_SPI_CLOCK_9MHZ        9*CIMAX_SPI_CLOCK_1MHZ
#define CIMAX_SPI_CLOCK_10MHZ       10*CIMAX_SPI_CLOCK_1MHZ
#define CIMAX_SPI_CLOCK_11MHZ       11*CIMAX_SPI_CLOCK_1MHZ
#define CIMAX_SPI_CLOCK_12MHZ       12*CIMAX_SPI_CLOCK_1MHZ

/******************************************************************************
 * Typedefs
 *****************************************************************************/

typedef struct {
   uint8    mode;
   uint32   clock;
} spiInitParams_t;

/******************************************************************************
 * Functions
 *****************************************************************************/
int32 CIMAX_SPI_Init (spiInitParams_t *pSpiInitParams);
int32 CIMAX_SPI_Write (uint8 *buff, uint32 numOfBytes);
int32 CIMAX_SPI_Read (uint8 *buff, uint32 numOfBytes);
int32 CIMAX_SPI_Term ();

int32 CIMAX_PIO_Init ();
int32 CIMAX_PIO_Reset ();
int32 CIMAX_PIO_GetIntState (uint8 *pState);
int32 CIMAX_PIO_Term ();


#endif   /* __CIMAX_SPI_PIO_H__  */