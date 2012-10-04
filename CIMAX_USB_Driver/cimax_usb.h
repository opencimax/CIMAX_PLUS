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

#ifndef __CIMAX_USB_H__
#define __CIMAX_USB_H__

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "os.h"

/******************************************************************************
 * Defines
 *****************************************************************************/
#define CIMAX_USB_BOOT_MODE   0
#define CIMAX_USB_FW_MODE     1


/******************************************************************************
 * Typedefs
 *****************************************************************************/

/******************************************************************************
 * Functions
 *****************************************************************************/
int32 CIMAX_USB_Init(uint8 mode);
int32 CIMAX_USB_BootMessage(uint8 requestType, uint8 request, uint16 value, 
                           uint16 index, uint8 *buff, uint16 len);
int32 CIMAX_USB_WriteAltSet2(uint8 *buff, uint16 numOfBytes);
int32 CIMAX_USB_ReadAltSet2(uint8 *buff);//, uint16 numOfBytes);
void  CIMAX_USB_Close();

#endif   /* __CIMAX_USB_H__  */