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

#ifndef __OS_H__
#define __OS_H__

/******************************************************************************
 * Includes
 *****************************************************************************/
#ifdef WIN32
   #include <stdio.h>
   #include <stdarg.h>
   #include <winsock.h>
   #define SHUT_RDWR   2
   typedef int socklen_t;
#else
   #include <errno.h>
   #include <sys/socket.h>
   #include <netinet/in.h>
   #include <arpa/inet.h>

   typedef int SOCKET;
   #define closesocket close
   int WSAGetLastError(void);
#endif

/******************************************************************************
 * Defines
 *****************************************************************************/
#ifndef __OS_FUNDAMENTAL_TYPES__
#define __OS_FUNDAMENTAL_TYPES__
  #undef    TRUE
  #define   TRUE              1
  #undef    FALSE
  #define   FALSE             0
  #undef    NULL
  #define   NULL              0

  #ifndef WIN32
  typedef unsigned char       BOOL;
  #endif

  typedef signed char         int8;
  typedef unsigned char       uint8;
  typedef unsigned char       byte;
  typedef signed short        int16;
  typedef unsigned short      uint16;
  typedef unsigned short      word;
  typedef signed long         int32;
  typedef unsigned long       uint32;
  typedef unsigned long       dword;
  typedef signed long long    int64;
  typedef unsigned long long  uint64;

  #define OS_SEEK_CUR    1
  #define OS_SEEK_END    2
  #define OS_SEEK_SET    0

#endif

#define osFile_tScanf fscanf

/******************************************************************************
 * Typedefs
 *****************************************************************************/
typedef void  osMutex_t;
typedef void  osSemaphore_t;
typedef void  osFile_t;
typedef void  osThread_t;

/******************************************************************************
 * Globals
 *****************************************************************************/
/******************************************************************************
 * Functions
 *****************************************************************************/
void*          OS_Malloc(int32 size);
void*          OS_Realloc(void* pData, int32 size);
void           OS_Free(void* pData);

osMutex_t*     OS_MutexCreate(void);
void           OS_MutexDelete(osMutex_t* pMutex);
void           OS_MutexLock(osMutex_t* pMutex);
void           OS_MutexUnlock(osMutex_t* pMutex);

osSemaphore_t* OS_SemaphoreCreate(void);
int32          OS_SemaphoreWait(osSemaphore_t* pSemaphore, uint32 ms);
int32          OS_SemaphoreSignal(osSemaphore_t* pSemaphore);
void           OS_SemaphoreDelete(osSemaphore_t* pSemaphore);

void           OS_Sleep(uint32 ms);

osFile_t*      OS_FileOpen(const char* pcFilename, const char* pcMode);
int32          OS_FileClose(osFile_t* pFile);
int32          OS_FileRead(void* pData, int32 size, osFile_t* pFile);
int32          OS_FileWrite(const void* pData, int32 size, osFile_t* pFile);
int32          OS_FileSeek(osFile_t* pFile, int32 offset, int32 origin);
int32          OS_FileTell(osFile_t* pFile);

osThread_t*    OS_ThreadCreate(int32 (*pstFunction)(void*), void* pParam,
                              int32 stackSize);
int32          OS_ThreadDelete(osThread_t* pThread);


#endif /* __OS_H__ */
