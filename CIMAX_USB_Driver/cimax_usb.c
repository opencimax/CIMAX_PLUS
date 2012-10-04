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

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "usb.h"
#include "cimax_usb.h"


/******************************************************************************
 * Defines
 *****************************************************************************/
#define CIMAX_USB_NO_ERROR          0
#define CIMAX_USB_ERROR             -1

#define CIMAX_USB_VID               0x1b0d
#define CIMAX_USB_PID               0x2f00

#define CIMAX_CONFIGURATION         0

#define CIMAX_INTERFACE             0

#define CIMAX_ALT_IF_BOOT_MODE      0  /* alternate interface 0 is used in boot mode   */
                                       
#define CIMAX_ALT_IF_FW_MODE        2  /* alternate interface 2 is used in FW mode     */
                                       
#define CIMAX_ALT_IF                CIMAX_ALT_IF_CONTROL_ONLY

#define CIMAX_ALT_SET_2_EP_WRITE          0x06
#define CIMAX_ALT_SET_2_EP_READ           0x85
#define CIMAX_ALT_SET_2_MAX_PACKET_SIZE   256



/* Response timeout for CIMaX+ boot commands */
#define CIMAX_BOOT_RESPONSE_TIMEOUT_MS 100
#define CIMAX_FW_RESPONSE_TIMEOUT_MS   10000

/******************************************************************************
 * Globals
 *****************************************************************************/
static usb_dev_handle *handle = NULL;

/******************************************************************************
 * Static functions
 *****************************************************************************/
static struct usb_device   *seek_dev(struct usb_bus *bus, int32 vid, int32 pid, 
                                   usb_dev_handle **pHandle);
static int32               open_interface(usb_dev_handle *handle, int32 interface, 
                                          int32 alternate);                                   

/******************************************************************************
 * Functions
 *****************************************************************************/
int32 CIMAX_USB_Init(uint8 mode)
{
   struct usb_bus    *bus = NULL;
   struct usb_device *dev = NULL;
   
   usb_init();          /* libusb library initialization */
   usb_find_busses();   /* find all busses */
   usb_find_devices();  /* find all connected devices */
    
   bus = usb_get_busses();
   if (bus == NULL)
   {
      printf("%s: no USB busses found!\n", __FUNCTION__);
      return CIMAX_USB_ERROR;
   }   
   
   /* USB busses scan */
   do
   {
      if (dev = seek_dev(bus, CIMAX_USB_VID, CIMAX_USB_PID, &handle))
   		break;
      bus=bus->next;
   }while(bus);
 
   /* Check if device is found */
   if (bus == NULL)
   {
      printf("%s: CIMaX+ device not found!\n", __FUNCTION__);
      return CIMAX_USB_ERROR;
   }
   
   /* Check device handle */
   if (handle == NULL)
   {
      printf("%s: invalid device handle!\n", __FUNCTION__);
      return CIMAX_USB_ERROR;
   }
   
   /* Open interface */
   if (open_interface(handle, CIMAX_INTERFACE, (mode==CIMAX_USB_BOOT_MODE)?CIMAX_ALT_IF_BOOT_MODE:CIMAX_ALT_IF_FW_MODE) != CIMAX_USB_NO_ERROR)
   {
      printf("%s: opening interface failed!\n", __FUNCTION__);
      usb_close(handle );
      return CIMAX_USB_ERROR;
   }
   
   return CIMAX_USB_NO_ERROR;
}

int32 CIMAX_USB_BootMessage(uint8 requestType, uint8 request, uint16 value, 
                           uint16 index, uint8 *buff, uint16 len)
{
   if (usb_control_msg(handle, requestType, request, value, index, 
      buff, len, CIMAX_BOOT_RESPONSE_TIMEOUT_MS) != len)
   {
      printf("%s: boot message failed!\n", __FUNCTION__);
      return CIMAX_USB_ERROR;
   }
   
   return len;
}

int32 CIMAX_USB_WriteAltSet2(uint8 *buff, uint16 numOfBytes)
{
   int32 bytesWritten = 0, counter, numOfBytesLeft;
   
   if (numOfBytes > CIMAX_ALT_SET_2_MAX_PACKET_SIZE)
   {
      for (counter = 0; counter < (numOfBytes / CIMAX_ALT_SET_2_MAX_PACKET_SIZE); counter++)
         bytesWritten += usb_bulk_write(handle, CIMAX_ALT_SET_2_EP_WRITE, buff + counter * CIMAX_ALT_SET_2_MAX_PACKET_SIZE, 
            CIMAX_ALT_SET_2_MAX_PACKET_SIZE, CIMAX_FW_RESPONSE_TIMEOUT_MS);
      
      numOfBytesLeft = numOfBytes - counter * CIMAX_ALT_SET_2_MAX_PACKET_SIZE;
      if (numOfBytesLeft > 0)
      {
         bytesWritten += usb_bulk_write(handle, CIMAX_ALT_SET_2_EP_WRITE, buff + counter * CIMAX_ALT_SET_2_MAX_PACKET_SIZE,
            numOfBytesLeft, CIMAX_FW_RESPONSE_TIMEOUT_MS);
      }
   } else 
   {
      bytesWritten = usb_bulk_write(handle, CIMAX_ALT_SET_2_EP_WRITE, buff, numOfBytes, 
         CIMAX_FW_RESPONSE_TIMEOUT_MS);
   }

   if (bytesWritten != numOfBytes)
   {
      printf("%s: write failed, error code %d!\n", __FUNCTION__, bytesWritten);
      return CIMAX_USB_ERROR;
   }
    
   return CIMAX_USB_NO_ERROR;
}
int32 CIMAX_USB_ReadAltSet2(uint8 *buff)
{
   int32 bytesRead = 0, bytesReadByTransaction = 0;
   
   while ((bytesReadByTransaction = usb_interrupt_read(handle, CIMAX_ALT_SET_2_EP_READ, buff + bytesReadByTransaction, 
                                 CIMAX_ALT_SET_2_MAX_PACKET_SIZE, CIMAX_FW_RESPONSE_TIMEOUT_MS)) == CIMAX_ALT_SET_2_MAX_PACKET_SIZE)
   {
      bytesRead += CIMAX_ALT_SET_2_MAX_PACKET_SIZE;
   }

   if (bytesReadByTransaction > 0)
   {
      bytesRead += bytesReadByTransaction;
   } else
   {
      if (bytesRead == 0)
      {
         bytesRead = bytesReadByTransaction;
      }
   }

   if (bytesRead < 0)
   {
      //printf("%s: read failed!\n", __FUNCTION__);
      return CIMAX_USB_ERROR;
   }
    
   return bytesRead;
}

void CIMAX_USB_Close()
{
   usb_release_interface(handle, CIMAX_INTERFACE);
   usb_close(handle);
}

/******************************************************************************
*  Internal helpers
******************************************************************************/
/**
* @ingroup     CIMaX+
*
* @brief       Seeks to and opens USB device specified by VID and PID.
*
* @param       bus      USB bus pointer
* @param       vid      vendor ID 
* @param       pid      product ID
* @param       pHandle  pointer to handle of opened USB device
*
* @return      device handle if successfull, NULL if failure.
*/
struct usb_device *seek_dev(struct usb_bus *bus, int32 vid, int32 pid, usb_dev_handle **pHandle)
{
   struct usb_device *dev = NULL;
   
   for (dev=bus->devices; dev; dev=dev->next)
   {
      if ((vid==dev->descriptor.idVendor) && (pid==dev->descriptor.idProduct))
      {
         *pHandle = usb_open(dev);
         break;
      }
   }
   
   return dev;
}

/**
* @ingroup     CIMaX+
*
* @brief       Opens interface and alternate interface of device specified by handle.
*
* @param       handle      USB device handle
* @param       interface   Selected interface 
* @param       alternate   Selected alternate interface
*
* @return      CIMAX_USB_NO_ERROR if successfull, CIMAX_USB_ERROR if failure.
*/
int32 open_interface(usb_dev_handle *handle, int32 interface, int32 alternate)
{
   struct usb_config_descriptor config_desc;
       
   if (usb_get_descriptor(handle, USB_DT_CONFIG, CIMAX_CONFIGURATION, &config_desc, USB_DT_CONFIG_SIZE) == USB_DT_CONFIG_SIZE )
   {
      /* Set configuration 1  */
      if (usb_set_configuration(handle, config_desc.bConfigurationValue) == 0)
      {
         printf("%s: configuration %d set.\n", __FUNCTION__, config_desc.bConfigurationValue);
         
         /*  Claim interface  */ 
         if (usb_claim_interface( handle, interface)  == 0)
         {
            printf("%s: interface %d claimed.\n", __FUNCTION__, interface);
            
            /* Set alternate interface */
            if (usb_set_altinterface(handle, alternate) == 0)
            {
               printf("%s: alternate interface %d set.\n", __FUNCTION__, alternate);
               return CIMAX_USB_NO_ERROR;
            }
            else
               printf("%s: setting alternate interface %d failed!\n", __FUNCTION__, alternate);
         }
         else
            printf("%s: claim interface %d failed!\n", __FUNCTION__, interface);
      }
      else
         printf("%s: setting configuration %d failed!\n", __FUNCTION__, config_desc.bConfigurationValue);
   }
   else
      printf("%s: unable to get device descriptor!\n",__FUNCTION__);
       
   return CIMAX_USB_ERROR;
}
