/** @file ArcusPerformaxDriver.h
 * Main interface to the Arcus Performax Driver library
 */


/** \mainpage Arcus Performax Driver Documentation

The Arcus Performax Driver library for linux mirrors the provided windows interface.

The following example provides a sample application.

\code
#include <stdio.h>
#include <string.h>
#include "ArcusPerformaxDriver.h"

int main(void)
{
	char 		lpDeviceString[PERFORMAX_MAX_DEVICE_STRLEN];
	AR_HANDLE	Handle; //usb handle
	char		out[64];
	char		in[64];
	AR_DWORD	num;
	int i;
	
	memset(out,0,64);
	memset(in,0,64);

	//acquire information
	
	if(!fnPerformaxComGetNumDevices(&num))
	{
		printf("error in fnPerformaxComGetNumDevices\n");
		return 1;
	}
	if(num<1)
	{
		printf( "No motor found\n");
		return 1;
	}

	if( !fnPerformaxComGetProductString(0, lpDeviceString, PERFORMAX_RETURN_SERIAL_NUMBER) ||
		!fnPerformaxComGetProductString(0, lpDeviceString, PERFORMAX_RETURN_DESCRIPTION) )
	{
		printf("error acquiring product string\n");
		return 1;
	}
	
	printf("device description: %s\n", lpDeviceString);
	
	//setup the connection
	
	if(!fnPerformaxComOpen(0,&Handle))
	{
		printf( "Error opening device\n");
		return 1;
	}
	
	if(!fnPerformaxComSetTimeouts(5000,5000))
	{
		printf("Error setting timeouts\n");
		return 1;
	}
	if(!fnPerformaxComFlush(Handle))
	{
		printf("Error flushing the coms\n");
		return 1;
	}
	
	// setup the device
	
	strcpy(out, "LSPD=3000"); //set low speed
	if(!fnPerformaxComSendRecv(Handle, out, 64,64, in))
	{
		printf("Could not send\n");
		return 1;
	}
	strcpy(out, "HSPD=30000"); //set high speed
	if(!fnPerformaxComSendRecv(Handle, out, 64,64, in))
	{
		printf("Could not send\n");
		return 1;
	}
	
	strcpy(out, "CUR=500"); //set current
	if(!fnPerformaxComSendRecv(Handle, out, 64,64, in))
	{
		printf("Could not send\n");
		return 1;
	}
	
	strcpy(out, "CUR"); //read current
	if(!fnPerformaxComSendRecv(Handle, out, 64,64, in))
	{
		printf("Could not send\n");
		return 1;
	}
	
	printf("current is set to: %s\n",in);
	
	strcpy(out, "ACC=300"); //set acceleration
	if(!fnPerformaxComSendRecv(Handle, out, 64,64, in))
	{
		printf("Could not send\n");
		return 1;
	}
	
	strcpy(out, "EO=1"); //enable device
	if(!fnPerformaxComSendRecv(Handle, out, 64,64, in))
	{
		printf("Could not send\n");
		return 1;
	}
	strcpy(out, "INC"); //set incremental mode
	if(!fnPerformaxComSendRecv(Handle, out, 64,64, in))
	{
		printf("Could not send\n");
		return 1;
	}
	
	printf("Motor is moving. Please wait.\n");
	
	for (i=0; i<5; i++)
	{
		do
		{
			strcpy(out, "X-32000"); //move the motor
			if(!fnPerformaxComSendRecv(Handle, out, 64,64, in))
			{
				printf("Could not send\n");
				return 1;
			}
			
			sleep(1);
		} while(in[0]!='O'); //motor is still moving. wait and retry;
		
		do
		{
			strcpy(out, "X32000"); //move the motor
			if(!fnPerformaxComSendRecv(Handle, out, 64,64, in))
			{
				printf("Could not send\n");
				return 1;
			}
			
			sleep(1);
		}while(in[0]!='O'); //motor is still moving. wait and retry;
	
	}
	
	if(!fnPerformaxComClose(Handle))
	{
		printf( "Error Closing\n");
		return 1;
	}
	
	printf("Motor connection has been closed\n");
}

\endcode
*/


#ifndef _ARCUSPERFORMAXDRIVER_H_
#define _ARCUSPERFORMAXDRIVER_H_

//libusb standard header file
#include <usb.h>

//defined by Arcus in windows library. Used as identifiers to fnPerformaxComGetProductString.
#define PERFORMAX_RETURN_SERIAL_NUMBER	0x0

//defined by Arcus in windows library. Used as identifiers to fnPerformaxComGetProductString.
#define PERFORMAX_RETURN_DESCRIPTION	0x1

//defined by Arcus in windows library. Maximum string length returned by fnPerformaxComGetProductString.
#define PERFORMAX_MAX_DEVICE_STRLEN		256

/// \name Arcus 
/// These are provived for straight compatibility with windows library.
/// \{

typedef int AR_BOOL;		///< C does not have a bool type. The type is arbitrary, 0 means 'false' any other value means true.
typedef long AR_DWORD;		///< Doubleword type.
typedef void AR_VOID;		///< Void type.
typedef usb_dev_handle*	AR_HANDLE;		///< This is a device handle in linux.

/// \}

extern "C" {

/// \name Arcus
/// These functions provide the same API as the windows library provided by Arcus.
/// \{

/** Returns the number of connected devices.
  \param numDevices (OUTPUT) returns number of connected devices discovered.
  \return true if successfull, false otherwise.
 */
AR_BOOL fnPerformaxComGetNumDevices(AR_DWORD *numDevices); 

/** Enumerates device serial numbers and descriptions.
  \param dwNumDevice (INPUT) Number of the device to read. Valid range is from 0 to numDevices as returned by fnPerformaxComGetNumDevices().
  \param lpDeviceString (OUTPUT) Returns in a string, the requested information.
  \param dwOptions (INPUT) either #PERFORMAX_RETURN_SERIAL_NUMBER or #PERFORMAX_RETURN_DESCRIPTION.
  \return true if successfull, false otherwise.
 */
AR_BOOL fnPerformaxComGetProductString(AR_DWORD dwNumDevice, AR_VOID *lpDeviceString, AR_DWORD dwOptions);

/** Opens a specific device.
  \param dwNumDevice (INPUT) Number of the device to read. Valid range is from 0 to numDevices as returned by fnPerformaxComGetNumDevices().
  \param pHandle (OUTPUT) Returns a handle describing the open device, to be used as input to other functions.
  \return true if successfull, false otherwise. 
*/
AR_BOOL fnPerformaxComOpen(AR_DWORD dwDeviceNum, AR_HANDLE *pHandle);

/** Closes an open device.
  \param pHandle (INPUT) Handle of an open device.
  \return true if successfull, false otherwise.
*/
AR_BOOL fnPerformaxComClose(AR_HANDLE pHandle);

/** Sets read and write timeouts
  \param dwReadTimeout (INPUT) read timeout to be set.
  \param dwWriteTimeout (INPUT) write timeout to be set.
  \return true if successfull, false otherwise.
*/
AR_BOOL fnPerformaxComSetTimeouts(AR_DWORD dwReadTimeout, AR_DWORD dwWriteTimeout);

/** Send a command and wait for the device to answer
  \param Handle (INPUT) Handle to an open device.
  \param wbuffer (INPUT) buffer to a string to be sent to the device.
  \param dwNumBytesToWrite (INPUT) number of characters to be sent to the device
  \param dwNumBytesToRead (INPUT) size of the read buffer
  \param rBuffer (OUTPUT) read buffer. Holds the device reply.
  \return true if successfull, false otherwise.
*/
AR_BOOL fnPerformaxComSendRecv(AR_HANDLE Handle, AR_VOID *wBuffer, AR_DWORD dwNumBytesToWrite, AR_DWORD dwNumBytesToRead, AR_VOID *rBuffer);

/** Send a Flush Command
  \param Handle (INPUT) Handle to an open device.
  \return true if successfull, false otherwise.
*/
AR_BOOL fnPerformaxComFlush(AR_HANDLE Handle);
/// \}

/** This function is not provided by Arcus. It does not need to be called on linux. 
  On windows, this function is used to setup the dll functions.
  \return true if successfull, false otherwise.
*/
//AR_BOOL InitializePerformaxLibrary(AR_VOID);
}
#endif

