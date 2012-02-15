#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdint.h>

#include "CanOpenMaster/can.h"
#include "drivers/common/Logging.h"
#include "drivers/common/RollingBuffer.h"

/* At time of writing, these constants are not defined in the headers */
#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

typedef enum eDriverState
{
    eDS_Inactive,
    eDS_Active
} eDriverState;

typedef enum eBaudRate
{
    eBR_Invalid = -1,
    eBR_10K = 0,
    eBR_20K,
    eBR_50K,
    eBR_100K,
    eBR_125K,
    eBR_250K,
    eBR_500K,
    eBR_800K,
    eBR_1M,
} eBaudRate;

static eBaudRate gBaudRate = eBR_125K;
static eDriverState gDriverState = eDS_Inactive;

static RollingBuffer gReadBuffer;
static unsigned char gReadBufferData[ 64*1024 ];

//--------------------------------------------------------------------------------------------------
uint8_t COM_DriverReceiveMessage( COM_DeviceHandle handle, COM_CanMessage* pMsgOut )
{
    LogMsg( eV_Info, "--- Entering Read Routine\n" );

/*    if ( eDS_Inactive == gDriverState )
    {
        LogMsg( eV_Info, "--- Leaving Read Routine\n" );
        return 1;
    }

    LogMsg( eV_Info, "Trying to read message\n" );

    if ( RB_GetNumBytesInBuffer( &gReadBuffer ) > 0 )
    {
        char readBuffer[ READ_BUFFER_CHUNK_SIZE ];
        unsigned int numBytesRead = RB_PeekAtBytes( &gReadBuffer, (unsigned char*)readBuffer, sizeof( readBuffer ) );
        bool bMsgRead = false;
        int replyStartIdx = 0;
        int numBytesUsed = 0;

        for ( int byteIdx = 0; byteIdx < numBytesRead; byteIdx++ )
        {
            if ( CANUSB_SUCCESS_BYTE == readBuffer[ byteIdx ] )
            {
                LogMsg( eV_Info, "Got success byte\n" );

                int numBytes = byteIdx - replyStartIdx;
                if ( numBytes > 0 )
                {
                    if ( 1 == numBytes
                        && 'z' == readBuffer[ byteIdx - 1 ] )
                    {
                        // This is a +ve send response. Ignore it
                    }
                    else
                    {
                        LogMsg( eV_Info, "%i bytes of data\n", numBytes );
                        bMsgRead = canusbToCanMsg( &readBuffer[ replyStartIdx ], numBytes, pMsgOut );
                    }

                    if ( !bMsgRead )
                    {
                        replyStartIdx = byteIdx + 1;
                    }
                    else
                    {
                        LogMsg( eV_Info, "Message read!\n" );
                        LogMsg( eV_Info, "%i bytes left\n", (numBytesRead - byteIdx)-1 );
                        numBytesUsed = byteIdx + 1;
                        break;  // Break out of checking loop
                    }
                }
                else
                {
                    LogMsg( eV_Info, "No data with success byte\n" );
                    LogMsg( eV_Info, "%i bytes left\n", (numBytesRead - byteIdx)-1 );
                    numBytesUsed = byteIdx + 1;
                    break;  // Break out of checking loop
                }
            }
            else if ( CANUSB_FAIL_BYTE == readBuffer[ byteIdx ] )
            {
                LogMsg( eV_Info, "Got fail byte\n" );
                LogMsg( eV_Info, "%i bytes left\n", (numBytesRead - byteIdx)-1 );
                numBytesUsed = byteIdx + 1;
                break;  // Break out of checking loop
            }
        }

        RB_AdvanceBuffer( &gReadBuffer, numBytesUsed );

        if ( bMsgRead )
        {
            LogMsg( eV_Info, "--- Leaving Read Routine\n" );
            return 0;
        }
        else
        {
            ReadDataIntoBuffer();
            //LogMsg( eV_Error, "%i bytes in buffer\n", RB_GetNumBytesInBuffer( &gReadBuffer ) );
            LogMsg( eV_Info, "No message found in buffer. User should retry\n" );

            return 2;
        }
    }
    else
    {
        ReadDataIntoBuffer();
        //LogMsg( eV_Error, "%i bytes in buffer\n", RB_GetNumBytesInBuffer( &gReadBuffer ) );

        LogMsg( eV_Info, "Nothing to read\n" );
        LogMsg( eV_Info, "--- Leaving Read Routine\n" );
        return 2;
    }*/

    return 0;
}

//--------------------------------------------------------------------------------------------------
uint8_t COM_DriverSendMessage( COM_DeviceHandle handle, COM_CanMessage* pMsg )
{
    LogMsg( eV_Info, "--- Entering Send Routine\n" );

/*    if ( eDS_Inactive == gDriverState )
    {
        LogMsg( eV_Info, "--- Leaving Send Routine\n" );
        return 1;
    }
    if ( (pMsg->mCobId & 0xF800) != 0 )
    {
        LogMsg( eV_Error, "Error: Driver can't handle 29-bit ids yet\n" );
        LogMsg( eV_Info, "--- Leaving Send Routine\n" );
        return 1;
    }
    if ( 0 != pMsg->mRtr )
    {
        LogMsg( eV_Error, "Error: Driver can't send RTR bit yet\n" );
        LogMsg( eV_Info, "--- Leaving Send Routine\n" );
        return 1;
    }
    if ( pMsg->mLength > 8 )
    {
        LogMsg( eV_Error, "Error: Too many bytes\n" );
        LogMsg( eV_Info, "--- Leaving Send Routine\n" );
        return 1;
    }

    char commandBuffer[ 128 ];
    char* pCurByte = commandBuffer;
    sprintf( pCurByte, "t%X%X%X%X",
             (pMsg->mCobId & 0x700) >> 8, (pMsg->mCobId & 0xF0) >> 4,
             (pMsg->mCobId & 0xF), (pMsg->mLength & 0xF) );
    pCurByte += 5;

    for ( int i = 0; i < pMsg->mLength; i++ )
    {
        sprintf( pCurByte, "%02X", pMsg->mData[ i ] );
        pCurByte += 2;
    }

    *(pCurByte++) = '\r';  // Terminate command
    *(pCurByte++) = '\0';

    LogMsg( eV_Info, "Sending %s\n", commandBuffer );

    int numBytesToSend = strlen( commandBuffer );
    int numBytesWritten = ftdi_write_data( &gContext, (unsigned char*)commandBuffer, numBytesToSend );
    if ( numBytesWritten < numBytesToSend )
    {
        LogMsg( eV_Error, "Error: Couldn't send all bytes. Result was %i\n", numBytesWritten );
        LogMsg( eV_Info, "--- Leaving Send Routine\n" );
        return 1;
    }

    /*if ( !SendCANUSBCommand( commandBuffer ) )
    {
        LogMsg( eV_Info, "Error: Unable to send CAN message\n" );
        LogMsg( eV_Info, "--- Leaving Send Routine\n" );
        return 1;
    }*/

    LogMsg( eV_Info, "--- Leaving Send Routine\n" );
    return 0;
}

//--------------------------------------------------------------------------------------------------
eBaudRate TranslateBaudRate( const char* optarg )
{
    if(!strcmp( optarg, "1M")) return eBR_1M;
    if(!strcmp( optarg, "500K")) return eBR_500K;
    if(!strcmp( optarg, "250K")) return eBR_250K;
    if(!strcmp( optarg, "125K")) return eBR_125K;
    if(!strcmp( optarg, "100K")) return eBR_100K;
    if(!strcmp( optarg, "50K")) return eBR_50K;
    if(!strcmp( optarg, "20K")) return eBR_20K;
    if(!strcmp( optarg, "10K")) return eBR_10K;
    if(!strcmp( optarg, "5K"))
    {
        LogMsg( eV_Warning, "Warning: 5K is unhandled baud rate. Ignoring...\n" );
        return eBR_Invalid;
    }
    if(!strcmp( optarg, "none")) return eBR_Invalid;
    return eBR_Invalid;
}

//--------------------------------------------------------------------------------------------------
COM_DeviceHandle COM_DriverDeviceOpen( const char* deviceName, const char* baudRate )
{
    LogMsg( eV_Error, "_-----> Trying to open SocketCAN device\n" );
    return NULL;
/*
    if ( eDS_Active == gDriverState )
    {
        return (COM_DeviceHandle)&gContext;
    }

    LogMsg( eV_Info, "Trying to open CANUSB driver\n" );

    // Setup a rolling buffer to take the incoming data
    RB_Init( &gReadBuffer, gReadBufferData, sizeof( gReadBufferData ) );

    // Initialise the FTDI library
    if ( ftdi_init( &gContext ) < 0 )
    {
        LogMsg( eV_Error, "Error: ftdi_init failed\n" );
        return NULL;
    }

    // Set a 3 second read and write timeout
    gContext.usb_read_timeout = 3000;
    gContext.usb_write_timeout = 3000;
    ftdi_read_data_set_chunksize( &gContext, READ_BUFFER_CHUNK_SIZE );

    // Open the device
    int result = ftdi_usb_open( &gContext, CANUSB_VENDOR_ID, CANUSB_PRODUCT_ID );
    if( result < 0 )
    {
        LogMsg( eV_Error, "Error: ftdi_usb_open failed. result=%d - %s\n", result, gContext.error_str );
    LogMsg( eV_Error, "=== Did you install udev/01-ftdi.rules?\n" );
        ftdi_deinit( &gContext );
        return NULL;
    }

    LogMsg( eV_Info, "Opened FTDI\n" );

    // Reset the CANUSB
    LogMsg( eV_Info, "Reset reply = %i\n", ftdi_usb_reset( &gContext ) );
    LogMsg( eV_Info, "Purge reply = %i\n", ftdi_usb_purge_buffers( &gContext ) );

    sleep( 1 );     // Short sleep to give the device time to settle

    SendResetMessage();
    LogMsg( eV_Info, "Sent reset message\n" );

    // Set the baudrate
    eBaudRate newBaudRate = TranslateBaudRate( baudRate );
    if ( eBR_Invalid != newBaudRate )
    {
        gBaudRate = newBaudRate;
    }

    char commandBuffer[ 64 ];
    sprintf( commandBuffer, "S%i\r", gBaudRate );
    if ( !SendCANUSBCommand( commandBuffer ) )
    {
        LogMsg( eV_Error, "Error: Unable to set CANUSB baud rate\n" );
        ftdi_usb_close( &gContext );
        ftdi_deinit( &gContext );
        return NULL;
    }

    LogMsg( eV_Info, "Set baud rate\n" );

    // Open the channel
    if ( !SendCANUSBCommand( "O\r" ) )
    {
        LogMsg( eV_Error, "Error: Unable to open CANUSB channel\n" );
        ftdi_usb_close( &gContext );
        ftdi_deinit( &gContext );
        return NULL;
    }

    LogMsg( eV_Info, "Opened channel\n" );

    // Success
    gDriverState = eDS_Active;
    return (COM_DeviceHandle)&gContext;*/
}

//--------------------------------------------------------------------------------------------------
void COM_DriverDeviceClose( COM_DeviceHandle handle )
{
   /* if ( eDS_Inactive != gDriverState )
    {
        SendCANUSBCommand( "C\r" );
        ftdi_usb_close( &gContext );
        ftdi_deinit( &gContext );
        gDriverState = eDS_Inactive;
    }*/
}

