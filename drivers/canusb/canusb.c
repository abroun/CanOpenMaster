
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdarg.h>
#include <unistd.h>

// Need libftdi
#include <ftdi.h>

#include "CanOpenMaster/can.h"
#include "RollingBuffer.h"

// This driver is limited at the moment in that you are only able
// to have the one CANUSB device active at any one time
#define CANUSB_VENDOR_ID 0x0403
#define CANUSB_PRODUCT_ID 0xFFA8

#define READ_BUFFER_CHUNK_SIZE 4096

#define CANUSB_SUCCESS_BYTE 13
#define CANUSB_FAIL_BYTE 7

typedef struct ftdi_context FTDIContext;

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

typedef enum eVerbosity
{
    eV_Invalid = -1,
    eV_Silent = 0,
    eV_Error,
    eV_Warning,
    eV_Info
} eVerbosity;

static eBaudRate gBaudRate = eBR_125K;
static eDriverState gDriverState = eDS_Inactive;
static FTDIContext gContext;
static eVerbosity gVerbosity = eV_Error;

static RollingBuffer gReadBuffer;
static unsigned char gReadBufferData[ 64*1024 ];

//------------------------------------------------------------------------------
void LogMsg( eVerbosity verbosity, const char* formatString, ... )
{
    if ( verbosity <= gVerbosity )
    {
        va_list argList;
        va_start( argList, formatString );

        vprintf( formatString, argList );
        
        va_end( argList );
    }
}

//------------------------------------------------------------------------------
void ReadDataIntoBuffer()
{
    char chunkBuffer[ READ_BUFFER_CHUNK_SIZE ];
    
    // Read as much as possible
    bool bFinished = false;
    while ( !bFinished )
    {
        int numBytesRead = ftdi_read_data( &gContext, (unsigned char*)chunkBuffer, sizeof( chunkBuffer ) );
        if ( 0 == numBytesRead )
        {
            // No data available
            bFinished = true;
        }
        else if ( numBytesRead < 0 )
        {
            LogMsg( eV_Error, "Error: Unable to read from CANUSB\n" );
            bFinished = true;
        }
        else // numBytesRead > 0
        {
            if ( !RB_TryToAddBytes( &gReadBuffer, (unsigned char*)chunkBuffer, numBytesRead ) )
            {
                LogMsg( eV_Error, "Error: Read buffer overflow\n" );
                bFinished = true;
            }
        }
    }
}

//------------------------------------------------------------------------------
bool SendCANUSBCommandAndGetReply( char* commandBuffer, 
                        char* resultBuffer, int resultBufferSize )
{
    bool bSuccess = false;
    
    
    LogMsg( eV_Info, "Sending %s\n", commandBuffer );
    
    int numBytesToSend = strlen( commandBuffer );
    int numBytesWritten = ftdi_write_data( &gContext, (unsigned char*)commandBuffer, numBytesToSend );
    if ( numBytesWritten < numBytesToSend )
    {
        LogMsg( eV_Error, "Error: Couldn't send all bytes. Result was %i\n", numBytesWritten );
        return false;
    }

    for ( ;; )  // Loop waiting for an answer - TODO: Put in timeout
    {
        ReadDataIntoBuffer();
        
        if ( RB_GetNumBytesInBuffer( &gReadBuffer ) > 0 )
        {
            char readBuffer[ READ_BUFFER_CHUNK_SIZE ];
            unsigned int numBytesRead = RB_PeekAtBytes( &gReadBuffer, (unsigned char*)readBuffer, sizeof( readBuffer ) );
            bool bGotResponse = false;
            int replyStartIdx = 0;
            int numBytesUsed = 0;
            
            for ( int byteIdx = 0; byteIdx < numBytesRead; byteIdx++ )
            {
                if ( CANUSB_SUCCESS_BYTE == readBuffer[ byteIdx ] )
                {
                    if ( byteIdx - replyStartIdx > 0 )
                    {
                        if ( NULL != resultBuffer )
                        {
                            // Return result string
                            int numBytesNeeded = (byteIdx - replyStartIdx) + 1;
                            if ( resultBufferSize >= numBytesNeeded )
                            {
                                memcpy( resultBuffer, &readBuffer[ replyStartIdx ], numBytesNeeded );
                                resultBuffer[ numBytesNeeded - 1 ] = '\0';
                                LogMsg( eV_Info, "Got +ve response and reply of %s\n", resultBuffer );
                                for ( int i = 0; i < numBytesNeeded; i++ )
                                {
                                    LogMsg( eV_Info, "%i ", resultBuffer[ i ] );
                                }
                                LogMsg( eV_Info, "\n" );
                            }
                            else
                            {
                                LogMsg( eV_Warning, "Warning: Not enough space for CANUSB result\n" );
                            }
                        }
                        else
                        {
                            LogMsg( eV_Warning, "Warning: Discarding reply of %i bytes\n", byteIdx );
                        }
                        
                        bSuccess = true;
                        bGotResponse = true;
                        numBytesUsed = byteIdx + 1;
                        break;  // Break out of checking loop
                    }
                    else
                    {
                        if ( resultBufferSize > 0 )
                        {
                            LogMsg( eV_Info, "Got ok but no reply\n" );
                            replyStartIdx = byteIdx + 1;
                        }
                        else
                        {
                            LogMsg( eV_Info, "Got +ve response\n" );
                            bSuccess = true;
                            bGotResponse = true;
                            numBytesUsed = byteIdx + 1;
                            break;  // Break out of checking loop
                        }
                    }
                }
                else if ( CANUSB_FAIL_BYTE == readBuffer[ byteIdx ] )
                {
                    LogMsg( eV_Info, "Got fail byte\n" );
                    bGotResponse = true;
                    numBytesUsed = byteIdx + 1;
                    break;  // Break out of checking loop
                }
            }
            
            RB_AdvanceBuffer( &gReadBuffer, numBytesUsed );
            
            if ( bGotResponse )
            {
                LogMsg( eV_Info, "Finished busy wait\n" );
                break;  // Break out of busy wait
            }
        }
    }
    
    return bSuccess;
}

//------------------------------------------------------------------------------
bool SendCANUSBCommand( char* commandBuffer )
{
    return SendCANUSBCommandAndGetReply( commandBuffer, NULL, 0 );
}

//------------------------------------------------------------------------------
void SendResetMessage()
{
    SendCANUSBCommand( "C\r" );
    SendCANUSBCommand( "\r" );
    SendCANUSBCommand( "\r" );
    SendCANUSBCommand( "\r" );
    
    char versionBuffer[ 64 ] = { 0 };
    if ( SendCANUSBCommandAndGetReply( "V\r", versionBuffer, sizeof( versionBuffer ) ) )
    {
        LogMsg( eV_Info, "Got version of %s\n", versionBuffer );
    }
    else
    {
        LogMsg( eV_Error, "Error: Unable to get version string\n" );
    }
    
    SendCANUSBCommand( "C\r" );
    SendCANUSBCommand( "\r" );
    SendCANUSBCommand( "\r" );
    SendCANUSBCommand( "\r" );
    SendCANUSBCommand( "Z0\r" );
    SendCANUSBCommand( "M00000000\r" );
    SendCANUSBCommand( "mFFFFFFFF\r" );
}

//------------------------------------------------------------------------------
bool canusbToCanMsg( char* p, int numBytes, COM_CanMessage* pMsg )
{
    int val;
    int i;
    short data_offset;   // Offset to dlc byte
    //char save;
    long unsigned int id;
    
    LogMsg( eV_Info, "MSG is ---> " );
    for ( int i = 0; i < numBytes; i++ )
    {
       LogMsg( eV_Info, "%c", p[ i ] );
    }
    LogMsg( eV_Info, " <--- End\n" );
    
    if ( numBytes < 1 )
    {
        return false;
    }
    
    if ( 't' == *p ) 
    {
        LogMsg( eV_Info, "Found t message\n" );
        
        if ( numBytes < 5 )
        {
            return false;
        }
        
        // Standard frame
        pMsg->mRtr = 0;
        pMsg->mLength = p[ 4 ] - '0';

        p[ 4 ] = 0;     // So that sscanf works
        sscanf( p + 1, "%lx", &id  );
        pMsg->mCobId = (unsigned short)id;
        data_offset = 5;
        
        if ( numBytes < 5 + pMsg->mLength )
        {
            return false;
        }
    }
    else
    {
        LogMsg( eV_Error, "Error: Msg type %c not handled yet\n", *p );
        return false;
    }
    /*else if ( 'r' == *p ) 
    {
        // Standard remote  frame
        pMsg->len = p[ 4 ] - '0';
        pMsg->flags = CANMSG_RTR;
        //data_offset = 5 - 1;// To make timestamp work
        data_offset = 5;
        //save = p[ 4 ];
        p[ 4 ] = 0;
        sscanf( p + 1, "%lx", &pMsg->id  );
        //p[ 4 ] = save;
    }
    else if ( 'T' == *p ) 
    {
        // Extended frame
        pMsg->flags = CANMSG_EXTENDED;
        data_offset = 10;
        pMsg->len = p[ 9 ] - '0';
        p[ 9 ] = 0;
        sscanf( p + 1, "%lx", &pMsg->id );
    }
    else if ( 'R' == *p ) 
    {
        // Extended remote frame
        pMsg->flags = CANMSG_EXTENDED | CANMSG_RTR;
        //data_offset = 10 - 1;// To make timestamp work
        data_offset = 10;
        pMsg->len = p[ 9 ] - '0';
        //save = p[ 9 ];
        p[ 9 ] = 0;
        sscanf( p + 1, "%lx", &pMsg->id );
        //p[ 9 ] = save;
    }*/
  
    //save = *(p + data_offset + 2 * pMsg->len );
  
    // Fill in data
    if ( 0 == pMsg->mRtr ) 
    {
        for ( i= MIN( pMsg->mLength, 8); i > 0; i-- ) 
        {
            *(p + data_offset + 2 * (i-1) + 2 )= 0;
            sscanf( p + data_offset + 2 * (i-1), "%x", &val );
            pMsg->mData[ i - 1 ] = val;
        }
    }

    /**(p + data_offset + 2 * pMsg->len ) = save;

    if ( !( pMsg->flags & CANMSG_RTR ) ) {
        // If timestamp is active - fetch it
        if ( 0x0d != *( p + data_offset + 2 * pMsg->len ) ) {
        p[ data_offset + 2 * ( pMsg->len ) + 4 ] = 0;
        sscanf( ( p + data_offset + 2 * ( pMsg->len ) ), "%x", &val );
        pMsg->timestamp = val;
        }
        else {
        pMsg->timestamp = 0;
        }
    }
    else {
        
        if ( 0x0d != *( p + data_offset ) ) {
        p[ data_offset + 4 ] = 0;
        sscanf( ( p + data_offset ), "%x", &val );
        pMsg->timestamp = val;
        }
        else {
        pMsg->timestamp = 0;
        }
    }*/

    return true;
}

//------------------------------------------------------------------------------
uint8_t COM_DriverReceiveMessage( COM_DriverHandle handle, COM_CanMessage* pMsgOut )
{
    LogMsg( eV_Info, "--- Entering Read Routine\n" );
    
    if ( eDS_Inactive == gDriverState )
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
    }
}

//------------------------------------------------------------------------------
uint8_t COM_DriverSendMessage( COM_DriverHandle handle, COM_CanMessage* pMsg )
{
    LogMsg( eV_Info, "--- Entering Send Routine\n" );
    
    if ( eDS_Inactive == gDriverState )
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

//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
/*UNS8 canChangeBaudRate_driver( CAN_HANDLE fd, char* baud )
{
    eBaudRate newBaudRate = TranslateBaudRate( baud );
    if ( eBR_Invalid != newBaudRate )
    {   
        if ( eDS_Inactive == gDriverState )
        {
            gBaudRate = newBaudRate;
        }
        else // eDS_Active == gDriverState
        {
            LogMsg( eV_Error, "canChangeBaudRate not yet supported by this driver\n");
        }
    }
    
    return 0;
}*/

//------------------------------------------------------------------------------
COM_DriverHandle COM_DriverOpen( const char* deviceName, const char* baudRate )
{
    if ( eDS_Active == gDriverState )
    {
        return (COM_DriverHandle)&gContext;
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
    return (COM_DriverHandle)&gContext;
}

//------------------------------------------------------------------------------
void COM_DriverClose( COM_DriverHandle handle )
{
    if ( eDS_Inactive != gDriverState )
    {
        SendCANUSBCommand( "C\r" );
        ftdi_usb_close( &gContext );
        ftdi_deinit( &gContext );
        gDriverState = eDS_Inactive;
    }
}
