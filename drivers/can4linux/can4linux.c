
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include "can4linux.h"
#include "CanOpenMaster/can.h"
#include "drivers/common/Logging.h"
#include "drivers/common/RollingBuffer.h"

typedef enum eDriverState
{
    eDS_Inactive,
    eDS_Active
} eDriverState;

typedef enum eBaudRate
{
    eBR_Invalid = -1,
    eBR_5K = 5,
    eBR_10K = 10,
    eBR_20K = 20,
    eBR_50K = 50,
    eBR_100K = 100,
    eBR_125K = 125,
    eBR_250K = 250,
    eBR_500K = 500,
    eBR_800K = 800,
    eBR_1M = 1000,
} eBaudRate;

#define MAX_DEVICE_NAMESIZE 128

typedef struct
{
    char mName[ MAX_DEVICE_NAMESIZE ];
    int32_t mFileDescriptor;
    bool mbInUse;
    RollingBuffer mReadBuffer;
    uint8_t mReadBufferData[ 64*1024 ];
} CanDeviceData;

static eDriverState gDriverState = eDS_Inactive;

#define MAX_NUM_DEVICES 4
static CanDeviceData gDevices[ MAX_NUM_DEVICES ];

//--------------------------------------------------------------------------------------------------
static int32_t handleToDeviceIdx( COM_DeviceHandle handle )
{
    char data[ sizeof( handle ) ];
    *((COM_DeviceHandle*)data) = handle;

    int32_t firstByte = sizeof( handle ) - sizeof( int32_t );
    return *((int32_t*)&data[ firstByte ]) - 1;
}

//--------------------------------------------------------------------------------------------------
static COM_DeviceHandle deviceIdxToHandle( int32_t deviceIdx )
{
    char data[ sizeof( COM_DeviceHandle ) ];
    int32_t firstByte = sizeof( COM_DeviceHandle ) - sizeof( int32_t );
    *((int32_t*)&data[ firstByte ]) = deviceIdx + 1;

    return *((COM_DeviceHandle*)data);
}

//--------------------------------------------------------------------------------------------------
uint8_t COM_DriverReceiveMessage( COM_DeviceHandle handle, COM_CanMessage* pMsgOut )
{
    LogMsg( eV_Info, "--- Entering Read Routine\n" );

    int32_t deviceIdx = handleToDeviceIdx( handle );
    if ( deviceIdx < 0 || deviceIdx >= MAX_NUM_DEVICES
        || !gDevices[ deviceIdx ].mbInUse )
    {
        LogMsg( eV_Error, "Invalid device handle\n" );
        return 1;
    }

    canmsg_t msgBuffer[ 100 ];
    ssize_t numMsgs = read( gDevices[ deviceIdx ].mFileDescriptor, &msgBuffer, sizeof( canmsg_t ) );



    if ( numMsgs <= 0 )
    {
        LogMsg( eV_Info, "No message found in buffer. User should retry\n" );
        return 2;
    }

    printf( "Got %i msgs\n", numMsgs );

    if ( numMsgs > 1 )
    {
    	LogMsg( eV_Error, "Too many messages returned. Dropping messages\n" );
    }

    /*
     * Controller Area Network Identifier structure
     *
     * bit 0-28 : CAN identifier (11/29 bit)
     * bit 29   : error frame flag (0 = data frame, 1 = error frame)
     * bit 30   : remote transmission request flag (1 = rtr frame)
     * bit 31   : frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
     */

    if ( msgBuffer[ 0 ].id & 0x80000000 )
    {
        //LogMsg( eV_Error, "Extended address received. Not handled yet\n" );
        //return 1;
    }

    pMsgOut->mRtr = ( msgBuffer[ 0 ].id & 0x40000000 ? 1 : 0 );
    pMsgOut->mCobId = msgBuffer[ 0 ].id & 0x7FF;
    pMsgOut->mLength = msgBuffer[ 0 ].length;
    memcpy( pMsgOut->mData, msgBuffer[ 0 ].data, sizeof( msgBuffer[ 0 ].data ) );

    LogMsg( eV_Info, "--- Message read\n" );
    return 0;
}

//--------------------------------------------------------------------------------------------------
uint8_t COM_DriverSendMessage( COM_DeviceHandle handle, COM_CanMessage* pMsg )
{
    LogMsg( eV_Info, "--- Entering Send Routine\n" );

    int32_t deviceIdx = handleToDeviceIdx( handle );
    if ( deviceIdx < 0 || deviceIdx >= MAX_NUM_DEVICES
        || !gDevices[ deviceIdx ].mbInUse )
    {
        LogMsg( eV_Error, "Invalid device handle\n" );
        return 1;
    }

    // Send a message to the CAN bus
    canmsg_t canMsg;
    canMsg.cob = pMsg->mCobId;
    canMsg.id = pMsg->mCobId;
    canMsg.flags = 0;
    if ( pMsg->mRtr )
    {
    	canMsg.flags |= MSG_RTR;
    }

    canMsg.length = pMsg->mLength;
    memcpy( canMsg.data, pMsg->mData, sizeof( pMsg->mData ) );

    ssize_t blocksSent = write( gDevices[ deviceIdx ].mFileDescriptor, &canMsg, sizeof( canMsg ) );

    //printf( "Sent %i msgs\n", blocksSent );
    if ( sizeof( canMsg ) != blocksSent )
    {
        LogMsg( eV_Error, "Error: Unable to send CAN message\n" );
        return 1;
    }

    LogMsg( eV_Info, "--- Message sent\n" );
    return 0;
}

//--------------------------------------------------------------------------------------------------
static void setBaudRate( int32_t fd, eBaudRate baudRate )
{
	if ( eBR_Invalid != baudRate )
	{
		Config_par_t  cfg;
		volatile Command_par_t cmd;

		cmd.cmd = CMD_STOP;
		ioctl( fd, CAN_IOCTL_COMMAND, &cmd );

		cfg.target = CONF_TIMING;
		cfg.val1   = baudRate;
		ioctl( fd, CAN_IOCTL_CONFIG, &cfg );

		cmd.cmd = CMD_START;
		ioctl( fd, CAN_IOCTL_COMMAND, &cmd );
	}
}

//--------------------------------------------------------------------------------------------------
static eBaudRate translateBaudRate( const char* optarg )
{
    if(!strcmp( optarg, "1M")) return eBR_1M;
    if(!strcmp( optarg, "500K")) return eBR_500K;
    if(!strcmp( optarg, "250K")) return eBR_250K;
    if(!strcmp( optarg, "125K")) return eBR_125K;
    if(!strcmp( optarg, "100K")) return eBR_100K;
    if(!strcmp( optarg, "50K")) return eBR_50K;
    if(!strcmp( optarg, "20K")) return eBR_20K;
    if(!strcmp( optarg, "10K")) return eBR_10K;
    if(!strcmp( optarg, "5K"))  return eBR_5K;
    if(!strcmp( optarg, "none")) return eBR_Invalid;
    return eBR_Invalid;
}

//--------------------------------------------------------------------------------------------------
COM_DeviceHandle COM_DriverDeviceOpen( const char* deviceName, const char* baudRate )
{
    if ( eDS_Inactive == gDriverState )
    {
        // Setup driver if this is the first time this routine has been called
        for ( int32_t i = 0; i < MAX_NUM_DEVICES; i++ )
        {
            gDevices[ i ].mbInUse = false;
        }

        gDriverState = eDS_Active;
    }

    // Check baud rate
    eBaudRate baudRateValue = translateBaudRate( baudRate );
    if ( eBR_Invalid == baudRateValue )
    {
    	LogMsg( eV_Error, "Invalid baud rate of %s given\n", baudRate );
		return NULL;
    }

    // Check device name
    if ( strlen( deviceName ) > MAX_DEVICE_NAMESIZE-1 )
    {
        LogMsg( eV_Error, "Device name is to long. Max length is %i\n", MAX_DEVICE_NAMESIZE-1 );
        return NULL;
    }

    // Check to see if the device is already open
    for ( int32_t i = 0; i < MAX_NUM_DEVICES; i++ )
    {
        if ( gDevices[ i ].mbInUse
            && strncmp( gDevices[ i ].mName, deviceName, MAX_DEVICE_NAMESIZE ) == 0 )
        {
            LogMsg( eV_Error, "%s is already open\n", deviceName );
            return NULL;
        }
    }

    // Look for an empty slot
    int32_t deviceIdx = -1;
    for ( int32_t i = 0; i < MAX_NUM_DEVICES; i++ )
    {
        if ( !gDevices[ i ].mbInUse )
        {
            deviceIdx = i;
            break;
        }
    }

    if ( -1 == deviceIdx )
    {
        LogMsg( eV_Error, "Unable to find slot for device. Please increase MAX_NUM_DEVICES\n" );
        return NULL;
    }

    // Open the device
    strcpy( gDevices[ deviceIdx ].mName, deviceName );
    gDevices[ deviceIdx ].mFileDescriptor = open( deviceName, O_RDWR | O_NONBLOCK );
    if ( gDevices[ deviceIdx ].mFileDescriptor < 0 )
    {
        LogMsg( eV_Error, "Unable to open CAN device %s\n", deviceName );
        return NULL;
    }

    // Set the baud rate
    setBaudRate( gDevices[ deviceIdx ].mFileDescriptor, baudRateValue );

    // Setup a rolling buffer to take the incoming data
    RB_Init( &gDevices[ deviceIdx ].mReadBuffer,
        gDevices[ deviceIdx ].mReadBufferData, sizeof( gDevices[ deviceIdx ].mReadBufferData ) );

    gDevices[ deviceIdx ].mbInUse = true;

    LogMsg( eV_Info, "Opened channel\n" );

    // Success
    return deviceIdxToHandle( deviceIdx );
}

//--------------------------------------------------------------------------------------------------
void COM_DriverDeviceClose( COM_DeviceHandle handle )
{
    int32_t deviceIdx = handleToDeviceIdx( handle );
    if ( deviceIdx >= 0 && deviceIdx < MAX_NUM_DEVICES
        && gDevices[ deviceIdx ].mbInUse )
    {
        close( gDevices[ deviceIdx ].mFileDescriptor );
        RB_Deinit( &gDevices[ deviceIdx ].mReadBuffer );
        gDevices[ deviceIdx ].mbInUse = false;
    }
}
