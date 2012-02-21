#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#define __USE_MISC
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

typedef struct
{
    char mName[ IF_NAMESIZE ];
    int32_t mSocketId;
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

    struct can_frame frame;

    ssize_t nbytes = read( gDevices[ deviceIdx ].mSocketId, &frame, sizeof(struct can_frame) );

    if ( nbytes <= 0 )
    {
        LogMsg( eV_Info, "No message found in buffer. User should retry\n" );
        return 2;
    }

    if ( nbytes < sizeof(struct can_frame) )
    {
        LogMsg( eV_Error, "Incomplete CAN frame\n" );
        return 1;
    }

    /*
     * Controller Area Network Identifier structure
     *
     * bit 0-28 : CAN identifier (11/29 bit)
     * bit 29   : error frame flag (0 = data frame, 1 = error frame)
     * bit 30   : remote transmission request flag (1 = rtr frame)
     * bit 31   : frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
     */

    if ( frame.can_id & 0x80000000 )
    {
        LogMsg( eV_Error, "Extended address received. Not handled yet\n" );
        return 1;
    }

    pMsgOut->mRtr = ( frame.can_id & 0x40000000 ? 1 : 0 );
    pMsgOut->mCobId = frame.can_id & 0x7FF;
    pMsgOut->mLength = frame.can_dlc;
    memcpy( pMsgOut->mData, frame.data, sizeof( frame.data ) );

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
    struct can_frame frame;
    frame.can_id = pMsg->mCobId;
    if ( pMsg->mRtr )
    {
        frame.can_id |= 0x40000000;
    }

    frame.can_dlc = pMsg->mLength;
    memcpy( frame.data, pMsg->mData, sizeof( pMsg->mData ) );

    ssize_t bytes_sent = write( gDevices[ deviceIdx ].mSocketId, &frame, sizeof(frame) );

    if ( sizeof(frame) != bytes_sent )
    {
        LogMsg( eV_Error, "Error: Unable to send CAN message\n" );
        return 1;
    }

    LogMsg( eV_Info, "--- Message sent\n" );
    return 0;
}

//--------------------------------------------------------------------------------------------------
// NOTE: We can't set the baud rate programmatically for a SocketCAN device as it may be shared
// by multiple applications. It has to be set system wide.
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

    // Check device name
    if ( strlen( deviceName ) > IF_NAMESIZE-1 )
    {
        LogMsg( eV_Error, "Device name is to long. Max length is %i\n", IF_NAMESIZE-1 );
        return NULL;
    }

    // Check to see if the device is already open
    for ( int32_t i = 0; i < MAX_NUM_DEVICES; i++ )
    {
        if ( gDevices[ i ].mbInUse
            && strncmp( gDevices[ i ].mName, deviceName, IF_NAMESIZE ) == 0 )
        {
            LogMsg( eV_Error, "%s is already open\n", deviceName );
            return NULL;
        }
    }

    LogMsg( eV_Info, "Trying to open CANUSB driver\n" );

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
    gDevices[ deviceIdx ].mSocketId = socket( PF_CAN, SOCK_RAW, CAN_RAW );
    if ( gDevices[ deviceIdx ].mSocketId < 0 )
    {
        LogMsg( eV_Error, "Unable to open socket\n" );
        return NULL;
    }

    // Specify non-blocking operation
    fcntl( gDevices[ deviceIdx ].mSocketId, F_SETFL, O_NONBLOCK );

    // Locate the interface we wish to use
    struct ifreq ifr;
    strcpy( ifr.ifr_name, deviceName );

    // ifr.ifr_ifindex gets filled with that device's index
    if ( ioctl( gDevices[ deviceIdx ].mSocketId, SIOCGIFINDEX, &ifr ) < 0 )
    {
        LogMsg( eV_Error, "Can't open %s\n", deviceName );
        return NULL;
    }

    // Select that CAN interface, and bind the socket to it. */
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if ( bind( gDevices[ deviceIdx ].mSocketId, (struct sockaddr*)&addr, sizeof(addr) ) < 0 )
    {
        LogMsg( eV_Error, "Can't bind %s to socket\n", deviceName );
        return NULL;
    }

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
        close( gDevices[ deviceIdx ].mSocketId );
        RB_Deinit( &gDevices[ deviceIdx ].mReadBuffer );
        gDevices[ deviceIdx ].mbInUse = false;
    }
}

