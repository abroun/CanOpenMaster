
#include <stdint.h>

//------------------------------------------------------------------------------
#ifndef COM_CAN_H
#define COM_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void* COM_DeviceHandle;

typedef struct 
{
    uint16_t mCobId; // message's ID
    uint8_t mRtr;     // remote transmission request. (0 if not rtr message, 1 if rtr message)
    uint8_t mLength;     // message's length (0 to 8)
    uint8_t mData[ 8 ]; // message's data
} COM_CanMessage;

#define COM_EMPTY_CAN_MSG { 0, 0, 0, { 0 } }

//------------------------------------------------------------------------------
typedef COM_DeviceHandle (*COM_DriverDeviceOpenFn)( const char* deviceName, const char* baudRate );
typedef void (*COM_DriverDeviceCloseFn)( COM_DeviceHandle handle );

//------------------------------------------------------------------------------
typedef uint8_t (*COM_DriverSendMessageFn)( COM_DeviceHandle handle, COM_CanMessage* pMsg );
typedef uint8_t (*COM_DriverReceiveMessageFn)( COM_DeviceHandle handle, COM_CanMessage* pMsgOut );

#ifdef __cplusplus
}
#endif

#endif // COM_CAN_H
