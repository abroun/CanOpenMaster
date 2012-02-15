//--------------------------------------------------------------------------------------------------
// File: CanDriver.h
// Desc: A class which loads a dynamic driver library and provides hooks for calling into the
//       library
//--------------------------------------------------------------------------------------------------

#ifndef CAN_DRIVER_H_
#define CAN_DRIVER_H_

//--------------------------------------------------------------------------------------------------
#include <string>
#include "CanOpenMaster/can.h"

//--------------------------------------------------------------------------------------------------
class CanDriver
{
    public: CanDriver( const std::string& driverFilename );
    public: virtual ~CanDriver();

    public: COM_DeviceHandle openDevice( const char* deviceName, const char* baudRate )
    {
        return mpFnOpenDevice( deviceName, baudRate );
    }

    public: void closeDevice( COM_DeviceHandle handle )
    {
        mpFnCloseDevice( handle );
    }

    public: uint8_t sendMessage( COM_DeviceHandle handle, COM_CanMessage* pMsg )
    {
        return mpFnSendMessage( handle, pMsg );
    }

    public: uint8_t receiveMessage( COM_DeviceHandle handle, COM_CanMessage* pMsgOut )
    {
        return mpFnReceiveMessage( handle, pMsgOut );
    }

    // Function hooks
    private: COM_DriverDeviceOpenFn mpFnOpenDevice;
    private: COM_DriverDeviceCloseFn mpFnCloseDevice;
    private: COM_DriverSendMessageFn mpFnSendMessage;
    private: COM_DriverReceiveMessageFn mpFnReceiveMessage;
    private: void* mLibraryHandle;
};


#endif // CAN_DRIVER_H_
