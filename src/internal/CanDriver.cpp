//--------------------------------------------------------------------------------------------------
// File: CanDriver.cpp
// Desc: A class which loads a dynamic driver library and provides hooks for calling into the
//       library
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
#include <dlfcn.h>
#include <stdexcept>
#include "CanDriver.h"

//--------------------------------------------------------------------------------------------------
CanDriver::CanDriver( const std::string& driverFilename )
{
    mLibraryHandle = dlopen( driverFilename.c_str(), RTLD_NOW );
    if ( !mLibraryHandle )
    {
        std::string errorString( "Can't load " );
        errorString += driverFilename;
        errorString += "\n";
        throw std::runtime_error( errorString );
    }

    const char* pError = NULL;
    mpFnOpenDevice = (COM_DriverDeviceOpenFn)dlsym( mLibraryHandle, "COM_DriverDeviceOpen" );
    pError = dlerror();

    if ( NULL == pError )
    {
        mpFnCloseDevice = (COM_DriverDeviceCloseFn)dlsym( mLibraryHandle, "COM_DriverDeviceClose" );
        pError = dlerror();
    }

    if ( NULL == pError )
    {
        mpFnSendMessage = (COM_DriverSendMessageFn)dlsym( mLibraryHandle, "COM_DriverSendMessage" );
        pError = dlerror();
    }

    if ( NULL == pError )
    {
        mpFnReceiveMessage = (COM_DriverReceiveMessageFn)dlsym( mLibraryHandle, "COM_DriverReceiveMessage" );
        pError = dlerror();
    }

    if ( NULL != pError )
    {
        std::string errorString( "Error linking to functions - " );
        errorString += pError;
        errorString += "\n";
        throw std::runtime_error( errorString );
    }
}

//--------------------------------------------------------------------------------------------------
CanDriver::~CanDriver()
{
    dlclose( mLibraryHandle );
}


