//------------------------------------------------------------------------------
// File: CanOpenMaster.cpp
// Desc: A library for implementing a simple master node on a CANBUS network
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include <list>
#include <stdlib.h>

#include <boost/thread.hpp>

#include "CanOpenMaster/CanOpenMaster.h"
#include "internal/CanChannel.h"

//------------------------------------------------------------------------------
static bool gbInitialised = false;
static bool gbShuttingDown = false;
static CanChannel* gpChannelBeingOpened = NULL;
static CanChannel* gpChannelToBeClosed = NULL;
static std::list<CanChannel*> gCanChannels;
static boost::thread gCanOpenMasterThread;

//------------------------------------------------------------------------------
void CanOpenMasterThread()
{
    static const uint32_t UPDATE_TIME_MS = (uint32_t)(1000.0 *(1.0f/100.0f));   // Time allowed for one loop to run at 100fps
    
    while ( !gbShuttingDown )
    {
        boost::posix_time::ptime startTime = boost::get_system_time();
        
        // Add a new channel if needed
        if ( NULL != gpChannelBeingOpened )
        {
            gCanChannels.push_back( gpChannelBeingOpened );
            gpChannelBeingOpened = NULL;
        }
        
        // Close a channel if needed
        if ( NULL != gpChannelToBeClosed )
        {
            gCanChannels.remove( gpChannelToBeClosed );
            
            delete gpChannelToBeClosed;
            gpChannelToBeClosed = NULL;
        }
        
        // Update any open channels
        for ( std::list<CanChannel*>::iterator iter = gCanChannels.begin(); iter != gCanChannels.end(); iter++ )
        {
            (*iter)->Update();
        }
        
        boost::posix_time::ptime idealEndTime = startTime + boost::posix_time::milliseconds( UPDATE_TIME_MS );
        boost::thread::sleep( idealEndTime );
    }
}

//------------------------------------------------------------------------------
bool isResponsive()
{
    return gbInitialised && !gbShuttingDown;
}

//------------------------------------------------------------------------------
bool COM_Init()
{
    if ( !gbInitialised )
    {
        // Setup module variables
        gbShuttingDown = false;
        gpChannelBeingOpened = NULL;
        gpChannelToBeClosed = NULL;
        gCanChannels.clear();
        gCanOpenMasterThread = boost::thread( CanOpenMasterThread );
        
        gbInitialised = true;
    }
    
    return gbInitialised;
}

//------------------------------------------------------------------------------
void COM_Deinit()
{
    if ( gbInitialised )
    {
        gbShuttingDown = true;
        
        if ( gCanOpenMasterThread.joinable() )
        {
            gCanOpenMasterThread.join();
        }
        
        // Close any channels that are still open
        for ( std::list<CanChannel*>::iterator iter = gCanChannels.begin(); iter != gCanChannels.end(); iter++ )
        {
            delete (*iter);
        }
        gCanChannels.clear();
        
        gbInitialised = false;
    }
}

//------------------------------------------------------------------------------
COM_CanChannelHandle COM_OpenChannel( const char* deviceName, 
                                      const char* baudRate,
                                      const COM_CanChannelCallbacks& callbacks )
{
    if ( !gbInitialised )
    {
        return NULL;
    }
    
    COM_CanChannelHandle handle = NULL;
    
    CanChannel* pChannel = CanChannel::OpenCanChannel( deviceName, baudRate, callbacks );
    if ( NULL != pChannel )
    {
        while ( NULL != gpChannelBeingOpened );  // Block until the variable is free
        gpChannelBeingOpened = pChannel;
        
        handle = (COM_CanChannelHandle)pChannel;
    }
    
    return handle;
}

//------------------------------------------------------------------------------
void COM_CloseChannel( COM_CanChannelHandle* pHandle )
{
    if ( isResponsive() )
    {
        while ( NULL != gpChannelToBeClosed );  // Block until the variable is free
        
        gpChannelToBeClosed = (CanChannel*)(*pHandle);
        
        while ( NULL != gpChannelToBeClosed );  // Block until the channel is closed
        *pHandle = NULL;
    }
}

//------------------------------------------------------------------------------
bool COM_QueueSdoReadMsg( COM_CanChannelHandle handle, uint8_t nodeId, 
                          uint16_t index, uint8_t subIndex, 
                          COM_SdoReadCallback readCB )
{
    bool bQueued = false;
    
    if ( isResponsive() && NULL != handle )
    {
        bQueued = ((CanChannel*)handle)->QueueSdoReadMsg( nodeId, index, subIndex, readCB );
    }
    
    return bQueued;
}

//------------------------------------------------------------------------------
bool COM_QueueSdoWriteMsg( COM_CanChannelHandle handle, uint8_t nodeId,
                           uint16_t index, uint8_t subIndex, 
                           COM_SdoWriteCallback writeCB, 
                           uint8_t* pData, uint8_t numBytes )
{
    bool bQueued = false;
    
    if ( isResponsive() && NULL != handle )
    {
        bQueued = ((CanChannel*)handle)->QueueSdoWriteMsg( nodeId, index, subIndex, 
                                                           writeCB, pData, numBytes );
    }
    
    return bQueued;
}

//------------------------------------------------------------------------------
bool COM_QueueNmtStartRemoteNode( COM_CanChannelHandle handle, uint8_t nodeId )
{
    bool bQueued = false;
    
    if ( isResponsive() && NULL != handle )
    {
        bQueued = ((CanChannel*)handle)->QueueNmtMessage( nodeId, eNMT_StartRemoteNode );
    }
    
    return bQueued;
}

//------------------------------------------------------------------------------
bool COM_QueueNmtStopRemoteNode( COM_CanChannelHandle handle, uint8_t nodeId )
{
    bool bQueued = false;
    
    if ( isResponsive() && NULL != handle )
    {
        bQueued = ((CanChannel*)handle)->QueueNmtMessage( nodeId, eNMT_StopRemoteNode );
    }
    
    return bQueued;
}

//------------------------------------------------------------------------------
bool COM_QueueNmtEnterPreOperational( COM_CanChannelHandle handle, uint8_t nodeId )
{
    bool bQueued = false;
    
    if ( isResponsive() && NULL != handle )
    {
        bQueued = ((CanChannel*)handle)->QueueNmtMessage( nodeId, eNMT_EnterPreOperational );
    }
    
    return bQueued;
}

//------------------------------------------------------------------------------
bool COM_QueueNmtResetNode( COM_CanChannelHandle handle, uint8_t nodeId )
{
    bool bQueued = false;
    
    if ( isResponsive() && NULL != handle )
    {
        bQueued = ((CanChannel*)handle)->QueueNmtMessage( nodeId, eNMT_ResetNode );
    }
    
    return bQueued;
}

//------------------------------------------------------------------------------
bool COM_QueueNmtResetCommunication( COM_CanChannelHandle handle, uint8_t nodeId )
{
    bool bQueued = false;
    
    if ( isResponsive() && NULL != handle )
    {
        bQueued = ((CanChannel*)handle)->QueueNmtMessage( nodeId, eNMT_ResetCommunication );
    }
    
    return bQueued;
}



