//------------------------------------------------------------------------------
// File: CanChannel.cpp
// Desc: A class for handling communications on a single CAN channel
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <boost/thread/thread_time.hpp>

#include "CanChannel.h"

//------------------------------------------------------------------------------
enum eBroadcastFunctionCode
{
    eBFC_NMT = 0,
    eBFC_SYNC = 1,
    eBFC_TIME = 2
};

// NOTE: // RX and TX are relative to the node, so use RX when sending a message
// to a node and TX when getting a message back from a node
enum ePeerToPeerFunctionCodeMask
{
    ePTPFC_EMCY = 1,
    ePTPFC_PDO1_TX = 3,
    ePTPFC_PDO1_RX,         
    ePTPFC_PDO2_TX,
    ePTPFC_PDO2_RX,
    ePTPFC_PDO3_TX,
    ePTPFC_PDO3_RX,
    ePTPFC_PDO4_TX,
    ePTPFC_PDO4_RX,
    ePTPFC_PDO_START = ePTPFC_PDO1_TX,
    ePTPFC_PDO_END = ePTPFC_PDO4_RX,
    ePTPFC_SDO_TX,
    ePTPFC_SDO_RX,
    ePTPFC_NMT_ERROR = 14,
};

//------------------------------------------------------------------------------
// CanChannel
//------------------------------------------------------------------------------
CanChannel::~CanChannel()
{
    // Shut down the read thread
    mbShuttingDown = true;
    if ( mReadThread.joinable() )
    {
        mReadThread.join();
    }
    
    // Close down the driver
    if ( NULL != mDriverHandle )
    {
        COM_DriverClose( mDriverHandle );
        mDriverHandle = NULL;
    }
}

//------------------------------------------------------------------------------
CanChannel* CanChannel::OpenCanChannel( const char* deviceName, const char* baudRate,
                                        const COM_CanChannelCallbacks& callbacks )
{
    CanChannel* pChannel = NULL;
    
    COM_DriverHandle driverHandle = COM_DriverOpen( deviceName, baudRate );
    if ( NULL != driverHandle )
    {
        pChannel = new CanChannel();
        pChannel->mDriverHandle = driverHandle;
        pChannel->mCallbacks = callbacks;
        
        // Start up a thread for reading messages from the CANBUS
        pChannel->mReadThread = boost::thread( CanChannel::CanOpenReadThread, pChannel );
    }
    
    return pChannel;
}

//------------------------------------------------------------------------------
void CanChannel::Update()
{
    boost::posix_time::ptime curTime = boost::get_system_time();
    
    // Process NMT slots
    for ( int nmtMessageType = 0; nmtMessageType < eNMT_NumNmtMessageTypes; nmtMessageType++ )
    {
        if ( mNmtSlots[ nmtMessageType ].mbFull )
        {
            SendNmtMessage( mNmtSlots[ nmtMessageType ].mNodeId, (eNmtMessageType)nmtMessageType );
            mNmtSlots[ nmtMessageType ].mbFull = false;
        }
    }
    
    // Process SDO slots
    for ( int nodeId = 1; nodeId < 128; nodeId++ )
    {
        // Process read slot
        if ( ProcessSdoSlot( &(mSdoReadSlots[ nodeId ]), curTime ) )
        {
            SendSdoReadMsg( nodeId, mSdoReadSlots[ nodeId ].mIndex, 
                            mSdoReadSlots[ nodeId ].mSubIndex );
            
            mSdoReadSlots[ nodeId ].mLastSendTime = curTime;
            mSdoReadSlots[ nodeId ].mbSentAtLeastOnce = true;                
        }
        
        // Process write slot
        if ( ProcessSdoSlot( &(mSdoWriteSlots[ nodeId ]), curTime ) )
        {
            SendSdoWriteMsg( nodeId, mSdoWriteSlots[ nodeId ].mIndex, 
                             mSdoWriteSlots[ nodeId ].mSubIndex,
                             mSdoWriteSlots[ nodeId ].mData,
                             mSdoWriteSlots[ nodeId ].mNumBytes );
            
            mSdoReadSlots[ nodeId ].mLastSendTime = curTime;
            mSdoReadSlots[ nodeId ].mbSentAtLeastOnce = true;                
        }
    }
}

//------------------------------------------------------------------------------
bool CanChannel::QueueNmtMessage( uint8_t nodeId, eNmtMessageType messageType )
{
    assert( messageType >= 0 && messageType < eNMT_NumNmtMessageTypes );
    
    bool bQueued = false;
    
    if ( !mNmtSlots[ messageType ].mbFull )
    {
        mNmtSlots[ messageType ].mNodeId = nodeId;
        mNmtSlots[ messageType ].mbFull = true;
        
        bQueued = true;
    }
    
    return bQueued;
}

//------------------------------------------------------------------------------
bool CanChannel::ProcessSdoSlot( SdoSlot* pSlotInOut, 
                     const boost::posix_time::ptime& curTime )
{
    const int32_t MILLISECONDS_BEFORE_RESEND = 1000;
 
    bool bSendMsg = false;
    
    if ( pSlotInOut->mbFull )
    {
        if ( pSlotInOut->mbShouldBeDeleted )
        {
            pSlotInOut->mbFull = false;
        }
        else
        {
            // Check to see if we should send a message
            if ( !pSlotInOut->mbSentAtLeastOnce )
            {
                bSendMsg = true;
            }
            else
            {
                if ( boost::posix_time::time_period( 
                    pSlotInOut->mLastSendTime, curTime ).length().total_milliseconds() >= MILLISECONDS_BEFORE_RESEND )
                {
                    bSendMsg = true;
                }
            }
        }
    }
    
    return bSendMsg;
}
    
//------------------------------------------------------------------------------
void CanChannel::SendNmtMessage( uint8_t nodeId, eNmtMessageType messageType ) const
{    
    COM_CanMessage msg = COM_EMPTY_CAN_MSG;
    msg.mCobId = 0;
    msg.mLength = 2;
    
    switch ( messageType )
    {
        case eNMT_StartRemoteNode:      msg.mData[ 0 ] = 1; break;
        case eNMT_StopRemoteNode:       msg.mData[ 0 ] = 2; break;
        case eNMT_EnterPreOperational:  msg.mData[ 0 ] = 128; break; 
        case eNMT_ResetNode:            msg.mData[ 0 ] = 129; break; 
        case eNMT_ResetCommunication:   msg.mData[ 0 ] = 130; break;
        default:
        {
            assert( false && "Unhandled NMT message type" );
        }
    }
    
    msg.mData[ 1 ] = nodeId;
    
    COM_DriverSendMessage( mDriverHandle, &msg );
}

//------------------------------------------------------------------------------
void CanChannel::SendSdoReadMsg( uint8_t nodeId, uint16_t index, uint8_t subIndex ) const
{
    COM_CanMessage msg = COM_EMPTY_CAN_MSG;
    msg.mCobId = (ePTPFC_SDO_RX << 7) | (nodeId & 0x7F);
    msg.mLength = 4;
    msg.mData[ 0 ] = 0x40;
    *((uint16_t*)&msg.mData[ 1 ]) = index;
    msg.mData[ 3 ] = subIndex;
    
    COM_DriverSendMessage( mDriverHandle, &msg );
}

//------------------------------------------------------------------------------
void CanChannel::SendSdoWriteMsg( uint8_t nodeId, uint16_t index, uint8_t subIndex, 
                                  uint8_t* pData, uint8_t numBytes ) const
{
    assert( numBytes >= 1 && numBytes <= 4 );
    
    COM_CanMessage msg = COM_EMPTY_CAN_MSG;
    msg.mCobId = (ePTPFC_SDO_RX << 7) | (nodeId & 0x7F);
    msg.mLength = 4 + numBytes;
    msg.mData[ 0 ] = 0x20 | ((4 - numBytes) << 2) | 0x03;
    *((uint16_t*)&msg.mData[ 1 ]) = index;
    msg.mData[ 3 ] = subIndex;
    
    for ( int i = 0; i < numBytes; i++ )
    {
        msg.mData[ 4 + i ] = pData[ i ];
    }
    
    COM_DriverSendMessage( mDriverHandle, &msg );
}            

//------------------------------------------------------------------------------
void CanChannel::CanOpenReadThread( CanChannel* pThis )
{
    while ( !pThis->mbShuttingDown )
    {
        // Read the next message from the CANBUS
        COM_CanMessage msg;
        if ( COM_DriverReceiveMessage( pThis->mDriverHandle, &msg ) == 0 )
        {
            uint8_t functionCode = (msg.mCobId >> 7) & 0xF;
            uint8_t nodeId = (uint8_t)(msg.mCobId & 0x7F);
            
            if ( 0 == nodeId )
            {
                // Broadcast message
            }
            else
            {
                // Peer to peer message
                if ( ePTPFC_EMCY == functionCode )  // Heartbeat
                {
                    if ( NULL != pThis->mCallbacks.mPostEmergencyCB )
                    {
                        uint16_t errCode = *((uint16_t*)(&msg.mData[ 0 ]));
                        uint8_t errReg = msg.mData[ 2 ];
                        pThis->mCallbacks.mPostEmergencyCB( (COM_CanChannelHandle)pThis, nodeId, errCode, errReg );
                    } 
                }
                else if ( ePTPFC_NMT_ERROR == functionCode )  // Heartbeat
                {
                    uint8_t heartbeatState = (msg.mData[ 0 ] & 0x7F);
                    switch ( heartbeatState )
                    {
                        case 0:         // Boot-up
                        {
                            if ( NULL != pThis->mCallbacks.mPostSlaveBootupCB )
                            {
                                pThis->mCallbacks.mPostSlaveBootupCB( (COM_CanChannelHandle)pThis, nodeId );
                            }
                            break;
                        }
                        case 4:     // Stopped
                        case 5:     // Operational
                        case 127:   // Pre-operational
                        {
                            break;
                        }
                    }
                }
            }
        }
    }
    
    printf( "Shutting down read thread\n" );
}
                                   
//------------------------------------------------------------------------------
CanChannel::CanChannel()
    : mDriverHandle( NULL ),
    mbShuttingDown( false )
{
    // Clear data structures
    memset( &mCallbacks, 0, sizeof( mCallbacks ) );
    memset( &mNmtSlots, 0, sizeof( mNmtSlots ) );
    memset( &mSdoReadSlots, 0, sizeof( mSdoReadSlots ) );
    memset( &mSdoWriteSlots, 0, sizeof( mSdoWriteSlots ) );
}
