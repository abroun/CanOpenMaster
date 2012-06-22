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
    
    // Close down the device
    if ( NULL != mDeviceHandle )
    {
        mpDriver->closeDevice( mDeviceHandle );
        mDeviceHandle = NULL;
    }
}

//------------------------------------------------------------------------------
CanChannel* CanChannel::OpenCanChannel( CanDriver* pDriver, const char* deviceName, const char* baudRate,
                                        const COM_CanChannelCallbacks& callbacks )
{
    CanChannel* pChannel = NULL;
    
    COM_DeviceHandle deviceHandle = pDriver->openDevice( deviceName, baudRate );
    if ( NULL != deviceHandle )
    {
        pChannel = new CanChannel();
        pChannel->mpDriver = pDriver;
        pChannel->mDeviceHandle = deviceHandle;
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
            
            mSdoWriteSlots[ nodeId ].mLastSendTime = curTime;
            mSdoWriteSlots[ nodeId ].mbSentAtLeastOnce = true;                
        }
    }
    
    // Process messages that have come from the CANBUS
    ProcessMessages();
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
bool CanChannel::QueueSdoReadMsg( uint8_t nodeId, uint16_t index, 
                                  uint8_t subIndex, COM_SdoReadCallback readCB )
{
    assert( nodeId < 128 );
    
    bool bQueued = false;
    
    if ( !mSdoReadSlots[ nodeId ].mbFull )
    {
        mSdoReadSlots[ nodeId ].mIndex = index;
        mSdoReadSlots[ nodeId ].mSubIndex = subIndex;
        mSdoReadSlots[ nodeId ].mbSentAtLeastOnce = false;
        mSdoReadSlots[ nodeId ].mbShouldBeDeleted = false;
        mSdoReadSlots[ nodeId ].mReadCB = readCB;
        
        mSdoReadSlots[ nodeId ].mbFull = true;
        
        bQueued = true;
    }
    
    return bQueued;
}

//------------------------------------------------------------------------------
bool CanChannel::QueueSdoWriteMsg( uint8_t nodeId, uint16_t index, 
                                   uint8_t subIndex, COM_SdoWriteCallback writeCB,
                                   const uint8_t* pData, uint8_t numBytes )
{
    assert( nodeId < 128 );
    assert( numBytes >= 1 && numBytes <= 4 );
    
    bool bQueued = false;
    
    if ( !mSdoWriteSlots[ nodeId ].mbFull )
    {
        mSdoWriteSlots[ nodeId ].mIndex = index;
        mSdoWriteSlots[ nodeId ].mSubIndex = subIndex;
        mSdoWriteSlots[ nodeId ].mbSentAtLeastOnce = false;
        mSdoWriteSlots[ nodeId ].mbShouldBeDeleted = false;
        
        for ( uint8_t i = 0; i < numBytes; i++ )
        {
            mSdoWriteSlots[ nodeId ].mData[ i ] = pData[ i ];
        }
        mSdoWriteSlots[ nodeId ].mNumBytes = numBytes;
        
        mSdoWriteSlots[ nodeId ].mWriteCB = writeCB;
        
        mSdoWriteSlots[ nodeId ].mbFull = true;
        
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
    
    mpDriver->sendMessage( mDeviceHandle, &msg );
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
    
    mpDriver->sendMessage( mDeviceHandle, &msg );
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
    
    mpDriver->sendMessage( mDeviceHandle, &msg );
}            

//------------------------------------------------------------------------------
void CanChannel::ProcessMessages()
{
    const uint32_t MAX_NUM_MESSAGES_TO_PROCESS_PER_FRAME = 32;
    
    uint32_t numMessagesProcessed = 0;
    while ( mMessageProduceCount - mMessageConsumeCount > 0     
        && numMessagesProcessed < MAX_NUM_MESSAGES_TO_PROCESS_PER_FRAME )
    {
        // We have messages to consume and we've not yet processed the maximum
        // number of messages this frame
        
        // Get the message
        COM_CanMessage msg = mMessageBuffer[ mMessageConsumeCount % MESSAGE_BUFFER_SIZE ];
        mMessageConsumeCount++;
        
        // Process the message
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
                if ( NULL != mCallbacks.mPostEmergencyCB )
                {
                    uint16_t errCode = *((uint16_t*)(&msg.mData[ 0 ]));
                    uint8_t errReg = msg.mData[ 2 ];
                    mCallbacks.mPostEmergencyCB( (COM_CanChannelHandle)this, nodeId, errCode, errReg );
                } 
            }
            else if ( ePTPFC_NMT_ERROR == functionCode )  // Heartbeat
            {
                uint8_t heartbeatState = (msg.mData[ 0 ] & 0x7F);
                switch ( heartbeatState )
                {
                    case 0:         // Boot-up
                    {
                        if ( NULL != mCallbacks.mPostSlaveBootupCB )
                        {
                            mCallbacks.mPostSlaveBootupCB( (COM_CanChannelHandle)this, nodeId );
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
            else if ( ePTPFC_SDO_TX == functionCode )   // SDO reply
            {
                uint8_t serverCommandSpecifier = (msg.mData[ 0 ] >> 5) & 0x07;
                if ( 2 == serverCommandSpecifier )
                {
                    // We're getting the reply to an SDO read
                    if ( mSdoReadSlots[ nodeId ].mbFull )
                    {
                        uint8_t numBytes = 4 - ((msg.mData[ 0 ] >> 2) & 0x03);
                        
                        COM_SdoReadCallback readCB = mSdoReadSlots[ nodeId ].mReadCB;
                        mSdoReadSlots[ nodeId ].mbFull = false; // Free the slot
                        
                        if ( NULL != readCB )
                        {
                            // Send the data up to the user
                            readCB( (COM_CanChannelHandle)this, nodeId, &(msg.mData[ 4 ]), numBytes );
                        }
                    }
                }
                else if ( 3 == serverCommandSpecifier )
                {
                    // We're getting the reply to an SDO write
                    if ( mSdoWriteSlots[ nodeId ].mbFull )
                    {
                        COM_SdoWriteCallback writeCB = mSdoWriteSlots[ nodeId ].mWriteCB;
                        mSdoWriteSlots[ nodeId ].mbFull = false; // Free the slot
                        
                        if ( NULL != writeCB )
                        {
                            // Let the user know that the write has completed
                            writeCB( (COM_CanChannelHandle)this, nodeId );
                        }
                    }
                }
            }
        }
        
        // Record the fact that we've processed a message
        numMessagesProcessed++;
    }
}

//------------------------------------------------------------------------------
void CanChannel::CanOpenReadThread( CanChannel* pThis )
{
	// This limit is set to stop the read thread from swamping the CPU. At the
	// maximum baud rate of 1MBs it should be sufficient for messages with a
	// 4 byte payload
	const uint32_t MAX_NUM_READS_PER_SECOND = 16000;
	const uint32_t MICROSECONDS_FOR_MESSAGE = 1000000 / MAX_NUM_READS_PER_SECOND;

    while ( !pThis->mbShuttingDown )
    {
        if ( MESSAGE_BUFFER_SIZE == pThis->mMessageProduceCount - pThis->mMessageConsumeCount )
        {
            // Spin whilst the message buffer is full
            boost::thread::yield();
            continue;
        }
 
        // Read the next message from the CANBUS
        COM_CanMessage msg;
        if ( pThis->mpDriver->receiveMessage( pThis->mDeviceHandle, &msg ) == 0 )
        {
            pThis->mMessageBuffer[ pThis->mMessageProduceCount % MESSAGE_BUFFER_SIZE ] = msg;
            pThis->mMessageProduceCount++;
        }
        else
        {
            boost::thread::yield();
        }

        boost::thread::sleep( boost::get_system_time()
        	+ boost::posix_time::microseconds( MICROSECONDS_FOR_MESSAGE ) );
    }
    
    printf( "Shutting down read thread\n" );
}
                                   
//------------------------------------------------------------------------------
CanChannel::CanChannel()
    : mDeviceHandle( NULL ),
    mbShuttingDown( false ),
    mMessageProduceCount( 0 ),
    mMessageConsumeCount( 0 )
{
    // Clear data structures
    memset( &mCallbacks, 0, sizeof( mCallbacks ) );
    memset( &mNmtSlots, 0, sizeof( mNmtSlots ) );
    memset( &mSdoReadSlots, 0, sizeof( mSdoReadSlots ) );
    memset( &mSdoWriteSlots, 0, sizeof( mSdoWriteSlots ) );
}
