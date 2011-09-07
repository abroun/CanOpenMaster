//------------------------------------------------------------------------------
// File: CanChannel.h
// Desc: A class for handling communications on a single CAN channel
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#ifndef CAN_CHANNEL_H
#define CAN_CHANNEL_H

//------------------------------------------------------------------------------
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

#include "CanOpenMaster/CanOpenMaster.h"
#include "CanOpenMaster/can.h"

//------------------------------------------------------------------------------
enum eNmtMessageType
{
    eNMT_StartRemoteNode,
    eNMT_StopRemoteNode,
    eNMT_EnterPreOperational,
    eNMT_ResetNode,
    eNMT_ResetCommunication,
    
    eNMT_NumNmtMessageTypes
};

//------------------------------------------------------------------------------
struct NmtSlot
{
    uint8_t mNodeId;
    
    // Flags
    bool mbFull;                    // Set by user, cleared by CanOpenMaster
 };

//------------------------------------------------------------------------------
enum eSdoSlotType
{
    eSST_Read,
    eSST_Write
};

//------------------------------------------------------------------------------
struct SdoSlot
{
    uint16_t mIndex;
    uint8_t mSubIndex;
    boost::posix_time::ptime mLastSendTime;
    
    // Flags
    // TODO: I'm assuming that reading from or writing to a flag value will be
    // atomic and that race conditions will be avoided because one thread takes
    // the responsibility for setting it and the other takes the responsibility
    // for clearing it
    bool mbFull;                    // Set by user, cleared by CanOpenMaster
    bool mbShouldBeDeleted;         // Set by user, cleared by CanOpenMaster
    bool mbSentAtLeastOnce;         // Set by CanOpenMaster, cleared by CanOpenMaster
};

//------------------------------------------------------------------------------
struct SdoReadSlot : SdoSlot
{
    COM_SdoReadCallback mReadCB;
};

//------------------------------------------------------------------------------
struct SdoWriteSlot : SdoSlot
{
    uint8_t mData[ 4 ];
    uint8_t mNumBytes;
    COM_SdoWriteCallback mWriteCB;
};

//------------------------------------------------------------------------------
class CanChannel
{
    //--------------------------------------------------------------------------
    public: virtual ~CanChannel();
    
    public: static CanChannel* OpenCanChannel( 
        const char* deviceName, const char* baudRate,
        const COM_CanChannelCallbacks& callbacks );

    //--------------------------------------------------------------------------
    public: void Update();
         
    //--------------------------------------------------------------------------
    // This routine attempts to queue an NMT message and returns false if the
    // slot for the NMT message type is full
    public: bool QueueNmtMessage( uint8_t nodeId, eNmtMessageType messageType );
    
    //--------------------------------------------------------------------------
    // These routines attempt to queue SDO messages and return false if there
    // is already a message pending for that node and message type
    public: bool QueueSdoReadMsg( uint8_t nodeId, uint16_t index, uint8_t subIndex, 
        COM_SdoReadCallback readCB );
    public: bool QueueSdoWriteMsg( uint8_t nodeId, uint16_t index, uint8_t subIndex, 
        COM_SdoWriteCallback writeCB, const uint8_t* pData, uint8_t numBytes );
    
    //--------------------------------------------------------------------------    
    // Processes a SDO slot and returns true if a message should be sent
    private: bool ProcessSdoSlot( SdoSlot* pSlotInOut, 
        const boost::posix_time::ptime& curTime );
    
    //--------------------------------------------------------------------------
    // Message sending routines
    private: void SendNmtMessage( uint8_t nodeId, eNmtMessageType messageType ) const;
    
    private: void SendSdoReadMsg( uint8_t nodeId, uint16_t index, uint8_t subIndex ) const;
    private: void SendSdoWriteMsg( uint8_t nodeId, uint16_t index, uint8_t subIndex, 
        uint8_t* pData, uint8_t numBytes ) const;
    
    // Process messages that come from the CANBUS
    private: void ProcessMessages();
        
    // Thread for reading from the CANBUS
    private: static void CanOpenReadThread( CanChannel* pThis );
    
    // Keep constructors private
    private: CanChannel();
    private: CanChannel( const CanChannel& channel );
    
    //--------------------------------------------------------------------------    
    // Members                             
    private: COM_CanChannelCallbacks mCallbacks;
    private: COM_DriverHandle mDriverHandle;
    private: bool mbShuttingDown;
    private: boost::thread mReadThread;
    
    private: NmtSlot mNmtSlots[ eNMT_NumNmtMessageTypes ];
    private: SdoReadSlot mSdoReadSlots[ 128 ];
    private: SdoWriteSlot mSdoWriteSlots[ 128 ];
    
    private: static const uint32_t MESSAGE_BUFFER_SIZE = 1024;
    private: COM_CanMessage mMessageBuffer[ MESSAGE_BUFFER_SIZE ];
    private: uint32_t mMessageProduceCount;
    private: uint32_t mMessageConsumeCount;
};

#endif // CAN_CHANNEL_H