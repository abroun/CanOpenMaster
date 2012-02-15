//------------------------------------------------------------------------------
// File: CanOpenMaster.h
// Desc: A library for implementing a simple master node on a CANBUS network
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#ifndef CAN_OPEN_MASTER_H
#define CAN_OPEN_MASTER_H

//------------------------------------------------------------------------------
#include <stdint.h>

//------------------------------------------------------------------------------
typedef void* COM_CanChannelHandle;

typedef void (*COM_HeartbeatErrorCallback)( COM_CanChannelHandle handle, uint8_t error );
typedef void (*COM_PostSyncCallback)( COM_CanChannelHandle handle );
typedef void (*COM_PostTpdoCallback)( COM_CanChannelHandle handle );
typedef void (*COM_PostEmergencyCallback)( COM_CanChannelHandle handle, uint8_t nodeId, uint16_t errCode, uint8_t errReg );
typedef void (*COM_PostSlaveBootupCallback)( COM_CanChannelHandle handle, uint8_t nodeId );

typedef void (*COM_SdoReadCallback)( COM_CanChannelHandle handle, uint8_t nodeId, uint8_t* pData, uint8_t numBytes );
typedef void (*COM_SdoWriteCallback)( COM_CanChannelHandle handle, uint8_t nodeId );

//------------------------------------------------------------------------------
typedef struct
{
    COM_HeartbeatErrorCallback mHeartbeatErrorCB;
    COM_PostSyncCallback mPostSyncCB;
    COM_PostTpdoCallback mPostTpdoCB;
    COM_PostEmergencyCallback mPostEmergencyCB;
    COM_PostSlaveBootupCallback mPostSlaveBootupCB;
} COM_CanChannelCallbacks;

//------------------------------------------------------------------------------
bool COM_Init();
void COM_Deinit();

COM_CanChannelHandle COM_OpenChannel( const char* driverLibraryName,
                                       const char* deviceName,
                                       const char* baudRate,
                                       const COM_CanChannelCallbacks& callbacks );

void COM_CloseChannel( COM_CanChannelHandle* pHandle );

//------------------------------------------------------------------------------
// SDO
//------------------------------------------------------------------------------
// These routines attempt to queue SDO messages and return false if there
// is already a message pending for that node and message type
bool COM_QueueSdoReadMsg( COM_CanChannelHandle handle, uint8_t nodeId, 
                          uint16_t index, uint8_t subIndex, 
                          COM_SdoReadCallback readCB );
bool COM_QueueSdoWriteMsg( COM_CanChannelHandle handle, uint8_t nodeId,
                           uint16_t index, uint8_t subIndex, 
                           COM_SdoWriteCallback writeCB, 
                           const uint8_t* pData, uint8_t numBytes );

//------------------------------------------------------------------------------
// NMT
//------------------------------------------------------------------------------
// These routines will attempt to queue a NMT message for sending and will
// return false if the slot used for queuing a message of that type is already
// full
bool COM_QueueNmtStartRemoteNode( COM_CanChannelHandle handle, uint8_t nodeId );
bool COM_QueueNmtStopRemoteNode( COM_CanChannelHandle handle, uint8_t nodeId );
bool COM_QueueNmtEnterPreOperational( COM_CanChannelHandle handle, uint8_t nodeId );
bool COM_QueueNmtResetNode( COM_CanChannelHandle handle, uint8_t nodeId );
bool COM_QueueNmtResetCommunication( COM_CanChannelHandle handle, uint8_t nodeId );

#endif // CAN_OPEN_MASTER_H
