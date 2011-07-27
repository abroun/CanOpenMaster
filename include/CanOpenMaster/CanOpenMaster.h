//------------------------------------------------------------------------------
// File: CanOpenMaster.h
// Desc: A library for implementing a simple master node on a CANBUS network
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#ifndef CAN_OPEN_MASTER_H
#define CAN_OPEN_MASTER_H

//------------------------------------------------------------------------------
class COM_CanChannel
{
};

//------------------------------------------------------------------------------
COM_CanChannel* COM_OpenChannel();
void COM_CloseChannel( COM_CanChannel** ppChannel );

#endif // CAN_OPEN_MASTER_H