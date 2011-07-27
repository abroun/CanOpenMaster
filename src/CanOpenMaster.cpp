//------------------------------------------------------------------------------
// File: CanOpenMaster.cpp
// Desc: A library for implementing a simple master node on a CANBUS network
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include <stdlib.h>
#include "CanOpenMaster/CanOpenMaster.h"

//------------------------------------------------------------------------------
COM_CanChannel* COM_OpenChannel()
{
    return new COM_CanChannel();
}

//------------------------------------------------------------------------------
void COM_CloseChannel( COM_CanChannel** ppChannel )
{
    if ( NULL != *ppChannel )
    {
        delete *ppChannel;
        *ppChannel = NULL;
    }
}

