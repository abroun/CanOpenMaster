//------------------------------------------------------------------------------
// File: RollingBuffer.c
// Desc: A simple rolling buffer implemented using a fixed size array
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "RollingBuffer.h"

//------------------------------------------------------------------------------
// RollingBuffer
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------    
// Helper routine for making sure that indicies stay within the buffer
unsigned int AddOffsetToIdx( const RollingBuffer* pBuffer, unsigned int idx, unsigned int offset )
{
    unsigned int numBytesToEnd = pBuffer->mBufferSize - idx;
    if ( offset < numBytesToEnd )
    {
        return idx + offset;
    }
    else
    {
        // Wrap around and start from the beginning of the buffer
        return offset - numBytesToEnd;
    }
}

//------------------------------------------------------------------------------    
void RB_Init( RollingBuffer* pBuffer, unsigned char* pDataBuffer, unsigned int bufferSize )
{
    pBuffer->mpBuffer = pDataBuffer;
    pBuffer->mBufferSize = bufferSize;
    pBuffer->mDataStartIdx = 0;
    pBuffer->mDataSize = 0;
}

//------------------------------------------------------------------------------    
void RB_Deinit( RollingBuffer* pBuffer )
{   
}
    
//------------------------------------------------------------------------------    
unsigned int RB_GetSize( const RollingBuffer* pBuffer )
{ 
    return pBuffer->mBufferSize; 
}

//------------------------------------------------------------------------------    
unsigned int RB_GetNumBytesInBuffer( const RollingBuffer* pBuffer )
{ 
    return pBuffer->mDataSize; 
}

//------------------------------------------------------------------------------    
unsigned int RB_GetFreeSpace( const RollingBuffer* pBuffer )
{ 
    return pBuffer->mBufferSize - pBuffer->mDataSize; 
}
    
//------------------------------------------------------------------------------    
void RB_Clear( RollingBuffer* pBuffer )
{
    pBuffer->mDataStartIdx = 0;
    pBuffer->mDataSize = 0;
}
    
//------------------------------------------------------------------------------    
unsigned int RB_ReadBytes( RollingBuffer* pBuffer, unsigned char* pDataOut, unsigned int numBytes )
{
    unsigned int numBytesRead = RB_PeekAtBytes( pBuffer, pDataOut, numBytes );
    RB_AdvanceBuffer( pBuffer, numBytesRead );
    
    return numBytesRead;
}
    
//------------------------------------------------------------------------------    
unsigned int RB_PeekAtBytes( const RollingBuffer* pBuffer, unsigned char* pDataOut, unsigned int numBytes )
{
    assert( NULL != pDataOut );
    
    unsigned int numBytesRead = numBytes;
    if ( numBytesRead > pBuffer->mDataSize )
    {
        numBytesRead = pBuffer->mDataSize;
    }
    
    // Test to see if we can do the read all in one go
    unsigned int numContiguousBytesAvailable = pBuffer->mBufferSize - pBuffer->mDataStartIdx;
    if ( numContiguousBytesAvailable >= numBytesRead )
    {
        memcpy( pDataOut, &(pBuffer->mpBuffer[ pBuffer->mDataStartIdx ]), numBytesRead );
    }
    else
    {
        // The read has to be done in 2 parts
        memcpy( pDataOut, &(pBuffer->mpBuffer[ pBuffer->mDataStartIdx ]), numContiguousBytesAvailable );
        
        unsigned int numBytesRemaining = numBytesRead - numContiguousBytesAvailable;
        memcpy( &pDataOut[ numContiguousBytesAvailable ],
                &(pBuffer->mpBuffer[ 0 ]), numBytesRemaining );
    }
    
    return numBytesRead; 
}
    
//------------------------------------------------------------------------------    
void RB_AdvanceBuffer( RollingBuffer* pBuffer, unsigned int numBytes )
{
    unsigned int bytesToAdvance = numBytes;
    if ( bytesToAdvance > pBuffer->mDataSize )
    {
        bytesToAdvance = pBuffer->mDataSize;
    }
    
    pBuffer->mDataStartIdx = AddOffsetToIdx( pBuffer, pBuffer->mDataStartIdx, bytesToAdvance );
    pBuffer->mDataSize -= bytesToAdvance;
}

//------------------------------------------------------------------------------    
bool RB_TryToAddBytes( RollingBuffer* pBuffer, const unsigned char* pData, unsigned int numBytes )
{
    bool bBytesAdded = false;
    
    if ( numBytes < RB_GetFreeSpace( pBuffer ) )
    {
        // Test to see if we can write the data all in one go
        unsigned int endOfDataIdx = AddOffsetToIdx( pBuffer, pBuffer->mDataStartIdx, pBuffer->mDataSize );
        unsigned int numContiguousBytesAvailable = pBuffer->mBufferSize - endOfDataIdx;
        if ( numContiguousBytesAvailable >= numBytes )
        {
            memcpy( &(pBuffer->mpBuffer[ endOfDataIdx ]), pData, numBytes );
        }
        else
        {
            // The write has to be done in 2 parts
            memcpy( &(pBuffer->mpBuffer[ endOfDataIdx ]), pData, numContiguousBytesAvailable );
            
            unsigned int numBytesRemaining = numBytes - numContiguousBytesAvailable;
            memcpy( &(pBuffer->mpBuffer[ 0 ]), 
                    &pData[ numContiguousBytesAvailable ], numBytesRemaining );
        }
        
        pBuffer->mDataSize += numBytes;
        bBytesAdded = true;
    }
    
    return bBytesAdded;
}
