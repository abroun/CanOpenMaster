//------------------------------------------------------------------------------
// File: RollingBuffer.h
// Desc: A simple rolling buffer.
//------------------------------------------------------------------------------
 
//------------------------------------------------------------------------------
#ifndef ROLLING_BUFFER_H
#define ROLLING_BUFFER_H

//------------------------------------------------------------------------------
#include <stdbool.h>

//------------------------------------------------------------------------------    
typedef struct 
{
    unsigned char* mpBuffer;
    unsigned int mBufferSize;
    unsigned int mDataStartIdx;
    unsigned int mDataSize;
} RollingBuffer;

//------------------------------------------------------------------------------    
void RB_Init( RollingBuffer* pBuffer, unsigned char* pDataBuffer, unsigned int bufferSize );
void RB_Deinit( RollingBuffer* pBuffer );
    
// Information about the buffer
unsigned int RB_GetSize( const RollingBuffer* pBuffer );
unsigned int RB_GetNumBytesInBuffer( const RollingBuffer* pBuffer );
unsigned int RB_GetFreeSpace( const RollingBuffer* pBuffer );
    
// Clears all data from the buffer
void RB_Clear( RollingBuffer* pBuffer );
    
// Reads data from the buffer and advances the read pointer.
// Returns the number of bytes that were actually read.
unsigned int RB_ReadBytes( RollingBuffer* pBuffer, unsigned char* pDataOut, unsigned int numBytes );
    
// Similar to ReadBytes but doesn't advance the buffer
// Returns the number of bytes that were actually read.
unsigned int RB_PeekAtBytes( const RollingBuffer* pBuffer, unsigned char* pDataOut, unsigned int numBytes );
    
// Advances the read pointer by at most numBytes
void RB_AdvanceBuffer( RollingBuffer* pBuffer, unsigned int numBytes );
    
// Attempts to add numBytes of data to the buffer. If there isn't
// enough space in the buffer then it returns false. Otherwise it
// returns true.
bool RB_TryToAddBytes( RollingBuffer* pBuffer, const unsigned char* pData, unsigned int numBytes );


#endif // ROLLING_BUFFER_H