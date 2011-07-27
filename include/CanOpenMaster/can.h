
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void* COM_DriverHandle;

typedef struct 
{
  uint16_t mCobId; /**< message's ID */
  uint8_t mRtr;     /**< remote transmission request. (0 if not rtr message, 1 if rtr message) */
  uint8_t mLength;     /**< message's length (0 to 8) */
  uint8_t mData[ 8 ]; /**< message's datas */
} COM_CanMessage;

#define COM_EMPTY_CAN_MSG { 0, 0, 0, { 0 } }

//------------------------------------------------------------------------------
COM_DriverHandle COM_DriverOpen( const char* deviceName, const char* baudRate );
void COM_DriverClose( COM_DriverHandle handle );

//------------------------------------------------------------------------------
uint8_t COM_DriverSendMessage( COM_DriverHandle handle, COM_CanMessage* pMsg );
uint8_t COM_DriverReceiveMessage( COM_DriverHandle handle, COM_CanMessage* pMsgOut );

#ifdef __cplusplus
}
#endif