//--------------------------------------------------------------------------------------------------
// File: Logging.h
// Desc:
//--------------------------------------------------------------------------------------------------

#ifndef LOGGING_H_
#define LOGGING_H_

typedef enum eVerbosity
{
    eV_Invalid = -1,
    eV_Silent = 0,
    eV_Error,
    eV_Warning,
    eV_Info
} eVerbosity;

void LogSetVerbosity( eVerbosity verbosity );
void LogMsg( eVerbosity verbosity, const char* formatString, ... );


#endif // LOGGING_H_
