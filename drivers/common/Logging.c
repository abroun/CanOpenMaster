//--------------------------------------------------------------------------------------------------
// File: Logging.c
// Desc: Logging functionality for drivers
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
#include "Logging.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

//--------------------------------------------------------------------------------------------------
static eVerbosity gVerbosity = eV_Error;

//--------------------------------------------------------------------------------------------------
void LogSetVerbosity( eVerbosity verbosity )
{
    gVerbosity = verbosity;
}

//--------------------------------------------------------------------------------------------------
void LogMsg( eVerbosity verbosity, const char* formatString, ... )
{
    if ( verbosity <= gVerbosity )
    {
        va_list argList;
        va_start( argList, formatString );

        vprintf( formatString, argList );

        va_end( argList );
    }
}
