cmake_minimum_required( VERSION 2.8 )
PROJECT( canopenmaster )

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include )

SET( CMAKE_C_FLAGS " -std=c99 -Wall " ) # Overrides default. May not be the best way to set this...
SET( CMAKE_CXX_FLAGS " -fPIC -std=c++0x -Wall " )

#-------------------------------------------------------------------------------
# CanOpenMaster library
#-------------------------------------------------------------------------------
SET( CanOpenMasterFiles 
    src/CanOpenMaster.cpp
    src/internal/CanChannel.cpp
    src/internal/CanDriver.cpp
    ) 

ADD_LIBRARY( CanOpenMaster ${CanOpenMasterFiles} )
ADD_LIBRARY( CanOpenMasterShared SHARED ${CanOpenMasterFiles} )
SET_TARGET_PROPERTIES( CanOpenMasterShared 
    PROPERTIES OUTPUT_NAME CanOpenMaster )
    
TARGET_LINK_LIBRARIES( CanOpenMaster dl )
TARGET_LINK_LIBRARIES( CanOpenMasterShared dl ) 

INSTALL( TARGETS CanOpenMaster
        ARCHIVE DESTINATION lib )
INSTALL( TARGETS CanOpenMasterShared
        LIBRARY DESTINATION lib )

INSTALL( DIRECTORY ${PROJECT_SOURCE_DIR}/include/CanOpenMaster DESTINATION include
          FILES_MATCHING PATTERN "*.h" )
          
#-------------------------------------------------------------------------------
# CanUSB driver
#-------------------------------------------------------------------------------

# Only make the dirver if we can find the FTDI library
FIND_FILE( FTDI_HEADER ftdi.h )

IF( FTDI_HEADER )
	SET( CanUSBDriverFiles 
	    drivers/canusb/canusb.c
	    drivers/common/Logging.c
	    drivers/common/RollingBuffer.c
	    ) 
	
	ADD_LIBRARY( CanUSBDriverShared SHARED ${CanUSBDriverFiles} )
	SET_TARGET_PROPERTIES( CanUSBDriverShared 
	    PROPERTIES OUTPUT_NAME CanUSBDriver )
	    
	TARGET_LINK_LIBRARIES( CanUSBDriverShared ftdi )
	
	INSTALL( TARGETS CanUSBDriverShared
	        LIBRARY DESTINATION lib ) 
ENDIF()
        
#-------------------------------------------------------------------------------
# SocketCAN driver
#-------------------------------------------------------------------------------
SET( SocketCanDriverFiles 
    drivers/socketcan/socketcan.c
    drivers/common/Logging.c
    drivers/common/RollingBuffer.c
    ) 

ADD_LIBRARY( SocketCanDriverShared SHARED ${SocketCanDriverFiles} )
SET_TARGET_PROPERTIES( SocketCanDriverShared 
    PROPERTIES OUTPUT_NAME SocketCanDriver )
    
TARGET_LINK_LIBRARIES( SocketCanDriverShared )

INSTALL( TARGETS SocketCanDriverShared
        LIBRARY DESTINATION lib )

#-------------------------------------------------------------------------------
# Example application
#-------------------------------------------------------------------------------
ADD_EXECUTABLE( CanOpenTest 
    examples/CanOpenTest/CanOpenTest.cpp )

TARGET_LINK_LIBRARIES( CanOpenTest 
    CanOpenMaster
    boost_thread
    )
#    pthread rt dl )

INSTALL( TARGETS CanOpenTest
        RUNTIME DESTINATION bin )
