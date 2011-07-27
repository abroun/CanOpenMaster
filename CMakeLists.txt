cmake_minimum_required( VERSION 2.8 )
PROJECT( canopenmaster )

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include )

SET( CMAKE_C_FLAGS " -std=c99 -Wall " ) # Ovverrides default. May not be the best way to set this...
SET( CMAKE_CXX_FLAGS " -Wall " )

#-------------------------------------------------------------------------------
# CanOpenMaster library
#-------------------------------------------------------------------------------
SET( CanOpenMasterFiles 
    src/CanOpenMaster.cpp
    src/internal/CanChannel.cpp
    drivers/canusb/canusb.c         # TODO: Split drivers out into seperate libraries
    drivers/canusb/RollingBuffer.c
    ) 

ADD_LIBRARY( CanOpenMaster ${CanOpenMasterFiles} )
ADD_LIBRARY( CanOpenMasterShared SHARED ${CanOpenMasterFiles} )
SET_TARGET_PROPERTIES( CanOpenMasterShared 
    PROPERTIES OUTPUT_NAME CanOpenMaster )

INSTALL( TARGETS CanOpenMaster
        ARCHIVE DESTINATION lib )
INSTALL( TARGETS CanOpenMasterShared
        LIBRARY DESTINATION lib )

INSTALL( DIRECTORY ${PROJECT_SOURCE_DIR}/include/CanOpenMaster DESTINATION include
          FILES_MATCHING PATTERN "*.h" )

#-------------------------------------------------------------------------------
# Example application
#-------------------------------------------------------------------------------
ADD_EXECUTABLE( CanOpenTest 
    examples/CanOpenTest/CanOpenTest.cpp )

TARGET_LINK_LIBRARIES( CanOpenTest 
    CanOpenMaster
    boost_thread
    ftdi        # Driver libraries
    )
#    pthread rt dl )

INSTALL( TARGETS CanOpenTest
        RUNTIME DESTINATION bin )
