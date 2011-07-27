cmake_minimum_required( VERSION 2.8 )
PROJECT( canopenmaster )

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include )

#-------------------------------------------------------------------------------
# CanOpenMaster library
#-------------------------------------------------------------------------------
SET( CanOpenMasterFiles 
    src/CanOpenMaster.cpp ) 

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
    CanOpenMaster )
#    pthread rt dl )

INSTALL( TARGETS CanOpenTest
        RUNTIME DESTINATION bin )
