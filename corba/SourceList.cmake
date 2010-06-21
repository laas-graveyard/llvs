IF(OMNIORB4_FOUND)

  SET(IDL_FILES "")

  SET(IDL_INCLUDE_DIR "${LLVS_SOURCE_DIR}/corba")
 
  #--------------------------------------------------------
  #  Common
  #--------------------------------------------------------

  SET(COMMON_IDL_NAME ${LLVS_SOURCE_DIR}/corba/common.idl)
  
  MESSAGE(STATUS ${IDL_INCLUDE_DIR})

  SET(common_CPP  "${LLVS_SOURCE_DIR}/src/commonSK.cc")
  SET(common_Header "${LLVS_SOURCE_DIR}/src/common.hh")

	 ADD_IDL_FILES(
        ${COMMON_IDL_NAME}
        SOURCE_SUFFIX SK.cc
        HEADER_SUFFIX .hh
        DESTINATION   ${LLVS_SOURCE_DIR}/src
        PATHS         ${IDL_INCLUDE_DIR}
  )

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${common_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${common_Header})


  LIST(APPEND IDL_FILES "${LLVS_SOURCE_DIR}/corba/common.idl")
  #--------------------------------------------------------
  #  BREP
  #--------------------------------------------------------

  SET(BRep_CPP  "${LLVS_SOURCE_DIR}/src/BRepSK.cc")
  SET(BRep_Header "${LLVS_SOURCE_DIR}/src/BRep.hh")


	 ADD_IDL_FILES(
        ${LLVS_SOURCE_DIR}/corba/BRep.idl
        SOURCE_SUFFIX SK.cc
        HEADER_SUFFIX .hh
        DESTINATION   ${LLVS_SOURCE_DIR}/src
        PATHS         ${IDL_INCLUDE_DIR}
  )
  
	SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${BRep_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${BRep_Header})


  LIST(APPEND IDL_FILES "${LLVS_SOURCE_DIR}/corba/BRep.idl")
  #--------------------------------------------------------
  #  STEREO VISION
  #--------------------------------------------------------

  SET(StereoVision_CPP  "${LLVS_SOURCE_DIR}/src/StereoVisionSK.cc")
  SET(StereoVision_Header "${LLVS_SOURCE_DIR}/src/StereoVision.hh")

	 ADD_IDL_FILES(
        ${LLVS_SOURCE_DIR}/corba/StereoVision.idl
        SOURCE_SUFFIX SK.cc
        HEADER_SUFFIX .hh
        DESTINATION   ${LLVS_SOURCE_DIR}/src
        PATHS         ${IDL_INCLUDE_DIR}
  )

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${StereoVision_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${StereoVision_Header})

  LIST(APPEND IDL_FILES "${LLVS_SOURCE_DIR}/corba/StereoVision.idl")
  #--------------------------------------------------------
  #  VISIONSENSOR 
  #--------------------------------------------------------


  SET(visionsensor_CPP  "${LLVS_SOURCE_DIR}/src/visionsensorSK.cc")
  SET(visionsensor_Header "${LLVS_SOURCE_DIR}/src/visionsensor.hh")

  SET(VISIONSENSOR_IDL_NAME ${LLVS_SOURCE_DIR}/corba/visionsensor.idl)

	 ADD_IDL_FILES(
        ${VISIONSENSOR_IDL_NAME}
        SOURCE_SUFFIX SK.cc
        HEADER_SUFFIX .hh
        DESTINATION   ${LLVS_SOURCE_DIR}/src
        PATHS         ${IDL_INCLUDE_DIR}
  )

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${visionsensor_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${visionsensor_Header})

  LIST(APPEND IDL_FILES "${LLVS_SOURCE_DIR}/corba/visionsensor.idl")
  #--------------------------------------------------------
  #  SCENE
  #--------------------------------------------------------


  SET(Scene_CPP  "${LLVS_SOURCE_DIR}/src/SceneSK.cc")
  SET(Scene_Header "${LLVS_SOURCE_DIR}/src/Scene.hh")

	 ADD_IDL_FILES(
        ${LLVS_SOURCE_DIR}/corba/Scene.idl
        SOURCE_SUFFIX SK.cc
        HEADER_SUFFIX .hh
        DESTINATION   ${LLVS_SOURCE_DIR}/src
        PATHS         ${IDL_INCLUDE_DIR}
  )

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${Scene_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${Scene_Header})


  LIST(APPEND IDL_FILES "${LLVS_SOURCE_DIR}/corba/Scene.idl")
  #--------------------------------------------------------
  #  GET GYRO AND ACCELEROMETER
  #--------------------------------------------------------



  SET(GetGyroAndAccelerometer_CPP  "${LLVS_SOURCE_DIR}/src/GetGyroAndAccelerometerSK.cc")
  SET(GetGyroAndAccelerometer_Header "${LLVS_SOURCE_DIR}/src/GetGyroAndAccelerometer.hh")

	 ADD_IDL_FILES(
        ${LLVS_SOURCE_DIR}/corba/GetGyroAndAccelerometer.idl
        SOURCE_SUFFIX SK.cc
        HEADER_SUFFIX .hh
        DESTINATION   ${LLVS_SOURCE_DIR}/src
        PATHS         ${IDL_INCLUDE_DIR}
  )

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${GetGyroAndAccelerometer_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${GetGyroAndAccelerometer_Header})


  LIST(APPEND IDL_FILES "${LLVS_SOURCE_DIR}/corba/GetGyroAndAccelerometer.idl")
  #--------------------------------------------------------
  #  SERVER COMMAND
  #--------------------------------------------------------


  SET(ServerCommand_CPP  "${LLVS_SOURCE_DIR}/src/ServerCommandSK.cc")
  SET(ServerCommand_Header "${LLVS_SOURCE_DIR}/src/ServerCommand.hh")

  ADD_IDL_FILES(
        ${LLVS_SOURCE_DIR}/corba/ServerCommand.idl
        SOURCE_SUFFIX SK.cc
        HEADER_SUFFIX .hh
        DESTINATION   ${LLVS_SOURCE_DIR}/src
        PATHS         ${IDL_INCLUDE_DIR}
  )

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${ServerCommand_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${ServerCommand_Header})

  LIST(APPEND IDL_FILES "${LLVS_SOURCE_DIR}/corba/ServerCommand.idl")
  #--------------------------------------------------------
  #  LLVS
  #--------------------------------------------------------

  SET(LowLevelVisionSystem_CPP  "${LLVS_SOURCE_DIR}/src/LowLevelVisionSystemSK.cc")
  SET(LowLevelVisionSystem_Header "${LLVS_SOURCE_DIR}/src/LowLevelVisionSystem.hh")
 
	 ADD_IDL_FILES(
        ${LLVS_SOURCE_DIR}/corba/LowLevelVisionSystem.idl
        SOURCE_SUFFIX SK.cc
        HEADER_SUFFIX .hh
        DESTINATION   ${LLVS_SOURCE_DIR}/src
        PATHS         ${IDL_INCLUDE_DIR}
  )

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${LowLevelVisionSystem_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${LowLevelVisionSystem_Header})

  LIST(APPEND IDL_FILES "${LLVS_SOURCE_DIR}/corba/LowLevelVisionSystem.idl")
  #--------------------------------------------------------
  #  MODEL TRACKER 
  #--------------------------------------------------------

  SET(ModelTrackerInterface_CPP  "${LLVS_SOURCE_DIR}/src/ModelTrackerInterfaceSK.cc")
  SET(ModelTrackerInterface_Header "${LLVS_SOURCE_DIR}/src/ModelTrackerInterface.hh")
 

	 ADD_IDL_FILES(
        ${LLVS_SOURCE_DIR}/corba/ModelTrackerInterface.idl
        SOURCE_SUFFIX SK.cc
        HEADER_SUFFIX .hh
        DESTINATION   ${LLVS_SOURCE_DIR}/src
        PATHS         ${IDL_INCLUDE_DIR}
  )

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${ModelTrackerInterface_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${ModelTrackerInterface_Header})


  LIST(APPEND IDL_FILES "${LLVS_SOURCE_DIR}/corba/ModelTrackerInterface.idl")


  #--------------------------------------------------------
  #  POINT TRACKER 
  #--------------------------------------------------------

  SET(PointTrackerInterface_CPP  "${LLVS_SOURCE_DIR}/src/PointTrackerInterfaceSK.cc")
  SET(PointTrackerInterface_Header "${LLVS_SOURCE_DIR}/src/PointTrackerInterface.hh")
 

	 ADD_IDL_FILES(
        ${LLVS_SOURCE_DIR}/corba/PointTrackerInterface.idl
        SOURCE_SUFFIX SK.cc
        HEADER_SUFFIX .hh
        DESTINATION   ${LLVS_SOURCE_DIR}/src
        PATHS         ${IDL_INCLUDE_DIR}
  )

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${PointTrackerInterface_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${PointTrackerInterface_Header})


  LIST(APPEND IDL_FILES "${LLVS_SOURCE_DIR}/corba/PointTrackerInterface.idl")

  #--------------------------------------------------------
  #  Gather everything
  #--------------------------------------------------------
  SET(LLVS_CXX_FLAGS "${LLVS_CXX_FLAGS} -I${LLVS_SOURCE_DIR}/src")





ENDIF(OMNIORB4_FOUND)
