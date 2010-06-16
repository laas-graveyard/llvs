IF(OMNIORB4_FOUND)

  SET(IDL_FILES "")

  SET(IDL_INCLUDE_DIR "-I${LLVS_SOURCE_DIR}/corba")
 
  #--------------------------------------------------------
  #  Common
  #--------------------------------------------------------

  SET(COMMON_IDL_NAME ${LLVS_SOURCE_DIR}/corba/common.idl)
  
  MESSAGE(STATUS ${IDL_INCLUDE_DIR})

  SET(common_CPP  "${LLVS_SOURCE_DIR}/src/commonSK.cc")
  SET(common_Header "${LLVS_SOURCE_DIR}/src/common.hh")

  IDLFILERULE(${COMMON_IDL_NAME}
              ${common_CPP}
              ${common_Header}
              ${LLVS_SOURCE_DIR}/src ${IDL_INCLUDE_DIR})

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${common_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${common_Header})


  LIST(APPEND IDL_FILES "${LLVS_SOURCE_DIR}/corba/common.idl")
  #--------------------------------------------------------
  #  BREP
  #--------------------------------------------------------

  SET(BRep_CPP  "${LLVS_SOURCE_DIR}/src/BRepSK.cc")
  SET(BRep_Header "${LLVS_SOURCE_DIR}/src/BRep.hh")

  IDLFILERULE(${LLVS_SOURCE_DIR}/corba/BRep.idl
              ${BRep_CPP}
              ${BRep_Header}
              ${LLVS_SOURCE_DIR}/src ${IDL_INCLUDE_DIR})

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${BRep_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${BRep_Header})


  LIST(APPEND IDL_FILES "${LLVS_SOURCE_DIR}/corba/BRep.idl")
  #--------------------------------------------------------
  #  STEREO VISION
  #--------------------------------------------------------

  SET(StereoVision_CPP  "${LLVS_SOURCE_DIR}/src/StereoVisionSK.cc")
  SET(StereoVision_Header "${LLVS_SOURCE_DIR}/src/StereoVision.hh")


  IDLFILERULE(${LLVS_SOURCE_DIR}/corba/StereoVision.idl
              ${StereoVision_CPP}
              ${StereoVision_Header}
              ${LLVS_SOURCE_DIR}/src ${IDL_INCLUDE_DIR})

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${StereoVision_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${StereoVision_Header})

  LIST(APPEND IDL_FILES "${LLVS_SOURCE_DIR}/corba/StereoVision.idl")
  #--------------------------------------------------------
  #  VISIONSENSOR 
  #--------------------------------------------------------


  SET(visionsensor_CPP  "${LLVS_SOURCE_DIR}/src/visionsensorSK.cc")
  SET(visionsensor_Header "${LLVS_SOURCE_DIR}/src/visionsensor.hh")

  SET(VISIONSENSOR_IDL_NAME ${LLVS_SOURCE_DIR}/corba/visionsensor.idl)

  IDLFILERULE(${VISIONSENSOR_IDL_NAME}
              ${visionsensor_CPP}
              ${visionsensor_Header}
              ${LLVS_SOURCE_DIR}/src ${IDL_INCLUDE_DIR})

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${visionsensor_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${visionsensor_Header})

  LIST(APPEND IDL_FILES "${LLVS_SOURCE_DIR}/corba/visionsensor.idl")
  #--------------------------------------------------------
  #  SCENE
  #--------------------------------------------------------


  SET(Scene_CPP  "${LLVS_SOURCE_DIR}/src/SceneSK.cc")
  SET(Scene_Header "${LLVS_SOURCE_DIR}/src/Scene.hh")

  IDLFILERULE(${LLVS_SOURCE_DIR}/corba/Scene.idl
              ${Scene_CPP}
              ${Scene_Header}
              ${LLVS_SOURCE_DIR}/src ${IDL_INCLUDE_DIR})

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${Scene_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${Scene_Header})


  LIST(APPEND IDL_FILES "${LLVS_SOURCE_DIR}/corba/Scene.idl")
  #--------------------------------------------------------
  #  GET GYRO AND ACCELEROMETER
  #--------------------------------------------------------



  SET(GetGyroAndAccelerometer_CPP  "${LLVS_SOURCE_DIR}/src/GetGyroAndAccelerometerSK.cc")
  SET(GetGyroAndAccelerometer_Header "${LLVS_SOURCE_DIR}/src/GetGyroAndAccelerometer.hh")

  IDLFILERULE(${LLVS_SOURCE_DIR}/corba/GetGyroAndAccelerometer.idl
              ${GetGyroAndAccelerometer_CPP}
              ${GetGyroAndAccelerometer_Header}
              ${LLVS_SOURCE_DIR}/src ${IDL_INCLUDE_DIR})

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${GetGyroAndAccelerometer_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${GetGyroAndAccelerometer_Header})


  LIST(APPEND IDL_FILES "${LLVS_SOURCE_DIR}/corba/GetGyroAndAccelerometer.idl")
  #--------------------------------------------------------
  #  SERVER COMMAND
  #--------------------------------------------------------


  SET(ServerCommand_CPP  "${LLVS_SOURCE_DIR}/src/ServerCommandSK.cc")
  SET(ServerCommand_Header "${LLVS_SOURCE_DIR}/src/ServerCommand.hh")

  IDLFILERULE(${LLVS_SOURCE_DIR}/corba/ServerCommand.idl
              ${ServerCommand_CPP}
              ${ServerCommand_Header}
              ${LLVS_SOURCE_DIR}/src ${IDL_INCLUDE_DIR})

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${ServerCommand_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${ServerCommand_Header})

  LIST(APPEND IDL_FILES "${LLVS_SOURCE_DIR}/corba/ServerCommand.idl")
  #--------------------------------------------------------
  #  LLVS
  #--------------------------------------------------------

  SET(LowLevelVisionSystem_CPP  "${LLVS_SOURCE_DIR}/src/LowLevelVisionSystemSK.cc")
  SET(LowLevelVisionSystem_Header "${LLVS_SOURCE_DIR}/src/LowLevelVisionSystem.hh")
 
  IDLFILERULE(${LLVS_SOURCE_DIR}/corba/LowLevelVisionSystem.idl
              ${LowLevelVisionSystem_CPP}
              ${LowLevelVisionSystem_Header}
              ${LLVS_SOURCE_DIR}/src ${IDL_INCLUDE_DIR})

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${LowLevelVisionSystem_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${LowLevelVisionSystem_Header})

  LIST(APPEND IDL_FILES "${LLVS_SOURCE_DIR}/corba/LowLevelVisionSystem.idl")
  #--------------------------------------------------------
  #  MODEL TRACKER 
  #--------------------------------------------------------

  SET(ModelTrackerInterface_CPP  "${LLVS_SOURCE_DIR}/src/ModelTrackerInterfaceSK.cc")
  SET(ModelTrackerInterface_Header "${LLVS_SOURCE_DIR}/src/ModelTrackerInterface.hh")
 

  IDLFILERULE(${LLVS_SOURCE_DIR}/corba/ModelTrackerInterface.idl
              ${ModelTrackerInterface_CPP}
              ${ModelTrackerInterface_Header}
              ${LLVS_SOURCE_DIR}/src ${IDL_INCLUDE_DIR})

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${ModelTrackerInterface_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${ModelTrackerInterface_Header})


  LIST(APPEND IDL_FILES "${LLVS_SOURCE_DIR}/corba/ModelTrackerInterface.idl")


  #--------------------------------------------------------
  #  POINT TRACKER 
  #--------------------------------------------------------

  SET(PointTrackerInterface_CPP  "${LLVS_SOURCE_DIR}/src/PointTrackerInterfaceSK.cc")
  SET(PointTrackerInterface_Header "${LLVS_SOURCE_DIR}/src/PointTrackerInterface.hh")
 

  IDLFILERULE(${LLVS_SOURCE_DIR}/corba/PointTrackerInterface.idl
              ${PointTrackerInterface_CPP}
              ${PointTrackerInterface_Header}
              ${LLVS_SOURCE_DIR}/src ${IDL_INCLUDE_DIR})

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${PointTrackerInterface_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${PointTrackerInterface_Header})


  LIST(APPEND IDL_FILES "${LLVS_SOURCE_DIR}/corba/PointTrackerInterface.idl")

  #--------------------------------------------------------
  #  Gather everything
  #--------------------------------------------------------
  SET(LLVS_CXX_FLAGS "${LLVS_CXX_FLAGS} -I${LLVS_SOURCE_DIR}/src")





ENDIF(OMNIORB4_FOUND)