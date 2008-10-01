IF(OPENHRP_FOUND)

  SET(IDL_INCLUDE_DIR "-I${OPENHRP_HOME}/Common/corba")
  
  MESSAGE(STATUS ${IDL_INCLUDE_DIR})

  SET(common_CPP  "${LLVS_SOURCE_DIR}/src/commonSK.cc")
  SET(common_Header "${LLVS_SOURCE_DIR}/src/common.hh")

  IDLFILERULE(${OPENHRP_HOME}/Common/corba/common.idl
              ${common_CPP}
              ${common_Header}
              ${LLVS_SOURCE_DIR}/src ${IDL_INCLUDE_DIR})

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${common_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${common_Header})

  SET(BRep_CPP  "${LLVS_SOURCE_DIR}/src/BRepSK.cc")
  SET(BRep_Header "${LLVS_SOURCE_DIR}/src/BRep.hh")

  IDLFILERULE(${LLVS_SOURCE_DIR}/corba/BRep.idl
              ${BRep_CPP}
              ${BRep_Header}
              ${LLVS_SOURCE_DIR}/src ${IDL_INCLUDE_DIR})

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${BRep_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${BRep_Header})

  SET(StereoVision_CPP  "${LLVS_SOURCE_DIR}/src/StereoVisionSK.cc")
  SET(StereoVision_Header "${LLVS_SOURCE_DIR}/src/StereoVision.hh")

  IDLFILERULE(${LLVS_SOURCE_DIR}/corba/StereoVision.idl
              ${StereoVision_CPP}
              ${StereoVision_Header}
              ${LLVS_SOURCE_DIR}/src ${IDL_INCLUDE_DIR})

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${StereoVision_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${StereoVision_Header})

  SET(visionsensor_CPP  "${LLVS_SOURCE_DIR}/src/visionsensorSK.cc")
  SET(visionsensor_Header "${LLVS_SOURCE_DIR}/src/visionsensor.hh")

  IDLFILERULE(${LLVS_SOURCE_DIR}/corba/visionsensor.idl
              ${visionsensor_CPP}
              ${visionsensor_Header}
              ${LLVS_SOURCE_DIR}/src ${IDL_INCLUDE_DIR})

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${visionsensor_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${visionsensor_Header})

  SET(Scene_CPP  "${LLVS_SOURCE_DIR}/src/SceneSK.cc")
  SET(Scene_Header "${LLVS_SOURCE_DIR}/src/Scene.hh")

  IDLFILERULE(${LLVS_SOURCE_DIR}/corba/Scene.idl
              ${Scene_CPP}
              ${Scene_Header}
              ${LLVS_SOURCE_DIR}/src ${IDL_INCLUDE_DIR})

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${Scene_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${Scene_Header})

  SET(GetGyroAndAccelerometer_CPP  "${LLVS_SOURCE_DIR}/src/GetGyroAndAccelerometerSK.cc")
  SET(GetGyroAndAccelerometer_Header "${LLVS_SOURCE_DIR}/src/GetGyroAndAccelerometer.hh")

  IDLFILERULE(${LLVS_SOURCE_DIR}/corba/GetGyroAndAccelerometer.idl
              ${GetGyroAndAccelerometer_CPP}
              ${GetGyroAndAccelerometer_Header}
              ${LLVS_SOURCE_DIR}/src ${IDL_INCLUDE_DIR})

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${GetGyroAndAccelerometer_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${GetGyroAndAccelerometer_Header})

  SET(LowLevelVisionSystem_CPP  "${LLVS_SOURCE_DIR}/src/LowLevelVisionSystemSK.cc")
  SET(LowLevelVisionSystem_Header "${LLVS_SOURCE_DIR}/src/LowLevelVisionSystem.hh")

  IDLFILERULE(${LLVS_SOURCE_DIR}/corba/LowLevelVisionSystem.idl
              ${LowLevelVisionSystem_CPP}
              ${LowLevelVisionSystem_Header}
              ${LLVS_SOURCE_DIR}/src ${IDL_INCLUDE_DIR})

  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ${LowLevelVisionSystem_CPP})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ${LowLevelVisionSystem_Header})

  SET(LLVS_CXX_FLAGS "${LLVS_CXX_FLAGS} -I${LLVS_SOURCE_DIR}/src")

ENDIF(OPENHRP_FOUND)