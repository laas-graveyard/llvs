# Olivier Stasse, JRL, CNRS/AIST
# Creation: 09/09/2008
# Copyright CNRS/AIST

SET(LLVS_SRC_FILES_1 
     	ColorDetection.cpp 
	ImagesInputMethod.cpp 
	SimulatorInputMethod.cpp 
	VisionBasicProcess.cpp 
	OpticalFlowProcess.cpp 
	LowLevelVisionServer.cpp 
	FindFeaturesInImage.cpp 
	ImageDifference.cpp 
	./Simu/FileImagesInputMethod.cpp )

IF (OMNIORB4_FOUND)
     SET (LLVS_SRC_FILES_1 ${LLVS_SRC_FILES_1}
	./Corba/Camera_impl.cpp 
	./Corba/StereoVision_impl.cpp )

     SET (LLVS_HEARDER_FILES_1 ${LLVS_HEADER_FILES_1}
       ./Corba/Camera_impl.hh
       ./Corba/StereoVision_impl.hh)

ENDIF(OMNIORB4_FOUND)

IF (VVV_FOUND)
  SET (LLVS_SRC_FILES_1 ${LLVS_SRC_FILES_1}
        ./VVV/DisparityProcess.cpp 
	./VVV/MotionEvaluationProcess.cpp 
	./VVV/RectificationProcess.cpp 
	./VVV/EdgeDetectionProcess.cpp 
	./VVV/BRepDetectionProcess.cpp )
ENDIF (VVV_FOUND)


IF (FindOpenCV)
    SET(LLVS_SRC_FILES_1 ${LLVS_SRC_FILES_1}
	./OpenCV/MireDetectionProcess.cpp )
ENDIF (FindOpenCV)

IF (VW_FOUND)
  IF (FindScene)
    SET(LLVS_SRC_FILES_1 ${LLVS_SRC_FILES_1}
	./MonoSlam/SingleCameraSLAMProcess.cpp 
	./MonoSlam/IEEE1394ImagesInputMethod.cpp
	./MonoSlam/hrp_model_creators.cpp 
	./MonoSlam/monoslamhrp.cpp 
	./MonoSlam/models_camera_height.cpp 
	./MonoSlam/models_waist_velocity.cpp 
	./MonoSlam/models_threed_gyro.cpp 
	./MonoSlam/models_orientation.cpp 
	./MonoSlam/models_wpg_motion.cpp 
	./MonoSlam/models_wpg_hybrid_motion.cpp )
  ENDIF (FindScene)
ENDIF (VW_FOUND)

FOREACH(src_filename ${LLVS_SRC_FILES_1})
  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ./src/${src_filename})
ENDFOREACH(src_filename)

FOREACH(src_filename ${LLVS_HEADER_FILES_1})
  SET(LLVS_HEADER_FILES ${LLVS_HEADER_FILES} ./include/llvs/${src_filename})
ENDFOREACH(src_filename)
	
	
   