# Olivier Stasse, JRL, CNRS/AIST
# Creation: 09/09/2008
# Copyright CNRS/AIST

SET(LLVS_SRC_FILES_1 
	ImagesInputMethod.cpp 
	VisionBasicProcess.cpp 
	LowLevelVisionServer.cpp 
	ConnectionToSot.cpp
	./Simu/SimulatorInputMethod.cpp 
	./Simu/FileImagesInputMethod.cpp  )

SET(LLVS_HEADER_FILES_1 
	ImagesInputMethod.h
	VisionBasicProcess.h 
	LowLevelVisionServer.h 
	./Simu/SimulatorInputMethod.h 
	./Simu/FileImagesInputMethod.h 
	./dc1394/IEEE1394DCImagesInputMethod.h )

IF (libdc1394-2_FOUND)
     SET (LLVS_SRC_FILES_1 ${LLVS_SRC_FILES_1}
	./dc1394/IEEE1394DCCameraParameters.cpp
	./dc1394/IEEE1394DCImagesInputMethod.cpp )

     SET (LLVS_HEARDER_FILES_1 ${LLVS_HEADER_FILES_1}
	./dc1394/IEEE1394DCCameraParameters.h
	./dc1394/IEEE1394DCImagesInputMethod.h )
ENDIF(libdc1394-2_FOUND)

IF (OMNIORB4_FOUND)
     SET (LLVS_SRC_FILES_1 ${LLVS_SRC_FILES_1}
	./Corba/Camera_impl.cpp 
	./Corba/StereoVision_impl.cpp 
	./Corba/ModelTrackerInterface_impl.cpp)

     SET (LLVS_HEARDER_FILES_1 ${LLVS_HEADER_FILES_1}
       ./Corba/Camera_impl.hh
       ./Corba/StereoVision_impl.hh
       ./Corba/ModelTrackerInterface_impl.h)
ENDIF(OMNIORB4_FOUND)

IF (VVV_FOUND)
  SET (LLVS_SRC_FILES_1 ${LLVS_SRC_FILES_1}
        ./VVV/DisparityProcess.cpp 
	./VVV/MotionEvaluationProcess.cpp 
	./VVV/RectificationProcess.cpp 
	./VVV/EdgeDetectionProcess.cpp 
	./VVV/BRepDetectionProcess.cpp 
     	./VVV/ColorDetection.cpp 
	./VVV/FindFeaturesInImage.cpp 
	./VVV/ImageDifference.cpp 
	)
  SET (LLVS_HEADER_FILES_1 ${LLVS_HEADER_FILES_1}
        ./VVV/DisparityProcess.h 
	./VVV/MotionEvaluationProcess.h 
	./VVV/RectificationProcess.h 
	./VVV/EdgeDetectionProcess.h 
	./VVV/BRepDetectionProcess.h 
     	./VVV/ColorDetection.h 
	./VVV/FindFeaturesInImage.h 
	./VVV/ImageDifference.h 
	)

ENDIF (VVV_FOUND)


IF (OPENCV_FOUND)
    SET(LLVS_SRC_FILES_1 ${LLVS_SRC_FILES_1}
	./OpenCV/StereoVisionProcess.cpp
	./OpenCV/MireDetectionProcess.cpp
  )
  SET (LLVS_HEADER_FILES_1 ${LLVS_HEADER_FILES_1}
  ./OpenCV/StereoVisionProcess.h 
	./OpenCV/MireDetectionProcess.h 
	)
ENDIF (OPENCV_FOUND)

# Model tracker uses Visp
IF (VISP_FOUND AND NMBT_FOUND)
    SET(LLVS_SRC_FILES_1 ${LLVS_SRC_FILES_1}
	./ModelTracker/nmbt/nmbtTrackingProcess.cpp	
  )
  SET (LLVS_HEADER_FILES_1 ${LLVS_HEADER_FILES_1}
  	./ModelTracker/nmbtTrackingProcess.h
  )
ENDIF (VISP_FOUND AND NMBT_FOUND)

# Visp image convertion need OpenCV
IF (VISP_FOUND AND OPENCV_FOUND)
    SET(LLVS_SRC_FILES_1 ${LLVS_SRC_FILES_1}
	./ViSP/vispConvertImageProcess.cpp	
  )
  SET (LLVS_HEADER_FILES_1 ${LLVS_HEADER_FILES_1}
  	./ViSP/vispConvertImageProcess.h
  )
ENDIF (VISP_FOUND AND OPENCV_FOUND)


# Visp image convertion need OpenCV
IF (VISP_FOUND)
    SET(LLVS_SRC_FILES_1 ${LLVS_SRC_FILES_1}
	./ViSP/vispUndistordedProcess.cpp	
  )
  SET (LLVS_HEADER_FILES_1 ${LLVS_HEADER_FILES_1}
  	./ViSP/vispUndistordedProcess.h
  )
ENDIF (VISP_FOUND)



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
	
	
   
