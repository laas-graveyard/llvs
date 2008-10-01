# Olivier Stasse, JRL, CNRS/AIST
# Creation: 09/09/2008
# Copyright CNRS/AIST

SET(LLVS_SRC_FILES_1 
     	ColorDetection.cpp 
	ImagesInputMethod.cpp 
	SimulatorInputMethod.cpp 
	FileImagesInputMethod.cpp 
	VisionBasicProcess.cpp 
	DisparityProcess.cpp 
	OpticalFlowProcess.cpp 
	MotionEvaluationProcess.cpp 
	Camera_impl.cpp 
	MireDetectionProcess.cpp 
	RectificationProcess.cpp 
	EdgeDetectionProcess.cpp 
	BRepDetectionProcess.cpp 
	SingleCameraSLAMProcess.cpp 
	IEEE1394ImagesInputMethod.cpp 
	FindFeaturesInImage.cpp 
	LowLevelVisionServer.cpp 
	hrp_model_creators.cpp 
	monoslamhrp.cpp 
	models_camera_height.cpp 
	models_waist_velocity.cpp 
	models_threed_gyro.cpp 
	models_orientation.cpp 
	ImageDifference.cpp 
	StereoVision_impl.cpp 
	models_wpg_motion.cpp 
	models_wpg_hybrid_motion.cpp )

FOREACH(src_filename ${LLVS_SRC_FILES_1})
  SET(LLVS_SRC_FILES ${LLVS_SRC_FILES} ./src/${src_filename})
ENDFOREACH(src_filename)

	
	
   