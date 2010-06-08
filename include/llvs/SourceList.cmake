SET(LLVS_HEADERS_FILES_1
        ColorDetection.h 
	ImagesInputMethod.h 
	FileImagesInputMethod.h 
	VisionBasicProcess.h 
	DisparityProcess.h 
	OpticalFlowProcess.h 
	MotionEvaluationProcess.h 
	RectificationProcess.h 
	LowLevelVisionServer.h 
	MireDetectionProcess.h 
	EdgeDetectionProcess.h 
	BRepDetectionProcess.h 
	SingleCameraSLAMProcess.h 
	FindFeaturesInImage.h
	Camera_impl.h 
	hrp_model_creators.h 
	monoslamhrp.h 
	models_camera_height.h 
	models_waist_velocity.h 
	models_threed_gyro.h 
	models_orientation.h 
	ImageDifference.h 
	StereoVision_impl.h 
	models_wpg_motion.h 
	models_wpg_hybrid_motion.h )


FOREACH(src_filename ${LLVS_HEADERS_FILES_1})
  SET(LLVS_HEADERS_FILES ${LLVS_HEADERS_FILES} ./src/${src_filename})
ENDFOREACH(src_filename)

