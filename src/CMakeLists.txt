##-----------------------------------------------------------------------------
## Copyright 2010, CNRS-AIST JRL
## From centralized version by Olivier Stasse
## Author Clement Petit
## Created 29/06/2010
##
## LLVS source section
##-----------------------------------------------------------------------------

#------------------------------------------------------------------------------
# Enumerate files required for compilation
#------------------------------------------------------------------------------
	ADD_LLV_SOURCE(
		MAIN
			MainEntryPoint.cpp
		GLOBAL
			ImagesInputMethod.cpp 
			VisionBasicProcess.cpp 
			LowLevelVisionServer.cpp 
			ConnectionToSot.cpp
			Simu/SimulatorInputMethod.cpp 
			Simu/FileImagesInputMethod.cpp
		HRP_BTL_SLAM
			BtlSlam/BtlSlamProcess.cpp
		DC_1394_V2
			dc1394/IEEE1394DCCameraParameters.cpp
			dc1394/IEEE1394DCImagesInputMethod.cpp
		OMNI_ORB_4
			Corba/Camera_impl.cpp 
			Corba/StereoVision_impl.cpp 
			Corba/ModelTrackerInterface_impl.cpp
			Corba/PointTrackerInterface_impl.cpp
			Corba/BtlSlamInterface_impl.cpp
		VVV
			VVV/DisparityProcess.cpp 
			VVV/MotionEvaluationProcess.cpp 
			VVV/RectificationProcess.cpp 
			VVV/EdgeDetectionProcess.cpp 
			VVV/BRepDetectionProcess.cpp 
			VVV/ColorDetection.cpp 
			VVV/FindFeaturesInImage.cpp 
			VVV/ImageDifference.cpp 
		OPEN_CV
			OpenCV/StereoVisionProcess.cpp
			OpenCV/MireDetectionProcess.cpp
		VISP
			ViSP/vispConvertImageProcess.cpp
			ViSP/CircularBufferTrackerData.cpp 
			ViSP/ComputeControlLawProcess.cpp
			ViSP/vispUndistordedProcess.cpp 
			PointTracker/PointTrackingProcess.cpp	
			PointTracker/CircularBufferPointTrackerData.cpp
			ModelTracker/nmbt/nmbtTrackingProcess.cpp
			ModelTracker/nmbt/kalmanOnNMBTProcess.cpp
		VW	
			MonoSlam/SingleCameraSLAMProcess.cpp 
			MonoSlam/IEEE1394ImagesInputMethod.cpp
			MonoSlam/hrp_model_creators.cpp 
			MonoSlam/monoslamhrp.cpp 
			MonoSlam/models_camera_height.cpp 
			MonoSlam/models_waist_velocity.cpp 
			MonoSlam/models_threed_gyro.cpp 
			MonoSlam/models_orientation.cpp 
			MonoSlam/models_wpg_motion.cpp 
			MonoSlam/models_wpg_hybrid_motion.cpp
	)

	include(LLVT-toolbox)
#------------------------------------------------------------------------------
# Add binary
#------------------------------------------------------------------------------
	ADD_LLV_CLIENT(
		STATIC LIBRARY
		${${PROJECT_NAME}_LIBRARY_NAME}
		SOURCES
			${${PROJECT_NAME}_SOURCE_LIST}
		COMPILE_FLAGS
			"-Wall -Wno-unused -fexceptions"
	)

#------------------------------------------------------------------------------
# Add library
#------------------------------------------------------------------------------
	ADD_LLV_CLIENT(
		${${PROJECT_NAME}_BINARY_NAME}
		SOURCES
			${${PROJECT_NAME}_MAIN_SOURCE_LIST}
		LINK_WITH
			${${PROJECT_NAME}_LIBRARY_NAME}
		COMPILE_FLAGS
			"-Wall -Wno-unused -fexceptions"
	)

#------------------------------------------------------------------------------
# Install
#------------------------------------------------------------------------------
  install(TARGETS ${${PROJECT_NAME}_BINARY_NAME} DESTINATION bin)
	install(TARGETS ${${PROJECT_NAME}_LIBRARY_NAME} DESTINATION lib)
