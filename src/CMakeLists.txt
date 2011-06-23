# Copyright (C) 2010 Thomas Moulard, Clement Petit, CNRS-AIST JRL, CNRS/AIST.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# Enumerate sources.
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
		ViSP/ComputeControlLawProcessIROS2010.cpp
		ViSP/vispUndistordedProcess.cpp
		ViSP/ColorDetectionOperators.cpp
		ViSP/ColorDetection.cpp
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

INCLUDE(LLVT-toolbox)

# Setup main library and server binary.
ADD_LLV_CLIENT(SHARED LIBRARY llvs SOURCES ${LLVS_SOURCE_LIST})
ADD_LLV_CLIENT(llvs-server SOURCES ${LLVS_MAIN_SOURCE_LIST} LINK_WITH llvs)

# Set shared library version.
SET_TARGET_PROPERTIES(llvs PROPERTIES SOVERSION ${PROJECT_VERSION})

# Installation rules.
INSTALL(TARGETS llvs-server DESTINATION bin)
INSTALL(TARGETS llvs DESTINATION lib)

# Link against Boost.
TARGET_LINK_LIBRARIES(llvs-server ${Boost_LIBRARIES})