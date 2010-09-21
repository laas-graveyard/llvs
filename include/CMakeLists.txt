# Copyright (C) 2010 Clement Petit, Thomas Moulard, Olivier Stasse,
#		     CNRS-AIST JRL, CNRS/AIST.
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

INCLUDE(include-toolbox)

# Generate the package dependent file llvsConfig.h
CONFIGURE_LLV_FILE(llvs/llvsConfig.h.cmake llvsConfig.h)

# Add shared headers
ADD_HEADERS(
  PREFIXES
	llvs
	llvs/dc1394
	llvs/Simu
	llvs/OpenCV
  FILES
	ImagesInputMethod.h
	FileImagesInputMethod.h
	VisionBasicProcess.h
	LowLevelVisionServer.h
	MireDetectionProcess.h
	IEEE1394DCImagesInputMethod.h
	SimulatorInputMethod.h
	StereoVisionProcess.h
  NO_TREE_MIRROR
  INSTALL DESTINATION include/llvs
  )


# Add LLVS only headers
ADD_HEADERS(
  PREFIXES
	llvs
	llvs/dc1394
	llvs/Corba
	llvs/BtlSlam
	llvs/ModelTracker
	llvs/MonoSlam
	llvs/OpenCV
	llvs/PointTracker
	llvs/Simu
	llvs/ViSP
	llvs/VVV
  FILES
	# BtlSlam
	BtlSlamProcess.h
	# Corba
	Camera_impl.h
	ModelTrackerInterface_impl.h
	PointTrackerInterface_impl.h
	StereoVision_impl.h
	BtlSlamInterface_impl.h
	# dc1394
	IEEE1394DCCameraParameters.h
	# ModelTracker
	nmbtTrackingProcess.h
	kalmanOnNMBTProcess.h
	# MonoSlam
	hrp_model_creators.h
	models_camera_height.h
	models_orientation.h
	models_threed_gyro.h
	models_waist_velocity.h
	models_wpg_hybrid_motion.h
	models_wpg_motion.h
	monoslamhrp.h
	SingleCameraSLAMProcess.h
	# OpenCV
	# PointTracker
	CircularBufferPointTrackerData.h
	PointTrackingProcess.h
	# Simu
	# ViSP
	CircularBufferTrackerData.h
	ComputeControlLawProcess.h
	vispConvertImageProcess.h
	vispUndistordedProcess.h
	# VVV
	BRepDetectionProcess.h
	ColorDetection.h
	DisparityProcess.h
	EdgeDetectionProcess.h
	FindFeaturesInImage.h
	IEEE1394ImagesInputMethod.h
	ImageDifference.h
	MotionEvaluationProcess.h
	RectificationProcess.h
	# Main include
	CircularBuffer.t.h
	ConnectionToSot.h
	oocmap.h
	OpticalFlowProcess.h
)
