/** @doc
    This class evaluates the camera position and build
    a sparse map of the world.
    It relies on the Scene library developped by Andrew Davison.


   CVS Information:
   $Id$
   $Author$
   $Date$
   $Revision$
   $Source$
   $Log$

   Copyright (c) 2003-2006,
   @author Olivier Stasse

   JRL-Japan, CNRS/AIST

   All rights reserved.

   Redistribution and use in source and binary forms, with or without modification,
   are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the CNRS and AIST nor the names of its contributors
   may be used to endorse or promote products derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
   AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
   OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef __HRP2_SingleCameraSLAM_H__
#define __HRP2_SingleCameraSLAM_H__

#ifdef __ORBIX__
#include <OBE/CORBA.h>
#include <OBE/CosNaming.h>
#include <Scene.h>
#endif

#ifdef OMNIORB4
#include <omniORB4/CORBA.h>
#include "Scene.hh"
#endif
#ifdef __ORBIX__
#include "GetGyroAndAccelerometer.h"
#endif

#ifdef OMNIORB4
#include "GetGyroAndAccelerometer.hh"
#endif


class LowLevelVisionServer;

#include <FindFeaturesInImage.h>
#include "VisionBasicProcess.h"
#include "monoslamhrp.h"

class HRP2SingleCameraSLAMProcess : public HRP2VisionBasicProcess
{
 public:
  /*! Constructor */
  HRP2SingleCameraSLAMProcess(CORBA::ORB_var orb,
			      CosNaming::NamingContext_var cxt,
			      LowLevelVisionServer * aLLVS);

  /*! Destructor */
  virtual ~HRP2SingleCameraSLAMProcess();

  /*! HRP2 VISION BASIC PROCESS PART */
  /*! Initialize the process */
  virtual int pInitializeTheProcess();

  /*! Realize the process */
  virtual int pRealizeTheProcess();

  /*! Cleanup the process */
  virtual int pCleanUpTheProcess();

  /*! Set Parameter */
  virtual int SetParameter(string aParameter, string aValue);

  /*! Set the input images
   * This method is needed to set up the reference to the input images.
   * They should be specified only once. The first image is the left image.
   * The second image is the second image. It is assume that those images
   * are corrected.
   */
  int SetInputImages(EPBM * lInputImages);

  /*! Get the position and the associated covariance
   *
   */
  int GetPositionAndCovariance(double Position[7], double Covariance[9]);

  /*! SCENE CONTROL PART */
  /*! Virtual functions */


  /*! Creates a copy of the object scene */
  int CreateCopyOfScene(SceneObject_var &aSO_var);

  /*! Creates the connection with GGAA */
  void GetCorbaConnectionToGGAAplugin();

  /*! Takes the gyroscope and the accelerometer according to the time stamp. */
  int GetGyroAcceleroFromTimeStamp(double lGyro[3], double lAccelerometer[3], double timeref,
				   double lWaistVelocity[2], double lOrientation[7]);

  /* Reference to the object allowing to do the link
     between the wide lens camera and the narrow ones.
     It also computes the matrix from the head reference frame to
     the wide lens camera.
     Therefore the matrix from the first calibration reference frame
     to the head has to be provided.
  */
  int SetFindFeaturesFromWideImage(FindFeaturesFromWideImage * aFFFWI,
				   double headTorg[16]);


 protected:

  /*! State to the tracking
   * True: Start to evaluate the position and follow the features across the video stream.
   * False: Stop.
   */
  bool m_TrackingState;

  /*! State of the mapping:
   * True : New features might be initialize.
   * False : No new features allowed.
   */
  bool m_MappingState;

  /*! State of the vision:
   * True : Search for new features.
   * False : Do not include informations from vision.
   */
  bool m_VisionFlag;

  /* ! State of the gyro:
   * True: Include the gyro into the update of the
   * kalman filter.
   * False: Do not include it.
   */
  bool m_GyroFlag;

  /* ! State of the Waist Velocity:
   * True : Include the measurement on the waist velocity.
   * False : Ignore them.
   */
  bool m_WaistVelocityFlag;

  /* ! State of the Camera height:
   * True: Include the camera height into the update of the
   * kalman filter.
   * False: Do not include it.
   */
  bool m_CameraHeightFlag;

  /* ! State of the Orientation:
   * True: Include the orientation into the update of the
   * kalman filter.
   * False : do not include it.
   */
  bool m_OrientationFlag;

  /* ! Reference to image input */
  EPBM * m_InputImage;

  /* ! Reference to the Get Gyro and Accelerometer plugin */
  GyroAndAccelerometerServer_var m_GyroAndAccelerometer;

  /* ! Image to be stored */
  VW::ImageMono<unsigned char> *m_grabbed_image;

  /* ! Reference to the ORB. */
  CORBA::ORB_var m_orb;

  /*! Reference to LLVS */
  LowLevelVisionServer * m_LLVS;

  /* ! Reference to the naming context. */
  CosNaming::NamingContext_var m_cxt;

  /* ! Instance of the Monocular SLAM interface object. */
  MonoSLAMHRP *m_MonoSLAMHRP;

  /*! Instance of the object allowing to do the link
    between the wide lens and the narrow lens. */
  FindFeaturesFromWideImage * m_FFFWI;

  /*1 Keep track of the transformation between the head and
    the first calibration reference frame. */
  double m_headTorg[16];

  /*! Matrix from the wide lens to the head. */
  VNL::Matrix<double> m_WL2Head;

  unsigned int m_NUMBER_OF_FEATURES_TO_SELECT;
  unsigned int m_NUMBER_OF_FEATURES_TO_KEEP_VISIBLE;
  unsigned int m_MAX_FEATURES_TO_INIT_AT_ONCE;
  double m_MIN_LAMBDA;
  double  m_MAX_LAMBDA;
  unsigned int m_NUMBER_OF_PARTICLES;
  double m_STANDARD_DEVIATION_DEPTH_RATIO;
  unsigned int m_MIN_NUMBER_OF_PARTICLES;
  double m_PRUNE_PROBABILITY_THRESHOLD;
  unsigned int m_ERASE_PARTIALLY_INIT_FEATURE_AFTER_THIS_MANY_ATTEMPTS;

};
#endif /* __HRP2_SingleCameraSLAM_H__ */

