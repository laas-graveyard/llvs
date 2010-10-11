/** @doc This object implements a simple color detection based on
    histogram.

   CVS Information:
   $Id$
   $Author$
   $Date$
   $Revision$
   $Source$
   $Log$

   Copyright (c) 2006,
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
#ifndef _HRP2_COLOR_DETECTION_H_
#define _HRP2_COLOR_DETECTION_H_

/*! This object defines the abstract class defining a basic
 * vision process
 *
 * Copyright (c) 2004 Olivier Stasse, JRL, CNRS/AIST
 *
 *
 */

#include <string>
#include <vector>

#ifdef OMNIORB4
#include <omniORB4/CORBA.h>
#endif

#ifdef __ORBIX__
#include <OBE/CORBA.h>
#include <OBE/CosNaming.h>
#endif


#include <VisionBasicProcess.h>

/*! headers from VVV */
extern "C"
{
#include <epbm.h>
};

/*! headers from VW */
#include <VW/Image/imagemono.h>
#include <VW/Image/imageyuv.h>

#if 0
#include <VisualServoingServer.h>
#endif

#include <ColorDetectionOperators.h>
#include <ProjectiveMatrix.h>
#include <Triangulation.h>

class LowLevelVisionServer;

/*! This object is used to derive all the vision process
 * at least for the LowLevelVisionServer.
 */
class HRP2ColorDetectionProcess : public HRP2VisionBasicProcess
{

 public:

  /*! Constructor */
  HRP2ColorDetectionProcess(CosNaming::NamingContext_var cxt,
			    LowLevelVisionServer *lLLVS,
			    std::string lCalibrationDirectory);

  /*! Destructor */
  virtual ~HRP2ColorDetectionProcess();


  /*! Initialize the process. */
  virtual int InitializeTheProcess();

  /*! Realize the process */
  virtual int RealizeTheProcess();

  /*! Cleanup the process */
  virtual int CleanUpTheProcess();

  /*! Set the input image using the EPBM structure */
  int SetInputImages(EPBM InputImage);

  /*! Set the input image using the EPBM structure */
  int SetInputImages(EPBM InputImage1, EPBM InputImage2);

  /*! Try connection to visual servoing. */
  int TryConnectionToVisualServoing();

  /*! Read robot matrix */
  void ReadRobotMatrix();

 protected:


  /* ! Reference to the LowLevelVisionServer. */
  LowLevelVisionServer * m_LLVS;

  /* ! Reference on the input image. */
  EPBM m_InputImage[2];

  /*! Image in RGB*/
  VW::ImageRGB<unsigned char> * m_ImageInputRGB[2];

  /*! Image result in mono.  */
  VW::ImageMono<unsigned char> * m_ImageResult[2];

  /*! Color detection Operator */
  VW::ColorDetectionOperators m_ColorDetector[2];

  /*! Number of cameras.
   * This number is initialized according to the SetInputImages
   * method called 1: (one argument), 2: (two arguments).
   */
  int m_NbOfCameras;


  /*! CORBA naming context provided by LLVS */
  CosNaming::NamingContext_var m_cxt;

  /*! Number of processes call without trial connection to the Visual Servoing plugin.
    This number is used to implement a connection strategy.
    Used in conjunction with m_IntervalBetweenConnectionTrials.
    */
  int m_NbOfProcessWithoutTrialConnection;

  /*! Number of call to the process before retrying a connection. */
  int m_IntervalBetweenConnectionTrials;

  /*! Store projective matrix for triangulation. */
  Geometry::ProjectiveMatrix * m_PI[2];

  /*! Object for computing triangulation . */
  Geometry::TriangulationMethods m_TM;

  /*! Calibration directory */
  std::string m_CalibrationDirectory;

  /*! Matrix of conversion between VVV and the head. */
  VNL::Matrix<double> m_HtO;

  /*! Check if this is the first time that the process applies */
  bool m_FirstTime;

  /*! State */
};

#include <LowLevelVisionServer.h>
#endif /* _HRP2_COLOR_DETECTION_H_ */
