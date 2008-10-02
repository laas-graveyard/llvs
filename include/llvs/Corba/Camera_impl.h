/** @doc This object implements an abstract class
    of camera.
    
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
#ifndef _CAMERA_IMPL_H_
#define _CAMERA_IMPL_H_

#ifdef __ORBIX__
#include <OBE/CORBA.h>
#include "LowLevelVisionServer.h"
#include "LowLevelVisionSystem_skel.h"
#endif

#ifdef OMNIORB4
#include <omniORB4/CORBA.h>
#include "LowLevelVisionSystem.hh"
#endif

#include <string>

#if (LLVS_HAVE_VVV>0)
extern "C"
{
#include "scm.h"
}
#endif

using namespace std;

/* This class implements the camera IDL
 * in C++
 *
 * Copyright 2004 (c), JRL/CNRS, AIST,
 *                    Olivier Stasse,
 */
class LowLevelVisionServer;

class Camera_impl : public virtual POA_HRP2Camera
{
 public:
  
  /*! Constructor*/
  Camera_impl(const char * location,LowLevelVisionServer *aLLVS);
  
  /*! Destructor */
  ~Camera_impl();

  /*! Interface : destroy() method */
  void destroy();
  
  /*! Interface : returns the camera parameter */
  virtual HRP2Camera::CameraParameter * GetCameraParameter()
    throw(CORBA::SystemException);

  /*! Interface : returns the intrinsic parameters */
  virtual HRP2Camera::IntrinsicParameters GetIntrinsicParameters()
   throw(CORBA::SystemException) ; 

  /*! Interface : returns the projective parameters */
  virtual HRP2Camera::ProjectiveParameters GetProjectiveParameters()
    throw(CORBA::SystemException);

  /*! Interface : returns the depth Buffer */
  virtual CORBA::Long SetAcquisitionSize(CORBA::Long aWidth, CORBA::Long aHeight)
    throw(CORBA::SystemException);
  
  /*! Set the father */
  void SetFather(LowLevelVisionServer *aFather);

  /*! Get the father */
  LowLevelVisionServer * GetFather();
    
  /*! Set the identifier */
  void SetIdentifier(int anID);

  /*! Get the identifier */
  int GetIdentifier();

  /*! Set the Name */
  void SetName(string aName);

  /*! Get the Name */
  string GetName();

  /*! Set the Camera Type */
  void SetCameraType(int aType);

  /*! Get the Camera Type */
  int GetCameraType();

  /*! Set Intrinsic parameters */
  void SetIntrinsicParameters(float aFocal, float aScale[2], float SkewFactor, float ImageCenter[2]);

  /*! Set CameraParameter */
  void SetCameraParameter(long aWidth, long aHeight, long CalibrationWidth, long CalibrationHeight);

  /*! Reset the camera parameters */
  void ResetCameraParameters(void);

  /*! Reset the intrinsic parameters */
  void ResetIntrinsicParameters(void);

#if (LLVS_HAVE_VVV>0)
  /*! Set Camera Projective Camera Parameters */
  void SetCameraProjectiveParameters(SCM_PARAMETER *sp, int camera_number);
#endif

  /*! Set the verbosity level */
  void SetVerbosity(int aVerbosity);

  /*! Get the verbosity level */
  int GetVerbosity();

  /*! Get the original projective matrix */
  void GetOriginalProjectiveMatrix(double oP[3][4]);

 protected:

  /*! Link to the LowLevelVisionServer */
  LowLevelVisionServer * m_LLVSFather;

  /*! Camera parameter */
  HRP2Camera::CameraParameter m_CameraParameter;
  
  /*! Intrinsic camera parameters */
  HRP2Camera::IntrinsicParameters m_IntrinsicParameter;

  /* ! Projective camera parameters */
  HRP2Camera::ProjectiveParameters m_ProjectiveParameters;

  /*! Verbosity level */
  int m_Verbosity;
};
#endif /* _CAMERA_IMPL_H_ */
