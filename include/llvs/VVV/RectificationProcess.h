/** @doc This object implements a visual process
    which rectify images to apply epipolar geometry.

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
#ifndef _HRP2_RECTIFICATION_PROCESS_H_
#define _HRP2_RECTIFICATION_PROCESS_H_

/*! This vision process implements the rectification
 * provided by VVV */
extern "C"
{
#include <epbm.h>
#include <scm.h>
};
#include <VisionBasicProcess.h>

class HRP2RectificationProcess : public HRP2VisionBasicProcess
{
 public:
  /*! Constructor
   */
  HRP2RectificationProcess();

  /* Destructor */
  virtual ~HRP2RectificationProcess();

  /* Realize the process */
  virtual int RealizeTheProcess();

  /* Set the camera model */
  int SetSP(SCM_PARAMETER *asp);

  /* Perform the convertion */
  CORBA::Long scm_ConvertImageLocal(CONST SCM_PARAMETER *sp,
				    CONST EPBM *I, EPBM *O,
				    int OriginalWidth,
				    int OriginalHeight);

  /* Set the input images */
  void SetInputImages(EPBM InputImages[3]);

  /* Set the output images */
  void SetOutputImages(EPBM OutputImages[3]);

  /* Image size during calibration.*/
  int SetCalibrationSize(CORBA::Long CalibrationWidth[3], CORBA::Long CalibrationHeight[3]);

 protected:

  /*! The input images */
  EPBM m_InputImages[3];

  /*! The output images */
  EPBM m_OutputImages[3];

  /*! Pointer to the camera parameter */
  SCM_PARAMETER *m_sp;

  /*! Calibration size */
  int m_CalibrationWidth[3];
  int m_CalibrationHeight[3];
};
#endif /* _HRP2_RECTIFICATION_PROCESS_H_ */
