/** @doc This object implements a visual process
    detecting a mire.
    
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
#ifndef _HRP2_MIRE_DETECTION_PROCESS_H_
#define _HRP2_MIRE_DETECTION_PROCESS_H_

#include "VisionBasicProcess.h"
/*! This object is used to derive all the vision process
 * at least for the LowLevelVisionServer.
 */

#if LLVS_HAVE_OPENCV

using namespace cv;


class HRP2MireDetectionProcess : public HRP2VisionBasicProcess
{

 public:

  /*! Constructor */
  HRP2MireDetectionProcess();

  /*! Destructor */
  virtual ~HRP2MireDetectionProcess();


  /*! Initialize the process. */
  virtual int InitializeTheProcess();

  /*! Realize the process */
  virtual int RealizeTheProcess();
  
  /*! Cleanup the process */
  virtual int CleanUpTheProcess();

  /* Detect the stereo mire */
  int DetectMireStereo();

  /* Set the input images */
  void SetInputImages( Mat InputImages[3]);
  
  /* Set the output images */
  void SetOutputImages( Mat OutputImages[3]);

  /* Free the images for internal purposes */
  void FreeImages();

  /* Set the chessboard size */
  void SetChessBoardSize(int NbCols, int NbRows);

 protected:

  /* Gray level images */
  Mat m_img, m_imd;

  /* Temporary images */
  Mat m_ImgTempG, m_ImgTempD;

  /* The input images */
  Mat m_InputImages[3];

  /* The output images */
  Mat m_OutputImages[3];

  /* Size of the chessboard */
  int m_ChessBoardSize[2];

  /* Two sets of corners. */
  CvPoint2D32f * m_Corners[2];

};
#endif /* LLVS_HAVE_OPENCV */

#endif /* _HRP2_MIRE_DETECTION_PROCESS_H_ */
