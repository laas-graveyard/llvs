/** @doc This object implements a visual process
    to detect edges.
    
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
#ifndef _HRP2_EDGE_DETECTION_PROCESS_H_
#define _HRP2_EDGE_DETECTION_PROCESS_H_

#include "VisionBasicProcess.h"

class HRP2EdgeDetectionProcess : public HRP2VisionBasicProcess
{
 public:
  /* Constructor */
  HRP2EdgeDetectionProcess();

  /* Destructor */
  ~HRP2EdgeDetectionProcess();
  
  /*! Initialize the process */
  virtual int InitializeTheProcess();

  /*! Compute the edges */
  virtual int RealizeTheProcess();

  /*! Cleanup the process */
  virtual int CleanUpTheProcess();

  /*! Set the input images 
   * This method is needed to set up the reference to the input images.
   * They should be specified only once. The first image is the left image.
   * The second image is the second image. It is assume that those images
   * are corrected.
   */
  int SetInputImages(EPBM lInputImages[3]);

  
  /* Set Image to process.
     Index for which the value is different from 0 
     calls for process. */
  void SetImageOnWhichToProcess(int ImagesOnWhichToProcess[3]);

  /*! Edge Image */
  EPBM m_Edge[3],m_Grad[3];
  

 protected:
  
  /*! Input Image */
  EPBM m_InputImage[3];

  /*! Statistical information on the edges */
  double m_mu[3], m_sigma[3];

  /*! Images on which perform the edge detection  3 at max.*/
  int m_ImagesOnWhichToProcess[3];

  /* Operator use to compute the gradient */
  int m_Operator;

  /* Constantes for chosing the operator for the gradient */
  static const int SOBEL=0;
  static const int LAPLACIAN=1;
};
#endif /* _HRP2_EDGE_DETECTION_PROCESS_H_ */
