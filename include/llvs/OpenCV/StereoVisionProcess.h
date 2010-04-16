/** @doc This object implements a visual process
    detecting a mire.
    
    CVS Information:
   $Id$
   $Author$
   $Date$
   $Revision$
   $Source$
   $Log$

   Copyright (c) 2003-2010,
   @author Torea Foissotte
   
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
#ifndef _HRP2_STEREO_VISION_PROCESS_H_
#define _HRP2_STEREO_VISION_PROCESS_H_

#ifdef LLVS_HAVE_OPENCV
#include <cv.h>
using namespace cv;
#endif 

#include <vector>

#include "VisionBasicProcess.h"
/*! HRP2VisionBasicProcess is used to derive all the vision process
 * at least for the LowLevelVisionServer.
 */

/* Lists the different stereo algorithm available */
enum StereoAlgoType { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2 };


class HRP2StereoVisionProcess : public HRP2VisionBasicProcess
{

 public:
 
  /*! Constructor */
  HRP2StereoVisionProcess();

  /*! Destructor */
  virtual ~HRP2StereoVisionProcess();


  /*! Initialize the process. */
  virtual int InitializeTheProcess();

  /*! Realize the process */
  virtual int RealizeTheProcess();
  
  /*! Cleanup the process */
  virtual int CleanUpTheProcess();

  /* Set the type of Stereo Vision algorithm to use */
  int setStereoAlgorithmType( StereoAlgoType stereoAlgo );
  
  /* Get which type of Stereo Vision algorithm is enabled */
  int getStereoAlgorithmType() { return m_StereoAlgo; };
  
  /* Set the parameters for the used stereo vision algorithm
     The meaning of the values depends on the algorithm used
	 and the order of the values is important!  */
  int setAlgoParameters( std::vector<int> parameters );

  /* Use the given input images to generate a range map */
  int ComputeRangeMap();

#ifdef LLVS_HAVE_OPENCV
  /* Set the input images: left, right, up */
  void SetInputImages( const Mat& InputImages[3] );
  
  /* Set the output image: copy the obtained range map to the given image */
  void SetOutputImage( Mat& OutputImage );
#else
	// TODO: check what kind of images to use when OpenCV is not used?
	// 		 For now, it is set to (void *)
	
  /* Set the input images: left, right, up */
  void SetInputImages( void *InputImages[3] );
  
  /* Set the output image: copy the obtained range map to the given image */
  void SetOutputImage( void *OutputImage );
#endif

  /* Free the images for internal purposes */
  void FreeImages();


 protected:

  StereoAlgoType m_StereoAlgo;
  
#ifdef LLVS_HAVE_OPENCV

  Mat m_InputImages[3];
  Mat m_OutputImage;
	
  StereoBM *m_stereoBM;
  StereoSGBM *m_stereoSGBM;
#else

	// TODO: check what kind of images to use when OpenCV is not used?
	// 		 For now, it is set to (void *)
  void* m_InputImages[3];
  void* m_OutputImage;
#endif

};

#endif /* _HRP2_STEREO_VISION_PROCESS_H_ */
