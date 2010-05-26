/** @doc This object implements a visual process
    to compute a disparity map from 2 camera images.
    
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

#include <vector>

#include "VisionBasicProcess.h"
/*! HRP2VisionBasicProcess is used to derive all the vision process
 * at least for the LowLevelVisionServer.
 */

#if LLVS_HAVE_OPENCV

using namespace cv;

/* Lists the different stereo algorithm available. These are available in OpenCV 2.1 */
enum StereoAlgoType { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_GC=3 };


class HRP2StereoVisionProcess : public HRP2VisionBasicProcess
{

 public:
 
  /*! Constructor */
  HRP2StereoVisionProcess();

  /*! Destructor */
  virtual ~HRP2StereoVisionProcess();


  /*! Initialize the process. */
  int InitializeTheProcess();

  /*! Realize the process */
  int RealizeTheProcess();
  
  /*! Cleanup the process */
  int CleanUpTheProcess();

  /* Set the type of Stereo Vision algorithm to use */
  int setStereoAlgorithmType( StereoAlgoType stereoAlgo );
  
  /* Get which type of Stereo Vision algorithm is enabled */
  int getStereoAlgorithmType() { return m_StereoAlgo; };
  
  /* Set the parameters for the used stereo vision algorithm
     The meaning of the values depends on the algorithm used
	 and the order of the values is thus important!  */
  int setAlgoParameters( std::vector<int> parameters );

  /* Set the input image sizes: width and height.
   * These sizes must be known before calling loadCameraParameters */
  void setImageSize( int w, int h );

  /* Load intrinsic and extrinsic parameters from given files.
   * The function initializes the m_rectifyMap matrices which are to be used
   * in stereoRectifyImages function, and the m_Q matrix used in Get3DPoints(). */
  int loadCameraParameters( const char* intrinsicFilename, const char* extrinsicFilename );

  /* Set the input images: left, right */
  int SetInputImages( Mat InputImages[2], bool rectified=false );

  /* Process the input images using the m_rectifyMap matrices.
   * Input images are replaced by the rectified version.
   * Requires that loadCameraParameters() and SetInputImages() have been executed before! */
  int stereoRectifyImages( );

  /* Use the rectified input images to generate a range map */
  int ComputeRangeMap();
  
  /* Get the output image: copy the obtained range map to the given image */
  int GetOutputImage( Mat& OutputImage );
  
  /* Get the point of 3D points computed using the disparity map.
   * ComputeRangeMap() should have been succesfully executed before */
  int Get3DPoints( Mat &xyz );
  

  /* Free the images for internal purposes */
  void FreeImages();


 protected:

  StereoAlgoType m_StereoAlgo;

  /* the set of parameters for the stereo algorithms
   * the meaning of each value depends on the algorithm active: default values in ()
   *  - STEREO_BM:
   *  preFilterCap (31), SADWindowSize (9), numberOfDisparities (256), uniquenessRatio (15), 
   *  speckleWindowSize (100), speckleRange (32), disp12MaxDiff (1), textureThreshold (10)
   * 
   *  - STEREO_SGBM and STEREO_HH:
   *  preFilterCap (63), SADWindowSize (9), numberOfDisparities (256), uniquenessRatio (10), 
   *  speckleWindowSize (100), speckleRange (32), disp12MaxDiff (1)
   * 
   *  - STEREO_GC:
   *  numberOfDisparities (256), maxIters (2)
   */
  std::vector<int> m_parameters;
  

  Mat m_InputImages[2];
  Mat m_OutputImage;
  
  Mat m_Q;

  Size m_imgSize;
  Mat m_rectifyMap[2][2];
  
  bool m_imgSizeKnown;
  bool m_inputImagesLoaded;
  bool m_cameraParamLoaded;
  bool m_imgRectified;
  bool m_rangeMapComputed;
	
  StereoBM *m_stereoBM;
  StereoSGBM *m_stereoSGBM;
  CvStereoGCState *m_stereoGCstate;
};

#endif /* LLVS_HAVE_OPENCV */

#endif /* _HRP2_STEREO_VISION_PROCESS_H_ */
