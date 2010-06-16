/** @doc This object implements a visual process to get a disparity map.
    
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
#include "OpenCV/StereoVisionProcess.h"
#include <iostream>
#include <fstream>
#include <math.h>

// Debug macros
#include "Debug.h"

#if LLVS_HAVE_OPENCV

HRP2StereoVisionProcess::HRP2StereoVisionProcess()
{
  m_ProcessName = "Stereo Vision";

  m_parameters.clear();

  m_cameraParamLoaded = m_imgSizeKnown = false;
  m_imgRectified = m_inputImagesLoaded = false;
  m_rangeMapComputed = false;
  
  m_imgSize = Size(0,0);
  
  m_stereoBM = NULL;
  m_stereoSGBM = NULL;
  m_stereoGCstate = NULL;

  setStereoAlgorithmType( STEREO_SGBM );
}


HRP2StereoVisionProcess::~HRP2StereoVisionProcess()
{ 
  FreeImages();
  
  m_parameters.clear();
  
  m_Q.release();

  if( m_stereoBM != NULL ) delete m_stereoBM;
  if( m_stereoSGBM != NULL ) delete m_stereoSGBM;
  if( m_stereoGCstate != NULL ) cvReleaseStereoGCState( &m_stereoGCstate );
}


int HRP2StereoVisionProcess::pInitializeTheProcess()
{
  
  return 0;
}


int HRP2StereoVisionProcess::pRealizeTheProcess()
{
  if( !m_Computing ) return 0;
  
  ComputeRangeMap();
  
  return 0;
}


int HRP2StereoVisionProcess::setAlgoParameters( std::vector<int> parameters )
{
  m_parameters.clear();
   
  for( unsigned int i=0; i<parameters.size(); ++i ) {
    m_parameters.push_back( parameters[i] );
  }
  
  setStereoAlgorithmType( m_StereoAlgo );
	
  return 0;
}


int HRP2StereoVisionProcess::setStereoAlgorithmType( StereoAlgoType stereoAlgo )
{
  switch( stereoAlgo ) {
    case STEREO_BM:
      if( m_stereoBM==NULL ) m_stereoBM = new StereoBM;
      
      m_stereoBM->state->preFilterCap = m_parameters.size() > 0 ? m_parameters[0] : 31;
      m_stereoBM->state->SADWindowSize = m_parameters.size() > 1 ? m_parameters[1] : 9;
      m_stereoBM->state->numberOfDisparities = m_parameters.size() > 2 ? m_parameters[2] : 256;
      m_stereoBM->state->uniquenessRatio = m_parameters.size() > 3 ? m_parameters[3] : 15;
      m_stereoBM->state->speckleWindowSize = m_parameters.size() > 4 ? m_parameters[4] : 100;
      m_stereoBM->state->speckleRange = m_parameters.size() > 5 ? m_parameters[5] : 32;
      m_stereoBM->state->disp12MaxDiff = m_parameters.size() > 6 ? m_parameters[6] : 1;
      m_stereoBM->state->textureThreshold = m_parameters.size() > 7 ? m_parameters[7] : 10;

      m_stereoBM->state->minDisparity = 0;
      break;
        
    case STEREO_SGBM:
    case STEREO_HH:
      {
      if( m_stereoSGBM==NULL ) m_stereoSGBM = new StereoSGBM;

      int cn = 1;
      if( m_inputImagesLoaded ) cn = m_InputImages[0].channels();

      m_stereoSGBM->preFilterCap = m_parameters.size() > 0 ? m_parameters[0] : 63;
      m_stereoSGBM->SADWindowSize = m_parameters.size() > 1 ? m_parameters[1] : 9;
      m_stereoSGBM->numberOfDisparities = m_parameters.size() > 2 ? m_parameters[2] : 256;
      m_stereoSGBM->uniquenessRatio = m_parameters.size() > 3 ? m_parameters[3] : 10;
      m_stereoSGBM->speckleWindowSize = m_parameters.size() > 4 ? m_parameters[4] : 100;
      m_stereoSGBM->speckleRange = m_parameters.size() > 5 ? m_parameters[5] : 32;
      m_stereoSGBM->disp12MaxDiff = m_parameters.size() > 6 ? m_parameters[6] : 1;

      m_stereoSGBM->P1 = 8*cn*m_stereoSGBM->SADWindowSize*m_stereoSGBM->SADWindowSize;
      m_stereoSGBM->P2 = 32*cn*m_stereoSGBM->SADWindowSize*m_stereoSGBM->SADWindowSize;
      m_stereoSGBM->fullDP = (stereoAlgo == STEREO_HH);
      m_stereoSGBM->minDisparity = 0;
      }
      break;
        
    case STEREO_GC:
      if( m_stereoGCstate==NULL ) {
        m_stereoGCstate = cvCreateStereoGCState( 
                  (m_parameters.size() > 0 ? m_parameters[0] : 256),
                  (m_parameters.size() > 1 ? m_parameters[1] : 2) );
      } else {
        m_stereoGCstate->numberOfDisparities = m_parameters.size() > 0 ? m_parameters[0] : 256;
        m_stereoGCstate->maxIters = m_parameters.size() > 1 ? m_parameters[1] : 2;
      }
      break;		

    default:
      return -1;
  }
	
  m_StereoAlgo = stereoAlgo;
	
  return 0;
}


void HRP2StereoVisionProcess::FreeImages()
{
  m_InputImages[0].release();
  m_InputImages[1].release();
  m_OutputImage.release();
  
  m_inputImagesLoaded = false;
  m_imgRectified = false;
  m_rangeMapComputed = false;  
}


void HRP2StereoVisionProcess::setImageSize( int w, int h )
{
  m_imgSize = Size( w, h );
  m_imgSizeKnown = true;
}


int HRP2StereoVisionProcess::loadCameraParameters( const char* intrinsicFilename, const char* extrinsicFilename )
{
  if( !m_imgSizeKnown ) return -1;
  
  // now only handling 2 cameras: left and right..
  FileStorage fs( intrinsicFilename, CV_STORAGE_READ );
  if( !fs.isOpened() ) return -2;

  Mat M1, D1, M2, D2;
  fs["M1"] >> M1;
  fs["D1"] >> D1;
  fs["M2"] >> M2;
  fs["D2"] >> D2;

  fs.open( extrinsicFilename, CV_STORAGE_READ );
  if( !fs.isOpened() ) return -3;

  Mat R, T, R1, P1, R2, P2;
  fs["R"] >> R;
  fs["T"] >> T;

  Rect roi1, roi2;

  stereoRectify( M1, D1, M2, D2, m_imgSize, R, T, R1, R2, P1, P2, m_Q, -1, m_imgSize, &roi1, &roi2 );
  initUndistortRectifyMap( M1, D1, R1, P1, m_imgSize, CV_16SC2, m_rectifyMap[0][0], m_rectifyMap[0][1] );
  initUndistortRectifyMap( M2, D2, R2, P2, m_imgSize, CV_16SC2, m_rectifyMap[1][0], m_rectifyMap[1][1] );
  
  if( m_StereoAlgo==STEREO_BM && m_stereoBM!=NULL ) {
    m_stereoBM->state->roi1 = roi1;
    m_stereoBM->state->roi2 = roi2;
  }
  
  m_cameraParamLoaded = true;
  
  return 0;
}


int HRP2StereoVisionProcess::stereoRectifyImages()
{
  if( !m_cameraParamLoaded || !m_inputImagesLoaded ) return -1;
  
  // only handling 2 cameras: left and right..
  Mat img1r, img2r;
  remap( m_InputImages[0], img1r, m_rectifyMap[0][0], m_rectifyMap[0][1], INTER_LINEAR );
  remap( m_InputImages[1], img2r, m_rectifyMap[1][0], m_rectifyMap[1][1], INTER_LINEAR );

  m_InputImages[0] = img1r;
  m_InputImages[1] = img2r;

  m_imgRectified = true;
  
  return 0;
}


int HRP2StereoVisionProcess::SetInputImages( Mat InputImages[2], bool rectified )
{
  FreeImages();

  if( InputImages[0].size() != InputImages[1].size() ) {
    m_inputImagesLoaded = false;
    m_imgSizeKnown = false;
    return -1;
  }

  m_InputImages[0] = InputImages[0].clone();
  m_InputImages[1] = InputImages[1].clone();
  m_imgSize = InputImages[0].size();
  
  // be careful about these specific cases:
  // 1- image sizes have changed
  // 2- input images are not rectified
  // => the user needs to call again loadCameraParameters !!

  m_imgSizeKnown = true;
  m_imgRectified = rectified;
  m_inputImagesLoaded = true;
  
  return 0;
}


int HRP2StereoVisionProcess::GetOutputImage( Mat& OutputImage )
{
  if( !m_rangeMapComputed ) return -1;
  
  OutputImage = m_OutputImage.clone();

  return 0;
}


int HRP2StereoVisionProcess::ComputeRangeMap()
{
  if( !m_inputImagesLoaded ) return -1;       // ERROR: no input images
  
  if( !m_imgRectified ) {
    if( stereoRectifyImages()!=0 ) return -2; // ERROR: input images are not rectified
  }
  
  switch( m_StereoAlgo ) {
    case STEREO_BM:
    {
      if( m_stereoBM==NULL ) return -3;
      (*m_stereoBM)( m_InputImages[0], m_InputImages[1], m_OutputImage );
    }
      break;
    case STEREO_SGBM:
    case STEREO_HH:
    {
      if( m_stereoSGBM==NULL ) return -3;
      (*m_stereoSGBM)( m_InputImages[0], m_InputImages[1], m_OutputImage );
    }
      break;

    case STEREO_GC:
    {
      CvMat omi1 = m_InputImages[0];
      CvMat omi2 = m_InputImages[1];
      CvSize size = cvGetSize( &omi1 );
      CvMat* disparity_left = cvCreateMat( size.height, size.width, CV_16S );
      CvMat* disparity_right = cvCreateMat( size.height, size.width, CV_16S );

      cvFindStereoCorrespondenceGC( &omi1, &omi2, disparity_left, disparity_right, m_stereoGCstate, 0 );

      m_OutputImage = disparity_left;
      
      //CvMat* disparity_left_visual = cvCreateMat( size.height, size.width, CV_8U );
      //cvConvertScale( disparity_left, disparity_left_visual, -16 );
      //m_OutputImage = disparity_left_visual;
    }
      break;

    default:
      break;
  }
  
  m_rangeMapComputed = true;
  
  return 0;
}


int HRP2StereoVisionProcess::Get3DPoints( Mat &xyz )
{
  if( !m_rangeMapComputed ) return -1;
  
  reprojectImageTo3D( m_OutputImage, xyz, m_Q, true );

  return 0;
}


int HRP2StereoVisionProcess::pCleanUpTheProcess()
{
  return 0;
}

#endif    /* LLVS_HAVE_OPENCV */
