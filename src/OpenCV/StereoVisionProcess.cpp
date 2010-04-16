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
#include <StereoVisionProcess.h>
#include <iostream>
#include <fstream>
#include <math.h>

// have to check if the line number is correct! 
// ..may end up with the macro definition line number.
#define ODEBUG2(x)
#define ODEBUG3(x) cerr << __FILE__ << ": l" << __LINE__ << ": " << x << endl
#define ODEBUG3_CONT(x) cerr << x 

#if 0
#define ODEBUG(x) cerr << __FILE__ << ": l" << __LINE__ << ": " << x << endl
#define ODEBUG_CONT(x) cerr << __FILE__ << ": l" << __LINE__ << ": " << x << endl
#else
#define ODEBUG(x) 
#define ODEBUG_CONT(x) 
#endif

HRP2StereoVisionProcess::HRP2StereoVisionProcess()
{
  m_ProcessName = "Stereo Vision";

  m_StereoAlgo = STEREO_SGBM;

#ifdef LLVS_HAVE_OPENCV
  m_stereoBM = NULL;
  m_stereoSGBM = new StereoSGBM;
#endif	
}


HRP2StereoVisionProcess::~HRP2StereoVisionProcess()
{ 
  FreeImages();  

#ifdef LLVS_HAVE_OPENCV
  if( m_stereoBM != NULL ) delete m_stereoBM;
  if( m_stereoSGBM != NULL ) delete m_stereoSGBM;
#endif	
}


int HRP2StereoVisionProcess::InitializeTheProcess()
{
  
  return 0;
}


int HRP2StereoVisionProcess::RealizeTheProcess()
{
  if( !m_Computing ) return 0;
  
  ComputeRangeMap();
  
  return 0;
}


int HRP2StereoVisionProcess::setAlgoParameters( std::vector<int> parameters )
{
  // TODO..
  switch( stereoAlgo ) {
    case STEREO_BM:
	  break;
			
	case STEREO_SGBM:
	  break;
			
	case STEREO_HH:
	  break;		

	default:
	  break;
  }
	
  return 0;
}


#ifdef LLVS_HAVE_OPENCV

int HRP2StereoVisionProcess::setStereoAlgorithmType( StereoAlgoType stereoAlgo )
{
  switch( stereoAlgo ) {
    case STEREO_BM:
	  if( m_stereoBM==NULL ) m_stereoBM = new StereoBM;
	  break;
			
	case STEREO_SGBM:
	  if( m_stereoSGBM==NULL ) m_stereoSGBM = new StereoSGBM;
	  break;
			
	case STEREO_HH:
	  // TODO..
	  break;		

	default:
	  break;
  }
	
  m_StereoAlgo = stereoAlgo;
	
  return 0;
}


void HRP2StereoVisionProcess::FreeImages()
{
  for ( int i=0; i<3; ++i ) {
	  m_InputImages[i].release();
  }
  m_OutputImage.release();
}

void HRP2StereoVisionProcess::SetInputImages( const Mat& InputImages[3] )
{
  FreeImages();

  for( int i=0; i<3; ++i ) {
    m_InputImages[i] = InputImages[i].clone();
  }

}

void HRP2StereoVisionProcess::SetOutputImage( Mat& OutputImage )
{
  m_OutputImage = OutputImage.clone();
}


int HRP2StereoVisionProcess::ComputeRangeMap()
{
  switch( m_StereoAlgo ) {
	case STEREO_BM:
	{
	  if( m_stereoBM==NULL ) return -1;
	  m_stereoBM( m_InputImages[0], m_InputImages[1], m_OutputImage );
	}
	  break;
	case STEREO_SGBM:
	{
	  if( m_stereoSGBM==NULL ) return -1;
	  m_stereoSGBM( m_InputImages[0], m_InputImages[1], m_OutputImage );
	}
	  break;

	case STEREO_HH:
	{
	}
	  break;

	default:
	  break;
  }
  return 0;
}


#else

// TODO: implement the functions when the image type is decided..

int HRP2StereoVisionProcess::setStereoAlgorithmType( StereoAlgoType stereoAlgo )
{
  // TODO ...
  m_StereoAlgo = stereoAlgo;
  return 0;
}


void HRP2StereoVisionProcess::FreeImages()
{
  for ( int i=0; i<3; ++i ) {
	  //m_InputImages[i].release();
	  //m_OutputImages[i].release();
  }
}

void HRP2StereoVisionProcess::SetInputImages( void* InputImages[3] )
{
  FreeImages();

  for( int i=0; i<3; ++i ) {
    //m_InputImages[i] = InputImages[i].clone();
  }

}

void HRP2StereoVisionProcess::SetOutputImages( void* OutputImages[3] )
{
  for( int i=0; i<3; ++i ) {
    //m_OutputImages[i] = OutputImages[i].clone();
  }
}

int HRP2StereoVisionProcess::ComputeRangeMap()
{
  return 0;
}
#endif


int HRP2StereoVisionProcess::CleanUpTheProcess()
{
  return 0;
}


