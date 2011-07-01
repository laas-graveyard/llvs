/** @doc This object implements a color detection algorithm
    based on histograms.

   Copyright (c) 2003-2011, 
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
#include "ColorDetection.h"

#include <iostream>
#include <fstream>
#include <math.h>

// Debug macros
#include <llvs/tools/Debug.h>

HRP2ColorDetectionProcess::HRP2ColorDetectionProcess()
{
  m_CTS = 0x0;
  m_ProcessName = "Color Detection Based On Histogram";
  m_Verbosity = 3;
  m_Computing=0;
  m_ParametersSize = 0;

  for(int i=0;i<2;i++)
    {
      m_ImageInputRGB[i] = 0;
      m_ImageResult[i] = 0;
    }

}

HRP2ColorDetectionProcess::~HRP2ColorDetectionProcess()
{
  if (m_ImageInputRGB!=0)
    {
      for(int i=0;i<m_NbOfCameras;i++)
	delete m_ImageInputRGB[i];
      delete m_ImageInputRGB;
    }

  if (m_ImageResult!=0)
    {
      for(int i=0;i<m_NbOfCameras;i++)
	delete m_ImageResult[i];
      delete m_ImageResult;
    }

}



int HRP2ColorDetectionProcess::pInitializeTheProcess()
{
  // Initialize the Color Detector variables.

  m_ColorDetector[0].ReadFromFileAndCreateThePixelList("ColorModel.ppm");
  m_ColorDetector[1].ReadFromFileAndCreateThePixelList("ColorModel.ppm");

  return 0;
}


int HRP2ColorDetectionProcess::pRealizeTheProcess()
{
  unsigned int lw,lh;
  double x[m_NbOfCameras],y[m_NbOfCameras];
  int x1[2]={-1,-1},x2[2]={-1,-1};
  double a3DPt[3];

  for(int i=0;i<m_NbOfCameras;i++)
    {
      lw = m_ImageInputRGB[i]->getWidth();
      lh = m_ImageInputRGB[i]->getHeight();
      ODEBUG("Before filtering on histogram for image :" << i);
      m_ColorDetector[i].FilterOnHistogram(m_InputImage[i],m_ImageResult[i]);      
      ODEBUG("After filtering on histogram for image :" << i);
      if (1)
	{
	  char Buffer[1024];
	  sprintf(Buffer,"IntermediateImageResult_%03d.pgm",i);
	  string aFileName(Buffer);
	  vpImageIo::writePGM(*m_ImageResult[i],aFileName);
	}

      m_ColorDetector[i].ComputeCoG(m_ImageResult[i],x[i],y[i]);

      if (m_CTS!=0)
	{
	  if ((x[i]!=-1.0) && (y[i]!=-1.0))
	    {
	      double projectedCoG[3];
	      projectedCoG[0] = ( 2.0 * x[i]/(double)lw - 1);
	      projectedCoG[1] = ( 2.0 * y[i] /(double)lh - 1 );
	      projectedCoG[2] = 1.0;
	      ODEBUG3("Cam mb :" << i << "("<<x[i]<< ","<<y[i]<<") : " 
		      << projectedCoG[0]<< " " 
		      << projectedCoG[1]<< " " 
		      << projectedCoG[2]<< " ");
	      m_CTS-> WriteObjectCoG(projectedCoG);
	    }
	}
    }
  return 0;
}

int HRP2ColorDetectionProcess::pCleanUpTheProcess()
{
  return 0;
}

/*! Set the ConnectionToSot  pointer */
void HRP2ColorDetectionProcess::SetConnectionToSot (llvs::ConnectionToSot * aCTS)
{
  m_CTS = aCTS;
}


int HRP2ColorDetectionProcess::SetInputImages(unsigned char * InputImage,
					      int lwidth, int lheight)
{
  m_InputImage[0] = InputImage;
  m_ImageInputRGB[0] = new vpImage<vpRGBa>(lheight,lwidth);
  m_ImageResult[0] = new vpImage<unsigned char>(lheight,lwidth);
  m_ColorDetector[0].InitializeIntermediateStructure(m_ImageInputRGB[0]);
  m_NbOfCameras=1;

  return 0;
}

int HRP2ColorDetectionProcess::SetInputImages(unsigned char * InputImage1,unsigned char * InputImage2,
					      int lwidth, int lheight)
{
  m_InputImage[0] = InputImage1;
  m_InputImage[1] = InputImage2;

  m_ImageInputRGB[0] = new vpImage<vpRGBa>(lheight,lwidth);
  m_ImageInputRGB[1] = new vpImage<vpRGBa>(lheight,lwidth);

  m_ImageResult[0] = new vpImage<unsigned char>(lheight,lwidth);
  m_ImageResult[1] = new vpImage<unsigned char>(lheight,lwidth);

  m_ColorDetector[0].InitializeIntermediateStructure(m_ImageInputRGB[0]);
  m_ColorDetector[1].InitializeIntermediateStructure(m_ImageInputRGB[1]);


  m_NbOfCameras=2;
  
  return 0;
}



