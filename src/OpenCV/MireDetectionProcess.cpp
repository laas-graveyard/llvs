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
#include <MireDetectionProcess.h>
#include <iostream>
#include <fstream>
#include <math.h>

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "HPR2MireDetectionProcess:" << x << endl
#define ODEBUG3_CONT(x) cerr << x 

#if 0
#define ODEBUG(x) cerr << "HPR2MireDetectionProcess:" <<  x << endl
#define ODEBUG_CONT(x) cerr << "HPR2MireDetectionProcess:" <<  x << endl
#else
#define ODEBUG(x) 
#define ODEBUG_CONT(x) 
#endif

HRP2MireDetectionProcess::HRP2MireDetectionProcess()
{
  m_ProcessName = "Mire Detection";

  m_img = 0;
  m_imd = 0;
  m_ImgTempG = 0;
  m_ImgTempD = 0;
  m_Corners[0] = 0;
  m_Corners[1] = 0;

  SetChessBoardSize(5,7);
}


HRP2MireDetectionProcess::~HRP2MireDetectionProcess()
{ 
  FreeImages();  
  
  for(int i=0;i<2;i++)
    if (m_Corners[i]!=0)
      delete m_Corners[i];
}

void HRP2MireDetectionProcess::SetChessBoardSize(int NbCols, int NbRows)
{
  if (m_Corners[0]!=0)
    delete m_Corners[0];
  if (m_Corners[1]!=0)
    delete m_Corners[1];

  m_ChessBoardSize[0] = NbCols;
  m_ChessBoardSize[1] = NbRows;
  

  ODEBUG3( NbCols << " " << NbRows);
  for(int i=0;i<2;i++)
    m_Corners[i] = new CvPoint2D32f[NbRows*NbCols];
}

int HRP2MireDetectionProcess::InitializeTheProcess()
{
  
  return 0;
}

int HRP2MireDetectionProcess::RealizeTheProcess()
{

  if (!m_Computing)
    return 0;
  

  int nb_corners;
  int i,j;
  for(j=0;j<m_InputImages[0].Width*m_InputImages[0].Height;j++)
    m_img->imageData[j] = ((unsigned char *)m_InputImages[0].Image)[j];

  for(j=0;j<m_InputImages[1].Width*m_InputImages[1].Height;j++)
    m_imd->imageData[j] = ((unsigned char *)m_InputImages[1].Image)[j];
  
  nb_corners = DetectMireStereo();
  
  ODEBUG3(nb_corners);
  if (m_Verbosity>3)
    {
      for(i=0;i<nb_corners;i++)
	{
	  cerr << m_Corners[0][i].x << " " << m_Corners[0][i].y << " " 
	       << m_Corners[1][i].x << " " << m_Corners[1][i].y << " " << endl;
	}
    }
 
  return 0;
}

void HRP2MireDetectionProcess::FreeImages()
{
  if (m_img!=0)
    {
      cvReleaseImage(&m_img);
      m_img = 0;
    }
  
  if (m_imd!=0)
    {
      cvReleaseImage(&m_imd);
      m_imd=0;
    }

  if (m_ImgTempG!=0)
    {
      cvReleaseImage(&m_ImgTempG);
      m_ImgTempG = 0;
    }

  if (m_ImgTempD!=0)
    {
      cvReleaseImage(&m_ImgTempD);
      m_ImgTempD = 0;
    }

}

void HRP2MireDetectionProcess::SetInputImages(EPBM InputImages[3])
{
  int i;
  
  FreeImages();

  for(i=0;i<3;i++)
    m_InputImages[i] = InputImages[i];

  CvSize sim; 
  
  sim.width = m_InputImages[0].Width; 
  sim.height = m_InputImages[0].Height;

  m_img = cvCreateImage(sim,8,1);
  m_ImgTempG = cvCreateImage(sim,8,1);


  sim.width = m_InputImages[1].Width; 
  sim.height = m_InputImages[1].Height;

  m_imd = cvCreateImage(sim,8,1);
  m_ImgTempD = cvCreateImage(sim,8,1);
	

}

void HRP2MireDetectionProcess::SetOutputImages(EPBM OutputImages[3])
{
  int i;
  
  for(i=0;i<3;i++)
    m_OutputImages[i] = OutputImages[i];
}

int HRP2MireDetectionProcess::DetectMireStereo()
{
  // From Benoit Telle's fonctions.cpp
  CvSize sim; sim.width = m_img->width; sim.height = m_img->height;
    
  // detection grossiere
  CvSize sizeChess;
  sizeChess.width = m_ChessBoardSize[0];
  sizeChess.height = m_ChessBoardSize[1];
  
  int nb_points = (m_ChessBoardSize[0]-1)* (m_ChessBoardSize[1]-1);
  int nb_cornerg = nb_points;
  int nb_cornerd = nb_points;


  cvFindChessBoardCornerGuesses(m_img,m_ImgTempG,0,sizeChess,m_Corners[0],&nb_cornerg);
  cvFindChessBoardCornerGuesses(m_imd,m_ImgTempD,0,sizeChess,m_Corners[1],&nb_cornerd);


  // subpixel precision
  CvSize win;
  win.width = 5;
  win.height = 5;
  
  CvSize zeroZone;
  zeroZone.width = -1;
  zeroZone.height = -1;
  
  CvTermCriteria SubPixelCritere;				
  SubPixelCritere.type = CV_TERMCRIT_ITER;//CV_TERMCRIT_EPS;
  //  SubPixelCritere.maxIter = 10;
  SubPixelCritere.epsilon = 0.000001f;
  
  int nb_corner = 0;
  
  if (nb_cornerg==nb_points)
    cvFindCornerSubPix( m_img, m_Corners[0], nb_cornerg,win, 
			zeroZone, SubPixelCritere );
  if (nb_cornerd==nb_points)
    cvFindCornerSubPix( m_imd, m_Corners[1], nb_cornerd,win, 
			zeroZone, SubPixelCritere );
  
  if(nb_cornerg==nb_points && nb_cornerd==nb_points)
    nb_corner = nb_points;
  else
    nb_corner = 0;
  
  
  return(nb_corner);
}

int HRP2MireDetectionProcess::CleanUpTheProcess()
{
  return 0;
}


