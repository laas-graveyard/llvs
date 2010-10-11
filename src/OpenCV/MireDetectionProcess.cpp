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
   @author Olivier Stasse, Torea Foissotte

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
#include "OpenCV/MireDetectionProcess.h"
#include <iostream>
#include <fstream>
#include <math.h>

// Debug macros
#include <llvs/tools/Debug.h>

#if LLVS_HAVE_OPENCV

HRP2MireDetectionProcess::HRP2MireDetectionProcess()
{
  m_ProcessName = "Mire Detection";

  m_Corners[0] = 0;
  m_Corners[1] = 0;

  SetChessBoardSize(5,7);
}


HRP2MireDetectionProcess::~HRP2MireDetectionProcess()
{
  if( m_Corners[0]!=0 ) delete m_Corners[0];
  if( m_Corners[1]!=0 ) delete m_Corners[1];
}

void HRP2MireDetectionProcess::SetChessBoardSize(int NbCols, int NbRows)
{
  m_ChessBoardSize[0] = NbCols;
  m_ChessBoardSize[1] = NbRows;

  ODEBUG3( NbCols << " " << NbRows);
  for(int i=0;i<2;i++)
    m_Corners[i] = new CvPoint2D32f[NbRows*NbCols];
}

int HRP2MireDetectionProcess::pInitializeTheProcess()
{

  return 0;
}

int HRP2MireDetectionProcess::pRealizeTheProcess()
{

  if (!m_Computing)
    return 0;

  int nb_corners;

  m_img = m_InputImages[0];
  m_imd = m_InputImages[1];

  nb_corners = DetectMireStereo();

  ODEBUG3(nb_corners);
  if (m_Verbosity>3)
    {
      for( int i=0;i<nb_corners;i++)
	{
	  cerr << m_Corners[0][i].x << " " << m_Corners[0][i].y << " "
	       << m_Corners[1][i].x << " " << m_Corners[1][i].y << " " << endl;
	}
    }

  return 0;
}

void HRP2MireDetectionProcess::SetInputImages( Mat InputImages[3] )
{
  m_InputImages[0] = InputImages[0].clone();
  m_InputImages[1] = InputImages[1].clone();
  m_InputImages[2] = InputImages[2].clone();

  m_img = Mat( m_InputImages[0].size(), CV_8UC1 );
  m_ImgTempG = Mat( m_InputImages[0].size(), CV_8UC1 );

  m_imd = Mat( m_InputImages[1].size(), CV_8UC1 );
  m_ImgTempD = Mat( m_InputImages[1].size(), CV_8UC1 );
}

void HRP2MireDetectionProcess::SetOutputImages( Mat OutputImages[3] )
{
  m_OutputImages[0] = OutputImages[0].clone();
  m_OutputImages[1] = OutputImages[1].clone();
  m_OutputImages[2] = OutputImages[2].clone();
}

int HRP2MireDetectionProcess::DetectMireStereo()
{
  // From Benoit Telle's fonctions.cpp
  CvSize sim;
  sim.width = m_img.cols;
  sim.height = m_img.rows;

  // detection grossiere
  CvSize sizeChess;
  sizeChess.width = m_ChessBoardSize[0];
  sizeChess.height = m_ChessBoardSize[1];

  int nb_points = (m_ChessBoardSize[0]-1)* (m_ChessBoardSize[1]-1);
  int nb_cornerg = nb_points;
  int nb_cornerd = nb_points;

  IplImage t_img = m_img;
  IplImage t_imd = m_imd;
  IplImage t_imtg = m_ImgTempG;
  IplImage t_imtd = m_ImgTempD;

  cvFindChessBoardCornerGuesses( &t_img, &t_imtg, 0, sizeChess, m_Corners[0], &nb_cornerg );
  cvFindChessBoardCornerGuesses( &t_imd, &t_imtd, 0, sizeChess, m_Corners[1], &nb_cornerd );


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
    cvFindCornerSubPix( &t_img, m_Corners[0], nb_cornerg, win, zeroZone, SubPixelCritere );

  if (nb_cornerd==nb_points)
    cvFindCornerSubPix( &t_imd, m_Corners[1], nb_cornerd,win, zeroZone, SubPixelCritere );

  if(nb_cornerg==nb_points && nb_cornerd==nb_points)
    nb_corner = nb_points;
  else
    nb_corner = 0;


  return(nb_corner);
}

int HRP2MireDetectionProcess::pCleanUpTheProcess()
{
  return 0;
}

#endif  /* LLVS_HAVE_OPENCV */
