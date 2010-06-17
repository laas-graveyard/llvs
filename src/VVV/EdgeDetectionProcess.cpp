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
#include <iostream>

#include "EdgeDetectionProcess.h"

// Debug macros
#include <llvs/tools/Debug.h>

extern "C" {
#include "epbmstep1.h"
}

HRP2EdgeDetectionProcess::HRP2EdgeDetectionProcess()
{
  m_ProcessName = "Edge Detection Process";
  for(int i=0;i<3;i++)
    m_ImagesOnWhichToProcess[i]=1;
      
}

HRP2EdgeDetectionProcess::~HRP2EdgeDetectionProcess()
{
}

void HRP2EdgeDetectionProcess::SetImageOnWhichToProcess(int ImagesOnWhichToProcess[3])
{
  for(int i=0;i<3;i++)
    m_ImagesOnWhichToProcess[i] = ImagesOnWhichToProcess[i];

}

int HRP2EdgeDetectionProcess::InitializeTheProcess()
{
  for(int i=0;i<3;i++)
    {
      m_Edge[i] = epbm_create(EPBM_BINARY_GRAY,
			      m_InputImage[i].Width,
			      m_InputImage[i].Height,
			      m_InputImage[i].Sign,
			      EPBM_INT32,
			      0);

      m_Grad[i] = epbm_create(EPBM_BINARY_GRAY,
			      m_InputImage[i].Width,
			      m_InputImage[i].Height,
			      m_InputImage[i].Sign,
			      EPBM_CHAR8,
			      0);
    }
  return 0;
}


int HRP2EdgeDetectionProcess::RealizeTheProcess()
{
  if (!m_Computing)
    return 0;
    
  epbm_save("Input0.epbm",&m_InputImage[0],0);
  switch(m_Operator)
    {
    case SOBEL:     
      for(int i=0;i<3;i++)
	if (m_ImagesOnWhichToProcess[i]!=0)
	  epbm_msobel(&m_InputImage[i],&m_Edge[i],&m_Grad[i],&m_mu[i], &m_sigma[i]);
      break;
    case LAPLACIAN:
      for(int i=0;i<3;i++)
	if (m_ImagesOnWhichToProcess[i]!=0)
	  epbm_laplacian(&m_InputImage[i],&m_Edge[i],&m_Grad[i],&m_mu[i], &m_sigma[i],0);
      break;
    }
  return 0;
}

int HRP2EdgeDetectionProcess::CleanUpTheProcess()
{
  for(int i=0;i<3;i++)
    {
      epbm_free(&m_Edge[i]);
      epbm_free(&m_Grad[i]);
    }
  return 0;
}


int HRP2EdgeDetectionProcess::SetInputImages(EPBM lInputImages[3])
{
  for(int i=0;i<3;i++)
    m_InputImage[i] = lInputImages[i];
  return 0;
}


