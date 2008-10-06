/** @doc This object implements the abstract
    part of the acquisition of images.

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
#include "ImagesInputMethod.h"
using namespace llvs;
HRP2ImagesInputMethod::HRP2ImagesInputMethod()
{
  
}

HRP2ImagesInputMethod::~HRP2ImagesInputMethod()
{
}

int HRP2ImagesInputMethod::GetSingleImage(unsigned char **Images, int camera, struct timeval &timestamp)
{
  return -1;
}

int HRP2ImagesInputMethod::SetImageSize(int lw, int lh, int CameraNumber)
{
  if ((CameraNumber<0) || (CameraNumber>3))
    return -1;

  m_ImagesWidth[CameraNumber] = lw;
  m_ImagesHeight[CameraNumber] = lh;
  return 0;
}

int HRP2ImagesInputMethod::GetImageSize(int &lw, int &lh, int CameraNumber)
{
 if ((CameraNumber<0) || (CameraNumber>3))
    return -1;

  lw = m_ImagesWidth[CameraNumber];
  lh = m_ImagesHeight[CameraNumber];
  return 0;
}

string HRP2ImagesInputMethod::GetFormat(unsigned int CameraNumber)
{
  string aFormat("none");
  return aFormat;
}

int HRP2ImagesInputMethod::SetLevelOfVerbosity(int VerbosityParameter)
{
  m_Verbosity = VerbosityParameter;
  return m_Verbosity;
}

int HRP2ImagesInputMethod::GetLevelOfVerbosity()
{
  return m_Verbosity;
}

unsigned int HRP2ImagesInputMethod::GetNumberOfCameras()
{
  return 0;
}

double HRP2ImagesInputMethod::NextTimeForGrabbing(int CameraNumber)
{
  
  return -1.0;
}
