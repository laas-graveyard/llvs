/** @doc This object implements the abstract basic visual process
    for the vision system.

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
#include <VisionBasicProcess.h>
#include <iostream>
#include <fstream>
#include <math.h>

#include "Debug.h"

HRP2VisionBasicProcess::HRP2VisionBasicProcess(int Instance)
  : m_Instance(Instance)
{
  m_ProcessName = "void";
  m_Verbosity = 0;
  m_Computing=1;
}

HRP2VisionBasicProcess::~HRP2VisionBasicProcess()
{
}

/*
int HRP2VisionBasicProcess::pInitializeTheProcess()
{
  return 0;
}

int HRP2VisionBasicProcess::pRealizeTheProcess()
{
  return 0;
}

int HRP2VisionBasicProcess::pCleanUpTheProcess()
{
  return 0;
}
*/
int HRP2VisionBasicProcess::StopProcess()
{
  m_Computing = 0;
  pStopProcess();
  return 1;
}

int HRP2VisionBasicProcess::StartProcess()
{
  cout << "Go through StartProcess" << endl;
  m_Computing = 1;
  pStartProcess();
  cout << "Went through StartProcess" << endl;
  return 1;

}

int HRP2VisionBasicProcess::GetStatus()
{
  return m_Computing;
}

unsigned char HRP2VisionBasicProcess::GetLevelOfVerbosity()
{
  return m_Verbosity;
}

void HRP2VisionBasicProcess::SetLevelOfVerbosity(unsigned char aVerbosity)
{
  m_Verbosity = aVerbosity;
}

string HRP2VisionBasicProcess::GetName()
{
  return m_ProcessName;
}

int HRP2VisionBasicProcess::SetParameter(string aParameter, string aValue)
{
  unsigned char ok = 0;

  for(unsigned int i=0;i<m_VectorOfParameters.size();i++)
    {
      if (m_VectorOfParameters[i] == aParameter)
	{
	  m_VectorOfValuesForParameters[i] = aValue;
	  ok = 1;
	}
    }
  
  if (!ok)
    {
      m_VectorOfParameters.insert(m_VectorOfParameters.end(), aParameter);
      m_VectorOfValuesForParameters.insert(m_VectorOfValuesForParameters.end(),
					   aValue);
      m_ParametersSize++;
    }

  // Call Virtual method to be reimplemented by inherited class.
  pSetParameter(aParameter,aValue);
  return m_ParametersSize-1;
}

int HRP2VisionBasicProcess::GetParameter(string & aParameter, string &aValue, int anIndex)
{
  if ((anIndex<0) || (anIndex>=m_ParametersSize))
    return -1;

  aParameter = m_VectorOfParameters[anIndex];
  aValue = m_VectorOfValuesForParameters[anIndex];

  pGetParameter(aParameter,aValue,anIndex);
  return 0;
}

int HRP2VisionBasicProcess::GetValueOfParameter(string aParameter, string &aValue)
{
 
  pGetValueOfParameter(aParameter,aValue);
  for(int i=0;i<m_ParametersSize;i++)
    {
      if (m_VectorOfParameters[i]==aParameter)
	{
	  aValue = m_VectorOfValuesForParameters[i];
	  return 0;
	}
    }
 
  return -1;
}

int HRP2VisionBasicProcess::GetParametersAndValues(vector<string> &ListOfParameters, vector<string> & ListOfValues)
{
  ListOfParameters = m_VectorOfParameters;
  ListOfValues = m_VectorOfValuesForParameters;
  return pGetParametersAndValues(ListOfParameters,ListOfValues);
}

#if ((LLVS_HAVE_OPENCV>0) && (LLVS_HAVE_VVV>0))

IplImage **HRP2VisionBasicProcess::ToIPL(unsigned char *Images[3],int Sizes[3][2])
{
  IplImage **Dests;
  int i,j,depth;
  CvSize image_size;

  Dests = new IplImage *[3];

  for(i=0;i<3;i++)
    {
      image_size.width = Sizes[i][0];
      image_size.height = Sizes[i][1];
      depth = 8;
      Dests[i] = cvCreateImage(image_size, depth, 1);
      for(j=0;j<Sizes[i][0]*Sizes[i][1];j++)
	Dests[i]->imageData[j] = Images[i][j];
    }

  return Dests;

}

void HRP2VisionBasicProcess::ToEPBMFile(IplImage *ThreeImages[3], string BaseName)
{
  int i;
  char aFileName[1024];
  bzero(aFileName,1024);

  sprintf(aFileName,"%s.epbm",BaseName.c_str());
  ofstream aofstream; 
  aofstream.open(aFileName,ofstream::out | iostream::binary);
  
  for(i=0;i<3;i++)
    {

      if (ThreeImages[i]==0)
	continue;

      if (aofstream.is_open())
	{
	  int j;
	  if (ThreeImages[i]->nChannels==1)
	    aofstream << "P5" <<endl;
	  else
	    aofstream << "P6" << endl;
	  aofstream << ThreeImages[i]->width << " " << ThreeImages[i]->height << endl;
	  aofstream << (int)(pow((double)2.0,(double)(ThreeImages[i]->depth))-1) << endl;
	  for(j=0;j<ThreeImages[i]->width * ThreeImages[i]->height*(ThreeImages[i]->depth/8)*
		ThreeImages[i]->nChannels;
	      j+=(ThreeImages[i]->depth/8)*ThreeImages[i]->nChannels)
	    aofstream.write(&ThreeImages[i]->imageData[j],
			    (ThreeImages[i]->depth/8)*ThreeImages[i]->nChannels);
	}

    }
  aofstream.close();
}

IplImage * HRP2VisionBasicProcess::FromEPBMToIPL(EPBM &anEPBM, int aMode, IplImage * ExistingIPL)
{
  IplImage *anImage=0;
  CvSize ImageSize;
  int depth=0;
  int channel=0;

  ImageSize.width = anEPBM.Width;
  ImageSize.height = anEPBM.Height;
  if (anEPBM.Magic2=='4')
    {
      depth=2;
      channel=1;
    }
  else 
    {
      depth=8;
      if (anEPBM.Magic2=='5')
	channel=1;
      else if (anEPBM.Magic2=='6')
	channel=3;
	  
    }
  switch(aMode)
    {

    case HEADER_IPL:
      anImage = new IplImage;
      anImage->imageData = (char *)anEPBM.Image;
      anImage->imageDataOrigin = (char *)anEPBM.Image;
      break;

    case EXISTING_IPL:
      anImage = ExistingIPL;
      if ((anImage->width!=anEPBM.Width) ||
	  (anImage->height!=anEPBM.Height)||
	  (anImage->depth!=depth) ||
	  (anImage->nChannels!=channel))
	return 0;
      break;

    case NEW_IPL:
      anImage = cvCreateImage(ImageSize,depth,channel);
      break;

    default:
      break;
    }

  if ((aMode==HEADER_IPL) || (aMode==EXISTING_IPL))
    {
      anImage->nSize = sizeof(IplImage);
      anImage->ID= 0;
      anImage->nChannels = channel;
      anImage->alphaChannel = 0;
      bzero(anImage->colorModel,4);
      bzero(anImage->channelSeq,4);
      anImage->depth = depth;
      anImage->dataOrder = 0;
      anImage->origin = 0;
      anImage->width = anEPBM.Width;
      anImage->height = anEPBM.Height;
      anImage->roi = 0;
      anImage->maskROI=0;
      anImage->imageId = 0;
      anImage->tileInfo = 0;
      anImage->imageSize = (int)(anImage->height*anImage->width*(anImage->depth/8.0)*anImage->nChannels);
      anImage->widthStep = anImage->width;
      bzero(anImage->BorderMode,4);
      bzero(anImage->BorderConst,4);      
      if (aMode==HEADER_IPL)
	return 0;
    }
  
  memcpy(anImage->imageData, anEPBM.Image,anImage->imageSize);

  return 0;
}
#endif

int HRP2VisionBasicProcess::GetInstance()
{
  return m_Instance;
}

int HRP2VisionBasicProcess::InitializeTheProcess()
{
  return pInitializeTheProcess();
}

int HRP2VisionBasicProcess::RealizeTheProcess()
{
  ODEBUG( m_ProcessName << " Activated:" << (int)m_Computing);
  if (m_Computing==1)
    return pRealizeTheProcess();
    
  return 0;
}

int HRP2VisionBasicProcess::CleanUpTheProcess()
{
  return pCleanUpTheProcess();
}
