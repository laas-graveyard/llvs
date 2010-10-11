/** @doc This object implements a visual process
    to find features inside the environment and get their
    3D location.

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
#include <FindFeaturesInImage.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <MonoSLAM/monoslaminterface.h>

// Debug macros
#include <llvs/tools/Debug.h>

HRP2FindFeaturesInImage::HRP2FindFeaturesInImage()
{
  m_ProcessName = "Find Features in image";
  m_Verbosity = 0;
  m_Computing=1;
  m_State = 0;
  m_FFFWI = 0;
  for(int i=0;i<3;i++)
    m_grabbed_image[i] = 0;
}

HRP2FindFeaturesInImage::~HRP2FindFeaturesInImage()
{
}


int HRP2FindFeaturesInImage::InitializeTheProcess(string PathForFirstCalibrationSet,
						  string PathForSndCalibrationSet,
						  string ConvexPolytopePath)
{
  if (m_FFFWI==0)
    m_FFFWI = new FindFeaturesFromWideImage(PathForFirstCalibrationSet,
					    PathForSndCalibrationSet,
					    ConvexPolytopePath);
  else
    {
      m_FFFWI->SetPathForSndCalibrationSet(PathForSndCalibrationSet);
      m_FFFWI->SetConvexPolytopePath(ConvexPolytopePath);
    }
  return 0;
}

int HRP2FindFeaturesInImage::RealizeTheProcess()
{

  if ((!m_Computing) || (!m_State))
    return 0;

  ODEBUG3("Go through here");
  for(int i=0;i<2;i++)
    {
      // Get back to the image.
      ImageConversions::MonoBuffer_to_VWImage((char *)m_InputImage[i].Image,m_grabbed_image[i],
					      m_InputImage[i].Width, m_InputImage[i].Height,
					      1, 1);
    }
  ImageConversions::MonoBuffer_to_VWImage((char *)m_InputImage[3].Image,m_grabbed_image[2],
					  m_InputImage[3].Width, m_InputImage[3].Height,
					  1, 1);
  if(m_State==1)
    {
      m_FFFWI->SetImages(m_grabbed_image[0],m_grabbed_image[1],m_grabbed_image[2]);
      m_FFFWI->FindSetOfPossibleInitialFeatures();
      m_State = 2;
      m_FeatureID=0;
    }

  if (m_State==3)
    {
      m_FFFWI->SetImages(m_grabbed_image[0],m_grabbed_image[1],m_grabbed_image[2]);
      m_FFFWI->FindAPatchIn3DFromWideLensImage(m_FeatureID++);

      if (m_FeatureID==m_FFFWI->NbOfFeaturesForInitialization())
	m_State=0;
      else
	m_State=2;
    }

  return 0;
}

int HRP2FindFeaturesInImage::CleanUpTheProcess()
{
  return 0;
}

int HRP2FindFeaturesInImage::StopProcess()
{
  m_Computing = 0;
  return 1;


}

int HRP2FindFeaturesInImage::StartProcess()
{
  cout <<m_ProcessName << endl;
  m_Computing = 1;
  return 1;

}

int HRP2FindFeaturesInImage::GetStatus()
{
  return m_Computing;
}

unsigned char HRP2FindFeaturesInImage::GetLevelOfVerbosity()
{
  return m_Verbosity;
}

void HRP2FindFeaturesInImage::SetLevelOfVerbosity(unsigned char aVerbosity)
{
  m_Verbosity = aVerbosity;
}

string HRP2FindFeaturesInImage::GetName()
{
  return m_ProcessName;
}

int HRP2FindFeaturesInImage::SetParameter(string aParameter, string aValue)
{
  unsigned char ok = 0;
  ODEBUG3("Came into SetParameter " << aParameter << " " << aValue);
  if (aParameter=="State")
    {
      int lState = strtol(aValue.c_str(), (char **)NULL, 10);
      if ((lState>=0) &&  (lState<4))
	m_State = lState;
      cout << "New value for State" << m_State << endl;
    }
  else if (aParameter=="pathsndcalib")
    {
      if (m_FFFWI!=0)
	m_FFFWI->SetPathForSndCalibrationSet(aValue);
      ok=1;
    }

  else if (aParameter=="polytopepath")
    {
      if (m_FFFWI!=0)
	m_FFFWI->SetConvexPolytopePath(aValue);
      ok=1;
    }


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
  return m_ParametersSize-1;
}

int HRP2FindFeaturesInImage::GetParameter(string & aParameter, string &aValue, int anIndex)
{
  if ((anIndex<0) || (anIndex>=m_ParametersSize))
    return -1;

  aParameter = m_VectorOfParameters[anIndex];
  aValue = m_VectorOfValuesForParameters[anIndex];
  return 0;
}

int HRP2FindFeaturesInImage::GetValueOfParameter(string aParameter, string &aValue)
{
  if (aParameter=="GetNbOfCandidates")
    {
      char Buffer[128];
      bzero(Buffer,128);
      sprintf(Buffer,"%d",m_FFFWI->NbOfFeaturesForInitialization());
      aValue = Buffer;
    }
  else  if (aParameter=="GetCandidatesAngles")
    {
      vector<FeatureCandidate> aVectorOfFC;
      m_FFFWI->GetFeatures(aVectorOfFC);
      aValue = "";
      for(unsigned int i=0;i<aVectorOfFC.size();i++)
	{
	  char Buffer[128];
	  bzero(Buffer,128);
	  sprintf(Buffer,"(%f,%f)",
		  aVectorOfFC[i].pan,
		  aVectorOfFC[i].tilt);
	  aValue += Buffer;
	}
    }
  else if (aParameter=="GetCandidatesPositions")
    {
      vector<FeatureCandidate> aVectorOfFC;
      m_FFFWI->GetFeatures(aVectorOfFC);
      aValue = "";
      for(unsigned int i=0;i<aVectorOfFC.size();i++)
	{
	  char Buffer[128];
	  bzero(Buffer,128);
	  sprintf(Buffer,"(%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f)",
		  aVectorOfFC[i].X[0],
		  aVectorOfFC[i].X[1],
		  aVectorOfFC[i].X[2],
		  aVectorOfFC[i].Cov[0],
		  aVectorOfFC[i].Cov[1],
		  aVectorOfFC[i].Cov[2],
		  aVectorOfFC[i].Cov[3],
		  aVectorOfFC[i].Cov[4],
		  aVectorOfFC[i].Cov[5],
		  aVectorOfFC[i].Cov[6],
		  aVectorOfFC[i].Cov[7],
		  aVectorOfFC[i].Cov[8]);
	  aValue += Buffer;
	}
    }
  else for(int i=0;i<m_ParametersSize;i++)
    {
      if (m_VectorOfParameters[i]==aParameter)
	{
	  aValue = m_VectorOfValuesForParameters[i];
	  return 0;
	}
    }

  return -1;
}

int HRP2FindFeaturesInImage::GetParametersAndValues(vector<string> &ListOfParameters, vector<string> & ListOfValues)
{
  ListOfParameters = m_VectorOfParameters;
  ListOfValues = m_VectorOfValuesForParameters;
  return 0;
}

int HRP2FindFeaturesInImage::SetInputImages(EPBM * aInputImage)
{

  for(int i=0;i<3;i++)
    {
      if (m_grabbed_image[i]!=0)
	delete m_grabbed_image[i];
      m_grabbed_image[i] = new ImageMono<unsigned char>(aInputImage[i].Width,
							aInputImage[i].Height);

      m_InputImage[i] = aInputImage[i];
    }

  m_InputImage[3] = aInputImage[3];
  return 0;
}

FindFeaturesFromWideImage * HRP2FindFeaturesInImage::GetFindFeaturesFromWideImage()
{
  return m_FFFWI;
}
