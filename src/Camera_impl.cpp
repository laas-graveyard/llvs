/** @doc This object implements an abstract class
    of camera.
    
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
#include <math.h>
#include "Camera_impl.h"
extern "C"
{
#include "scm.h"
}
/* Constructor for this class */
Camera_impl::Camera_impl(const char *location,LowLevelVisionServer *aLLVS)
{
  SetFather(aLLVS);
  ResetCameraParameters();
  ResetIntrinsicParameters();
}

/* Destructor for this class */
Camera_impl::~Camera_impl()
{
}

void Camera_impl::SetVerbosity(int aVerbosity)
{
  m_Verbosity = aVerbosity;
}

int Camera_impl::GetVerbosity()
{
  return m_Verbosity;
}

void Camera_impl::ResetCameraParameters()
{
  m_CameraParameter.type = -1;
  m_CameraParameter.CameraId = -1;
  m_CameraParameter.CameraName = "";
  m_CameraParameter.Width=-1;
  m_CameraParameter.Height=-1;
  m_CameraParameter.CalibrationWidth = -1;
  m_CameraParameter.CalibrationHeight = -1;
  
}

void Camera_impl::ResetIntrinsicParameters()
{
  m_IntrinsicParameter.Focal = 0.0;
  m_IntrinsicParameter.SkewFactor = 0.0;
  for(int i=0;i<2;i++)
    m_IntrinsicParameter.Scale[i] = 0.0;
  for(int i=0;i<2;i++)
    m_IntrinsicParameter.ImageCenter[i] = 0.0;
  
}

/* Set the father */
void Camera_impl::SetFather(LowLevelVisionServer *aFather)
{
  m_LLVSFather = aFather;
}

/* Get the father */
LowLevelVisionServer * Camera_impl::GetFather()
{
  return m_LLVSFather;
}

/* Destroy the camera */
void Camera_impl::destroy()
{

}

HRP2Camera::CameraParameter *  Camera_impl::GetCameraParameter() throw (CORBA::SystemException)
{
  HRP2Camera::CameraParameter *aCP = new HRP2Camera::CameraParameter;
  *aCP = m_CameraParameter;
  return aCP;
}

/* Set the identifier */
void Camera_impl::SetIdentifier(int anID)
{
  m_CameraParameter.CameraId = anID;
}

/* Get the identifier */
int Camera_impl::GetIdentifier()
{
  return m_CameraParameter.CameraId;
}

/* Set the Name */
void Camera_impl::SetName(string aName)
{
  m_CameraParameter.CameraName = aName.c_str();
}

/* Get the Name */
string Camera_impl::GetName()
{
  return CORBA::string_dup(m_CameraParameter.CameraName);
}

/* Set the Camera Type */
void Camera_impl::SetCameraType(int aCameraType)
{
  m_CameraParameter.type = aCameraType;
}

/* Get the Name */
int Camera_impl::GetCameraType()
{
  return m_CameraParameter.type;
}

/* Set Acquisition size */
CORBA::Long Camera_impl::SetAcquisitionSize(CORBA::Long aWidth, CORBA::Long aHeight)
  throw(CORBA::SystemException)
{
  m_CameraParameter.Width = aWidth;
  m_CameraParameter.Height = aHeight;

  return 0;
}

HRP2Camera::IntrinsicParameters Camera_impl::GetIntrinsicParameters()
  throw(CORBA::SystemException)
{
  return m_IntrinsicParameter;
}

void Camera_impl::SetIntrinsicParameters(float aFocal, float aScale[2], float SkewFactor, float ImageCenter[2])
{
  m_IntrinsicParameter.Focal = aFocal;
  for(int i=0;i<2;i++)
    m_IntrinsicParameter.Scale[i] = aScale[i];

  m_IntrinsicParameter.SkewFactor = SkewFactor;
  for(int i=0;i<2;i++)
    m_IntrinsicParameter.ImageCenter[i] = 
      ImageCenter[i];
  
}

void Camera_impl::SetCameraParameter(long aWidth,long aHeight, long CalibrationWidth, long CalibrationHeight)
{
  m_CameraParameter.Width = aWidth;
  m_CameraParameter.Height = aHeight;
  m_CameraParameter.CalibrationWidth = CalibrationWidth;
  m_CameraParameter.CalibrationHeight = CalibrationHeight;
}
					 
void Camera_impl::SetCameraProjectiveParameters(  SCM_PARAMETER *sp, int camera_number)
{

  double f;
  MAT_Vector t;
  MAT_Matrix R;
  double q[3][3];
  double r[3][3];
  double a[3][3];
  double aspectratio;
  double theta;
  MAT_Vector iccr;
  int i,j,l;
  t = mat_new_vector(3,0);
  R = mat_new_matrix(3,3,0);
  iccr = mat_new_vector(3,0);

  
  scm_CalcCameraParameter(sp,camera_number,
			  (MAT_Vector)t,  
			  (MAT_Matrix)R,
			  &f, &aspectratio,&theta,(MAT_Vector)&iccr,0);
  
  //      fprintf(stderr,"f: %f aspectratio: %f theta: %f\n",f,aspectratio,theta);  
  for(j=0;j<3;j++)
    {
      m_ProjectiveParameters.PositionVector[j] = t[j];
      if (m_Verbosity>3)
	printf("PositionCamera[%d][%d]= %20.20f;\n",
		camera_number,j,t[j]);
    }
  
  for(j=0;j<3;j++)
    {
      for(i=0;i<3;i++)
	{
	  m_ProjectiveParameters.RotationMatrix[j][i] = R[j][i];
	  if (m_Verbosity>3)
	    printf("RotationCamera[%d][%d][%d] = %20.20f;",
		    camera_number,j,i,R[j][i]);
	}
      if (m_Verbosity>3)
	fprintf(stderr,"\n");
    }
  
  scm_H2q((double (*)[4])sp->H[camera_number][0],q,0);
  scm_q2r(q,r,0);
  scm_matrix_by_matrix_3D((double *)q, (double *)r, (double *)a);
  
  for(j=0;j<3;j++)
    {
      for(i=0;i<3;i++)
	{
	  m_ProjectiveParameters.IntrinsicMatrix[j][i] = a[j][i];
	  if (m_Verbosity>3)
	    fprintf(stderr,"IntrinsicParameter[%d][%d][%d] = %20.20f;",
		    camera_number,j,i,a[j][i]);
	}
      if (m_Verbosity>3)
	fprintf(stderr,"IntrinsicParameter[%d][%d][3] = 0.0;\n",camera_number,j);
    }
  

  for(j=0;j<3;j++)
    {
      for(i=0;i<4;i++)
	{
	  m_ProjectiveParameters.ProjectiveMatrix[j][i] = sp->H[camera_number][j][i];
	  if (m_Verbosity>3)
	    fprintf(stderr,"IntrinsicParameter[%d][%d][%d] = %20.20f;",
		    camera_number,j,i,a[j][i]);
	}
      if (m_Verbosity>3)
	fprintf(stderr,"IntrinsicParameter[%d][%d][3] = 0.0;\n",camera_number,j);
    }
    
}

void Camera_impl::GetOriginalProjectiveMatrix(double oP[3][4])
{
  int sdet = 1;
  for(int i=0;i<3;i++)
    for(int j=0;j<4;j++)
      oP[i][j] = m_ProjectiveParameters.ProjectiveMatrix[i][j];
  
  double w = sqrt(oP[2][0] * oP[2][0] + oP[2][1] * oP[2][1] + oP[2][2] * oP[2][2]);
  
  if (m_ProjectiveParameters.IntrinsicMatrix[0][0]<0.0)
    sdet = -1;
    
  for(int i=0;i<3;i++)
    for(int j=0;j<4;j++)
	oP[i][j] = sdet*oP[i][j]/w;

}

HRP2Camera::ProjectiveParameters Camera_impl::GetProjectiveParameters()
  throw(CORBA::SystemException)
{
  return m_ProjectiveParameters;
}
