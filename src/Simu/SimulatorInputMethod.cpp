/** @doc This object implements the link between
    the Integrated Simulation Environnment with the
    overall vision system.

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

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include "Simu/SimulatorInputMethod.h"
#include <llvs/tools/Debug.h>


using namespace llvs;

HRP2SimulatorInputMethod::HRP2SimulatorInputMethod(int argc, char *argv[], CORBA::ORB_var ns)
  : HRP2ImagesInputMethod(),
    m_vsensor(),
    m_cameras(),
    m_subsamplingX(),
    m_subsamplingY()
{
  CORBA::Object_var obj;

  if (CORBA::is_nil(ns))
    throw "Invalid Corba nameserver";

  obj = ns->resolve_initial_references("NameService");

  CosNaming::NamingContext_var cxt;
  cxt = CosNaming::NamingContext::_narrow(obj);

  // Prepare to get context from the Name Service.
  CosNaming::Name ncFactory;

  ncFactory.length(1);
  ncFactory[0].id = CORBA::string_dup("ViewSimulator");
  //ncFactory[0].kind = CORBA::string_dup("");

  CORBA::Object_var viewSimulatorObj = cxt->resolve(ncFactory);
  if (CORBA::is_nil(viewSimulatorObj.in()))
    throw "Failed to resolve ViewSimulator object";

  m_vsensor = OpenHRP::ViewSimulator::_narrow(cxt->resolve(ncFactory));
  if (CORBA::is_nil(m_vsensor.in()))
    throw "Failed to narrow ViewSimulator object";

  // Fill cameras attribute.
  m_vsensor->getCameraSequence(m_cameras);

  if (!m_cameras->length())
    throw "invalid cameraSequence object";

  // Fill width/height tables.
  // FIXME: this makes the assumption that cameras cannot be added/removed at runtime.
  unsigned nCameras = m_cameras->length ();

  ODEBUG(nCameras << " camera(s) have been detected");

  m_ImagesWidth.resize (nCameras);
  m_ImagesHeight.resize (nCameras);
  m_depth.resize (nCameras);

  m_subsamplingX.resize (nCameras, 1);
  m_subsamplingY.resize (nCameras, 1);

  for (unsigned i = 0; i < nCameras; ++i)
    {
      OpenHRP::Camera_var camera = m_cameras[i];
      OpenHRP::Camera::CameraParameter* cameraParameter =
	camera->getCameraParameter();
      assert (cameraParameter);

      m_ImagesWidth[i] = cameraParameter->width;
      m_ImagesHeight[i] = cameraParameter->height;
      m_depth[i] = 0;

      switch (cameraParameter->type)
	{
	case OpenHRP::Camera::COLOR:
	  m_depth[i] = 3;
	  break;

	case OpenHRP::Camera::MONO:
	  m_depth[i] = 1;
	  break;

	case OpenHRP::Camera::NONE:
	case OpenHRP::Camera::DEPTH:
	case OpenHRP::Camera::COLOR_DEPTH:
	case OpenHRP::Camera::MONO_DEPTH:
	  throw "OpenHRP image kind is not supported";
	  break;

	default:
	  assert (0 && "Should never happen.");
	}
    }
}

HRP2SimulatorInputMethod::~HRP2SimulatorInputMethod()
{

}

int HRP2SimulatorInputMethod::GetImage(unsigned char ** ImageLeft, unsigned char ** ImageRight, unsigned char ** ImageUp)
{
  unsigned cameraId = 0;
  double timestamp = 0.;
  int res = 0;

  res = GetSingleImage(ImageLeft, 0, timestamp);
  if (res < 0)
    return res;

  res = GetSingleImage(ImageRight, 0, timestamp);
  if (res < 0)
    return res;

  res = GetSingleImage(ImageUp, 0, timestamp);
  if (res < 0)
    return res;

  return res;
}

unsigned int
HRP2SimulatorInputMethod::GetSingleImage(unsigned char **Image, const unsigned int& SemanticCamera, double &timestamp)
{
  CORBA::Long cameraNumber = SemanticCamera; //FIXME: map correctly indexes.

  ODEBUG("cameraNumber =" << cameraNumber);

  CORBA::Long nCameras = m_cameras->length ();
  if (cameraNumber >= nCameras)
    return ERROR_UNDEFINED_SEMANTIC_CAMERA;

  OpenHRP::Camera_var camera = m_cameras[cameraNumber];
  OpenHRP::ImageData_var imageData = camera->getImageData();

  // Image width and height information is available through the camera and through
  // the image, here make sure that the two information are consistent.
  if (imageData->width - (m_ImagesWidth[cameraNumber]*m_subsamplingX[cameraNumber]) != 0)
    throw "image width != camera image width is not supported";
  if (imageData->height - (m_ImagesHeight[cameraNumber]*m_subsamplingY[cameraNumber]) != 0)
    throw "image height != camera image height is not supported";

  unsigned size =
    m_ImagesWidth[cameraNumber] * m_ImagesHeight[cameraNumber] * m_depth[cameraNumber];

  ODEBUG("Total size: " << size << " " <<  imageData->octetData.length());

  union
  {
    unsigned long al;
    unsigned char ac[4];
  } FromL2C;

  //#define LLVS_SERVER_SIDE_IMAGE_RECORDING
#ifdef LLVS_SERVER_SIDE_IMAGE_RECORDING
  if (1)
    {
      static unsigned lcounter[4] = { 0,0,0,0};
      FILE *fp;
      char Buffer[1024];
      bzero(Buffer,1024);
      sprintf(Buffer,"SIMcheck_%01d_%06d.ppm",
	      SemanticCamera,(int)lcounter[SemanticCamera]);
      fp = fopen(Buffer,"w");

      fprintf(fp,"P6\n");
      fprintf(fp,"%d %d\n255\n",(int)m_ImagesWidth[SemanticCamera],(int)m_ImagesHeight[SemanticCamera]);
      unsigned int lidx = 0;
      ODEBUG3("Size from View simulator:"
	      << "OD:" << imageData->octetData.length() << endl
	      << "LD:" << imageData->longData.length() << endl
	      << "FD:" << imageData->floatData.length() << endl
	      << "Size:" << imageData->width << "x" << imageData->height );

      switch(imageData->format)
	{
	case OpenHRP::ARGB:
	  ODEBUG3("Format:ARGB");
	  break;
	case OpenHRP::GRAY:
	  ODEBUG3("Format:GRAY" );
	  break;
	case OpenHRP::DEPTH:
	  ODEBUG3("Format:DEPTH" );
	  break;
	case OpenHRP::RGB:
	  ODEBUG3("Format:RGB" );
	  break;
	default:
	  ODEBUG3("Format:unknown");
	  break;
	}

      for(unsigned int m=0;m<imageData->longData.length();m++)
	{
	  FromL2C.al = imageData->longData[m];
	  fprintf(fp,"%c%c%c",FromL2C.ac[2],FromL2C.ac[1],FromL2C.ac[0]);
	}
      lcounter[SemanticCamera]++;
    }
#endif // LLVS_SERVER_SIDE_IMAGE_RECORDING

      switch(imageData->format)
	{
	case OpenHRP::ARGB:
	  {
	    unsigned idx = 0, lidx = 0;
	    while (lidx < imageData->longData.length())
	      {
		FromL2C.al = imageData->longData[lidx];
		(*Image)[idx++] = FromL2C.ac[2];
		(*Image)[idx++] = FromL2C.ac[1];
		(*Image)[idx++] = FromL2C.ac[0];

		lidx += m_subsamplingX[cameraNumber];
		if (lidx % imageData->width <= 1)
		  lidx += (m_subsamplingY[cameraNumber] - 1) * imageData->width;
	      }
	  }
	  break;

	case OpenHRP::GRAY:
	case OpenHRP::DEPTH:
	case OpenHRP::RGB:
	  throw "image kind is not supported";
	  break;

	default:
	  assert (0 && "should never happen");
	  break;
	}


  return RESULT_OK;
}

//FIXME: Use semantic camera designation instead of physical camera id
unsigned int
HRP2SimulatorInputMethod::SetImageSize(int lw, int lh, const unsigned int& CameraNumber)
{
  if (CameraNumber >= m_ImagesWidth.size())
    return ERROR_UNDEFINED_PHYSICAL_CAMERA;

  if (m_ImagesWidth[CameraNumber] - lw != 0
      || m_ImagesHeight[CameraNumber] - lh != 0)
    {
      m_subsamplingX[CameraNumber] = m_ImagesWidth[CameraNumber] / lw;
      m_subsamplingY[CameraNumber] = m_ImagesHeight[CameraNumber] / lh;

      if (!m_subsamplingX[CameraNumber] || !m_subsamplingY[CameraNumber])
	throw "image size increment is not supported";
    }

  m_ImagesWidth[CameraNumber] = lw;
  m_ImagesHeight[CameraNumber] = lh;

  return RESULT_OK;
}

//FIXME: Use semantic camera designation instead of physical camera id
unsigned int
HRP2SimulatorInputMethod::GetImageSize(int &lw, int &lh, const unsigned int& CameraNumber)
const
{
  if (CameraNumber >= m_ImagesWidth.size())
    return ERROR_UNDEFINED_PHYSICAL_CAMERA;

  lw = m_ImagesWidth[CameraNumber];
  lh = m_ImagesHeight[CameraNumber];
  return RESULT_OK;
}

string
HRP2SimulatorInputMethod::GetFormat(const unsigned int&)
const
{
  string aFormat("RGB");
  return aFormat;
}

bool
HRP2SimulatorInputMethod::CameraPresent()
const
{
  return true;
}
/*! \brief Initialize the grabbing system.
*/
bool HRP2SimulatorInputMethod::Initialize()
{
  return true;
}

/*! \brief Cleanup the grabbing system.
  @return: a negative value in case of an error,
  0 otherwise.
*/
void HRP2SimulatorInputMethod::Cleanup()
{
}


unsigned int HRP2SimulatorInputMethod::GetNumberOfCameras() const
{
  return m_cameras->length ();
}


int HRP2SimulatorInputMethod::GetSemanticOfCamera(const unsigned int& CameraNumberOnWS)
{
  return CameraNumberOnWS;
}
