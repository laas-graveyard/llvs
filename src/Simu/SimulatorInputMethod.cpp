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
#include <stdio.h>
#include <stdlib.h>
#include "Simu/SimulatorInputMethod.h"

#ifdef __ORBIX__
#include "common.h"
#include "visionsensor.h"
#endif

#ifdef OMNIORB4
#include "common.hh"
#include "visionsensor.hh"
#endif

using namespace llvs;

HRP2SimulatorInputMethod::HRP2SimulatorInputMethod(int argc, char *argv[], CORBA::ORB_var ns) 
  : HRP2ImagesInputMethod()
{
#if 0
  CORBA::Object_var obj;

  if (CORBA::is_nil(ns))
    fprintf(stderr,"HRP2SimulatorInputMethod::HRP2SimulatorInputMethod ns is nil\n");

  obj = ns->resolve_initial_references("NameService");

  fprintf(stderr,"HRP2SimulatorInputMethod::HRP2SimulatorInputMethod Step 1\n");

  CosNaming::NamingContext_var
    cxt;
  
  fprintf(stderr,"HRP2SimulatorInputMethod::HRP2SimulatorInputMethod Step 2\n");
  cxt = CosNaming::NamingContext::_narrow(obj);


  
  fprintf(stderr,"HRP2SimulatorInputMethod::HRP2SimulatorInputMethod Step 3\n");
  // Prepare to get context from the Name Service.
#ifdef __OMNIORB4__
#define CORBA_string_dup CORBA::string_dup
#endif

  CosNaming::Name 
    ncFactory;

  ncFactory.length(1);
  ncFactory[0].id = CORBA_string_dup("VisionSensorFactory");
  ncFactory[0].kind = CORBA_string_dup("");
  VisionSensorFactory_var vfactory = 
    VisionSensorFactory::_narrow(cxt->resolve(ncFactory));
  m_vsensor = vfactory -> createVisionSensor(); 

  fprintf(stderr,"HRP2SimulatorInputMethod::HRP2SimulatorInputMethod Step 4\n");
  ncFactory[0].id = CORBA_string_dup("IntegratorFactory");
  ncFactory[0].kind = CORBA_string_dup("");
  IntegratorFactory_var ifactory = IntegratorFactory::_narrow(cxt -> resolve(ncFactory));
  m_integrator = ifactory -> createIntegrator(); 
  m_vsensor->getCameraSeq(m_cameras);
#endif
  m_ColorMode = BW;

}

HRP2SimulatorInputMethod::~HRP2SimulatorInputMethod()
{
}

int HRP2SimulatorInputMethod::GetImage(unsigned char ** ImageLeft, unsigned char ** ImageRight, unsigned char ** ImageUp)
{

  return -1;
}

int HRP2SimulatorInputMethod::SetImageSize(int lw, int lh, int CameraNumber)
{
  if ((CameraNumber<0) || (CameraNumber>2))
    return -1;

  m_ImagesWidth[CameraNumber] = lw;
  m_ImagesHeight[CameraNumber] = lh;
  return 0;
}

int HRP2SimulatorInputMethod::GetImageSize(int &lw, int &lh, int CameraNumber)
{
  if ((CameraNumber<0) || (CameraNumber>2))
    return -1;

  lw = m_ImagesWidth[CameraNumber];
  lh = m_ImagesHeight[CameraNumber];
  return 0;
}

string HRP2SimulatorInputMethod::GetFormat()
{
  string aFormat("Simulator based information");
  return aFormat;
}

bool HRP2SimulatorInputMethod::CameraPresent()
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
