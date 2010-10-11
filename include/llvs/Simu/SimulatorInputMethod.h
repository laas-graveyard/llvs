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
#ifndef _HRP2_INPUT_IMAGES_SIMULATOR_METHOD_H_
#define _HRP2_INPUT_IMAGES_SIMULATOR_METHOD_H_

#ifdef OMNIORB4
#include <omniORB4/CORBA.h>
#endif

#ifdef __ORBIX__
#include <OBE/CORBA.h>
#include <OBE/CosNaming.h>
#endif


/* This object defines the abstract class for the input method
   related to the low level HRP2 vision system.
   This will be used to input either files, either grabbed images,
   or event simulated ones.

   Copyright CNRS,JRL/AIST, 2004,
   Olivier Stasse

   11/06/2004: Creation
*/

#include <iostream>
#include <string>
#include <assert.h>
using namespace std;


#include "ImagesInputMethod.h"
#ifdef __ORBIX__
#include "common.h"
#include "visionsensor.h"
#endif

#ifdef OMNIORB4
#include "common.hh"
#include "ViewSimulator.hh"
#endif


/*! \brief This object implements the grabbing of the images from the simulator.
 *
 */
namespace llvs
{
  /*! \class This class defines how to simulate the input send to the Low Level Vision Server.
   */
  class HRP2SimulatorInputMethod : public HRP2ImagesInputMethod
    {
    public:
      /*! Constructor */
      HRP2SimulatorInputMethod(int argc, char *argv[],CORBA::ORB_var ns);

      /*! Destructor */
      virtual ~HRP2SimulatorInputMethod();

      /*! \brief Takes a new image.
       * @param[in] unsigned char * ImageLeft : A pointer where to store the bottom left image.
       * @param[in] unsigned char * ImageRight : A pointer where to store the bottom right image.
       * @param[in] unsigned char * ImageUp : A pointer where to store the upper image.
       */
      virtual int GetImage(unsigned char **ImageLeft, unsigned char **ImageRight, unsigned char **ImageUp);

      virtual unsigned int GetSingleImage(unsigned char **Image, const unsigned int& SemanticCamera, double &timestamp);

      /*! \brief Set the size of the image willing to be grabbed. */
      virtual unsigned int SetImageSize(int lw, int lh, const unsigned int& CameraNumber);

      /*! \brief Get the current image size for the appropriate camera */
      virtual unsigned int GetImageSize(int &lw, int &lh, const unsigned int& SemanticCameraNumber) const;

      /*! \brief Get the current format of the image */
      virtual string GetFormat(const unsigned int& SemanticCamera) const;

      /*! \brief Tells if at least one camera is present. */
      virtual bool CameraPresent() const;

      /*! \brief Initialize the grabbing system.
	@return: True if initialization was successful.
	False otherwise.
       */
      virtual bool Initialize();

      /*! \brief Cleanup the grabbing system.
       */
      virtual void Cleanup();

      /*! \brief Return the number of cameras being simulated. */
      virtual unsigned int GetNumberOfCameras() const;

      /*! \brief Return the link between the detected camera
       and its semantic. */
      virtual int GetSemanticOfCamera(const unsigned int& CameraNumberOnWS);

    protected:
      OpenHRP::ViewSimulator_var m_vsensor;
      OpenHRP::CameraSequence_var m_cameras;
      std::vector<unsigned> m_subsamplingX;
      std::vector<unsigned> m_subsamplingY;
    };

};
#endif /* _HRP2_INPUT_SIMULATOR_METHOD_H_ */
