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
#ifndef _HRP2_INPUT_METHOD_H_
#define _HRP2_INPUT_METHOD_H_


#include <string>
#include <sys/time.h>
#include <vector>

using namespace std;

namespace llvs
{
  /*! This class defines the abstract class for the input method
    related to the low level HRP2 vision system.
    This will be used to input either files, either grabbed images,
    or event simulated ones.
   
    Copyright CNRS,JRL/AIST, 2004,
    Olivier Stasse

    07/12/2004: Modification for the new camera.
    16/05/2004: Creation
  */
  class HRP2ImagesInputMethod
    {
    public:
			
			/* ------------------
			 * Global error flags    
			 * ----------------- */

			/* Operation went through without any errors */
			static const unsigned int RESULT_OK                       = 0;

			/* The given physical camera number is out of bounds */
			static const unsigned int ERROR_UNDEFINED_PHYSICAL_CAMERA = 1;

			/* The given semantic is known but there is no physical camera 
			 * currently associated to this semantic. You need to connect 
			 * more cameras or change the semantic of your current physical 
			 * cameras. */
			static const unsigned int ERROR_NO_CAMERA_ASSIGNED        = 2;

			/* Camera is well linked to a semantic, but physical camera
			 * is no more existing.*/
			static const unsigned int ERROR_CAMERA_MISSING            = 3;

			/* The given semantic is out of bounds (unknwon semantic) */
			static const unsigned int ERROR_UNDEFINED_SEMANTIC_CAMERA = 4;

			/* An error occured during the dc1394 snapshot request */
			static const unsigned int ERROR_SNAP_EXCEPTION            = 5;

			/* Cannot find out the camera format (RGB, RAW, etc.) */
			static const unsigned int ERROR_UNKNOWN_FORMAT            = 6;

			/* You may have called a method while ImagesInputMethod
			 * object is not well formed (still initializing or 
			 * already destroyed!) */
			static const unsigned int ERROR_IMAGE_INPUT_NOT_READY    = 7;
  
      /*! Constructor */
      HRP2ImagesInputMethod();
  
      /*! Destructor */
      virtual ~HRP2ImagesInputMethod();

      /*! Takes a new image.
       * Input: 
       * Image : A pointer where to store the image.
       * SemanticCamera : Reference to the image itself (semantic number).
			 * timestamp : The image time stamp
			 * Output:
			 * RESULT_OK if process went through without errors.
			 * Else it returns a non-null number corresponding to
			 * a specific error reason. Please see above list of
			 * handled errors.
       */
      virtual unsigned int GetSingleImage(unsigned char **Image, const unsigned int& SemanticCamera, struct timeval &timestamp) = 0;

      /*! Set the size of the image willing to be grabbed. */
      virtual unsigned int SetImageSize(int lw, int lh, const unsigned int& SemanticCamera);

      /*! Get the current image size for the appropriate camera */
      virtual unsigned int GetImageSize(int &lw, int &lh, const unsigned int& SemanticCamera) const;

      /*! \brief Get the current format of the image.
	@param[in] CameraNumber: The camera for which the format is asked. 
      */
      virtual string GetFormat(const unsigned int& SemanticCamera) const;

      /*! Set the level of verbosity */
      int SetLevelOfVerbosity(const int& VerbosityParameter);

      /*! Get the level of verbosity */
      int GetLevelOfVerbosity() const;

      /*! \brief Get the number of camera */
      virtual unsigned int GetNumberOfCameras() const;

      /*! \brief Get the next time for grabbing an image. */
      virtual double NextTimeForGrabbing(const unsigned int& CameraNumber);

      /*! \brief Returns true if the initialization phase was correct and there is one camera connected
	(abstract method) */
      virtual bool CameraPresent() const = 0;

      /*! \brief Initialize the grabbing system. 
          @return: True if initialization was successful.
          False otherwise.
       */
      virtual bool Initialize()=0;

      /*! \brief Cleanup the grabbing system. 
       */
      virtual void Cleanup()=0;

      /*! \brief Get semantic image.
	More precisely, this link any camera to a specific
	data flow which is related to a specific image.
	Currently by convention we assume a 4 cameras vision 
	base system. For purposes of debugging, it is possible
	to not have the all system on your desktop.
	You can therefore relate a camera to a specific meaning:
	0 is the left image.
	1 is the right image.
	2 is the cyclope image
	3 is the wide image.
	In order to specify the mapping please see the Visual System Profile
	format.
	\param CameraNumberOnWS : The index of the camera detected on your computer.
	\return int : The index of the image in the semantic described below.
       */
      virtual int GetSemanticOfCamera(const unsigned int& CameraNumberOnWS)=0;

    protected:

      /*! Members of the class storing the size of the images. 
	All the four images have the same size. 
      */
      std::vector<unsigned int> m_ImagesWidth, m_ImagesHeight;

      /*! Depth of the images. */
      std::vector<unsigned int> m_depth;

      /*! Level of verbosity */
      int m_Verbosity;
    };
};  
#endif /* _HRP2_INPUT_METHOD_H_ */
