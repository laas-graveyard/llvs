/** @doc This object implements a visual process
    to get IEEE camera images using the DC libraries.
    

   Copyright (c) 2003-2006, 
   @author Olivier Stasse, 
   
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
#ifndef _HRP2_IEEE1394_DC_INPUT_METHOD_H_
#define _HRP2_IEEE1394_DC_INPUT_METHOD_H_


#include <ImagesInputMethod.h>
#include <VisionBasicProcess.h>
#include <string>

#include <pthread.h>

/*! Includes for 1394 communications. */
#include <libraw1394/raw1394.h>
#include <dc1394/dc1394.h>

#include <vector>
using namespace std;

namespace llvs
{
  class HRP2IEEE1394DCImagesInputMethod : public HRP2ImagesInputMethod, public HRP2VisionBasicProcess
    {
    public:

      static const int CAMERA_LEFT = 0;
      static const int CAMERA_RIGHT = 1;
      static const int CAMERA_UP = 2;
      static const int CAMERA_WIDE = 3;
    
      /*! Constructor */
      HRP2IEEE1394DCImagesInputMethod(void);
  
      /*! Destructor */
      virtual ~HRP2IEEE1394DCImagesInputMethod();
      /*! Takes a new image.
       * Input :
       * \param unsigned char * Image:  A pointer where to store the image.
       * \param int camera: The camera index.
       */
      virtual int GetSingleImage(unsigned char **Image, int camera,struct timeval &timestamp);


      int GetImageSinglePGM(unsigned char **Image, int camera, struct timeval &timestamp);
      int GetImageSingleRaw(unsigned char **Image, int camera, struct timeval &timestamp);
      int GetImageSingleRGB(unsigned char **Image, int camera, struct timeval &timestamp);

      /* Real implementation for single PGM */
      int GetImagePGM(unsigned char *Image, int camera);
  
      /*! \brief Get the current format of the image 
	according to the camera index. 
	@param[in] CameraNumber: camera to which the format applies.
      */
      virtual string GetFormat(unsigned int CameraNumber);

      /*! \brief Set the format of the current image: default PGM 
	@param[in] aFormat: Name of the format to use.
	@param[in] CameraNumber: Camera which should switch to format aFormat.
      */
      int SetFormat(string aFormat, unsigned int CameraNumber);

      /*! Get the current image size for the appropriate camera */
      virtual int GetImageSize(int &lw, int &lh, int CameraNumber);

      /*! Set the size of the image willing to be grabbed. */
      virtual int SetImageSize(int lw, int lh, int CameraNumber);


      /*! Initialize the cameras */
      void InitializeCameras();

      /*! Initialize the board */
      void InitializeBoard();
  
      /*! Stop the the board */
      void StopBoard();
  
      /*! Set parameter value */
      virtual int SetParameter(string aParameter, string aValue);
  
      /*! Override Start Process */
      virtual int StartProcess();
  
      /*! Override Stop Process */
      virtual int StopProcess();

      void GetCameraFeatureValue(string aCamera, string aFeature, string &aValue);
      void SetCameraFeatureValue(string aCamera, string aFeature, string aValue);

      /*! \name Reimplement the ImagesInputMethod abstract interface 
	@{
       */

      /*! \brief Returns the number of cameras 
	Here the number of IEEE 1394 cameras detected.
       */
      virtual unsigned int GetNumberOfCameras();
      
      /*! \brief Returns true if one camera is detected. */
      bool CameraPresent();
      
      /*! \brief Initialize the grabbing system. 
	@return: a negative value in case of an error,
	0 otherwise.
       */
      virtual int Initialize();

      /*! \brief Cleanup the grabbing system. 
	@return: a negative value in case of an error,
	0 otherwise.
       */
      virtual int Cleanup();

      
      /*! @} */

      void StartContinuousShot();
      void StopContinuousShot();
  
      /*! Returns the next time when the camera CameraNumber
	will  grab. */
      virtual double NextTimeForGrabbing(int CameraNumber);
  
      /*! From FrameRate to Time */
      void FromFrameRateToTime(int CameraNumber);

    protected:

      /*! Number of cameras */
      unsigned int m_numCameras;
  
      /*! Pointer to the copy memory. */
      vector<unsigned char *> m_TmpImage;

      /*! Format */
      vector<string> m_Format;

      /*! Prefixes for cameras */
      vector<string> m_Prefixes;
  
      /*! Prefixes for features */
      vector<string> m_Features;

      /*! Keep time for each camera. */
      vector <double> m_LastGrabbingTime;
  
      /*! Keep the period for each grabbing. */
      vector<double> m_GrabbingPeriod;
  
      /*! \brief Mutex to protect the device. */
      pthread_mutex_t m_mutex_device;

      /*! \name Fields specific to 1394 access. 
	@{
      */

      /*! \brief Handle on the 1394 device */
      dc1394_t * m_HandleDC1394;
  
      /*! \brief Cameras Ids */
      vector<dc1394camera_t *> m_DC1394Cameras;

      /*! \brief Ref towards 1394 data structure */
      vector<dc1394video_frame_t *> m_VideoFrames;

      /*! \brief Local Images size the width and the height for each image.*/
      vector<int> m_BoardImagesWidth, m_BoardImagesHeight;

      /*! \brief This field tells us if one camera has been detected or not. */
      bool m_AtLeastOneCameraPresent;

      /*! @} */
    };
};  
#endif 
