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

/*! Include from llvs */
#include <dc1394/IEEE1394DCCameraParameters.h>

#include <vector>
using namespace std;

//FIXME: Export this definition in a proper place
#define LLVS_CAMERA_NUMBER_MAX 	255

namespace llvs
{
  
  /*! \brief Profile of camera parameters for a vision system.*/
  class VisionSystemProfile
  {
  public:
    /*! \brief Visual System Profile name */
    string m_Name;

    /*! \brief Location of the file described the system. */
    string m_FileNameDescription;

    /*! \brief The vector of camera parameters associated
      with this description. */
    vector<IEEE1394DCCameraParameters *> m_CameraParameters;

    /*! Constructor */
    VisionSystemProfile();

    /*! Destructor */
    ~VisionSystemProfile();
  };

  /*! \brief This class is in charge of handling IEEE1394 cameras. */
  class HRP2IEEE1394DCImagesInputMethod : public HRP2ImagesInputMethod, public HRP2VisionBasicProcess
    {
    public:
			
			/* Camera semantic definition */
      static const int CAMERA_LEFT = 0;
      static const int CAMERA_RIGHT = 1;
      static const int CAMERA_UP = 2;
      static const int CAMERA_WIDE = 3;
      
      static const int YUV422_TO_RGB = 0;
      static const int BAYER_TO_RGB = 1;

			/* Limit parameters */
			static const unsigned int UNDEFINED_CAMERA_NUMBER = LLVS_CAMERA_NUMBER_MAX + 1;

			/* Global error flags */
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


      /*! Constructor */
      HRP2IEEE1394DCImagesInputMethod(void) throw(const char*);
  
      /*! Destructor */
      virtual ~HRP2IEEE1394DCImagesInputMethod();
      /*! Takes a new image.
       * Input :
       * \param unsigned char * Image:  A pointer where to store the image.
       * \param int camera: The camera index.
       */
      virtual int GetSingleImage(unsigned char **Image, unsigned int SemanticCamera,struct timeval &timestamp);


      unsigned int GetImageSinglePGM(unsigned char **Image, const unsigned int& cameraNumber, struct timeval &timestamp);
      unsigned int GetImageSingleRaw(unsigned char **Image, const unsigned int& cameraNumber, struct timeval &timestamp);
      unsigned int GetImageSingleRGB(unsigned char **Image, const unsigned int& cameraNumber, struct timeval &timestamp);

      /* Real implementation for single PGM */
      int GetImagePGM(unsigned char *Image, int SemanticCamera);
  
      /*! \brief Get the current format of the image 
	according to the camera index. 
	@param[in] CameraNumber: camera to which the format applies.
      */
      virtual string GetFormat(unsigned int SemanticCameraNumber);

      /*! \brief Set the format of the current image: default PGM 
	@param[in] aFormat: Name of the format to use.
	@param[in] CameraNumber: Camera which should switch to format aFormat.
      */
      int SetFormat(string aFormat, unsigned int SemanticCameraNumber);

      /*! Get the current image size for the appropriate camera 
       */
      virtual int GetImageSize(int &lw, int &lh, unsigned int SemanticCameraNumber);

      /*! Set the size of the image willing to be grabbed. 
	\param CameraNumber specifies the Semantic camera number.
       */
      virtual int SetImageSize(int lw, int lh, unsigned int CameraNumber);


      /*! Initialize the cameras */
      void InitializeCameras();

      /*! Initialize a camera */
      void InitializeCamera(IEEE1394DCCameraParameters &CamParams);

      /*! Decide to which the features should be set according
	to the vendor and the name of the product. */
      void DecideBasicFeatureOnCamera(dc1394camera_t &aCamera,
				      dc1394video_mode_t &res,
				      dc1394framerate_t &fps,
				      unsigned int InternalCameraNumber);

      /*! Initialize the board */
      void InitializeBoard() throw(const char*);
  
      /*! Stop the the board */
      void StopBoard();
  
      /*! Set parameter value */
      virtual int SetParameter(string aParameter, string aValue);
  
      /*! Override Start Process */
      virtual int StartProcess() throw(const char*);
  
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
          @return: True if initialization was successful.
          False otherwise.
       */
      virtual bool Initialize();

      /*! \brief Cleanup the grabbing system. 
       */
      virtual void Cleanup();

      /*! @} */

      void StartContinuousShot();
      void StopContinuousShot();
  
      /*! Returns the next time when the camera CameraNumber
	will  grab. */
      virtual double NextTimeForGrabbing(int CameraNumber);
  
      /*! From FrameRate to Time */
      void FromFrameRateToTime(int CameraNumber);

      /*! Provide semantic */
      int GetSemanticOfCamera(int lCameraIndexOnComputer);

      /*! Initialize the process. */
      int pInitializeTheProcess(){return 0;};

      /*! Realize the process */
      int pRealizeTheProcess(){return 0;};
  
      /*! Cleanup the process */
      int pCleanUpTheProcess(){return 0;};


    protected:

			/*! Put in <CameraNumber> the physical camera number 
			 * linked to <SemanticCamera>. Default return value is RETURN_OK.
			 * If an error occured and the physical camera number could not
			 * be deducted, then <CameraNumber> is set to UNDEFINED_CAMERA_NUMBER
			 * and return value can be one of the following (please refer to
			 * interface header for detailed information about these errors):
			 *
			 *   - ERROR_UNDEFINED_SEMANTIC_CAMERA
			 *   - ERROR_NO_CAMERA_ASSIGNED
			 *   - ERROR_CAMERA_MISSING
			 */ 
			unsigned int GetCameraNumber(const unsigned int& SemanticCamera,
					                         unsigned int& CameraNumber) const;

      /*! Clean memory when stopping the board. */
      void CleanMemory();

      /*! Number of cameras */
      unsigned int m_numCameras;

      /*! Map from semantic camera to real ones */
      vector<int> m_MapFromSemanticToRealCamera;

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

      /*! \brief Mode to convert from raw image to RGB */
      unsigned int m_ModeRaw2RGB;

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

      /*! \name Methods related to vision system profiles. 
	@{
       */

      /*! \brief List of Vision System Profile. 
	At least one is asked for. 
       */
      vector<VisionSystemProfile *> m_VisionSystemProfiles;

      /*! \brief Index of the best vision profile for the current detected cameras set. */
      int m_CurrentVisionSystemProfileID;

      /*! \brief Detect the best vision system profile. 
	The algorithm is simple we count the number of cameras 
	describe in the vision system profile. The one with
	the highest number of camera present win.
	Return false if two vision profiles have the same score.
	Otherwise the default behavior is to keep all the cameras
	detected and try a default configuration.
       */
      bool DetectTheBestVisionSystemProfile();

      /*! \brief Read configuration files in the VVV format. */
      void ReadConfigurationFileVVVFormat(string aFileName, string ProfileName);

      /*! \brief Read configuration files in the VSP format. */
      void ReadConfigurationFileVSPFormat(string aFileName, string ProfileName);

      /*! @} */
    };
};  
#endif 
