/** @doc This object implements the CORBA server providing
    Low Level Vision on the HRP-2 Vision processor.


   Copyright (c) 2003-2008, 
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
#ifndef _HRP2_VISION_SERVER_
#define _HRP2_VISION_SERVER_

#include <sys/time.h>

#ifdef __ORBIX__
#include <OBE/CORBA.h>
#include "LowLevelVisionSystem_skel.h"
#include "LowLevelVisionSystem.h"
#endif

#ifdef OMNIORB4
#include <omniORB4/CORBA.h>
#include "LowLevelVisionSystem.hh"
#endif


#include <llvsConfig.h>

#include <Corba/Camera_impl.h>
#include <Corba/StereoVision_impl.h>
#include <Corba/ModelTrackerInterface_impl.h>
#include <Corba/PointTrackerInterface_impl.h>
#include <VisionBasicProcess.h>
#include <ConnectionToSot.h>

/*! Inclusion specific to VVV */
#if (LLVS_HAVE_VVV>0)

  extern "C"
  {
#include <epbm.h>
#include <vfgb.h>
#include <vfgbutil.h>
#include <vfgbtype.h>
  }
#include "VVV/RectificationProcess.h"
#include "VVV/EdgeDetectionProcess.h"
#include "VVV/BRepDetectionProcess.h"
#include "VVV/DisparityProcess.h"
#include "VVV/FindFeaturesInImage.h"
#include "VVV/ColorDetection.h"
#include "VVV/MireDetectionProcess.h"
#include "VVV/ImageDifference.h"
#endif

#if ((LLVS_HAVE_VW>0) &  (LLVS_HAVE_SCENE>0))
#include "SingleCameraSLAMProcess.h"
#endif

#include "ImagesInputMethod.h"

#if (LLVS_HAVE_VISP>0)
/* From the visp library */
#include "visp/vpXmlParserCamera.h"
#include "visp/vpHomogeneousMatrix.h"

/* Internal framework */
#include "ViSP/vispConvertImageProcess.h"
#include "ViSP/vispUndistordedProcess.h"

#include "PointTracker/PointTrackingProcess.h"
#include "PointTracker/CircularBufferPointTrackerData.h"

#endif


#if (LLVS_HAVE_NMBT>0)
#include "ModelTracker/nmbtTrackingProcess.h"
#include "ViSP/CircularBufferTrackerData.h"
#include "ViSP/ComputeControlLawProcess.h"
#endif

#if (LLVS_HAVE_BTL_SLAM>0)
#include "BtlSlam/BtlSlamProcess.h"
#endif

#include <vector>

using namespace std;

namespace llvs
{

  class Camera_impl ;

  typedef struct 
  {
    unsigned int Width, Height;
  } ImageSize;

  /**! LowLevelVisionServer: Corba Server for Low Level Vision processing.
     This includes:
     Image acquistion, 
     Loading of the calibration parameters, 
     Image rectification,
     Optical Flow, and Disparity.
     The optical flow are the Lucas and Kanade method modified by Simoncelli.
     The disparity is computed using the camera calibration parameters, and
     a correlation function based on the absolute value of the difference.
     Self Localization and Map building.
  */
  class LowLevelVisionServer: public virtual POA_LowLevelVisionSystem,
    public virtual PortableServer::RefCountServantBase
    {
    public:


      /*! \brief Constructor
       * \param MethodForInputImages: Specify how to get the images.
       * FRAMEGRABBER: takes the images from the available video board.
       * FILES: takes the images from a directory, specified by filename.
       * FILE: take just one image specified by filename.
       * SIMULATION: take the images from simulation.
       * \param filename: filename or directory depending on the
       * choosen value for MethodForInputImages.
       * 
       */
      LowLevelVisionServer(LowLevelVisionSystem::InputMode MethodForInputImages, 
			   LowLevelVisionSystem::SynchroMode SynchroMethodForInputImages, 
			   string filename, 
			   CORBA::ORB_var orb, 
			   int Verbosity,
			   string lCalibdir) throw(const char*);

      /* Destructor */
      virtual ~LowLevelVisionServer();

      /*! Returns version of the vision system */
      CORBA::Long GetVersion() throw(CORBA::SystemException) ;

      /*! Start the low level data-flow process */
      CORBA::Long StartMainProcess() throw(CORBA::SystemException) ; 

      /*! Stop the low level data-flow process */
      CORBA::Long StopMainProcess() throw(CORBA::SystemException);

      /*! Start a low level data-flow process */
      CORBA::Long StartProcess(const char *aProcessName)
	throw(CORBA::SystemException);

      /*! Stop a low level data-flow process */
      CORBA::Long StopProcess(const char *aProcessName)
	throw(CORBA::SystemException);
 

      /*! Load Calibration information */
      CORBA::Long LoadCalibrationInformation();

      /*! Load image size used during calibration */
      CORBA::Long CalibLoadSize(const char *file, long int *width, long int *height);

      /*! Set the grabbed image size 
       * Because of some algorithms used, right now this size is used for ALL the cameras
       * of the system.
       */
      CORBA::Long SetImagesGrabbedSize(CORBA::Long SemanticCameraID,
				       CORBA::Long lw, CORBA::Long lh)
	throw(CORBA::SystemException);

      /*! Get the image from the frame grabber. */
      CORBA::Long GetImageFromFrameGrabber();

      /*! Free the binary images */
      CORBA::Long FreeBinaryImages();

      /*! Applies the flow of operations */
      CORBA::Long ApplyingProcess();
 
      /*! Interface: Trigger the grabbing of an image */
      CORBA::Long TriggerSynchro() 
	throw(CORBA::SystemException);

      /*! Dumping to epbm */
      void SetDumpImageMode(LowLevelVisionSystem::DumpImageMode aDumpImageMode,
			    const FloatBuffer &Informations,
			    CORBA::Long SizeOfInformations)
	throw(CORBA::SystemException);
 
      /*! Get the dumping image mode */
      LowLevelVisionSystem::DumpImageMode  GetDumpImageMode(void)
	throw(CORBA::SystemException);

      /*! Constantes */
      static const CORBA::Long CAMERA_LEFT  = 0;
      static const CORBA::Long CAMERA_RIGHT = 1;
      static const CORBA::Long CAMERA_UP    = 2;
      static const CORBA::Long CAMERA_WIDE  = 3;
      /*! Interface : returns the synchronization mode */
      LowLevelVisionSystem::InputMode GetInputMode()
	throw(CORBA::SystemException) ;
 
      /*! Interface : returns the synchronization mode */
      LowLevelVisionSystem::SynchroMode SynchronizationMode()
	throw(CORBA::SystemException) ;
      
      /*! Interface : set the synchronization mode and allows to switch between
	still images and data flow. */
      void SetSynchronizationMode(LowLevelVisionSystem::SynchroMode aSynchronizationMode)
	throw(CORBA::SystemException);

      
      /*! Set the image */
      CORBA::Long SetImage(const ColorBuffer & cbuf, CORBA::Long CameraID, CORBA::Long aWidth, CORBA::Long aHeight)
	throw(CORBA::SystemException) ;

      /*! Set the verbose mode */
      CORBA::Long SetVerboseMode(CORBA::Long VerboseMode);

      /*! Get the verbose mode */
      CORBA::Long GetVerboseMode();

      /*! Bind the object to a Name */
      CORBA::Boolean bindObjectToName(CORBA::Object_ptr objref);

      /*! Set the calibration directory */
      CORBA::Long SetCalibrationDirectory(string aCalibrationDir);

      /*! Get the calibration directory */
      string GetCalibrationDirectory();

      /*! Get the images sizes */
      void GetImageSizes(vector<ImageSize> &Sizes);

      /*! Get the image size for one image */
      void GetImageSize(int Size[2], unsigned int CameraId);

      /*! Get the direct access to the image memory */
      void GetImageMemory(vector<unsigned char *> & BinaryImages);
 
      /*! Get the direct access for one image */
      unsigned char * GetImageMemory(unsigned int CameraID);

      /*! Interface: Get an image */
      virtual CORBA::Long getImage(CORBA::Long SemanticCamera, ImageData_out anImage, char *& Format)
	throw(CORBA::SystemException);

      /*! Interface: Get an edge image */
      virtual CORBA::Long getEdgeImage(CORBA::Long SemanticCamera, ImageData_out anImage, char *& Format)
	throw(CORBA::SystemException);

      /*! Interface: Get a rectified image */
      CORBA::Long getRectifiedImage(CORBA::Long SemanticCamera, ImageData_out anImage, char *& Format)
	throw(CORBA::SystemException);

      /*! Interface: Get range map */
      CORBA::Long getRangeMap(RangeMap_out RangeMap, char *&Format)
	throw(CORBA::SystemException);

      /*! Interface: Get image derivative 
       * \param CameraID: Identifier of the camera from which the derivative is extracted.
       * \param DerivativeID: To be of the identifier specified in the LowLevelVisionSystem IDL.
       * \param ImageDerivative: The FloatBuffer_var in which the data are provided. They are given
       *  in the usual image representation ( FirstRow, SecondRow,...,HeightRow).
       * \param Width: Width of the image derivative. Generally it is less than the initial image width,
       * and depends upon the mask used for the computation.
       * \param Height: Height of the image derivative. Generally it is less than the initial image height,
       * and depends upon the mask used for the computation.
       */
      CORBA::Long getImageDerivative(CORBA::Long CameraID,
				     CORBA::Long DerivativeID,
				     FloatBuffer_out ImageDerivative,
				     CORBA::Long_out Width,
				     CORBA::Long_out Height)
	throw(CORBA::SystemException);

      /*! Interface: Get the optical flow for a given camera */
      CORBA::Long getOpticalFlow(CORBA::Long CameraID,
				 FloatBuffer_out OpticalFlow,
				 FloatBuffer_out Confidence,
				 CORBA::Long_out Width,
				 CORBA::Long_out Height)
	throw(CORBA::SystemException);

      /*! Interface: Get the Harris detector */
      CORBA::Long getHarrisDetector(CORBA::Long CameraID,
				    FloatBuffer_out Harris,
				    CORBA::Long_out Width,
				    CORBA::Long_out Height)
	throw(CORBA::SystemException);

      /*! Interface: Set the acquisition size.
       * This will works only if the acquisition method is using the framegrabber.
       */
      CORBA::Long SetAcquisitionSize(CORBA::Long SemanticCameraID,
				     CORBA::Long aWidth, CORBA::Long aHeight)
	throw(CORBA::SystemException);

      /*! Destruction */  
      virtual void destroy() throw (CORBA::SystemException);

      /*! Returns the object related to the asked camera */
      Camera_impl * GetCamera(unsigned int CameraNb);

      /*! Interface: Get the camera sequence */
      CORBA::Long getCameraSeq( HRP2CameraSeq_out cameras) throw (CORBA::SystemException);

      /*! Interface: Get the process parameters and theirs values */
      CORBA::Long GetProcessParameters(const char * aProcessName,
				       HRP2LowLevelVisionParametersSeq_out ParametersSeq,
				       HRP2LowLevelVisionParametersSeq_out ParametersValueSeq)
	throw (CORBA::SystemException);

      /*! Interface : Get the process parameter and its value. */
      CORBA::Long GetProcessParameter(const char * aProcessName,
				      const char * aParamaterName,
				      CORBA::String_out ParameterValue)
	throw (CORBA::SystemException);

      /*! Interface: Returns the status of a low level data-flow process */
      CORBA::Long ProcessStatus(const char *aProcessName)
	throw (CORBA::SystemException);

      /*! Interface: Returns the matrix to project visual information in the head 
       * reference frame. 
       */
      CORBA::Long GetMatrixHeadTOrg(DoubleBuffer_out HeadTOrg)
	throw (CORBA::SystemException);

      /*! Returns the matrix Head frame - Original Vision frame 
       * @param HeadTOrg : pointer to an array of double.
       */
      void GetMatrixHeadTOrg(double *HeadTOrg);

      /*! Display the list of Vision Basic processes name in the
       * standard output error */
      void DisplayProcessesNames();

      /*! Set the parameter of a visual process given by its name */
      void SetAProcessParameterAndValue(string aProcessName, 
					string aParameter, 
					string aValue);

      /*! Set the parameter of a visual process given by its name */
      void SetAProcessParameterAndValue(const char * aProcessName, 
					const char * aParameter, 
					const char * aValue)
	throw (CORBA::SystemException);

      /*! If set to one dump the entry images 
       *  in /tmp/REALcheck*.pgm
       */
      void SetCheckEntry(unsigned char ADumpMode);

      /*! Return dump mode */
      unsigned char GetCheckEntry(void);
 
      /*! Get the Vision process sequence */
      CORBA::Long getLowLevelVisionProcesses(HRP2LowLevelVisionProcessesSeq_out ProcessesSeq)
	throw (CORBA::SystemException);

      /*! Check the image format asked by the user, and put the appropriate on inside Format */
      void CheckImageFormat(char *& Format);
 
      /*! Check the range map format asked by the user, and put the appropriate on inside Format */
      void CheckRangeMapFormat(char *& Format);

      /*! Read the robot calibration file. */
      void ReadRbtCalib();

      /*! Set the robot vision calibration directory */
      void SetRobotVisionCalibrationDirectory(string lRVCalibDir);
 
      /* ! Undistorted the image */
      void LensDistorsionCorrection();

      /* ! Provides a Boundary Representationm */
      CORBA::Long GetBoundaryRepresentation(CBREPSeq_out aBrep)
	throw (CORBA::SystemException);

      /* ! Create the memory stack for storing images */
      void  CreateStack();

      /* ! Destroy the stack storing images */
      void DeleteStack();

      /* ! Store images on stack */
      void StoreImageOnStack(int image);

      /* ! Record images from the stack on the disk */
      void RecordImagesOnDisk(int image);

      /* ! Interface: Get the scene object of the Single Camera Visual process */
      CORBA::Long GetSceneObject(SceneObject_out aSceneObject)
	throw (CORBA::SystemException);
 
      /* ! Interface: Delete a feature nside the Single Camera Visual process */
      void SceneDeleteFeature(CORBA::Long FeatureLabel)
	throw (CORBA::SystemException);

      /* ! Interface: Returns the current image identifier.*/
      virtual CORBA::Long GetImageIdentifier()
	throw (CORBA::SystemException);

      /* ! Interface: Returns the reference of the stereo vision object */
      StereoVision_ptr getStereoVision() 
	throw (CORBA::SystemException);

      /* ! Interface: Returns the reference of the Model Tracker object */
      ModelTrackerInterface_ptr getModelTracker() 
	throw (CORBA::SystemException);


      /* ! Interface: Returns the reference of the Point Tracker object */
      PointTrackerInterface_ptr getPointTracker() 
	throw (CORBA::SystemException);

      /* ! Get Object reference */
      CORBA::Object_ptr getObjectReference(string ServerID, string ServerKind);

      CORBA::Object_ptr getObjectReference(vector<string> & ServerID, vector<string> & ServerKind);

      /* ! Create Name context */
      void CreateNameContext(void);

      /* ! Set the SLAM image */
      void SetTheSLAMImage(int anIndex);

      /* ! Get the SLAM image */
      int GetTheSLAMImage();

#if (LLVS_HAVE_VVV>0)

      /*! \name Interface specific to VVV. 
	@{ */

      /*! Image rectification using a small modification */
      CORBA::Long scm_ConvertImageLocal(const SCM_PARAMETER *sp, const EPBM *I, EPBM *O,
					int OriginalWidth,int OriginalHeight);

      /*! Load camera parameter matrix */
      CORBA::Long CalibLoadPinholeParameter(const char *file, EPBM_PinHoleParameter *pin);

      /*! Rectify the images */
      CORBA::Long RectifyImages(EPBM *, EPBM *, int, int);

      /*! Returns a link to the disparity process to get the range image */
      HRP2DisparityProcess * GetDisparityProcess();

      /* @} */
#endif
      
      /*! \brief Cleanup the grabbing. */
      void CleanUpGrabbing();

    protected:

      /*! List of Low Level Vision Process. */
      vector <HRP2VisionBasicProcess *> m_ListOfProcesses;

      /*! Default value are 80 and 60 */
      vector<CORBA::Long> m_Width, m_Height;

      /*! Image Size used during calibration */
      vector<CORBA::Long> m_CalibrationWidth, m_CalibrationHeight;

    public:


#if (LLVS_HAVE_NMBT>0)

      /*! Circular Buffer */
      CircularModelTrackerData * m_CBonNMBT;
      

      /*! Model Tracker process. */
      HRP2nmbtTrackingProcess *m_ModelTrackerProcess;
    
      /*! Compute Control Law process. */
      HRP2ComputeControlLawProcess *m_ComputeControlLawProcess;
  
    private:
      /*! Corba object handling Tracker requests.*/
      ModelTrackerInterface_impl *  
	m_ModelTrackerCorbaRequestProcess_impl;
    
      /* struct save in Circular Buffer*/
      CBTrackerData*           m_CBTrackerData;
      
#endif

      
    protected:
#if (LLVS_HAVE_VVV>0)
      /*! Initial image in the VVV formalism. */
      EPBM m_epbm[4];

      /*! Undistorted image using the calibration information */
      EPBM m_epbm_distorted[4];

      /*! Corrected image using the calibration information */
      EPBM m_CorrectedImages[4];

      /*! File descriptor to the frame grabber */
      VFGB m_VFGBforTheFrameGrabber;

      /*! Structure storing all the camera models. */
      SCM_PARAMETER m_sp;

      /*! Disparity Process */
      HRP2DisparityProcess *m_DP;

      /*! Rectification process. */
      HRP2RectificationProcess *m_Rectification;

      /*! Mire detection process. */
      HRP2MireDetectionProcess *m_MireDetectionProcess;

     
      /*! Optical Flow Process */
      HRP2OpticalFlowProcess *m_OP;

      /*! Motion Evualation Process */
      HRP2MotionEvaluationProcess *m_MEP;

      /*! EdgeDetectionProcess */
      HRP2EdgeDetectionProcess * m_EdgeDetection;

      /*! BRepDetectionProcess */
      HRP2BRepDetectionProcess * m_BRepDetection;

      /*! FindFeaturesProcess */
      HRP2FindFeaturesInImage * m_FFII; 

      /*! Image Difference */
      HRP2ImageDifferenceProcess * m_ImgDiff;

      /*! Process to perform color detection */
      HRP2ColorDetectionProcess * m_ColorDetection;

      /*! VVV structures related to the calibration information */

      /*! Camera parameter */
      EPBM_PinHoleParameter m_PinHoleParameter[4];
 
      /*! Intensity parameters */
      EPBM_IntensityParameter m_IntensityParameter[4];

      /*! Distorsion parameters */
      EPBM_DistortionParameter m_DistortionParameter[4];

#endif

#if (LLVS_HAVE_BTL_SLAM>0)

			HRP2BtlSlamProcess* m_BtlSlamProcess;

#endif //LLVS_HAVE_BTL_SLAM

#if (LLVS_HAVE_SCENE>0)
      /*! SingleCameraSLAMProcess */
      HRP2SingleCameraSLAMProcess * m_SingleCameraSLAM;
#endif


#if (LLVS_HAVE_VISP>0)
    public:
      /*! Circular Buffer */
      CircularPointTrackerData * m_CBonPointTracker;

      /*! Model Tracker process. */
      HRP2PointTrackingProcess *m_PointTrackerProcess;

    private:
      /*! Corba object handling Point Tracker requests.*/
      PointTrackerInterface_impl * 
	m_PointTrackerCorbaRequestProcess_impl;
   
      /* struct save in Circular Buffer*/
      CBPointTrackerData*   m_CBPointTrackerData;
 
    protected:
      /*Visp grey undistorded image for wide cam*/
      vpImage<unsigned char>* m_Widecam_image_undistorded;

      /* Visp Camera parameters*/
      vpCameraParameters      m_Widecam_param;

      /* Vision Process to undistort images using ViSP*/
      HRP2vispUndistordedProcess* m_vispUndistordedProcess;
      
      /* Path to the camera parmeter XML*/
      std::string             m_CamParamPath;
 
#endif

      /*! Binary maps of the images */
      vector<unsigned char *>m_BinaryImages;

      /*! Binary maps of the images */
      vector<unsigned char *>m_BinaryImages_corrected;

      /*! Binary maps of the images undistorted */
      vector<unsigned char *>m_BinaryImages_undistorted;

      /*! A pointer to the object providing the images */
      HRP2ImagesInputMethod * m_ImagesInputMethod;

      /*! Integer storing the Input method. */
      LowLevelVisionSystem::InputMode m_TypeOfInputMethod;

      /*! Stores the Synchronization method */
      LowLevelVisionSystem::SynchroMode m_TypeOfSynchro;

      /*! Connection to SoT. */
      ConnectionToSot * m_CTS;
      /*! Verbosity Level */
      int m_Verbosity;

      /*! Vision Calibration directory */
      string m_CalibrationDirectory;

      /*! Robot and Vision Calibration directory */
      string m_RobotVisionCalibrationDirectory;

      /*! Boolean value on the overall computation */
      unsigned char m_Computing;

      /*! Camera parameters object for the CORBA interface */
      vector<Camera_impl *> m_Cameras;

      /*! CORBA object counterparts of those objects. */
      vector<HRP2Camera_var> m_Cameras_var;

      /*! Implementation of the stereo vision object */
      StereoVision_impl * m_StereoVision_impl;


      /*! Boolean vector to check the input of the module */
      unsigned char m_CheckEntry;

      /*! Dump image mode */
      LowLevelVisionSystem::DumpImageMode m_DumpImageMode;

      /*! Informations related to the image */
      vector <float> m_DumpInformations;

      /*! Boolean on the constructor end */
      bool m_EndOfConstructor;

      /*! Boolean on the Framegrabber's trigger. */
      bool m_SynchroTrigger;

      /* Matrix from Vision reference Frame to HEAD_JOINT1 
	 reference frame */
      double m_headTorg[16];

      /*! Timestamp for images */
      vector<struct timeval> m_timestamps;

      /*! Depth of the acquired image */
      vector<unsigned int> m_depth;


      /*! Local Image Format */
      string m_ImageFormat;

 
      unsigned char *m_StoredImages;
      struct timeval *m_StoredTimeStamp;
      double * m_SideOfTheImage;
      double * m_StoredCameraPosOri;
      double * m_StoredCameraCov;
      unsigned long int m_IndexSI;
      unsigned long int m_IndexSensorsStack;
      unsigned long int m_MaxSI;
      unsigned long int m_NumberOfImagesToStack;
      unsigned long int m_TheSLAMImage;

      int m_ImageCounter;

      // Store the orb.
      CORBA::ORB_var m_orb;

      // Context for naming service.
      CosNaming::NamingContext_var m_cxt;
      int dump1,dump2;
    };

};

#endif /* _HRP2_VISION_SERVER_ */

