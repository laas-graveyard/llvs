/** @doc This object implements the CORBA server providing
    Low Level Vision on the HRP-2 Vision processor.

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
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
//#define _GNU_SOURCE
#include <getopt.h>

#ifdef OMNIORB4
#include <omniORB4/CORBA.h>
#endif /* OMNIORB4 */

#ifdef __ORBIX__
#include <OBE/CORBA.h>
#include <OBE/CosNaming.h>
#endif

#if (LLVS_HAVE_VVV>0)
extern "C"
{
#include <ct3001.h>
#include <pebutil.h>
#include <calib.h>
#include <vfgbtype.h>
#include <vfgb.h>
#include <vfgbct3001.h>
#include <scm.h>
}
#endif


using namespace std;

#include "Corba/Camera_impl.h"
#include "LowLevelVisionServer.h"
#include "Simu/FileImagesInputMethod.h"
#include "Simu/SimulatorInputMethod.h"

#if (LLVS_HAVE_VVV>0)
#include "VVV/IEEE1394ImagesInputMethod.h"
#endif

#if (LLVS_HAVE_DC1394_V2>0)
#include "dc1394/IEEE1394DCImagesInputMethod.h"
#endif

#if(LLVS_HAVE_KALMAN_FILTER>0)
#include "ModelTracker/kalmanOnNMBTProcess.h"
#endif  

#if (LLVS_HAVE_HRP_BTL_SLAM>0)
#include "Corba/BtlSlamInterface_impl.h"
#endif

#include <llvs/tools/Debug.h>

using namespace llvs;

union PixelEncode_t
{
  int aInt;
  unsigned char Images[4];
};

LowLevelVisionServer::LowLevelVisionServer(LowLevelVisionSystem::InputMode MethodForInputImages,
					   LowLevelVisionSystem::SynchroMode SynchroMethodForInputImages,
					   string afilename,
					   CORBA::ORB_var orb,
					   int Verbosity=0,
					   string lCalibDir="") throw(const char*)
{



  m_Computing = 0;
  m_Verbosity = Verbosity;
#if (LLVS_HAVE_VVV>0)
  m_Rectification = 0;
  m_MireDetectionProcess= 0;
  m_DP = 0;
  m_MEP = 0;
  m_OP = 0;
  m_EdgeDetection = 0;
  m_BRepDetection = 0;
  m_FFII = 0;
#endif

#if (LLVS_HAVE_VVV>0)
  m_SingleCameraSLAM = 0;
#endif

  m_orb = orb;
  CreateNameContext();

  ODEBUG("Step 0");
  m_CheckEntry = 0;

  m_EndOfConstructor = false;
  m_DumpImageMode = LowLevelVisionSystem::SINGLE;
  m_DumpInformations.clear();


  m_StoredImages=0;
  m_StoredTimeStamp=0;
  m_SideOfTheImage=0;
  m_StoredCameraPosOri=0;
  m_ImageCounter = 0;
  m_TheSLAMImage = 0;

  m_ImagesInputMethod = 0;


  /* ******* Create the Stereo Vision object *********** */
  /* It is an interface to call the VVV scripts . */
  m_StereoVision_impl = new StereoVision_impl(m_orb,this);

#if (LLVS_HAVE_NMBT>0)
  /*! Is is an interface to call the NMBT tracker. */
  m_ModelTrackerCorbaRequestProcess_impl =
    new ModelTrackerInterface_impl(this);
#endif

#if (LLVS_HAVE_VISP>0)
  /*! Is is an interface to call the Point tracker. */
  m_PointTrackerCorbaRequestProcess_impl =
    new PointTrackerInterface_impl(this);
#endif

  ODEBUG("Step 1");

#if (LLVS_HAVE_VVV>0)
  /* Initialize the epbm images */

  epbm_minitialize(m_epbm,4);
  epbm_minitialize(m_CorrectedImages,4);
  epbm_minitialize(m_epbm_distorted,4);
  for(int i=0;i<4;i++)
    if (m_depth[i]==1)
      {
	m_epbm[i].Magic2 = EPBM_BINARY_GRAY;
	m_CorrectedImages[i].Magic2= EPBM_BINARY_GRAY;
	m_epbm_distorted[i].Magic2=EPBM_BINARY_GRAY;
      }
    else if (m_depth[i]==3)
      {
	m_epbm[i].Magic2 = EPBM_BINARY_COLOR;
	m_CorrectedImages[i].Magic2= EPBM_BINARY_COLOR;
	m_epbm_distorted[i].Magic2=EPBM_BINARY_COLOR;
      }
#endif

  /* Create the appropriate object depending on the target. */
  m_TypeOfInputMethod = MethodForInputImages;
  m_TypeOfSynchro = SynchroMethodForInputImages;

  ODEBUG("Step 2");
  ODEBUG("Type of Input Method: " << m_TypeOfInputMethod << " (files:" << LowLevelVisionSystem::FILES <<", framegrabber:"<< LowLevelVisionSystem::FRAMEGRABBER);
  switch(m_TypeOfInputMethod)
    {

    case LowLevelVisionSystem::FRAMEGRABBER :
#if (LLVS_HAVE_VVV>0)
      if(m_ImagesInputMethod==0)
	{
	  HRP2IEEE1394ImagesInputMethod *aIIIM=0;

	  aIIIM = 	new HRP2IEEE1394ImagesInputMethod();
	  m_ImagesInputMethod = (HRP2ImagesInputMethod *)aIIIM;
	  m_ListOfProcesses.insert(m_ListOfProcesses.end(),aIIIM);
	}
#endif

#if (LLVS_HAVE_DC1394_V2>0)
      ODEBUG("FRAMEGRABBER, DC1394_V2 Images Input Method: " << m_ImagesInputMethod );
      if(m_ImagesInputMethod==0)
	{
	  HRP2IEEE1394DCImagesInputMethod *aIIIM=0;

	  aIIIM = 	new HRP2IEEE1394DCImagesInputMethod();
	  m_ImagesInputMethod = (HRP2ImagesInputMethod *)aIIIM;
	  m_ListOfProcesses.insert(m_ListOfProcesses.end(),aIIIM);
	}
#endif

      break;

    case LowLevelVisionSystem::FILES :
      m_ImagesInputMethod = (HRP2ImagesInputMethod *) new HRP2FileImagesInputMethod(HRP2FileImagesInputMethod::DIRECTORY);
      ((HRP2FileImagesInputMethod *)m_ImagesInputMethod)->SetBaseName(afilename);
      break;

    case LowLevelVisionSystem::FILESINGLE :
      m_ImagesInputMethod = (HRP2ImagesInputMethod *) new HRP2FileImagesInputMethod(HRP2FileImagesInputMethod::ONEIMAGE);
      ((HRP2FileImagesInputMethod *)m_ImagesInputMethod)->SetBaseName(afilename);
      break;

    case LowLevelVisionSystem::SIMULATION:
      m_ImagesInputMethod = (HRP2ImagesInputMethod *) new HRP2SimulatorInputMethod(0,0,m_orb);

    default:
      break;

    }

  if (!m_ImagesInputMethod)
    {
      throw("No vision system detected or simulation method set on");
    }

  if (!m_ImagesInputMethod->CameraPresent())
    {
      throw("No camera detected or simulation method set on");
    }

  /* Resize all the associated vectors */
  int lNbCams = 4;//m_ImagesInputMethod->GetNumberOfCameras();
  m_Width.resize(lNbCams);
  m_Height.resize(lNbCams);
  m_depth.resize(lNbCams);
  m_BinaryImages.resize(lNbCams);
  m_BinaryImages_corrected.resize(lNbCams);
  m_BinaryImages_undistorted.resize(lNbCams);
  m_timestamps.resize(lNbCams);

#if (LLVS_HAVE_VVV>0)
  m_epbm.resize(lNbCams);
  m_CorrectedImages.resize(lNbCams);
  m_epbm_distorted.resize(lNbCams);
#endif

  /* ******* Create the Camera parameter objects *********** */
  m_Cameras.resize(lNbCams);
  m_Cameras_var.resize(lNbCams);

  for(unsigned int i=0;i<m_ImagesInputMethod->GetNumberOfCameras();i++)
    {
      m_Cameras[i] = new Camera_impl(0,this);
      m_Cameras_var[i] = m_Cameras[i]->_this();
    }

  ODEBUG("Step 3");
  if (m_ImagesInputMethod!=0)
    {
      //      m_ImagesInputMethod->SetLevelOfVerbosity(m_Verbosity);
      m_ImagesInputMethod->SetLevelOfVerbosity(5);
    }

  /* Fix the size of the image */
  SetImagesGrabbedSize(CAMERA_LEFT,640,480);
  SetImagesGrabbedSize(CAMERA_RIGHT,640,480);
  SetImagesGrabbedSize(CAMERA_UP,640,480);
  SetImagesGrabbedSize(CAMERA_WIDE,320,240);

  /* Set the calibration directory  */
  SetCalibrationDirectory(lCalibDir);

  /* Load Calibration parameters */
  LoadCalibrationInformation();


  ODEBUG("Step 4");
  /* ********** PROCESS CREATION ***************** */

#if (LLVS_HAVE_VVV>0)
  /* Rectification Process */
  m_Rectification = new HRP2RectificationProcess();
  m_Rectification->StopProcess();
  //  m_Rectification->SetInputImages(m_epbm_distorted);
  m_Rectification->SetInputImages(m_epbm);
  m_Rectification->SetOutputImages(m_CorrectedImages);
  m_Rectification->SetSP(&m_sp);
  m_Rectification->SetCalibrationSize(m_CalibrationWidth,m_CalibrationHeight);
  m_ListOfProcesses.insert(m_ListOfProcesses.end(),m_Rectification);

  /* Mire detection process */
  m_MireDetectionProcess = new HRP2MireDetectionProcess();
  m_MireDetectionProcess->StopProcess();
  m_MireDetectionProcess->SetInputImages(m_epbm);
  m_ListOfProcesses.insert(m_ListOfProcesses.end(),m_MireDetectionProcess);

  /* Disparity process */
  m_DP = new HRP2DisparityProcess(0);
  m_DP->SetInputImages(m_CorrectedImages);
  m_DP->SetCalibrationSize(m_CalibrationWidth[0], m_CalibrationHeight[0]);
  m_DP->InitializeTheProcess(m_CalibrationWidth[0], m_CalibrationHeight[0]);
  m_DP->SetLevelOfVerbosity(Verbosity);
  m_DP->StopProcess();
  m_ListOfProcesses.insert(m_ListOfProcesses.end(),m_DP);

  /* Optical Flow process */
#if 0
  m_OP = new HRP2OpticalFlowProcess(m_Width, m_Height);
  m_OP->SetInputImages(m_CorrectedImages);
  m_OP->InitializeTheProcess();
  m_OP->SetLevelOfVerbosity(Verbosity);
  m_OP->StopProcess();
  m_ListOfProcesses.insert(m_ListOfProcesses.end(),m_OP);


  /* Motion Evaluation process */
  m_MEP = new HRP2MotionEvaluationProcess();
  m_MEP->InitializeTheProcess();
  m_MEP->SetLevelOfVerbosity(Verbosity);
  m_MEP->StopProcess();
  m_ListOfProcesses.insert(m_ListOfProcesses.end(),m_MEP);

  /* Edge Detection process */
  m_EdgeDetection = new HRP2EdgeDetectionProcess();
  m_EdgeDetection->SetInputImages(m_CorrectedImages);
  m_EdgeDetection->InitializeTheProcess();
  m_EdgeDetection->SetLevelOfVerbosity(Verbosity);
  m_EdgeDetection->StopProcess();
  m_ListOfProcesses.insert(m_ListOfProcesses.end(),m_EdgeDetection);

  /* BRep Detection process */
  m_BRepDetection = new HRP2BRepDetectionProcess();
  m_BRepDetection->SetInputImages(m_CorrectedImages);
  m_BRepDetection->InitializeTheProcess();
  m_BRepDetection->SetLevelOfVerbosity(Verbosity);
  m_BRepDetection->StopProcess();
  m_ListOfProcesses.insert(m_ListOfProcesses.end(),m_BRepDetection);

#endif


  /* Create FindFeatureProcess */
  m_FFII = new HRP2FindFeaturesInImage();
  m_FFII->SetInputImages(m_epbm);
  m_FFII->SetLevelOfVerbosity(Verbosity);
  m_FFII->InitializeTheProcess(m_CalibrationDirectory);
  m_FFII->StopProcess();
  m_ListOfProcesses.insert(m_ListOfProcesses.end(), m_FFII);

  /* Create ImageDifferenceProcess */
  m_ImgDiff = new HRP2ImageDifferenceProcess();
  m_ImgDiff->SetInputImages(m_epbm);
  m_ImgDiff->SetTimeStamp(m_timestamps);
  m_ImgDiff->StopProcess();
  m_ImgDiff->InitializeTheProcess();
  m_ImgDiff->SetLevelOfVerbosity(Verbosity);
  m_ListOfProcesses.insert(m_ListOfProcesses.end(), m_ImgDiff);


  /* Create Color detection process */
  m_ColorDetection = new HRP2ColorDetectionProcess(m_cxt,this,
						   m_CalibrationDirectory);
  m_ColorDetection->SetInputImages(m_epbm[0],m_epbm[1]);
  m_ColorDetection->InitializeTheProcess();
  m_ColorDetection->StopProcess();

  //  m_ColorDetection->StopProcess();
  m_ListOfProcesses.insert(m_ListOfProcesses.end(),m_ColorDetection);
#endif

#if (LLVS_HAVE_SCENE>0)
  /* Single Camera SLAM process */
  m_SingleCameraSLAM = new HRP2SingleCameraSLAMProcess(m_orb,m_cxt,this);
  m_SingleCameraSLAM->SetInputImages(&m_epbm[m_TheSLAMImage]);
  m_SingleCameraSLAM->SetLevelOfVerbosity(Verbosity);
  m_SingleCameraSLAM->StopProcess();
  m_ListOfProcesses.insert(m_ListOfProcesses.end(), m_SingleCameraSLAM);
#endif


#if(LLVS_HAVE_VISP>0)

  /*!ViSP Undistort process. */
  m_CamParamPath="./data/ViSP/hrp2CamParam/hrp2.xml";
  m_Widecam_image_undistorded = new vpImage<unsigned char>;
  m_Widecam_image_undistorded -> resize( m_Height[CAMERA_WIDE],m_Width[CAMERA_WIDE]);

  vpXmlParserCamera       m_ParserCam;
  m_ParserCam.parse(m_Widecam_param,
		    m_CamParamPath.c_str(),
		    "cam1394_3",
		    vpCameraParameters::perspectiveProjWithDistortion,
		    m_Width[CAMERA_WIDE],
		    m_Height[CAMERA_WIDE]);

  m_vispUndistordedProcess = new HRP2vispUndistordedProcess(HRP2vispUndistordedProcess::RGB_VISPU8);
  m_vispUndistordedProcess->InitializeTheProcess();

  m_vispUndistordedProcess->StopProcess();
  m_vispUndistordedProcess->SetImages(&(m_BinaryImages[CAMERA_WIDE]),
				      m_Widecam_image_undistorded);
  m_vispUndistordedProcess->SetCameraParameters(m_Widecam_param);
  m_ListOfProcesses.insert(m_ListOfProcesses.end(), m_vispUndistordedProcess);


  /*! Point Tracker process. */
  m_PointTrackerProcess = new HRP2PointTrackingProcess();
  m_PointTrackerProcess->SetInputVispImages (m_Widecam_image_undistorded);
  m_PointTrackerProcess->StopProcess();
  m_ListOfProcesses.insert(m_ListOfProcesses.end(),m_PointTrackerProcess);


  /* Circular Buffer for the point tracker data*/
  m_CBPointTrackerData= new CBPointTrackerData();
  m_CBPointTrackerData->image=m_Widecam_image_undistorded;
  m_CBPointTrackerData->timestamp=&m_timestamps[CAMERA_WIDE];



  m_CBonPointTracker=new CircularPointTrackerData(3);
  m_CBonPointTracker->SetPointTrackerPointer(m_PointTrackerProcess);
  m_CBonPointTracker->SetDatum(m_CBPointTrackerData);
  m_CBonPointTracker->StopProcess();
  m_ListOfProcesses.insert(m_ListOfProcesses.end(), m_CBonPointTracker);


#endif

#if (LLVS_HAVE_NMBT>0)
  /*! Model Tracker process. */


  //TODO find a better way to do define the m_ModelTrackerProcess
  bool useKalmanFilter=false;
  if( useKalmanFilter && LLVS_HAVE_KALMAN_FILTER>0)
    {
      ODEBUG("creation of HRP2KalmanOnNMBTProcess");

      m_ModelTrackerProcess = new HRP2KalmanOnNMBTProcess();
      HRP2KalmanOnNMBTProcess* lKalmanOnNMBTProcess;
      lKalmanOnNMBTProcess = dynamic_cast<HRP2KalmanOnNMBTProcess*> (m_ModelTrackerProcess);
      lKalmanOnNMBTProcess->SetTimeStamp(&m_timestamps[CAMERA_WIDE]);
    }
  else
    {
      ODEBUG("creation of HRP2nmbtTrackingProcess");
      m_ModelTrackerProcess = new HRP2nmbtTrackingProcess();
    }

  m_ModelTrackerProcess->SetInputVispImages (m_Widecam_image_undistorded);
  m_ModelTrackerProcess->StopProcess();
  m_ListOfProcesses.insert(m_ListOfProcesses.end(),m_ModelTrackerProcess);


  /*! Compute Control Law process. */
  m_ComputeControlLawProcess = new HRP2ComputeControlLawProcess();
  m_ComputeControlLawProcess->SetTracker(m_ModelTrackerProcess);
  m_ComputeControlLawProcess->StopProcess();
  m_ListOfProcesses.insert(m_ListOfProcesses.end(),m_ComputeControlLawProcess);


  /* Circular Buffer for the tracker data*/
  m_CBTrackerData= new CBTrackerData();
  m_CBTrackerData->image=m_Widecam_image_undistorded;
  m_CBTrackerData->timestamp=&m_timestamps[CAMERA_WIDE];


  m_CBonNMBT=new CircularModelTrackerData(3);
  m_CBonNMBT->SetTrackerPointer(m_ModelTrackerProcess);
  m_CBonNMBT->SetDatum(m_CBTrackerData);
  m_CBonNMBT->StopProcess();
  m_ListOfProcesses.insert(m_ListOfProcesses.end(), m_CBonNMBT);
#endif

#if (LLVS_HAVE_HRP_BTL_SLAM>0)

  m_BtlSlamProcess = new HRP2BtlSlamProcess();
  m_BtlSlamProcess->SetInputImages(&m_BinaryImages[CAMERA_WIDE]);
  m_ListOfProcesses.insert(m_ListOfProcesses.end(), m_BtlSlamProcess);

#endif //LLVS_HAVE_HRP_BTL_SLAM

  /* Set the framegrabber trigger to zero. */
  m_SynchroTrigger = false;


  ODEBUG("Step 5");


  m_RobotVisionCalibrationDirectory="/home/hrpuser/hrp2/HRP2eyes/rbt_calib/hmat";
  for(int i=0;i<16;i++)
    m_headTorg[i] = 0.0;

  m_CTS=0;
  try
    {
      m_CTS = new ConnectionToSot(this);
      if (!m_CTS->SetCorbaReference())
	{
	  cout << "Connection to SoT disabled." << endl;
	  delete m_CTS;
	  m_CTS=0;
	}
      else
	{
	  m_CTS->Init();
	  m_CTS->StartThreadOnConnectionSot();
	}
    }
  catch(...)
    {
      m_CTS=0;
    }

#if (LLVS_HAVE_NMBT>0)
  m_ComputeControlLawProcess->SetConnectionToSot(m_CTS);
#endif

  /* SHOULD ALWAYS BE AT THE END */
  m_EndOfConstructor = true;


}

LowLevelVisionServer::~LowLevelVisionServer()
{

#if (LLVS_HAVE_VVV>0)
  if (m_DP!=0)
    delete m_DP;

  if (m_OP!=0)
    delete m_OP;

  if (m_MEP!=0)
    delete m_MEP;
#endif

  /* Free the memory taken by the binary images */
  FreeBinaryImages();

  /* Close the input image method. */
  delete m_ImagesInputMethod;

  if (m_CheckEntry)
    DeleteStack();

  /* Release the implementation of servant objects */
  for(int i=0;i<4;i++)
    if (m_Cameras[i] !=0)
      delete m_Cameras[i];

#if (LLVS_HAVE_VVV>0)
  scm_Free(&m_sp);
#endif

}


CORBA::Long
LowLevelVisionServer::FreeBinaryImages()
{
  for(unsigned int i=0;i<m_BinaryImages.size();i++)
    {
      if (m_BinaryImages[i]!=0)
	{
	  delete[] m_BinaryImages[i];
	  m_BinaryImages[i] = 0;
	}
    }

#if (LLVS_HAVE_VVV>0)
  for(unsigned int i=0;i<m_CorrectedImages[i].size();i++)
    {
      if (m_CorrectedImages[i].Image!=0)
	delete (unsigned char *)m_CorrectedImages[i].Image;
    }

  for(unsigned int i=0;i<m_epbm_distorted.size();i++)
    {
      if (m_epbm_distorted[i].Image!=0)
	delete (unsigned char *)m_epbm_distorted[i].Image;
    }
#endif
  return 0;
}

CORBA::Long
LowLevelVisionServer::SetImagesGrabbedSize(CORBA::Long SemanticCameraID, CORBA::Long lw, CORBA::Long lh)
  throw(CORBA::SystemException)
{
  double RatioForFocal =1.0;
  unsigned char first_assignment = 0;
  int res =0;
  /* Should first check if the ratio lw/lh
     is the same than previously. Otherwise a different
     calibration data set should be used.
  */
  if (m_ImagesInputMethod==0)
    return -1;


  if ((m_Width[SemanticCameraID]!=0) && (m_Height[SemanticCameraID]!=0))
    {
      if (((double)lw/(double)lh)!=((double)m_Width[SemanticCameraID]/(double)m_Height[SemanticCameraID]))
	{
	  res= -1;
	}

      RatioForFocal = (double)lw / m_Width[SemanticCameraID];
    }
  else
    {

      /* This is the first size assignment */
      RatioForFocal = 1.0;
      first_assignment = 1;
    }

  m_Width[SemanticCameraID] = lw;
  m_Height[SemanticCameraID] = lh;

  m_ImagesInputMethod->SetImageSize(m_Width[SemanticCameraID], m_Height[SemanticCameraID],SemanticCameraID);

#if (LLVS_HAVE_VVV>0)
  m_VFGBforTheFrameGrabber._width = lw;
  m_VFGBforTheFrameGrabber._height = lh;
#endif
  /*
    ct3001_image_set_size((CT3001_SYSCONF*)(m_VFGBforTheFrameGrabber._sys),
    m_VFGBforTheFrameGrabber._width,
    m_VFGBforTheFrameGrabber._height);
  */
  //  FreeBinaryImages();


  unsigned char ** local_BinaryImages;
  if (m_Cameras[SemanticCameraID]!=0)
    m_Cameras[SemanticCameraID]->SetAcquisitionSize(m_Width[SemanticCameraID],m_Height[SemanticCameraID]);

#if (LLVS_HAVE_VVV>0)
  if (m_CorrectedImages[SemanticCameraID].Image!=0)
    {
      delete (unsigned char *)m_CorrectedImages[SemanticCameraID].Image;
    }

  SemanticCameraIDf (m_epbm_distorted[SemanticCameraID].Image!=0)
    {
      delete (unsigned char *)m_epbm_distorted[SemanticCameraID].Image;
    }
#endif

  m_ImageFormat = m_ImagesInputMethod->GetFormat(SemanticCameraID);

  ODEBUG("m_ImageFormat :" << m_ImageFormat);
  if (m_ImageFormat=="PGM")
    m_depth[SemanticCameraID]=1;
  else if (m_ImageFormat=="RAW")
    m_depth[SemanticCameraID]=1;
  else if (m_ImageFormat=="RGB")
    m_depth[SemanticCameraID]=3;

  /*
    cout << "Depth" << endl;
    for(int i=0;i<4;i++)
    cout << m_depth[i] << " ";
    cout << endl;
  */

  delete [] m_BinaryImages[SemanticCameraID];

  ODEBUG("lw: " << lw << "lh " << lh << "depth: " << m_depth[SemanticCameraID]);
  m_BinaryImages[SemanticCameraID] = new unsigned char[lw*lh*m_depth[SemanticCameraID]];

  /* NO NEED TO FREE corrected and undistorted
     as it is already done with m_CorrectedImages and m_epbm_distorted */
  m_BinaryImages_corrected[SemanticCameraID] = new unsigned char[lw*lh*m_depth[SemanticCameraID]];

  m_BinaryImages_undistorted[SemanticCameraID] = new unsigned char [lw*lh*m_depth[SemanticCameraID]];


#if (LLVS_HAVE_VVV>0)
  epbm_set_default_value(&(m_epbm[SemanticCameraID]),0);
  if (m_depth[SemanticCameraID]==1)
    {
      m_epbm[SemanticCameraID].Magic2 = EPBM_BINARY_GRAY;
      m_CorrectedImages[SemanticCameraID].Magic2= EPBM_BINARY_GRAY;
      m_epbm_distorted[SemanticCameraID].Magic2=EPBM_BINARY_GRAY;
    }
  else if (m_depth[SemanticCameraID]==3)
    {
      m_epbm[SemanticCameraID].Magic2 = EPBM_BINARY_COLOR;
      m_CorrectedImages[SemanticCameraID].Magic2= EPBM_BINARY_COLOR;
      m_epbm_distorted[SemanticCameraID].Magic2=EPBM_BINARY_COLOR;
    }

  m_epbm[SemanticCameraID].Image = m_BinaryImages[SemanticCameraID];
  m_CorrectedImages[SemanticCameraID].Image = m_BinaryImages_corrected[SemanticCameraID];
  m_epbm_distorted[SemanticCameraID].Image = m_BinaryImages_undistorted[i];
  m_epbm[SemanticCameraID].Width = m_CorrectedImages[SemanticCameraID].Width =
    m_epbm_distorted[SemanticCameraID].Width = lw;
  m_epbm[SemanticCameraID].Height = m_CorrectedImages[SemanticCameraID].Height =
    m_epbm_distorted[SemanticCameraID].Height =  lh;

  if (m_ImagesInputMethod!=0)

    {
      m_ImagesInputMethod->GetImageSize(m_epbm[SemanticCameraID].Width,m_epbm[SemanticCameraID].Height,SemanticCameraID);
    }

  m_epbm[SemanticCameraID].Label |= EPBM_HAVE_PINHOLEPARAMETER_MASK;
#endif


#if (LLVS_HAVE_VVV>0)
  if (m_Rectification!=0)
    {
      m_Rectification->SetInputImages(m_epbm);
      //      m_Rectification->SetInputImages(m_epbm_distorted);
      m_Rectification->SetOutputImages(m_CorrectedImages);
    }

  if (m_MireDetectionProcess!=0)
    {
      m_MireDetectionProcess->SetInputImages(m_epbm);
    }

  if (m_DP!=0)
    {
      m_DP->SetInputImages(m_CorrectedImages);
      m_DP->InitializeTheProcess(m_CalibrationWidth[0], m_CalibrationHeight[0]);
    }

  if (m_OP!=0)
    {
      m_OP->SetInputImages(m_CorrectedImages);
      m_OP->InitializeTheProcess();
    }


  if (m_MEP!=0)
    {
      m_MEP->InitializeTheProcess();
    }

  if (m_FFII!=0)
    {
      m_FFII->SetInputImages(m_epbm);
      m_FFII->InitializeTheProcess(m_CalibrationDirectory);
    }
  if (m_ImgDiff!=0)
    {
      m_ImgDiff->SetInputImages(m_epbm);
      m_ImgDiff->InitializeTheProcess();
    }

  if (m_ColorDetection!=0)
    {
      m_ColorDetection->InitializeTheProcess();
    }

#endif

#if (LLVS_HAVE_SLAM>0)
  if (m_SingleCameraSLAM!=0)
    {
      m_FFII->SetInputImages(m_epbm+m_TheSLAMImage);
      m_SingleCameraSLAM->SetFindFeaturesFromWideImage(m_FFII->GetFindFeaturesFromWideImage(),m_headTorg);
    }
#endif

  return res;
}

LowLevelVisionSystem::InputMode
LowLevelVisionServer::GetInputMode()
  throw(CORBA::SystemException)
{
  return m_TypeOfInputMethod;
}

CORBA::Long
LowLevelVisionServer::TriggerSynchro()
  throw(CORBA::SystemException)
{
  static unsigned char start =1;
  static struct timeval time_last;
  struct timeval time_current;

  ODEBUG("TriggerSynchro() beginning");
  gettimeofday(&time_current,0);
  if (start!=1)
    {
      double current_diff = time_current.tv_sec - time_last.tv_sec
	+ 0.000001 * (time_current.tv_usec - time_last.tv_usec);

      if (m_Verbosity>=3)
	cout << "trigger  " << current_diff << endl;
    }
  else
    start = 0;
  time_last = time_current;

  if (m_Verbosity>=3)
    cout << "Trigger" << endl;

  m_SynchroTrigger = true;
  ODEBUG("TriggerSynchro() endiing");
  return 0;
}

void LowLevelVisionServer::LensDistorsionCorrection()
{
#if (LLVS_HAVE_VVV>0)
  EPBM I;
  int i, j;
  unsigned char *uptr;
  double fi, fj;
  int ni, nj;
  static MAT_Vector cri=0, cro=0;

  if (cri==0)

    cri = mat_new_vector(2, 0);

  if (cro==0)
    cro = mat_new_vector(2, 0);

  for(int l=0;l<3;l++)
    {
      for (i = 0, uptr = (unsigned char *)m_epbm_distorted[l].Image;
	   i < m_epbm[l].Height; i++) {
	for (j = 0; j < m_epbm[l].Width; j++, uptr++) {
	  if ((double)j == m_DistortionParameter[l].cold &&
	      (double)i == m_DistortionParameter[l].rowd) {
	    *uptr = epbm_uc_getpixel(&m_epbm[l], i, j);
	  }
	  else {
	    cri[0] = (double)j;
	    cri[1] = (double)i;
	    calib_cr2distortedcr(&m_DistortionParameter[l], cri, cro);
	    nj = (int)floor(cro[0]);
	    fj = cro[0] - (double)nj;
	    ni = (int)floor(cro[1]);
	    fi = cro[1] - (double)ni;
	    //	    cout << cri[0] << " " << cri[1] << " " << nj << " " << ni;
	    if ((ni < 0) || (m_epbm_distorted[l].Height - 1 <= ni) ||
		(nj < 0) || (m_epbm_distorted[l].Width - 1 <= nj))
	      {
		*uptr = 0;
		//cout << " wrong " << m_epbm_distorted[i].Height << " " << m_epbm_distorted[i].Width;
	      }
	    else
	      *uptr = (unsigned char)
		(epbm_uc_getpixel(&m_epbm[l], ni, nj) * (1.0 - fi) * (1.0 - fj) +
		 epbm_uc_getpixel(&m_epbm[l], ni + 1, nj) * fi * (1.0 - fj) +
		 epbm_uc_getpixel(&m_epbm[l], ni, nj + 1) * (1.0 - fi) * fj +
		 epbm_uc_getpixel(&m_epbm[l], ni + 1, nj + 1) * fi * fj + 0.5);
	    //	    cout << endl;
	  }
	}
      }
    }
#endif
}

CORBA::Long
LowLevelVisionServer::ApplyingProcess()
{
  static double LastProcessTime = 0.0;
  static double WaistedTime = 0.0;
  static double PureProcess=0.0;
  static double BufferTime=0.0;
  static double CompleteProcessTime=0.0;
  static double ElapsedTime = 0.0;
  static double Variance=0.0, PrevBufferTime;
  static int IndexBuffer=0;
  static int MissedFrame = 0;
  static unsigned char FirstTime = 1;

  ODEBUG( __FILE__ << " " << __LINE__ );
  if ((!m_Computing) || (!m_EndOfConstructor)
      || (m_ImagesInputMethod==0))
    {
      ODEBUG("Do not go in ");
      return -1;
    }
  if (!m_ImagesInputMethod->CameraPresent())
    {
      ODEBUG("No camera available");
      return -1;
    }

  /* Time measurement */
  struct timeval before, before2, before3, after;
  static timeval prev_before = { 0,0};
  static timeval prev_before2 = { 0,0};

  gettimeofday(&before,0);
  int NbOfWait=0;
  ODEBUG("RealizeTheProcess: Scheduling the Grabbing");
  ODEBUG("RealizeTheProcess: My synchro is :" );

  int ResFromGIFF=-1;
  if (m_TypeOfSynchro==LowLevelVisionSystem::SYNCHRO_TRIGGER)
    {
      ODEBUG("TRIGGER");
      do
	{
	  usleep(1000);
	  // Modification 28/12/2005
	  // for exhausting the buffer of images.
	  NbOfWait++;
	  if ((NbOfWait==2) &&
	      (m_TypeOfInputMethod==LowLevelVisionSystem::FRAMEGRABBER))
	    {
	      ResFromGIFF=GetImageFromFrameGrabber();
	      NbOfWait=0;
	    }
	}
      while((!m_SynchroTrigger) ||
	    (ResFromGIFF==-1) );
      // Check if the trigger is due to a change of mode
      if(m_TypeOfSynchro == LowLevelVisionSystem::SYNCHRO_TRIGGER)
	{
	  m_SynchroTrigger = false;
	}
    }
  else
    {
      ODEBUG("FLOW");
      bool TooFast = false;
      bool GoIn=false;
      double lelapsed_time;
      if ((prev_before2.tv_sec!=0) ||
	  (prev_before2.tv_usec!=0))
	{
	  lelapsed_time =(before.tv_sec - prev_before2.tv_sec)+ 0.000001 * (before.tv_usec - prev_before2.tv_usec);
	  if (lelapsed_time < (0.015 - LastProcessTime))
	    TooFast  =true;
	}

      if (FirstTime)
	GoIn = true;
      if (TooFast)
	GoIn = true;
      struct timeval current_wait;
      if ((m_TypeOfSynchro==LowLevelVisionSystem::SYNCHRO_FLOW) &&
	  GoIn)
	{
	  ODEBUG("RealizeTheProcess: really going in ");
	  do
	    {
	      usleep(1000);

	      if (TooFast)
		{
		  gettimeofday(&current_wait,0);

		  lelapsed_time =(current_wait.tv_sec - prev_before2.tv_sec)
		    + 0.000001 * (current_wait.tv_usec - prev_before2.tv_usec);
		  if (lelapsed_time > (0.015 - LastProcessTime ))
		    TooFast = false;
		}

	      if((FirstTime) && (m_SynchroTrigger))
		GoIn = false;

	      if ((!FirstTime) && TooFast==false)
		GoIn = false;

	      //sleep(1);
	    }
	  while(GoIn) ;
	}
      // Check if the trigger is due to a change of mode
      if(m_TypeOfSynchro == LowLevelVisionSystem::SYNCHRO_FLOW)
	{
	  m_SynchroTrigger = false;
	}
      FirstTime = 0;
    }

  gettimeofday(&before2,0);
  /* Get the image from the input method */
  if (ResFromGIFF==-1)
    ResFromGIFF=GetImageFromFrameGrabber();
  ODEBUG("images grabbed" << ResFromGIFF);


  gettimeofday(&before3,0);
  //  LensDistorsionCorrection();


  if ( (m_DumpImageMode==LowLevelVisionSystem::SINGLE) ||
       (m_DumpImageMode==LowLevelVisionSystem::FLOW))
    {
      ODEBUG("Here STEP 1\n");
      static unsigned long counter=0;
      char Buffer[1024];
      bzero(Buffer,1024);
      sprintf(Buffer,"DumpImage_%06ld.epbm",counter);

#if (LLVS_HAVE_VVV>0)
      epbm_msave(Buffer,m_epbm,4,0);
#endif


      if (m_DumpInformations.size()!=0)
	{
	  bzero(Buffer,1024);
	  sprintf(Buffer,"DumpImage_%06ld.data",counter);

	  FILE *fp = fopen(Buffer,"w");
	  if (fp!=0)
	    {
	      for(unsigned int j=0;j<m_DumpInformations.size();j++)
		{
		  fprintf(fp,"%f ",m_DumpInformations[j]);
		}
	      fprintf(fp,"\n");
	      fclose(fp);
	    }
	}
      counter++;
      if (m_DumpImageMode==LowLevelVisionSystem::SINGLE)
	m_DumpImageMode=LowLevelVisionSystem::NONE;

      ODEBUG("Here STEP 2\n");
    }
  ODEBUG("Image grabbed" );

  // Perform computation for each process.
  unsigned int NbOfActiveProcesses = 0;
  for(unsigned int i=0;i<m_ListOfProcesses.size();i++)
    {
      ODEBUG("Process: " << m_ListOfProcesses[i]->GetName());
      m_ListOfProcesses[i]->RealizeTheProcess();
      if (m_ListOfProcesses[i]->GetStatus())
	{
	  ODEBUG(m_ListOfProcesses[i]->GetName());
	  NbOfActiveProcesses++;
	}
    }

  /* Applying Motion Evaluation */
#if 0
  if (m_MEP!=0)
    {
      if ((m_DP!=0) && (m_OP!=0))
	{
	  m_DP->UpdateAMotionEvaluationProcess(m_MEP);
	  m_OP->UpdateAMotionEvaluationProcess(m_MEP);
	}
      m_MEP->RealizeTheProcess();
    }
#endif
  ODEBUG("Motion evaluation computed");


  gettimeofday(&after,0);
  double waisted_time = (before2.tv_sec - before.tv_sec) + 0.000001*(before2.tv_usec - before.tv_usec);
  double current_time = (after.tv_sec - before.tv_sec) + 0.000001*(after.tv_usec - before.tv_usec);
  double current_time2 = (after.tv_sec - before2.tv_sec) + 0.000001*(after.tv_usec - before2.tv_usec);
  double current_time3 = (after.tv_sec - before3.tv_sec) + 0.000001*(after.tv_usec - before3.tv_usec);

  if ((prev_before.tv_sec!=0) ||
      (prev_before.tv_usec!=0))
    {
      double lelapsed_time =(before.tv_sec - prev_before.tv_sec)+ 0.000001 * (before.tv_usec - prev_before.tv_usec);
      ElapsedTime += lelapsed_time;
    }
  prev_before = before;
  prev_before2 = before2;
  BufferTime+=current_time2;
  PureProcess+=current_time3;
  CompleteProcessTime+=current_time;
  Variance += fabs(PrevBufferTime - current_time2);

  //  LastProcessTime = current_time2;
  LastProcessTime = before.tv_sec + 0.000001 * before.tv_usec;
  ODEBUG("Total computation time : "<< current_time3 << endl);

  if (GetVerboseMode()>=1)
    cout << "In the loop: " << current_time << " " << current_time2 << " " << current_time3 << endl;

#if 0
  if (m_TypeOfSynchro==LowLevelVisionSystem::SYNCHRO_FLOW)
    {
      if (current_time<0.033)
	usleep((unsigned int) (0.75*(33000 - (unsigned int)(1000000.0 * current_time))));
    }
#endif

  ODEBUG("NbOfActiveProcesses"<<NbOfActiveProcesses);

  if ((NbOfActiveProcesses>0)&&
      (IndexBuffer==300)  )
    {
      cout << "Complete process time :" << CompleteProcessTime/(double)IndexBuffer << endl;
      cout << "Time consumption average with acquisition : " << BufferTime/(double)IndexBuffer<< endl;
      cout << "Time consumption average without acquisition : " << PureProcess/(double)IndexBuffer<< endl;
      cout << "Average elapsed time: " << ElapsedTime/(double)IndexBuffer << endl;
      cout << "Variance for acquisition time: " << Variance/(double)IndexBuffer<< endl;
      cout << "MissedFrame : " << MissedFrame<<endl;
      CompleteProcessTime = 0.0;
      MissedFrame = 0;
      PrevBufferTime = BufferTime/(double)IndexBuffer;
      BufferTime = 0.0;
      ElapsedTime= 0.0;
      PureProcess = 0.0;
      Variance = 0;
      IndexBuffer=-1;
    }
  else if (IndexBuffer == 1000000)
    IndexBuffer = -1;

  if ((NbOfActiveProcesses==0) && (IndexBuffer==-1))
    {
      cout << "No active process" << endl;
    }

  m_ImageCounter++;
  IndexBuffer++;

  ODEBUG("End Index buffer: "<< IndexBuffer);
  return 0;
}

CORBA::Long
LowLevelVisionServer::GetImageFromFrameGrabber()
{
  int result=-1;
  unsigned int r;

  if (m_Verbosity>=2)
    cout << "Before ImagesInput Method : " << m_ImagesInputMethod->GetNumberOfCameras() << endl;

  if (m_ImagesInputMethod!=0)
    {/*
       string lFormat =     m_ImagesInputMethod->GetFormat(0);
       if ((lFormat=="PGM") && (m_depth[0]==3))
       {
       for(int unsigned j=0;j<4;j++)
       SetImagesGrabbedSize(j,m_Width[0],m_Height[0]);
       }

       if ((lFormat=="RAW") && (m_depth[0]==3))
       {
       for(int unsigned j=0;j<4;j++)
       SetImagesGrabbedSize(j,m_Width[0],m_Height[0]);
       }


       if ((lFormat=="RGB") && (m_depth[0]==1))
       {

       for(int unsigned j=0;j<4;j++)
       SetImagesGrabbedSize(j,m_Width[0],m_Height[0]);
       }
       ODEBUG(" lFormat: " << lFormat);

     */
      struct timeval tv_current;
      double CurrentTime;
      gettimeofday(&tv_current,0);
      CurrentTime = tv_current.tv_sec + 0.000001 * tv_current.tv_usec;

      int lNbOfCameras = m_ImagesInputMethod->GetNumberOfCameras();

      for(int li=0;li<lNbOfCameras;li++)
	{
	  if (m_ImagesInputMethod->NextTimeForGrabbing(li)<CurrentTime)
	    {
	      int SemanticCamera = m_ImagesInputMethod->GetSemanticOfCamera(li);
	      r = m_ImagesInputMethod->GetSingleImage(&m_BinaryImages[SemanticCamera],
						      SemanticCamera,
						      m_timestamps[SemanticCamera]);
	      if(r != HRP2ImagesInputMethod::RESULT_OK)
		{
		  ODEBUG("Could not grab image on camera semantic #" << SemanticCamera << " : Error code #" << r);
		  result = -1;
		}
	      else
		{
		  result = 0;
		}
	      if ((m_CheckEntry) && (r == HRP2ImagesInputMethod::RESULT_OK) && (li!=2))
		StoreImageOnStack(li);

	    }

	}

      /* Test if a reallocation has taking place
       * This may be the case while reading a file.
       */
      //FIXME: Simulation version of InputMethod (e.g. File) are not using
      // interface conventions regarding the result return value.
      // r == 2 has then no meaning and may make a collision with an
      // already defined error value. Lines regarding this test
      // are commented to avoid unwanted behaviour. To be fixed.
      /*
	if (r==2)
	{
	int lw,lh,i;
	if (m_ImagesInputMethod->GetImageSize(lw,lh,0)<0)
	cout << "Problem with the size" << endl;

	if (m_Verbosity>=2)
	cout << "Reallocation detected : " << lw << " " << lh << endl;

	// Fix the size of the image
	//	  for(int li=0;li<lNbOfCameras;li++)
	///	  SetImagesGrabbedSize(lw,lh);

	}
      */
    } // (if inputMethod is allocated)
  if (m_Verbosity>=2)
    cout << "After ImagesInput Method : " << endl;

  if (m_CheckEntry)
    {
#if 0
      static unsigned long int lcounter=0;
      FILE *fp;

      for(int i=0; i<m_ImagesInputMethod->GetNumberOfCameras(); i++)
	{
	  int k,l;
	  unsigned char *pt = m_BinaryImages[i];
	  char Buffer[1024];
	  bzero(Buffer,1024);
	  if (m_depth[0]==1)
	    sprintf(Buffer,"REALcheck%06d.pgm",lcounter++);
	  else
	    sprintf(Buffer,"REALcheck%06d.ppm",lcounter++);
	  fp = fopen(Buffer,"w");
	  if (m_Verbosity>=2)
	    cout << "Save the image : " << Buffer << endl;

	  if (fp!=0)
	    {
	      double TimeStamp=m_timestamps[i];
	      if (m_depth[0]==1)
		fprintf(fp,"P5\n");
	      else if (m_depth[0]==3)
		fprintf(fp,"P6\n");
	      fprintf(fp,"#%f\n%d %d\n255\n",TimeStamp,m_epbm[i].Width,m_epbm[i].Height);
	      for(l=0;l<m_epbm[i].Height;l++)
		{
		  for(k=0;k<m_epbm[i].Width;k++)
		    {
		      for(int m=0;m<m_depth[0];m++)
			fprintf(fp,"%c",*pt++);
		    }
		}

	      fclose(fp);
	    }
	}

      {
	char Buffer[1024];
	bzero(Buffer,1024);
	sprintf(Buffer,"DumpImage.epbm");
	epbm_msave(Buffer,m_epbm,4,0);
      }
#else
#endif

    }
  return result;
}

CORBA::Long LowLevelVisionServer::GetVersion()
  throw(CORBA::SystemException)
{

  return 1;
}

CORBA::Long LowLevelVisionServer::StartMainProcess()
  throw(CORBA::SystemException)
{
  m_Computing = 1;
  return 1;
}

CORBA::Long LowLevelVisionServer::StopMainProcess()
  throw(CORBA::SystemException)
{
  m_Computing = 0;

  for(unsigned int i=0;i<m_ListOfProcesses.size();i++)
    m_ListOfProcesses[i]->StopProcess();
  return 1;
}

void LowLevelVisionServer::CleanUpGrabbing()
{

  if (m_ImagesInputMethod!=0)
    m_ImagesInputMethod->Cleanup();
}

CORBA::Long LowLevelVisionServer::ProcessStatus(const char *aProcessName)
  throw(CORBA::SystemException)
{

  for(unsigned int i=0;i<m_ListOfProcesses.size();i++)
    {
      if (m_ListOfProcesses[i]->GetName()==aProcessName)
	{
	  return m_ListOfProcesses[i]->GetStatus();
	}
    }
  return 0;
}

CORBA::Long LowLevelVisionServer::StartProcess(const char *aProcessName)
  throw(CORBA::SystemException)
{
  ODEBUG("Process asked to be start :" << aProcessName);
  CORBA::Long r=1;
  for(unsigned int i=0;i<m_ListOfProcesses.size();i++)
    {

      if (m_ListOfProcesses[i]->GetName()==aProcessName)
	{
	  ODEBUG("Start process " << aProcessName << " " << i);
	  m_ListOfProcesses[i]->StartProcess();
	  ODEBUG( aProcessName << " " <<
		  m_ListOfProcesses[i]->GetStatus() << " " <<
		  m_ListOfProcesses[i] );
	}
    }
  return r;
}

CORBA::Long LowLevelVisionServer::StopProcess(const char *aProcessName)
  throw(CORBA::SystemException)
{

  CORBA::Long r=1;
  for(unsigned int i=0;i<m_ListOfProcesses.size();i++)
    {
      if (m_ListOfProcesses[i]->GetName()==aProcessName)
	{
	  m_ListOfProcesses[i]->StopProcess();
	}
    }

  return r;
}

#if (LLVS_HAVE_VVV>0)

CORBA::Long
LowLevelVisionServer::RectifyImages(CONST EPBM I[4],
				    EPBM O[4], CONST int n,
				    CONST int op_zoom)
{
  int i;

  /* 画像枚数が２、３枚以外のとき */
  if (n != 2 && n != 3) {
    return -1;
  }

  static unsigned char first_time = 1;

  if (first_time==1)
    {
      calib_check_epbm(&I[0], &I[1], 0);

      for(int i=0;i<2;i++)
	{
	  O[i].Width = m_CalibrationWidth[i];
	  O[i].Height = m_CalibrationHeight[i];
	}
      scm_Init(&O[0], &O[1], (n == 2) ? NULL : &O[2], op_zoom, &m_sp, 0);
      for(int i=0;i<2;i++)
	{
	  O[i].Width = I[i].Width;
	  O[i].Height = I[i].Height;
	}
      first_time = 0;
    }



  /* L,Rの画像が同一のものではないかすでに変換されている */

  for (i = 0; i < n; i++)
    {
      char FileName[256];
      scm_ConvertImageLocal(&m_sp, &I[i], &O[i], m_CalibrationWidth[i],m_CalibrationHeight[i]);
      //scm_ConvertImage(&m_sp, &I[i], &O[i],0);
#if 0
      sprintf(FileName,"/tmp/checkO_%03d.epbm",i);
      epbm_save(FileName,&O[i],0);
      sprintf(FileName,"/tmp/checkI_%03d.epbm",i);
      epbm_save(FileName,&I[i],0);
#endif

      O[i].Label |= EPBM_CONVERTED_SCM_MASK;
      O[i].PinHoleParameter->f = m_sp.f;
    }

  return 0;
}
#endif

LowLevelVisionSystem::SynchroMode
LowLevelVisionServer::SynchronizationMode()
  throw(CORBA::SystemException)
{
  cout << "LowLevelVisionServer::SynchronizationMode(): " <<m_TypeOfSynchro<<endl;
  return  m_TypeOfSynchro;
}


void
LowLevelVisionServer::SetSynchronizationMode(LowLevelVisionSystem::SynchroMode aSynchronizationMode)
  throw(CORBA::SystemException)
{
  m_TypeOfSynchro = aSynchronizationMode;
}

CORBA::Long
LowLevelVisionServer::SetImage(const ColorBuffer & cbuf, CORBA::Long SemanticCameraID,
			       CORBA::Long aWidth, CORBA::Long aHeight)
  throw(CORBA::SystemException)
{
  PixelEncode_t aPixelEncoding[4];
  int dec1, dec2;

  ODEBUG2( "LowLevelVisionServer::SetImage : beginning " << aWidth << " " << aHeight );

  /* Set the image, currently only used by the simulator */
  if ((m_Width[SemanticCameraID]!=aWidth) || (m_Height[SemanticCameraID]!=aHeight))
    SetImagesGrabbedSize(SemanticCameraID, aWidth,aHeight);

  ODEBUG("LowLevelVisionServer::SetImage : before copying the image " );

  dec1 = aWidth*aHeight;
  dec2 = dec1 *2;
  for(int j=0;j<aHeight;j++)

    {
      int index = j*aWidth;

      for(int i=0;i<aWidth;i++)
	{
	  aPixelEncoding[0].aInt = cbuf[index+i];
	  aPixelEncoding[1].aInt = cbuf[dec1+index+i];
	  aPixelEncoding[2].aInt = cbuf[dec2+index+i];
	  m_BinaryImages[0][index+i] = (unsigned char)
	    (aPixelEncoding[0].Images[0] *.299 + aPixelEncoding[0].Images[1] *.587 + aPixelEncoding[0].Images[2] *.114 );
	  m_BinaryImages[1][index+i] = (unsigned char)
	    (aPixelEncoding[1].Images[0] *.299 + aPixelEncoding[1].Images[1] *.587 + aPixelEncoding[1].Images[2] *.114 );
	  m_BinaryImages[2][index+i] = (unsigned char)
	    (aPixelEncoding[2].Images[0] *.299 + aPixelEncoding[2].Images[1] *.587 + aPixelEncoding[2].Images[2] *.114 );
	}
    }
  ODEBUG2( "LowLevelVisionServer::SetImage : before ApplyingProcess " );

  ApplyingProcess();

  ODEBUG2( "LowLevelVisionServer::SetImage : after ApplyingProcess " );

  return 0;
}

CORBA::Long
LowLevelVisionServer::CalibLoadSize(const char *file, long int *width, long int *height)
{
  FILE *fp;

  if ((fp = fopen(file,"r")) == NULL)
    return(-1);
  else
    fscanf(fp,"%ld %ld\n",width,height);
  fclose(fp);
  return(0);
}

#if (LLVS_HAVE_VVV>0)
CORBA::Long
LowLevelVisionServer::CalibLoadPinholeParameter(
						CONST char *file,
						EPBM_PinHoleParameter *pin)
{
  FILE *fp;
  int i;

  if((fp=fopen(file, "r")) == NULL)
    {
      fprintf(stderr,"vvvstereo_loadcalib: CANNOT OPEN '%s'",file);
      return -1;
    }
  else
    {

      for(i=0; i<3; i++)
	{
	  fscanf(fp,"%lf %lf %lf %lf\n",
		 &(pin->H[i][0]),&(pin->H[i][1]),&(pin->H[i][2]),&(pin->H[i][3]));
	}

    }
  if(fclose(fp) != 0)
    {
      fprintf(stderr,"vvvstereo_loadcalib: CANNOT CLOSE '%s'",file);
      return -1;
    }

  pin->f = SCM_PINHOLE_F;
  pin->m = 0;

  return 0 ;
}
#endif

CORBA::Long LowLevelVisionServer::LoadCalibrationInformation()
{

  char cdir[1024]="/usr/local/VVV/var/calib/ct3001-0/";
  char pathname[512];

  /* load camera parameters */
  int i,lw,lh;
  for(i=0;i<3;i++)
    {

      if (m_CalibrationDirectory.size()!=0)
	{
	  bzero(cdir,1024);
	  sprintf(cdir,"%s",m_CalibrationDirectory.c_str());
	  if (m_Verbosity>1)
	    cerr << cdir << endl;
	}

      if (sprintf(pathname,"%s/Size.%d",cdir,i) < 0)
	{
	  if (m_Verbosity>1)
	    fprintf(stderr,"HEREH : vvvstereo_loadcalib: BAD NAME, \"%s\".",pathname);
	  continue;
	}

      if(CalibLoadSize((char *)pathname,&m_CalibrationWidth[i],&m_CalibrationHeight[i]) != 0)
	{
	  if (m_Verbosity>1)
	    fprintf(stderr,"CalibLoadSize: CANNOT OPEN FILE,'%s'.", pathname);
 	  continue;
	}

      if (sprintf(pathname,"%s/Calib.%d",cdir,i) < 0)
	{
	  if (m_Verbosity>1)
	    fprintf(stderr,"Path to calibration data invalid: \"%s\".",pathname);
	  continue;
	}
#if (LLVS_HAVE_VVV>0)
      CalibLoadPinholeParameter((char *)pathname,&m_PinHoleParameter[i]);
      /* Copy in m_sp */
      {
	for(int k=0;k<3;k++)
	  {
	    for(int l=0;l<4;l++)
	      {
		m_sp.H[i][k][l] = m_PinHoleParameter[i].H[k][l];
	      }
	  }
      }

      if(sprintf(pathname, "%s/Distortion.%d",cdir,i) < 0)
	{
	  if (m_Verbosity>1)
	    fprintf(stderr,"vvvstereo_loadcalib: BAD NAME, \"%s\".",pathname);
	  continue;
	}
      calib_load_distortion_parameter(pathname,&m_DistortionParameter[i],0);


      if (m_Verbosity>=3)
	{
	  cout << "a : " << m_DistortionParameter[i].a << endl;
	  cout << "b : " << m_DistortionParameter[i].b << endl;
	  cout << "cold : " << m_DistortionParameter[i].cold << endl;
	  cout << "rowd : " << m_DistortionParameter[i].rowd << endl;
	}
      if (1)
	{
	  static MAT_Vector t=0;
	  static MAT_Matrix R=0;
	  double f, aspectratio, theta ;
	  double iccr[2];
	  SCM_PARAMETER * p_sp = &m_sp;

	  if ((t==0) && (R==0))
	    {
	      t = new DOUBLE64[4];
	      R = new DOUBLE64 *[4];
	      for(int j =0;j<4;j++)
		R[j] = new DOUBLE64[4];
	    }

	  scm_CalcCameraParameter(p_sp, i,
				  t, (DOUBLE64**)R, &f,
				  &aspectratio, & theta, iccr,0);
	  if (i==2)
	    {
	      delete t;
	      for(int j=0;j<3;j++)
		delete R[j];
	      delete R;
	    }

	  float aFocal = f;
	  float aScale[2] = {0.0,0.0};
	  float aSkewFactor = 0.0;
	  float anImageCenter[2];

	  string Names[4] = {"LEFT","RIGHT","UP","WIDE"};
	  anImageCenter[0] = iccr[0];
	  anImageCenter[1] = iccr[1];
	  m_Cameras[i]->SetIntrinsicParameters(f,aScale, aSkewFactor, anImageCenter);
	  m_Cameras[i]->SetName(Names[i]);
	  m_Cameras[i]->SetIdentifier(i);
	  m_Cameras[i]->SetCameraParameter(m_Width,m_Height,m_CalibrationWidth[i],m_CalibrationHeight[i]);
	  m_Cameras[i]->SetCameraProjectiveParameters(p_sp,i);

	}
#endif

    }

#if (LLVS_HAVE_VVV>0)
  for(i=0;i<4;i++)
    {

      lw = m_epbm[i].Width;
      lh = m_epbm[i].Height;
      if (i!=3)
	pebutil_loadcalib(&(m_epbm[i]),
			  i,cdir,0);

      m_epbm[i].Width = lw;
      m_epbm[i].Height =lh;
      m_epbm[i].Label |= EPBM_HAVE_PINHOLEPARAMETER_MASK;
      if (m_depth[i]==1)
	m_epbm[i].Magic2 = EPBM_BINARY_GRAY;
      else
	m_epbm[i].Magic2 = EPBM_BINARY_COLOR;
    }
  for(i=0;i<4;i++)
    {
      epbm_dup_header(&m_CorrectedImages[i],&m_epbm[i],0);
      if (m_depth[i]==1)
	m_CorrectedImages[i].Magic2 = EPBM_BINARY_GRAY;
      else
	m_CorrectedImages[i].Magic2 = EPBM_BINARY_COLOR;

      epbm_dup_header(&m_epbm_distorted[i],&m_epbm[i],0);
      if (m_depth[i]==1)
	m_epbm_distorted[i].Magic2 = EPBM_BINARY_GRAY;
      else
	m_epbm_distorted[i].Magic2 = EPBM_BINARY_COLOR;

      m_CorrectedImages[i].Label |= EPBM_HAVE_PINHOLEPARAMETER_MASK;
      m_epbm_distorted[i].Label |= EPBM_HAVE_PINHOLEPARAMETER_MASK;
    }

#endif
  ReadRbtCalib();


  return 0;
}


void LowLevelVisionServer::ReadRbtCalib()
{
  ifstream aifstream;
  string aFileName;
  aFileName = m_RobotVisionCalibrationDirectory;
  aFileName +="/";
  aFileName +="headTorg.hmat";


  /* Now read the information linking the vision reference frame to
     the head reference frame */
  aifstream.open((char *)aFileName.c_str(),ifstream::in);

  if (aifstream.is_open())
    {
      for(int i=0;i<16;i++)
	aifstream >> m_headTorg[i];
      aifstream.close();
      if (m_Verbosity>=3)
	cout << " Able to read " << aFileName << endl;
    }
  else
    {
      if (m_Verbosity>=3)
	cerr << "Unable to open " << aFileName << endl;
    }
  ODEBUG("headTorg: " << m_headTorg[0] << " "
	 << m_headTorg[1] << " "
	 << m_headTorg[2] << " "
	 << m_headTorg[3] << " " << endl
	 << m_headTorg[4] << " "
	 << m_headTorg[5] << " "
	 << m_headTorg[6] << " "
	 << m_headTorg[7] << " " << endl
	 << m_headTorg[8] << " "
	 << m_headTorg[9] << " "
	 << m_headTorg[10] << " "
	 << m_headTorg[11] << " "<< endl
	 << m_headTorg[12] << " "
	 << m_headTorg[13] << " "
	 << m_headTorg[14] << " "
	 << m_headTorg[15] << " "
	 );
}

#if (LLVS_HAVE_VVV>0)
CORBA::Long
LowLevelVisionServer::scm_ConvertImageLocal(CONST SCM_PARAMETER *sp, CONST EPBM *I, EPBM *O,
					    int OriginalWidth,int OriginalHeight)
{
  double scr[2], cr[2];
  double frow, fcol;
  int scm_row, scm_col, nrow, ncol;
  int i, j, chkl, chkr, lcv= SCM_VERIFY;
  double fromOrig2CurX,fromOrig2CurY;
  double fromCur2OrigX,fromCur2OrigY;
  double ImageNR[2];


  /*  fprintf(stderr,"First : (%d %d %d %d)\n",
      OriginalWidth,OriginalHeight,
      I->Width,I->Height);
  */
  for (chkl = 0, chkr = 0, i = 0; i < 3; i++) {
    for (j = 0; j < 4; j++) {
      if (O->PinHoleParameter->H[i][j] == sp->H[0][i][j])
	chkl++;
      if (O->PinHoleParameter->H[i][j] == sp->H[1][i][j])
	chkr++;
    }
  }
  if (chkl == 12)
    {
      lcv = SCM_LEFT;
    }
  else if (chkr == 12)
    {
      lcv = SCM_RIGHT;
    }
  else {
    fprintf(stderr,"scm_ConvertImage: EPBM DATA IS NOT GOOD");
    return -1;
  }

  /* From currently corrected image to original corrected image. */
  fromCur2OrigX = (double)OriginalWidth/(double)I->Width;
  fromCur2OrigY = (double)OriginalHeight/(double)I->Height;

  /* Compute the "virtual" size of the original none corrected image */
  scr[0] = (double)OriginalWidth;
  scr[1] = (double)OriginalHeight;

  scm_SCMcr2cr(sp, lcv, scr, ImageNR);

  /* From original none corrected image to currently non corrected image */
  fromOrig2CurX = (double)I->Width/(double)OriginalWidth;
  fromOrig2CurY = (double)I->Height/(double)OriginalHeight;

#ifdef OBDEBUG
  if (lcv==SCM_LEFT)
    fprintf(stderr,"scm_ConvertImage: SCM_LEFT\n");
  else if (lcv==SCM_RIGHT)
    fprintf(stderr,"scm_ConvertImage: SCM_RIGHT\n");
#endif

  if (I->Magic2 == EPBM_BINARY_GRAY) {
    for (scm_row = 0; scm_row < O->Height; scm_row++) {
      for (scm_col = 0; scm_col < O->Width; scm_col++) {

	/*	fprintf(stderr,"(%d %d %d %d)\n",
		OriginalWidth,OriginalHeight,
		I->Width,I->Height);

		fprintf(stderr,"{%f %f %f %f}\n",
		fromCur2OrigX,fromCur2OrigY,
		fromOrig2CurX,fromOrig2CurY);
	*/
	scr[0] = fromCur2OrigX*(double)scm_col;
	scr[1] = fromCur2OrigY*(double)scm_row;
	scm_SCMcr2cr(sp, lcv, scr, cr);
	nrow = (int)floor(fromOrig2CurY*cr[1]);
	frow = fromOrig2CurY*cr[1] - (double)nrow;
	ncol = (int)floor(fromOrig2CurX*cr[0]);
	fcol = fromOrig2CurX*cr[0] - (double)ncol;


	if ((nrow < 0) || (I->Height - 1 <= nrow) ||
	    (ncol < 0) || (I->Width - 1 <= ncol))
	  epbm_uc_getpixel(O, scm_row, scm_col) = 0;
	else
	  {
	    epbm_uc_getpixel(O, scm_row, scm_col) =
	      (unsigned char)
	      (epbm_uc_getpixel(I, nrow, ncol) * (1.0 - frow) * (1.0 - fcol) +
	       epbm_uc_getpixel(I, nrow + 1, ncol) * frow * (1.0 - fcol) +
	       epbm_uc_getpixel(I, nrow, ncol + 1) * (1.0 - frow) * fcol +
	       epbm_uc_getpixel(I, nrow + 1, ncol + 1) * frow * fcol +
	       0.5);
	    /*
	      fprintf(stderr,"%d %d %f %f %f %f %d %d %f %f %f %f %d %d\n",
	      scm_col,scm_row,scr[0],scr[1],cr[0],cr[1],
	      ncol, nrow, ImageNR[0], ImageNR[1], fromOrig2CurX, fromOrig2CurY, OriginalWidth, OriginalHeight);
	    */

	  }
      }
    }
  }
  else if (I->Magic2 == EPBM_BINARY_COLOR) {
    for (scm_row = 0; scm_row < O->Height; scm_row++) {
      for (scm_col = 0; scm_col < O->Width; scm_col++) {
	scr[0] = fromCur2OrigX*(double)scm_col;
	scr[1] = fromCur2OrigY*(double)scm_row;
	scm_SCMcr2cr(sp, lcv, scr, cr);
	nrow = (int)floor(fromOrig2CurY*cr[1]);
	frow = fromOrig2CurY*cr[1] - (double)nrow;
	ncol = (int)floor(fromOrig2CurX*cr[0]);
	fcol = fromOrig2CurX*cr[0] - (double)ncol;
	if ((nrow < 0) || (I->Height - 1 <= nrow) ||
	    (ncol < 0) || (I->Width - 1 <= ncol))
	  epbm_uc_cgetpixel(O, EPBM_RED, scm_row, scm_col) =
	    epbm_uc_cgetpixel(O, EPBM_GREEN, scm_row, scm_col) =
	    epbm_uc_cgetpixel(O, EPBM_BLUE, scm_row, scm_col) = 0;
	else {
	  epbm_uc_cgetpixel(O, EPBM_RED, scm_row, scm_col) =
	    (unsigned char)
	    (epbm_uc_cgetpixel(I, EPBM_RED, nrow, ncol) *
	     (1.0 - frow) * (1.0 - fcol) +
	     epbm_uc_cgetpixel(I, EPBM_RED, nrow + 1, ncol) *
	     frow * (1.0 - fcol) +
	     epbm_uc_cgetpixel(I, EPBM_RED, nrow, ncol + 1) *
	     (1.0 - frow) * fcol +
	     epbm_uc_cgetpixel(I, EPBM_RED, nrow + 1, ncol + 1) *
	     frow * fcol +
	     0.5);
	  epbm_uc_cgetpixel(O, EPBM_GREEN, scm_row, scm_col) =
	    (unsigned char)
	    (epbm_uc_cgetpixel(I, EPBM_GREEN, nrow, ncol) *
	     (1.0 - frow) * (1.0 - fcol) +
	     epbm_uc_cgetpixel(I, EPBM_GREEN, nrow + 1, ncol) *
	     frow * (1.0 - fcol) +
	     epbm_uc_cgetpixel(I, EPBM_GREEN, nrow, ncol + 1) *
	     (1.0 - frow) * fcol +
	     epbm_uc_cgetpixel(I, EPBM_GREEN, nrow + 1, ncol + 1) *
	     frow * fcol +
	     0.5);
	  epbm_uc_cgetpixel(O, EPBM_BLUE, scm_row, scm_col) =
	    (unsigned char)
	    (epbm_uc_cgetpixel(I, EPBM_BLUE, nrow, ncol) *
	     (1.0 - frow) * (1.0 - fcol) +
	     epbm_uc_cgetpixel(I, EPBM_BLUE, nrow + 1, ncol) *
	     frow * (1.0 - fcol) +
	     epbm_uc_cgetpixel(I, EPBM_BLUE, nrow, ncol + 1) *
	     (1.0 - frow) * fcol +
	     epbm_uc_cgetpixel(I, EPBM_BLUE, nrow + 1, ncol + 1) *
	     frow * fcol +
	     0.5);
	}
      }
    }
  }

  return 0;
}
#endif

//////////////////////////////////////////////////////////////////////

void
LowLevelVisionServer::CreateNameContext()
{
  if (m_Verbosity>=2)
    cerr << "bindObjectToName: step 1\n";
  try
    {
      // Obtain a reference to the root context of the Name service:
      CORBA::Object_var obj;
      if (CORBA::is_nil(m_orb))
	{
	  if (m_Verbosity>=1)
	    cerr << "bindObjectToName : step 1.0.5" << endl;
	}

      obj = m_orb->resolve_initial_references("NameService");
      if (m_Verbosity>=2)
	cerr << "bindObjectToName: step 1.1\n";
      if (CORBA::is_nil(obj))
	{
	  cerr << "bindObjectToName : step 1.1.5" << endl;
	}
      // Narrow the reference returned.
      m_cxt = CosNaming::NamingContext::_narrow(obj);
      if (m_Verbosity>=2)
	cerr << "bindObjectToName: step 1.2\n";
      if( CORBA::is_nil(m_cxt) )
	{
	  if (m_Verbosity>=1)
	    cerr << "Failed to narrow the root naming context." << endl;
	  return ;
	}
    } catch(CosNaming::NamingContext::NotFound ex)
    {
      cerr << "Not Found" << endl;
      return;
    }
  catch(CORBA::ORB::InvalidName& ex)
    {
      // This should not happen!
      if (m_Verbosity>=1)
	cerr << "Service required is invalid [does not exist]." << endl;
      return ;
    }
  catch(CORBA::SystemException&)
    {
      if (m_Verbosity>=1)
	cerr << "Caught a CORBA::SystemException while using the naming service, "
	     << "in bindObjectToName step 1"
	     << endl;
      return ;
    }
  catch(...)
    {
      cerr << "Caught something" << endl;
      return;
    }
  if (m_Verbosity>=2)
    {
      cerr << "bindObjectToName: step 2\n";
      cerr << "Vision server phase 1" << endl;
    }
  // Bind a context called "test" to the root context:
}

CORBA::Boolean
LowLevelVisionServer::bindObjectToName(CORBA::Object_ptr objref)

{
  if (m_Verbosity>=2)
    cerr << "Vision server phase 3" << endl;
  // Bind objref with name Echo to the testContext:
  CosNaming::Name objectName;
  objectName.length(1);
  objectName[0].id   = CORBA::string_dup("LowLevelVisionSystem");   // string copied
  objectName[0].kind = CORBA::string_dup("VisionServer"); // string copied

  try
    {
      m_cxt->bind(objectName, objref);
    }
  catch(CosNaming::NamingContext::AlreadyBound& ex)
    //  catch(...)
    {
      m_cxt->rebind(objectName, objref);
    }
  // Note: Using rebind() will overwrite any Object previously bound
  //       to /test/Echo with obj.
  //       Alternatively, bind() can be used, which will raise a
  //       CosNaming::NamingContext::AlreadyBound exception if the name
  //       supplied is already bound to an object.

  // Amendment: When using OrbixNames, it is necessary to first try bind
  // and then rebind, as rebind on it's own will throw a NotFoundexception if
  // the Name has not already been bound. [This is incorrect behaviour -
  // it should just bind].

  catch(CORBA::COMM_FAILURE& ex)
    {
      if (m_Verbosity>=1)
	cerr << "Caught system exception COMM_FAILURE -- unable to contact the "
	     << "naming service." << endl;
      return 0;
    }
  catch(CORBA::SystemException&)
    {
      if (m_Verbosity>=1)
	cerr << "Caught a CORBA::SystemException while using the naming service."
	     << endl;
      return 0;
    }
  catch(...)
    {
      cerr<< "Pb not identify" << endl;
      return 0;
    }
  if (m_Verbosity>=2)
    cerr << "Vision server phase 4" << endl;
  return 1;
}

CORBA::Object_ptr LowLevelVisionServer::getObjectReference(string ServerID, string ServerKind)
{
  CosNaming::NamingContext_var rootContext;

  try
    {
      // Obtain a reference to the root context of the Name service:
      CORBA::Object_var obj;
      obj = m_orb->resolve_initial_references("NameService");

      // Narrow the reference returned.
      rootContext = CosNaming::NamingContext::_narrow(obj);


      if( CORBA::is_nil(rootContext) )
	{
	  cerr << "Failed to narrow the root naming context." << endl;
	  return CORBA::Object::_nil();
	}
    }

  catch(CORBA::ORB::InvalidName& ex) {
    // This should not happen!
    cerr << "Service required is invalid [does not exist]." << endl;
    return CORBA::Object::_nil();
  }

  catch(...)
    {
      cout << "Unknown Error" << endl;
      return CORBA::Object::_nil();
    }
  // Create a name object, containing the name test/context:
  CosNaming::Name name;
  name.length(1);

#if 0
  name[0].id   = (const char*) "LowLevelVisionSystem";       // string copied
  name[0].kind = (const char*) "VisionServer"; // string copied
#else
  name[0].id   = ServerID.c_str();
  name[0].kind = ServerKind.c_str();
#endif

  /*  name[1].id = (const char*) "Echo";
      name[1].kind = (const char*) "Object";*/

  // Note on kind: The kind field is used to indicate the type
  // of the object. This is to avoid conventions such as that used
  // by files (name.type -- e.g. test.ps = postscript etc.)

  try {
    // Resolve the name to an object reference.
    return rootContext->resolve(name);
  }
  catch(CosNaming::NamingContext::NotFound& ex) {
    // This exception is thrown if any of the components of the
    // path [contexts or the object] aren't found:
    cerr << "Context not found." << endl;
  }
  catch(CORBA::COMM_FAILURE& ex) {
    cerr << "Caught system exception COMM_FAILURE -- unable to contact the "
         << "naming service." << endl;
  }
  catch(CORBA::SystemException&) {
    cerr << "Caught a CORBA::SystemException while using the naming service."
	 << endl;
  }
  cout << "Here...failed to find the Server " << ServerID << endl;
  return CORBA::Object::_nil();
}

CORBA::Object_ptr LowLevelVisionServer::getObjectReference(vector<string> & ServerID, vector<string> & ServerKind)
{
  CosNaming::NamingContext_var rootContext;

  try
    {
      // Obtain a reference to the root context of the Name service:
      CORBA::Object_var obj;
      obj = m_orb->resolve_initial_references("NameService");

      // Narrow the reference returned.
      rootContext = CosNaming::NamingContext::_narrow(obj);


      if( CORBA::is_nil(rootContext) )
	{
	  cerr << "Failed to narrow the root naming context." << endl;
	  return CORBA::Object::_nil();
	}
    }

  catch(CORBA::ORB::InvalidName& ex) {
    // This should not happen!
    cerr << "Service required is invalid [does not exist]." << endl;
    return CORBA::Object::_nil();
  }

  catch(...)
    {
      cout << "Unknown Error" << endl;
      return CORBA::Object::_nil();
    }
  // Create a name object, containing the name test/context:
  CosNaming::Name name;
  name.length(ServerID.size());

#if 0
  name[0].id   = (const char*) "LowLevelVisionSystem";       // string copied
  name[0].kind = (const char*) "VisionServer"; // string copied
#else
  for(unsigned int i=0;i<ServerID.size();i++)
    {
      name[i].id   = ServerID[i].c_str();
      name[i].kind = ServerKind[i].c_str();
      cout << "name+kind: " << name[i].id << "." << name[i].kind<< endl;
    }
#endif

  /*  name[1].id = (const char*) "Echo";
      name[1].kind = (const char*) "Object";*/

  // Note on kind: The kind field is used to indicate the type
  // of the object. This is to avoid conventions such as that used
  // by files (name.type -- e.g. test.ps = postscript etc.)

  try {
    // Resolve the name to an object reference.
    return rootContext->resolve(name);
  }
  catch(CosNaming::NamingContext::NotFound& ex) {
    // This exception is thrown if any of the components of the
    // path [contexts or the object] aren't found:
    cerr << "Context not found." << endl;
  }
  catch(CORBA::COMM_FAILURE& ex) {
    cerr << "Caught system exception COMM_FAILURE -- unable to contact the "
         << "naming service." << endl;
  }
  catch(CORBA::SystemException&) {
    cerr << "Caught a CORBA::SystemException while using the naming service."
	 << endl;
  }
  cout << "Here...failed to find the Server " << ServerID[0] << endl;
  return CORBA::Object::_nil();
}

CORBA::Long LowLevelVisionServer::SetVerboseMode(CORBA::Long VerboseMode)
{
  m_Verbosity = VerboseMode;
  return 0;
}

CORBA::Long LowLevelVisionServer::GetVerboseMode()
{
  return m_Verbosity;
}


CORBA::Long LowLevelVisionServer::SetCalibrationDirectory(string aCalibrationDirectory)
{
  m_CalibrationDirectory = aCalibrationDirectory;
  ODEBUG("LowLevelVisionServer::SetCalibrationDirectory(string aCalibrationDirectory)"
	 << m_CalibrationDirectory.c_str());
  return 0;
}

string LowLevelVisionServer::GetCalibrationDirectory()
{
  string aCalibDir = m_CalibrationDirectory;
  return aCalibDir;
}

/* Get the image size */
void LowLevelVisionServer::GetImageSizes(vector<ImageSize> &Sizes)
{
  for(unsigned int i=0;i<m_ImagesInputMethod->GetNumberOfCameras();i++)
    {
      Sizes[i].Width  = m_Width[i];
      Sizes[i].Height = m_Height[i];
    }
}

/* Get one image size */
void LowLevelVisionServer::GetImageSize(int Size[2],unsigned int CameraID)
{
  Size[0] = -1;
  Size[1] = -1;
  if ((CameraID<0) || (CameraID>m_ImagesInputMethod->GetNumberOfCameras()))
    {
      Size[0] = m_Width[CameraID];
      Size[1] = m_Height[CameraID];
    }

}

/* Get the direct access to the image memory */
void LowLevelVisionServer::GetImageMemory(vector<unsigned char *> &BinaryImages)
{
  for(unsigned int i=0;i<m_BinaryImages.size();i++)
    BinaryImages[i] = m_BinaryImages[i];

}

/* Get the direct access to one image memory */
unsigned char * LowLevelVisionServer::GetImageMemory(unsigned int CameraID)
{
  if ((CameraID<0) || (CameraID>=m_BinaryImages.size()))
    return 0 ;

  return m_BinaryImages[CameraID];

}

/* Returns the related camera */
Camera_impl * LowLevelVisionServer::GetCamera(unsigned int CameraNb)
{
  if ((CameraNb<0) ||
      (CameraNb>=m_Cameras.size()))
    return 0;

  return m_Cameras[CameraNb];
}

#if (LLVS_HAVE_VVV>0)
/* Returns a link to the disparity process */
HRP2DisparityProcess * LowLevelVisionServer::GetDisparityProcess()
{
  return m_DP;
}
#endif

/* Check the format, and put the appropriate one */
void LowLevelVisionServer::CheckImageFormat(char *&Format)
{

  if (strcmp(Format,m_ImageFormat.c_str()))
    {
      if (m_Verbosity>1)
	cerr << "Only GrayScaleChar format is supported right now" << endl;

      if (strlen(Format) < strlen(m_ImageFormat.c_str()))
	{
	  CORBA::string_free(Format);
	  Format = CORBA::string_dup(m_ImageFormat.c_str());
	}
      else
	strcpy(Format,m_ImageFormat.c_str());

    }

}


/* Get an image */
CORBA::Long LowLevelVisionServer::getImage(CORBA::Long SemanticCamera, ImageData_out anImage, char *& Format)
  throw(CORBA::SystemException)
{
  ImageData_var an2Image = new ImageData;
  int i,j, index =0 ;

  ODEBUG("getImage :" << SemanticCamera << " " << string(Format) << " " <<
	 m_BinaryImages.size());
  if ((SemanticCamera<0) || ((unsigned int)SemanticCamera>=m_BinaryImages.size()) )
    {
      ODEBUG("No image to transmit");
      an2Image->format=GRAY;
      an2Image->width=0;
      an2Image->height=0;
      an2Image->octetData.length(0);
      an2Image->longData.length(0);
      anImage = an2Image._retn();
      return -1;
    }

  CheckImageFormat(Format);

  CORBA::Long Size = m_Height[SemanticCamera]*m_Width[SemanticCamera]*m_depth[SemanticCamera];
  if (m_depth[SemanticCamera]==3)
    an2Image->format=ARGB;
  else if (m_depth[SemanticCamera]==1)
    an2Image->format=GRAY;

  an2Image->width = m_Width[SemanticCamera];
  an2Image->height = m_Height[SemanticCamera];

  ODEBUG("Size: " << Size);
  an2Image->octetData.length(Size);
  an2Image->longData.length(2);


  unsigned char *pt = m_BinaryImages[SemanticCamera];
  for(j=0;j<(int)(m_Height[SemanticCamera]*m_Width[SemanticCamera]*m_depth[SemanticCamera]);j++)
    an2Image->octetData[j] = *pt++;

  an2Image->longData[0] = (long)m_timestamps[SemanticCamera];
  an2Image->longData[1] = (long)(m_timestamps[SemanticCamera]-an2Image->longData[0])*1e6;

  anImage = an2Image._retn();

  ODEBUG("m_depth["<<SemanticCamera << "]="<<(int)m_depth[SemanticCamera]<< endl);
  return Size;
}

/* Get a rectified image */
CORBA::Long LowLevelVisionServer::getRectifiedImage(CORBA::Long SemanticCamera, ImageData_out anImage, char *& Format)
  throw(CORBA::SystemException)
{
  ImageData_var an2Image = new ImageData;
  int i,j, index =0 ;

  cout << "SemanticCamera:" << SemanticCamera << std::endl;
  if ((SemanticCamera<0) || ((unsigned int) SemanticCamera>m_ImagesInputMethod->GetNumberOfCameras()))
    {
      an2Image->octetData.length(0);
      an2Image->longData.length(0);
      an2Image->floatData.length(0);
      an2Image->width=320;
      an2Image->height=240;
      anImage = an2Image._retn();
      return -1;
    }

  CheckImageFormat(Format);
#if (LLVS_HAVE_VVV>0)
  an2Image->octetData.length(m_Height[SemanticCamera] * m_Width[SemanticCamera]*m_depth[SemanticCamera]);

  for(j=0;j<(int)(m_Height[SemanticCamera]*m_Width[SemanticCamera]*m_depth[SemanticCamera]);j++)
    an2Image->octetData[j] = ((unsigned char *)m_CorrectedImages[SemanticCamera].Image)[j];
  //an2Image->octetData[j] = ((unsigned char *)m_epbm_distorted[SemanticCamera].Image)[j];
#endif

#if (LLVS_HAVE_VISP>0)

  an2Image->octetData.length(320*240);
  an2Image->floatData.length(0);
  an2Image->width=320;
  an2Image->height=240;
  an2Image->longData.length(2);
  an2Image->format=GRAY;//PixelFormat::GRAY;
  an2Image->longData[0] =(long) m_timestamps[SemanticCamera];
  an2Image->longData[1] = (long)(m_timestamps[SemanticCamera]-an2Image->longData[0])*1e6;


  unsigned char *pt =m_Widecam_image_undistorded->bitmap;

  for(j=0;j<(int)(320*240);j++)
    an2Image->octetData[j] = *pt++;

#ifdef _DEBUG_MODE_ON_
  {
    ofstream aofstream;

    for(unsigned int l=0;l<an2Image->octetData.length();l++)
      {
	aofstream << an2Image->octetData[l];
      }

    aofstream.close();
  }
#endif

#endif

  anImage = an2Image._retn();

  CORBA::Long lr=320*240;
  return lr;
}

/* Get the edge image */
CORBA::Long LowLevelVisionServer::getEdgeImage(CORBA::Long SemanticCamera, ImageData_out anImage, char *&Format)
  throw(CORBA::SystemException)
{
  ImageData_var an2Image = new ImageData;
  int i,j, index =0 ;


  if ((SemanticCamera<0) || ((unsigned int)SemanticCamera>m_ImagesInputMethod->GetNumberOfCameras())
#if (LLVS_HAVE_VVV>0)
      || (m_EdgeDetection==0)
#endif
      )
    {
      an2Image->longData.length(0);
      anImage = an2Image._retn();
      return -1;
    }

  an2Image->longData.length(m_Height[SemanticCamera] * m_Width[SemanticCamera]);
#if (LLVS_HAVE_VVV>0)
  for(j=0;j<m_Height[SemanticCamera]*m_Width[SemanticCamera];j++)
    an2Image->longData[j] = ((unsigned int *)m_EdgeDetection->m_Edge[SemanticCamera].Image)[j];
#endif

  anImage = an2Image._retn();
  return m_Height[SemanticCamera]*m_Width[SemanticCamera];

}

/* Check the format, and put the appropriate one */
void LowLevelVisionServer::CheckRangeMapFormat(char *&Format)
{
  const char * CurrentFormat = "XYZGrayScaleChar";


  if (!strcmp(Format,"XYZGrayScaleImageRange"))
    {
      if (m_Verbosity>1)
	cerr << "XYZGrayScaleImageRange supported" << endl;

    }
  else if (!strcmp(Format,"XYZColorImageRange"))
    {
      if (m_Verbosity>1)
	cerr << "XYZColorImageRange supported" << endl;

    }
  else if (!strcmp(Format,"XYZColorChar"))
    {
      if (m_Verbosity>1)
	cerr << "XYZColorChar supported" << endl;

    }
  else if (!strcmp(Format,"PointsAndBBError"))
    {
      if (m_Verbosity>1)
	cerr << "PointsAndBBError supported" << endl;
    }
  else if (!strcmp(Format,"PointsAndHZError"))
    {
      if (m_Verbosity>1)
	cerr << "PointsAndBBError supported" << endl;
    }
  else if (strcmp(Format,CurrentFormat))
    {
      if (m_Verbosity>1)
	cerr << "Only "
	     << "XYZGrayScaleChar and " << endl
	     << "XYZColorImageRange and " << endl
	     << "XYZGrayScaleImageRange and " << endl
	     << "PointsAndBBError" << endl
	     << "PointsAndHZError" << endl
	     << "formats are supported right now" << endl
	     << "not " << Format << endl;

      if (strlen(Format) < strlen(CurrentFormat))
	{
	  CORBA::string_free(Format);
	  Format = CORBA::string_dup(CurrentFormat);
	}
      else
	strcpy(Format,CurrentFormat);

    }

}


/* Get a range map. */
CORBA::Long LowLevelVisionServer::getRangeMap(RangeMap_out aRangeMapOut, char *&Format)
  throw(CORBA::SystemException)
{
  int NbOfPoints=0;
  RangeMap_var aRangeMapVar;
  aRangeMapVar = new RangeMap;

#if (LLVS_HAVE_VVV>0)

  RANGE arange;

  int i,j,k=0;
  int PointSize=4;
  if (m_DP==0)
    return -1;

  CheckRangeMapFormat(Format);
  arange = m_DP->GetRangeData();

  if (arange.PixelList==0)
    return -1;


  if (!strcmp(Format,"XYZGrayScaleImageRange") ||
      (!strcmp(Format,"XYZColorImageRange")))
    {
      NbOfPoints = arange.MapSize;

      bool bColor = false;
      if (!strcmp(Format,"XYZColorImageRange"))
	{
	  bColor=true;
	  PointSize=6;
	}
      // Allocate the memory place
      aRangeMapVar->length(NbOfPoints*PointSize);

      for(j=0;j<arange.Height;j++)
	for(i=0;i<arange.Width;i++)
	  {
	    PixelData *aPD;
	    aPD = arange.Map[j * arange.Width + i];
	    if (aPD!=0)
	      {
		aRangeMapVar[k++] = (float)aPD->Dot[0];
		aRangeMapVar[k++] = (float)aPD->Dot[1];
		aRangeMapVar[k++] = (float)aPD->Dot[2];
		aRangeMapVar[k++] = (float)aPD->Color[0];
		if (bColor)
		  {
		    aRangeMapVar[k++] = (float)aPD->Color[1];
		    aRangeMapVar[k++] = (float)aPD->Color[2];
		  }
	      }
	    else
	      {
		aRangeMapVar[k++] = -1.0;
		aRangeMapVar[k++] = -1.0;
		aRangeMapVar[k++] = -1.0;
		aRangeMapVar[k++] = -1.0;
		if (bColor)
		  {
		    aRangeMapVar[k++] = -1.0;
		    aRangeMapVar[k++] = -1.0;
		  }

	      }
	  }
    }
  else if (!strcmp(Format,"PointsAndBBError"))
    {
      int SubsampleIA = m_DP->GetSubsampleIA();
      float * BoundingBoxes=0;
      int l;
      BoundingBoxes = m_DP->GetBoundingBoxes();

      NbOfPoints = arange.PixelCount;

      // Allocate the memory place
      aRangeMapVar->length(NbOfPoints*7);

      for(j=0;j<NbOfPoints;j+=SubsampleIA)
	{
	  PixelData *aPD;
	  aPD = arange.PixelList + j;
	  if (aPD!=0)
	    {
	      float * atab = &BoundingBoxes[6*j];

	      aRangeMapVar[k++] = atab[0];
	      aRangeMapVar[k++] = atab[1];
	      aRangeMapVar[k++] = atab[2];
	      aRangeMapVar[k++] = (float)aPD->Color[0];

	      for(i=0;i<3;i++)
		{
		  aRangeMapVar[k++] = atab[3+i];
		}
	    }
	}
      NbOfPoints /= SubsampleIA;
    }
  else if (!strcmp(Format,"PointsAndHZError"))
    {
      ODEBUG("getRangeMap: Through PointsAndHZError ");
      int SubsampleHZ = m_DP->GetSubsampleHZ();
      float * BoundingBoxes=0;
      int l;
      BoundingBoxes = m_DP->GetBoundingBoxes();

      NbOfPoints = arange.PixelCount;

      // Allocate the memory place
      aRangeMapVar->length(NbOfPoints*13);

      for(j=0;j<NbOfPoints;j+=SubsampleHZ)
	{
	  PixelData *aPD;
	  aPD = arange.PixelList + j;
	  if (aPD!=0)
	    {
	      float * atab = &BoundingBoxes[12*j];

	      for(i=0;i<3;i++)
		aRangeMapVar[k++] = atab[i];

	      aRangeMapVar[k++] = (float)aPD->Color[0];

	      for(i=0;i<9;i++)
		{
		  aRangeMapVar[k++] = atab[i];
		}
	    }
	}
      NbOfPoints /= SubsampleHZ;
    }
  else if (!strcmp(Format,"XYZGrayScaleChar") ||
	   (!strcmp(Format,"XYZColorChar")))
    {
      NbOfPoints = arange.PixelCount;

      bool bColor = false;
      if (!strcmp(Format,"XYZColorChar"))
	{
	  bColor=true;
	  PointSize=6;
	}

      // Allocate the memory place
      aRangeMapVar->length(NbOfPoints*PointSize);

      for(j=0;j<NbOfPoints;j++)
	{
	  PixelData *aPD;
	  aPD = arange.PixelList + j;
	  if (aPD!=0)
	    {
	      aRangeMapVar[k++] = (float)aPD->Dot[0];
	      aRangeMapVar[k++] = (float)aPD->Dot[1];
	      aRangeMapVar[k++] = (float)aPD->Dot[2];
	      aRangeMapVar[k++] = (float)aPD->Color[0];
	      if (bColor)
		{
		  aRangeMapVar[k++] = (float)aPD->Color[1];
		  aRangeMapVar[k++] = (float)aPD->Color[2];
		}

	    }
	}
    }

#else
  aRangeMapVar->length(0);
#endif

  aRangeMapOut = aRangeMapVar._retn();
  return NbOfPoints;
}

/* Get image derivative */
CORBA::Long LowLevelVisionServer::getImageDerivative(CORBA::Long CameraID,
						     CORBA::Long aDerivativeID,
						     FloatBuffer_out ImageDerivative,
						     CORBA::Long_out Width,
						     CORBA::Long_out Height)
  throw(CORBA::SystemException)
{
#if (LLVS_HAVE_MMX>0)
  MMXFlow *aFlow=0;
  MM_F_32 *p_DerivativeImage=0;
#else
  void *aFlow=0;
#endif
  int index = 0;
  FloatBuffer_var ImageDerivativeVar;

  ODEBUG2("Ici 1");
  ImageDerivativeVar = new FloatBuffer;
  ODEBUG2("Ici 2");
  if (
#if (LLVS_HAVE_MMX>0)
      (m_OP==0) ||
#endif
      (CameraID==CAMERA_UP))
    {
      ImageDerivativeVar->length(0);
      ImageDerivative = ImageDerivativeVar._retn();
      return -1;
    }
  ODEBUG2("Ici 4");
#if (LLVS_HAVE_MMX>0)
  m_OP->GetFlowStructure(&aFlow, (int)CameraID, (int)
			 HRP2OpticalFlowProcess::FLOW_STRUCTURE_OPF);
#endif

  ODEBUG2("Ici 5");
  if (aFlow==0)
    {
      ImageDerivativeVar->length(1);
      ImageDerivative = ImageDerivativeVar._retn();
      return -1;
    }

#if (LLVS_HAVE_MMX>0)
  ODEBUG2("Ici 6");
  switch (aDerivativeID)
    {

    case LowLevelVisionSystem::IDT:
      Width = aFlow->Width-4; Height = aFlow->Height-4;
      break;
    case LowLevelVisionSystem::IDX:
      Width = aFlow->Width-4 ; Height = aFlow->Height-4;
      break;
    case LowLevelVisionSystem::IDY:
      Width = aFlow->Width-4  ; Height = aFlow->Height-4;
      break;
    case LowLevelVisionSystem::IDXDX:
      Width = aFlow->Width-4  ; Height = aFlow->Height-4;
      break;
    case LowLevelVisionSystem::IDXDY:
      Width = aFlow->Width-4  ; Height = aFlow->Height-4;
      break;
    case LowLevelVisionSystem::IDYDY:
      Width = aFlow->Width-4  ; Height = aFlow->Height-4;
      break;
    case LowLevelVisionSystem::IDXDX2:
      Width = aFlow->Width-8  ; Height = aFlow->Height-8;
      break;
    case LowLevelVisionSystem::IDXDY2:
      Width = aFlow->Width-8  ; Height = aFlow->Height-8;
      break;
    case LowLevelVisionSystem::IDYDY2:
      Width = aFlow->Width-8  ; Height = aFlow->Height-8;
      break;
    default:
      break;

    }


  ImageDerivativeVar->length(aFlow->Width*aFlow->Height);

  for(int j=0;j<Height;j++)
    {
      switch (aDerivativeID)
	{
	case LowLevelVisionSystem::IDT:
	  p_DerivativeImage = (MM_F_32 *)aFlow->dt->p[j].s;
	  break;
	case LowLevelVisionSystem::IDX:
	  p_DerivativeImage = (MM_F_32 *)aFlow->dx->p[j].s;
	  break;
	case LowLevelVisionSystem::IDY:
	  p_DerivativeImage = (MM_F_32 *)aFlow->dy->p[j].s;
	  break;
	case LowLevelVisionSystem::IDXDX:
	  p_DerivativeImage = (MM_F_32 *)aFlow->dxdx->p[j].s;
	  break;
	case LowLevelVisionSystem::IDXDY:
	  p_DerivativeImage = (MM_F_32 *)aFlow->dxdy->p[j].s;
	  break;
	case LowLevelVisionSystem::IDYDY:
	  p_DerivativeImage = (MM_F_32 *)aFlow->dydy->p[j].s;
	  break;
	case LowLevelVisionSystem::IDXDX2:
	  p_DerivativeImage = (MM_F_32 *)aFlow->dxdx2->p[j].s;
	  break;
	case LowLevelVisionSystem::IDXDY2:
	  p_DerivativeImage = (MM_F_32 *)aFlow->dxdy2->p[j].s;
	  break;
	case LowLevelVisionSystem::IDYDY2:
	  p_DerivativeImage = (MM_F_32 *)aFlow->dydy2->p[j].s;
	  break;
	default:
	  break;

	}

      for(int i=0;i<Width;i++)
	{

	  ImageDerivativeVar[index] = (float) p_DerivativeImage[index];
	  index++;
	}
    }
#endif
  ImageDerivative = ImageDerivativeVar._retn();
  return 0;
}

/* Get Optical Flow */
CORBA::Long LowLevelVisionServer::getOpticalFlow(CORBA::Long CameraID,
						 FloatBuffer_out OpticalFlow,
						 FloatBuffer_out Confidence,
						 CORBA::Long_out Width,
						 CORBA::Long_out Height)
  throw(CORBA::SystemException)
{
#if (LLVS_HAVE_MMX>0)
  MMXFlow *aFlow;
  MM_F_32 *p_DerivativeImage[4];
#endif
  int index = 0;
  FloatBuffer_var OpticalFlowVar;
  FloatBuffer_var ConfidenceVar;

  OpticalFlowVar = new FloatBuffer;
  ConfidenceVar = new FloatBuffer;

  if (
#if (LLVS_HAVE_MMX>0)
      (m_OP==0) ||
#endif
      (CameraID==CAMERA_UP))
    {
      OpticalFlowVar->length(0);
      ConfidenceVar->length(0);
      OpticalFlow = OpticalFlowVar._retn();
      Confidence = ConfidenceVar._retn();
      return -1;
    }

#if (LLVS_HAVE_MMX>0)
  m_OP->GetFlowStructure(&aFlow, (int)CameraID, (int)
			 HRP2OpticalFlowProcess::FLOW_STRUCTURE_OPF);

  /* Dimension of the optical flow . */
  Width = aFlow->Width-8;
  Height = aFlow->Height-8;
#else
  Width = 0;
  Height = 0;
#endif

  /* Memory allocation for the optical flow and the confidence flow.*/
  OpticalFlowVar->length(2*Width*Height);
  ConfidenceVar->length(2*Width*Height);

#if (LLVS_HAVE_MMX>0)
  for(int j=0;j<Height;j++)
    {
      p_DerivativeImage[0] = (MM_F_32 *)aFlow->V[0]->p[j].s;
      p_DerivativeImage[1] = (MM_F_32 *)aFlow->V[1]->p[j].s;
      p_DerivativeImage[2] = (MM_F_32 *)aFlow->L[0]->p[j].s;
      p_DerivativeImage[3] = (MM_F_32 *)aFlow->L[1]->p[j].s;

      for(int i=0;i<Width;i++)
	{
	  OpticalFlowVar[2*index] = p_DerivativeImage[0][index];
	  OpticalFlowVar[2*index+1] = p_DerivativeImage[1][index];

	  ConfidenceVar[2*index] = p_DerivativeImage[2][index];
	  ConfidenceVar[2*index+1] = p_DerivativeImage[3][index];
	  index++;
	}
    }
#endif
  OpticalFlow = OpticalFlowVar._retn();
  Confidence = ConfidenceVar._retn();
  return 0;
}

/* Get Harris detector */
CORBA::Long LowLevelVisionServer::getHarrisDetector(CORBA::Long CameraID,
						    FloatBuffer_out Harris,
						    CORBA::Long_out Width,
						    CORBA::Long_out Height
						    )
  throw(CORBA::SystemException)
{
#if (LLVS_HAVE_MMX>0)
  MMXFlow *aFlow;
  MM_F_32 *p_DerivativeImage;
#else
  void * aFlow =0;
#endif

  int index = 0;
  FloatBuffer_var HarrisVar;

  HarrisVar = new FloatBuffer;
  if (
#if (LLVS_HAVE_MMX>0)
      (m_OP==0) ||
#endif
      (CameraID==CAMERA_UP))
    {
      HarrisVar->length(0);
      Harris = HarrisVar._retn();
      return -1;
    }
#if (LLVS_HAVE_MMX>0)
  m_OP->GetFlowStructure(&aFlow, (int)CameraID, (int)
			 HRP2OpticalFlowProcess::FLOW_STRUCTURE_HARRIS);
#endif

  if (aFlow==0)
    {
      HarrisVar->length(0);
      Harris = HarrisVar._retn();
      return -1;
    }

  /* Dimension of the harris detector . */
#if (LLVS_HAVE_MMX>0)
  Width = aFlow->Width-8;
  Height = aFlow->Height-8;
#else
  Width = Height = 0;
#endif

  HarrisVar->length(Width*Height);

#if (LLVS_HAVE_MMX>0)
  for(int j=0;j<Height;j++)
    {
      p_DerivativeImage = (MM_F_32 *)aFlow->Harris->p[j].s;

      for(int i=0;i<Width;i++)
	{
	  HarrisVar[index] = p_DerivativeImage[i];
	  index++;
	}
    }
#endif

  Harris = HarrisVar._retn();

  return 0;
}

void LowLevelVisionServer::destroy()
  throw(CORBA::SystemException)
{
  /* Right now does nothing */
}

CORBA::Long LowLevelVisionServer::getCameraSeq(HRP2CameraSeq_out cameras)
  throw(CORBA::SystemException)
{


  // Create the sequence of camera.
  HRP2CameraSeq_var tmp_cameras = new HRP2CameraSeq;


  // Set its size to three.
  int lNbCams=0;
  if (m_ImagesInputMethod!=0)
    lNbCams = m_ImagesInputMethod->GetNumberOfCameras();
  tmp_cameras->length(lNbCams);

  for(int i=0;i < lNbCams;i++)
    {
      if (m_Cameras[i]!=0)
	/* The CORBA object corresponding
	   to the servant instance of the camera object (Camera_impl)
	   is inserted */
	{
	  tmp_cameras[i]= m_Cameras[i]->_this();
	}
    }

  // Returns the sequence in the out variable.
  cameras = tmp_cameras._retn();
  return 4;
}


/* Display the process name */
void LowLevelVisionServer::DisplayProcessesNames()
{
  for(unsigned int i=0;i<m_ListOfProcesses.size();i++)
    {
      cerr << " Process : " << m_ListOfProcesses[i]->GetName() << endl;
    }
}

/* Get the CameraSeq names */
CORBA::Long LowLevelVisionServer::getLowLevelVisionProcesses(HRP2LowLevelVisionProcessesSeq_out ProcessesSeq)
  throw(CORBA::SystemException)
{
  HRP2LowLevelVisionProcessesSeq_var aSeqOfProc = new HRP2LowLevelVisionProcessesSeq;

  aSeqOfProc->length(m_ListOfProcesses.size());

  for(unsigned int i=0;i<m_ListOfProcesses.size();i++)
    {
      aSeqOfProc[i] = m_ListOfProcesses[i]->GetName().c_str();
    }
  ProcessesSeq = aSeqOfProc._retn();
  return m_ListOfProcesses.size();
}

/* */
void LowLevelVisionServer::SetAProcessParameterAndValue(string aProcessName, string aParameter, string aValue)
{
  ODEBUG( aParameter << " " << aValue );
  for(unsigned int i=0;i<m_ListOfProcesses.size();i++)
    {
      if (m_ListOfProcesses[i]->GetName()==aProcessName)
	{
	  m_ListOfProcesses[i]->SetParameter(aParameter,aValue);
	}
    }
}


void LowLevelVisionServer::SetCheckEntry(unsigned char ADumpMode)
{
  m_CheckEntry = ADumpMode;
  ODEBUG("m_CheckEntry:" << m_CheckEntry);
  if (m_CheckEntry)
    {
      CreateStack();
    }

}

unsigned char LowLevelVisionServer::GetCheckEntry(void)
{
  return m_CheckEntry;
}

CORBA::Long LowLevelVisionServer::SetAcquisitionSize(CORBA::Long SemanticCameraID,
						     CORBA::Long aWidth,CORBA::Long aHeight)
  throw(CORBA::SystemException)
{
  if (m_TypeOfInputMethod!=LowLevelVisionSystem::FRAMEGRABBER)
    return -1;

  for(int i=0;i<4;i++)
    {
      if ((m_CalibrationWidth[i]<aWidth) ||
	  (m_CalibrationHeight[i]<aHeight))
	return -1;
    }
  return SetImagesGrabbedSize(SemanticCameraID, aWidth,aHeight);
}


void LowLevelVisionServer::SetAProcessParameterAndValue(const char *aProcessName, const char *aParameter, const char *aValue)
  throw(CORBA::SystemException)

{
  string aPN(aProcessName);
  string aP(aParameter);
  string aV(aValue);

  SetAProcessParameterAndValue(aPN, aP, aV);
}

CORBA::Long LowLevelVisionServer::GetProcessParameter(const char * aProcessName,
						      const char * aParameterName,
						      CORBA::String_out ParameterValue)
  throw(CORBA::SystemException)
{
  int index=-1;
  int NbOfParameters=0;

  HRP2LowLevelVisionParametersSeq_var ParametersSeq_var = new HRP2LowLevelVisionParametersSeq;
  HRP2LowLevelVisionParametersSeq_var ParametersValueSeq_var = new HRP2LowLevelVisionParametersSeq;

  /* Search for the process. */
  for(unsigned int i=0;i<m_ListOfProcesses.size();i++)
    {
      if (!strcmp(m_ListOfProcesses[i]->GetName().c_str(),aProcessName))
	{
	  index = i;
	  break;
	}
    }

  /* Did not find */
  if (index==-1)
    {
      ParameterValue=CORBA::string_dup("");
      return 0;
    }
  string aSParameterName = aParameterName;
  string aValue;
  m_ListOfProcesses[index]->GetValueOfParameter(aSParameterName,aValue);
  ParameterValue = CORBA::string_dup(aValue.c_str());
  return 1;
}

CORBA::Long LowLevelVisionServer::GetProcessParameters(const char * aProcessName,
						       HRP2LowLevelVisionParametersSeq_out ParametersSeq,
						       HRP2LowLevelVisionParametersSeq_out ParametersValueSeq)
  throw(CORBA::SystemException)
{
  int index=-1;
  int NbOfParameters=0;

  HRP2LowLevelVisionParametersSeq_var ParametersSeq_var = new HRP2LowLevelVisionParametersSeq;
  HRP2LowLevelVisionParametersSeq_var ParametersValueSeq_var = new HRP2LowLevelVisionParametersSeq;

  /* Search for the process. */
  for(unsigned int i=0;i<m_ListOfProcesses.size();i++)
    {
      if (!strcmp(m_ListOfProcesses[i]->GetName().c_str(),aProcessName))
	{
	  index = i;
	  break;
	}
    }

  /* Did not find */
  if (index==-1)
    {
      ParametersSeq_var->length(0);
      ParametersValueSeq_var->length(0);
      ParametersSeq = ParametersSeq_var._retn();
      ParametersValueSeq = ParametersValueSeq_var._retn();
      return 0;
    }

  vector<string> VectorOfParams;
  vector<string> VectorOfParamsValue;

  m_ListOfProcesses[index]->GetParametersAndValues(VectorOfParams, VectorOfParamsValue);
  ParametersSeq_var->length(VectorOfParams.size());
  ParametersValueSeq_var->length(VectorOfParamsValue.size());

  for(unsigned int i=0;i<VectorOfParams.size();i++)
    {
      ParametersSeq_var[i] = CORBA::string_dup(VectorOfParams[i].c_str());
      ParametersValueSeq_var[i] = CORBA::string_dup(VectorOfParamsValue[i].c_str());
    }

  ParametersSeq = ParametersSeq_var._retn();
  ParametersValueSeq = ParametersValueSeq_var._retn();

  return VectorOfParams.size();
}

LowLevelVisionSystem::DumpImageMode LowLevelVisionServer::GetDumpImageMode()
  throw(CORBA::SystemException)

{
  return m_DumpImageMode;
}

void LowLevelVisionServer::SetDumpImageMode(LowLevelVisionSystem::DumpImageMode aDumpImageMode,
					    const FloatBuffer &Informations,
					    CORBA::Long SizeOfInformations)
  throw(CORBA::SystemException)
{

  m_DumpInformations.clear();
  for(int i=0;i<SizeOfInformations;i++)
    {
      m_DumpInformations.insert(m_DumpInformations.end(), Informations[i]);
    }
  m_DumpImageMode = aDumpImageMode;
}

void LowLevelVisionServer::SetRobotVisionCalibrationDirectory(string lRVCalibDir)
{
  m_RobotVisionCalibrationDirectory = lRVCalibDir;
  ODEBUG("LowLevelVisionServer::SetRobotVisionCalibrationDirectory"
	 << m_RobotVisionCalibrationDirectory.c_str() );

  ReadRbtCalib();
}

CORBA::Long LowLevelVisionServer::GetMatrixHeadTOrg(DoubleBuffer_out HeadTOrg)
  throw(CORBA::SystemException)
{
  DoubleBuffer_var HeadTOrgVar;

  HeadTOrgVar = new DoubleBuffer;
  HeadTOrgVar->length(16);
  for(int i=0;i<16;i++)
    HeadTOrgVar[i] = m_headTorg[i];
  HeadTOrg = HeadTOrgVar._retn();
  return 0;
}

void LowLevelVisionServer::GetMatrixHeadTOrg(double *HeadTOrg)
{
  if (!HeadTOrg)
    return;

  for(int i=0;i<16;i++)
    {
      HeadTOrg[i] = m_headTorg[i];
      ODEBUG(HeadTOrg[i] );
    }

  return;
}

CORBA::Long LowLevelVisionServer::GetBoundaryRepresentation(CBREPSeq_out aBrep)
  throw(CORBA::SystemException)
{
  CBREPSeq_var aBrepVar;
  int NbOfImagesProcessed= 0;

#if (LLVS_HAVE_VVV>0)
  if (m_BRepDetection!=0)
    NbOfImagesProcessed = m_BRepDetection->BuildBrepVar(aBrepVar);
#endif
  aBrep = aBrepVar._retn();

  return NbOfImagesProcessed;
}

void LowLevelVisionServer::CreateStack()
{
  ODEBUG("Here " << m_Width[0] << " " << m_Height[0]);
  //m_MaxSI = 33*120;
  m_MaxSI = 33*60;
  m_IndexSI = 0;
  m_NumberOfImagesToStack=m_ImagesInputMethod->GetNumberOfCameras();
  ODEBUG3("m_NumberOfImagesToStack"<< m_NumberOfImagesToStack);
  m_IndexSensorsStack = 0;

  ODEBUG(m_Width[0]*m_Height[0]*m_MaxSI);
  m_StoredImages= new unsigned char[m_Width[0] * m_Height[0] * m_MaxSI * m_depth[0]];
  if (m_StoredImages==0)
    ODEBUG3("COULD NOT ALLOCATE ENOUGH MEMORY for stack");
  else
    ODEBUG3("COULD ALLOCATE ENOUGH MEMORY for stack");

  m_StoredTimeStamp = new double[m_MaxSI];
  if (m_StoredImages==0)
    ODEBUG3("COULD NOT ALLOCATE ENOUGH MEMORY for timestamps");

  m_StoredCameraCov = new double[9*m_MaxSI];
  if (m_StoredCameraCov==0)
    ODEBUG3("COULD NOT ALLOCATE ENOUGH MEMORY for Gyro");
  else
    ODEBUG3("COULD ALLOCATE ENOUGH MEMORY for Gyro");

  m_SideOfTheImage = new double[2*m_MaxSI];
  if (m_SideOfTheImage==0)
    ODEBUG3("COULD NOT ALLOCATE ENOUGH MEMORY for Waist Velocity");
  else
    ODEBUG3("COULD ALLOCATE ENOUGH MEMORY for Waist Velocity");

  m_StoredCameraPosOri = new double[7*m_MaxSI];
  if (m_StoredCameraPosOri==0)
    ODEBUG3("COULD NOT ALLOCATE ENOUGH MEMORY for Waist Orientation");
  else
    ODEBUG3("COULD ALLOCATE ENOUGH MEMORY for Waist Orientation");

  ODEBUG3("Finished");


}

void LowLevelVisionServer::DeleteStack()
{
  if (m_StoredImages!=0)
    delete m_StoredImages;

  if (m_StoredTimeStamp!=0)
    delete m_StoredTimeStamp;

  if (m_StoredCameraCov!=0)
    delete m_StoredCameraCov;

  if (m_SideOfTheImage!=0)
    delete m_SideOfTheImage;

  if (m_StoredCameraPosOri!=0)
    delete m_StoredCameraPosOri;

}

void LowLevelVisionServer::StoreImageOnStack(int image)
{

  ODEBUG("Calls StoreImageOnStack");
  if (m_StoredImages==0)
    return;

  if (m_StoredTimeStamp==0)
    return;

  unsigned char *ptdst = m_StoredImages + m_IndexSI*m_Width[0]*m_Height[0]*m_depth[0];
  unsigned char *ptsrc = m_BinaryImages[image];
  for(unsigned int l=0;l<m_Height[0]*m_Width[0]*m_depth[0];l++)
    *ptdst++ = *ptsrc++;

  m_StoredTimeStamp[m_IndexSI] = m_timestamps[image];
  m_SideOfTheImage[m_IndexSI]=(double)image;

#if (LLVS_HAVE_SCENE>0)
  if ((m_StoredCameraCov!=0) && (m_SingleCameraSLAM!=0))
    {

      double timeref = m_timestamps[image];
#if 0
      m_SingleCameraSLAM->GetGyroAcceleroFromTimeStamp(m_StoredCameraCov+6*m_IndexSensorsStack,
						       m_StoredCameraCov+6*m_IndexSensorsStack+3,
						       timeref,
						       m_SideOfTheImage+2*m_IndexSensorsStack,
						       m_StoredCameraPosOri+7*m_IndexSensorsStack);
#else
      m_SingleCameraSLAM->GetPositionAndCovariance(m_StoredCameraPosOri+7*m_IndexSI,
						   m_StoredCameraCov+9*m_IndexSI);
      m_SideOfTheImage[m_IndexSI]=(double)image;
      //      cout << "SideOfTheImage " << m_SideOfTheImage[m_IndexSI]
      //   << " " << image << endl;
#endif
      //      m_IndexSensorsStack++;
    }
#endif

  m_IndexSI++;
  if(m_IndexSI==m_MaxSI)
    m_IndexSI = 0;



}

void LowLevelVisionServer::RecordImagesOnDisk(int image)
{

  if (m_CTS!=0)
    {
      string aFileName("TSPositionAttitude.dat");
      m_CTS->DumpCircularBuffer(aFileName);
      m_CTS->StopThreadOnConnectionSot();
    }

  ODEBUG("Recording images on disk.");
  if (m_CheckEntry)
    {
      ODEBUG3("Check entry correct.." << m_MaxSI);
      if (m_StoredImages==0)
	{
	  ODEBUG3("No stored images.");
	  return;
	}

      ODEBUG3("RecordImagesOnDisk begin");
      FILE *fp,*fp_sensors;
      char Buffer[1024];

      unsigned int ldepth = m_NumberOfImagesToStack;
      ODEBUG3("Number of cameras:" << ldepth);
      for(unsigned int i=0; i<ldepth; i++)
	{
	  bzero(Buffer,1024);
	  sprintf(Buffer,"TimeStamp%02d.dat",i);
	  fp = fopen(Buffer,"w");

	  /* Store timestamp */
	  if (fp!=0)
	    {
	      double prevTimeStamp=0;
	      for(unsigned int j=0; j<m_MaxSI/ldepth; j++)
		{
		  double TimeStamp=m_StoredTimeStamp[j*ldepth+i];
		  if (j==0)
		    prevTimeStamp=TimeStamp;
		  fprintf(fp,"%f %f\n",TimeStamp,TimeStamp-prevTimeStamp);
		  prevTimeStamp=TimeStamp;
		}
	      fclose(fp);

	    }
	}

      bzero(Buffer,1024);
      sprintf(Buffer,"CameraCov.dat");
      fp_sensors = fopen(Buffer,"w");
      if (fp_sensors!=0)
	{
	  for(unsigned int i=0; i<m_MaxSI*9; i++)
	    {
	      fprintf(fp_sensors,"%f ",m_StoredCameraCov[i]);
	      if ((i>0) && (i%9==8))
		{
		  double TimeStamp=m_StoredTimeStamp[i/9];
		  fprintf(fp_sensors,"%f\n",TimeStamp);
		}
	    }
	  fclose(fp_sensors);
	}

      bzero(Buffer,1024);
      sprintf(Buffer,"SideOfImage.dat");
      fp_sensors = fopen(Buffer,"w");
      if (fp_sensors!=0)
	{
	  for(unsigned int i=0; i<m_MaxSI; i++)
	    {
	      fprintf(fp_sensors,"%f\n",m_SideOfTheImage[i]);
	      /*
		fprintf(fp_sensors,"%f ",m_SideOfTheImage[i]);
		if ((i>0) && (i%2==1))*
		fprintf(fp_sensors,"\n");*/
	    }
	  fclose(fp_sensors);
	}

      bzero(Buffer,1024);
      sprintf(Buffer,"CameraPositionOrientation.dat");
      fp_sensors = fopen(Buffer,"w");
      if (fp_sensors!=0)
	{
	  for(unsigned int i=0; i<m_MaxSI*7; i++)
	    {
	      fprintf(fp_sensors,"%f ",m_StoredCameraPosOri[i]);
	      if ((i>0) && (i%7==6))
		{
		  double TimeStamp=m_StoredTimeStamp[i/7];
		  fprintf(fp_sensors," %f\n",TimeStamp);
		}
	    }
	  fclose(fp_sensors);
	}

      unsigned long int lCounter[m_NumberOfImagesToStack];
      for(unsigned int li=0;li<m_NumberOfImagesToStack;li++)
	lCounter[li]= 0;

      for(unsigned int i=0; i<m_MaxSI; i++)
	{
	  int k,l;
	  unsigned char *pt = m_StoredImages+i*m_Width[0]*m_Height[0]*m_depth[0];
	  char Buffer[1024];
	  bzero(Buffer,1024);
	  char BufExten[30];
	  bzero(BufExten,30);
	  if (m_depth[0]==1)
	    sprintf(BufExten,".pgm");
	  else
	    sprintf(BufExten,".ppm");

	  if (m_NumberOfImagesToStack==1)
	    sprintf(Buffer,"dumpcheck%06d%s",i,BufExten);
	  else if (m_NumberOfImagesToStack>=0)
	    {
              int lindex = (int)m_SideOfTheImage[i];
	      sprintf(Buffer,"dumpcheck%06ld_%02d%s",lCounter[lindex]++,lindex,BufExten);
	    }

	  fp = fopen(Buffer,"w");
	  if (i%100==0)
	    cout << "Save the image : " << Buffer << endl;

	  if (fp!=0)
	    {
	      double TimeStamp=m_StoredTimeStamp[i];
	      if (m_depth[0]==1)
		fprintf(fp,"P5\n# TimeStamp: %f\n%d %d\n255\n",TimeStamp,(int)m_Width[0],(int)m_Height[0]);
	      else if (m_depth[0]==3)
		fprintf(fp,"P6\n# TimeStamp: %f\n%d %d\n255\n",TimeStamp,(int)m_Width[0],(int)m_Height[0]);
	      fwrite(pt,m_Height[0] * m_Width[0] * m_depth[0] ,1,fp);
	      /*
		for(l=0;l<m_Height;l++)
		{
		for(k=0;k<m_Width;k++)
		{
		for(int m=0;m<m_SizeOfPixelForStack;m++)
		fprintf(fp,"%c",*pt++);
		}
		}
	      */
	      fclose(fp);
	    }
	}
      ODEBUG3("RecordImagesOnDisk end");
    }
}


CORBA::Long LowLevelVisionServer::GetSceneObject(SceneObject_out aSceneObject)
  throw(CORBA::SystemException)
{

  SceneObject_var aSceneObjectvar = new SceneObject;

#if  (LLVS_HAVE_SCENE>0)
  if (m_SingleCameraSLAM!=0)
    {
      m_SingleCameraSLAM->CreateCopyOfScene(aSceneObjectvar);
    }
#endif

  aSceneObject = aSceneObjectvar._retn();
  return 1;
}

void LowLevelVisionServer::SceneDeleteFeature(CORBA::Long FeatureLabel)
  throw(CORBA::SystemException)
{

}

CORBA::Long LowLevelVisionServer::GetImageIdentifier()
  throw(CORBA::SystemException)
{
  return m_ImageCounter;
}

StereoVision_ptr LowLevelVisionServer::getStereoVision()
  throw(CORBA::SystemException)
{
  StereoVision_var tmp_StereoVision;

  tmp_StereoVision = m_StereoVision_impl->_this();
  return tmp_StereoVision._retn();
}

ModelTrackerInterface_ptr LowLevelVisionServer::getModelTracker()
  throw(CORBA::SystemException)
{
  ModelTrackerInterface_var tmp_ModelTrackerInterface;
#if (LLVS_HAVE_NMBT>0)
  tmp_ModelTrackerInterface = m_ModelTrackerCorbaRequestProcess_impl->_this();
#endif
  return tmp_ModelTrackerInterface._retn();
}

PointTrackerInterface_ptr LowLevelVisionServer::getPointTracker()
  throw(CORBA::SystemException)
{
  PointTrackerInterface_var tmp_PointTrackerInterface;
#if (LLVS_HAVE_VISP>0)
  tmp_PointTrackerInterface = m_PointTrackerCorbaRequestProcess_impl->_this();
#endif
  return tmp_PointTrackerInterface._retn();
}


void LowLevelVisionServer::SetTheSLAMImage(int anIndex)
{
  m_TheSLAMImage = (unsigned long int) anIndex;
#if (LLVS_HAVE_SCENE>0)
  m_SingleCameraSLAM->SetInputImages(&m_epbm[m_TheSLAMImage]);
#endif
}

BtlSlamInterface_ptr
LowLevelVisionServer::getBtlSlamInterface()
  throw(CORBA::SystemException)
{
  BtlSlamInterface_var interface;
#if (LLVS_HAVE_HRP_BTL_SLAM>0)
  interface = m_BtlSlamProcess->GetInterface()->_this();
#else
  ODEBUG3("[BtlSlamInterface] WARNING! Interface has not been compiled");
#endif
  return interface._retn();
}

int LowLevelVisionServer::GetTheSLAMImage()
{
  return (int) m_TheSLAMImage;
}

