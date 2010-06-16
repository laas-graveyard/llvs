/** @doc This object implements a visual process to get a disparity map.

    Copyright (c) 2010, 
    @author Stephane Embarki
   
    JRL-Japan, CNRS/AIST
    
    See license file for information on license.
*/
#ifndef _HRP2_POINT_TRACKING_PROCESS_H_
#define _HRP2_POINT_TRACKING_PROCESS_H_

#include <iostream>

#include "VisionBasicProcess.h"
/*! HRP2VisionBasicProcess is used to derive all the vision process
 * at least for the LowLevelVisionServer.
 */


#if LLVS_HAVE_VISP
 
// include visp lib files
#include <visp/vpImage.h>
#include <visp/vpPoint.h>
#include <visp/vpDot2.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpHomogeneousMatrix.h>


/*!
 This class implements tracks an object model on a VISP image
 remark:all function that uses X11 are developped in the client
 */
class HRP2PointTrackingProcess : public HRP2VisionBasicProcess
{

 public:
 
 
  /*! Constructor */
  HRP2PointTrackingProcess();

  /*! Destructor */
  virtual ~HRP2PointTrackingProcess();

  /*! Default Param*/
  int SetDefaultParam(); 

  /*! Initialize the process. */
  int pInitializeTheProcess();

  /*! Realize the process */
  int pRealizeTheProcess();
  
  /*! Cleanup the process */
  int pCleanUpTheProcess();

  /*! Set a parameter */
  int pSetParameter(string aParameter, string aValue);
  

  /*!Compute the Pose*/
  int computePose();
 
  /*!Realize the tracking*/
  int Tracking();

  /*!Set the target model 
    and the initial coordinate for the tracked Dot*/
  int Init(vector<vpPoint> Target, vector<vpImagePoint*> Point, unsigned int nbPoint);

  /*! Set tracker parameters : camera parameters */
  void SetCameraParameters(const vpCameraParameters & _cam);
 
  /*! Set the image */
  void SetInputVispImages(vpImage<unsigned char> * _I);  
   
  /*! Get tracker parameters : camera parameters */
  void GetCameraParameters(vpCameraParameters & _cam); 
   
  /*! Get the image */
  void GetInputVispImages(vpImage<unsigned char> & _I); 
   
  /*! Get the inputcMo */
  void GetOutputcMo(vpHomogeneousMatrix & _outputcMo);

  /*! Get the vpDot2*/
  void GetvpDot2( vector<vpDot2*> & DotList);


  /*! Get the vpImagePoint*/
  void GetvpImagePoint(vector<vpImagePoint*> &IPList);
  
  /*! Get Image Height*/
  void GetHeight(int&_height)   ;
  
  /*! Get Image Width*/
  void GetWidth(int&_width)  ;

 
private:
  
  // TODO : uniformiser les 3.

  /*! Parse camera parameters*/
  int ParseCamParam();
  

protected:

  /*!Number of Point*/
  unsigned int m_NbPoint;

  /*! visp 3D Point*/
  vector<vpPoint> m_PointList;

  /*! visp Dot2*/
  vector<vpDot2*> m_Dot2List;

  /*! visp Image Ponit*/
  vector<vpImagePoint*> m_vpIPList;
    
  /*! visp images*/
  vpImage<unsigned char> *m_inputVispImage;
  
  /*! image dimensions*/
  int m_imageHeight;
  int m_imageWidth;
    
  /*! computed transformation between the object and the camera*/
  vpHomogeneousMatrix m_outputcMo;  
 
  /*! camera parameters*/
  vpCameraParameters m_cam; 

  /*! path cam param*/
  string m_pathCam;
  
  /*! name cam*/
  string m_nameCam;


  /*! perspective type*/
  vpCameraParameters::vpCameraParametersProjType m_projType;

 public:
  bool m_inputImagesLoaded;
  bool m_cameraParamLoaded;
  bool m_InitDone;
  bool m_trackerTrackSuccess; 	
};

#endif // LLVS_HAVE_VISP


#endif 
