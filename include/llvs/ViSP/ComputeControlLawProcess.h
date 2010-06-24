/** @doc Object to convert back and forth a ViSP image into a OpenCV image 
    

   Copyright (c) 2003-2010,
   @author Stephane Embarki

   see License file for more information
   on the license applied to this code.
   
   
*/
#ifndef _HRP2_COMPUTE_CONTROL_LAW_PROCESS_H_
#define _HRP2_COMPUTE_CONTROL_LAW_PROCESS_H_

#include <iostream>



#include "VisionBasicProcess.h"


// include visp lib files

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpServo.h>
#include <visp/vpFeatureTranslation.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpTwistMatrix.h>

#include "ModelTracker/nmbtTrackingProcess.h"

#include <ConnectionToSot.h>



/*!  \brief This class implements tracks an object model on a VISP image
  remark:all function that uses X11 are developped in the client
*/
class HRP2ComputeControlLawProcess : public HRP2VisionBasicProcess
{
 public:

  typedef enum
    {  
	FREE,
	HEIGHT_LIMITED,
	PLAN_MOTION,
	ON_GROUND

    } typeMotion;
  
  /*! Constructor */
  HRP2ComputeControlLawProcess();

  /*! Destructor */
  virtual ~HRP2ComputeControlLawProcess();

  /*! Set a parameter */
  int SetParameter(string aParameter, string aValue);
   
  /*! Set the nmbt tracker pointer */
  void SetTracker(HRP2nmbtTrackingProcess* anmbt);

  /*! Set the cdMo */
  void SetcdMo ( vpHomogeneousMatrix acdMo);

  /*! Set the ConnectionToSot  pointer */
  void SetConnectionToSot (llvs::ConnectionToSot * aCTS);

  /*! Set the type of test on motion*/
  void SetMotionTest(typeMotion aMotion,
		     const vector<double> &limit = vector<double>(0));
   
  /*! Get the cdMc */
  void GetcdMc ( vpHomogeneousMatrix &acdMc);

  inline void GethVc(vpTwistMatrix& hVc){hVc=m_hVc;} 
  
  /*! Get the ComputeV */
  void GetComputeVelocity ( vpColVector &aCV);

  /*! Get the Error */
  void GetError ( double &aError);


 protected:

  /*! Initialize the process. */
  int pInitializeTheProcess();
  
  /*! Realize the process */
  int pRealizeTheProcess();
  
  /*! Cleanup the process */
  int pCleanUpTheProcess();
  
  /*! Start the process */
  int pStartProcess();

  /*! Test if object is still on the ground  */
  bool objectOnGround(const vpHomogeneousMatrix &afMo);

  /*! Test on object plan  Motion */
  bool planMotion(const vpHomogeneousMatrix &afMo);

  /*! Test the object high position*/
  bool heightInLimit(const vpHomogeneousMatrix &afMo);

  /*! Test the object Motion */
  bool TestObjectMotion(const vpHomogeneousMatrix &afMo);
 
 public:
   /*!Convert velocity from camera to waist*/
   int changeVelocityFrame(const vpColVector&velCam,
			   vpColVector&velWaist,
			   const double *poseHeadInFoot,
			   const double *poseWaistInFoot,
			   vpHomogeneousMatrix & afMh);

 protected:
   /*!Init the parameters*/
   int init();
   
   /*!load the cameraHeadTransform*/
   int loadcMh(vpHomogeneousMatrix &M);
  
   /* Nmbt tracking process*/
   HRP2nmbtTrackingProcess* m_nmbt;
   
   /* Connection to Stack of Task*/
   llvs::ConnectionToSot * m_CTS;

   /* Object pose in the current camera frame*/
   vpHomogeneousMatrix m_cMo;

   /* Object pose in the desired camera frame*/
   vpHomogeneousMatrix m_cdMo;

   /* Current camera pose in the desired camera frame*/
   vpHomogeneousMatrix m_cdMc;
   
   /*Change velocity frame from camera to head*/
   vpTwistMatrix m_hVc;

   /* Current camera pose in the head frame*/
   vpHomogeneousMatrix m_headMcamera;

   /* Visual Servoing Task */
   vpServo m_Task;  
    
   /* Visual Servoing Translation Feature*/
   vpFeatureTranslation* m_FT;
 
   /* Visual Servoing Utheta Feature*/
   vpFeatureThetaU *  m_FThU;

   /* Visual Servoing gain */
   double m_Lambda;
  
   /* Visual Servoing error */
   double m_Error;

   /* Heiht limitation for the model*/
   double m_ModelHeightLimit;

   /* Rotation on X axis limitation for the model */
   double m_RxLimit;

   /* Rotation on X axis limitation for the model */
   double m_RyLimit;

   /* Save of the last object pose in foot frame*/
   vpHomogeneousMatrix m_LastfMo;
   
   /* Control velocity expressed in the Waist Frame */ 
   vpColVector m_ComputeV;

   /* Define type of motion tested*/
   typeMotion m_MotionTested;

   /*Flag to check if the process is initialized*/
   bool m_ProcessInitialized;
   
   /*Flag to check if cMo is set*/
   bool m_cdMoSet;

   /*Flag to check if the control law is computed*/
   bool m_ControlLawComputed;
   
};


#endif 
