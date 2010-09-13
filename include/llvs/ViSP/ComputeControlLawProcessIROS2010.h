/** @doc Object to compute the control law described in:
   Cancelling the sway motion of dynamic walking in visual servoing
   C. Dune, A. Herdt, O. Stasse, P.-B. Wieber, K. Yokoi, E. Yoshida
   IROS 2010. 

   Copyright (c) 2010,
   @author Olivier Stasse

   see License file for more information
   on the license applied to this code.
   
   
*/
#ifndef _HRP2_COMPUTE_CONTROL_LAW_PROCESS_IROS_2010_H_
#define _HRP2_COMPUTE_CONTROL_LAW_PROCESS_IROS_2010_H_

/*! System includes */
#include <iostream>

/*! LLVS includes */
#include "VisionBasicProcess.h"


/*! ViSP includes */

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpServo.h>
#include <visp/vpFeatureTranslation.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpTwistMatrix.h>

/*! Tracking */
#include "ModelTracker/nmbtTrackingProcess.h"

#include <ConnectionToSot.h>


struct TimedInteractionMatrix
{
  vpMatrix L;
  double timestamp;
} ;

/*!  \brief This class implements tracks an object model on a VISP image
  remark:all function that uses X11 are developped in the client
*/
class HRP2ComputeControlLawProcessIROS2010 : public HRP2VisionBasicProcess
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
  HRP2ComputeControlLawProcessIROS2010();

  /*! Destructor */
  virtual ~HRP2ComputeControlLawProcessIROS2010();

  /*! Set a parameter */
  int pSetParameter(string aParameter, string aValue);
   
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

  /*! Put Velocity value at zero when lower than 0.02  */
  int ZeroVelocity(double * VelRef);

public:

  /*! Velocity saturation*/
  int VelocitySaturation(const vpColVector &RawVel,double *SatVel);

 
 
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
  
   
  /*Stop*/
  int stop(double * VelRef);
  
  
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
  
  /* Store the initial  model height */
  double m_IninitHeight;
   
  /* Rotation on X axis limitation for the model */
  double m_RxLimit;

  /* Rotation on X axis limitation for the model */
  double m_RyLimit;

  /* Velocity maximun send to Robot*/
  vpColVector m_Velmax;

  /* Velocity value for puting it to zero*/
  vpColVector m_Velzero;

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
  
  /*Flag to check if the control law must be computed*/
  bool m_RealiseControlLaw;
  
  /**/ 
  std::string m_internalState; 

  /*! \brief Additional term to compute the integral term of the control law. */
  double m_Lbk;

  /*! \brief Previous Interaction Matrix. */
  TimedInteractionMatrix m_prevL;

  /*! \brief History of com speed reference. */
  vector<double> m_dcomref;
};


#endif 
