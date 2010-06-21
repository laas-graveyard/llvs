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
  
  /*! Get the cdMc */
  void GetcdMc ( vpHomogeneousMatrix &acdMc);

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


   HRP2nmbtTrackingProcess* m_nmbt;
   
   llvs::ConnectionToSot * m_CTS;

   /* Object pose in camera frame*/
   vpHomogeneousMatrix m_cMo;

   /* Object pose desired in camera frame*/
   vpHomogeneousMatrix m_cdMo;

   /* Camera pose desired in camera frame*/
   vpHomogeneousMatrix m_cdMc;

   vpServo m_Task;  

   vpFeatureTranslation* m_FT;
 
   vpFeatureThetaU *  m_FThU;

   vpTwistMatrix m_hVc;
   
   double m_Lambda;

   double m_Error;

   vpColVector m_ComputeV;

   bool m_ProcessInitialized;
   bool m_cdMoSet;

   bool m_ControlLawComputed;

};



#endif 
