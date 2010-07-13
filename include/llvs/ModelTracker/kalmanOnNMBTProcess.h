/** @doc Object to convert back and forth a ViSP image into a OpenCV image 
    

   Copyright (c) 2003-2010,
   @author Stephane Embarki

   see License file for more information
   on the license applied to this code.
   
   
*/
#ifndef _KALMAN_ON_NMBT_PROCESS_H_
#define _KALMAN_ON_NMBT_PROCESS_H_

#include <iostream>

#include "VisionBasicProcess.h"
/*! HRP2VisionBasicProcess is used to derive all the vision process
 * at least for the LowLevelVisionServer.
 */


// include visp lib files
#include <visp/vpColVector.h>
#include <visp/vpMatrix.h>

#include "ConnectionToSot.h"
#include "ModelTracker/nmbtTrackingProcess.h"

class seKalman;
class MeasureModel;
class StateModel;

class HRP2KalmanOnNMBTProcess: public HRP2nmbtTrackingProcess
{

 public:

  typedef enum
  {  
    VEL_CAM,
    VEL_OBJ,
    ACC_CAM,
    ACC_OBJ,
    CTL_CAM,
    CTL_OBJ
  } typeState ;	
  
  /*! Constructor */
  HRP2KalmanOnNMBTProcess();
  
  /*! Destructor */
  virtual ~HRP2KalmanOnNMBTProcess();

  /*! Set a parameter */
  int pSetParameter(string aParameter, string aValue);
 

 protected:

  /*! Initialize the process. */
  int pInitializeTheProcess();
  
  /*! Realize the process */
  int pRealizeTheProcess();
  
  /*! Cleanup the process */
  int pCleanUpTheProcess();

  /*! Convert vpHomogeneousMatrix to vpColverctor*/
  void ConvertHMatrixToCVector(const vpHomogeneousMatrix & aHM,
			       vpColVector &aCV);
  
  /*! Convert vpHomogeneousMatrix to vpColverctor*/
  void ConvertCVectorToHMatrix(const vpColVector & aCV,
			       vpHomogeneousMatrix & aHM);
 public:
   
  /*State molidelisation type */
  typeState m_StateType;

  /*Kalman Filter object*/
  seKalman* m_Kalman;

  /*State molidelisation object*/
  StateModel* m_StateModel;

  /*State molidelisation object*/
  MeasureModel* m_MeasureModel;

  /*State molidelisation object*/
  int m_StateSize;

  /* Measure vector*/
  vpColVector m_Y;
  
  /*State covrariance matrix*/
  vpMatrix m_P;
  
  /*State noise variance vector*/
  vpColVector m_N;

  /*Measure noise variance Matrix*/
  vpMatrix m_R;

 /*Measure noise variance Matrix*/
  bool m_ReIntializedNMBT;


};

#endif 
