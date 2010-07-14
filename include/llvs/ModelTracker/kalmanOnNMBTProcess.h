/** @doc Object to convert back and forth a ViSP image into a OpenCV image 
    

   Copyright (c) 2003-2010,
   @author Stephane Embarki

   see License file for more information
   on the license applied to this code.
   
   
*/
#ifndef _KALMAN_ON_NMBT_PROCESS_H_
#define _KALMAN_ON_NMBT_PROCESS_H_

#include <iostream>
#include <queue>

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

  /*! Set pointer on m_timestamp in LLVS*/
  int SetTimeStamp(double* aTimeStamp);
   
  /*! Set the ConnectionToSot  pointer */
  void SetConnectionToSot (llvs::ConnectionToSot * aCTS);

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

  void KalmanOnSoT();
  
  struct dataSoT
  {
    vpColVector Y;
    vpColVector U;
    double timeStamp;
    
  };
   
  /*State molidelisation type */
  typeState m_StateType;

  /*Kalman Filter object*/
  seKalman* m_Kalman;

  /*State molidelisation object*/
  StateModel* m_StateModel;

  /*Tracker measure molidelisation object*/
  MeasureModel* m_TrackerModel;

  /*SoT measure molidelisation object*/
  MeasureModel* m_SoTModel;

  /* Connection to Stack of Task*/
  llvs::ConnectionToSot * m_CTS;


  /*State molidelisation size*/
  int m_StateSize;

  /* Measure vector*/
  vpColVector m_Y;

  /* Control vector*/
  vpColVector m_U;

  /* FIFO of dataSoT */
  queue<dataSoT> m_DataSot;

  /* TimeStamp in LLVS*/
  double* m_TimeStampLLVS;

  /* TimeStamp in LLVS*/
  double  m_LastTimeStamp;
  
  /*State covrariance matrix*/
  vpMatrix m_P;
  
  /*State noise variance vector*/
  vpColVector m_N;

  /*Tracker noise variance Matrix*/
  vpMatrix m_RTracker;

  /*SoT noise variance Matrix*/
  vpMatrix m_RSoT;

 public:
  /*Measure noise variance Matrix*/
  bool m_ReIntializedNMBT;

  /*m_TimeStampLLVS is initialized*/
  bool m_TimeStampInitialize;

};

#endif 
