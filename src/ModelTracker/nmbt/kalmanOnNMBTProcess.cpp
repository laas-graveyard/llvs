/** @doc This object implements a visual process to get a disparity map.

    Copyright (c) 2010, 
    @author Stephane Embarki
   
    JRL-Japan, CNRS/AIST
    
    See license file for information on license.
*/
#include <llvs/tools/Debug.h>

#include "ModelTracker/kalmanOnNMBTProcess.h"
//#include "ModelTracker/nmbtTrackingProcess.h"

/* Kalman include*/
#include "KalmanFilter/TrackerModel.h"
#include "KalmanFilter/sekalman.h"
#include "KalmanFilter/SE3ModelVconstRo.h"
#include "KalmanFilter/SE3ModelVconstRc.h"
#include "KalmanFilter/SE3ModelAtconstRo.h"
#include "KalmanFilter/SE3ModelAtconstRc.h"
#include "KalmanFilter/SE3ModelUvRo.h"
#include "KalmanFilter/SE3ModelUvRc.h"

//using namespace llvs;
/*!-------------------------------------
  Default constructor
  -------------------------------------*/

HRP2KalmanOnNMBTProcess::HRP2KalmanOnNMBTProcess()
{                       
  m_ProcessName ="KalmanOnNMBTProcess";
  
  m_StateSize=12;
 
  m_StateType=VEL_OBJ;

  m_StateModel=0x0;
  m_MeasureModel=0x0;
  m_Kalman=0x0;

  m_P.resize(m_StateSize,m_StateSize);
  m_P.setIdentity();

  m_Y.resize(6);

  m_N.resize(6);

  m_R.resize(6,6);
  m_R.setIdentity();
  m_R*=0.25;

  m_ReIntializedNMBT=false;
}



/*!-------------------------------------
  Destructor
  ------------------------------------- */
HRP2KalmanOnNMBTProcess:: ~HRP2KalmanOnNMBTProcess()
{
  delete m_StateModel;
  delete m_MeasureModel;
  delete m_Kalman; 
}


/*!-------------------------------------
  Sets the parameters

The parameter names can be :

KALMAN_STATE : VEL_CAM
               VEL_OBJ
               ACC_CAM
	       ACC_OBJ
	       COM_CAM
	       COM_OBJ

KALMAN_PVAR
KALMAN_NVAR
KALMAN_RTRACKER
KALMAN_REINIT : ON or OFF
-------------------------------------*/
int HRP2KalmanOnNMBTProcess::pSetParameter(std::string aParameter, 
					       std::string aValue)
{
 
  // get the 6 first parameter to find the parameter type
  // get 6 letters starting from the letter number 0
  string paramType = aParameter.substr(0,6);

  if(paramType=="KALMAN")
    {
      std::string lParam(aParameter);
      lParam.erase(0,7);
      if(lParam == "STATE")
	{
	  if(aValue == "VEL_CAM")
	    {
	      m_StateSize=12;
	      m_StateType=VEL_CAM;
	    }
	  else if(aValue == "VEL_OBJ")
	    {
	      m_StateSize=12;
	      m_StateType=VEL_OBJ;
	    } 
	  else if(aValue == "ACC_CAM")
	    {
	      m_StateSize=15;
	      m_StateType=ACC_CAM;
	    } 
	  else if(aValue == "ACC_OBJ")
	    {
	      m_StateSize=15;
	      m_StateType=ACC_OBJ;
	    } 
	  else if(aValue == "COM_CAM")
	    { 
	      m_StateSize=12;
	      m_StateType=COM_CAM;
	    }
	  else if(aValue == "COM_OBJ")
	    {
	      m_StateSize=12;
	      m_StateType=COM_OBJ;
	    }
	   else 
	     {
	       cout << "Warning : KALMAN_STATE unknown value :"<< aValue << endl; 
	       return -1;
	     }
	}
      else if(lParam == "NVAR")
	{
	  int found=0;
	  string tmp;
      
	  for ( int i=0; i<6;++i)
	    {
	  
	      found=aValue.find(":");
	      tmp=aValue.substr(0,found);
	      aValue.erase(0,found+1);
	      m_N[i]=atof(tmp.c_str());

	      ODEBUG3("m_N["<<i<<"] : "<< m_N[i]);
	    }
	}
      else if(lParam == "PVAR")
	{
	  int found=0;
	  string tmp;
	  
	  m_P.resize(m_StateSize,m_StateSize);
	  m_P.setIdentity();

	  for ( int i=0; i<m_StateSize;++i)
	    {
	  
	      found=aValue.find(":");
	      tmp=aValue.substr(0,found);
	      aValue.erase(0,found+1);
	      m_P[i][i]=atof(tmp.c_str());

	      ODEBUG3("m_P["<<i<<"]["<<i<<"] : "<< m_P[i][i]);
	    }
	  
	}
      else if(lParam == "RTRACKER")
	{
	  int found=0;
	  string tmp;
	  	  
	  m_R.setIdentity();

	  for ( int i=0; i<m_StateSize;++i)
	    {
	  
	      found=aValue.find(":");
	      tmp=aValue.substr(0,found);
	      aValue.erase(0,found+1);
	      m_R[i][i]=atof(tmp.c_str());

	      ODEBUG3("m_R["<<i<<"]["<<i<<"] : "<< m_R[i][i]);
	    }
	}
      else if(lParam == "REINIT")
	{
	  if(aValue == "ON")
	    {
	      m_ReIntializedNMBT=true;
	    }
	  else if(aValue == "OFF")
	    {
	      m_ReIntializedNMBT=false;
	    }
	  else 
	    { 
	      cout << "Warning : KALMAN_REINIT unknown value :"<< aValue << endl; 
	      return -1;
	    }
	}

      else 
       {
	 cout << "Warning : unknown parameter :"<< lParam << endl; 
	 return -1;
       }
    }
  else
    {
      HRP2nmbtTrackingProcess::pSetParameter(aParameter,aValue);
    }

 
 
  return 0;
}


/*!------------------------------------- 
  Initialize the process. 
  -------------------------------------*/
int HRP2KalmanOnNMBTProcess:: pInitializeTheProcess()
{

  HRP2nmbtTrackingProcess:: pInitializeTheProcess();
  
  switch(m_StateType)
    {
    case VEL_CAM:
      m_StateModel=new SE3ModelVconstRc(m_N);
      break;

    case VEL_OBJ:
      m_StateModel=new SE3ModelVconstRo(m_N);
      break;
    case ACC_CAM:
      m_StateModel=new SE3ModelAtconstRo(m_N);
      break;
    case ACC_OBJ:
      m_StateModel=new SE3ModelAtconstRc(m_N);
      break;
    case COM_CAM:
      m_StateModel=new SE3ModelUvRo(m_N);
      break;
    case COM_OBJ:
      m_StateModel=new SE3ModelUvRc(m_N);
      break;
    };

  m_MeasureModel=new TrackerModel(m_R,m_StateSize);

  m_Kalman=new seKalman(m_MeasureModel,m_StateModel,m_Y,m_P);

  return 0;
}

/*!------------------------------------- 
  Realize the process 
-------------------------------------*/
int HRP2KalmanOnNMBTProcess::pRealizeTheProcess()
{
  //TODO cMo inversion when working in object frame

  double dt=0.033;
  vpColVector U;
  m_Kalman->prediction(dt,U);

  vpColVector lX(m_StateSize);
  lX=m_Kalman->getXpre();

  vpHomogeneousMatrix lcMo;
  ConvertCVectorToHMatrix(lX,lcMo);

  m_tracker.setcMo(lcMo);
 
  HRP2nmbtTrackingProcess::pRealizeTheProcess();

  ConvertHMatrixToCVector(m_outputcMo,m_Y);
  
  m_Kalman->update(m_Y);

  if(m_ReIntializedNMBT)
    {
      lX=m_Kalman->getXup();
      ConvertCVectorToHMatrix(lX,lcMo);

      m_tracker.init(*m_inputVispImage,lcMo);
    }

  return 0;
}

/*
  Clean up the process
*/
int  HRP2KalmanOnNMBTProcess::pCleanUpTheProcess()
{
    
  return 0;
} 

/*! Convert vpHomogeneousMatrix to vpColverctor*/
void HRP2KalmanOnNMBTProcess::ConvertHMatrixToCVector(const vpHomogeneousMatrix & aHM,
						      vpColVector &aCV)
{
  vpTranslationVector t;
  aHM.extract(t);

  vpRotationMatrix Rthu;
  aHM.extract(Rthu);

  vpThetaUVector ThU;
  ThU.buildFrom(Rthu);
  
  for(int i=0;i<3;i++)
    {
      aCV[i]=t[i];
      aCV[i+3]=ThU[i];		
    }

}


/*! Convert vpHomogeneousMatrix to vpColverctor*/
void HRP2KalmanOnNMBTProcess::ConvertCVectorToHMatrix(const vpColVector & aCV,
			      vpHomogeneousMatrix & aHM)
{
  vpTranslationVector t;
  vpThetaUVector ThU;

  for(int i=0;i<3;i++)
    {
      t[i]=aCV[i];
      ThU[i]=aCV[i+3];		
    }
  aHM.buildFrom(t,ThU);

}
