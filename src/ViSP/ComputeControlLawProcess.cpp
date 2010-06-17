/** @doc This object implements a visual process to get a disparity map.

    Copyright (c) 2010, 
    @author Stephane Embarki
   
    JRL-Japan, CNRS/AIST
    
    See license file for information on license.
*/
#include <Debug.h>

#include "ViSP/ComputeControlLawProcess.h"
#include <iostream>
#include <sstream>
#include <cstdlib>

#include <visp/vpConfig.h>


/*!-------------------------------------
  Default constructor
  -------------------------------------*/

HRP2ComputeControlLawProcess::HRP2ComputeControlLawProcess()
{
   m_ProcessName ="ComputeControlLawProcess";

  m_FT= new vpFeatureTranslation(vpFeatureTranslation::cdMc);
  m_FThU= new vpFeatureThetaU(vpFeatureThetaU::cdRc);

  m_ProcessInitialized = false;

  m_cdMoSet = false;

  m_Lambda = 0.6;

}



/*!-------------------------------------
  Destructor
  ------------------------------------- */
HRP2ComputeControlLawProcess:: ~HRP2ComputeControlLawProcess()
{
  delete m_FT;
  delete m_FThU;
 
}



/*!-------------------------------------
  Sets the parameters

  The parameter names can be :

  LAMBDA

  -------------------------------------*/
int HRP2ComputeControlLawProcess::SetParameter(std::string aParameter, 
					      std::string aValue)
{
  // use of the generic function to add the parameter in the parameter list
  // A parameter can be or cannot be associated with a value, 
  // thus an empty string for Value is correct.
  // If the parameter already exist is value is overwritten. 
  // If this is valid the index parameter >=0 is returned,
  // -1 otherwise.
 
  
  if (aParameter=="LAMBDA")
    {
     
      m_Lambda= atof(aValue.c_str());
      
      m_Task.setLambda( m_Lambda);
    }
  else 
    {
      cout << "Warning : unknown parameter :"<< aParameter << endl; 
      return -1;
    }
  
   return 0;
}

/*! Set the nmbt tracker pointer */
void HRP2ComputeControlLawProcess::SetTracker(HRP2nmbtTrackingProcess* anmbt)
{
  m_nmbt = anmbt;
}

/*! Set the cdMo */
void HRP2ComputeControlLawProcess::SetcdMo ( vpHomogeneousMatrix acdMo)
{
  m_cdMo=acdMo;
  m_cdMoSet = true;
}

/*! Get the cdMc */
void  HRP2ComputeControlLawProcess::GetcdMc ( vpHomogeneousMatrix &acdMc)
{
  acdMc=m_cdMc;
}

/*! Get the ComputeV */
void  HRP2ComputeControlLawProcess::GetComputeVelocity ( vpColVector &aCV)
{
  aCV=m_ComputeV;
}

/*! Get the Error */
void  HRP2ComputeControlLawProcess::GetError ( double &aError)
{
  aError=m_Error;
}


 /*! Start the process */
int HRP2ComputeControlLawProcess::pStartProcess()
{
  if (m_cdMoSet == true)
    {
      if ( m_ProcessInitialized == false)
	{
	  pInitializeTheProcess();
	}
    }
  else
    {
      m_Computing = 0;
      cout<< "cdMo must be set to start the compute controle law process"; 
      return -1;
    }
  return 0;
}



/*!------------------------------------- 
  Initialize the process. 
  -------------------------------------*/
int HRP2ComputeControlLawProcess:: pInitializeTheProcess()
{
  m_Task.setServo(vpServo::EYEINHAND_CAMERA);
  
  m_Task.setInteractionMatrixType(vpServo::CURRENT);

  m_Task.setLambda( m_Lambda);

  m_cdMc =m_cdMo*m_cMo.inverse() ;


  m_FT->buildFrom(m_cdMc);
  m_FThU->buildFrom(m_cdMc);
	
  m_Task.addFeature( *m_FT) ;
  m_Task.addFeature(*m_FThU) ;

#if 1
  m_Task.print() ;
#endif
  
  m_ProcessInitialized =true;
  
  return 0;
}

/*!------------------------------------- 
  Realize the process 
   -------------------------------------*/
int HRP2ComputeControlLawProcess::pRealizeTheProcess()
{
  m_nmbt->GetOutputcMo(m_cMo);
  
  cout<<"m.cMo : "<<m_cMo<<endl;
 
  cout<<"m.cdMo : "<<m_cdMo<<endl;

  m_cdMc = m_cdMo*m_cMo.inverse();


  cout<<"m.cdMc : "<<m_cdMc<<endl;

  m_FT->buildFrom(m_cdMc) ;
  m_FThU->buildFrom(m_cdMc) ;
  m_ComputeV = m_Task.computeControlLaw() ;

  m_Error = m_Task.error.sumSquare();

  return 0;
 
}

int  HRP2ComputeControlLawProcess::pCleanUpTheProcess()
{
    
  return 0;
} 
