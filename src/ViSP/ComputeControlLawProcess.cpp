/** @doc This object implements a visual process to get a disparity map.

    Copyright (c) 2010, 
    @author Stephane Embarki
   
    JRL-Japan, CNRS/AIST
    
    See license file for information on license.
*/
#include <Debug.h>

#include <math.h>

#include "ViSP/ComputeControlLawProcess.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdlib>


#include <sys/time.h>
#include <visp/vpConfig.h>
#include <visp/vpThetaUVector.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpTranslationVector.h>

#define SATURATE(x, s) (fabs(x) > s ? (x < 0 ? -s : s) : x)


using namespace std;
using namespace llvs;
/*!-------------------------------------
  Default constructor
  -------------------------------------*/

HRP2ComputeControlLawProcess::HRP2ComputeControlLawProcess()
{
  m_ProcessName ="ComputeControlLawProcess";

  m_FT= new vpFeatureTranslation(vpFeatureTranslation::cdMc);
  m_FThU= new vpFeatureThetaU(vpFeatureThetaU::cdRc);

  m_ProcessInitialized = false;

  m_ControlLawComputed = false;

  m_cdMoSet = false;

  m_Lambda = 0.6;

  m_nmbt = 0x0;

  m_CTS=0x0;

  m_ComputeV.resize(6);

  m_ModelHeightLimit =0.9;

  vpHomogeneousMatrix cameraMhead;

	
  ifstream file;

  string name="./data/ViSP/hrp2CamParam/cMh.3";
  file.open (name.c_str());

  for(int i=0; i<4; ++i)	
    {
      for(int j=0; j<4; ++j)	
	{
	  file>>cameraMhead[i][j];
	}
    }

  file.close();	
	
  m_headMcamera = cameraMhead.inverse();
    
  m_hVc.buildFrom(m_headMcamera);
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

/*! Set the ConnectionToSot  pointer */
void HRP2ComputeControlLawProcess::SetConnectionToSot (ConnectionToSot * aCTS)
{
  m_CTS = aCTS;
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

  ofstream aof;
  aof.open("dumpcommand.dat",ofstream::out);
  aof.close();
  return 0;
}

/*!------------------------------------- 
  Realize the process 
  -------------------------------------*/
int HRP2ComputeControlLawProcess::pRealizeTheProcess()
{
  m_ControlLawComputed = false;
  int r;
  ODEBUG("m_nmbt:" << (int)m_nmbt);
  if (m_nmbt->m_trackerTrackSuccess)
    {
      m_nmbt->GetOutputcMo(m_cMo);
  
      ODEBUG("m.cMo : "<<m_cMo);
 
      ODEBUG("m.cdMo : "<<m_cdMo);

      m_cdMc = m_cdMo*m_cMo.inverse();


      ODEBUG("m.cdMc : "<<m_cdMc);

      m_FT->buildFrom(m_cdMc) ;
      m_FThU->buildFrom(m_cdMc) ;

      vpColVector cVelocity(6);
      
      ODEBUG("Before Task.computecontroLaw!");
      cVelocity = m_Task.computeControlLaw() ;

      ODEBUG("Before SumSquare!");
      m_Error = m_Task.error.sumSquare();
      
      vpThetaUVector VthU(cVelocity[3],cVelocity[4],cVelocity[5]);

      ODEBUG("Before Vrxyz!");
      vpRxyzVector Vrxyz(VthU);

      cVelocity[3]=Vrxyz[0];
      cVelocity[4]=Vrxyz[1];
      cVelocity[5]=Vrxyz[2];

      

      r=0;

      m_ControlLawComputed = true;

      double headprpy[6];
      double waistprpy[6];
      
      // Read nom information from SoT.
      if (m_CTS!=0)
	{
	  m_CTS->ReadHeadRPYSignals(headprpy);
	  m_CTS->ReadWaistRPYSignals(waistprpy);
	}
      
      // Transfer velocity from cemra to waist frame
      vpRxyzVector headRxyz (headprpy[3],headprpy[4],headprpy[5]);
      vpRxyzVector waistRxyz(waistprpy[3],waistprpy[4],waistprpy[5]);

      vpThetaUVector headThU (headRxyz);
      vpThetaUVector waistThU (waistRxyz);
      

      vpTranslationVector  headT (headprpy[0],headprpy[1],headprpy[2]);
      vpTranslationVector  waistT(waistprpy[0],waistprpy[1],waistprpy[2]);

      vpHomogeneousMatrix  fMh (headT ,headThU);
      vpHomogeneousMatrix  fMw (waistT, waistThU);

      ODEBUG( "headT" << fMh);

      ODEBUG(   "waistT"<<endl << fMw);

      vpHomogeneousMatrix wMh = fMw.inverse()*fMh;

      vpTwistMatrix wVh (wMh);

      
      vpHomogeneousMatrix fMo = fMh*m_headMcamera*m_cMo;

      ODEBUG3(  "fMo[2][3]: \n"<<fMo[2][3]);

      if(fMo[2][3]<m_ModelHeightLimit)
	{
	  m_ComputeV = wVh*m_hVc *cVelocity;
	}
      else
	{
	  m_ComputeV = 0;
	  m_ComputeV[0]=0.001;
	  r=-1;
	}
    }
  else
    {
 
      m_ComputeV = 0;
      m_ComputeV[0] = 0.001;
      
      r=-1;
    }

  if(r==-1)
    m_Computing = 0;

  double velref[3];
  
  velref[0]=SATURATE( m_ComputeV[0], 0.2);
  velref[1]= SATURATE( m_ComputeV[1], 0.1);
  velref[2]= SATURATE(m_ComputeV[5], 0.18);


  if (m_CTS!=0)
    {
      m_CTS-> WriteVelocityReference(velref);
    }


  ODEBUG("Staying alive !");

#if 1
  ofstream aof;
  aof.open("dumpcommand.dat",ofstream::app);
  struct timeval atv;
  gettimeofday(&atv,0);
  aof.precision(15);
  aof << atv.tv_sec + 0.000001 * atv.tv_usec << " " << m_ComputeV.t() << endl;
  aof<< velref[0]<< "  "<< velref[1]<< "  "<<velref[2]<< endl;
  aof.close();

#endif

  ODEBUG("Going out of ComputeControlLawProcess !");
  return r;
 
}

int  HRP2ComputeControlLawProcess::pCleanUpTheProcess()
{
    
  return 0;
} 
