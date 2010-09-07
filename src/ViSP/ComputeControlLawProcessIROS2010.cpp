/** @doc This object implements a visual process to get a disparity map.

    Copyright (c) 2010, 
    @author Stephane Embarki, 
    C. Dune,
   
    JRL-Japan, CNRS/AIST
    
    See license file for information on license.
*/
#include <llvs/tools/Debug.h>

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

inline double min (double a, double b)
{ return (a < b ? a : b); }


using namespace std;
using namespace llvs;
/*!-------------------------------------
  Default constructor
  -------------------------------------*/

HRP2ComputeControlLawProcess::HRP2ComputeControlLawProcess()
{
  m_ProcessName ="ComputeControlLawProcess";
  init();
}



/*!-------------------------------------
  Destructor
  ------------------------------------- */
HRP2ComputeControlLawProcess:: ~HRP2ComputeControlLawProcess()
{
  delete m_FT;
  delete m_FThU;
 
}


int HRP2ComputeControlLawProcess::init()
{
 
  m_FT= new vpFeatureTranslation(vpFeatureTranslation::cdMc);
  m_FThU= new vpFeatureThetaU(vpFeatureThetaU::cdRc);
  m_Lambda = 0.6;
  m_nmbt = 0x0;
  m_CTS=0x0;
  m_ProcessInitialized = false;
  m_ControlLawComputed = false;
  m_RealiseControlLaw= true;
  m_cdMoSet = false;

  /* default motion test*/
  m_MotionTested=FREE;
  m_ModelHeightLimit=0;
  m_RxLimit=0;
  m_RyLimit=0;

  m_Velmax.resize(3);
  m_Velmax=0.1;
  m_Velzero.resize(3);
  m_Velzero=0.02;

  // load 
  vpHomogeneousMatrix cameraMhead;
  loadcMh(cameraMhead);
	
 
  m_headMcamera = cameraMhead.inverse();
  m_hVc.buildFrom(m_headMcamera);
   
  m_ModelHeightLimit =0.9;

  /*! Set to zero the integral term. */
  m_Lbk=0.0;
  return 0 ;
}


int HRP2ComputeControlLawProcess::loadcMh(vpHomogeneousMatrix& cMh)
{
 

  //TODO : this file name shouldn't be written here
  ifstream file;
  string name="./data/ViSP/hrp2CamParam/cMh.3";
  file.open (name.c_str());
  
  try
    {
      cMh.load(file);
    }
  catch(vpException a) // file doesn't exist
    { 
      cout << "---- Open Exception Default---------------"<<endl;
      cout<<"----Wide camera extrinsic parameters ---- " <<endl;
      vpCTRACE << endl <<a;
      cout << "------------------ " <<endl;
      // fill a defaut matrix
      cMh[0][0]=0.016937505;   cMh[0][1]=-0.9997821632;  cMh[0][2]= -0.01217327283; cMh[0][3]=-0.05195444004;
      cMh[1][0]=-0.1698427574; cMh[1][1]=0.009121740254; cMh[1][2]= -0.9854286431;  cMh[1][3]= 0.1036017168 ;
      cMh[2][0]=0.9853256992 ; cMh[2][1]=0.0187590039;   cMh[2][2]= -0.1696511946;  cMh[2][3]=-0.04729590458 ;
      cMh[3][0]=0; cMh[3][1]=0;cMh[3][2]= 0;  cMh[3][3]=1 ;
      return -1;
    } 
    
  file.close();
  return 0;
}


/*!-------------------------------------
  Sets the parameters

  The parameter names can be :

  LAMBDA

  MOTION_FREE
  MOTION_GROUND
  MOTION_PLAN

  VEL_MAX
-------------------------------------*/
int HRP2ComputeControlLawProcess::pSetParameter(std::string aParameter, 
					       std::string aValue)
{
  // use of the generic function to add the parameter in the parameter list
  // A parameter can be or cannot be associated with a value, 
  // thus an empty string for Value is correct.
  // If the parameter already exist is value is overwritten. 
  // If this is valid the index parameter >=0 is returned,
  // -1 otherwise.
 
  ODEBUG("Set parameter in CCL  ");

  if (aParameter=="LAMBDA")
    {
     
      m_Lambda= atof(aValue.c_str());
      
      ODEBUG("LAMBDA : "<< m_Lambda);
      
      m_Task.setLambda( m_Lambda);
    }
  else if (aParameter=="MOTION_FREE")
    { 
      SetMotionTest(FREE);
    }
  else if (aParameter=="MOTION_GROUND")
    { 
      int found=0;
      string tmp;
      vector<double> limit(3);

      for ( int i=0; i<3;++i)
	{
	  found=aValue.find(":");
	  tmp=aValue.substr(0,found);
	  aValue.erase(0,found+1);
	  limit[i]=atof(tmp.c_str());
	  ODEBUG("limit["<<i<<"] : "<<limit[i]);
	}
                 
      SetMotionTest(ON_GROUND,limit);
    }
 else if (aParameter=="MOTION_PLAN")
    { 
      int found=0;
      string tmp;
      vector<double> limit(3);

      for ( int i=0; i<3;++i)
	{
	  
	  found=aValue.find(":");
	  tmp=aValue.substr(0,found);
	  aValue.erase(0,found+1);
	  limit[i]=atof(tmp.c_str());

	  ODEBUG("limit["<<i<<"] : "<<limit[i]);
	}
                 
      SetMotionTest(PLAN_MOTION,limit);
    }
 else if (aParameter=="MOTION_HL")
    { 
 
      vector<double> limit(1);
      limit[0]=atof(aValue.c_str());

      ODEBUG("limit[0] : "<<limit[0]);
                       
      SetMotionTest(HEIGHT_LIMITED,limit);
    }
 else if (aParameter=="VEL_MAX")
    { 
      int found=0;
      string tmp;
      
      for ( int i=0; i<3;++i)
	{
	  
	  found=aValue.find(":");
	  tmp=aValue.substr(0,found);
	  aValue.erase(0,found+1);
	  m_Velmax[i]=atof(tmp.c_str());

	  ODEBUG("Velmax["<<i<<"] : "<< m_Velmax[i]);
	}
             
    }
 else if (aParameter=="VEL_ZERO")
    { 
      int found=0;
      string tmp;
      
      for ( int i=0; i<3;++i)
	{
	  
	  found=aValue.find(":");
	  tmp=aValue.substr(0,found);
	  aValue.erase(0,found+1);
	  m_Velzero[i]=atof(tmp.c_str());

	  ODEBUG("Velzero["<<i<<"] : "<< m_Velmax[i]);
	}
             
    }
 else if (aParameter=="INTERNAL_STATE")
    { 
      m_internalState=aValue;             
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

  double headprpy[6];
  double waistprpy[6];
 
  // Read nom information from SoT.
  if (m_CTS!=0)
    {
      m_CTS->ReadHeadRPYSignals(headprpy);
      
      vpRxyzVector headInFootRxyz(headprpy[3],	
				  headprpy[4],
				  headprpy[5]);
      
      vpThetaUVector headInFootThU     (headInFootRxyz);
      
      vpTranslationVector  headInFootT (headprpy[0],
					headprpy[1],
					headprpy[2]); 
      
      vpHomogeneousMatrix  fMh (headInFootT ,headInFootThU);
      
      m_LastfMo = fMh*m_headMcamera*m_cdMo;
      
      m_IninitHeight = m_LastfMo[2][3];
    }
  else
    {
      m_IninitHeight = 0;
    }

  m_RealiseControlLaw=true;
}

/*! Set the ConnectionToSot  pointer */
void HRP2ComputeControlLawProcess::SetConnectionToSot (ConnectionToSot * aCTS)
{
  m_CTS = aCTS;
}

 /*! Set the type of test on motion*/
void  HRP2ComputeControlLawProcess::SetMotionTest(typeMotion aMotion,
						  const vector<double> &limits)
{
  m_MotionTested = aMotion;
  
  if(m_MotionTested == HEIGHT_LIMITED && limits.size()==1)
    {
      m_ModelHeightLimit = limits[0] ;
    }
  else if (m_MotionTested == ON_GROUND && limits.size()==3 )
    {
      m_ModelHeightLimit = limits[0] ;
      m_RxLimit = limits[1] ; 
      m_RyLimit = limits[2] ;
    }
  else if (m_MotionTested == PLAN_MOTION && limits.size()==3 )
    {
      m_ModelHeightLimit = limits[0] ;
      m_RxLimit = limits[1] ; 
      m_RyLimit = limits[2] ;
    }

  else 
    {
      cout << " The vector<double> size doesn't match with the typeMotion"<<endl;
    }
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

#if 1

  ofstream aof;
  vpTranslationVector cdTo(m_cdMo[0][3],m_cdMo[1][3],m_cdMo[2][3]);
  vpThetaUVector cdThUo(m_cdMo);
  vpRxyzVector cdRxyzo (cdThUo);

  aof.open("dumpcommand.dat",ofstream::out);
  aof <<"# cdMo :  "<<cdThUo.t()<< "  "<<cdRxyzo.t() <<endl
      <<"# TimeStamp (1 value) / Pose cMo(6 values)/ |Error| (1 value)"
      <<" / Error Vectot (6 values)/ Pose fMo(6 values) "
      <<"/ Control in camera Frame(6 values) / Control in waist Frame(6 values)"
      <<"/ Control send to SoT (3 values) / Control from SoT in ref frame (2 values) "
      <<"/ Theta out pg (1 value)/ Control from SoT in robot frame (2 values) " <<endl;
  
  aof.close();

#endif

  return 0;
}

/*!------------------------------------- 
  Realize the process 
  -------------------------------------*/
int HRP2ComputeControlLawProcess::pRealizeTheProcess()
{
  m_ComputeV.resize(6);
  m_ControlLawComputed = false;
  int r=-1;
  
  // store the velocity in camera frame
  vpColVector cVelocity(6);
  
  // store the velocity in waist frame
  vpColVector wVelocity(6);

  // store the velocity send to SoT
  double velref[3];

  vpHomogeneousMatrix fMo;


  // the stop criterion is based on the infinity norm of
  // a 3ddl vector corresponding to the 3 controled ddl X [0],Z[2] and Ry[4]
  // in the camera frame
  vpColVector error3ddl(3);
  double error3ddlInfinityNorm=100;
  double errorThreshold=0.1;
  
  if ( m_nmbt->m_trackerTrackSuccess )
    {
      m_nmbt->GetOutputcMo(m_cMo);
      ODEBUG("m.cMo : "<<m_cMo);
      ODEBUG("m.cdMo : "<<m_cdMo);
      m_cdMc = m_cdMo*m_cMo.inverse();

      ODEBUG("m.cdMc : "<<m_cdMc);
      m_FT->buildFrom(m_cdMc) ;
      m_FThU->buildFrom(m_cdMc) ;
      
  
      ODEBUG("Before Task.computecontroLaw!");
      cVelocity = m_Task.computeControlLaw() ;
      
      vpMatrix lL = m_Task.L;
      
      ODEBUG("Before SumSquare!");
      m_Error = m_Task.error.sumSquare();
      
      // build the 3ddl error vector from the 6 ddl one
      error3ddl[0]= m_Task.error[0];
      error3ddl[1]= m_Task.error[2];
      error3ddl[2]= m_Task.error[4];
      error3ddlInfinityNorm=error3ddl.infinityNorm();
                  
      // express the rotation as rx ry rz
      vpThetaUVector VthU(cVelocity[3],cVelocity[4],cVelocity[5]);
      ODEBUG("Before Vrxyz!");
      vpRxyzVector Vrxyz(VthU);
      cVelocity[3]=Vrxyz[0];
      cVelocity[4]=Vrxyz[1];
      cVelocity[5]=Vrxyz[2];

      // Compute control ok before testing
      r=0;
      m_ControlLawComputed = true;
      double headprpy[6];
      double waistprpy[6];
      
      // Get the current robot frames from SoT.
      if (m_CTS!=0)
	{
	  m_CTS->ReadHeadRPYSignals(headprpy);
	  m_CTS->ReadWaistRPYSignals(waistprpy);
	
      
	  //Create homogeneousMatrix to store the head position in foot frame   
	  vpHomogeneousMatrix  fMh;


	  // Change the velocity frame from camera to waist
	  changeVelocityFrame(cVelocity,
			      wVelocity,
			      headprpy,
			      waistprpy,
			      fMh);
      
	  //compute the obect  position in foot frame 
	  fMo = fMh*m_headMcamera*m_cMo;

  
	  if(TestObjectMotion(fMo))
	    {
	      m_ComputeV = wVelocity;
	    }
	  else
	    {
	      cerr << "Error in Compute control law >> object motion out of limit !!!" << endl; 
	      r=-1;
	    }
	}
      else 
	{
	  ODEBUG3("Unable to contact SoT !!"); 
	  r=-2;
	}
    }
  else
    {
       
      ODEBUG3( "Error in Compute control law >> the tracking failed !!!"); 
      r=-3;
    }

  VelocitySaturation(m_ComputeV,velref);


  // Test the stop criteria
  if(error3ddlInfinityNorm<errorThreshold)
    {
      cerr << "Compute control law >> Finish : " << error3ddlInfinityNorm << std::endl;
      r=-4;
    }


  if(r<0 ||  m_RealiseControlLaw==false)
    {
      ODEBUG3("r=" << r << " m_RealiseControlLaw = "<< m_RealiseControlLaw);
      stop(velref);
      m_RealiseControlLaw=false;
    }


  double waistcom[3];
  double comattitude[3];
  

 
  if (m_CTS!=0)
    {
      ODEBUG3("velref : " << velref[0] << " "<< velref[1] << " "<< velref[2] );
      m_CTS-> WriteVelocityReference(velref);

      m_CTS-> ReadWaistComSignals(waistcom);

      m_CTS-> ReadComAttitudeSignals(comattitude);
    }
  else
    {
      for (int i=0; i<3;++i)
	{
	  waistcom[i]=0;
	  comattitude[i]=0;
	}
    }


  ODEBUG("Staying alive !");
#if 1
  
  vpTranslationVector cTo(m_cMo[0][3],m_cMo[1][3],m_cMo[2][3]);
  vpThetaUVector cThUo(m_cMo);
  vpRxyzVector cRxyzo (cThUo);


  vpTranslationVector fTo(fMo[0][3],fMo[1][3],fMo[2][3]);
  vpThetaUVector fThUo(fMo);
  vpRxyzVector fRxyzo (fThUo);


  /*Change of frame from ref to robot  */
  vpTranslationVector refTcom;
  //Only z rotation
  vpRxyzVector   refRxyzcom(0,0,comattitude[2]);
  vpThetaUVector refThUcom(refRxyzcom);
  vpHomogeneousMatrix refMcom(refTcom,refThUcom);

  vpVelocityTwistMatrix comVref(refMcom.inverse());

  vpColVector waistCtlVelinRef(6);
  waistCtlVelinRef[0]=waistcom[0];
  waistCtlVelinRef[1]=waistcom[1];
  
  vpColVector waistCtlVelRobot(6);
  waistCtlVelRobot=comVref*waistCtlVelinRef;
  
  ofstream aof;
  aof.open("dumpcommand.dat",ofstream::app);
  struct timeval atv;
  gettimeofday(&atv,0);
  aof.precision(15);
  aof << atv.tv_sec + 0.000001 * atv.tv_usec << "  "
      << cTo.t()<<"  "<< cRxyzo.t()<<"  "<<m_Error<<"  "
      <<  m_Task.error.t() << fTo.t()<<"  "<< fRxyzo.t()<<"  "
      << cVelocity[0]<<"  "<<cVelocity[1]<<"  "<<cVelocity[2]<<"  "
      << cVelocity[3]<<"  "<<cVelocity[4]<<"  "<<cVelocity[5]<<"  ";

  if (m_CTS!=0)
    {
      aof << wVelocity[0]<<"  "<<wVelocity[1]<<"  "<<wVelocity[2]<<"  "
	  << wVelocity[3]<<"  "<<wVelocity[4]<<"  "<<wVelocity[5]<<"  "
	  << velref[0]<<"  "<< velref[1]<<"  "<<velref[2]<<"  "
	  << waistcom[0] <<"  " << waistcom[1]<<"  "<< comattitude[2]<<"  "
	  << waistCtlVelRobot[0]<<"  " <<waistCtlVelRobot[1]<<"  "<<endl;
    }
  else
    {
      aof <<endl;
    }
  aof.close();

#endif

  return r;
 
}

/*
  Clean up the process
*/
int  HRP2ComputeControlLawProcess::pCleanUpTheProcess()
{

  double* velref;
  velref=new double[3];
  
  stop(velref);
  if (m_CTS!=0)
    {
      m_CTS-> WriteVelocityReference(velref);
    }

  return 0;
} 

/*
 Change the velocity frame
 
 \brief :
 The velocituy results from the visual servoing control
 law. It is basically expressed in the camera frame
 Using the current measurements, we can compute the
 transformation between the camera frame in the waist
 
 wMc and the correponding Twist wVc
 
 and use it to change the velocity frame.

 
 return : 0  if ok
          -1 if the input velocity is not of size 6
          
  
 warning : attention nothing garanties the size 
           of   poseHeadInFoot and poseWaistInFoot

 */
int  HRP2ComputeControlLawProcess::changeVelocityFrame(const vpColVector& velCam,
						       vpColVector& velWaist,
						       const double *poseHeadInFoot,
						       const double *poseWaistInFoot,
						       vpHomogeneousMatrix &afMh )
{
  // test on the input velocity
  if(velCam.getRows()!=6)
    {
      return -1;
    }
  
  velWaist.resize(6); 
  
  //Express rotation in theta U 
  vpRxyzVector headInFootRxyz      (poseHeadInFoot[3],
				    poseHeadInFoot[4],
				    poseHeadInFoot[5]);
  vpRxyzVector waistInFootRxyz     (poseWaistInFoot[3],
				    poseWaistInFoot[4],
				    poseWaistInFoot[5]);
  vpThetaUVector headInFootThU     (headInFootRxyz);
  vpThetaUVector waistInFootThU    (waistInFootRxyz);

  //Create translation vectors      
  vpTranslationVector  headInFootT (poseHeadInFoot[0],
				    poseHeadInFoot[1],
				    poseHeadInFoot[2]);
  vpTranslationVector  waistInFootT(poseWaistInFoot[0],
				    poseWaistInFoot[1],
				    poseWaistInFoot[2]);

  //Create the corresponding homogeneousMatrix    
  vpHomogeneousMatrix  fMh (headInFootT ,headInFootThU);
  vpHomogeneousMatrix  fMw (waistInFootT, waistInFootThU);
 
  afMh=fMh;

  //Compute the position of the head in the waist
  vpHomogeneousMatrix wMh = fMw.inverse()*fMh;
 
  //Compute the associate twist matrix
  vpVelocityTwistMatrix wVh (wMh);
 
  //The position of the current camera in the head should
  // have been previously loaded
  //Change the velocity frame
  velWaist = wVh*m_hVc*velCam;
  
  return 0;
}

/*! Test on object plan  Motion */
bool HRP2ComputeControlLawProcess::planMotion(const vpHomogeneousMatrix &afMo)
{
  
  vpHomogeneousMatrix olMo = m_LastfMo.inverse()* afMo;
 
  vpThetaUVector olThUo(olMo);
  vpRxyzVector   olRxyzo(olThUo);
  
  if((fabs(olRxyzo[0])< m_RxLimit) &&
     (fabs(olRxyzo[1])< m_RyLimit) &&
     (fabs(olMo[2][3])< m_ModelHeightLimit ))
 
    {
      return true;
    }
  else
    {
       return false;
    }
}



bool HRP2ComputeControlLawProcess::objectOnGround(const vpHomogeneousMatrix &afMo)
{
  if (heightInLimit(afMo))
    {
      vpThetaUVector objectInFootThU(afMo);
      vpRxyzVector   objectInFootRxyz(objectInFootThU);

      if( fabs(objectInFootRxyz[0])< m_RxLimit
	 && fabs(objectInFootRxyz[1])< m_RyLimit )
	{
	  return true;
	}
      else
	{
	  ODEBUG3(endl<<"|Rx| =" << fabs( objectInFootRxyz[0]) <<"<"<< m_RxLimit
		  <<endl<<"OR"<<endl
		  <<"|Ry| =" << fabs(objectInFootRxyz[1]) <<"<"<<m_RyLimit<<endl);

	  return false;
	}
    }
  else
    {
      return false; 
    }
}


bool HRP2ComputeControlLawProcess::heightInLimit(const vpHomogeneousMatrix &afMo)
{
 
  if( fabs( afMo[2][3] - m_IninitHeight)< m_ModelHeightLimit )
    {
      return true;
    }
  else
    {
      ODEBUG3(  "afMo[2][3]- m_IninitHeight: \n"<<afMo[2][3]- m_IninitHeight);
      return false;
    }
}


/*! Test the object Motion */
bool HRP2ComputeControlLawProcess::TestObjectMotion(const vpHomogeneousMatrix &afMh)
{
  bool r = false;

  if (m_MotionTested == FREE)
    {
      r =true;
    }
  else if(m_MotionTested == HEIGHT_LIMITED)
    {
      r=heightInLimit(afMh);
    }
  else if (m_MotionTested == ON_GROUND)
    {
      r=objectOnGround(afMh);
    }
  return r; 
}

/*! Velocity saturation

   RawVel is the velocity expressed in the waist frame 6ddl : Vx Vy Vz ThetaUx ThetaUy ThetaUz
   VelRef in the saturated velocity expressed in the waist frame 3ddl: Vx Vy Rz

*/
int HRP2ComputeControlLawProcess::VelocitySaturation(const vpColVector &RawVel,double * VelRef )
{
  vpColVector dv(0.05*m_Velmax);
  vpColVector Vinf  = m_Velmax-dv;
  vpColVector Vsup  = m_Velmax+dv;
 

  ///compute the 3ddl vector corresponding to the input
  vpColVector RawVel3ddl(3);
  RawVel3ddl[0]= RawVel[0];
  RawVel3ddl[1]= RawVel[1];
  RawVel3ddl[2]= RawVel[5];


  // temporary abs value of the input velocity
  double absRawVel;
  // normalization factor
  double fac = 1; 
  // for all the coeff
  for (int i=0; i<3;++i)
    {   
      absRawVel=fabs(RawVel3ddl[i]);
      
      fac = min(fabs(fac),m_Velmax[i]/(absRawVel+0.00001));
  
      // to prevent from discontinuities
      if( (Vinf[i]<=absRawVel) & (absRawVel <= Vsup[i]) )
	{
	  double newfac = 1/(2*dv[i]*absRawVel)*
	    ((absRawVel-Vinf[i])*m_Velmax[i]
	     +(Vsup[i]-absRawVel)*Vinf[i]);

	  fac  = min(fabs(fac),fabs(newfac));
	}
    }
  
for (int i=0; i<3;++i)
    {   
      VelRef[i]=RawVel3ddl[i]*fac;
    }
 return 0;
}

/*! Put Velocity value at zero when lower than 0.02  */
int HRP2ComputeControlLawProcess::stop(double * VelRef)
{
  m_RealiseControlLaw=false;
  for (int i=0;i<3;i++)
    VelRef[i]=0;
  return 0;
}

int HRP2ComputeControlLawProcess::ZeroVelocity(double * VelRef)
{

  if(VelRef[0]< m_Velzero[0] && VelRef[1]<m_Velzero[1] && VelRef[2]<m_Velzero[2])
    {
      VelRef[0]=0.00001;
      VelRef[1]=0;
      VelRef[2]=0;
    }
  else if (VelRef[0]<m_Velzero[0])
    {
      VelRef[0]=0;
    }
  else if (VelRef[1]<m_Velzero[1])
    {
      VelRef[1]=0;
    }
  else if (VelRef[2]<m_Velzero[2])
    {
      VelRef[2]=0;
    }
  
  return 0;
}
