/**
 * StereoVisionクラス実装ファイル
 */


#include <iostream>
#include <stdio.h>
using namespace std;

#ifdef _ORBIX_
#include <OBE/CORBA.h>
#endif


#ifdef OMNIORB4
#include <omniORB4/CORBA.h>
#endif
#include <Corba/StereoVision_impl.h>

#include "oocmap.h"

#define ODEBUG2(x)
//#define ODEBUG3(x) cerr << "StereoVision_impl:" << x << endl

#if 0
#define ODEBUG(x) cerr << "StereoVision_impl:" <<  x << endl
#else
#define ODEBUG(x) 
#endif

//#define StereoVISION_DEBUG
// **********************************************************************
//
// StereoVisionクラス実装
//
// Keisuke Saito (Kernel Co.,Ltd.)
// version 0.2  (2002/01/24)
//
// **********************************************************************

using namespace llvs;

/**
 * コンストラクタ
 * @param   orb     ORBへの参照
 */
#ifdef USE_IMR
StereoVision_impl::StereoVision_impl(CORBA::ORB_ptr orb,PortableServer::POA_ptr poa,
				     LowLevelVisionServer *LLVS)
  : orb_(CORBA::ORB::_duplicate(orb)),
    poa_(PortableServer::POA::_duplicate(poa))
#else
    StereoVision_impl::StereoVision_impl(CORBA_ORB_ptr orb,
					 LowLevelVisionServer *LLVS) 
    : orb_(CORBA_ORB::_duplicate(orb))
#endif
{
  //#ifdef StereoVISION_DEBUG
  //#endif

  m_LLVS = LLVS;
  m_CommandsDir="/home/hrpuser/hrp2/OpenHRP2/StereoVision_rio/server";
  //m_CommandsDir="/home/hrpuser/hrp2/HRP2eyes/";
}

/**
 * デストラクタ
 */
StereoVision_impl::~StereoVision_impl()
{
#ifdef StereoVISION_DEBUG
  cout << "StereoVision_impl::~StereoVision_impl()" << endl;
#endif
}

#ifdef USE_IMR
PortableServer::POA_ptr
StereoVision_impl::_default_POA()
{
  return PortableServer::POA::_duplicate(poa_);
}
#endif


CORBA::Boolean 
StereoVision_impl::rbt2scmCalibStart(
) throw(CORBA::SystemException)
{
  string command = "rbt2scmCalibStart.sh";
  FILE    *fp;//, *popen();
  char buffer[500];// add 02/02/14 fukase

  sprintf(buffer,"cd %s;./%s",(char *)m_CommandsDir.c_str(),(char *)command.c_str());

  /* open the command ("/bin/ls") and get a file pointer */
  StopLLVSGrabbing();
  sleep(1);
  if ( (fp = popen( buffer, "r" )) == NULL ){
    cout << "process open error :" << command << endl;
    return false;
  }
  sleep(2);
  RestoreLLVSGrabbing();

  CORBA::Boolean  ret = false;
  int val = 0;
  fscanf(fp,"%d",&val);

  if(val != 0) {
    ret = true;
  }
    
  /* close the file pointer */
  pclose( fp );
    
  return ret;
}

CORBA::Boolean 
StereoVision_impl::rbt2scmCalibEnd(
) throw(CORBA::SystemException)
{
  string command = "rbt2scmCalibEnd.sh";
  FILE    *fp;//, *popen();
  char buffer[500];// add 02/02/14 fukase

  sprintf(buffer,"cd %s;./%s",(char *)m_CommandsDir.c_str(),(char *)command.c_str());

  /* open the command ("/bin/ls") and get a file pointer */
  StopLLVSGrabbing();
  sleep(1);
  if ( (fp = popen( buffer, "r" )) == NULL ){
    cout << "process open error :" << command << endl;
    return false;
  }
  sleep(2);
  RestoreLLVSGrabbing();

  CORBA::Boolean  ret = false;
  int val = 0;
  fscanf(fp,"%d",&val);

  if(val != 0) {
    ret = true;
  }
    
  /* close the file pointer */
  pclose( fp );
    
  return ret;
}


CORBA::Boolean 
StereoVision_impl::detectCrossMark(
				   const TransformQuaternion& robotHeadPos,
				   const TransformQuaternion& robotHandPos
				   )throw(CORBA::SystemException)
{
  string command = "detectCrossMark.sh";
  FILE    *fp;//, *popen();
  char buffer[500];
  
  sprintf(buffer,"%s/%s %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
	  (char *)m_CommandsDir.c_str(),
	  (char *)command.c_str(),
	  robotHeadPos.px,
	  robotHeadPos.py,
	  robotHeadPos.pz,
	  robotHeadPos.qx,
	  robotHeadPos.qy,
	  robotHeadPos.qz,
	  robotHeadPos.qw,
	  robotHandPos.px,
	  robotHandPos.py,
	  robotHandPos.pz,
	  robotHandPos.qx,
	  robotHandPos.qy,
	  robotHandPos.qz,
	  robotHandPos.qw
	  );
    
  /* open the command ("/bin/ls") and get a file pointer */

  StopLLVSGrabbing();
  sleep(1);    

  if ( (fp = popen(buffer, "r" )) == NULL ){
    cout << "process open error :" << command << endl;
    return false;
  }
  sleep(2);
  RestoreLLVSGrabbing();
  CORBA::Boolean  ret = false;
  int val = 0;
  fscanf(fp,"%d",&val);
    
  if(val != 0) {
    ret = true;
  }
    
  /* close the file pointer */
  pclose( fp );
    
  return ret;
}


//
// IDL:StereoVision/getObjectPosition:1.0
//
CORBA::Boolean 
StereoVision_impl::getObjectPosition(
				     const char* name,
				     const TransformQuaternion& robotHeadPos,
				     TransformQuaternion_out ObjectPosition
				     ) throw(CORBA::SystemException)
{
  FILE    *fp;//, *popen();
  char buffer[500];
  
  TransformQuaternion_var pp = new TransformQuaternion();
  pp->px = pp->py = pp->pz = 0.0;
  pp->qx = pp->qy = pp->qz = pp->qw = 0.0;
  
  sprintf(buffer,"cd %s; ./get%sPosition.sh %f %f %f %f %f %f %f", 
	  (char *)m_CommandsDir.c_str(),
	  name,
	  robotHeadPos.px,
	  robotHeadPos.py,
	  robotHeadPos.pz,
	  robotHeadPos.qx,
	  robotHeadPos.qy,
	  robotHeadPos.qz,
	  robotHeadPos.qw
	  );
  cout << buffer << endl;
  StopLLVSGrabbing();  
  sleep(1);
  /* open the command ("/bin/ls") and get a file pointer */
  if ( (fp = popen( buffer, "r" )) == NULL ){
    cout << "process open error :" << buffer << endl;
    ObjectPosition = pp;
    return false;
  }
  ODEBUG("I went through here");
  sleep(4);
  ODEBUG("I waited 2 secondes.");
  RestoreLLVSGrabbing();
  ODEBUG("Grabbing restored.");
  
  CORBA::Boolean  ret = false;
  int val = 0;
  fscanf(fp,"%d",&val);
  
  if(val != 0) {
    
    ret = true;
    
    float f0,f1,f2,f3,f4,f5,f6;
    fscanf(fp,"%f %f %f %f %f %f %f",
	   &f0,&f1,&f2,&f3,&f4,&f5,&f6);
    pp->px = f0;
    pp->py = f1;
    pp->pz = f2;
    pp->qx = f3;
    pp->qy = f4;
    pp->qz = f5;
    pp->qw = f6;
    
  }
  ObjectPosition = pp;
  
  /* close the file pointer */
  pclose( fp );
  
  return ret;
}

//
// IDL:StereoVision/StartProcess:1.0
//
CORBA::Boolean 
StereoVision_impl::StartProcess(const char* ProcessName)throw(CORBA::SystemException)
{


  FILE    *fp;//, *popen();
  char buffer[500];

  string sPN(ProcessName);

  /*  
      if (sPN.find("/")!=std::string::npos)
      sprintf(buffer,"%s", 
      ProcessName);
      else */
  sprintf(buffer,"cd %s; ./%s.sh", 
	  (char *)m_CommandsDir.c_str(),
	  ProcessName);
  
  cout << "Buffer: " << buffer << endl;
  /* open the command ("/bin/ls") and get a file pointer */

  StopLLVSGrabbing();

  sleep(1);
  if ( (fp = popen( buffer, "r" )) == NULL ){
    cout << "process open error :" << buffer << endl;
    return false;
  }
  sleep(2);
  RestoreLLVSGrabbing();

  CORBA::Boolean  ret = false;
 
  int val = 0;
  fscanf(fp,"%d",&val);
  
  if(val != 0) {
    
    ret = true;
    
  }
  
  /* close the file pointer */
  pclose( fp );
  
  return ret;

}

//
// IDL:StereoVision/StopProcess:1.0
//
CORBA::Boolean 
StereoVision_impl::StopProcess(const char* ProcessName)
  throw(CORBA::SystemException)
{
  return true;
}

//
// IDL:StereoVision/getImage:1.0
//
CORBA::Long 
StereoVision_impl::getImage(CORBA::Long CameraID,ImageData_out anImage,char*& Format)throw(CORBA::SystemException)
{
  return -1;
}


//
// IDL:StereoVision/getRangeMap:1.0
//
CORBA::Long 
StereoVision_impl::getRangeMap(RangeMap_out aRangeMap, char *&Format)throw(CORBA::SystemException)
{
  return -1;
}

void StereoVision_impl::StopLLVSGrabbing()
{
  m_LLVSGrabbingStatus = m_LLVS->ProcessStatus("IEEE1394 Image grabbing");
  if (m_LLVSGrabbingStatus)
    {
      m_LLVS->StopProcess("IEEE1394 Image grabbing");
    }
  
}

void StereoVision_impl::RestoreLLVSGrabbing()
{
  if (m_LLVSGrabbingStatus)
    {
      m_LLVS->StartProcess("IEEE1394 Image grabbing");
    }  
}
