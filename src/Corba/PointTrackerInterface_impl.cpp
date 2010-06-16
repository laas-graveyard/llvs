/** @doc This object implements a visual process to get a disparity map.

    Copyright (c) 2010, 
    @author Stephane Embarki
   
    JRL-Japan, CNRS/AIST
    
    See license file for information on license.
*/


#include <iostream>
#include <stdio.h>
using namespace std;

#ifdef OMNIORB4
#include <omniORB4/CORBA.h>
#endif
#include <Corba/PointTrackerInterface_impl.h>


#include "llvsConfig.h"

#if   (LLVS_HAVE_VISP>0)
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpPoint.h>
#include <visp/vpImagePoint.h>
#endif



using namespace llvs;

PointTrackerInterface_impl::PointTrackerInterface_impl(LowLevelVisionServer * LLVS)
{
  m_LLVS = LLVS;
}

PointTrackerInterface_impl::~PointTrackerInterface_impl()
{
   
}



CORBA::Boolean
PointTrackerInterface_impl::Init(const PointTrackerInterface::DataTarget & aData)
{

#if (LLVS_HAVE_VISP>0)
  if ((aData.Target.length()/3)==(aData.UV.length()/2))
    {
      int nbPoint = aData.UV.length()/2;
      
      vector<vpPoint> aTarget;
      aTarget.resize(nbPoint);
      
       vector<vpImagePoint*> vpIP;
       vpIP.resize(nbPoint);
       for(int i=0;i<nbPoint;++i)
	 {
	   vpIP[i]=new vpImagePoint;
	 }
      
       for(int i =0;i<nbPoint;++i )
	{
	  vpIP[i]->set_u(aData.UV[i]);
	  vpIP[i]->set_v(aData.UV[i+1]);
	  aTarget[i].setWorldCoordinates (aData.Target[i],
					 aData.Target[i+1],
					  aData.Target[i+2]);
	}
      
       m_LLVS->m_PointTrackerProcess->Init(aTarget,
					     vpIP,
					     nbPoint);
      
      return true;
    }
  else 
    return false;
  
#else

    cout<< " Need ViSP to use SetcMo function"<< endl;
   return false;
#endif



}
CORBA::Boolean
PointTrackerInterface_impl::GetcMo(PointTrackerInterface::HomogeneousMatrix& acMo)
{
  /*
#if (LLVS_HAVE_NMBT>0)


   m_LLVS->m_CBonNMBT->ReadData(m_CBTD);
  

  acMo.cMo[0][0]=m_CBTD.cMo[0][0];
  acMo.cMo[0][1]=m_CBTD.cMo[0][1];
  acMo.cMo[0][2]=m_CBTD.cMo[0][2];
  acMo.cMo[0][3]=m_CBTD.cMo[0][3];
  acMo.cMo[1][0]=m_CBTD.cMo[1][0];
  acMo.cMo[1][1]=m_CBTD.cMo[1][1];
  acMo.cMo[1][2]=m_CBTD.cMo[1][2];
  acMo.cMo[1][3]=m_CBTD.cMo[1][3];
  acMo.cMo[2][0]=m_CBTD.cMo[2][0];
  acMo.cMo[2][1]=m_CBTD.cMo[2][1];
  acMo.cMo[2][2]=m_CBTD.cMo[2][2];
  acMo.cMo[2][3]=m_CBTD.cMo[2][3];
  acMo.cMo[3][0]=0;
  acMo.cMo[3][1]=0;
  acMo.cMo[3][2]=0;
  acMo.cMo[3][3]=1;
  return true;

#else

    cout<< " Need NMBT to use GetcMo function"<< endl;
    return false;
#endif
  */
  return false;
}

CORBA::Boolean
PointTrackerInterface_impl::GetDebugInfoObject(PointTrackerInterface::DebugInfoObject_out aDIO)
{
  return false;
}
