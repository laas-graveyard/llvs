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
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpCameraParameters.h>
#endif



using namespace llvs;

PointTrackerInterface_impl::PointTrackerInterface_impl(LowLevelVisionServer * LLVS)
{
  m_LLVS = LLVS;
#if (LLVS_HAVE_VISP>0)
  m_CBPTD.image = new vpImage<unsigned char>(240,320);
  m_CBPTD.timestamp = new double(0);
#endif
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
	  vpIP[i]->set_u(aData.UV[2*i]);
	  vpIP[i]->set_v(aData.UV[2*i+1]);
	  aTarget[i].setWorldCoordinates (aData.Target[3*i],
					  aData.Target[3*i+1],
					  aData.Target[3*i+2]);
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

#if (LLVS_HAVE_VISP>0)


   m_LLVS->m_CBonPointTracker->ReadData(m_CBPTD);


  acMo.cMo[0][0]=m_CBPTD.cMo[0][0];
  acMo.cMo[0][1]=m_CBPTD.cMo[0][1];
  acMo.cMo[0][2]=m_CBPTD.cMo[0][2];
  acMo.cMo[0][3]=m_CBPTD.cMo[0][3];
  acMo.cMo[1][0]=m_CBPTD.cMo[1][0];
  acMo.cMo[1][1]=m_CBPTD.cMo[1][1];
  acMo.cMo[1][2]=m_CBPTD.cMo[1][2];
  acMo.cMo[1][3]=m_CBPTD.cMo[1][3];
  acMo.cMo[2][0]=m_CBPTD.cMo[2][0];
  acMo.cMo[2][1]=m_CBPTD.cMo[2][1];
  acMo.cMo[2][2]=m_CBPTD.cMo[2][2];
  acMo.cMo[2][3]=m_CBPTD.cMo[2][3];
  acMo.cMo[3][0]=0;
  acMo.cMo[3][1]=0;
  acMo.cMo[3][2]=0;
  acMo.cMo[3][3]=1;
  return true;

#else

    cout<< " Need VISP to use GetcMo function"<< endl;
    return false;
#endif

  return false;
}

CORBA::Boolean
PointTrackerInterface_impl::GetDebugInfoObject(PointTrackerInterface::DebugInfoObject_out aDIO)
{

#if (LLVS_HAVE_VISP>0)

  m_LLVS->m_CBonPointTracker->ReadData(m_CBPTD);

#endif

  PointTrackerInterface::DebugInfoObject_var aDIOv =
    new PointTrackerInterface::DebugInfoObject;

  aDIOv->anImgData.octetData.length(320*240);
  aDIOv->anImgData.width=320;
  aDIOv->anImgData.height=240;
  aDIOv->anImgData.longData.length(2);
  aDIOv->anImgData.format=GRAY;

#if (LLVS_HAVE_VISP>0)


  aDIOv->anImgData.longData[0] =*m_CBPTD.timestamp;

  unsigned char *pt =m_CBPTD.image->bitmap;

  for(int j=0;j<(int)(320*240);j++)
    aDIOv->anImgData.octetData[j] = *pt++;

  vpImagePoint lvpIP;

   aDIOv->UV.length(2*m_CBPTD.vpIP.size());

  for(unsigned int i=0; i<m_CBPTD.vpIP.size();++i)
    {
      aDIOv-> UV[2*i]=m_CBPTD.vpIP[i]->get_u();
      aDIOv-> UV[2*i+1]=m_CBPTD.vpIP[i]->get_v();
    }


  aDIOv->aHM.cMo[0][0]=m_CBPTD.cMo[0][0];
  aDIOv->aHM.cMo[0][1]=m_CBPTD.cMo[0][1];
  aDIOv->aHM.cMo[0][2]=m_CBPTD.cMo[0][2];
  aDIOv->aHM.cMo[0][3]=m_CBPTD.cMo[0][3];
  aDIOv->aHM.cMo[1][0]=m_CBPTD.cMo[1][0];
  aDIOv->aHM.cMo[1][1]=m_CBPTD.cMo[1][1];
  aDIOv->aHM.cMo[1][2]=m_CBPTD.cMo[1][2];
  aDIOv->aHM.cMo[1][3]=m_CBPTD.cMo[1][3];
  aDIOv->aHM.cMo[2][0]=m_CBPTD.cMo[2][0];
  aDIOv->aHM.cMo[2][1]=m_CBPTD.cMo[2][1];
  aDIOv->aHM.cMo[2][2]=m_CBPTD.cMo[2][2];
  aDIOv->aHM.cMo[2][3]=m_CBPTD.cMo[2][3];
  aDIOv->aHM.cMo[3][0]=0;
  aDIOv->aHM.cMo[3][1]=0;
  aDIOv->aHM.cMo[3][2]=0;
  aDIOv->aHM.cMo[3][3]=1;

#else
  aDIOv->anImgData.longData[0] = 0;
  aDIOv->anImgData.longData[1] = 0;
#endif

  aDIO = aDIOv._retn();
  return 0;




  return false;
}

CORBA::Boolean
PointTrackerInterface_impl::GetPointCoord(PointTrackerInterface::PointCoord_out aPC)
{

#if (LLVS_HAVE_VISP>0)

  PointTrackerInterface::PointCoord_var PCv =
    new PointTrackerInterface::PointCoord;

   m_LLVS->m_CBonPointTracker->ReadData(m_CBPTD);

   double x=0;
   double y=0;

   vpCameraParameters cam;
   m_LLVS->m_PointTrackerProcess->GetCameraParameters(cam);

 for(unsigned int i=0; i<m_CBPTD.vpIP.size();++i)
    {
      vpPixelMeterConversion::convertPoint( cam ,*m_CBPTD.vpIP[i] , x , y ) ;
      PCv->X[i]=x;
      PCv->Y[i]=y;
    }

 aPC = PCv._retn();

  return 0;

#else
  return 1;
#endif

}

