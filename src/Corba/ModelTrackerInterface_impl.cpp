/**

ModelTracker Interface Class
Implements the corba interface for
The 3D Model Tracker

date : 07/06/2010
author Stephane Embarki & Claire Dune

**/


#include <iostream>
#include <stdio.h>
using namespace std;

#ifdef OMNIORB4
#include <omniORB4/CORBA.h>
#endif
#include <Corba/ModelTrackerInterface_impl.h>


#include "llvsConfig.h"

#if   (LLVS_HAVE_VISP>0)
#include <visp/vpHomogeneousMatrix.h>
#endif



using namespace llvs;

ModelTrackerInterface_impl::ModelTrackerInterface_impl(LowLevelVisionServer * LLVS)
{
  m_LLVS = LLVS;
#if (LLVS_HAVE_VISP>0)
  m_CBTD.image = new vpImage<unsigned char>(240,320);
  m_CBTD.timestamp = new double(0);
#endif  
}

ModelTrackerInterface_impl::~ModelTrackerInterface_impl()
{
#if (LLVS_HAVE_VISP>0)
  delete m_CBTD.image;
#endif  
}



CORBA::Boolean
ModelTrackerInterface_impl::SetcMo(const ModelTrackerInterface::HomogeneousMatrix& acMo)
{

#if (LLVS_HAVE_VISP>0)


  vpHomogeneousMatrix cMo;
  
  cMo[0][0]=acMo.cMo[0][0];
  cMo[0][1]=acMo.cMo[0][1];
  cMo[0][2]=acMo.cMo[0][2];
  cMo[0][3]=acMo.cMo[0][3];
  cMo[1][0]=acMo.cMo[1][0];
  cMo[1][1]=acMo.cMo[1][1];
  cMo[1][2]=acMo.cMo[1][2];
  cMo[1][3]=acMo.cMo[1][3];
  cMo[2][0]=acMo.cMo[2][0];
  cMo[2][1]=acMo.cMo[2][1];
  cMo[2][2]=acMo.cMo[2][2];
  cMo[2][3]=acMo.cMo[2][3];

  m_LLVS->m_ModelTrackerProcess->SetcMo(cMo);
   return true;
#else

    cout<< " Need ViSP to use SetcMo function"<< endl;
   return false;
#endif



}

CORBA::Boolean
ModelTrackerInterface_impl::SetcdMo(const ModelTrackerInterface::HomogeneousMatrix& aHM)
{

#if (LLVS_HAVE_VISP>0)


  vpHomogeneousMatrix cdMo;
  
  cdMo[0][0]=aHM.cMo[0][0];
  cdMo[0][1]=aHM.cMo[0][1];
  cdMo[0][2]=aHM.cMo[0][2];
  cdMo[0][3]=aHM.cMo[0][3];
  cdMo[1][0]=aHM.cMo[1][0];
  cdMo[1][1]=aHM.cMo[1][1];
  cdMo[1][2]=aHM.cMo[1][2];
  cdMo[1][3]=aHM.cMo[1][3];
  cdMo[2][0]=aHM.cMo[2][0];
  cdMo[2][1]=aHM.cMo[2][1];
  cdMo[2][2]=aHM.cMo[2][2];
  cdMo[2][3]=aHM.cMo[2][3];

  m_LLVS->m_ComputeControlLawProcess->SetcdMo(cdMo);
   return true;
#else

    cout<< " Need ViSP to use SetcdMo function"<< endl;
   return false;
#endif

}





CORBA::Boolean
ModelTrackerInterface_impl::GetcMo(ModelTrackerInterface::HomogeneousMatrix& acMo)
{

#if (LLVS_HAVE_VISP>0)

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

}

CORBA::Boolean
ModelTrackerInterface_impl::GetDebugInfoObject(ModelTrackerInterface::DebugInfoObject_out aDIO)
{
#if (LLVS_HAVE_VISP>0)

  m_LLVS->m_CBonNMBT->ReadData(m_CBTD);
  
#endif

  ModelTrackerInterface::DebugInfoObject_var aDIOv = 
    new ModelTrackerInterface::DebugInfoObject;

  aDIOv->anImgData.octetData.length(320*240);
  aDIOv->anImgData.width=320;
  aDIOv->anImgData.height=240;
  aDIOv->anImgData.longData.length(2);
  aDIOv->anImgData.format=GRAY;
#if (LLVS_HAVE_VISP>0)


  aDIOv->anImgData.longData[0] =*m_CBTD.timestamp;
  aDIOv->anImgData.longData[1] =(*m_CBTD.timestamp-aDIOv->anImgData.longData[0])*1e6;

  unsigned char *pt =m_CBTD.image->bitmap;

  for(int j=0;j<(int)(320*240);j++)
    aDIOv->anImgData.octetData[j] = *pt++;  

  //  ODEBUG3(" m_CBTD.cMo " << m_CBTD.cMo);
  aDIOv->aData.cMo[0][0]=m_CBTD.cMo[0][0];
  aDIOv->aData.cMo[0][1]=m_CBTD.cMo[0][1];
  aDIOv->aData.cMo[0][2]=m_CBTD.cMo[0][2];
  aDIOv->aData.cMo[0][3]=m_CBTD.cMo[0][3];
  aDIOv->aData.cMo[1][0]=m_CBTD.cMo[1][0];
  aDIOv->aData.cMo[1][1]=m_CBTD.cMo[1][1];
  aDIOv->aData.cMo[1][2]=m_CBTD.cMo[1][2];
  aDIOv->aData.cMo[1][3]=m_CBTD.cMo[1][3];
  aDIOv->aData.cMo[2][0]=m_CBTD.cMo[2][0];
  aDIOv->aData.cMo[2][1]=m_CBTD.cMo[2][1];
  aDIOv->aData.cMo[2][2]=m_CBTD.cMo[2][2];
  aDIOv->aData.cMo[2][3]=m_CBTD.cMo[2][3];
  aDIOv->aData.cMo[3][0]=0;
  aDIOv->aData.cMo[3][1]=0;
  aDIOv->aData.cMo[3][2]=0;
  aDIOv->aData.cMo[3][3]=1;

#else
  aDIOv->anImgData.longData[0] = 0;
  aDIOv->anImgData.longData[1] = 0;
#endif

  aDIO = aDIOv._retn();
  return 0;
}
