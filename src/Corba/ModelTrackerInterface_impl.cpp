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


#include"llvsConfig.h"

#if   (LLVS_HAVE_VISP>0)
#include <visp/vpHomogeneousMatrix.h>
#endif



using namespace llvs;

ModelTrackerInterface_impl::ModelTrackerInterface_impl(LowLevelVisionServer * LLVS)
{

   m_LLVS = LLVS;
}

ModelTrackerInterface_impl::~ModelTrackerInterface_impl()
{
}



CORBA::Boolean
ModelTrackerInterface_impl::SetcMo(const ModelTrackerInterface::HomogeneousMatrix& acMo)
{

#if(LLVS_HAVE_VISP>0)


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
ModelTrackerInterface_impl::GetcMo(ModelTrackerInterface::HomogeneousMatrix& acMo)
{

#if(LLVS_HAVE_VISP>0)

  vpHomogeneousMatrix cMo;
  
  m_LLVS->m_ModelTrackerProcess->GetOutputcMo(cMo);

  acMo.cMo[0][0]=cMo[0][0];
  acMo.cMo[0][1]=cMo[0][1];
  acMo.cMo[0][2]=cMo[0][2];
  acMo.cMo[0][3]=cMo[0][3];
  acMo.cMo[1][0]=cMo[1][0];
  acMo.cMo[1][1]=cMo[1][1];
  acMo.cMo[1][2]=cMo[1][2];
  acMo.cMo[1][3]=cMo[1][3];
  acMo.cMo[2][0]=cMo[2][0];
  acMo.cMo[2][1]=cMo[2][1];
  acMo.cMo[2][2]=cMo[2][2];
  acMo.cMo[2][3]=cMo[2][3];
  
  return true;
#else

    cout<< " Need ViSP to use GetcMo function"<< endl;
    return false;
#endif
  

}
