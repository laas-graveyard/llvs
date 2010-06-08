
/**

ModelTracker Interface Class
Implements the corba interface for
The 3D Model Tracker

date : 07/06/2010
author Stephane Embarki & Claire Dune

**/


#ifndef MODEL_TRACKER_INTERFACE_IMPL
#define MODEL_TRACKER_INTERFACE_IMPL

#include "llvsConfig.h"
 
#if (LLVS_HAVE_OMNIORB4>0)
#include <omniORB4/CORBA.h>
#include "ModelTrackerInterface.hh"
#endif


namespace llvs
{
  class LowLevelVisionServer;
  
  class ModelTrackerInterface_impl :
  public virtual POA_ModelTrackerInterface
  {
  protected:
    
    // Ref. to LLVS to set tracker parameters.
    LowLevelVisionServer * m_LLVS;
    int m_ModelTrackerStatus;
    
  public:
    
    //ModelTrackerInterface_impl(CORBA_ORB_ptr orb,
    //		LowLevelVisionServer * LLVS);
    ModelTrackerInterface_impl(LowLevelVisionServer * LLVS);
    
    ~ModelTrackerInterface_impl();
    
    virtual CORBA::Boolean SetcMo(const ModelTrackerInterface::HomogeneousMatrix& acMo);
    
    virtual CORBA::Boolean GetcMo(ModelTrackerInterface::HomogeneousMatrix &acMo);
    
    
    
  };
  
  
};

#include "LowLevelVisionServer.h"
#endif 
