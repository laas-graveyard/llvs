
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

#include "ViSP/CircularBufferTrackerData.h"


namespace llvs
{
  class LowLevelVisionServer;
  
  class ModelTrackerInterface_impl :
  public virtual POA_ModelTrackerInterface
  {
  protected:
    
    // Ref. to LLVS to set tracker parameters.
    LowLevelVisionServer * m_LLVS;

#if (LLVS_HAVE_NMBT>0)

    CBTrackerData m_CBTD;

#endif
    
    
  public:
    
    ModelTrackerInterface_impl(LowLevelVisionServer * LLVS);
    
    ~ModelTrackerInterface_impl();
    
    virtual CORBA::Boolean SetcMo(const ModelTrackerInterface::HomogeneousMatrix& acMo);

    virtual CORBA::Boolean SetcdMo(const ModelTrackerInterface::HomogeneousMatrix& aHM);
   
    virtual CORBA::Boolean GetcMo(ModelTrackerInterface::HomogeneousMatrix &acMo);

    virtual CORBA::Boolean GetDebugInfoObject(ModelTrackerInterface::DebugInfoObject_out aDIO);
    
    
  };
  
  
};

#include "LowLevelVisionServer.h"

#endif 
