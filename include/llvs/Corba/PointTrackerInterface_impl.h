/** @doc This object implements a visual process to get a disparity map.

    Copyright (c) 2010,
    @author Stephane Embarki

    JRL-Japan, CNRS/AIST

    See license file for information on license.
*/



#ifndef POINT_TRACKER_INTERFACE_IMPL
#define POINT_TRACKER_INTERFACE_IMPL

#include "llvsConfig.h"

#if (LLVS_HAVE_OMNIORB4>0)
#include <omniORB4/CORBA.h>
#include "PointTrackerInterface.hh"
#endif

#if (LLVS_HAVE_VISP>0)

#include "PointTracker/CircularBufferPointTrackerData.h"

#endif

namespace llvs
{
  class LowLevelVisionServer;

  class PointTrackerInterface_impl :
  public virtual POA_PointTrackerInterface
  {
  protected:

    // Ref. to LLVS to set tracker parameters.
    LowLevelVisionServer * m_LLVS;

#if (LLVS_HAVE_VISP>0)

    CBPointTrackerData m_CBPTD;

#endif

  public:

    PointTrackerInterface_impl(LowLevelVisionServer * LLVS);

    ~PointTrackerInterface_impl();

    virtual CORBA::Boolean Init(const PointTrackerInterface::DataTarget & aData);

    virtual CORBA::Boolean GetcMo(PointTrackerInterface::HomogeneousMatrix &acMo);

    virtual CORBA::Boolean GetDebugInfoObject(PointTrackerInterface::DebugInfoObject_out aDIO);

    virtual CORBA::Boolean GetPointCoord(PointTrackerInterface::PointCoord_out PC);
  };


};

#include "LowLevelVisionServer.h"

#endif
