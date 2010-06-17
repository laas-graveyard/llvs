/** @doc Class to store data information coming from 
    the model tracker.

   Copyright (c) 2010,
   @author O. Stasse

*/

#ifndef _CIRCULAR_BUFFER_POINT_TRACKER_LLVS_H_
#define _CIRCULAR_BUFFER_POINT_TRACKER_LLVS_H_

#include "CircularBuffer.t.h"

#if (LLVS_HAVE_VISP>0)


#include "PointTracker/PointTrackingProcess.h"

#include <visp/vpImage.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImageIo.h>
#include <visp/vpImagePoint.h>



struct CBPointTrackerData
{
  vpImage<unsigned char> * image;
  timeval* timestamp;
  vector<vpImagePoint*> vpIP;
  vpHomogeneousMatrix cMo;

  CBPointTrackerData(){};
  CBPointTrackerData(const CBPointTrackerData &aCBPTD)
  {
    image = aCBPTD.image;
    timestamp = aCBPTD.timestamp;
    vpIP = aCBPTD.vpIP;
  }
  void operator=(const CBPointTrackerData &);
};

class CircularPointTrackerData:public CircularBuffer<CBPointTrackerData>
{
 public:
 CircularPointTrackerData(int SizeOfCB):
  CircularBuffer<CBPointTrackerData>(SizeOfCB),
    m_CBPointTrackerData(0)
      {
	for(unsigned int i=0;i<m_CircularBuffer.size();i++)
	  {
	    m_CircularBuffer[i].onedatum.image= new vpImage<unsigned char>(240,320); 
	    m_CircularBuffer[i].onedatum.timestamp= new timeval;
	  }
		
	m_ProcessName = "CircularPointTrackerData";
      }

  ~CircularPointTrackerData()
    {
      for(unsigned int i=0;i<m_CircularBuffer.size();i++)
	{
	  delete m_CircularBuffer[i].onedatum.image;
	  delete m_CircularBuffer[i].onedatum.timestamp;
	  
	  for(unsigned int j=0;j<m_CircularBuffer[i].onedatum.vpIP.size();j++)
	    {
	      delete m_CircularBuffer[i].onedatum.vpIP[j];
	    }
	}
    }

  /* Give the tracker pointer. */
  void SetPointTrackerPointer(HRP2PointTrackingProcess * aPointTracker)
  {
    m_PointTracker = aPointTracker ;
  }
  

 protected:
  int pRealizeTheProcess()
  {
    m_PointTracker->GetOutputcMo(m_Datum->cMo);

    m_PointTracker->GetvpImagePoint(m_Datum->vpIP);

    CircularBuffer<CBPointTrackerData>::pRealizeTheProcess();

    return 0;
  }
  
 private:
  HRP2PointTrackingProcess * m_PointTracker;
  CBPointTrackerData * m_CBPointTrackerData;
};
#endif  // (LLVS_HAVE_NMBT>0)

#endif /* _CIRCULAR_BUFFER_MODEL_TRACKER_LLVS_H_ */
