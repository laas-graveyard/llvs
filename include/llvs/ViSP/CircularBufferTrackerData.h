/** @doc Class to store data information coming from 
    the model tracker.

   Copyright (c) 2010,
   @author O. Stasse

*/

#ifndef _CIRCULAR_BUFFER_MODEL_TRACKER_LLVS_H_
#define _CIRCULAR_BUFFER_MODEL_TRACKER_LLVS_H_

#include "CircularBuffer.t.h"

#if (LLVS_HAVE_VISP>0)

#include <visp/vpImage.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImageIo.h>
#include "ModelTracker/nmbtTrackingProcess.h"


struct CBTrackerData
{
  vpImage<unsigned char> * image;
  double* timestamp;
  vpHomogeneousMatrix cMo;
  double CoG[2];

  CBTrackerData(){};
  CBTrackerData(const CBTrackerData &aCBTD)
  {
    image = aCBTD.image;
    timestamp = aCBTD.timestamp;
    cMo = aCBTD.cMo;
    CoG[0] = aCBTD.CoG[0];
    CoG[1] = aCBTD.CoG[1];
  }
  void operator=(const CBTrackerData &);
};

class CircularModelTrackerData:public CircularBuffer<CBTrackerData>
{
 public:
 CircularModelTrackerData(int SizeOfCB,string lProcessName):
  CircularBuffer<CBTrackerData>(SizeOfCB),
    m_CBTrackerData(0)
      {
	for(unsigned int i=0;i<m_CircularBuffer.size();i++)
	  {
            //FIXME : The size of the image shouldn't  be hardly written.
            // Yet, we don't have any information  about the image size in this class
            // it should be added ... 
	    m_CircularBuffer[i].onedatum.image= new vpImage<unsigned char>(240,320); 
	    m_CircularBuffer[i].onedatum.timestamp= new double(0);
	  }
		
	m_ProcessName = lProcessName;
      }

  ~CircularModelTrackerData()
    {
      for(unsigned int i=0;i<m_CircularBuffer.size();i++)
	{
	  delete m_CircularBuffer[i].onedatum.image;
	  delete m_CircularBuffer[i].onedatum.timestamp;
	}
    }

  /* Give the tracker pointer. */
  void SetTrackerPointer(HRP2nmbtTrackingProcess * anmbt)
  {
    m_nmbt = anmbt;
  }
  

 protected:
  int pRealizeTheProcess()
  {
    
    m_nmbt->GetOutputcMo(m_Datum->cMo);
    vpColVector avec;
    m_nmbt->GetProjectedObj(avec);
    m_Datum->CoG[0] = avec[0];
    m_Datum->CoG[1] = avec[1];
    CircularBuffer<CBTrackerData>::pRealizeTheProcess();

    return 0;
  }
  
 private:
  HRP2nmbtTrackingProcess * m_nmbt;
  CBTrackerData * m_CBTrackerData;
  
};
#endif  // (LLVS_HAVE_VISP>0)

#endif /* _CIRCULAR_BUFFER_MODEL_TRACKER_LLVS_H_ */
