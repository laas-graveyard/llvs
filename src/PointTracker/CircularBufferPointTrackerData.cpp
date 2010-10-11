
#include "PointTracker/CircularBufferPointTrackerData.h"


void CBPointTrackerData::operator=(const CBPointTrackerData &a)
{

  *image = *a.image;
  *timestamp = *a.timestamp;
  cMo = a.cMo;

  if(vpIP.size()!=a.vpIP.size())
    {
       for(unsigned int i=0;i<vpIP.size();i++)
	 {
	   delete vpIP[i];
	 }
       vpIP.clear();
       vpIP.resize(a.vpIP.size());
       for(unsigned int i=0;i<vpIP.size();i++)
	 {
	   vpIP[i]= new vpImagePoint;
	 }
    }

  for(unsigned int i=0;i<vpIP.size();i++)
    {
      *vpIP[i]=*(a.vpIP[i]);
    }

}

