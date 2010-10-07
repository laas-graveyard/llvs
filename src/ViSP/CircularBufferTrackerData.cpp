
#include "ViSP/CircularBufferTrackerData.h"


void CBTrackerData::operator=(const CBTrackerData &a)
{
 
  *image = *a.image;
  *timestamp = *a.timestamp;
  cMo = a.cMo;
  CoG[0] = a.CoG[0];
  CoG[1] = a.CoG[1];

}

