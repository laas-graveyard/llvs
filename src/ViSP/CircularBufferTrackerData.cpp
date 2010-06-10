
#include "ViSP/CircularBufferTrackerData.h"

#include<visp/vpImage.h>


void CBTrackerData::operator=(const CBTrackerData &a)
{
  *image = *a.image;
  timestamp = a.timestamp;
  cMo = a.cMo;
}

