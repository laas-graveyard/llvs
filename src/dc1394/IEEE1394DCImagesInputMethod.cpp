#include <stdlib.h>
#include <iostream>
#include <fstream>


using namespace std;


using std::showbase;

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "HPR2IEEE1304DCImagesInputMethod:" << x << endl
#define ODEBUG3_CONT(x) cerr << x 

#if 1
#define ODEBUG(x) cerr << "HPR2IEEE1304DCImagesInputMethod:" <<  x << endl
#define ODEBUG_CONT(x) cerr << "HPR2IEEE1304ImagesInputMethod:" <<  x << endl
#else
#define ODEBUG(x) 
#define ODEBUG_CONT(x) 
#endif



#include "IEEE1394DCImagesInputMethod.h"

/************************************************************************
*  static functions							*
************************************************************************/



/**************************************************************
 * class Images Input                                         *
 **************************************************************/
HRP2IEEE1394DCImagesInputMethod::HRP2IEEE1394DCImagesInputMethod() : HRP2ImagesInputMethod()
{
  m_numCameras = 0;

  /* File descriptor to the frame grabber. */
  ODEBUG("Through the constructor ");
  InitializeBoard();
  ODEBUG("Through the constructor");

  m_LastGrabbingTime.resize(m_numCameras);
  for(int li=0;li<m_numCameras;li++)
    m_LastGrabbingTime[li]=-1.0;

  m_GrabbingPeriod.resize(m_numCameras);
  
  m_Format = "PGM";
  m_Computing = 1;

  HRP2VisionBasicProcess::m_ProcessName = "IEEE1394 Image grabbing";

  
  string aPar = "Format";
  string aValue = m_Format;

  SetParameter(aPar,aValue);

  string Prefixes[4] = { "LEFT", "RIGHT","CYCL","WIDE"};
  string Features[5] = { "BRIGHTNESS", "AUTO_EXPOSURE","GAMMA","SHUTTER","GAIN"};  

  for(int i=0;i<4;i++)
    m_Prefixes[i] = Prefixes[i];
  
  for(int i=0;i<5;i++)
    m_Features[i] = Features[i];
  
  ODEBUG("Before setting the parameters.");

  for(int i=0;i<4;i++)
    {

      for(int j=0;j<5;j++)
	{
	  string ParameterName = m_Prefixes[i] + "|" + m_Features[j];
	  string ParameterValue;
	  GetCameraFeatureValue(m_Prefixes[i],m_Features[j],ParameterValue);
	  HRP2VisionBasicProcess::SetParameter(ParameterName,ParameterValue);
	  
	}
    }
  for(int i=0;i<m_numCameras;i++)
    FromFrameRateToTime(i);
    
  StartContinuousShot();
  ODEBUG("After setting the parameters.");
}

void HRP2IEEE1394DCImagesInputMethod::GetCameraFeatureValue(string aCamera, string aFeature, string &aValue)
{

  int iCamera=0;
  dc1394feature_info_t lFeature;

  if (aCamera=="LEFT")
    iCamera = 0;
  else if (aCamera=="RIGHT")
    iCamera = 1;
  else if (aCamera=="CYCL")
    iCamera = 2;
  else if (aCamera=="WIDE")
    iCamera = 3;

  if (aFeature=="BRIGHTNESS")
    lFeature->feature_id = DC1394_FEATURE_BRIGHTNESS;
  else if (aFeature=="AUTO_EXPOSURE")
    lFeature->feature_id = DC1394_FEATURE_EXPOSURE;
  else if (aFeature=="WHITE_BALANCE")
    lFeature->feature_id = DC1394_FEATURE_WHITE_BALANCE;
  else if (aFeature=="GAMMA")
    lFeature->feature_id = DC1394_FEATURE_GAMMA;
  else if (aFeature=="SHUTTER")
    lFeature->feature_id = DC1394_FEATURE_SHUTTER;
   else if (aFeature=="GAIN")
    lFeature->feature_id = DC1394_FEATURE_GAIN;

  u_int avalue=0;

  dc1304_feature_get_value(&m_DC1394Cameras[iCamera],lFeature,&avalue);
  char Buffer[1024];
  bzero(Buffer,1024);
  sprintf(Buffer,"%d",avalue);
  aValue = Buffer;
}

void HRP2IEEE1394DCImagesInputMethod::SetCameraFeatureValue(string aCamera, string aFeature, string aValue)
{
  dc1394feature_info_t lFeature;

  if (aCamera=="LEFT")
    iCamera = 0;
  else if (aCamera=="RIGHT")
    iCamera = 1;
  else if (aCamera=="CYCL")
    iCamera = 2;
  else if (aCamera=="WIDE")
    iCamera = 3;

  if (aFeature=="BRIGHTNESS")
    lFeature->feature_id = DC1394_FEATURE_BRIGHTNESS;
  else if (aFeature=="AUTO_EXPOSURE")
    lFeature->feature_id = DC1394_FEATURE_EXPOSURE;
  else if (aFeature=="WHITE_BALANCE")
    lFeature->feature_id = DC1394_FEATURE_WHITE_BALANCE;
  else if (aFeature=="GAMMA")
    lFeature->feature_id = DC1394_FEATURE_GAMMA;
  else if (aFeature=="SHUTTER")
    lFeature->feature_id = DC1394_FEATURE_SHUTTER;
   else if (aFeature=="GAIN")
    lFeature->feature_id = DC1394_FEATURE_GAIN;

  
  StopContinuousShot();
  u_int avalue;
  avalue = atoi(aValue.c_str());
  dc1304_feature_set_value(&m_DC1394Cameras[iCamera],lFeature,avalue);
  StartContinuousShot();  
}

HRP2IEEE1394DCImagesInputMethod::~HRP2IEEE1394DCImagesInputMethod()
{

  /* Close the frame grabber. */
  StopBoard();

  for(int i=0;i<4;i++)
    {
      if (m_TmpImage[i] != 0)
	delete m_TmpImage[i];
    }
}

int HRP2IEEE1394DCImagesInputMethod::StartProcess()
{
  ODEBUG3("StartProcess: Phase 1");
  HRP2VisionBasicProcess::StartProcess();
  ODEBUG3("StartProcess: Phase 2");
  InitializeBoard();
  ODEBUG3("StartProcess: Phase 3");
  StartContinuousShot();
  ODEBUG3("StartProcess: Phase 4");
  return 0;
}


int HRP2IEEE1394DCImagesInputMethod::StopProcess()
{
  HRP2VisionBasicProcess::StopProcess();
  StopBoard();
  return 0;
}



int HRP2IEEE1394DCImagesInputMethod::GetSingleImage(unsigned char **Image, int camera, struct timeval &timestamp)
{
  int r=-1;
  ODEBUG(m_Format);
  if (m_Format=="PGM")
    r = GetImageSinglePGM(Image,camera,timestamp);
  else if (m_Format=="RAW")
    r = GetImageSingleRaw(Image,camera,timestamp);
  else if (m_Format=="RGB")
    r = GetImageSingleRGB(Image,camera,timestamp);
  return r;
}

int HRP2IEEE1394DCImagesInputMethod::GetImage(unsigned char **ImageLeft, unsigned char **ImageRight, 
					    unsigned char **ImageUp, unsigned char **ImageWide)
{
  int r = -1; 
  if (m_Format=="PGM")
    r = GetImagePGM(ImageLeft,ImageRight, ImageUp, ImageWide);
  else if (m_Format=="RGB")
    r = GetImageRGB(ImageLeft,ImageRight, ImageUp, ImageWide);
  else if (m_Format=="RAW")
    r = GetImageRaw(ImageLeft,ImageRight, ImageUp, ImageWide);

  return r;
}

int HRP2IEEE1394DCImagesInputMethod::GetImageRaw(unsigned char **ImageLeft, unsigned char **ImageRight, 
					       unsigned char **ImageUp, unsigned char **ImageWide)
{
  unsigned char * ImagesDst[4];

  vector<dc1394video_frame_t *> frame;
  dc1394error_t err;

  frame.resize(m_numCameras);

#define LOCAL_TYPE unsigned char *

  ODEBUG("GetImage");
  LOCAL_TYPE ImagesTab[4];
  
  
  if ((m_TmpImage[0]==0) ||
      (m_TmpImage[1]==0) ||
      (m_TmpImage[2]==0) ||
      (m_TmpImage[3]==0))
    {
        ImagesTab[0] = (LOCAL_TYPE)*ImageLeft;
	ImagesTab[1] = (LOCAL_TYPE)*ImageRight;
	ImagesTab[2] = (LOCAL_TYPE)*ImageUp;

      try
	{
	  /*-----------------------------------------------------------------------
	   *  capture one frame
	   *-----------------------------------------------------------------------*/
	  for(unsigned int i=0;i<m_numCameras;i++)
	    {
	      err=dc1394_capture_dequeue(m_DC1394Cameras[i], DC1394_CAPTURE_POLICY_WAIT, &frame[i]);
	      DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not capture a frame");
	    }

	}
      catch(std::exception &except)
	{
	  ODEBUG("Exception during snap: " << except.what() );
	}
    }
  else
    {

      ImagesTab[0] = (LOCAL_TYPE) m_TmpImage[0];
      ImagesTab[1] = (LOCAL_TYPE) m_TmpImage[1];
      ImagesTab[2] = (LOCAL_TYPE) m_TmpImage[2];
      ImagesTab[3] = (LOCAL_TYPE) m_TmpImage[3];

      ImagesDst[0] = *ImageLeft;
      ImagesDst[1] = *ImageRight;
      ImagesDst[2] = *ImageUp;
      ImagesDst[3] = *ImageWide;

      ODEBUG("Get Images " );
      try
	{
	  /*-----------------------------------------------------------------------
	   *  capture one frame
	   *-----------------------------------------------------------------------*/
	  for(unsigned int i=0;i<m_numCameras;i++)
	    {
	      err=dc1394_capture_dequeue(m_DC1394Cameras[i], DC1394_CAPTURE_POLICY_WAIT, &frame[i]);
	      DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not capture a frame");
	    }

	}
      catch(std::exception &except)
	{
	  ODEBUG("Exception during snap: " << except.what() );
	}

      ODEBUG("Get Images finito...");

      for(int k=0;k<4;k++)
	{
	  unsigned char *ImgSrc,*ImgDst;
	  ImgDst =(unsigned char *) ImagesDst[k];

	  if (ImgDst==0)
	    continue;

	  int intervalw, intervalh, indexd, indexs;
	  unsigned int BWidth, BHeight;
	  BWidth = m_BoardImagesWidth[k];
	  BHeight = m_BoardImagesHeight[k];
	  intervalw = BWidth / m_ImagesWidth[k];
	  intervalh =  BHeight/ m_ImagesHeight[k];
	  

	  ImgSrc = m_TmpImage[k];

	  for(unsigned int j=0;j<m_ImagesHeight[k];j+=2)
	    {
	      for(unsigned int i=0;i<m_ImagesWidth[k];i+=2)
		{
		  indexd = j * m_ImagesWidth[k] + i ;
		  
		  indexs = j * intervalh * BWidth  + i * intervalw ;
		  unsigned long localsum[4] = {0,0,0,0};
		  for(int l=0;l<intervalh;l+=2)
		    {
		      int lindexs = l * BWidth   + indexs;
		      for(int m=0;m<intervalw;m+=2)
			{
			   localsum[0] += ImgSrc[lindexs+m];
			   localsum[1] += ImgSrc[lindexs+m+1];
			}
		      lindexs += BWidth;
		      for(int m=0;m<intervalw;m+=2)
			{
			   localsum[2] += ImgSrc[lindexs+m];
			   localsum[3] += ImgSrc[lindexs+m+1];
			}
		    }

		  ImgDst[indexd+0] = (unsigned char ) ((float)localsum[0]/(float)(intervalh*intervalw));
		  ImgDst[indexd+1] = (unsigned char ) ((float)localsum[1]/(float)(intervalh*intervalw));
		  ImgDst[indexd+m_ImagesWidth[k]+0] = (unsigned char ) ((float)localsum[2]/(float)(intervalh*intervalw));
		  ImgDst[indexd+m_ImagesWidth[k]+1] = (unsigned char ) ((float)localsum[3]/(float)(intervalh*intervalw));
		}
	    }
	}

    }
  
  return 0;
}
int HRP2IEEE1394DCImagesInputMethod::GetImagePGM(unsigned char **ImageLeft, unsigned char **ImageRight, 
					       unsigned char **ImageUp, unsigned char **ImageWide)
{
  unsigned char * ImagesDst[4];

  if (m_Board==0)
    return 0;


#define LOCAL_TYPE unsigned char *

  ODEBUG("GetImage");
  LOCAL_TYPE ImagesTab[4];
  
  
  if ((m_TmpImage[0]==0) ||
      (m_TmpImage[1]==0) ||
      (m_TmpImage[2]==0) ||
      (m_TmpImage[3]==0))
    {
        ImagesTab[0] = (LOCAL_TYPE)*ImageLeft;
	ImagesTab[1] = (LOCAL_TYPE)*ImageRight;
	ImagesTab[2] = (LOCAL_TYPE)*ImageUp;

      try
	{
	  m_Board->snap(ImagesTab);
	}
      catch(std::exception &except)
	{
	  ODEBUG("Exception during snap: " << except.what() );
	}
    }
  else
    {

      ImagesTab[0] = (LOCAL_TYPE) m_TmpImage[0];
      ImagesTab[1] = (LOCAL_TYPE) m_TmpImage[1];
      ImagesTab[2] = (LOCAL_TYPE) m_TmpImage[2];
      ImagesTab[3] = (LOCAL_TYPE) m_TmpImage[3];

      ImagesDst[0] = *ImageLeft;
      ImagesDst[1] = *ImageRight;
      ImagesDst[2] = *ImageUp;
      ImagesDst[3] = *ImageWide;

      ODEBUG("Get Images " );
      try
	{
	  m_Board->snap(ImagesTab);
	}
      catch(std::exception &except)
	{
	  ODEBUG("Exception during snap: " << except.what() );
	}

      //m_Board->get_images(ImagesTab);
      ODEBUG("Get Images finito...");

      for(int k=0;k<4;k++)
	{
	  unsigned char *ImgSrc,*ImgDst;
	  ImgDst =(unsigned char *) ImagesDst[k];

	  if (ImgDst==0)
	    continue;

	  int intervalw, intervalh, indexd, indexs;
	  unsigned int BWidth, BHeight;
	  BWidth = m_Board->width();
	  BHeight = m_Board->height();
	  intervalw = BWidth / m_ImagesWidth[k];
	  intervalh =  BHeight/ m_ImagesHeight[k];
	  

	  ImgSrc = m_TmpImage[k];

	  for(unsigned int j=0;j<m_ImagesHeight[k];j++)
	    {
	      for(unsigned int i=0;i<m_ImagesWidth[k];i++)
		{
		  indexd = j * m_ImagesWidth[k] + i ;
		  
		  indexs = j * intervalh * BWidth * 3  + i * intervalw *3;
		  unsigned long localsum = 0;
		  for(int l=0;l<intervalh;l++)
		    {
		      int lindexs = l * BWidth * 3  + indexs;
		      for(int m=0;m<intervalw;m++)
			{
			   localsum += ImgSrc[lindexs+m*3];
			}
		    }

		  ImgDst[indexd] = (unsigned char ) (localsum/(intervalh*intervalw));
		}
	    }
	}

    }

  
  return 0;
}

int HRP2IEEE1394DCImagesInputMethod::GetImageSinglePGM(unsigned char **Image, int camera, struct timeval &timestamp)
{
  unsigned char * ImagesDst;

  if (m_Board==0)
    return 0;
  struct timeval tval;
  double time1, time2;
#define LOCAL_TYPE unsigned char *

  ODEBUG("GetImageSinglePGM cam: " << camera);
  LOCAL_TYPE ImagesTab[1];
  
  if (m_TmpImage[0]==0)
    {
        ImagesTab[0] = (LOCAL_TYPE)*Image;

      try
	{
	  m_Board->snapfromI(ImagesTab[0],camera);
	  gettimeofday(&timestamp,0);
	}
      catch(std::exception &except)
	{
	  ODEBUG("Exception during snap: " << except.what() );
	}
    }
  else
    {

      ImagesTab[0] = (LOCAL_TYPE) m_TmpImage[0];

      ImagesDst = *Image;

      ODEBUG("Get Images " );
      try
	{
	  m_Board->snapfromI(ImagesTab[0],camera);
	  gettimeofday(&timestamp,0);
	}
      catch(std::exception &except)
	{
	  ODEBUG("Exception during snap: " << except.what() );
	}

      time1 = timestamp.tv_sec + 0.000001* timestamp.tv_usec;

      //m_Board->get_images(ImagesTab);
      ODEBUG("Get Images finito...");

      unsigned char *ImgSrc,*ImgDst;
      ImgDst =(unsigned char *) ImagesDst;
      

      int intervalw, intervalh, indexd, indexs;
      unsigned int BWidth, BHeight;
      BWidth = m_Board->width();
      BHeight = m_Board->height();
      intervalw = BWidth / m_ImagesWidth[camera];
      intervalh =  BHeight/ m_ImagesHeight[camera];
      
      
      ImgSrc = m_TmpImage[0];
      
      for(unsigned int j=0;j<m_ImagesHeight[camera];j++)
	    {
	      for(unsigned int i=0;i<m_ImagesWidth[camera];i++)
		{
		  indexd = j * m_ImagesWidth[camera] + i ;
		  
		  indexs = j * intervalh * BWidth * 3  + i * intervalw *3;
		  unsigned long localsum = 0;
		  for(int l=0;l<intervalh;l++)
		    {
		      int lindexs = l * BWidth * 3  + indexs;
		      for(int m=0;m<intervalw;m++)
			{
			  localsum += ImgSrc[lindexs+m*3];
			}
		    }
		  
		  ImgDst[indexd] = (unsigned char ) (localsum/(intervalh*intervalw));
		}
	    }
    }
  
  //gettimeofday(&tval,0);
  //time2 = tval.tv_sec + 0.000001* tval.tv_usec;
  //ODEBUG3( time2 - time1);

  m_LastGrabbingTime[camera]= timestamp.tv_sec + 0.000001* timestamp.tv_usec;
  return 0;
}

int HRP2IEEE1394DCImagesInputMethod::GetImageSingleRGB(unsigned char **Image, int camera, struct timeval &timestamp)
{
  unsigned char * ImagesDst;

  if (m_Board==0)
    return 0;
  struct timeval tval;
  double time1, time2;
#define LOCAL_TYPE unsigned char *

  ODEBUG("GetImageSingleRGB cam: " << camera);
  LOCAL_TYPE ImagesTab[1];
  
  if (m_TmpImage[0]==0)
    {
      ImagesTab[0] = (LOCAL_TYPE)*Image;
      
      try
	{
	  m_Board->snapfromI(ImagesTab[0],camera);
	  gettimeofday(&timestamp,0);
	}
      catch(std::exception &except)
	{
	  ODEBUG("Exception during snap: " << except.what() );
	}
    }
  else
    {

      ImagesTab[0] = (LOCAL_TYPE) m_TmpImage[0];

      ImagesDst = *Image;

      try
	{
	  m_Board->snapfromI(ImagesTab[0],camera);
	  gettimeofday(&timestamp,0);
	}
      catch(std::exception &except)
	{
	  ODEBUG("Exception during snap: " << except.what() );
	}

      time1 = timestamp.tv_sec + 0.000001* timestamp.tv_usec;

      //m_Board->get_images(ImagesTab);
      ODEBUG("Get Images finito...");

      unsigned char *ImgSrc,*ImgDst;
      ImgDst =(unsigned char *) ImagesDst;
      

      int intervalw, intervalh, indexd, indexs;
      unsigned int BWidth, BHeight;
      BWidth = m_Board->width();
      BHeight = m_Board->height();
      intervalw = BWidth / m_ImagesWidth[camera];
      intervalh =  BHeight/ m_ImagesHeight[camera];
      
      
      ImgSrc = m_TmpImage[0];

 
      for(unsigned int j=0;j<m_ImagesHeight[camera];j++)
	{
	  for(unsigned int i=0;i<m_ImagesWidth[camera];i++)
	    {
	      indexd = j * m_ImagesWidth[camera]*3 + i*3 ;
	      
	      indexs = j * intervalh * BWidth * 3  + i * intervalw *3;
	      unsigned long localsum[3] = {0,0,0};
	      for(int l=0;l<intervalh;l++)
		{
		  int lindexs = l * BWidth * 3  + indexs;
		  for(int m=0;m<intervalw;m++)
		    {
		      for(int n=0;n<3;n++)
			localsum[n] += ImgSrc[lindexs+m*3+n];
		    }
		}
	      for(int n=0;n<3;n++)
		ImgDst[indexd+n] = (unsigned char ) (localsum[n]/(intervalh*intervalw));
	    }
	}

#if 0
      ofstream aof;
      char Buffer[128];
      sprintf(Buffer,"dump_Dst_%d.ppm",camera);
      aof.open(Buffer,ofstream::out);
      aof << "P6\n"<< BWidth << " " << BHeight << endl;
      aof << "255\n";
      
      for(int j=0;j<m_ImagesHeight[camera]*m_ImagesWidth[camera]*3;j++)
	aof << (unsigned char) ImgDst[j];
      aof.close();
#endif

    }
  
  //gettimeofday(&tval,0);
  //time2 = tval.tv_sec + 0.000001* tval.tv_usec;
  //ODEBUG3( time2 - time1);
  m_LastGrabbingTime[camera]= timestamp.tv_sec + 0.000001* timestamp.tv_usec;   
  return 0;
}

int HRP2IEEE1394DCImagesInputMethod::GetImageSingleRaw(unsigned char **Image, int camera, struct timeval &timestamp)
{
  unsigned char * ImagesDst;

  if (m_Board==0)
    return 0;

#define LOCAL_TYPE unsigned char *

  ODEBUG("GetImageSinglePGM cam: " << camera);
  LOCAL_TYPE ImagesTab[1];
  
  
  if (m_TmpImage[0]==0)
    {
        ImagesTab[0] = (LOCAL_TYPE)*Image;

      try
	{
	  m_Board->snapcfromI(ImagesTab[0],camera);
	  gettimeofday(&timestamp,0);
	}
      catch(std::exception &except)
	{
	  ODEBUG("Exception during snap: " << except.what() );
	}
    }
  else
    {

      ImagesTab[0] = (LOCAL_TYPE) m_TmpImage[0];

      ImagesDst = *Image;

      ODEBUG("Get Images " );
      try
	{
	  m_Board->snapcfromI(ImagesTab[0],camera);
	  gettimeofday(&timestamp,0);
	}
      catch(std::exception &except)
	{
	  ODEBUG("Exception during snap: " << except.what() );
	}

      //m_Board->get_images(ImagesTab);
      ODEBUG("Get Images finito...");

      unsigned char *ImgSrc,*ImgDst;
      ImgDst =(unsigned char *) ImagesDst;
      

      int intervalw, intervalh, indexd, indexs;
      unsigned int BWidth, BHeight;
      BWidth = m_Board->width();
      BHeight = m_Board->height();

      intervalw = BWidth / m_ImagesWidth[camera];
      intervalh =  BHeight/ m_ImagesHeight[camera];
      
      
      ImgSrc = m_TmpImage[0];
      
      for(unsigned int j=0;j<m_ImagesHeight[camera];j+=2)
	    {
	      for(unsigned int i=0;i<m_ImagesWidth[camera];i+=2)
		{

		  indexd = j * m_ImagesWidth[camera] + i ;
		  
		  indexs = j * intervalh * BWidth  + i * intervalw ;
		  unsigned long localsum[4] = {0,0,0,0};
		  for(int l=0;l<2*intervalh;l+=2)
		    {
		      int lindexs = l * BWidth   + indexs;
		      for(int m=0;m<2*intervalw;m+=2)
			{
			   localsum[0] += ImgSrc[lindexs+m];
			   localsum[1] += ImgSrc[lindexs+m+1];
			}
		      lindexs += BWidth;
		      for(int m=0;m<2*intervalw;m+=2)
			{
			   localsum[2] += ImgSrc[lindexs+m];
			   localsum[3] += ImgSrc[lindexs+m+1];
			}
		    }

		  ImgDst[indexd+0] = (unsigned char ) ((float)localsum[0]/(float)(intervalh*intervalw));
		  ImgDst[indexd+1] = (unsigned char ) ((float)localsum[1]/(float)(intervalh*intervalw));
		  ImgDst[indexd+m_ImagesWidth[camera]+0] = (unsigned char ) ((float)localsum[2]/(float)(intervalh*intervalw));
		  ImgDst[indexd+m_ImagesWidth[camera]+1] = (unsigned char ) ((float)localsum[3]/(float)(intervalh*intervalw));

		}
	    }
    }

  m_LastGrabbingTime[camera]= timestamp.tv_sec + 0.000001* timestamp.tv_usec;    
  return 0;
}

int HRP2IEEE1394DCImagesInputMethod::GetImageRGB(unsigned char **ImageLeft, unsigned char **ImageRight, 
					       unsigned char **ImageUp, unsigned char **ImageWide)
{
  unsigned char * ImagesDst[4];

  if (m_Board==0)
    return 0;

#define LOCAL_TYPE unsigned char *

  ODEBUG("GetImage");
  LOCAL_TYPE ImagesTab[4];
  
  
  if ((m_TmpImage[0]==0) ||
      (m_TmpImage[1]==0) ||
       (m_TmpImage[2]==0) ||
      (m_TmpImage[3]==0))
    {
        ImagesTab[0] = (LOCAL_TYPE)*ImageLeft;
	ImagesTab[1] = (LOCAL_TYPE)*ImageRight;
	ImagesTab[2] = (LOCAL_TYPE)*ImageUp;

      try
	{
	  m_Board->snap(ImagesTab);
	}
      catch(std::exception &except)
	{
	  ODEBUG("Exception during snap: " << except.what() );
	}
    }
  else
    {

      ImagesTab[0] = (LOCAL_TYPE) m_TmpImage[0];
      ImagesTab[1] = (LOCAL_TYPE) m_TmpImage[1];
      ImagesTab[2] = (LOCAL_TYPE) m_TmpImage[2];
      ImagesTab[3] = (LOCAL_TYPE) m_TmpImage[3];

      ImagesDst[0] = *ImageLeft;
      ImagesDst[1] = *ImageRight;
      ImagesDst[2] = *ImageUp;
      ImagesDst[3] = *ImageWide;

      ODEBUG("Get Images " );
      try
	{
	  m_Board->snap(ImagesTab);
	}
      catch(std::exception &except)
	{
	  ODEBUG("Exception during snap: " << except.what() );
	}

      //m_Board->get_images(ImagesTab);
      ODEBUG("Get Images finito...");
      struct timeval timestamp;


      for(int k=0;k<4;k++)
	{
	  gettimeofday(&timestamp,0);

	  m_LastGrabbingTime[k]= timestamp.tv_sec + 0.000001* timestamp.tv_usec;  
	  unsigned char *ImgSrc,*ImgDst;
	  ImgDst =(unsigned char *) ImagesDst[k];

	  if (ImgDst==0)
	    continue;

	  int intervalw, intervalh, indexd, indexs;
	  unsigned int BWidth, BHeight;
	  BWidth = m_Board->width();
	  BHeight = m_Board->height();
	  intervalw = BWidth / m_ImagesWidth[k];
	  intervalh =  BHeight/ m_ImagesHeight[k];
	  	  

	  ImgSrc = m_TmpImage[k];

	  for(unsigned int j=0;j<m_ImagesHeight[k];j++)
	    {
	      for(unsigned int i=0;i<m_ImagesWidth[k];i++)
		{
		  indexd = j * m_ImagesWidth[k] *3  + i * 3 ;
		  
		  indexs = j * intervalh * BWidth * 3  + i * intervalw *3;
		  unsigned long localsum[3] = {0,0,0};
		  for(int l=0;l<intervalh;l++)
		    {
		      int lindexs = l * BWidth * 3  + indexs;
		      for(int m=0;m<intervalw;m++)
			{
			   localsum[0] += ImgSrc[lindexs+m*3];
			   localsum[1] += ImgSrc[lindexs+m*3+1];
			   localsum[2] += ImgSrc[lindexs+m*3+2];
			}
		    }

		  ImgDst[indexd  ] = (unsigned char ) (localsum[0]/(intervalh*intervalw));
		  ImgDst[indexd+1] = (unsigned char ) (localsum[1]/(intervalh*intervalw));
		  ImgDst[indexd+2] = (unsigned char ) (localsum[2]/(intervalh*intervalw));
		}
	    }
	}

    }

  return 0;
}

int HRP2IEEE1394DCImagesInputMethod::SetImageSize(int lw, int lh, int CameraNumber)
{
  if (m_Board==0)
    return 0;
    
  int i;
  ODEBUG("Debug images");
  for(i=0;i<4;i++)
    {
      m_ImagesWidth[i] = lw;
      m_ImagesHeight[i] = lh;
    }

  if ((m_Board->width() != (unsigned int)lw) ||
      (m_Board->height() != (unsigned int)lh) ||
      (m_Format!="YUV422"))
    {
      ODEBUG3("Allocation for m_TmpImage");
      if ((m_TmpImage[0]==0) &&
	  (m_TmpImage[1]==0) &&
	  (m_TmpImage[2]==0) &&
	  (m_TmpImage[3]==0))
	{
	  for(int i=0;i<4;i++)
	    {
	      if (m_TmpImage[i]==0)
		m_TmpImage[i] = new unsigned char[m_Board->width() * m_Board->height() * 4];
	    }
	}
    }

  return 0;
}


int HRP2IEEE1394DCImagesInputMethod::GetImageSize(int &lw, int &lh, int CameraNumber)
{
  lw = m_ImagesWidth[CameraNumber];
  lh = m_ImagesHeight[CameraNumber];
  return 0;
}


string HRP2IEEE1394DCImagesInputMethod::GetFormat()
{
  return m_Format;
}

int HRP2IEEE1394DCImagesInputMethod::SetFormat(string aFormat)
{
  if (aFormat=="RGB")
    {
      m_Format = "RGB";
      ODEBUG("Format Image : RGB");
      return 0;
    }
  else if (aFormat=="PGM")
    {
      m_Format = "PGM";
      ODEBUG("Format Image : PGM");
      return 0;
    }
  else if (aFormat=="RAW")
    {
      m_Format = "RAW";
      ODEBUG("Format Image : RAW");
      return 0;
    }
  
  return -1;
}


int HRP2IEEE1394DCImagesInputMethod::SetParameter(string aParameter, string aValue)
{
  HRP2VisionBasicProcess::SetParameter(aParameter,aValue);
  string CameraPrefix;
  unsigned char IsACamera = 0;
  unsigned int lpos = 0;

  CameraPrefix = aParameter.substr(0,4);
  if ((CameraPrefix=="LEFT") ||
      (CameraPrefix=="CYCL") ||
      (CameraPrefix=="WIDE") )
    {
      IsACamera = 1;
      lpos = 5;
    }
  else if (CameraPrefix=="RIGH")
    {
      CameraPrefix = aParameter.substr(0,5);
      if (CameraPrefix=="RIGHT")
	{
	  IsACamera = 1;
	  lpos=6;
	}
    }
  
  if (IsACamera)
    {
      unsigned char IsFeature=0;
      
      string lFeature = aParameter.substr(lpos,aParameter.length()-lpos);
      for(int i=0;i<5;i++)
	if (lFeature == m_Features[i])
	  {
	    IsFeature=1;
	    break;
	  }
      
      if (IsFeature)
	{
	  ODEBUG(" Modificatio of camera parameters: " << CameraPrefix << " " << lFeature << " " << aValue);
	  SetCameraFeatureValue(CameraPrefix,lFeature,aValue);
	}
    }

  if (aParameter=="Format")
    {
      SetFormat(aValue);      
    }
    
}
void HRP2IEEE1394DCImagesInputMethod::InitializeBoard()
{
  try 
    {

      /*! List of camera. */
      dc1394camera_list_t * list;
  
      m_HandleDC1394 =  dc1394_new ();
      dc1394error_t err;
      err = dc1394_camera_enumerate(m_HandleDC1394, &list);

      j = 0;
      for (i = 0; i < list->num; i++) {
        m_DC1394Cameras[j] = dc1394_camera_new (m_HandleDC1394, list->ids[i].guid);
        if (!m_DC1394Cameras[j]) {
	  dc1394_log_warning("Failed to initialize camera with guid %llx", list->ids[i].guid);
	  continue;
        }
        j++;
      }
      m_numCameras = j;
      dc1394_camera_free_list (list);
    
    }
  catch(...)
    {
      ODEBUG3("Unable to initialize the board correctly\n");
      return;
    }
  InitializeCameras();
  ODEBUG("InitializeBoard");
}

void HRP2IEEE1394DCImagesInputMethod::InitializeCameras()
{
  res=DC1394_VIDEO_MODE_640x480_YUV411;
  fps=DC1394_VIDEO_FRAMERATE_30;
  
  for (i = 0; i < m_numCameras; i++) 
    {
      
      err=dc1394_video_set_iso_speed(m_DC1394Cameras[i], DC1394_ISO_SPEED_400);
      DC1394_ERR_CLN_RTN(err,cleanup(),"Could not set ISO speed");
      
      err=dc1394_video_set_mode(m_DC1394Cameras[i], res);
      DC1394_ERR_CLN_RTN(err,cleanup(),"Could not set video mode");
      
      err=dc1394_video_set_framerate(m_DC1394Cameras[i], fps);
      DC1394_ERR_CLN_RTN(err,cleanup(),"Could not set framerate");
      
      err=dc1394_capture_setup(m_DC1394Cameras[i],NUM_BUFFERS, DC1394_CAPTURE_FLAGS_DEFAULT);
      DC1394_ERR_CLN_RTN(err,cleanup(),
			 "Could not setup camera-\nmake sure \
                          that the video mode and framerate \
                          are\nsupported by your camera");

      m_BoardImagesWidth[i]= 640;
      m_BoardImagesHeight[i]= 480;

    }
}
    



void HRP2IEEE1394DCImagesInputMethod::StartContinuousShot()
{
  dc1394error_t err;
  for(unsigned int i=0;i<m_DC1394Cameras.size();i++)
    {
      err=dc1394_video_set_transmission(m_DC1394Cameras[i], DC1394_ON);
      DC1394_ERR_CLN_RTN(err,cleanup(),"Could not start camera iso transmission");
    }

}

void HRP2IEEE1394DCImagesInputMethod::StopContinuousShot()
{
  for(unsigned int i=0;i<m_DC1394Cameras.size();i++)
    {
      dc1394_video_set_transmission(m_DC1394Cameras[i], DC1394_OFF);
      dc1394_capture_stop(m_DC1394Cameras[i]);
    }
}


void HRP2IEEE1394DCImagesInputMethod::StopBoard()
{
  for(unsigned int i=0;i<m_DC1394Cameras.size();i++)
    {
      dc1394_video_set_transmission(m_DC1394Cameras[i], DC1394_OFF);
      dc1394_capture_stop(m_DC1394Cameras[i]);
      dc1394_camera_free(m_DC1394Cameras[i]);
    }
  dc1394_free (m_HandleDC1394);
  
  ODEBUG("StopBoard");
}

int HRP2IEEE1394DCImagesInputMethod::GetNumberOfCameras()
{
  return m_numCameras;
}

void HRP2IEEE1394DCImagesInputMethod::FromFrameRateToTime(int CameraNumber)
{
  
}


double HRP2IEEE1394DCImagesInputMethod::NextTimeForGrabbing(int CameraNumber)
{
  return m_LastGrabbingTime[CameraNumber]+ m_GrabbingPeriod[CameraNumber];
}
