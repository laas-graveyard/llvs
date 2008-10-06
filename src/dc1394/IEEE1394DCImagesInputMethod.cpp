#include <stdlib.h>
#include <iostream>
#include <fstream>


using namespace std;


using std::showbase;

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "HPR2IEEE1394DCImagesInputMethod:" << x << endl
#define ODEBUG3_CONT(x) cerr << x 

#if 1
#define ODEBUG(x) cerr << "HPR2IEEE1394DCImagesInputMethod:" <<  x << endl
#define ODEBUG_CONT(x) cerr << "HPR2IEEE1394ImagesInputMethod:" <<  x << endl
#else
#define ODEBUG(x) 
#define ODEBUG_CONT(x) 
#endif



#include <dc1394/IEEE1394DCImagesInputMethod.h>

/************************************************************************
*  static functions							*
************************************************************************/


using namespace llvs;
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
  m_Format.resize(m_numCameras);
  for(unsigned int li=0;li<m_numCameras;li++)
    {
      m_LastGrabbingTime[li]=-1.0;
      m_Format[li] = "PGM";
    }

  m_GrabbingPeriod.resize(m_numCameras);
  

  m_Computing = 1;

  HRP2VisionBasicProcess::m_ProcessName = "IEEE1394 Image grabbing";

  

  string Prefixes[4] = { "LEFT", "RIGHT","CYCL","WIDE"};
  string Features[6] = { "BRIGHTNESS", "AUTO_EXPOSURE","GAMMA","SHUTTER","GAIN","Format"};  

  for(int i=0;i<4;i++)
    m_Prefixes[i] = Prefixes[i];
  
  for(int i=0;i<5;i++)
    m_Features[i] = Features[i];
  
  ODEBUG("Before setting the parameters.");

  for(int i=0;i<4;i++)
    {

      for(int j=0;j<6;j++)
	{
	  string ParameterName = m_Prefixes[i] + "|" + m_Features[j];
	  string ParameterValue;
	  GetCameraFeatureValue(m_Prefixes[i],m_Features[j],ParameterValue);
	  HRP2VisionBasicProcess::SetParameter(ParameterName,ParameterValue);
	}
    }
  for(unsigned int i=0;i<m_numCameras;i++)
    FromFrameRateToTime(i);
    
  StartContinuousShot();
  ODEBUG("After setting the parameters.");
}

void HRP2IEEE1394DCImagesInputMethod::GetCameraFeatureValue(string aCamera, string aFeature, string &aValue)
{

  int iCamera=0;
  bool Is1394Feature=true;
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
    lFeature.id = DC1394_FEATURE_BRIGHTNESS;
  else if (aFeature=="AUTO_EXPOSURE")
    lFeature.id = DC1394_FEATURE_EXPOSURE;
  else if (aFeature=="WHITE_BALANCE")
    lFeature.id = DC1394_FEATURE_WHITE_BALANCE;
  else if (aFeature=="GAMMA")
    lFeature.id = DC1394_FEATURE_GAMMA;
  else if (aFeature=="SHUTTER")
    lFeature.id = DC1394_FEATURE_SHUTTER;
  else if (aFeature=="GAIN")
    lFeature.id = DC1394_FEATURE_GAIN;
  else if (aFeature=="Format")
    {
      Is1394Feature = false;
      aValue = m_Format[iCamera];
      return;
    }

  if(Is1394Feature)
    {
      u_int avalue=0;
      
      dc1394_feature_get_value(m_DC1394Cameras[iCamera],lFeature.id,&avalue);
      char Buffer[1024];
      bzero(Buffer,1024);
      sprintf(Buffer,"%d",avalue);
      aValue = Buffer;
    }
}

void HRP2IEEE1394DCImagesInputMethod::SetCameraFeatureValue(string aCamera, string aFeature, string aValue)
{
  dc1394feature_info_t lFeature;
  bool Is1394Feature=true;
  unsigned int iCamera;

  if (aCamera=="LEFT")
    iCamera = 0;
  else if (aCamera=="RIGHT")
    iCamera = 1;
  else if (aCamera=="CYCL")
    iCamera = 2;
  else if (aCamera=="WIDE")
    iCamera = 3;

  if (aFeature=="BRIGHTNESS")
    lFeature.id = DC1394_FEATURE_BRIGHTNESS;
  else if (aFeature=="AUTO_EXPOSURE")
    lFeature.id = DC1394_FEATURE_EXPOSURE;
  else if (aFeature=="WHITE_BALANCE")
    lFeature.id = DC1394_FEATURE_WHITE_BALANCE;
  else if (aFeature=="GAMMA")
    lFeature.id = DC1394_FEATURE_GAMMA;
  else if (aFeature=="SHUTTER")
    lFeature.id = DC1394_FEATURE_SHUTTER;
  else if (aFeature=="GAIN")
    lFeature.id = DC1394_FEATURE_GAIN;
  else if (aFeature=="Format")
    {
      // TODO : Improve the way we can change the format
      // depending on the camera system.
      bool Is1394Feature=false;
      m_Format[iCamera] = aValue;
      return;
    }

  
  StopContinuousShot();
  u_int avalue;
  avalue = atoi(aValue.c_str());
  dc1394_feature_set_value(m_DC1394Cameras[iCamera],lFeature.id,avalue);
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
  ODEBUG(m_Format[camera]);
  if (m_Format[camera]=="PGM")
    r = GetImageSinglePGM(Image,camera,timestamp);
  else if (m_Format[camera]=="RAW")
    r = GetImageSingleRaw(Image,camera,timestamp);
  else if (m_Format[camera]=="RGB")
    r = GetImageSingleRGB(Image,camera,timestamp);
  return r;
}



int HRP2IEEE1394DCImagesInputMethod::GetImageSinglePGM(unsigned char **Image, int camera, struct timeval &timestamp)
{

  if ((camera<0) || ((unsigned int)camera> m_DC1394Cameras.size()))
    return -1;
  unsigned char * ImagesDst;

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
	  if (dc1394_capture_dequeue(m_DC1394Cameras[camera], DC1394_CAPTURE_POLICY_WAIT, &m_VideoFrames[camera])
	      !=DC1394_SUCCESS)
	      dc1394_log_error("Failed to capture from camera %d", camera);
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
	  if (dc1394_capture_dequeue(m_DC1394Cameras[camera], DC1394_CAPTURE_POLICY_WAIT, &m_VideoFrames[camera])
	      !=DC1394_SUCCESS)
	    dc1394_log_error("Failed to capture from camera %d", camera);
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
      BWidth = m_BoardImagesWidth[camera];
      BHeight = m_BoardImagesHeight[camera];
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

  
  if (m_VideoFrames[camera])
    dc1394_capture_enqueue (m_DC1394Cameras[camera], m_VideoFrames[camera]);


  //gettimeofday(&tval,0);
  //time2 = tval.tv_sec + 0.000001* tval.tv_usec;
  //ODEBUG3( time2 - time1);

  m_LastGrabbingTime[camera]= timestamp.tv_sec + 0.000001* timestamp.tv_usec;
  return 0;
}

int HRP2IEEE1394DCImagesInputMethod::GetImageSingleRGB(unsigned char **Image, int camera, struct timeval &timestamp)
{
  unsigned char * ImagesDst;

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
	  if (dc1394_capture_dequeue(m_DC1394Cameras[camera], DC1394_CAPTURE_POLICY_WAIT, &m_VideoFrames[camera])
	      !=DC1394_SUCCESS)
	    dc1394_log_error("Failed to capture from camera %d", camera);
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
	  if (dc1394_capture_dequeue(m_DC1394Cameras[camera], DC1394_CAPTURE_POLICY_WAIT, &m_VideoFrames[camera])
	      !=DC1394_SUCCESS)
	     dc1394_log_error("Failed to capture from camera %d", camera);
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
      BWidth = m_BoardImagesWidth[camera];
      BHeight = m_BoardImagesHeight[camera];
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
  
  if (m_VideoFrames[camera])
    dc1394_capture_enqueue (m_DC1394Cameras[camera], m_VideoFrames[camera]);
  
  //gettimeofday(&tval,0);
  //time2 = tval.tv_sec + 0.000001* tval.tv_usec;
  //ODEBUG3( time2 - time1);
  m_LastGrabbingTime[camera]= timestamp.tv_sec + 0.000001* timestamp.tv_usec;   
  return 0;
}

int HRP2IEEE1394DCImagesInputMethod::GetImageSingleRaw(unsigned char **Image, int camera, struct timeval &timestamp)
{
  unsigned char * ImagesDst;

#define LOCAL_TYPE unsigned char *

  ODEBUG("GetImageSinglePGM cam: " << camera);
  LOCAL_TYPE ImagesTab[1];
  
  
  if (m_TmpImage[0]==0)
    {
        ImagesTab[0] = (LOCAL_TYPE)*Image;

      try
	{
	  if (dc1394_capture_dequeue(m_DC1394Cameras[camera], DC1394_CAPTURE_POLICY_WAIT, &m_VideoFrames[camera])
	      !=DC1394_SUCCESS)
	     dc1394_log_error("Failed to capture from camera %d", camera);

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
	  if (dc1394_capture_dequeue(m_DC1394Cameras[camera], DC1394_CAPTURE_POLICY_WAIT, &m_VideoFrames[camera])
	      !=DC1394_SUCCESS)
	     dc1394_log_error("Failed to capture from camera %d", camera);
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
      BWidth = m_BoardImagesWidth[camera];
      BHeight = m_BoardImagesHeight[camera];

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
  if (m_VideoFrames[camera])
    dc1394_capture_enqueue (m_DC1394Cameras[camera], m_VideoFrames[camera]);

  m_LastGrabbingTime[camera]= timestamp.tv_sec + 0.000001* timestamp.tv_usec;    
  return 0;
}


int HRP2IEEE1394DCImagesInputMethod::SetImageSize(int lw, int lh, int CameraNumber)
{
    
  int i;
  ODEBUG("Debug images");
  for(i=0;i<4;i++)
    {
      m_ImagesWidth[i] = lw;
      m_ImagesHeight[i] = lh;
    }

  if ((m_BoardImagesWidth[CameraNumber] != lw) ||
      (m_BoardImagesHeight[CameraNumber] != lh) ||
      (m_Format[CameraNumber]!="YUV422"))
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
		m_TmpImage[i] = new unsigned char[m_BoardImagesWidth[CameraNumber] * 
						  m_BoardImagesHeight[CameraNumber] * 4];
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


string HRP2IEEE1394DCImagesInputMethod::GetFormat(unsigned int CameraNumber)
{
  if ((CameraNumber>0) && (CameraNumber<m_Format.size()))
      return m_Format[CameraNumber];
  string ErrorMsg("Error Format : wrong camera id.");
  return ErrorMsg;
}

int HRP2IEEE1394DCImagesInputMethod::SetFormat(string aFormat, unsigned int CameraNumber)
{
  if ((CameraNumber<0) || (CameraNumber>=m_Format.size()))
    return -1;

  if (aFormat=="RGB")
    {
      m_Format[CameraNumber] = "RGB";
      ODEBUG("Format Image : RGB");
      return 0;
    }
  else if (aFormat=="PGM")
    {
      m_Format[CameraNumber] = "PGM";
      ODEBUG("Format Image : PGM");
      return 0;
    }
  else if (aFormat=="RAW")
    {
      m_Format[CameraNumber] = "RAW";
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
      unsigned int lindex=0; // left by default.
      if (CameraPrefix=="RIGH")
	lindex =1 ;
      else if (CameraPrefix=="CYCL")
	lindex =2 ;
      else if (CameraPrefix=="WIDE")
	lindex =3 ;

      SetFormat(aValue,lindex);      
    }
  return 0;
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

      m_DC1394Cameras.resize(list->num);
      m_VideoFrames.resize(list->num);

      unsigned int j=0;
      for (unsigned i = 0; i < list->num; i++) {
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
  
  dc1394video_mode_t res=DC1394_VIDEO_MODE_640x480_YUV411;
  dc1394framerate_t fps=DC1394_FRAMERATE_30;
  dc1394error_t err;
  unsigned int NUM_BUFFERS=8;
      
  for (unsigned int i = 0; i < m_numCameras; i++) 
    {
      
      err=dc1394_video_set_iso_speed(m_DC1394Cameras[i], DC1394_ISO_SPEED_400);
      DC1394_ERR(err,"Could not set ISO speed");
      
      err=dc1394_video_set_mode(m_DC1394Cameras[i], res);
      DC1394_ERR(err,"Could not set video mode");
      
      err=dc1394_video_set_framerate(m_DC1394Cameras[i], fps);
      DC1394_ERR(err,"Could not set framerate");
      
      err=dc1394_capture_setup(m_DC1394Cameras[i],NUM_BUFFERS, DC1394_CAPTURE_FLAGS_DEFAULT);
      DC1394_ERR(err,"Could not setup camera-\nmake sure \
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
      DC1394_ERR(err,"Could not start camera iso transmission");
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

unsigned int HRP2IEEE1394DCImagesInputMethod::GetNumberOfCameras()
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
