#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <sstream>

#include <dc1394/conversions.h>

using namespace std;


using std::showbase;

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "HPR2IEEE1394DCImagesInputMethod:" << x << endl
#define ODEBUG3_CONT(x) cerr << x 

#if 0
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

VisionSystemProfile::VisionSystemProfile()
{
}

VisionSystemProfile::~VisionSystemProfile()
{
  for(unsigned int i=0;i<m_CameraParameters.size();i++)
    {
      if (m_CameraParameters[i]!=0)
	delete m_CameraParameters[i];
    }
}

/**************************************************************
 * class Images Input                                         *
 **************************************************************/
HRP2IEEE1394DCImagesInputMethod::HRP2IEEE1394DCImagesInputMethod() : HRP2ImagesInputMethod()
{

  pthread_mutexattr_t lmutattr;
  pthread_mutexattr_init(&lmutattr);
  pthread_mutex_init(&m_mutex_device,&lmutattr);

  m_mutex_device;

  m_numCameras = 0;
  m_AtLeastOneCameraPresent = false;

  HRP2VisionBasicProcess::m_ProcessName = "IEEE1394 Image grabbing";
  
  string VisionSystemProfileDefault("vsp:default");
  string VSPDValue("Default.vsp");
  SetParameter(VisionSystemProfileDefault,
	     VSPDValue);
  
  /* File descriptor to the frame grabber. */
  ODEBUG("Through the constructor ");
  InitializeBoard();
  ODEBUG("Through the constructor");

  m_LastGrabbingTime.resize(m_numCameras);
  m_Format.resize(m_numCameras);
  for(unsigned int li=0;li<m_numCameras;li++)
    {
      m_LastGrabbingTime[li]=-1.0;
      m_Format[li] = "RGB";
    }

  m_GrabbingPeriod.resize(m_numCameras);
  

  m_Computing = 1;

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
      
      pthread_mutex_lock(&m_mutex_device);
      dc1394_feature_get_value(m_DC1394Cameras[iCamera],lFeature.id,&avalue);
      pthread_mutex_unlock(&m_mutex_device);
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
  //  StopBoard();

  for(unsigned int i=0;i<m_numCameras;i++)
    {
      if (m_TmpImage[i] != 0)
	delete m_TmpImage[i];
    }
}

int HRP2IEEE1394DCImagesInputMethod::StartProcess()
{
  ODEBUG("StartProcess: Phase 1");
  HRP2VisionBasicProcess::StartProcess();
  ODEBUG("StartProcess: Phase 2");
  InitializeBoard();
  ODEBUG("StartProcess: Phase 3");
  StartContinuousShot();
  ODEBUG("StartProcess: Phase 4");
  return 0;
}


int HRP2IEEE1394DCImagesInputMethod::StopProcess()
{
  HRP2VisionBasicProcess::StopProcess();
  return 0;
}

int HRP2IEEE1394DCImagesInputMethod::Initialize()
{
  if (m_DC1394Cameras.size()==0)
    InitializeBoard();
  return 0;
}

int HRP2IEEE1394DCImagesInputMethod::Cleanup()
{
  StopProcess();
  if (m_DC1394Cameras.size()!=0)
    StopBoard();
  return 0;
}


int HRP2IEEE1394DCImagesInputMethod::GetSingleImage(unsigned char **Image, int camera, struct timeval &timestamp)
{
  if (m_Computing==0)
    return -1;

  int r=-1;
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
	  pthread_mutex_lock(&m_mutex_device);
	  if (dc1394_capture_dequeue(m_DC1394Cameras[camera], 
				     DC1394_CAPTURE_POLICY_WAIT, 
				     &m_VideoFrames[camera])
	      !=DC1394_SUCCESS)
	    dc1394_log_error("Failed to capture from camera %d", camera);
	  pthread_mutex_unlock(&m_mutex_device);
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
	  pthread_mutex_lock(&m_mutex_device);
	  if (dc1394_capture_dequeue(m_DC1394Cameras[camera], 
				     DC1394_CAPTURE_POLICY_WAIT, 
				     &m_VideoFrames[camera])
	      !=DC1394_SUCCESS)
	    dc1394_log_error("Failed to capture from camera %d", camera);
	  pthread_mutex_unlock(&m_mutex_device);
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
      
      
      ImgSrc = m_TmpImage[camera];
      
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
    {
      pthread_mutex_lock(&m_mutex_device);
      dc1394_capture_enqueue (m_DC1394Cameras[camera], m_VideoFrames[camera]);
      pthread_mutex_unlock(&m_mutex_device);
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

  struct timeval tval;
  double time1, time2;
#define LOCAL_TYPE unsigned char *

  ODEBUG("GetImageSingleRGB cam: " << camera);
  LOCAL_TYPE ImagesTab[1];
  
  if (m_TmpImage[camera]==0)
    {
      ImagesTab[0] = (LOCAL_TYPE)*Image;
      
      try
	{
	  pthread_mutex_lock(&m_mutex_device);
	  if (dc1394_capture_dequeue(m_DC1394Cameras[camera], DC1394_CAPTURE_POLICY_WAIT, &m_VideoFrames[camera])
	      !=DC1394_SUCCESS)
	    dc1394_log_error("Failed to capture from camera %d", camera);
	  pthread_mutex_unlock(&m_mutex_device);
	  gettimeofday(&timestamp,0);
	}
      catch(std::exception &except)
	{
	  ODEBUG("Exception during snap: " << except.what() );
	}
      ODEBUG("Before converting " << m_BoardImagesWidth[camera] << " " 
	      <<      m_BoardImagesHeight[camera] << " " );
      dc1394_convert_to_RGB8(m_VideoFrames[camera]->image,
			     *Image,
			     m_BoardImagesWidth[camera],
			     m_BoardImagesHeight[camera],
			     DC1394_BYTE_ORDER_UYVY,
			     DC1394_COLOR_CODING_YUV422,1);
      ODEBUG("After converting");
#if 0

      ofstream aof;
      char Buffer[128];
      sprintf(Buffer,"dump_Dst_%d.ppm",camera);
      aof.open(Buffer,ofstream::out);
      aof << "P6\n"<< m_ImagesWidth[camera] << " " << m_ImagesHeight[camera] << endl;
      aof << "255\n";
      
      unsigned char *pt =  *Image;
      for(int j=0;j<m_ImagesHeight[camera]*m_ImagesWidth[camera]*3;j++)
	aof << (unsigned char)*pt++;
      aof.close();
#endif

    }
  else
    {

      ImagesTab[0] = (LOCAL_TYPE) m_TmpImage[camera];

      ImagesDst = *Image;

      try
	{
	  pthread_mutex_lock(&m_mutex_device);
	  if (dc1394_capture_dequeue(m_DC1394Cameras[camera], DC1394_CAPTURE_POLICY_WAIT, &m_VideoFrames[camera])
	      !=DC1394_SUCCESS)
	     dc1394_log_error("Failed to capture from camera %d", camera);
	  pthread_mutex_unlock(&m_mutex_device);
	  gettimeofday(&timestamp,0);
	}
      catch(std::exception &except)
	{
	  ODEBUG("Exception during snap: " << except.what() );
	}

      time1 = timestamp.tv_sec + 0.000001* timestamp.tv_usec;

      ODEBUG("Get Images finito...");

      unsigned char *ImgSrc,*ImgDst;
      ImgDst =(unsigned char *) ImagesDst;
      

      int intervalw, intervalh, indexd, indexs;
      unsigned int BWidth, BHeight;
      BWidth = m_BoardImagesWidth[camera];
      BHeight = m_BoardImagesHeight[camera];
      intervalw = BWidth / m_ImagesWidth[camera];
      intervalh =  BHeight/ m_ImagesHeight[camera];
            
      ImgSrc = m_VideoFrames[camera]->image;

 
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
    {
      pthread_mutex_lock(&m_mutex_device);
      dc1394_capture_enqueue (m_DC1394Cameras[camera], m_VideoFrames[camera]);
      pthread_mutex_unlock(&m_mutex_device);
    }
  
  //gettimeofday(&tval,0);
  //time2 = tval.tv_sec + 0.000001* tval.tv_usec;
  //ODEBUG( time2 - time1);
  m_LastGrabbingTime[camera]= timestamp.tv_sec + 0.000001* timestamp.tv_usec;   
  return 0;
}

int HRP2IEEE1394DCImagesInputMethod::GetImageSingleRaw(unsigned char **Image, int camera, struct timeval &timestamp)
{
  unsigned char * ImagesDst;

#define LOCAL_TYPE unsigned char *

  ODEBUG("GetImageSinglePGM cam: " << camera);
  LOCAL_TYPE ImagesTab[1];
  
  
  if (m_TmpImage[camera]==0)
    {
        ImagesTab[0] = (LOCAL_TYPE)*Image;

      try
	{
	  pthread_mutex_lock(&m_mutex_device);
	  if (dc1394_capture_dequeue(m_DC1394Cameras[camera], DC1394_CAPTURE_POLICY_WAIT, &m_VideoFrames[camera])
	      !=DC1394_SUCCESS)
	     dc1394_log_error("Failed to capture from camera %d", camera);
	  pthread_mutex_unlock(&m_mutex_device);
	  gettimeofday(&timestamp,0);
	}
      catch(std::exception &except)
	{
	  ODEBUG("Exception during snap: " << except.what() );
	}
    }
  else
    {

      ImagesTab[0] = (LOCAL_TYPE) m_TmpImage[camera];

      ImagesDst = *Image;

      ODEBUG("Get Images " );
      try
	{
	  pthread_mutex_lock(&m_mutex_device);
	  if (dc1394_capture_dequeue(m_DC1394Cameras[camera], DC1394_CAPTURE_POLICY_WAIT, &m_VideoFrames[camera])
	      !=DC1394_SUCCESS)
	     dc1394_log_error("Failed to capture from camera %d", camera);
	  pthread_mutex_unlock(&m_mutex_device);
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
      
      
      ImgSrc = m_TmpImage[camera];
      
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
    {
      pthread_mutex_lock(&m_mutex_device);
      dc1394_capture_enqueue (m_DC1394Cameras[camera], m_VideoFrames[camera]);
      pthread_mutex_unlock(&m_mutex_device);
    }

  m_LastGrabbingTime[camera]= timestamp.tv_sec + 0.000001* timestamp.tv_usec;    
  return 0;
}


int HRP2IEEE1394DCImagesInputMethod::SetImageSize(int lw, int lh, int CameraNumber)
{
    
  if ((CameraNumber<0)|| ((unsigned int)CameraNumber>=m_DC1394Cameras.size()))
    return -1;

  m_ImagesWidth[CameraNumber] = lw;
  m_ImagesHeight[CameraNumber] = lh;
  ODEBUG("Debug images");
  ODEBUG("Allocation for m_TmpImage");
  if ((m_BoardImagesWidth[CameraNumber]!=lw) ||
      (m_BoardImagesHeight[CameraNumber]!=lh))
    {
      if (m_TmpImage[CameraNumber]==0)
	m_TmpImage[CameraNumber] = new unsigned char[m_BoardImagesWidth[CameraNumber] * 
						     m_BoardImagesHeight[CameraNumber] * 4];
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
  if ((CameraNumber>=0) && (CameraNumber<m_Format.size()))
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
  else if (CameraPrefix=="vsp:")
    {
      lpos=5;
      string ProfileName = aParameter.substr(lpos,aParameter.length()-lpos);
      ReadConfigurationFileVVVFormat(aValue,ProfileName);
    }
  
  if (IsACamera)
    {
      unsigned char IsFeature=0;
      
      string lFeature = aParameter.substr(lpos,aParameter.length()-lpos);
      for(unsigned int i=0;i<m_Features.size();i++)
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
  ODEBUG("Start InitializeBoard");
  try 
    {

      /*! List of camera. */
      dc1394camera_list_t * list;
  
      m_HandleDC1394 =  dc1394_new ();
      dc1394error_t err;
      err = dc1394_camera_enumerate(m_HandleDC1394, &list);

      m_DC1394Cameras.resize(list->num);
      m_VideoFrames.resize(list->num);
      m_BoardImagesWidth.resize(list->num);
      m_BoardImagesHeight.resize(list->num);
      m_ImagesWidth.resize(list->num);
      m_ImagesHeight.resize(list->num);
      m_TmpImage.resize(list->num);
      m_GrabbingPeriod.resize(list->num);
      unsigned int j=0;
      for (unsigned i = 0; i < list->num; i++) {
        m_DC1394Cameras[j] = dc1394_camera_new (m_HandleDC1394, list->ids[i].guid);
        if (!m_DC1394Cameras[j]) {
	  dc1394_log_warning("Failed to initialize camera with guid %llx", list->ids[i].guid);
	  continue;
        }
	else
	  {
	    cout << "Initialized " << i << " camera with guid "<<list->ids[i].guid << endl;
	  }
        j++;
      }
      m_numCameras = j;
      dc1394_camera_free_list (list);

      if (m_numCameras==0)
	m_AtLeastOneCameraPresent = false;
      else
	m_AtLeastOneCameraPresent = true;
    
    }
  catch(...)
    {
      ODEBUG("Unable to initialize the board correctly\n");
      return;
    }
  InitializeCameras();
  ODEBUG("End of InitializeBoard");
}


void HRP2IEEE1394DCImagesInputMethod::DecideBasicFeatureOnCamera(dc1394camera_t &aCamera,
								 dc1394video_mode_t &res,
								 dc1394framerate_t &fps,
								 unsigned int InternalCameraNb)
{
  ODEBUG3("Vendor name :" << aCamera.vendor << " aCamera name " << aCamera.model);

  if (!strcmp(aCamera.vendor,"Unibrain"))
    {
      if (!strcmp(aCamera.model,"Fire-i 1.2"))
	{
	  res = DC1394_VIDEO_MODE_320x240_YUV422;
	  fps = DC1394_FRAMERATE_30;
	  m_BoardImagesWidth[InternalCameraNb]= 320;
	  m_BoardImagesHeight[InternalCameraNb]= 240;
	  ODEBUG3("Found the camera settings !");
	}
    }
  if (!strcmp(aCamera.vendor,""))
    {
      if (!strcmp(aCamera.model,"Fire-i 1.2"))
	{
	  res = DC1394_VIDEO_MODE_320x240_YUV422;
	  fps = DC1394_FRAMERATE_30;
	  m_BoardImagesWidth[InternalCameraNb]= 320;
	  m_BoardImagesHeight[InternalCameraNb]= 240;
	  ODEBUG3("Found the camera settings !");
	}
    }
}
void HRP2IEEE1394DCImagesInputMethod::InitializeCameras()
{
  ODEBUG("Begin InitializeCameras()");
  if (!m_AtLeastOneCameraPresent)
    return;

  for (unsigned int i = 0; i < m_numCameras; i++) 
    {
      
      dc1394video_mode_t res=DC1394_VIDEO_MODE_320x240_YUV422;
      dc1394framerate_t fps=DC1394_FRAMERATE_30;
      
      DecideBasicFeatureOnCamera(*m_DC1394Cameras[i],res,fps,i);
      dc1394error_t err;
      unsigned int NUM_BUFFERS=8;
      
      pthread_mutex_lock(&m_mutex_device);      
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
      pthread_mutex_unlock(&m_mutex_device);

      if (fps==DC1394_FRAMERATE_30)
	{
	  m_GrabbingPeriod[i]=1.0/30.0;
	}
    }
  ODEBUG("End InitializeCameras()");
}
    



void HRP2IEEE1394DCImagesInputMethod::StartContinuousShot()
{
  if (!m_AtLeastOneCameraPresent)
    return;

  dc1394error_t err;
  for(unsigned int i=0;i<m_DC1394Cameras.size();i++)
    {
      err=dc1394_video_set_transmission(m_DC1394Cameras[i], DC1394_ON);
      DC1394_ERR(err,"Could not start camera iso transmission");
    }

}

void HRP2IEEE1394DCImagesInputMethod::StopContinuousShot()
{
  if (!m_AtLeastOneCameraPresent)
    return;

  for(unsigned int i=0;i<m_DC1394Cameras.size();i++)
    {
      pthread_mutex_lock(&m_mutex_device);
      dc1394_video_set_transmission(m_DC1394Cameras[i], DC1394_OFF);
      dc1394_capture_stop(m_DC1394Cameras[i]);
      pthread_mutex_unlock(&m_mutex_device);
    }
}


void HRP2IEEE1394DCImagesInputMethod::StopBoard()
{
  for(unsigned int i=0;i<m_DC1394Cameras.size();i++)
    {
      pthread_mutex_lock(&m_mutex_device);
      dc1394_video_set_transmission(m_DC1394Cameras[i], DC1394_OFF);
      dc1394_capture_stop(m_DC1394Cameras[i]);
      dc1394_camera_free(m_DC1394Cameras[i]);
      pthread_mutex_unlock(&m_mutex_device);
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

bool HRP2IEEE1394DCImagesInputMethod::CameraPresent()
{
  return m_AtLeastOneCameraPresent;
}

void HRP2IEEE1394DCImagesInputMethod::ReadConfigurationFileVVVFormat(string aFileName,
								     string ProfileName)
{
  ifstream aif;
  unsigned int lBoardNumber;
  unsigned int lNbOfCameras;

  VisionSystemProfile aVSP;

  aif.open((const char *)aFileName.c_str(),ifstream::in);
  if (aif.is_open())
    {
      aVSP.m_Name = ProfileName;
      aVSP.m_FileNameDescription = aFileName;

      aif >> lBoardNumber;
      aif >> lNbOfCameras;
      
      aVSP.m_CameraParameters.resize(lNbOfCameras);

      for(unsigned int i=0;i<lNbOfCameras;i++)
	{
	  string lGUID,lFormat,tmp,lFPS;
	  unsigned int lBrightness, lExposure;
	  aVSP.m_CameraParameters[i] = new IEEE1394DCCameraParameters();
	  aVSP.m_CameraParameters[i]->SetCameraNumberInUserSemantic(i);
	  aVSP.m_CameraParameters[i]->SetBoardNumber(lBoardNumber);
	  
	  aif >> lGUID;
	  aVSP.m_CameraParameters[i]->SetGUID(lGUID);
	  ODEBUG3("GUID:" << lGUID);

	  aif >> lFormat;
	  aVSP.m_CameraParameters[i]->SetFormat(lFormat);
	  ODEBUG3("Format:" << lFormat);

	  aif >> lFPS;
	  aVSP.m_CameraParameters[i]->SetFPS(lFPS);
	  ODEBUG3("FPS:" << lFPS);

	  aif >> tmp;
	  if (tmp=="BRIGHTNESS")
	    {
	      aif >> lBrightness;
	      aVSP.m_CameraParameters[i]->SetBrightness(lBrightness);
	      ODEBUG3("Brightness:" << lBrightness);
	    }
	  aif >> tmp;
	  if (tmp=="AUTO_EXPOSURE")
	    {
	      aif >> lExposure;
	      aVSP.m_CameraParameters[i]->SetExposure(lExposure);
	      ODEBUG3("Exposure:" << lExposure);
	    }
	  aif >> tmp;
	  if (tmp=="WHITE_BALANCE")
	    {
	      unsigned int lWhiteBalance[2];
	      aif >> lWhiteBalance[0];
	      aif >> lWhiteBalance[1];
	      aVSP.m_CameraParameters[i]->SetWhiteBalance(lWhiteBalance);
	      ODEBUG3("WhiteBalance : " << lWhiteBalance[0] << " " <<lWhiteBalance[1]);
	    }
	  aif >> tmp;
	  if (tmp=="GAMMA")
	    {
	      unsigned int lGamma;
	      aif >> lGamma;
	      aVSP.m_CameraParameters[i]->SetGamma(lGamma);
	      ODEBUG3("Gamma : " << lGamma);
	    }
	  
	  aif >> tmp;
	  if (tmp=="SHUTTER")
	    {
	      unsigned int lShutter;
	      aif >> lShutter;
	      aVSP.m_CameraParameters[i]->SetShutter(lShutter);
	      ODEBUG3("Shutter : " << lShutter);
	    }

	  	  aif >> tmp;
	  if (tmp=="GAIN")
	    {
	      unsigned int lGain;
	      aif >> lGain;
	      aVSP.m_CameraParameters[i]->SetGain(lGain);
	      ODEBUG3("Gain : " << lGain);
	    }
	  
	}
      
      aif.close();

      m_VisionSystemProfiles.insert(m_VisionSystemProfiles.end(),
					    aVSP);
    }
}

void HRP2IEEE1394DCImagesInputMethod::DetectTheBestVisionSystemProfile()
{
  int IndexBestCandidate=-1;
  int ScoreBestCandidate=-1;
  vector<unsigned int> lScoreCandidates;
  lScoreCandidates.resize(m_VisionSystemProfiles.size());
  
  for(unsigned int i=0;i<m_VisionSystemProfiles.size();i++)
    {
      lScoreCandidates[i] = 0;

      for(unsigned int j=0;j<m_VisionSystemProfiles[i].m_CameraParameters.size();j++)
	{
	  string sVSPCameraGUID = m_VisionSystemProfiles[i].m_CameraParameters[i]->GetGUID();
	  istringstream is(sVSPCameraGUID);
	  uint64_t VSPCameraGUID;
	  is >> VSPCameraGUID;
	  ODEBUG3("VSPCameraGUID: " << VSPCameraGUID);
	  for(unsigned int k=0;k<m_DC1394Cameras.size();k++)
	    {
	      if (VSPCameraGUID==m_DC1394Cameras[k]->guid)
		lScoreCandidates[i]++;
	    }
	}
      if (lScoreCandidates[i]>(unsigned int)ScoreBestCandidate)
	{
	  IndexBestCandidate = (int)i;
	  ScoreBestCandidate = (int)lScoreCandidates[i];
	}
    }
  if ((IndexBestCandidate>-1) && (ScoreBestCandidate>0))
    {
      m_CurrentVisionSystemProfileID = IndexBestCandidate;
    }
  
}
