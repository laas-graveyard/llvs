/** @doc This object implements a visual process to get a disparity map.

    Copyright (c) 2010, 
    @author Stephane Embarki
   
    JRL-Japan, CNRS/AIST
    
    See license file for information on license.
*/
#include "PointTracker/PointTrackingProcess.h"

#if (LLVS_HAVE_VISP>0)

#include <sstream>

#include <visp/vpConfig.h>
#include <visp/vpXmlParserCamera.h>
#include <visp/vpImageIo.h>
#include <visp/vpImagePoint.h>
#include <visp/vpDisplayX.h>
#include <visp/vpPose.h>
#include <visp/vpPixelMeterConversion.h>
#include <Debug.h>


/*!-------------------------------------
Default constructor
 -------------------------------------*/

HRP2PointTrackingProcess::HRP2PointTrackingProcess()
{
  m_ProcessName = "PointTrackingProcess";
  m_cameraParamLoaded = false;
  m_inputImagesLoaded = false;
  m_InitDone = false;
  m_trackerTrackSuccess =false;
  m_inputVispImage=0x0;
  SetDefaultParam();
}



/*!-------------------------------------
Destructor
------------------------------------- */
HRP2PointTrackingProcess:: ~HRP2PointTrackingProcess()
{
  m_cameraParamLoaded = false;
  m_inputImagesLoaded = false;
  m_InitDone = false;
  m_trackerTrackSuccess =false;
  m_inputVispImage=0x0;
}


/*!------------------------------------- 
Set tracker parameters : camera parameters 
-------------------------------------*/
void HRP2PointTrackingProcess::SetCameraParameters(const vpCameraParameters & _cam)
{
  m_cam=_cam;
}

/*! Get tracker parameters : camera parameters */
void  HRP2PointTrackingProcess::GetCameraParameters(vpCameraParameters & _cam)
{
  _cam=m_cam;
}  


/*! Get the image */
void  HRP2PointTrackingProcess::GetInputVispImages(vpImage<unsigned char> & _I)
{
  _I=*(m_inputVispImage);
}  
  
  
/*! Get the inputcMo */
void  HRP2PointTrackingProcess::GetOutputcMo(vpHomogeneousMatrix & _outputcMo)
{
  _outputcMo=this->m_outputcMo;
} 

/*! Get the vpDot2*/
void HRP2PointTrackingProcess::GetvpDot2( vector<vpDot2*> &DotList)
{
  DotList.resize(m_NbPoint);
  for(unsigned int i=0; i<m_NbPoint;++i)
    {
      *DotList[i]=*m_Dot2List[i];
    }

}
 
/*! Get the vpImagePoint*/
void HRP2PointTrackingProcess::GetvpImagePoint(vector<vpImagePoint*> &IPList)
{
  if(IPList.size()!=m_vpIPList.size())
    {
       for(unsigned int i=0;i<IPList.size();i++)
	 {
	   delete IPList[i];
	 }
       IPList.clear();
       IPList.resize(m_NbPoint);
       for(unsigned int i=0;i<IPList.size();i++)
	 {
	   IPList[i]= new vpImagePoint;
	 }
    }

  for(unsigned int i=0;i<m_NbPoint;i++)
    {
      *IPList[i]=*m_vpIPList[i];
    }
}


/*! Get Image Height*/
void  HRP2PointTrackingProcess::GetHeight(int&_height)
{
  _height= m_imageHeight;
}   

/*! Get Image Width*/
void  HRP2PointTrackingProcess::GetWidth(int&_width)
{
  _width = m_imageWidth;
}  

/*!-------------------------------------
   Set Default Parameters
    - cMo  : init pose of the object in the image
    - path : path to file .wrl, the target
-------------------------------------*/
int HRP2PointTrackingProcess::SetDefaultParam()
{
 
  //---------------------------------
  // Load the default camera parameters
  //--------------------------------- 

  // init path to xml file
  string camParamPath ("./data/ViSP/hrp2CamParam/hrp2.xml");
  m_pathCam =camParamPath;

  // init cam name
  m_nameCam = "cam1394_3_rectif";

  // init proje Type
  m_projType= vpCameraParameters::perspectiveProjWithoutDistortion;
  
  // init image dim
  m_imageWidth= 320;
  m_imageHeight= 240;
  
  // parse the cam
  ParseCamParam();

  // set the tracker cam parameters
  //m_tracker.setCameraParameters(m_cam) ;
  m_cameraParamLoaded = true;

  //---------------------------------
  // everything is ok
  //---------------------------------
  return 0;

}


/*!-------------------------------------

Parse camera parameters

----------------------------------------*/
int HRP2PointTrackingProcess::ParseCamParam()
{

#if defined(VISP_HAVE_XML2) 
  //create a parser
  vpXmlParserCamera parser;
  parser.parse(m_cam,
		m_pathCam.c_str(),
		m_nameCam.c_str(),
		m_projType,
		m_imageWidth, 
		m_imageHeight);

  ODEBUG("camera parameter:\n"<< m_cam);
  ODEBUG("camera Path:\n"<< m_pathCam.c_str());
   m_cameraParamLoaded=true;
   return 0;
#else
  
   cerr << "Error: No parser available cannot parse the camera file"<<end;
   return -1;
   
#endif

   
}


/*-------------------------------------

Set input image

 --------------------------------------*/

void HRP2PointTrackingProcess::SetInputVispImages(vpImage<unsigned char> * _I)
{
  m_inputVispImage=_I;
  m_inputImagesLoaded=true;
 
  // if the dimension are different from the stored
  // one update the dimension and the calib param
  if (m_imageWidth!=(int) m_inputVispImage->getWidth())
    {
      m_imageWidth  = m_inputVispImage->getWidth();
      m_imageHeight = m_inputVispImage->getWidth();
      ParseCamParam();
    }
}  


/*!-------------------------------------
 Sets the tracker parameters

The parameter names can be :

parameters for the moving edge tracking

PATH_CAM              path name to the cam xml file

CAME_PROJ_TYP         camera projection type : "withDistorsion" or "withoutDistorsion"
CAME_NAME             camera name

-------------------------------------*/
int HRP2PointTrackingProcess::pSetParameter(std::string aParameter, std::string aValue)
{
  // use of the generic function to add the parameter in the parameter list
  // A parameter can be or cannot be associated with a value, 
  // thus an empty string for Value is correct.
  // If the parameter already exist is value is overwritten. 
  // If this is valid the index parameter >=0 is returned,
  // -1 otherwise.

  //int outputVBPSetParameters = HRP2VisionBasicProcess::SetParameter(aParameter,aValue);

  // get the 4 first parameter to find the parameter type
  // get 4 letters starting from the letter number 0
  string paramType = aParameter.substr(0,4);
  // get the  first parameter to find the parameter type
  // get 3 letters starting from the letter number 5
  string paramId = aParameter.substr(5,3);
  
  //cout << "paramType: " << paramType   << endl;
  //cout << "paramId: " << paramId   << endl;
  bool isAPathParam(false);
  bool isACameraParam(false);

  if(paramType=="PATH")
    {
      isAPathParam = true;
    }
  else if(paramType=="CAME")
    {
      isACameraParam =true; 
    }
  else
    {
      cout << "Warning : unknown parameter :"<< aParameter << endl; 
      return -1;
    }

  
//-------- PATH ------------//
   if(isAPathParam)
    {
     if (paramId =="CAM")//"PATH_CAM"
       {
         m_pathCam = aValue;
         ParseCamParam();
       }
     else 
       {
	 cout << "Warning : unknown path parameter :"<< paramId << endl; 
	 return -1;
       }
    }
   
  
//------- CAMERA--------------//
  else if(isACameraParam )
    {
      if (paramId=="PRO")//"CAME_PROJ_TYP"
       { 
         if(aValue=="withDistorsion")
	   {
	     m_projType=vpCameraParameters::perspectiveProjWithDistortion;
	     ParseCamParam();
	   }
	 else if(aValue=="withoutDistorsion")
	   {
	     m_projType=vpCameraParameters::perspectiveProjWithoutDistortion;
	     ParseCamParam();
	   }
	 else
           {
	     cout << "Warning : unknown distorsion type :"<< aValue << endl; 
             return -2;
           }

       }
      else if(paramId=="NAM")//"CAME_NAME"
       {
	 m_nameCam = aValue;
	 ParseCamParam();
       }
      else 
       {
	 cout << "Warning : unknown path parameter :"<< paramId << endl; 
	 return -1;
       }

    }
 
  //return(outputVBPSetParameters);
  return 0;
}


/*!------------------------------------- 
Initialize the process : initialize the Tracking

-------------------------------------*/
int HRP2PointTrackingProcess:: pInitializeTheProcess()
{

  for (unsigned int i = 0 ; i < m_NbPoint; i++)
    {
     
      m_Dot2List[i]->initTracking( *m_inputVispImage,*m_vpIPList[i]);
    } 

  m_outputcMo.setIdentity();
  m_trackerTrackSuccess = false;
 
  return 0;
}


/*!------------------------------------- 
Realize the process 
the tracker has previously been initialised with: 
a cMo that is the init transform between the camera and the object
a pointer on an Image
some parameter for the tracking
the object model
   
-------------------------------------*/
int HRP2PointTrackingProcess::pRealizeTheProcess()
{
  m_trackerTrackSuccess = false;
 
  if(m_inputImagesLoaded && m_InitDone )
    {
 
      try
	{  	
	  Tracking();
	}
      catch(std::string a) // tracking got lost
	{
	    
	  // set the tracking flag
	  m_trackerTrackSuccess= false;
	  
	  // set the cMo matrix to identity   
	  m_outputcMo.setIdentity();
	  
	  // return a negative value
	  return -1;
	}
    
      // tracking succeed
      m_trackerTrackSuccess= true;
      
      // Compute the resulting transform between the object and the image  
      computePose();
      
      return 0;
    }
  else 
    {
      cerr << "Error :HRP2PointTrackingProcess::RealizeTheProcess>>  No image or No init " <<endl; 
      return -1;   
    }
}


/*!-------------------------------------
  Set the target model 
  and the initial coordinate for the tracked Dot 
-------------------------------------*/
int HRP2PointTrackingProcess::Init(vector<vpPoint> Target,
				   vector<vpImagePoint*> IP,
				   unsigned int nbPoint)
{
  

  if( Target.size()==nbPoint && IP.size()==nbPoint)
    {
      
      m_NbPoint=nbPoint;
      
      m_PointList.resize(m_NbPoint);
      m_PointList = Target;
      
      m_Dot2List.resize(m_NbPoint);
      m_vpIPList.resize(m_NbPoint);

      for (unsigned int i =0; i<m_NbPoint; ++i)
	{
	  m_vpIPList[i]=new vpImagePoint;
	  m_Dot2List[i]=new vpDot2;
	  *(m_vpIPList[i])=*(IP[i]);
	}  

    }
  else
    {
      cerr << "The vector haven't the size :: " << nbPoint <<endl; 
      return -1;
    }

  pInitializeTheProcess();

  m_InitDone=true;

  return 0;

}



/*!-------------------------------------
 Realize the Traking 
-------------------------------------*/
int HRP2PointTrackingProcess::Tracking()
{
  for (unsigned int i = 0 ; i < m_NbPoint; i++)
    {
      m_Dot2List[i]->track( *m_inputVispImage, *m_vpIPList[i] ) ;
    }
  return 0;
}
/*-------------------------------------
  Compute the Pose
-------------------------------------*/
int HRP2PointTrackingProcess::computePose()
{
  // pixel-> meter conversion
  for (unsigned int i=0 ; i < m_NbPoint ; i++)
    {
      // u[i]. v[i] are expressed in pixel
      // conversion in meter is achieved using
      // x = (u-u0)/px
      // y = (v-v0)/py
      // where px, py, u0, v0 are the intrinsic camera parameters
      double x,y ;
      vpPixelMeterConversion::convertPoint( m_cam ,*m_vpIPList[i] , x , y )  ;
      m_PointList[i].set_x(x) ;
      m_PointList[i].set_y(y) ;
    }
  
  
  // The vpPose class mainly contents a list of vpPoint (that is (X,Y,Z, x, y) )
  vpPose pose ;
  pose.clearPoint() ;
    
  for (unsigned int i= 0 ; i < m_NbPoint ; ++i)
    {
      pose.addPoint( m_PointList[i]) ; // and added to the pose computation point list
    }
  
  // compute the initial pose using Dementhon method followed by a non linear
  // minimisation method
  
  
  // Pose by Lagrange it provides an initialization of the pose
  vpHomogeneousMatrix cMo;
  pose.computePose(vpPose::LAGRANGE, cMo ) ;
  
  // the pose is now refined using the virtual visual servoing approach
  // Warning: cMo needs to be initialized otherwise it may  diverge
  pose.computePose( vpPose::LOWE, cMo ) ;
  
  m_outputcMo=cMo;

  return 0;
}
 
  
/*!-------------------------------------
 Cleanup the process 
-------------------------------------*/
int HRP2PointTrackingProcess::pCleanUpTheProcess()
{
  //m_tracker.resetTracker();
  return 0;
}


 
#endif
