/** @doc This object implements a visual process to get a disparity map.

    Copyright (c) 2010, 
    @author Stephane Embarki
   
    JRL-Japan, CNRS/AIST
    
    See license file for information on license.
*/



#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <visp/vpParseArgv.h>

#include <iostream>

//#include <windows.h>
//#include <winbase.h>

#include "llvsConfig.h"

#if (LLVS_HAVE_VISP>0)


#include "PointTracker/PointTrackingProcess.h"

//#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vp1394TwoGrabber.h> 

//#include <visp/vpPixelMeterConversion.h>

#include <visp/vpPose.h>
//#include <visp/vpDot2.h>
//#include <visp/vpImagePoint.h>

// List of allowed command line options
#define GETOPTARGS  "di:p:hf:g:n:s:l:c"


using namespace std;

int main()
{
  
  // creation d une image en niveau de gris
  vpImage<unsigned char> I ;
  
  //creer un grabber pour communiquer avec la webcam
  vp1394TwoGrabber g  ;
  
  // displays pour l'affichage
  vpDisplayX d;
  
  // on recupere directement ces information du grabber
  vpTRACE(" grab the images");
  try{
    
    // on etablit la connexion
     g.open(I);
    // acquire une image
    //get image
    
    // acquiert depuis la web cam et stocke dans I 
    g.acquire(I);
    

    
  }
  catch(...){
    vpTRACE ( "Cannot acquire an image... " ) ;
    exit(-1);
    
  }

  // set the display properties
    char title[50];
    sprintf(title,"Image");
    d.init(I,100,100,title); 
    
    // affichage : dispolay + flush
    
    vpDisplay::display(I);
    vpDisplay::flush(I);
    
 
  
  // display all images till the user click on the image 0 
  std::cout << "Clic on the first view to start the acquisition, another click will stop it"<<std::endl;
  //getClick bloquant
  vpDisplay::getClick(I);
  
  vpTRACE("acquire many images");
  bool stopFlag = false;
  while (!stopFlag)
    {
      try{
	g.acquire(I);
	vpDisplay::display(I);
	vpDisplay::flush(I);
      }catch(...){
	
	vpTRACE ( "Cannot acquire the images... " ) ;
	exit(-1);
      }
      
      //getClick non bloquant : on arrete des que l'utilisateur clic
      stopFlag = vpDisplay::getClick(I,false);
      
    }

  stopFlag = false;
  
  // ------------------------------------------------------//
  // camera intrinsic parameters
  // ----------------------------------------------------//
  double px = 1000;
  double py = 1000;
  double u0 = 320;       
  double v0 = 240;
  vpCameraParameters cam(px,py,u0,v0) ;
  
  // ------------------------------------------------------//
  // target description
  // ----------------------------------------------------//
  
  int nbpts(6);
 
  // We need a structure that content both the 3D coordinates of the point
  // in the object frame and the 2D coordinates of the point expressed in meter
  // the vpPoint class is ok for that
  vector<vpPoint>   P_Target;
  P_Target.resize(nbpts);

  // we set the 3D points coordinates (in meter !) in the object/world frame
  P_Target[0].setWorldCoordinates( -0.1, -0.05 , 0 ) ; // (X,Y,Z)
  P_Target[1].setWorldCoordinates( 0.02 , -0.06 , 0 ) ;
  P_Target[2].setWorldCoordinates( 0.05 , 0.07 , 0 ) ;
  P_Target[3].setWorldCoordinates( -0.1 , 0.06 , 0 ) ;
  P_Target[4].setWorldCoordinates( 0.1 , -0.02 ,  0 ) ;
  P_Target[5].setWorldCoordinates( 0 , 0 ,  0 );
  
  // define the vpDot structure, here 5 dots will tracked
  vector<vpDot2*> points2D;
  points2D.resize(nbpts);
  for(int i=0;i<nbpts;++i)
    {
      points2D[i]=new vpDot2;
    }

  for (int i=0 ; i < nbpts ; i++)
    {
      // by using setGraphics, we request to see the all the pixel of the dot
      // in green on the screen.
      // It uses the overlay image plane.
      // The default of this setting is that it is time consumming
      points2D[i]->setGraphics(true);
    }


  vector<vpImagePoint*> vpIP;
  vpIP.resize(nbpts);
  for(int i=0;i<nbpts;++i)
    {
      vpIP[i]=new vpImagePoint;
    }


 try{
    for (int i=0 ; i < nbpts ; i++)
      {
	// tracking is initalized
	// if no other parameters are given to the iniTracking(..) method
	// a right mouse click on the dot is expected
	// dot location can also be specified explicitely in the initTracking
	// method  : d.initTracking(I,u,v)  where u is the column index and v is
	// the row index
	
	points2D[i]->initTracking( I ) ;
	cout<<*points2D[i]<<endl;
	// track the dot and returns its coordinates in the image
	// results are given in float since many many are usually considered
	//
	// an expcetion is thrown by the track method if
	//  - dot is lost
	//  - the number of pixel is too small
	//  - too many pixels are detected (this is usual when a "big" specularity
	//    occurs. The threshold can be modified using the
	//    setNbMaxPoint(int) method
	

	points2D[i]->track( I, *vpIP[i] );

	vpDisplay :: flush(I);
      }
    
  }
  catch(...)
    {
      vpERROR_TRACE("Error in tracking initialization ") ;
      throw ;
    }


 HRP2PointTrackingProcess * PointTracker;
 PointTracker=new HRP2PointTrackingProcess;

 PointTracker->SetCameraParameters(cam);
 PointTracker->SetInputVispImages(&I);

 PointTracker->Init(P_Target, vpIP , nbpts);
 
 // une fois le point clicke, on peut redemarrer l'acquisition en boucle
 std::cout << "Clic on the first view to launch the tracking \nanother click will stop it"<<std::endl;
 //getClick bloquant
 vpDisplay::getClick(I);
  
 

 vpHomogeneousMatrix cMo;

 cout << "Tx \t Ty \t Tz "<<endl;
  
 while (!stopFlag)
   {
     try
       {
	 g.acquire(I);
	 vpDisplay::display(I);
	 vpDisplay::flush(I);
	 
       	 PointTracker->pRealizeTheProcess();
	 
	 PointTracker->GetvpImagePoint(vpIP);
       
	 for (int i = 0 ; i < nbpts ; i++)
	   {
	     vpDisplay::displayCross(I,*vpIP[i] ,10,vpColor::red) ;
	   }
	
	 vpDisplay::flush(I);
	 
		
       }catch(...){
       
       vpTRACE ( "Cannot acquire the images... " ) ;
       exit(-1);
     }
      
      
     PointTracker->computePose();
     
     PointTracker->GetOutputcMo(cMo);

     // display the compute pose
     // vpdisplay::( I , cMo , cam , 0.05 , vpColor::red ) ;
     vpDisplay::flush(I) ;
      
      
      cout <<  cMo[0][3] << "\t "<< cMo[1][3] << "\t " << cMo[2][3]  <<endl;
      
      //getClick non bloquant
      stopFlag = vpDisplay::getClick(I,false);
      
   }

  
 // wait for the user action
 std::cout << "Clic on the first view to close the application"<<std::endl;
 vpDisplay::getClick(I);
  
		
 // end of main

 return(0);
}


#endif 
 
