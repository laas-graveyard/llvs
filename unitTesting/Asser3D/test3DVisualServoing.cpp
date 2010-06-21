/** @doc This file implements the access to IEEE 1394 cameras
    through the dc library.

   CVS Information:
   $Id$
   $Author$
   $Date$
   $Revision$
   $Source$
   $Log$

   Copyright (c) 2003-2006, 
   @author Claire Dune,
   2010
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the CNRS and AIST nor the names of its contributors 
   may be used to endorse or promote products derived from this software without specific prior written permission.
   
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
   AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
   OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//standard lib and system lib
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

using namespace std;

//LLVS lib
#include "ModelTracker/nmbtTrackingProcess.h"


#if LLVS_HAVE_VISP && LLVS_HAVE_NMBT

//tracker lib
#include <nmbt/nmbtTracking.h>

//visp
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpServo.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureTranslation.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpColVector.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpServoDisplay.h>

#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vp1394TwoGrabber.h> 

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "HPR2Tracker:" << x << endl
#define ODEBUG3_CONT(x) cerr << x 

#if 0
#define ODEBUG(x) cerr << "HPR2Tracker:" <<  x << endl
#define ODEBUG_CONT(x) cerr << "HPR2Tracker:" <<  x << endl
#else
#define ODEBUG(x) 
#define ODEBUG_CONT(x) 
#endif



// test the functions implemented in the class 
int main(void)
{
  cout <<" -------------------------- " << endl;
  cout << endl; 
  cout <<" Test AsserVisu class"    << endl;
  cout << endl; 
  cout <<" ---------------------------" << endl;


  // creation d une image en niveau de gris
  vpImage<unsigned char> I ;
  
  //creer un grabber pour communiquer avec la webcam
  vp1394TwoGrabber g  ;
  
  g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_320x240_YUV422 );

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
    
    // set the display properties
    char title[50];
    sprintf(title,"Image");
    d.init(I,100,100,title); 
    
    // affichage : dispolay + flush
    
    vpDisplay::display(I);
    vpDisplay::flush(I);
    
    
  }
  catch(...){
    vpTRACE ( "Cannot acquire an image... " ) ;
    exit(-1);
    
  }
  
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
 

  // use this pose to init the LLVS tracker
  HRP2nmbtTrackingProcess trackerServer;
  
  // get the camera parameters Client and Server must have
  // the same parameter
  vpCameraParameters cam;
  trackerServer.GetCameraParameters(cam);

  // string operator
   ostringstream tmp_stream;
   
  // path to model without extension 
  tmp_stream.str("");
  tmp_stream<<  "./data/model/WoodenBox/WoodenBox" ;
  string modelPath =  tmp_stream.str() ;
  
  // path to vrml with .wrl extension
  tmp_stream<<".wrl"; 
  string vrmlPath =  tmp_stream.str() ;
  tmp_stream.str("");
 
  // create a temporary tracker
  nmbtTracking trackerClient;
  trackerClient.loadModel( vrmlPath.c_str());
  trackerClient.setCameraParameters(cam);
   
 
  // init the tracking with clicks
  trackerClient.initClick(I, modelPath.c_str()) ; 
  
  // create an initial pose
  vpHomogeneousMatrix cMo;
  trackerClient.getPose(cMo);
  cout << "cMo \n"<< cMo<<endl ;
  //trackerClient.track(Isrc);  
  //trackerClient.getPose(cMo);
  cout << "cMo after track \n"<< cMo<<endl ;
  trackerClient.display(I, cMo, cam, vpColor::red,2); 
  

  // init
  vpHomogeneousMatrix cdMo(cMo), cdMc;
  vpServo task ;
  vpTRACE("define the task") ;
  vpTRACE("\t we want an eye-in-hand control law") ;
  vpTRACE("\t robot is controlled in the camera frame") ;
  task.setServo(vpServo::EYEINHAND_CAMERA) ;
	
  vpTRACE("Compute the interation matrix at the current position") ;
  task.setInteractionMatrixType(vpServo::CURRENT) ;

  task.setLambda(0.6) ;
	

  cdMc = cdMo*cMo.inverse() ;
  vpTRACE("Compute the displacement to realize") ;
  vpFeatureTranslation t(vpFeatureTranslation::cdMc) ;
  vpFeatureThetaU tu(vpFeatureThetaU::cdRc);
  t.buildFrom(cdMc) ;
  tu.buildFrom(cdMc) ;
	
  vpTRACE("Build the task ") ;
  task.addFeature(tu) ;
  task.addFeature(t) ;

  task.print() ;
  // set the tracker parameters
  trackerServer.SetcMo(cMo); // The Client should give this
  trackerServer.SetInputVispImages(&(I));


  vpColVector v(6) ;
  stopFlag=false;
  double error;
  int iter = 0;
  while (!stopFlag)
    {
     iter++;
      try{
	g.acquire(I);
	vpDisplay::display(I);
	vpDisplay::flush(I);

      }catch(...){
	
	vpTRACE ( "Cannot acquire the images... " ) ;
	exit(-1);
      }
   
     
      // initialise the process, init the moving edge
      // with cMo and I stored
      trackerServer.InitializeTheProcess();
      trackerServer.RealizeTheProcess();
      // get the tracking result
      trackerServer.GetcMo(cMo);

      // display it using the client tracker
      trackerClient.display(I, cMo, cam, vpColor::red,2); 

      //boucle
      cdMc = cdMo*cMo.inverse() ;
      t.buildFrom(cdMc) ;
      tu.buildFrom(cdMc) ;
      v = task.computeControlLaw() ;
      error =  task.error.sumSquare();

      cout << "-----------" << iter <<endl;
      cout << "Velocity : " << endl <<v.t()<<endl;
      cout << "Error :" << error << endl;
      cout << "cdMc : " << endl << cdMc <<endl;
        
      //getClick non bloquant
      stopFlag = vpDisplay::getClick(I,false);
      
    }


  cout<< "Click on the image to exit" << endl;
  vpDisplay::getClick(I);  
  cout << endl;  
  cout <<" -------------------------- " << endl; 
  cout << endl; 
  cout <<"    .....End of the test    "    << endl;
  cout << endl; 
  cout <<" ---------------------------" << endl;
  // everything went well
  return 0;
}

#else

int main(void)
{
  cout <<" Cannot Test NMBT tracking class: NMBT not found!" << endl;
  return 0;
}

#endif
