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

//LLVS lib
#include "ModelTracker/nmbtTrackingProcess.h"

//tracker lib
#include<nmbt/nmbtTracking.h>

//visp
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>

using namespace std;



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
  cout <<" Test NMBT tracking class"    << endl;
  cout << endl; 
  cout <<" ---------------------------" << endl;
 

  // use this pose to init the LLVS tracker
  HRP2nmbtTrackingProcess trackerServer;
  
  // get the camera parameters Client and Server must have
  // the same parameter
  vpCameraParameters cam;
  trackerServer.GetCameraParameters(cam);

  // string operator
   ostringstream tmp_stream;
  
  // Create the path
  char* homePath;
  homePath                    = getenv ("HOME");
  string defaultPath          = "data";
  string imagePath            = "images/imageWide.ppm"  ;
  string objectName           = "WoodenBox";
  
  // path to image
  tmp_stream<< homePath << "/"<<defaultPath<<"/"<< imagePath;
  imagePath = tmp_stream.str() ;
  
  // path to model without extension 
  tmp_stream.str("");
  tmp_stream<< homePath << "/"<<defaultPath<<"/model/"<<objectName<<"/"<<objectName ;
  string modelPath =  tmp_stream.str() ;
  
  // path to vrml with .wrl extension
  tmp_stream<<".wrl"; 
  string vrmlPath =  tmp_stream.str() ;
  tmp_stream.str("");
 
  // read the image 
  vpImage<unsigned char> Isrc;
  vpImageIo::readPPM(Isrc,imagePath.c_str());   
    
  // create the display associated to the image
  vpDisplayX display(Isrc,0,0,"Image");
  vpDisplay::display(Isrc);
  vpDisplay::flush(Isrc);

  // create a temporary tracker
  nmbtTracking trackerClient;
  trackerClient.loadModel( vrmlPath.c_str());
  trackerClient.setCameraParameters(cam);
   
 
  // init the tracking with clicks
  trackerClient.initClick(Isrc, modelPath.c_str()) ; 
  
  // create an initial pose
  vpHomogeneousMatrix cMo;
  trackerClient.getPose(cMo);
  cout << "cMo \n"<< cMo<<endl ;
  //trackerClient.track(Isrc);  
  //trackerClient.getPose(cMo);
  cout << "cMo after track \n"<< cMo<<endl ;
  trackerClient.display(Isrc, cMo, cam, vpColor::red,2); 
   
   
  // set the tracker parameters
  trackerServer.SetcMo(cMo); // The Client should give this
  trackerServer.SetInputVispImages(&(Isrc)); // LLVS should give this

  // initialise the process, init the moving edge
  // with cMo and I stored
  trackerServer.InitializeTheProcess();
  trackerServer.RealizeTheProcess();

  // get the tracking result
  trackerServer.GetcMo(cMo);
  
  // display it using the client tracker
  trackerClient.display(Isrc, cMo, cam, vpColor::red,2); 
  
  


  cout<< "Click on the image to exit" << endl;
  vpDisplay::getClick(Isrc);
  cout << endl;  
  cout <<" -------------------------- " << endl; 
  cout << endl; 
  cout <<"    .....End of the test    "    << endl;
  cout << endl; 
  cout <<" ---------------------------" << endl;
  // everything went well
  return 0;
}


