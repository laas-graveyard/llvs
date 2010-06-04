/** @doc This file implements test for vispConvertImage.cpp


   Copyright (c) 2010, 
   @author Stephane EMBARKI,
   
   JRL-Japan, CNRS/AIST

   See license file for information on license.
*/



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


#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

#include "llvsConfig.h"

#if (LLVS_HAVE_VISP>0)
//visp
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageConvert.h>
#include <visp/vpXmlParserCamera.h>

// LLVS
#include "ViSP/vispUndistordedProcess.h"

using namespace std;



// test the parameters functions
int main(void)
{



  cout <<" -------------------------- " << endl;
  cout << endl; 
  cout <<"    Test image Convertion "    << endl;
  cout <<"      cv::Mat to vpImage"    << endl;
  cout << endl; 
  cout <<" ---------------------------" << endl;

  // string operator
   ostringstream tmp_stream;

 ostringstream tmp_stream2;
  // Create the path
  char* homePath;
  homePath                    = getenv ("HOME");
  string defaultPath          = "data";
  string imagePath            = "images/imageWide1.ppm"  ;

  string cameraPath           ="/data/hrp2CamParam/hrp2.xml";


  // path to image
  tmp_stream<< homePath << "/"<<defaultPath<<"/"<< imagePath;
  imagePath = tmp_stream.str() ;

  vpCameraParameters cam;


    tmp_stream2<<homePath <<cameraPath;

    cameraPath=tmp_stream2.str();


  vpXmlParserCamera parser;
  parser.parse(cam,
	       cameraPath.c_str(),
	       "cam1394_3",
	       vpCameraParameters::perspectiveProjWithDistortion,
	       320, 
	       240);


 vpImage<vpRGBa> IvispRGB;
  vpImageIo::readPPM(IvispRGB,imagePath.c_str());

 
 // create the display associated to the image
  vpDisplayX displayrgbg(IvispRGB,0,0,"Image grey undistorded");
  vpDisplay::display(IvispRGB);
  vpDisplay::flush(IvispRGB);

  std::cout << "USER : wait for the user to click on the view to continue" <<endl;
  vpDisplay::getClick(IvispRGB);



   unsigned char Iraw[3*320*240] ;

  vpImageConvert::RGBaToRGB(&IvispRGB.bitmap[0].R,Iraw,320*240);

  
  vpImage<unsigned char>* IvispGrey;
  
  IvispGrey=new  vpImage<unsigned char>;

  IvispGrey->resize(240,320);
  /*
  HRP2vispUndistordedProcess undistort(HRP2vispUndistordedProcess::RGB_VISPU8 );


  undistort.InitializeTheProcess();

  undistort.SetImages(Iraw ,IvispGrey);

  undistort.SetCameraParameters(cam);

  undistort.RealizeTheProcess();
  
  // create the display associated to the image
  vpDisplayX displayg(*IvispGrey,0,600,"Image grey undistorded");
  vpDisplay::display(*IvispGrey);
  vpDisplay::flush(*IvispGrey);*/

  std::cout << "USER : wait for the user to click on the view to continue" <<endl;
  vpDisplay::getClick(*IvispGrey);

  return 0;

}
#else

int main(void)
{
  std::cout <<std::endl<<"Need ViSP to execute this example"<<std::endl;

  return 0;
}

#endif
