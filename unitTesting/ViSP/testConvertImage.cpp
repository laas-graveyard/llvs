/** @doc This file implements test for vispConvertImage.cpp


   Copyright (c) 2010, 
   @author Stephane EMBARKI,
   
   JRL-Japan, CNRS/AIST

   See license file for information on license.
*/



#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#if (LLVS_HAVE_VISP>0 && LLVS_HAVE_OPENCV >0)
//visp
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>


// OpenCV
#include "cv.h"

// LLVS
#include "ViSP/vispConvertImageProcess.h"

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

    // Create the path
  char* homePath;
  homePath                    = getenv ("HOME");
  string defaultPath          = "data";
  string imagePath            = "images/imageWide1.ppm"  ;

  // path to image
  tmp_stream<< homePath << "/"<<defaultPath<<"/"<< imagePath;
  imagePath = tmp_stream.str() ;

  // Read an image on a disk with openCV library
  cv::Mat Imat;
  Imat = cv::imread(imagePath.c_str());


  cv::Mat* Imatp = &Imat;

  // Save an image on a disk with openCV library
  ostringstream tmp_stream2;
  imagePath            = "images/imageWide1CV.ppm"  ;
  tmp_stream2<< homePath << "/"<<defaultPath<<"/"<< imagePath;
  imagePath = tmp_stream2.str() ;

  cv::imwrite(imagePath.c_str(),Imat );

  // create the ViSP grey image 
  vpImage<unsigned char> Ivispg;
  vpImage<unsigned char>* Ivispgp = &Ivispg;



  HRP2vispConvertImageProcess convIprocU8(HRP2vispConvertImageProcess::MAT_VISPU8 );

  convIprocU8.InitializeTheProcess();

  convIprocU8.SetImages (Ivispgp,Imatp);

  convIprocU8.RealizeTheProcess();


  // create the display associated to the image
  vpDisplayX displayg(Ivispg,0,0,"ImageGrey");
  vpDisplay::display(Ivispg);
  vpDisplay::flush(Ivispg);

  std::cout << "USER : wait for the user to click on the view to continue" <<endl;
  vpDisplay::getClick(Ivispg);


 // create the ViSP grey image 
  vpImage<vpRGBa> IvispRGB;
  vpImage<vpRGBa>* IvispRGBp = &IvispRGB;

  HRP2vispConvertImageProcess convIprocRGB (HRP2vispConvertImageProcess::MAT_VISPRGB );

  convIprocRGB.InitializeTheProcess();

  convIprocRGB.SetImages (IvispRGBp,Imatp);

  convIprocRGB.RealizeTheProcess();


 // create the display associated to the image
  vpDisplayX displayRGB(IvispRGB,600,0,"ImageRGB");
  vpDisplay::display(IvispRGB);
  vpDisplay::flush(IvispRGB);

  std::cout << "USER : wait for the user to click on the view to continue" <<endl;
  vpDisplay::getClick(IvispRGB);



  cv::Mat ImatresU8;

  cv::Mat* ImatresU8p = &ImatresU8;

 HRP2vispConvertImageProcess convIprocU8Mat (HRP2vispConvertImageProcess::VISPU8_MAT );

  convIprocU8Mat.InitializeTheProcess();

  convIprocU8Mat.SetImages (Ivispgp,ImatresU8p);

  convIprocU8Mat.RealizeTheProcess();

  cv::namedWindow("MatGrey"  ,CV_WINDOW_AUTOSIZE );

  cv::imshow( "MatGrey", ImatresU8 );



  cvWaitKey(0);


  cv::Mat ImatresRGB;

  cv::Mat* ImatresRGBp = &ImatresRGB;

  HRP2vispConvertImageProcess convIprocRGBMat (HRP2vispConvertImageProcess::VISPRGB_MAT );

  convIprocRGBMat.InitializeTheProcess();

  convIprocRGBMat.SetImages (IvispRGBp,ImatresRGBp);

  convIprocRGBMat.RealizeTheProcess();


  cv::namedWindow("MatColor"  ,CV_WINDOW_AUTOSIZE );

  cv::imshow( "MatColor", ImatresRGB );

  std::cout << "USER : wait for the user to click press a key to continue" <<endl;

  cvWaitKey(0);


  return 0;
}


#else

int main(void)
{
  std::cout <<std::endl<<"Need ViSP and OpenCv to execute this example"<<std::endl;

  return 0;

}
#endif
