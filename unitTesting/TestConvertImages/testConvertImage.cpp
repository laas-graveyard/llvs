/** @doc This file implements the access to IEEE 1394 cameras
    through the dc library.

   CVS Information:
   $Id$
   $Author$
   $Date$
   $Revision$
   $Source$
   $Log$

   Copyright (c) 23-26, 
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

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

//visp
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageConvert.h>
// OpenCV
#include "cv.h"


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

void
convertGrey(const cv::Mat* src,
      vpImage<unsigned char> &dest, bool flip)
{
  int nChannel = src->channels();
  int depth = src->depth();
  int height = src->rows;
  int width = src->cols;
  int widthStep = src->step;
  int lineStep = (flip) ? 1 : 0;


cout << width << "Heigth" <<height<<endl;
 cout<< "nChannel :" << nChannel<<" depth :" << depth<<endl;
 
  if (flip == false)
  {
  	if(widthStep == width){
  	  if(nChannel == 1 && depth == 0){
  	    dest.resize(height,width) ;
  	    memcpy(dest.bitmap, src->data,
  	            height*width);
  	  }
  	  if(nChannel == 3 && depth == 0){
  	    dest.resize(height,width) ;
  	    vpImageConvert::BGRToGrey((unsigned char*)src->data,dest.bitmap,width,height,false);
  	  }
  	}
  	else{
  	  if(nChannel == 1 && depth == 0){
  	    dest.resize(height,width) ;
  	    for (int i =0  ; i < height ; i++){
  	      memcpy(dest.bitmap+i*width, src->data + i*widthStep,
  	            width);
  	    }
  	  }
  	  if(nChannel == 3 && depth == 0){
  	    dest.resize(height,width) ;
  	    for (int i = 0  ; i < height ; i++){
  	      vpImageConvert::BGRToGrey((unsigned char*)src->data + i*widthStep,
  	                  dest.bitmap + i*width,width,1,false);
  	    }
  	  }
  	}
  }
  else 
  {
  	  if(nChannel == 1 && depth == 0){
	    unsigned char* beginOutput = (unsigned char*)dest.bitmap;
  	    dest.resize(height,width) ;
  	    for (int i =0  ; i < height ; i++){
  	      memcpy(beginOutput + lineStep * ( 4 * width * ( height - 1 - i ) ) , src->data + i*widthStep,
  	            width);
  	    }
  	  }
  	  if(nChannel == 3 && depth == 0){
  	    dest.resize(height,width) ;
  	    //for (int i = 0  ; i < height ; i++){
  	      vpImageConvert::BGRToGrey((unsigned char*)src->data /*+ i*widthStep*/,
  	                  dest.bitmap /*+ i*width*/,width,height/*1*/,true);
  	    //}
  	  }
  }
}

void convertRGBa(const cv::Mat* src, vpImage<vpRGBa> & dest, bool flip)
{
  int nChannel = src->channels();
  int depth = src->depth();
  int height = src->rows;
  int width = src->cols;
  int widthStep = src->step;
  int lineStep = (flip) ? 1 : 0;


  if(nChannel == 3 && depth == 0){
    dest.resize(height,width);

    //starting source address
    unsigned char* input = (unsigned char*)src->data;
    unsigned char* line;
    unsigned char* beginOutput = (unsigned char*)dest.bitmap;
    unsigned char* output = NULL;

    for(int i=0 ; i < height ; i++)
    {
      line = input;
      output = beginOutput + lineStep * ( 4 * width * ( height - 1 - i ) ) + (1-lineStep) * 4 * width * i;
      for(int j=0 ; j < width ; j++)
        {
          *(output++) = *(line+2);
          *(output++) = *(line+1);
          *(output++) = *(line);
          *(output++) = 0;

          line+=3;
        }
      //go to the next line
      input+=widthStep;
    }
  }
  else if(nChannel == 1 && depth == 0 ){
    dest.resize(height,width);
    //starting source address
    unsigned char * input = (unsigned char*)src->data;
    unsigned char * line;
    unsigned char* beginOutput = (unsigned char*)dest.bitmap;
    unsigned char* output = NULL;

    for(int i=0 ; i < height ; i++)
    {
      line = input;
      output = beginOutput + lineStep * ( 4 * width * ( height - 1 - i ) ) + (1-lineStep) * 4 * width * i;
      for(int j=0 ; j < width ; j++)
        {
          *output++ = *(line);
          *output++ = *(line);
          *output++ = *(line);
          *output++ = *(line);;

          line++;
        }
      //go to the next line
      input+=widthStep;
    }
  }
}


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

  convertGrey(Imatp ,Ivispg , false);

  // create the display associated to the image
  vpDisplayX display(Ivispg,0,0,"ImageGrey");
  vpDisplay::display(Ivispg);
  vpDisplay::flush(Ivispg);

  std::cout << "USER : wait for the user to click on the view to continue" <<endl;
  vpDisplay::getClick(Ivispg);


  // create the ViSP grey image 
  vpImage<vpRGBa> IvispRgb;

  convertRGBa(Imatp ,IvispRgb , false);

  // create the display associated to the image
  vpDisplayX displayRGB(IvispRgb,600,0,"ImageRGBa");
  vpDisplay::display(IvispRgb);
  vpDisplay::flush(IvispRgb);

  std::cout << "USER : wait for the user to click on the view to continue" <<endl;
  vpDisplay::getClick(IvispRgb);
  


//  std::cout << "USER : wait for the user to click on the view to continue" <<endl;
//  vpDisplay::getClick(Ivisp);


  return 0;
}
