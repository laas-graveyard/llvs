#include <iostream>
#include <fstream>

#include <visp/vpImageMorphology.h>
#include <visp/vpImageIo.h>
#include "ColorDetectionOperators.h"

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "ColorDetectionOperator:" << x << endl
using namespace::std;
#if 0
#define ODEBUG(x) cerr << "ColorDetectionOperator:" <<  x << endl
#else
#define ODEBUG(x) 
#endif


ColorDetectionOperators::ColorDetectionOperators()
{
  m_ImgRes = 0;
}

ColorDetectionOperators::~ColorDetectionOperators()
{
  if (m_ImgRes==0)
    delete m_ImgRes;
  
  if (m_ImageIntermediate==0)
    delete m_ImageIntermediate;
}

int ColorDetectionOperators::InitializeIntermediateStructure(vpImage<vpRGBa> *in)
{
  m_ImageIntermediate = new vpImage<unsigned char>(in->getHeight(),in->getWidth());
  m_ImgRes = new float[in->getWidth()*in->getHeight()];
  return 0;
}


int ColorDetectionOperators::ComputeCoG(vpImage<unsigned char> *in,
					double &x, double &y)
{
  double CoG[2]={0.0,0.0};

  unsigned int lw,lh, nbpixels=0;
  lw = in->getWidth();
  lh = in->getHeight();
  for(unsigned int j=0;j<lh;j++)
    {

      unsigned char  * p = (*in)[j];
      for(unsigned int i=0;i<lw;i++)
	{
	  if (*p==255)
	    {
	      CoG[0]+=i;
	      CoG[1]+=j;
	      nbpixels++;
	    }
	  p++;
	}
    }

  if (nbpixels!=0)
    {
      CoG[0] /=(double)nbpixels;
      CoG[1] /=(double)nbpixels;
    }
  else 
    {
      CoG[0] =-1.0;
      CoG[1] =-1.0;
    }

  
  x= CoG[0];
  y= CoG[1];
  return 0;
}

float lmin(float a,float b, float c)
{
  if (a<b) { if (a<c) { return a;} else {return c;} }	
  else { if (b<c) { return b; } else {return c; } } 
}

float lmax(float a,float b, float c)
{
  if (a>b) { if (a>c) { return a;} else {return c;} }	
  else { if (b>c) { return b; } else {return c; } } 
}

void RGBtoHSV( float r, float g, float b, float *h, float *s, float *v )
{
  float min, max, delta;
  min = lmin( r, g, b );
  max = lmax( r, g, b );
  *v = max;				// v
  delta = max - min;
  if( max != 0 )
    *s = delta / max;		// s
  else {
    // r = g = b = 0		// s = 0, v is undefined
    *s = 0;
    *h = -1;
    return;
  }
  if( r == max )
    *h = ( g - b ) / delta;		// between yellow & magenta
  else if( g == max )
    *h = 2 + ( b - r ) / delta;	// between cyan & yellow
  else
    *h = 4 + ( r - g ) / delta;	// between magenta & cyan
  *h *= 60;				// degrees
  if( *h < 0 )
    *h += 360;
}


int ColorDetectionOperators::ReadFromFileAndCreateThePixelList(char *FileName)
{
  vpImage<vpRGBa> anImageRGB;
  std::string aFileName = FileName;
  
  vpImageIo::readPPM(anImageRGB,FileName);

  unsigned int lw=anImageRGB.getWidth(),
    lh = anImageRGB.getHeight();
  for(int i=0;i<3;i++)
    for(int j=0;j<256;j++)
      {
	m_Histogram[i][j]=0.0;
	m_HistogramMono[j]=0.0;
      }


  int nbpixels=0;
  for(unsigned int j=0;j<lh;j++)
    {
      vpRGBa *apRGB;
      float r,g,b,h,s,v;
      unsigned char uc_h,uc_v,uc_s;

      apRGB = anImageRGB[j];


      for(unsigned int i=0;i<lw;i++)
	{
	 
	  r = apRGB->R; g= apRGB->G; b = apRGB->B;
	  RGBtoHSV(r,g,b,&h,&s,&v);
	  uc_h = (unsigned char)(h*255.0/360.0); 
	  uc_s = (unsigned char)(s*255.0); 
	  uc_v= (unsigned char)(v*255.0);
							
	  //	  if (apyuv->y<250)
	  if ( (apRGB->R!=255) ||  (apRGB->G!=255) || (apRGB->B!=255))
	    {
	      
	      //	      cout << (int)apyuv->y << " " << (int)apyuv->G << " " << (int)apyuv->B << endl;
	      m_Histogram[0][uc_h]++;
	      m_Histogram[1][uc_s]++;
	      m_Histogram[2][uc_v]++;
	      nbpixels++;
	    }
	  apRGB++;
	}
    }
  //  cout << "Number of pixels " << nbpixels << endl;

  for(int i=0;i<256;i++)
    {
      m_HistogramMono[i] /= nbpixels;
      for(int j=0;j<3;j++)
	m_Histogram[j][i] /= nbpixels;
    }

  std::ofstream aof;
  aof.open("output.dat",std::ofstream::out);
  if (aof.is_open())
    {
      for(int i=0;i<256;i++)
	aof << m_Histogram[0][i] << " "
	    << m_Histogram[1][i] << " "
	    << m_Histogram[2][i] << std::endl;
      
      aof.close();

    }

  return 0;
}



int ColorDetectionOperators::FilterOnHistogram(unsigned char * in,
					       vpImage<unsigned char> * out)

{
  
  vpImage<unsigned char> *dest_p = out;
  vpImage<vpRGBa> *test = new vpImage<vpRGBa>(out->getHeight(),
					      out->getWidth());

  //std::ofstream aof;
  //aof.open("output3d.dat",std::ofstream::out);
  
  double max=-1.0;
  double sum_proba=0.0;

  unsigned int lh=out->getHeight(),
    lw=out->getWidth();

  float *fp = m_ImgRes;
  unsigned char * p= in;
  unsigned char * p1=p+1;
  unsigned char * p2=p1+1;
  unsigned char uc_h,uc_s,uc_v;
  float r,g,b,h,s,v;

  ODEBUG("Starting point on FilterOnHistogram");
  for(unsigned int j=0;j<lh;j++)
    {
      vpRGBa *pRGBa = (*test)[j];
      for(unsigned int i=0;i<lw;i++)
	{

	  r = *p; g= *p1; b = *p2;
	  pRGBa->R = *p;
	  pRGBa->G = *p1;
	  pRGBa->B = *p2;
	  RGBtoHSV(r,g,b,&h,&s,&v);
	  uc_h = (unsigned char)(h*255.0/360.0); 
	  uc_s = (unsigned char)(s*255.0); 
	  uc_v= (unsigned char)(v*255.0);
	  register float ftmp;

	  *fp = ftmp = m_Histogram[0][uc_h] *
	    m_Histogram[1][uc_s] *
	    m_Histogram[2][uc_v];
	  
	  if (max < ftmp)
	    max = ftmp;

	  sum_proba += ftmp;
	  fp++;
	  p+=3;p1+=3;p2+=3;
	  pRGBa++;
	}
    }

  ODEBUG("sum_proba: " << sum_proba << " max: " << max );
  fp = m_ImgRes;
  for(unsigned int j=0;j<lh;j++)
    {
      unsigned char * ldest_p = (*dest_p)[j];
      for(unsigned int i=0;i<lw;i++)
	{

	  *fp *= 1.0 / sum_proba;
	  //cout << i << " " << j << " " << *fp << endl;
	  //if ((*fp>1e-5) && (max>
	  if (*fp>1e-5) 
	    {
	      *ldest_p++ = 255;
	    }
	  else
	    *ldest_p++=0;
	    //dest_p[j*lw+i] = (unsigned char) (255.0 * ImgRes[j*lw+i] /max) ;
	  fp++;
	}
    }


  
  //  aof.close();
  
  // Apply a morphological operator on the result.
  
  //  m_ImageIntermediate->WriteImage("ImageIntermediate0.pgm");

  //aMO.Opening(*m_ImageIntermediate,*out,2);
  unsigned char ucdown=0, ucup = 255;
  vpImageMorphology::erosion<unsigned char>(*out, ucdown , ucup, 
					    vpImageMorphology::CONNEXITY_4);
  vpImageMorphology::dilatation<unsigned char>(*out, 0 , 255, 
					     vpImageMorphology::CONNEXITY_4);
  
  vpImageIo::writePGM(*out,"ImageIntermediate0.pgm");
  vpImageIo::writePPM(*test,"ImageIntermediate0.ppm");
  return 0;
}
