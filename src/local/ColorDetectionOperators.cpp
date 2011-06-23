#include <iostream>
#include <fstream>

#include "VW/Image/imageio_ppm.h"
#include "MorphologicalOperators.h"
#include "ColorDetectionOperators.h"
using namespace VW;

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

int ColorDetectionOperators::InitializeIntermediateStructure(unsigned int lwidth,
							     unsigned int lheight)
{
  m_ImageIntermediate = new unsigned char[lwidth*lheight];
  m_ImgRes = new float[lwidth()*lheight];
  return 0;
}

int ColorDetectionOperators::InitializeIntermediateStructure(unsigned int lwidth,
							     unsigned int lheight)
{
  m_ImageIntermediate = new unsigned char[lwidth*lheight];
  m_ImgRes = new float[lwidth*lheight];
  return 0;
}

int ColorDetectionOperators::ComputeCoG(unsigned char * in,
					unsigned int lwidth,
					unsigned int lheight,
					double &x, double &y)
{
  double CoG[2]={0.0,0.0};

  unsigned int lw,lh, nbpixels=0;
  lw = lwidth;
  lh = lheight;
  unsigned char  * p = in;
  for(unsigned int j=0;j<lh;j++)
    {

	
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

int ColorDetectionOperators::ReadPPM(std::string &aFileName,
				     unsigned char * & ap,
				     unsigned int & lwidth,
				     unsigned int & lheight)
{

  ifstream aif;
  aif.open(aFileName,ifstream::in);
  if(!aif.is_open())
    {
      aif.close();
      return -1;
    }

  string type;
  aif >> type;
  if (type!="P6")
    {
      aif.close();
      return -1;
    }

  unsigned int ldatasize;

  aif >> lwidth;
  aif >> lheight;
  aif >> ldatasize;

  if ((lwidth<=0) || (lwidth>1000000))
    {
      aif.close();
      return -1;
    }

  if ((lheight<=0) || (lheight>1000000))
    {
      aif.close();
      return -1;
    }

  if (ldatasize!=255)
    {
      aif.close();
      return -1;
    }
  
  ap = new unsigned char[lwidth*lheight*3];

  
  aif.read(ap,lwidth*lheight*3);
    
  
  aif.close();
}


int ColorDetectionOperators::ReadFromFileAndCreateThePixelList(unsigned char *FileName)
{
  unsigned char *anImageRGB;
  std::string aFileName = FileName;
  unsigned int lw,lh;
  
  
  ReadPPM(FileName,anImageMono,lw,lh);

  for(int i=0;i<3;i++)
    for(int j=0;j<256;j++)
      {
	m_Histogram[i][j]=0.0;
	m_HistogramMono[j]=0.0;
      }


  int nbpixels=0;
  for(unsigned int j=0;j<lh;j++)
    {
      unsigned char* apyuv = anImageRGB[j*3*lwidth];
      for(unsigned int i=0;i<lw;i++)
	{
#if 0
	  if (j==0)
	    cout << (int)apyuv->y << " " << (int)apyuv->u << " " << (int)apyuv->v << endl;
#endif
	  
	  //	  if (apyuv->y<250)
	  if ( (apyuv->r!=255) ||  (apyuv->g!=255) ||  (apyuv->b!=255))
	    {
	      
	      //cout << (int)apyuv->y << " " << (int)apyuv->u << " " << (int)apyuv->v << endl;
	      m_Histogram[0][apyuv[0]]++;
	      m_Histogram[1][apyuv[1]]++;
	      m_Histogram[2][apyuv[2]]++;
	      nbpixels++;
	    }
	  apyuv+=3;
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
					       unsigned char * out)
 
{
  
   return FilterOnHistogram(in, out);
  
}

int ColorDetectionOperators::FilterOnHistogram(unsigned char * in,
					       unsigned char * out,
					       unsigned int lwidth,
					       unsigned int lheight)

{
  
  unsigned char *dest_p = m_ImageIntermediate;

  //std::ofstream aof;
  //aof.open("output3d.dat",std::ofstream::out);
  
  double max=-1.0;
  double sum_proba=0.0;

  unsigned int lh=lheight,lw=lwidth;

  unsigned char *p  = in ;
  unsigned char *p1=0,*p2=0;

  p1 = p+1;
  p2 = p1+1;

  
  float *fp = m_ImgRes;

  for(unsigned int j=0;j<lh;j++)
    {

      for(unsigned int i=0;i<lw;i++)
	{

	  register float ftmp;

	  *fp = ftmp = m_Histogram[0][*p] *
	    m_Histogram[1][*p1] *
	    m_Histogram[2][*p2];
	  
	  if (max < ftmp)
	    max = ftmp;

	  sum_proba += ftmp;
	  fp++;
	  p+=3;
	  p1+=3;
	  p2+=3;
	}
    }

  ODEBUG("sum_proba: " << sum_proba << " max: " << max );
  fp = m_ImgRes;
  for(unsigned int j=0;j<lh;j++)
    {
      for(unsigned int i=0;i<lw;i++)
	{

	  *fp *= 1.0 / sum_proba;
	  //	  aof << i << " " << j << " " <<ImgRes[j*lw+i] << endl;
	  //	  if ((*fp>1e-5) && (max>
	  if ((*fp>1e-5) 
	    {
	      *dest_p++ = 255;
	    }
	  else
	    *dest_p++=0;
	    //dest_p[j*lw+i] = (unsigned char) (255.0 * ImgRes[j*lw+i] /max) ;
	  fp++;
	}
    }


  
  //  aof.close();
  
  // Apply a morphological operator on the result.
  VW::MorphologicalOperators aMO;
  
  //  m_ImageIntermediate->WriteImage("ImageIntermediate0.pgm");
  ODEBUG("before closing");
  aMO.Opening(*m_ImageIntermediate,*out,2);
  ODEBUG("after closing");
  return 0;
}
