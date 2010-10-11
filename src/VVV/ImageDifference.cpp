#include <iostream>
#include <fstream>

#include "ImageDifference.h"

// Debug macros
#include <llvs/tools/Debug.h>

HRP2ImageDifferenceProcess::HRP2ImageDifferenceProcess()
  : HRP2VisionBasicProcess()
{
  m_IndexOfImage =0;
  m_Computing=0;

  m_ProcessName = "Image Difference Process";
  for(int i=0;i<3;i++)
    m_ImagesOnWhichToProcess[i]=0;

  m_ImagesOnWhichToProcess[0]=1; // On the left.
  m_ImagesOnWhichToProcess[1]=1; // and on the right.
  m_BufferOfImages = 0;
  m_BufferImage = false;

  string aParameter("BufferImage");
  string aValue("false");
  SetParameter(aParameter,aValue);

  aParameter="SaveImage";
  aValue="0";
  SetParameter(aParameter,aValue);
  m_NbOfSecondsToGrab=1;
}

HRP2ImageDifferenceProcess::~HRP2ImageDifferenceProcess()
{
}

void HRP2ImageDifferenceProcess::SaveImages()
{
  char Buffer[1024];
  ofstream of;

 sprintf(Buffer,"TimeStamp.dat");
 of.open(Buffer,ofstream::out);
 if (of.is_open())
   {

     for(int i=0;i<33*3*m_NbOfSecondsToGrab;i++)
       {
	 sprintf(Buffer,"%20.10f",
		 m_BufferOfTimeStamp[i]);
	 of << Buffer << endl;

       }
     of.close();
   }
  unsigned char * src= (unsigned char *)m_BufferOfImages;
  int size=m_InputImage[0].Width*m_InputImage[0].Height;
  for(int i=0;i<33*3*m_NbOfSecondsToGrab;i++)
    {

      sprintf(Buffer,"ImageDiff_%06d.pgm",i);
      of.open(Buffer,ofstream::out);
      if (of.is_open())
	{
	  sprintf(Buffer,"P5\n# TimeStamp: %20.10f\n%d %d\n255\n",
		  m_BufferOfTimeStamp[i],
		  m_InputImage[0].Width,
		  m_InputImage[0].Height);
	  of << Buffer;
	  of.write((char *)src,size);
	  src +=size;
	  of.close();
	}
    }


}

void HRP2ImageDifferenceProcess::SetImageOnWhichToProcess(int ImagesOnWhichToProcess[3])
{
  for(int i=0;i<3;i++)
    m_ImagesOnWhichToProcess[i] = ImagesOnWhichToProcess[i];

}

int HRP2ImageDifferenceProcess::InitializeTheProcess()
{
  for(int i=0;i<3;i++)
    {
      m_PrevInputImage[i] = epbm_create(EPBM_BINARY_GRAY,
					m_InputImage[i].Width,
					m_InputImage[i].Height,
					m_InputImage[i].Sign,
					EPBM_INT32,
					0);

      m_DifferenceOfImages[i] = epbm_create(EPBM_BINARY_GRAY,
					    m_InputImage[i].Width,
					    m_InputImage[i].Height,
					    m_InputImage[i].Sign,
					    EPBM_INT32,
					    0);
    }
  return 0;
}


int HRP2ImageDifferenceProcess::RealizeTheProcess()
{
  ODEBUG("Flag 1");
  if (!m_Computing)
    return 0;

  ODEBUG("Flag 2");
  for(int i=0;i<3;i++)
    {
      if (m_ImagesOnWhichToProcess[i])
	{
	  unsigned char *src1 = (unsigned char *)m_InputImage[i].Image;
	  unsigned char *src2 = (unsigned char *)m_PrevInputImage[i].Image;

	  if (!m_BufferImage)
	    {
	      unsigned char *dst = (unsigned char *)m_DifferenceOfImages[i].Image;
	      for( int k=0;k<m_InputImage[i].Height *
		    m_InputImage[i].Width;k++)
		{
		  int aChar= *src1 - *src2;
		  if (aChar<0)
		    *dst++ = (unsigned char)-aChar;
		  else
		    *dst++ = (unsigned char)aChar;
		  *src2++ = *src1++;
		}
	    }
	  else
	    {

	      unsigned char *dst = (unsigned char *)(m_BufferOfImages+(m_IndexOfImage*m_InputImage[0].Width*m_InputImage[0].Height));
	      for(int k=0;k<m_InputImage[i].Height *
		    m_InputImage[i].Width;k++)
		{
		  int aChar= *src1/* - *src2*/;
		  if (aChar<0)
		    *dst++ = (unsigned char)-aChar;
		  else
		    *dst++ = (unsigned char)aChar;

		  *src2++ = *src1++;
		}
	      m_BufferOfTimeStamp[m_IndexOfImage]=
		m_timestamps[i].tv_sec + 0.000001 * m_timestamps[i].tv_usec;

	      m_IndexOfImage++;
	      if (m_IndexOfImage>=33*3*m_NbOfSecondsToGrab)
		m_IndexOfImage=0;
	    }
	}
    }
  ODEBUG("Flag 3");
  return 0;
}

int HRP2ImageDifferenceProcess::CleanUpTheProcess()
{
  for(int i=0;i<3;i++)
    {
      epbm_free(&m_PrevInputImage[i]);
      epbm_free(&m_DifferenceOfImages[i]);
     }
  return 0;
}


int HRP2ImageDifferenceProcess::SetInputImages(EPBM lInputImages[3])
{
  for(int i=0;i<3;i++)
    m_InputImage[i] = lInputImages[i];

  m_BufferOfImages = new unsigned char[33*m_NbOfSecondsToGrab*3*m_InputImage[0].Width*m_InputImage[0].Height];
  m_BufferOfTimeStamp = new double[33*m_NbOfSecondsToGrab*3];
  if (m_BufferOfImages==0)
    ODEBUG3("Big pb ahead...");
  else
    ODEBUG3("Able to allocate memory.");
  return 0;
}

int HRP2ImageDifferenceProcess::SetTimeStamp(struct timeval * ltimestamps)
{
  m_timestamps = ltimestamps;
  return 0;
}


int HRP2ImageDifferenceProcess::SetParameter(string aParameter, string aValue)
{
  int r=-1;
  unsigned char ok = 1;

  if (aParameter=="BufferImage")
    {
      if (aValue=="false")
	m_BufferImage = false;
      else if (aValue=="true")
	m_BufferImage = true;
      else
	ok = 0;
    }

  if (aParameter=="SaveImage")
    {
      if (aValue=="1")
	{
	  SaveImages();
	}
    }

  if (ok)
    r = HRP2VisionBasicProcess::SetParameter(aParameter, aValue);


  return r;

}


