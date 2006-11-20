/** @doc This object implements the link between 
    a collection of files with the
    overall vision system.

   CVS Information:
   $Id$
   $Author$
   $Date$
   $Revision$
   $Source$
   $Log$

   Copyright (c) 2003-2006, 
   @author Olivier Stasse
   
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
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <sys/time.h>

#include "FileImagesInputMethod.h"

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "HPR2FileImagesInputMethod:" << x << endl

#if 0
#define ODEBUG(x) cerr << "HPR2FileImagesInputMethod:" <<  x << endl
#else
#define ODEBUG(x) 
#endif

using namespace std;

HRP2FileImagesInputMethod::HRP2FileImagesInputMethod(int MethodForInputImages) : HRP2ImagesInputMethod()
{
  m_MethodHowReading = MethodForInputImages;
  m_NbOfImages = 0;
  m_ReadImageData.clear();
  m_InitValue = 0;
}

HRP2FileImagesInputMethod::~HRP2FileImagesInputMethod()
{

}

int HRP2FileImagesInputMethod::SetBaseName(string aFileName)
{
  m_BaseName = aFileName;
  return 0;
}

string HRP2FileImagesInputMethod::GetBaseName()
{
  return m_BaseName;
}

int HRP2FileImagesInputMethod::ReadEPBMImageHeader(ifstream & aifstream, int CameraNumber)
{
  string ImageFormat;
  string PixelLength;
  string DataType;
  string Sign;
  string Endian;
  string PinHoleParameter[12],PinHoleParameterF,PinHoleParameterM;
  string ConvertFlag; string Author;
  string Date;
  string DistorsionParameter[4];
  string WidthAndHeight, NbOfValues;
  int r = 0;
  SimpleImage * aSI=0;

  /* Needed */
  getline(aifstream,ImageFormat);
  if (m_Verbosity>=4)
    cout << "Image Format : " << ImageFormat << endl;
  
  /* Optional */
  getline(aifstream,PixelLength);
  if (m_Verbosity>=4)
    cout << "Pixel Length : " << PixelLength << endl;

  /* Optional */
  if(PixelLength.substr(0,1)=="#")
    getline(aifstream,DataType);
  else 
    DataType = PixelLength;
  if (m_Verbosity>=4)
    cout << "DataType : " << DataType << endl;

  /* Optional */
  if (DataType.substr(0,1)=="#")
    getline(aifstream,Sign);
  else
    Sign = DataType;
  if (m_Verbosity>=4)
    cout << "Sign : " << Sign << endl;

  /* Optional */
  if (Sign.substr(0,1)=="#")
    getline(aifstream,Endian);
  else
    Endian = Sign;
  if (m_Verbosity>=4)
    cout << "Endian : " << Endian << endl;

  
  /* Pin Hole parameters */
  /* Optional */
  int i=-1;

  if (Endian.substr(0,1)=="#")
    {   
      for(i=0;i<12;i++)
	{
	  getline(aifstream,PinHoleParameter[i]);
	  if (PinHoleParameter[i].substr(0,1)!="#")
	    break;
	  if (m_Verbosity>=4)
	    cout << "PinHoleParameter[i] : " << PinHoleParameter[i] << endl;
	}
    }
  
  if (i!=12)
    {
      if (i!=-1)
	PinHoleParameterF = PinHoleParameter[i];
      else 
	PinHoleParameterF = Endian;
    }
  else
    getline(aifstream,PinHoleParameterF);
  if (m_Verbosity>=4)
    cout << "PinHoleParameterF : " << PinHoleParameterF << endl;

  /* Optional */
  if (PinHoleParameterF.substr(0,1)=="#")
    getline(aifstream,PinHoleParameterM);
  else 
    PinHoleParameterM = PinHoleParameterF;

  if (m_Verbosity>=4)
    cout << "PinHoleParameterM : " << PinHoleParameterM << endl;

  
  /* Distorsion parameters */
  /* Optional */
  i = -1;
  if (PinHoleParameterM.substr(0,1)=="#")
    {
      for(i=0;i<4;i++)
	{
	  getline(aifstream,DistorsionParameter[i]);
	  if (DistorsionParameter[i].substr(0,1)!="#")
	    break;
	  if (m_Verbosity>=4)
	    cout << "DistorsonParameter[i]: " << DistorsionParameter[i] << endl;
	}
    }

  if (i!=4)
    {
      if (i==-1)
	ConvertFlag = PinHoleParameterM;
      else
	ConvertFlag = DistorsionParameter[i];
    }
  else
    getline(aifstream,ConvertFlag);
  if (m_Verbosity>=4)
    cout << "ConvertFlag : " << ConvertFlag << endl;

  /* Optional */
  if (ConvertFlag.substr(0,1)=="#")
    getline(aifstream,Author);
  else
    Author = ConvertFlag;
  if (m_Verbosity>=4)
    cout << "Author :" << Author << endl;

  /* Optional */
  if (Author.substr(0,1)=="#")
    getline(aifstream,Date);
  else
    Date = Author;
  if (m_Verbosity>=4)
    cout << "Date : " << Date << endl;

  /* Needed */
  if (Date.substr(0,1)=="#")
    getline(aifstream,WidthAndHeight);
  else
    {
      WidthAndHeight = Date;
      if (WidthAndHeight.size()==0)
	getline(aifstream,WidthAndHeight);
    }

  while ((WidthAndHeight.substr(0,1)==" ") ||
	 (WidthAndHeight.substr(0,1)=="\t") ||
	 (WidthAndHeight.substr(0,1)=="\n"))
    {
      if (WidthAndHeight.size()!=0)
	WidthAndHeight.erase(0,1);
      else 
	{
	  do 
	    {
	      getline(aifstream,WidthAndHeight);
	    }
	  while(WidthAndHeight.substr(0,1)=="#");
	}
    }

  if (m_Verbosity>=4)
    cout << "WidthAndHeight :" << WidthAndHeight << endl;

  {
    string lWidth, lHeight;
    int pwidth;
    unsigned int Width,Height;
    pwidth = WidthAndHeight.find(" ",0);
    lWidth = WidthAndHeight.substr(0,pwidth);
    lHeight = WidthAndHeight.substr(pwidth+1,WidthAndHeight.length()-pwidth-1);
    Width = atoi(lWidth.c_str());
    Height = atoi(lHeight.c_str());

    if ((Width!=0) && (Height!=0))
      {
	if (m_Verbosity>=4)
	  {
	    cout << "COMPDIM:" << Width << " " << m_ImagesWidth[CameraNumber] << endl;
	    cout << "COMPDIM:" << Height << " " << m_ImagesHeight[CameraNumber] << endl;
	  }
	/*
	if (Width!=m_ImagesWidth[CameraNumber])
	  {
	    m_ImagesWidth[CameraNumber] = Width;
	    r=2;
	  }
	
	if (Height!=m_ImagesHeight[CameraNumber])
	  {
	    m_ImagesHeight[CameraNumber] = Height;
	    r=2;
	  } 
	*/
	if ((unsigned int)CameraNumber<m_ReadImageData.size())
	  {
	    aSI = m_ReadImageData[CameraNumber];
	    if ((aSI->width!=Width) ||
		(aSI->height!=Height))
	      {
		delete aSI->Data ;
		aSI->width = Width;
		aSI->height = Height;
		aSI->Data = new unsigned char[Height*Width];
	      }
	  }
	else
	  {
	    aSI = new SimpleImage;
	    aSI->width= Width;
	    aSI->height = Height;
	    aSI->Data = new unsigned char[Height*Width];
	    m_ReadImageData.insert(m_ReadImageData.end(),aSI);
	  }
      }
    else 
      r = 0;

  }
  getline(aifstream,NbOfValues);
  if (m_Verbosity>=4)
    cout << "NbOfValues : " << NbOfValues << endl;

  return r;
}

int HRP2FileImagesInputMethod::ReadEPBMBodyImage(ifstream & aifstream,
						 unsigned char **buffer,
						 int lw, int lh)
{


  for(int i=0;i<lw*lh;i++)
    aifstream.read(&((*(char **)buffer)[i]),sizeof(unsigned char));

  //  cout << " HRP2FileImagesInputMethod::ReadEPBMBodyImage "  << lw << " "<< lh<< " " << aifstream.eof() << endl;
  int k=0,r=0;
  char aChar;
  bool cont=true;
  if (aifstream.eof())
    cont = false;

  while(cont)
    {
      aifstream.read(&aChar,sizeof(unsigned char));
      //      cout << (int)aChar << " " << k++ << endl;
      if (aChar=='P')
	cont=false;
      if (aifstream.eof())
	cont=false;
    }
#if 0
  ofstream aofstream;
  aofstream.open("/tmp/toto.pgm",iostream::out | iostream::binary);
  if (aofstream.is_open())
    { 
      aofstream << "P5\n" << lw << " " << lh << endl;
      aofstream << 255 << endl;
      aofstream.write(*buffer,lw*lh);
    }
  aofstream.close();
#endif

  if (!aifstream.eof())
    return 0;
  else 
    return 1;

}

int HRP2FileImagesInputMethod::ReadEPBMFileImage(string & aFileName,
						 unsigned char **ImageLeft, 
						 unsigned char **ImageRight,
						 unsigned char **ImageUp,
						 int mode)
{
  int res=0;
  ifstream aifstream;
  unsigned char **ImageWide=0;
  aifstream.open(aFileName.c_str(),iostream::in | iostream::binary);
  if (m_Verbosity>=3)
    cerr << " Start in ReadEPBMFileImage " << aFileName<< endl;

  if (aifstream.is_open())
    {
      int r;
      r = ReadEPBMImageHeader(aifstream,0);
      if ((r==2) && (mode!=1))
	{
	  if (*ImageLeft!=0)
	    delete *ImageLeft;
	  *ImageLeft = new unsigned char[m_ImagesWidth[0]*m_ImagesHeight[0]];
	  fprintf(stderr,"Reallocation for ImageLeft\n");
	  res =2;
	}

      if (mode==0)
	{
	  if (ReadEPBMBodyImage(aifstream,ImageLeft,m_ImagesWidth[0],m_ImagesHeight[0])==1)
	    return res;
	}
      else if (mode==1)
	{
	  if ((r=ReadEPBMBodyImage(aifstream,
				&m_ReadImageData[0]->Data,
				m_ReadImageData[0]->width,
				m_ReadImageData[0]->height))==1)
	    return res;
	}


      
      r = ReadEPBMImageHeader(aifstream,1);

      if ((r==2) && (mode!=1))
	{
	  if (*ImageRight!=0)
	    delete *ImageRight;
	  
	  *ImageRight = new unsigned char[m_ImagesWidth[1]*m_ImagesHeight[1]];
	  fprintf(stderr,"Reallocation for ImageRight %d %d\n",
		  m_ImagesWidth[1], m_ImagesHeight[1]);
	  res = 2;
	}

      if (mode==0)
	{
	  if (ReadEPBMBodyImage(aifstream,ImageRight,m_ImagesWidth[1],m_ImagesHeight[1])==1)
	    return res;
	}
      else if (mode==1)
	{
	  if (ReadEPBMBodyImage(aifstream,
				&m_ReadImageData[1]->Data,
				m_ReadImageData[1]->width,
				m_ReadImageData[1]->height)==1)
	    return res;
	}

      
      r = ReadEPBMImageHeader(aifstream,2);
      if ((r==2) && (mode!=1))
	{
	  if (*ImageUp!=0)
	    delete *ImageUp;
	  *ImageUp = new unsigned char[m_ImagesWidth[2]*m_ImagesHeight[2]];
	  fprintf(stderr,"Reallocation for ImageUp\n");
	  res =2;
	}	    
      
      if (mode==0)
	{
	  if (ReadEPBMBodyImage(aifstream,ImageUp,m_ImagesWidth[2],m_ImagesHeight[2])==1)
	    return res;
	}
      else if (mode==1)
	{
	  if (ReadEPBMBodyImage(aifstream,
				&m_ReadImageData[2]->Data,
				m_ReadImageData[2]->width,
				m_ReadImageData[2]->height)==1)
	    return res;
	}

      
      r = ReadEPBMImageHeader(aifstream,3);

      if ((r==2) && (mode!=1))
	{
	  if (*ImageWide!=0)
	    delete *ImageWide;
	  
	  *ImageWide = new unsigned char[m_ImagesWidth[3]*m_ImagesHeight[3]];
	  fprintf(stderr,"Reallocation for ImageWide %d %d\n",
		  m_ImagesWidth[3], m_ImagesHeight[3]);
	  res = 3;
	}

      if (mode==0)
	{
	  if (ReadEPBMBodyImage(aifstream,ImageRight,m_ImagesWidth[3],m_ImagesHeight[3])==1)
	    return res;
	}
      else if (mode==1)
	{
	  if (ReadEPBMBodyImage(aifstream,
				&m_ReadImageData[3]->Data,
				m_ReadImageData[3]->width,
				m_ReadImageData[3]->height)==1)
	    return res;
	}

    }
  else
    {
      if (m_Verbosity>=1)
	cout << "Unable to open : " << aFileName << " " << m_BaseName << endl;
      return -2;
    }

  aifstream.close();

  return res;
}
  
int HRP2FileImagesInputMethod::GetImage(unsigned char **ImageLeft, unsigned char **ImageRight, unsigned char **ImageUp)
{
  int r=0;

  ODEBUG("Start of FilesImagesInputMethod ");
  switch (m_MethodHowReading)
    {

    case ONEIMAGE:
      r = ReadEPBMFileImage(m_BaseName, ImageLeft, ImageRight, ImageUp,0);
      break;

    case DIRECTORY:
      {
	string alocalfilename;
	do 
	  {
	    char Buffer[1024];
	    
	    bzero(Buffer,1024);
	    
	    sprintf(Buffer,"%s%06d.epbm",
		    m_BaseName.c_str(),m_NbOfImages);
	    alocalfilename = Buffer;
	    
	    r = ReadEPBMFileImage(alocalfilename, ImageLeft, 
				  ImageRight, ImageUp,0);
	    m_NbOfImages++;
	  }
	while (r<0);

      }
      break;
    }
  ODEBUG("End of FilesImagesInputMethod ");
  return r;
}

int HRP2FileImagesInputMethod::GetSingleImage(unsigned char **Image, int camera, struct timeval &timestamp)
{
  int r=0;

  ODEBUG3("GetSingleImage::" << m_InitValue );

  ODEBUG("Start of GetSingleImage " << m_ReadImageData.size()<< " " << camera);
  if (camera==0)
    {
      switch (m_MethodHowReading)
	{
	  
	case ONEIMAGE:
	  r = ReadEPBMFileImage(m_BaseName, 0, 0, 0, 1);
	  m_NbOfImages++;
	  break;
	  
	case DIRECTORY:
	  {
	    string alocalfilename;
	    do 
	      {
		char Buffer[1024];
		
		bzero(Buffer,1024);
		
		sprintf(Buffer,"%s%06d.epbm",
			m_BaseName.c_str(),m_NbOfImages);
		ODEBUG( "Base name : "<< m_BaseName.c_str());
		ODEBUG( "Buffer: " <<Buffer);

		alocalfilename = Buffer;
		ODEBUG("Directory Mode " << m_BaseName);
		r = ReadEPBMFileImage(alocalfilename, 0, 0, 0, 1);
		m_NbOfImages++;
	      }
	    while (r<0);
	    
	  }
	  break;
	}

      if (m_InitValue==2)
	{
	  //r = 2;
	  m_InitValue = 0;
	}
    }
  

  if (*Image==0)
    {
      if (*Image!=0)
	delete *Image;
      *Image = new unsigned char [m_ImagesWidth[camera]*m_ImagesHeight[camera]];
      //      r = 2;
    }

  int intervalw, intervalh, indexd, indexs;
  int BWidth = m_ReadImageData[camera]->width; 
  intervalw =  BWidth / m_ImagesWidth[camera];
  intervalh =  m_ReadImageData[camera]->height/ m_ImagesHeight[camera];

  intervalw = intervalw <0 ? 1: intervalw;
  intervalh = intervalh <0 ? 1: intervalh;

  int ldepth = 1;
  ODEBUG3("Width :" << m_ImagesWidth[camera] << " Height :" << m_ImagesHeight[camera]);
  for(unsigned int j=0;j<m_ImagesHeight[camera];j++)
    {
      for(unsigned int i=0;i<m_ImagesWidth[camera];i++)
	{
	  indexd = j * m_ImagesWidth[camera] + i ;
	  
	  indexs = j * intervalh * BWidth * ldepth  + i * intervalw *ldepth;
	  unsigned long localsum = 0;
	  for(int l=0;l<intervalh;l++)
	    {
	      int lindexs = l * BWidth * ldepth  + indexs;
	      for(int m=0;m<intervalw;m++)
		{
		  localsum += m_ReadImageData[camera]->Data[lindexs+m*ldepth];
		}
	    }
	  
	  (*Image)[indexd] = (unsigned char ) (localsum/(intervalh*intervalw));
	}
    }
  
  //  memcpy(*Image,m_ReadImageData[camera]->Data,
  //m_ReadImageData[camera]->width * m_ReadImageData[camera]->height);

  gettimeofday(&timestamp,0);
  ODEBUG("End of GetSingleImage ");
  return r;

}


string HRP2FileImagesInputMethod::GetFormat()
{
  string aFormat("PGM");
  return aFormat;
}

int HRP2FileImagesInputMethod::GetNumberOfCameras()
{
  if (m_NbOfImages==0)
    {
      struct timeval atimestamp;
      unsigned char * anImage = 0;
      GetSingleImage(&anImage, 0, atimestamp);
      if (anImage!=0)
	delete anImage;
      m_NbOfImages = 0;
      // To be remember the next get single is called.
      // to indicate that a reallocation and a change of size is needed.
      m_InitValue = 2;
    }
  return m_ReadImageData.size();
}
