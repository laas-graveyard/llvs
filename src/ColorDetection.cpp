/** @doc This object implements a color detection algorithm
    based on histograms.

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
#include <ColorDetection.h>

#include <VW/Image/imageio_ppm.h>
#include <VW/Image/imageconversions.h>
#include <iostream>
#include <fstream>
#include <math.h>


#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "ColorDetection:" << x << endl

#if 0
#define ODEBUG(x) cerr << "ColorDetection:" <<  x << endl
#else
#define ODEBUG(x) 
#endif

HRP2ColorDetectionProcess::HRP2ColorDetectionProcess(CosNaming::NamingContext_var cxt,
						     LowLevelVisionServer *lLLVS,
						     string lCalibrationDirectory)
{
  m_CalibrationDirectory = lCalibrationDirectory;
  m_LLVS = lLLVS;
  m_cxt = cxt;
  m_ProcessName = "Color Detection Based On Histogram";
  m_Verbosity = 3;
  m_Computing=0;
  m_ParametersSize = 0;
  m_HtO.Resize(4,4);
  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++)
      if (i==j)
	m_HtO[i][j] = 1.0;
      else
	m_HtO[i][j] = 0.0;

  for(int i=0;i<2;i++)
    {
      m_ImageInputRGB[i] = 0;
      m_ImageResult[i] = 0;
    }
  m_IntervalBetweenConnectionTrials = 10;
  m_NbOfProcessWithoutTrialConnection = 0;

  for (int i=0;i<2;i++)
    {
      m_PI[0] = 0;
    }
  
  m_FirstTime=true;
}

HRP2ColorDetectionProcess::~HRP2ColorDetectionProcess()
{
  if (m_ImageInputRGB!=0)
    {
      for(int i=0;i<m_NbOfCameras;i++)
	delete m_ImageInputRGB[i];
      delete m_ImageInputRGB;
    }

  if (m_ImageResult!=0)
    {
      for(int i=0;i<m_NbOfCameras;i++)
	delete m_ImageResult[i];
      delete m_ImageResult;
    }

}



int HRP2ColorDetectionProcess::InitializeTheProcess()
{
  // Initialize the Color Detector variables.

  m_ColorDetector[0].ReadFromFileAndCreateThePixelList("ColorModel.ppm");
  m_ColorDetector[1].ReadFromFileAndCreateThePixelList("ColorModel.ppm");

  // Get the projective matrices.
  if ((m_LLVS!=0) && (m_NbOfCameras!=0))
    {
      std::string CameraName = m_CalibrationDirectory;
      CameraName += "/Calib.0";
      m_PI[0] = new ProjectiveMatrix(CameraName);
      if(m_NbOfCameras>=2)
	{
	  CameraName = m_CalibrationDirectory;
	  CameraName += "/Calib.1";
	  m_PI[1] = new ProjectiveMatrix(CameraName);
	}
            
    }
  // Try to connect to the remote object performing the task.
  TryConnectionToVisualServoing();


  return 0;
}

void HRP2ColorDetectionProcess::ReadRobotMatrix()
{
  double aFB[16];
  m_LLVS->GetMatrixHeadTOrg(aFB);
  for(int i=0;i<4;i++)
    {
      for(int j=0;j<4;j++)
	m_HtO[i][j] =  aFB[i*4+j];
    }
  ODEBUG("m_HtO : " << m_HtO);
}

int HRP2ColorDetectionProcess::TryConnectionToVisualServoing()
{

  // Connection to the visual servoing plugin.
  CosNaming::Name ncName;
  ncName.length(1);
  ncName[0].kind = CORBA::string_dup("");
  
  if (CORBA::is_nil(m_cxt))
    return 0;

  ncName[0].id = CORBA::string_dup("vs");

  CORBA::Object_ptr anObject=0;
  
  try 
    {
      anObject = m_cxt->resolve(ncName);
    } 
  catch(...)
    {
      ODEBUG3("Visual Servoing not found");
    }

#if 0
  try
    {
      m_VisualServoing = VisualServoingServer::_narrow(anObject);
    }
  catch(...)
    {
      ODEBUG3("Visual Servoing not narrowed");
    }
#endif
  

  return 0;
}

int HRP2ColorDetectionProcess::RealizeTheProcess()
{
  if (!m_Computing)
    return 0;
  
  unsigned int lw,lh;
  double x[m_NbOfCameras],y[m_NbOfCameras];
  int x1[2]={-1,-1},x2[2]={-1,-1};
  double a3DPt[3];
  VNL::Vector<double> Q(4);
  VNL::Vector<double> QHead(3);
  if (m_FirstTime)
    {
      ReadRobotMatrix();
      m_FirstTime = false;
    }

  for(int i=0;i<m_NbOfCameras;i++)
    {
      m_ImageInputRGB[i]->GetSize(lw,lh);
      /* VW::ImageConversions::RGBBuffer_to_VWImage((char *)m_InputImage[i].Image,
	 m_ImageInputRGB[i],lw,lh);*/
      
      m_ColorDetector[i].FilterOnHistogram((unsigned char *)m_InputImage[i].Image,m_ImageResult[i]);

      
      {
	char Buffer[1024];
	sprintf(Buffer,"IntermediateImageResult_%03d.ppm",i);
	m_ImageResult[i]->WriteImage(Buffer);
      }

      m_ColorDetector[i].ComputeCoG(m_ImageResult[i],x[i],y[i]);

      if ((i==1) && (x[0]>=0.0) && (x[1]>=0.0)
	  && (y[0]>=0.0) && (y[1]>=0.0))
	{
	  
	  x1[0] = (int) x[0]; x1[1] = (int)y[0];
	  x2[0] = (int) x[1]; x2[1] = (int)y[1];
	  m_TM.Triangulation(*m_PI[0],*m_PI[1],x1,x2,a3DPt);
	  for(int j=0;j<3;j++)
	    Q[j] = a3DPt[j];
	  Q[3] = 1.0;

	  QHead = m_HtO * Q;
	  QHead *= 0.001;
	  ODEBUG( "Triangulation: " 
		   << a3DPt[0] << " " 
		   << a3DPt[1] << " " 
		   << a3DPt[2]);
	  ODEBUG3("QHead :" << QHead <<endl );
	       
	
#if 0
	  if (!CORBA::is_nil(m_VisualServoing))
	    {
	      try 
		{
		  // m_VisualServoing->SendTarget2DPosition(x[i],y[i]);
		  CORBA::Double x_left = x[0];
		  CORBA::Double y_left = y[0];
		  CORBA::Double X = QHead[0];
		  CORBA::Double Y = QHead[1];
		  CORBA::Double Z = QHead[2];
	      
		  m_VisualServoing->SendTarget3DPosition(x_left,y_left,X,Y,Z);
		  ODEBUG("Send " << x << " " << y << " to visual servoing");
		}
	      
	      catch(...)
		{
		  
		  ODEBUG("Sorry the data was not send to the visual servoing plugin" );
		  if (m_NbOfProcessWithoutTrialConnection>m_IntervalBetweenConnectionTrials)
		    {
		      CORBA::release(m_VisualServoing);
		      TryConnectionToVisualServoing();
		      m_NbOfProcessWithoutTrialConnection=0;
		    }
		  else
		    m_NbOfProcessWithoutTrialConnection++;
		  
		}
	    }
	  else 
	    {
	      if (m_NbOfProcessWithoutTrialConnection>m_IntervalBetweenConnectionTrials)
		{
		  TryConnectionToVisualServoing();
		  m_NbOfProcessWithoutTrialConnection=0;
		}
	      else
		m_NbOfProcessWithoutTrialConnection++;
	    }
#endif
	}
      /*
      if ((x==-1) && (y==-1)) 
	{
	  epbm_save("ImageInput.epbm",&m_InputImage[i],0);
	  m_ImageResult[i]->WriteImage("ImageIntermediate.pgm");
	  exit(0);
	  //m_ImageInputRGB[0]->WriteImage("ImageInput.ppm"); 
	}
	ODEBUG3("Center ( " << i << " ) :"<< x << " " << y ); */
    }
  return 0;
}

int HRP2ColorDetectionProcess::CleanUpTheProcess()
{
  return 0;
}


int HRP2ColorDetectionProcess::SetInputImages(EPBM InputImage)
{
  m_InputImage[0] = InputImage;
  m_ImageInputRGB[0] = new VW::ImageRGB<unsigned char>(m_InputImage[0].Width,m_InputImage[0].Height);
  m_ImageResult[0] = new VW::ImageMono<unsigned char>(m_InputImage[0].Width,m_InputImage[0].Height);
  m_ColorDetector[0].InitializeIntermediateStructure(m_ImageInputRGB[0]);
  m_NbOfCameras=1;

  return 0;
}

int HRP2ColorDetectionProcess::SetInputImages(EPBM InputImage1,EPBM InputImage2)
{
  m_InputImage[0] = InputImage1;
  m_InputImage[1] = InputImage2;

  m_ImageInputRGB[0] = new VW::ImageRGB<unsigned char>(m_InputImage[0].Width,m_InputImage[0].Height);                     
  m_ImageInputRGB[1] = new VW::ImageRGB<unsigned char>(m_InputImage[1].Width,m_InputImage[1].Height);

  m_ImageResult[0] = new VW::ImageMono<unsigned char>(m_InputImage[0].Width,m_InputImage[0].Height);
  m_ImageResult[1] = new VW::ImageMono<unsigned char>(m_InputImage[1].Width,m_InputImage[1].Height);

  m_ColorDetector[0].InitializeIntermediateStructure(m_ImageInputRGB[0]);
  m_ColorDetector[1].InitializeIntermediateStructure(m_ImageInputRGB[1]);


  m_NbOfCameras=2;
  
  return 0;
}



#if 0
/* TO DO */
int HRP2ColorDetectionProcess::GetValueOfParameter(string aParameter, string &aValue)
{
}


int HRP2ColorDetectionProcess::StateMachineForFridgeDection()
{
  
}
#endif
