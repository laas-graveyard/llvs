/** @doc This object implements a visual process
    performing disparity computation.

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
#include <iostream>
#include <fstream>
#include "DisparityProcess.h"
#include "reconstruction.h"
#include "isoluminance.h"

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "HPR2DisparityProcess:" << x << endl
#define ODEBUG3_CONT(x) cerr << x 

#if 1
#define ODEBUG(x) cerr << "HPR2DisparityProcess:" <<  x << endl
#define ODEBUG_CONT(x) cerr << "HPR2DisparityProcess:" <<  x << endl
#else
#define ODEBUG(x) 
#define ODEBUG_CONT(x) 
#endif

extern "C"
{
#include "epbm.h"
#include "rangeopen.h"
}

#define HRP2DPINDEX(x,y,z) (x*m_ImageSizeInLUT[0]*m_ImageSizeInLUT[1])+(y*m_ImageSizeInLUT[0])+z

HRP2DisparityProcess::HRP2DisparityProcess(unsigned char BooleanForErrorModel) : HRP2VisionBasicProcess()
{
  m_ProcessName = "Disparity Map (VVV)";
  m_PointsInImageLeft = 0;
  m_PointsInImageRight = 0;
  m_UseErrorModel = BooleanForErrorModel;
  m_DIPs = 0;
  m_ModeToDumpMatching3DPoints = DUMP_MODE_3D_POINTS_OFF;

  m_TexturelessMasking = false;

  string aParameter ="Disparity";
  string aValue = "";

  /* Default value for the disparity process */

  /* iWndows size to compute disparity */
  aParameter = "-w";
  aValue = "31";
  SetParameter(aParameter,aValue);

  /* Min depth for disparity computation. */
  aParameter = "-z";
  aValue = "200";
  SetParameter(aParameter,aValue);

  /* Max depth for disparity computation. */
  aParameter = "-Z";
  aValue = "500";
  SetParameter(aParameter,aValue);

  /* Subsampling for error models */
  m_SubsampleIA=1;
  m_SubsampleHZ=1;

  if (m_UseErrorModel)
    {
      /* By default set the size to 80 by 60 */
      InitializeErrorModel();
    }
  
}

HRP2DisparityProcess::~HRP2DisparityProcess()
{

  if (m_PointsInImageLeft!=0)
    delete m_PointsInImageLeft;

  if (m_PointsInImageRight!=0)
    delete m_PointsInImageRight;

  epbm_free(&m_Depbm);

}

void HRP2DisparityProcess::InitializeErrorModel()
{
  if (m_BoundingBox!=0)
    {
      delete m_BoundingBox;
      m_BoundingBox = 0;
    }
    
  if ((m_ErrorModel==ERROR_MODEL_ONLINE_INTERVALS) ||
      (m_ErrorModel==ERROR_MODEL_HZ))
    {
	
      if (m_ErrorModel==ERROR_MODEL_ONLINE_INTERVALS)
	{
	  m_CamL.create(3,4);
	  m_CamR.create(3,4);
	  m_BoundingBox = new float [6*m_Depbm.Width*m_Depbm.Height];
	}
      else if (m_ErrorModel==ERROR_MODEL_HZ)
	{
	  m_PLeft = new TU::Matrix<double> (3,4);
	  m_PRight = new TU::Matrix<double> (3,4);
	  m_BoundingBox = new float [12*m_Depbm.Width*m_Depbm.Height];
	}
      /* Initialize the camera models */
      for(int i=0;i<2;i++)
	{
	  double lH[3][4];
	  
	  /* Store the rigid transformation */
	  //	  scm_sp2SCMH(&m_sp, SCM_LEFT,lH,0);
	  //corr_scm_Htow2c(lH,m_w2c[i]);
	  corr_scm_Htow2c(m_sp.H[i],m_w2c[i]);
	  scm_inverse_matrix_3D((double *)m_w2c[i],(double *) m_w2c[i], NULL);
	  /* Denormalize the matrix, might be unnecessary */
	  double oP[3][4];
	  for(int k=0;k<3;k++)
	    {
	      for(int j=0;j<4;j++)
		{
		  oP[k][j] = m_sp.H[i][k][j];
		  //cout << m_w2c[i][k][j] << " ";
		}
	      //cout << endl;
	    }
	  
	  
	  double w = sqrt(oP[2][0] * oP[2][0] + oP[2][1] * oP[2][1] + oP[2][2] * oP[2][2]);

	  int sdet=1;
	  
	  double q[3][3];
	  double r[3][3];
	  double a[3][3];
  
	  scm_H2q((double (*)[4])m_sp.H[i][0],q,0);
	  scm_q2r(q,r,0);
	  scm_matrix_by_matrix_3D((double *)q, (double *)r, (double *)a);

	  if (a[0][0]<0.0)
	    sdet = -1;
	    
	  for(int k=0;k<3;k++)
	    {
	      for(int j=0;j<4;j++)
		{
		  oP[k][j] = sdet*oP[k][j]/w;
		}
	    }
	  /* Store into the structure needed for Interval Analysis. */
	  for(int k=0;k<3;k++)
	    {
	      for(int j=0;j<4;j++)
		{
		  if (i==0)
		    {
		      if (m_ErrorModel==ERROR_MODEL_ONLINE_INTERVALS)
			{
			  m_CamL.set(m_sp.H[i][k][j],k+1,j+1);
			}
		      else if (m_ErrorModel==ERROR_MODEL_HZ)
			{
			  (*m_PLeft)[k][j]=m_sp.H[i][k][j];
			}
		    }
		  else if (i==1)
		    //		    m_CamR.set(oP[k][j],k+1,j+1);
		    {
		      if (m_ErrorModel==ERROR_MODEL_ONLINE_INTERVALS)
			{
			  m_CamR.set(m_sp.H[i][k][j],k+1,j+1);
			}
		      else if (m_ErrorModel==ERROR_MODEL_HZ)
			{
			  (*m_PRight)[k][j]=m_sp.H[i][k][j];
			}
		    }
		}
	    }
	}
    }
  
  if (m_ErrorModel==ERROR_MODEL_ONLINE_INTERVALS)
    {
 
      m_NbOfPointsWithInformation = 0;
      
      if (m_Verbosity)
	cerr << "... Continuing the initialization of HRP2DisparityProcess " << endl
	     << "... Read Look Up Table ... " << endl;
      ReadLookUpTable();
      if (m_Verbosity)
	cerr << "... Finished to read the Look Up Table ..." << endl
	     << "... Finished the initialization of HRP2DisparityProcess" << endl;

      if (m_DIPs!=0)
	delete m_DIPs;
      
      m_DIPs = 0;
      
      int TotalSize = m_ImageSizeInLUT[0] * 
	m_ImageSizeInLUT[1];

      m_DIPs = new DataIntervalPoint_t *[TotalSize];
      for(int i=0;i<TotalSize;i++)
	  m_DIPs[i] = 0; 
 

    }

}

TU::Matrix<double> HRP2DisparityProcess::BuildCovarianceMatrixHZ(TU::Vector<double> q1, TU::Vector<double> q2, 
					   TU::Matrix<double> P1, TU::Matrix<double> P2,
					   TU::Matrix<double> CovX, TU::Matrix<double> Theta)
{
  TU::Matrix<double> A(4,4);
  TU::Matrix<double> dAdX(4,4);
  
  // Build A
#if 0
  cout << "Theta: "<< endl<<Theta<<endl;
  cout << "q1:" << q1 << endl;
  cout << "q2:" << q2 << endl;
  cout << "P1:" << P1 <<endl;
  cout << "P2:" << P2 <<endl;
#endif
  TU::Matrix<double> W(4,4);
  TU::Matrix<double> vTheta(4,1),Mp(4,4);

  for(int j=0;j<4;j++)
    {
      vTheta[j][0] = Theta[j][j];
      for(int i=0;i<4;i++)
	{
	  W[j][i] = 0.0;
	  if (j<2)
	    Mp[j][i] = P1[2][i];
	  else
	    Mp[j][i] = P2[2][i];
	}
    }
  for(int i=0;i<4;i++)
    {
      double tmp = P1[2][i]*Theta[i][i];
      W[0][0] +=tmp;
      W[1][1] +=tmp;
    }
  for(int i=0;i<4;i++)
    {
      double tmp = P2[2][i]*Theta[i][i];
      W[2][2] += tmp;
      W[3][3] += tmp;
    }
  
  for(int i=0;i<4;i++)
    if (W[i][i]!=0.0)
      W[i][i]=1/W[i][i];

  // First line and second line
  for(int j=0;j<2;j++)
    for(int i=0;i<4;i++)
      A[j][i] = (q1[j]*P1[2][i] - P1[j][i]);
  
  // Third line and fourth line
  for(int j=0;j<2;j++)
    for(int i=0;i<4;i++)
      A[2+j][i] = (q2[j]*P2[2][i] - P2[j][i]);

  
  //  cout << "A: " << endl << A <<endl;
  // Build A+
  TU::Matrix<double> N(4,4);
#if 0
  cout << "Before computing N" << "A: " << endl<< 
    A << endl << " CovX :" << endl <<
    CovX<<  endl;
  cout << " CovX.inv() "<< endl <<
    CovX.inv();
    
  cout << " CovX.inv() * A " << endl <<
    CovX.inv() * A;
#endif
  double w=sqrt(Theta[0][0]*Theta[0][0] + 
	 Theta[1][1]*Theta[1][1] +
	 Theta[2][2]*Theta[2][2] +
	 Theta[3][3]*Theta[3][3]);


  N = A*W;
  
  // cout << "N:" << endl << N;

  //  cout << "Rank: " << rank<< " Tol: " << tol << " svd[0]/svd[2]:"<<svd[0]/svd[2]<< endl;

  TU::Matrix<double> CovTheta(4,4);
  //  CovTheta = N.pinv(tol) * M.trns() * CovX * M * N.pinv(tol).trns();
  CovTheta = N.trns() * CovX.inv() * N;
  TU::SVDecomposition<double> svd(CovTheta);
  double tol= svd[0]/svd[2]+1;
  int rank=1;

  CovTheta = CovTheta.pinv(tol);
  /*
  for(int j=0;j<4;j++)
    for(int i=0;i<4;i++)
      CovTheta[j][i] = CovTheta[j][i]/CovTheta[4][4];
  */
  //CovTheta = N.pinv(tol).trns() * M.trns() * CovX * M * N.pinv(tol);
  //  cout << "M " <<endl<<M;
  //  cout << "Theta:" <<endl<<Theta;
  //  cout << "dAdX.trns() * A * Theta "<< endl << dAdX.trns() * A * Theta;
  //  cout << "A.trns() * dAdX *Theta " <<endl<<A.trns() * dAdX *Theta;
  //  cout << " N.pinv(tol) * M.trns()"<<endl<<N.pinv(tol) * M.trns();
  //  cout << "M * N.pinv(tol).trns() "<< endl<<M * N.pinv(tol).trns();
  //  cout << "Almost CT:"<< endl<<N.pinv(tol) * M.trns()* M * N.pinv(tol).trns();
  //  cout << "CovTheta:" << endl<<CovTheta<<endl;
  return CovTheta;
}

int HRP2DisparityProcess::InitializeTheProcess(int CalibrationWidth, int CalibrationHeight)
{
#if 0
  int argc=14;
  char *argv[14]= { "Disparity", "-w", "31","-Flimit", "-z","1000","-Z","3000", "-FT", "20.0","-Fspike-area","30", "-FC","-Ffake"};
#else
#if 0
  int argc=9;
  char *argv[9]= { "Disparity", "-w", "15","-Flimit", "-z","2400","-Z","5000","-FC"};
#else
#if 0
  int argc=9;
  char *argv[9]= { "Disparity", "-w", "31","-z","200","-Z","500","-Flimit","-FC" };
#endif
#endif
#endif
  //char *argv[4]= { "Disparity", "-w", "31", "-Flimit"};
  /* Arguments construction during the initialization process */
  int argc = 1;
  char **argv;
  
  for(int i=0;i<m_ParametersSize;i++)
    {
      if (m_VectorOfParameters[i].length()!=0)
	argc++;
      if (m_VectorOfValuesForParameters[i].length()!=0)
	argc++;
    }
  
  argv = (char **)new char *[argc];

  char ProgramName[124]="Disparity";
  int n=0;
  argv[n]= new char[strlen(ProgramName)+1];
  bzero(argv[n],strlen(ProgramName)+1);
  strcpy(argv[n],ProgramName);
  n++;

  for(int i=0;i<m_ParametersSize;i++)
    {
      if (m_VectorOfParameters[i].length()!=0)
	{
	  argv[n] = new char[m_VectorOfParameters[i].length()];
	  bzero(argv[n],m_VectorOfParameters[i].length()+1);
	  strcpy(argv[n],m_VectorOfParameters[i].c_str());
	  n++;
	}
      if (m_VectorOfValuesForParameters[i].length()!=0)
	{
	  argv[n] = new char[m_VectorOfValuesForParameters[i].length()+1];
	  bzero(argv[n],m_VectorOfValuesForParameters[i].length()+1);
	  strcpy(argv[n],m_VectorOfValuesForParameters[i].c_str());
	  n++;
	}


    }
#if 0
  ODEBUG("Insider Disparity Process Arguments ");
  for(int i=0;i<argc;i++)
    ODEBUG_CONT(argv[i] << " ");
  ODEBUG("");
  ODEBUG("/***********************************/");
#endif
  if (m_Verbosity)
    cerr << "Started the initialization of HRP2DisparityProcess ..." << endl;
  init_compute_disparity_map(argc, argv,
			     m_InputImage,
			     m_Depbm,
			     m_rng,
			     m_para,
			     m_sp,
			     m_IPFC );
  m_para.calibration_image_width = CalibrationWidth;
  m_para.calibration_image_height = CalibrationHeight;
  
  if (m_Verbosity)
    cerr << "Phase 2 for the initialization of HRP2DisparityProcess ..." << endl;
  //  fprintf(stderr,"DisparityProcess: Init_Process - Save range data: %s\n",m_IPFC.rngfname);  
  /* Initialize the data space for the points needed
     for the error */
  if (m_PointsInImageLeft!=0)
    delete m_PointsInImageLeft;

  if (m_PointsInImageRight!=0)
    delete m_PointsInImageRight;

  if (m_Verbosity)
    cerr << "Phase 3 for the initialization of HRP2DisparityProcess ..." << endl;

  m_PointsInImageLeft = new int [2*m_Depbm.Width*m_Depbm.Height];
  m_PointsInImageRight = new int [2*m_Depbm.Width*m_Depbm.Height];
  m_ColorInImageLeft = new unsigned char [3*m_Depbm.Width*m_Depbm.Height];
  
  InitializeErrorModel();
  
  if (m_Verbosity)
    cerr << "Phase 4 for the initialization of HRP2DisparityProcess ..." << endl;

  if (m_TexturelessMasking)
    {
      int argc = 1;
      char *argv[0] ;

      ODEBUG("Initialization of the TexturelessImage ");
      for(int i=0;i<3;i++)
	{
	  m_TexturelessImage[i] = epbm_create( EPBM_BINARY_GRAY, m_InputImage[0].Width, m_InputImage[0].Height,
					       EPBM_UNSIGNED, EPBM_CHAR8, NULL );
	}
      m_InitParForIsoL.nimages= 2;
      epbm_init_isoluminance(&m_InitParForIsoL, m_InputImage, argc, argv);
    }
  if (m_Verbosity)
    cerr << "End of  the initialization of HRP2DisparityProcess ..." << endl;

  return 0;
}

float * HRP2DisparityProcess::GetBoundingBoxes()
{
  return m_BoundingBox;
}

int HRP2DisparityProcess::RealizeTheProcess()
{
  ODEBUG3((int)m_Computing);
  if (!m_Computing)
    return 0;
  EPBM lInput[2];

  if (m_InputImage[0].Magic2==EPBM_BINARY_COLOR)
    {
      for(int i=0;i<2;i++)
	{
	  for (int scm_row = 0; scm_row < m_InputImage[i].Height; scm_row++) {
	    for (int scm_col = 0; scm_col < m_InputImage[i].Width; scm_col++) {
	      epbm_uc_putpixel(&m_GrayInputImage[i],scm_row, scm_col,
			       (unsigned char) (0.299 * epbm_uc_cgetpixel(&m_InputImage[i],EPBM_RED,scm_row, scm_col) +
						0.587 * epbm_uc_cgetpixel(&m_InputImage[i],EPBM_GREEN,scm_row, scm_col) +
						0.114 * epbm_uc_cgetpixel(&m_InputImage[i],EPBM_BLUE,scm_row, scm_col)));
			       
	    }
	  }
	}

      
      lInput[0] = m_GrayInputImage[0];
      lInput[1] = m_GrayInputImage[1];
    }
  else
    {
       lInput[0] = m_InputImage[0];
       lInput[1] = m_InputImage[1];
    }
#if 1
  static int lstaticImageIndex = 0;
  char FileName[256];
  sprintf(FileName,"/tmp/checkLeftInput%06d.epbm",lstaticImageIndex);
  epbm_save(FileName,&m_InputImage[0],0);
  sprintf(FileName,"/tmp/checkRightInput%06d.epbm",lstaticImageIndex);
  lstaticImageIndex++;
  epbm_save(FileName,&m_InputImage[1],0);
#endif

  /* Compute the disparity map */
  ODEBUG("Start the disparity process. " << m_InputImage[0].Magic2);
  if (m_TexturelessMasking)
    {
      ODEBUG("Textureless Masking is working");
      epbm_isoluminance_main(lInput, &m_TexturelessImage[0], &m_InitParForIsoL,1);
      //      epbm_save("TexturelessImage.epbm",&m_TexturelessImage[0],0);
      compute_disparity_map(lInput, m_Depbm, m_rng, m_para, m_sp, m_IPFC,&m_TexturelessImage[0],&m_InputImage[0]);
    }
  else 
    compute_disparity_map(lInput, m_Depbm, m_rng, m_para, m_sp, m_IPFC, (EPBM*)0,&m_InputImage[0]);

  ODEBUG("End the disparity process.");

  if (m_Verbosity>=2)
    {    

      if (m_Verbosity>=4)
	{    
	  
	  for(int i=0;i<m_rng.MapSize;i++)
	    {
	      PixelData *pmap;
	      pmap = (m_rng.Map)[i];
	      if (pmap!=0)
		{
		  cerr << "DisparityProcess:RealizeTheProcess: Position :" << i % m_rng.Width << " " << i / m_rng.Width << " ";
		  for (int k = 0; k < 3; k++)
		    {
		      cerr << pmap << " " <<*(pmap->Dot +k) << " ";
		    }
		  cerr << ";";
		}
	      if (i%m_rng.Width==0)
		cerr <<endl;
	    }
	}
      cout << "************************************************"<<endl;
      cerr << "DisparityProcess::RealizeTheProcess : Number of points : " << m_rng.DotCount << endl;
    }


  /* Use the error model if specified */
  if (m_UseErrorModel)
    {
      if (m_ErrorModel==ERROR_MODEL_ONLINE_INTERVALS)
	ComputeIntervalError(ERROR_MODEL_ONLINE_INTERVALS);
      else if (m_ErrorModel==ERROR_MODEL_HZ)
	ComputeHZError();
    }

  if ((m_ModeToDumpMatching3DPoints==DUMP_MODE_3D_POINTS_PLY) ||
      (m_ModeToDumpMatching3DPoints==DUMP_MODE_3D_RANGE_DATA_AND_PLY))
    WriteToPlyFormat();

  
  if ((m_ModeToDumpMatching3DPoints==DUMP_MODE_3D_RANGE_DATA) ||
      (m_ModeToDumpMatching3DPoints==DUMP_MODE_3D_RANGE_DATA_AND_PLY))
    range_save("E.rng",&m_rng,0);

  return 0;
}

int HRP2DisparityProcess::CleanUpTheProcess()
{
  return 0;
}

int HRP2DisparityProcess::SetInputImages(EPBM lInputImages[2])
{
  m_InputImage[0] = lInputImages[0];
  m_InputImage[1] = lInputImages[1];

  if (m_InputImage[0].Magic2==EPBM_BINARY_COLOR)
    {
      for(int i=0;i<2;i++)
	m_GrayInputImage[i] = epbm_create(EPBM_BINARY_GRAY, lInputImages[i].Width, lInputImages[i].Height,
					  EPBM_UNSIGNED, EPBM_CHAR8, NULL );
    }
  return 0;
}


int HRP2DisparityProcess::SetModeDumpMatching3DPoints(int aMode)
{
  if ((aMode>=0) && (aMode<3))
    {
      m_ModeToDumpMatching3DPoints = aMode;
      return 0;
    }

  return -1;
}

int HRP2DisparityProcess::GetModeDumpMatching3DPoints()
{
  return m_ModeToDumpMatching3DPoints;
}

/********* Methods related to the internal error model ***************************/

/* Find the matching points in the original reference frame 
 * At the end m_NbOfPoitnsWithInformation gives the number of pairs found.
 * m_PointsInImageLeft[2*i] and m_PointsInImageLeft[2*i+1] contains
 * the coordinates x and y respectivly which match the  point
 * m_PointsInImageRight[2*i] and m_PointsInImageRight[2*i+1] 
 * in the Right image.
 */
int HRP2DisparityProcess::FindMatchingInImagesCoordinates()
{
  int i,j;
  int disp;
  int offset = m_para.dmin_SCM;
  double scrL[2], scrR[2],crL[2],crR[2];
  string aFileName;
  FILE *fp = 0;
  double OriginScrLeft[2] ,OriginScrRight[2];
  double OriginCrRight[2],OriginCrLeft[2];
  double intervalh, intervalw;
  

  intervalw = (double)m_para.calibration_image_width  / (double)m_para.image_width;
  intervalh = (double)m_para.calibration_image_height / (double)m_para.image_height;

  scm_SCMcr2cr( &m_sp, SCM_LEFT, OriginScrLeft, OriginCrLeft);
  scm_SCMcr2cr( &m_sp, SCM_LEFT, OriginScrRight, OriginCrRight);
  ODEBUG("Value of the origin in the virtual original images: " <<
	 OriginCrLeft[0]/intervalw << " " << OriginCrRight[1]/intervalh << " " << 
	 OriginScrLeft[0]/intervalw << " " << OriginScrRight[1]/intervalh );

  if ((m_ModeToDumpMatching3DPoints==DUMP_MODE_3D_POINTS_MATCHING) ||
      (m_ModeToDumpMatching3DPoints==DUMP_MODE_3D_POINTS_INTERVALS_MATCHING))
    {
      aFileName = "MatchedPoints.dat";
      fp = fopen(aFileName.c_str(),"w");
    }

  m_NbOfPointsWithInformation = 0;

  /* For each couple of 2D point you should reconstruct the coordinates 
   * of the 2D points in the uncorrected images.
   */
  ofstream dump_tmp;
  dump_tmp.open("dump_interval.dat",ofstream::out);

  for( i=0; i<m_Depbm.Height; i++ ){
    for( j=0; j<m_Depbm.Width; j++ ){
      
      
      /* Take the disparity */
      if(m_Depbm.DataType == EPBM_CHAR8){
	disp = epbm_uc_getpixel( &m_Depbm, i, j );
	if( disp == DISPARITY_ERROR_NUMBER_UCHAR ){
	  continue;
	}
      }else{
	disp = epbm_s_getpixel( &m_Depbm, i, j );
 	if( disp == DISPARITY_ERROR_NUMBER_SHORT ){
	  continue;
	}
      }
      
      /* Goes back to the unreconstructed images */
      scrL[0] = j*intervalw;
      scrL[1] = i*intervalh;
      if(m_IPFC.disp_f)
	{
	  scrR[0] = j*intervalw - (m_IPFC.disp_f[i*m_Depbm.Width+j]*intervalw + offset*intervalw);
	}
      else
	{
	  scrR[0] = j*intervalw - (disp + offset)*intervalw;
	}
      scrR[1] = i*intervalh;

      scm_SCMcr2cr( &m_sp, SCM_LEFT, scrL, crL);
      scm_SCMcr2cr( &m_sp, SCM_RIGHT, scrR, crR);
      
      /* IMPORTANT:
	 The current coordinates are given in the CALIBRATION reference frame ! 
      */
      m_PointsInImageLeft[2*m_NbOfPointsWithInformation] = (int)(crL[0]/intervalw);
      m_PointsInImageLeft[2*m_NbOfPointsWithInformation+1] = (int)(crL[1]/intervalh);      
      m_PointsInImageRight[2*m_NbOfPointsWithInformation] = (int)(crR[0]/intervalw);
      m_PointsInImageRight[2*m_NbOfPointsWithInformation+1] = (int)(crR[1]/intervalh);
      m_ColorInImageLeft[3*m_NbOfPointsWithInformation] = 
      m_ColorInImageLeft[3*m_NbOfPointsWithInformation+1] = 
      m_ColorInImageLeft[3*m_NbOfPointsWithInformation+2] = epbm_uc_getpixel( &m_InputImage[0], i, j );
      if ((m_ModeToDumpMatching3DPoints==DUMP_MODE_3D_POINTS_MATCHING) ||
	  (m_ModeToDumpMatching3DPoints==DUMP_MODE_3D_POINTS_INTERVALS_MATCHING))
	fprintf(fp,"%f %f %f %f %d\n",
		crL[0],crL[1],crR[0],crR[1], (int)m_ColorInImageLeft[3*m_NbOfPointsWithInformation] );
      /*      printf("%d %d %d %d\n",
	     m_PointsInImageLeft[2*m_NbOfPointsWithInformation],
	     m_PointsInImageLeft[2*m_NbOfPointsWithInformation+1],
	     m_PointsInImageRight[2*m_NbOfPointsWithInformation],
	     m_PointsInImageRight[2*m_NbOfPointsWithInformation+1]); */

      dump_tmp << crL[0]<< " " << crL[1]<< " " << crR[0]<< " " << crR[1]  << " "
	       << scrL[0]<< " " << scrL[1]<< " " << scrR[0]<< " " << scrR[1] << " "
	       << m_IPFC.disp_f << " " << offset << " " << intervalw << " " 
	       << disp << endl;
      m_NbOfPointsWithInformation++;
    }
  }
  dump_tmp.close();
  if ((m_ModeToDumpMatching3DPoints==DUMP_MODE_3D_POINTS_MATCHING) ||
      (m_ModeToDumpMatching3DPoints==DUMP_MODE_3D_POINTS_INTERVALS_MATCHING))
    fclose(fp);
  return 0;
}

int HRP2DisparityProcess::GetSubsampleIA()
{
  return m_SubsampleIA;
}

int HRP2DisparityProcess::GetSubsampleHZ()
{
  return m_SubsampleHZ;
}

void HRP2DisparityProcess::SetSubsampleIA(int aValue)
{
  m_SubsampleIA = aValue;
}

int HRP2DisparityProcess::ComputeHZError()
{
  TU::Vector<double> q1(2),q2(2);
  TU::Matrix<double> Theta(4,4),CovTheta(4,4);
  TU::Matrix<double> CovX(4,4);

  double intervalw,intervalh;

  intervalw = (double)m_para.calibration_image_width / (double)m_para.image_width;
  intervalh = (double)m_para.calibration_image_height / (double)m_para.image_height;
  for(int l2j=0;l2j<4;l2j++)
    for(int l2i=0;l2i<4;l2i++)
      Theta[l2j][l2i] = 0.0;
  Theta[3][3] = 1.0;		  

  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++)
      CovX[i][j] = 0.0;
  for(int i=0;i<4;i++)
    CovX[i][i] = 0.7*0.7; // Standard deviation = 0.7
    
  PixelData *aPD;
  ODEBUG("Start of ComputeHZError() " << m_rng.MapSize);
  for (int i=0;i<m_rng.PixelCount;i+=m_SubsampleHZ)
    {
      aPD = m_rng.PixelList+i;
      q1[0] = m_PointsInImageLeft[2*i]*intervalw;
      q1[1] = m_PointsInImageLeft[2*i+1]*intervalh;
      q2[0] = m_PointsInImageRight[2*i]*intervalw;
      q2[1] = m_PointsInImageRight[2*i+1]*intervalw;
      
      ODEBUG( aPD->Dot[0] << " " << aPD->Dot[1] << " " << aPD->Dot[2]);
      Theta[0][0] = aPD->Dot[0];
      Theta[1][1] = aPD->Dot[1];
      Theta[2][2] = aPD->Dot[2];
      CovTheta = BuildCovarianceMatrixHZ(q1, q2,
				      (*m_PRight), (*m_PLeft),
				      CovX, Theta);
      ODEBUG("phase2");
      m_BoundingBox[12*i+0] = Theta[0][0];
      m_BoundingBox[12*i+1] = Theta[1][1];
      m_BoundingBox[12*i+2] = Theta[2][2];
      ODEBUG("phase3");
      for(int j=0;j<3;j++)
	for(int k=0;k<3;k++)
	  m_BoundingBox[12*i+3+3*j+k] = CovTheta[j][k];
      ODEBUG("phase4");
      
    }
  ODEBUG("ComputeHZError");
  return 0;
}

int HRP2DisparityProcess::ComputeIntervalError(int MethodToCompute)
{
  Appariement match;  
  MatrixInterval Xnh;
  DataIntervalPoint_t *aDIP;
  double intervalh, intervalw;
  ofstream aofstream;

  intervalw = (double)m_para.calibration_image_width / (double)m_para.image_width;
  intervalh = (double)m_para.calibration_image_height / (double)m_para.image_height;
  
  int Contractor = KRAWCZYK;
  int Computation_Method = 1;

  match.ptG.create(3,4);
  match.ptD.create(3,4);
  match.ptGcor.create(3,1);
  match.ptDcor.create(3,1);
  
  Xnh.create(3,1);
  
  FindMatchingInImagesCoordinates();
  
  if ((m_ModeToDumpMatching3DPoints==DUMP_MODE_3D_POINTS_INTERVALS) ||
      (m_ModeToDumpMatching3DPoints==DUMP_MODE_3D_POINTS_INTERVALS_MATCHING))
    {
      aofstream.open("Intervals.dat",ofstream::out);
      aofstream << m_NbOfPointsWithInformation/m_SubsampleIA << endl;
    }
  if (m_Verbosity>=3)
    printf("Number of points : %d %d\n", m_NbOfPointsWithInformation/m_SubsampleIA,
	   MethodToCompute);

  for (int i=0;i<m_NbOfPointsWithInformation;i+=m_SubsampleIA)
    {
      
      switch (MethodToCompute)
	{
	  
	case ERROR_MODEL_ONLINE_INTERVALS:
	  /* Initializes the data */
	  ODEBUG(intervalw << " "<< intervalh);
	  match.ptG.set(m_PointsInImageLeft[2*i]*intervalw,intervalw/2.0,1,1);
	  match.ptG.set(m_PointsInImageLeft[2*i+1]*intervalh,intervalh/2.0,2,1);
	  match.ptG.set(1.0,3,1);
	  
	  match.ptD.set(m_PointsInImageRight[2*i]*intervalw,intervalw/2.0,1,1);
	  match.ptD.set(m_PointsInImageRight[2*i+1]*intervalh,intervalh/2.0,2,1);
	  match.ptD.set(1.0,3,1);
	  match.length=1;
	  
	  /* Computes the scene */
	  if (Computation_Method ==0)
	    {
	      if (i==0)
		ComputeScene(match,Contractor,m_CamL,m_CamR,m_NbOfPointsWithInformation,&m_sp,10);
	      else 
		ComputeScene(match,Contractor,m_CamL,m_CamR,-1,&m_sp,10);
	    }
	  else
	    {
	      	MatrixInterval iql; iql.create(3,1);
		MatrixInterval iqr; iqr.create(3,1);
		MatrixInterval iQ ; iQ.create(3,1);
		int nb_iterations = 7;
		double uncertainty=0.5;

		finterval Gx,Gy;
		Gx = match.ptG.get(1,1);
		Gy = match.ptG.get(2,1);
		iql.set(Gx.mid,uncertainty,1,1);
		iql.set(Gy.mid,uncertainty,2,1);
		iql.set(1,0,3,1);
		
		Gx = match.ptD.get(1,1);
		Gy = match.ptD.get(2,1);
		iqr.set(Gx.mid,uncertainty,1,1);
		iqr.set(Gy.mid,uncertainty,2,1);
		iqr.set(1,0,3,1);	
		//m_CamL.print("CamL");
		//		m_CamR.print("CamR");
		/*printf("iReconstruction3D : %f %f %f %f\n",
		       iqr.get(1,1), iqr.get(2,1),
		       match.ptD.get(1,1).mid, match.ptD.get(2,1).mid); */
		//Xnh = iReconstruction3D(iql, iqr, m_CamL, m_CamR,Contractor,nb_iterations)*(-1);       	
		Xnh = iReconstruction3D1(iql, iqr, m_CamL, m_CamR)*(-1);       	
		//		Xnh.print("Xnh");
	    }

	  /* Store the center 
	   * The disparity and the reconstructed point are done 
	   * in the image coordinates reference frame, thus we have to
	   * perform a rigid transformation to get back to the
	   * VVV world coordinates.
	   */
#if 0
	  for (int k = 0; k< 3; k++)
	    {
	      
	      m_BoundingBox[6*i+k] = m_w2c[0][k][0] *(float)Xnh.get(1,1).mid +
		m_w2c[0][k][1] *(float)Xnh.get(2,1).mid +
		m_w2c[0][k][2] *(float)Xnh.get(3,1).mid + m_w2c[0][k][3];
	    }
#else
	  m_BoundingBox[6*i+0] = (float)Xnh.get(1,1).mid;
	  m_BoundingBox[6*i+1] = (float)Xnh.get(2,1).mid;
	  m_BoundingBox[6*i+2] = (float)Xnh.get(3,1).mid;
	
#endif
	  /* Store the bouding box related to this center */
	  m_BoundingBox[6*i+3] = (float)Xnh.get(1,1).rad;
	  m_BoundingBox[6*i+4] = (float)Xnh.get(2,1).rad;
	  m_BoundingBox[6*i+5] = (float)Xnh.get(3,1).rad;
	  ODEBUG( m_BoundingBox[6*i+0] << " "
		   << m_BoundingBox[6*i+1] << " "
		   << m_BoundingBox[6*i+2] << " "
		   << m_BoundingBox[6*i+3] << " "
		   << m_BoundingBox[6*i+4] << " "
		   << m_BoundingBox[6*i+5]);
	  break;

	case ERROR_MODEL_LUT_INTERVALS:

	  /* Takes the information from the LUT */
	  aDIP = m_DIPs[HRP2DPINDEX(m_PointsInImageLeft[2*i],
				   m_PointsInImageLeft[2*i+1],
				   m_PointsInImageRight[2*i])];

	  if (aDIP!=0)
	    {
	      /* Store the center */
	      m_BoundingBox[6*i+0] = aDIP->Center[0];
	      m_BoundingBox[6*i+1] = aDIP->Center[1];
	      m_BoundingBox[6*i+2] = aDIP->Center[2];

	      /* Store the bouding box related to the center */
	      m_BoundingBox[6*i+3] = aDIP->Radius[0];
	      m_BoundingBox[6*i+4] = aDIP->Radius[1];
	      m_BoundingBox[6*i+5] = aDIP->Radius[2];
	    }
	  break;

	case ERROR_MODEL_HIRSCHMULLER:
	  break;
	  
	}

      if ((m_ModeToDumpMatching3DPoints==DUMP_MODE_3D_POINTS_INTERVALS) ||
	  (m_ModeToDumpMatching3DPoints==DUMP_MODE_3D_POINTS_INTERVALS_MATCHING))
	{
	  for(int j=0;j<6;j++)
	    aofstream << m_BoundingBox[6*i+j] << " ";
	  
	  for(int j=0;j<3;j++)
	    aofstream << (int)m_ColorInImageLeft[3*i+j] << " ";

	  aofstream << endl;
	}

      if (m_Verbosity>=3)
	if (i%10==0)
	  cout << i << endl;
    } 

  if ((m_ModeToDumpMatching3DPoints==DUMP_MODE_3D_POINTS_INTERVALS) ||
      (m_ModeToDumpMatching3DPoints==DUMP_MODE_3D_POINTS_INTERVALS_MATCHING))

    aofstream.close();
  return 0;
}

int HRP2DisparityProcess::ReadLookUpTable()
{
  string StereoSensor("LUT.dat");
  /* Read the camera model */
  ifstream a2is;

  a2is.open(StereoSensor.c_str(),ios::binary);
  if (a2is.is_open())
    {
      int NbOfPoints = 0;
      while (!a2is.eof())
	{
	  int CoordLeft[2], CoordRight[2];
	  float LocalCenter[3],LocalRadius[3];
	  
	  a2is.read((char *)CoordLeft,2*sizeof(int));
	  a2is.read((char *)CoordRight,2*sizeof(int));

	  if ((CoordLeft[0] >=0) && (CoordLeft[0] <80) &&
	      (CoordLeft[1] >=0) && (CoordLeft[1] <60) &&
	      (CoordRight[0]>=0) && (CoordRight[0]<80) &&
	      (CoordRight[1]>=0) && (CoordRight[1]<60))
	    {
	      m_DIPs[HRP2DPINDEX(CoordLeft[0],CoordLeft[1],CoordRight[0])] =
		new DataIntervalPoint_t; 
	      if (m_DIPs[HRP2DPINDEX(CoordLeft[0],CoordLeft[1],CoordRight[0])]==0)
		{
		  printf("Stop\n");
		  exit(0);
		}

	      if ((NbOfPoints%50==0) && m_Verbosity)
		{
		  printf("NbOfPoints: %d %d %d %d %d %p \n",
			 NbOfPoints,
			 NbOfPoints*sizeof(DataIntervalPoint_t),
			 CoordLeft[0],CoordLeft[1],CoordRight[0],
			 m_DIPs[HRP2DPINDEX(CoordLeft[0],CoordLeft[1],CoordRight[0])]);
		}
	      NbOfPoints++;	      

	      a2is.read((char *)m_DIPs[HRP2DPINDEX(CoordLeft[0],CoordLeft[1],CoordRight[0])]->Center,
		      3*sizeof(float));
	      a2is.read((char *)m_DIPs[HRP2DPINDEX(CoordLeft[0],CoordLeft[1],CoordRight[0])]->Radius,
		      3*sizeof(float));
	    }
	}

      a2is.close();
    }
  return 0;
}


int HRP2DisparityProcess::SetImageSizeInLUT(int width, int height)
{
  int TotalSize = width*height*width;
  m_ImageSizeInLUT[0] = width;
  m_ImageSizeInLUT[1] = height;
  


  return 0;
}

int HRP2DisparityProcess::GetImageSizeInLUT(int & width, int &height)
{
  width = m_ImageSizeInLUT[0];
  height = m_ImageSizeInLUT[1];
  return 0;
}



int HRP2DisparityProcess::GetCalibrationSize(int &width, int &height)
{
  width = m_para.calibration_image_width;
  height = m_para.calibration_image_height;
  return 0;
}

int HRP2DisparityProcess::SetCalibrationSize(int width, int height)
{
  m_para.calibration_image_width = width;
  m_para.calibration_image_height = height;
  return 0;
}

int HRP2DisparityProcess::UpdateAMotionEvaluationProcess(HRP2MotionEvaluationProcess *aMEP)
{
  if (aMEP!=0)
    {
      aMEP->UpdateRangeData(m_rng);
    }
  return 0;
}

RANGE  HRP2DisparityProcess::GetRangeData()
{
  return m_rng;
}

void HRP2DisparityProcess::WriteToPlyFormat(void)
{
  /* Dump to the appropriate range of data. */
  string PtsFileName="RangeData";
  string BoundingBoxFileName= "BoudingBox";
  static int localcounter = 0;
  char Buffer[50];
  ofstream fout1;
  int NbOfPoints=0;
  float BoundingBoxes[3][2]= { {1000000.0, -1000000.0},
			       {1000000.0, -1000000.0},
			       {1000000.0, -1000000.0} };

  if (m_Verbosity>=2)
    cerr << "HRP2DisparityProcess::WriteToPlyFormat : " <<localcounter << endl;

  sprintf(Buffer,"%06d",localcounter);

  PtsFileName+=Buffer;
  PtsFileName+=".ply";

  float TranslationVec[3] = {0.0,0.0,0.0}, LinearCoef=1000.0;

  /* Vertex in the range map */  
  for(int i=0;i<m_rng.PixelCount;i++)
    {
      PixelData *pmap;
      pmap = (m_rng.PixelList)+i;
      if (pmap!=0)
	{
	  //	  if ((pmap->Color[0]<128) && (pmap->Dot[2]<-2.0))
	    {
	      
	      for(int j= 0;j<3;j++)
		{
		  /* Min and Max on X, Y and Z */
		  if (pmap->Dot[j]<BoundingBoxes[j][0])
		    BoundingBoxes[j][0] = pmap->Dot[j];
		  if (pmap->Dot[j]>BoundingBoxes[j][1])
		    BoundingBoxes[j][1] = pmap->Dot[j];
		}
	      NbOfPoints++;
	    }
	}
    }
  
  fout1.open(PtsFileName.c_str());
  fout1 << "ply" << endl;
  fout1 << "format ascii 1.0" << endl;
  fout1 << "obj_info num_cols " << m_rng.Width << endl;
  fout1 << "obj_info num_rows " << m_rng.Height << endl;
  fout1 << "element vertex " << NbOfPoints << endl;
  fout1 << "property float x" << endl;
  fout1 << "property float y" << endl;
  fout1 << "property float z" << endl;
  fout1 << "element range_grid " << m_rng.Width * m_rng.Height << endl;
  fout1 << "property list uchar int vertex_indices" << endl;
  fout1 << "end_header" << endl;


  /* Computes translation and linear coefficient */
#if 0
  /*  X min: -0.09475 X max: 0.061
  Y min: 0.0357363 Y max: 0.18794
  Z min: -0.0586982 Z max: 0.0587228 */

  for(int i=0;i<3;i++)
    {
      float r = BoundingBoxes[i][1] - BoundingBoxes[i][0];
      TranslationVec[i] =  BoundingBoxes[i][0];
      
      if (LinearCoef < r)
	LinearCoef = r;
    }

  
  LinearCoef *= 1/0.06;
#endif

  /* Vertex in the range map */
  for(int i=0;i<m_rng.PixelCount;i++)
    {
      PixelData *pmap;
      pmap = (m_rng.PixelList)+i;
      if (pmap!=0)
	{
	  if (pmap->Color[0]<128)
	    {
	      for(int j=0;j<3;j++)
		{
		  float x = pmap->Dot[j];
		  
		  x -= TranslationVec[j];
		  x /= LinearCoef;
		  
		  fout1 << x << " ";
		  
		}
	      fout1 << endl;
	    }
	}
    }
  
  /* Goes through the range map 
   * and create the link between the matrix and the range map.
   */
  int IndexPoint = 0;
  for(int j=0;j<m_rng.Height;j++)
    {
      for(int i=0;i<m_rng.Width;i++)
	{
	  PixelData *pmap;
	  pmap = (m_rng.Map)[j*m_rng.Width+i];
	  if (pmap!=0)
	    {
	      if (pmap->Color[0]<128)
		{
		  
		  fout1 << "1 "
			<< IndexPoint++;
		}
	       else fout1<<0;
	    }
	  else fout1 << 0;
	  fout1 << endl;
	}
    }
  
  
  fout1.close();

  /* Create the bounding box file */
  BoundingBoxFileName+=Buffer;
  BoundingBoxFileName+=".ply";
  fout1.open(BoundingBoxFileName.c_str());
  fout1 << "ply" <<endl;
  fout1 << "format ascii 1.0" <<endl;
  fout1 << "element vertex 6" <<endl;
  fout1 << "property float x" << endl;
  fout1 << "property float y" << endl;
  fout1 << "property float z" << endl;
  fout1 << "element faces 0" << endl;
  fout1 << "property list uchar int vertex_indices" << endl;
  fout1 << "end_header" << endl;
  
  fout1 << (BoundingBoxes[0][0]-TranslationVec[0])/LinearCoef << " 0.0 0.0" << endl;
  fout1 << (BoundingBoxes[0][1]-TranslationVec[0])/LinearCoef << " 0.0 0.0" << endl;
  fout1 << "0.0 " << (BoundingBoxes[1][0]-TranslationVec[1])/LinearCoef << " 0.0" << endl;
  fout1 << "0.0 " << (BoundingBoxes[1][1]-TranslationVec[1])/LinearCoef << " 0.0" << endl;

  fout1 << "0.0 0.0 " << (BoundingBoxes[2][0]-TranslationVec[2])/LinearCoef  << endl;
  fout1 << "0.0 0.0 " << (BoundingBoxes[2][1]-TranslationVec[2])/LinearCoef  << endl;

  fout1.close();
  localcounter++;
}

int HRP2DisparityProcess::SetParameter(string aParameter, string aValue)
{
  int r;

  r = HRP2VisionBasicProcess::SetParameter(aParameter, aValue);

  ODEBUG("Parameter : " << aParameter << " Value : " << aValue << " " << m_ParametersSize);
  int argc = 1;
  char **argv, **argv_store;
  char ProgramName[124]="Disparity";

  for(int i=0;i<m_ParametersSize;i++)
    {
      if (m_VectorOfParameters[i]=="dump_mode")
	{
	}
      else if (m_VectorOfParameters[i]=="error_model")
	{
	}
      else
	{
	  if (m_VectorOfParameters[i]=="-Tmask")
	    {
	      /* Initialization of the texture less masking */
	      if (!m_TexturelessMasking)
		{
		  int largc = 1;
		  char *largv[1];
		  largv[0] = ProgramName;
		  ODEBUG("Initialization of the TexturelessImage ");
		  for(int j=0;j<3;j++)
		    {
		      m_TexturelessImage[j] = epbm_create( EPBM_BINARY_GRAY, m_InputImage[0].Width, m_InputImage[0].Height,
							   EPBM_UNSIGNED, EPBM_CHAR8, NULL );
		    }
		  m_InitParForIsoL.nimages = 2;
		  epbm_init_isoluminance(&m_InitParForIsoL, m_InputImage, largc, largv);
		}
	      m_TexturelessMasking = true;
	    }
	  ODEBUG( m_VectorOfParameters[i] << " " << m_VectorOfValuesForParameters[i] << " ");
	  if ( m_VectorOfParameters[i].length()!=0)
	    argc++;
	  if (m_VectorOfValuesForParameters[i].length()!=0)
	    argc++;
	}
    }
  
  argv = (char **)new char *[argc];
  argv_store = (char **)new char *[argc];

  // First argument.
  int n=0;
  argv[n]= new char[strlen(ProgramName)+1];
  bzero(argv[n],strlen(ProgramName)+1);
  strcpy(argv[n],ProgramName);
  n++;

  for(int i=0;i<m_ParametersSize;i++)
    {
      unsigned char dump_mode = 0;
      unsigned char lerror_model =0;

      if (m_VectorOfParameters[i].length()!=0)
	{
	  if (m_VectorOfParameters[i]=="dump_mode")
	    {
	      dump_mode = 1;
	    }
	  else if (m_VectorOfParameters[i]=="error_model")
	    {
	      lerror_model =1;
	    }
	  else
	    {
	      ODEBUG(  "VP " << m_VectorOfParameters[i].length() << " n: " << n << " "<<  m_VectorOfParameters[i]);
	      argv[n] = new char[m_VectorOfParameters[i].length()+1];
	      bzero(argv[n],m_VectorOfParameters[i].length()+1);
	      strcpy(argv[n],m_VectorOfParameters[i].c_str());
	      n++;
	    }
	}
      if (m_VectorOfValuesForParameters[i].length()!=0)
	{
	  if (dump_mode)
	    {
	      if (m_VectorOfValuesForParameters[i]=="off")
		m_ModeToDumpMatching3DPoints = DUMP_MODE_3D_POINTS_OFF;
	      else if (m_VectorOfValuesForParameters[i]=="matching")
		m_ModeToDumpMatching3DPoints = DUMP_MODE_3D_POINTS_MATCHING;
	      else if (m_VectorOfValuesForParameters[i]=="ply")
		m_ModeToDumpMatching3DPoints = DUMP_MODE_3D_POINTS_PLY;
	      else if (m_VectorOfValuesForParameters[i]=="rangedata")
		m_ModeToDumpMatching3DPoints = DUMP_MODE_3D_RANGE_DATA;
	      else if (m_VectorOfValuesForParameters[i]=="rd_ply")
		m_ModeToDumpMatching3DPoints = DUMP_MODE_3D_RANGE_DATA_AND_PLY;
	      else if (m_VectorOfValuesForParameters[i]=="pts_intervals")
		m_ModeToDumpMatching3DPoints = DUMP_MODE_3D_POINTS_INTERVALS;
	      else if (m_VectorOfValuesForParameters[i]=="pts_intervals_matching")
		m_ModeToDumpMatching3DPoints = DUMP_MODE_3D_POINTS_INTERVALS_MATCHING;
	      if (m_Verbosity>=2)
		ODEBUG3( "dump_mode " << m_ModeToDumpMatching3DPoints << " " << m_VectorOfValuesForParameters[i]);
	    }
	  else if (lerror_model)
	    {
	      m_UseErrorModel = 1;
	      if (m_VectorOfValuesForParameters[i]=="intervals")
		{
		  m_ErrorModel = ERROR_MODEL_ONLINE_INTERVALS;
		}
	      else if (m_VectorOfValuesForParameters[i]=="hz")
		{
		  m_ErrorModel = ERROR_MODEL_HZ;
		}	
	      InitializeErrorModel();
	    }
	  else
	    {
	      ODEBUG( "VP_value " << m_VectorOfParameters[i].length() << " n: " << n 
		       << " " << m_VectorOfValuesForParameters[i]) ;
	      argv[n] = new char[m_VectorOfValuesForParameters[i].length()+1];
	      bzero(argv[n],m_VectorOfValuesForParameters[i].length()+1);
	      strcpy(argv[n],m_VectorOfValuesForParameters[i].c_str());
	      n++;
	    }
	}


    }

  ODEBUG("Insider Disparity Process Arguments " );
  for(int i=0;i<argc;i++)
    {
      ODEBUG(argv[i]);
    }
  ODEBUG("/***********************************/");

  for(int i=0;i<argc;i++)
    {
      argv_store[i] = argv[i];
    }
  
  /* Because this is a F... VVV unsafe function we have to store 
     the pointers for further deleting. */
  ODEBUG("SET PARAMETER OF LIBRARY");
  setParameter(&m_para,argc,argv);


  for(int i=0;i<argc;i++)
    delete argv_store[i];
  delete argv;
  delete argv_store;

  return r;

}

int HRP2DisparityProcess::SetErrorModel(int anErrorModel)
{
  if ((anErrorModel>=0) &&
      (anErrorModel<3))
    {
      m_ErrorModel = anErrorModel;
      return 0;
    }
  return -1;
}

int HRP2DisparityProcess::GetErrorModel(void)
{
  return m_ErrorModel;
}

