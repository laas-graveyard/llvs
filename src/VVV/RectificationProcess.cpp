#include <math.h>
#include <iostream>

using namespace std;

extern "C"
{
#include <calib.h>
}
#ifdef __ORBIX__
#include <OBE/CORBA.h>
#include <OBE/CosNaming.h>
#endif

#ifdef OMNIORB4
#include <omniORB4/CORBA.h>
#endif

#include "RectificationProcess.h"

// Debug macros
#include "Debug.h"

HRP2RectificationProcess::HRP2RectificationProcess()
{
  m_ProcessName = "Rectification (VVV)";
  m_sp = 0;
  m_Computing = 0;
}

HRP2RectificationProcess::~HRP2RectificationProcess()
{

}

void HRP2RectificationProcess::SetInputImages(EPBM InputImages[3])
{
  int i;
  
  for(i=0;i<3;i++)
    m_InputImages[i] = InputImages[i];
  
}

void HRP2RectificationProcess::SetOutputImages(EPBM OutputImages[3])
{
  int i;
  
  for(i=0;i<3;i++)
    m_OutputImages[i] = OutputImages[i];
}


int HRP2RectificationProcess::SetCalibrationSize(CORBA::Long CalibrationWidth[3], CORBA::Long CalibrationHeight[3])
{
  for(int i=0;i<3;i++)
    {
      m_CalibrationWidth[i] = CalibrationWidth[i];
      m_CalibrationHeight[i] = CalibrationHeight[i];
    }
  return 0;
}

int HRP2RectificationProcess::SetSP(SCM_PARAMETER *asp)
{
  m_sp = asp;
  return 0;
}


CORBA::Long
HRP2RectificationProcess::scm_ConvertImageLocal(CONST SCM_PARAMETER *sp, CONST EPBM *I, EPBM *O,
						int OriginalWidth,int OriginalHeight)
{
  double scr[2], cr[2];
  double frow, fcol;
  int scm_row, scm_col, nrow, ncol;
  int i, j, chkl, chkr, lcv= SCM_VERIFY;
  double fromOrig2CurX,fromOrig2CurY;
  double fromCur2OrigX,fromCur2OrigY;
  double ImageNR[2];

  

  for (chkl = 0, chkr = 0, i = 0; i < 3; i++) {
    for (j = 0; j < 4; j++) {
      if (O->PinHoleParameter->H[i][j] == sp->H[0][i][j])
	chkl++;
      if (O->PinHoleParameter->H[i][j] == sp->H[1][i][j])
	chkr++;
    }
  }
  if (chkl == 12)
    {
      lcv = SCM_LEFT;
    }
  else if (chkr == 12)
    {
      lcv = SCM_RIGHT;
    }
  else {
    fprintf(stderr,"scm_ConvertImage: EPBM DATA IS NOT GOOD");
    return -1;
  }
  
  /* From currently corrected image to original corrected image. */
  fromCur2OrigX = (double)OriginalWidth/(double)I->Width;
  fromCur2OrigY = (double)OriginalHeight/(double)I->Height;

  /* Compute the "virtual" size of the original none corrected image */
  scr[0] = (double)OriginalWidth;
  scr[1] = (double)OriginalHeight;
  
  scm_SCMcr2cr(sp, lcv, scr, ImageNR);

  /* From original none corrected image to currently non corrected image */
  fromOrig2CurX = (double)I->Width/(double)OriginalWidth;
  fromOrig2CurY = (double)I->Height/(double)OriginalHeight;

#ifdef OBDEBUG
  if (lcv==SCM_LEFT)
    fprintf(stderr,"scm_ConvertImage: SCM_LEFT\n");
  else if (lcv==SCM_RIGHT)
    fprintf(stderr,"scm_ConvertImage: SCM_RIGHT\n");
#endif

  if (I->Magic2 == EPBM_BINARY_GRAY) {
    for (scm_row = 0; scm_row < O->Height; scm_row++) {
      for (scm_col = 0; scm_col < O->Width; scm_col++) {
	
	/*	fprintf(stderr,"(%d %d %d %d)\n",
		OriginalWidth,OriginalHeight,
		I->Width,I->Height);

	fprintf(stderr,"{%f %f %f %f}\n",
		fromCur2OrigX,fromCur2OrigY,
		fromOrig2CurX,fromOrig2CurY); 
	*/	
	scr[0] = fromCur2OrigX*(double)scm_col;
	scr[1] = fromCur2OrigY*(double)scm_row;
	scm_SCMcr2cr(sp, lcv, scr, cr);
	nrow = (int)floor(fromOrig2CurY*cr[1]);
	frow = fromOrig2CurY*cr[1] - (double)nrow;
	ncol = (int)floor(fromOrig2CurX*cr[0]);
	fcol = fromOrig2CurX*cr[0] - (double)ncol;
	
	
	if ((nrow < 0) || (I->Height - 1 <= nrow) ||
	    (ncol < 0) || (I->Width - 1 <= ncol))
	  epbm_uc_getpixel(O, scm_row, scm_col) = 0;
	else
	  {
	    epbm_uc_getpixel(O, scm_row, scm_col) =
	      (unsigned char)
	      (epbm_uc_getpixel(I, nrow, ncol) * (1.0 - frow) * (1.0 - fcol) +
	       epbm_uc_getpixel(I, nrow + 1, ncol) * frow * (1.0 - fcol) +
	       epbm_uc_getpixel(I, nrow, ncol + 1) * (1.0 - frow) * fcol +
	       epbm_uc_getpixel(I, nrow + 1, ncol + 1) * frow * fcol +
	       0.5);
	    /*
	    fprintf(stderr,"%d %d %f %f %f %f %d %d %f %f %f %f %d %d\n", 
		    scm_col,scm_row,scr[0],scr[1],cr[0],cr[1],
		    ncol, nrow, ImageNR[0], ImageNR[1], fromOrig2CurX, fromOrig2CurY, OriginalWidth, OriginalHeight); 
	    */
		
	  }
      }
    }
  }
  else if (I->Magic2 == EPBM_BINARY_COLOR) {
    for (scm_row = 0; scm_row < O->Height; scm_row++) {
      for (scm_col = 0; scm_col < O->Width; scm_col++) {
	scr[0] = fromCur2OrigX*(double)scm_col;
	scr[1] = fromCur2OrigY*(double)scm_row;
	scm_SCMcr2cr(sp, lcv, scr, cr);
	nrow = (int)floor(fromOrig2CurY*cr[1]);
	frow = fromOrig2CurY*cr[1] - (double)nrow;
	ncol = (int)floor(fromOrig2CurX*cr[0]);
	fcol = fromOrig2CurX*cr[0] - (double)ncol;
	if ((nrow < 0) || (I->Height - 1 <= nrow) ||
	    (ncol < 0) || (I->Width - 1 <= ncol))
	  epbm_uc_cgetpixel(O, EPBM_RED, scm_row, scm_col) =
	    epbm_uc_cgetpixel(O, EPBM_GREEN, scm_row, scm_col) =
	    epbm_uc_cgetpixel(O, EPBM_BLUE, scm_row, scm_col) = 0;
	else {
	  epbm_uc_cgetpixel(O, EPBM_RED, scm_row, scm_col) =
	    (unsigned char)
	    (epbm_uc_cgetpixel(I, EPBM_RED, nrow, ncol) *
	     (1.0 - frow) * (1.0 - fcol) +
	     epbm_uc_cgetpixel(I, EPBM_RED, nrow + 1, ncol) *
	     frow * (1.0 - fcol) +
	     epbm_uc_cgetpixel(I, EPBM_RED, nrow, ncol + 1) *
	     (1.0 - frow) * fcol +
	     epbm_uc_cgetpixel(I, EPBM_RED, nrow + 1, ncol + 1) *
	     frow * fcol +
	     0.5);
	  epbm_uc_cgetpixel(O, EPBM_GREEN, scm_row, scm_col) =
	    (unsigned char)
	    (epbm_uc_cgetpixel(I, EPBM_GREEN, nrow, ncol) *
	     (1.0 - frow) * (1.0 - fcol) +
	     epbm_uc_cgetpixel(I, EPBM_GREEN, nrow + 1, ncol) *
	     frow * (1.0 - fcol) +
	     epbm_uc_cgetpixel(I, EPBM_GREEN, nrow, ncol + 1) *
	     (1.0 - frow) * fcol +
	     epbm_uc_cgetpixel(I, EPBM_GREEN, nrow + 1, ncol + 1) *
	     frow * fcol +
	     0.5);
	  epbm_uc_cgetpixel(O, EPBM_BLUE, scm_row, scm_col) =
	    (unsigned char)
	    (epbm_uc_cgetpixel(I, EPBM_BLUE, nrow, ncol) *
	     (1.0 - frow) * (1.0 - fcol) +
	     epbm_uc_cgetpixel(I, EPBM_BLUE, nrow + 1, ncol) *
	     frow * (1.0 - fcol) +
	     epbm_uc_cgetpixel(I, EPBM_BLUE, nrow, ncol + 1) *
	     (1.0 - frow) * fcol +
	     epbm_uc_cgetpixel(I, EPBM_BLUE, nrow + 1, ncol + 1) *
	     frow * fcol +
	     0.5);
	}
      }
    }
  }

  return 0;
}

int HRP2RectificationProcess::RealizeTheProcess()
{     

  ODEBUG3( m_Computing << " " << m_sp );
  if (m_Computing==0)
    return 0;

  int i;
  int n =2 ;
  //int op_zoom = SCM_PINHOLE_F_NULL;
  int op_zoom = 0;
  /* 画像枚数が２、３枚以外のとき */
  if (n != 2 && n != 3) 
    {
      return -1;
    }
  
  if (m_sp==0)
    return -2;


  static unsigned char first_time = 1;

  if (first_time==1)
    {
      calib_check_epbm(&m_InputImages[0], &m_InputImages[1], 0);
      
      for(i=0;i<2;i++)
	{
	  m_OutputImages[i].Width = m_CalibrationWidth[i];
	  m_OutputImages[i].Height = m_CalibrationHeight[i];
	}
      scm_Init(&m_OutputImages[0], &m_OutputImages[1], (n == 2) ? NULL : &m_OutputImages[2], op_zoom, m_sp, 0);
      for(i=0;i<2;i++)
	{
	  m_OutputImages[i].Width = m_InputImages[i].Width;
	  m_OutputImages[i].Height = m_InputImages[i].Height;
	}
      first_time = 0;
    }



  /* L,Rの画像が同一のものではないかすでに変換されている */

  for (i = 0; i < n; i++) 
    {
      char FileName[256];
      scm_ConvertImageLocal(m_sp, &m_InputImages[i], &m_OutputImages[i], m_CalibrationWidth[i],m_CalibrationHeight[i]);
      //scm_ConvertImage(&m_sp, &m_InputImages[i], &m_OutputImages[i],0);
#if 0
      sprintf(FileName,"/tmp/checkm_OutputImage_%03d.epbm",i);
      epbm_save(FileName,&m_OutputImages[i],0);
      sprintf(FileName,"/tmp/checkI_%03d.epbm",i);
      epbm_save(FileName,&m_InputImages[i],0);
#endif

      m_OutputImages[i].Label |= EPBM_CONVERTED_SCM_MASK;
      m_OutputImages[i].PinHoleParameter->f = m_sp->f;
    }
  
  return 0;
}
