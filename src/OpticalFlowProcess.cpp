/** @doc This object implements a visual process
    performing optical flow.

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
#include <libmmx.h>
#include "OpticalFlowProcess.h"
#include "LowLevelVisionServer.h"

// Debug macros
#include <llvs/tools/Debug.h>

using namespace std;

int HarrisAdmitPointBasedOnNeighbor(ImageOfTiles_t *anImageOfTiles, int x, int y)
{
  int m,n,w,l, tx, ty, criterion;
  int deptx, endtx, depty, endty;
  Tile_t *aTile;
  TilePoint_t *aTP;

  if ( anImageOfTiles==0)
    return -1;

  int   DistanceThreshold = anImageOfTiles->SeparationSpace*anImageOfTiles->SeparationSpace;
  w = anImageOfTiles->SeparationSpace/anImageOfTiles->TilingDownSize;

  /* Compute the change of coordinates */
  /* For x */
  tx = x/anImageOfTiles->TilingDownSize;

  /* Check the limits */
  deptx = tx - w < 0 ? 0 : tx -w;
  endtx = tx + w >= anImageOfTiles->Width ? anImageOfTiles->Width-1 : tx + w;

  /* For y */
  ty = y/anImageOfTiles->TilingDownSize;

  /* Check the limits */
  depty = ty - w < 0 ? 0 : ty -w;
  endty = ty + w >= anImageOfTiles->Height ? anImageOfTiles->Height-1 : ty + w;

  for(n=depty;n<=endty;n++)
    {
      for(m=deptx;m<=endtx;m++)
	{
	  aTile = &anImageOfTiles->Tiles[n*anImageOfTiles->Width + m];

	  for(l=0;l<aTile->CurrentNbOfPoints;l++)
	    {
	      aTP = &aTile->TilePoints[l];
	      criterion = (aTP->Coordinates[0]-x)*(aTP->Coordinates[0]-x)+
		(aTP->Coordinates[1]-y)*(aTP->Coordinates[1]-y);

	      if (criterion<=DistanceThreshold)
		return -1;
	    }
	}
    }
  return 0;
}

int HarrisFilterFeaturePoints(multiset <TilePoint_t *, TilePoint_lt>
			      * aSetOfPoint,
			      ImageOfTiles_t *anImageOfTiles,
			      float Threshold)
{

  int i,j,r=0;
  int k,l, index, NbOfPts=0;
  Tile_t *aTile;
  TilePoint_t *aTP;
  multiset <TilePoint_t *,TilePoint_lt>::iterator it_TP;
  int NbAcceptedPoints=0;

  it_TP = aSetOfPoint->begin();
  while(it_TP!=aSetOfPoint->end())
    {

      aTP = *it_TP;
      if (aTP!=0)
	{
	  if (aTP->Value>Threshold)
	    {
	      i = aTP->Coordinates[0];
	      j = aTP->Coordinates[1];
	      k = i/anImageOfTiles->TilingDownSize;
	      l = j/anImageOfTiles->TilingDownSize;

	      aTile = &anImageOfTiles->Tiles[l*anImageOfTiles->Width + k];
	      index = aTile->CurrentNbOfPoints;
	      if ( index < anImageOfTiles->MaxNbOfFeaturesByTile)
		{
		  if ((r=HarrisAdmitPointBasedOnNeighbor(anImageOfTiles,i,j))==0)
		    {
		      aTile->TilePoints[index].Coordinates[0] = i;
		      aTile->TilePoints[index].Coordinates[1] = j;
		      aTile->TilePoints[index].Value = aTP->Value;
		      aTile->CurrentNbOfPoints++;
		      NbAcceptedPoints++;
		      if (NbAcceptedPoints>anImageOfTiles->MaxNumberOfPoints)
			return 0;
		    }
		}
	    }
	}
      NbOfPts++;
      it_TP++;
    }

  return 0;
}

HRP2OpticalFlowProcess::HRP2OpticalFlowProcess(int Width, int Height) : HRP2VisionBasicProcess()
{
  m_Width = Width;
  m_Height = Height;
  m_ProcessName = "Optical Flow and Harris detector Process";
  m_ProcessOPFLeftCamera = 1;
  m_ProcessHarrisLeftCamera = 1 ;

  m_ProcessOPFRightCamera = 0;
  m_ProcessHarrisRightCamera = 0;

  m_MaxNbOfHarrisMatchingPoint = 40;

  m_eigImage = 0;
  m_tempImage = 0;

  m_IplInputImage = 0;
  m_DumpingLevel = 0;
  m_HarrisMethod = HARRIS_MMX;

  m_SizeOfTheImage = 8;
  m_NbOfPossiblePtPerTile = 1;
  m_SeparationSpace = 8;
  m_TotalNbOfPoints = 200;
  m_HarrisThreshold = 80.0;

  m_tau = 1.9;
  //  m_tau =0.2;
  m_Harris_k = 0.04;

  SetToZeroOpticalFlowStructures();
}

HRP2OpticalFlowProcess::~HRP2OpticalFlowProcess()
{
  FreeOpticalFlowStructures();
}

int HRP2OpticalFlowProcess::SetToZeroOpticalFlowStructures()
{
  int i;

  m_LEFT_OPF_m = 0;

  m_RIGHT_OPF_m = 0;

  MMXSetToZeroFlow(m_LEFT_Flow);
  MMXSetToZeroFlow(m_RIGHT_Flow);

  m_LEFT_Flow = 0;
  m_RIGHT_Flow = 0;

  MMXSetToZeroFlow(m_LEFT_Harris_Flow);
  MMXSetToZeroFlow(m_RIGHT_Harris_Flow);
  m_LEFT_Harris_Flow = 0;
  m_RIGHT_Harris_Flow = 0;

  m_LEFT_PreviousImage = 0;
  m_LEFT_Harris_PreviousImage = 0;

  for(int i=0;i<2;i++)
    m_ImageOfTiles[i] = 0;

  m_PreviousImageOfTiles = 0;
  return 0;

}

int HRP2OpticalFlowProcess::SaveLastImages(void)
{
  int i=0,j=0;
  MM_F_32 *p_LeftHarrisImage, *p_RightHarrisImage;
  float max=0.0;

  for(i=0;i<m_InputImages[0].Height;i++)
    {
      for(j=0;j<m_InputImages[0].Width;j++)
	{
	  ((MM_F_32 *)m_LEFT_PreviousImage->p[i].s)[j] = ((MM_F_32 *)m_LEFT_OPF_m->p[i].s)[j];
	}
    }

  for(i=0;i<m_LEFT_Flow->Harris->rows;i++)
    {

      p_LeftHarrisImage = (MM_F_32 *)m_LEFT_Harris_Flow->Harris->p[i].s;
      p_RightHarrisImage = (MM_F_32 *)m_LEFT_Harris_PreviousImage->p[i].s;

      for(j=0;j<m_LEFT_Flow->Harris->cols;j++)
	  {
	    float tmp;
	    ((MM_F_32 *)m_LEFT_Harris_PreviousImage->p[i].s)[j] =
	      ((MM_F_32 *)m_LEFT_Flow->Harris->p[i].s)[j];

	    tmp = p_LeftHarrisImage[j];

	    if (max<tmp)
	      max = tmp;

	  }
    }

  if (m_DumpingLevel>=2)
  {
    static unsigned int counter = 0;
    char Buffer[1024];
    FILE *fp=0,*fp2=0;
    bzero(Buffer,1024);
    sprintf(Buffer,"/tmp/OPF_LEFTImage_%06d.pgm",counter);

    fp = fopen(Buffer,"w");


    bzero(Buffer,1024);
    sprintf(Buffer,"/tmp/OPF_PRECLEFTImage_%06d.pgm",counter);

    fp2 = fopen(Buffer,"w");
    fprintf(fp,"P5\n%d %d\n%d\n",
	    m_LEFT_Flow->Harris->cols,
	    m_LEFT_Flow->Harris->rows,
	    255);

    fprintf(fp2,"P5\n%d %d\n%d\n",
	    m_LEFT_Harris_PreviousImage->cols,
	    m_LEFT_Harris_PreviousImage->rows,
	    255);

    for(i=0;i<m_LEFT_Flow->Harris->rows;i++)
      {
	p_LeftHarrisImage = (MM_F_32 *)m_LEFT_Harris_Flow->Harris->p[i].s;
	p_RightHarrisImage = (MM_F_32 *)m_LEFT_Harris_PreviousImage->p[i].s;

	for(j=0;j<m_LEFT_Flow->Harris->cols;j++)
	  {
	    float x;
	    /*	    fprintf(stderr,"%d %d : %f\n",j,i,
		    p_LeftHarrisImage[j]);  */
	    x = p_LeftHarrisImage[j]+0.0;
	    x = x < 0 ? 0.0 : x;
	    fprintf(fp,"%c",(unsigned char)( x < 255.0 ? x :255.0));
	    fprintf(fp2,"%c",(unsigned char)((p_RightHarrisImage[j]<40000000.0 ? 0.0 :
					      p_RightHarrisImage[j]) < 50000000.0 ?
					     p_RightHarrisImage[j]*255.0 : 255.0));

	  }
       }
    ODEBUG(" Max : "<< max);
    if (fp!=0)
      fclose(fp);

    if (fp2!=0)
      fclose(fp2);
    counter++;
  }
  return 0;
}

int HRP2OpticalFlowProcess::ComputesOpticalFlowAndHarris()
{

  if (m_ProcessOPFLeftCamera)
    {
      MMXLucas(m_LEFT_OPF_m,m_LEFT_Flow);
      if (m_ProcessHarrisLeftCamera)
	{
	  m_LEFT_Harris_Flow->tau = m_Harris_k;
	  MMXHarris(m_LEFT_OPF_m,m_LEFT_Harris_Flow);
  	  FilteringHarrisPoint(m_LEFT_Harris_Flow, m_ImageOfTiles[0]);

	  if (m_DumpingLevel>=2)
	    {
	      static unsigned int counter = 0;
	      char Buffer[1024];
	      float max =0.0;
	      FILE *fp=0;
	      MM_F_32 *p_LeftHarrisImage;
	      int j;

	      bzero(Buffer,1024);
	      sprintf(Buffer,"/tmp/HarrisImageLEFT_%06d.pgm",counter);

	      fp = fopen(Buffer,"w");

	      fprintf(fp,"P5\n%d %d\n%d\n",
		      m_LEFT_Harris_Flow->Harris->cols,
		      m_LEFT_Harris_Flow->Harris->rows,
		      255);
	      for(int i=0;i<m_LEFT_Harris_Flow->Harris->rows;i++)
		{
		  p_LeftHarrisImage = (MM_F_32 *)m_LEFT_Harris_Flow->Harris->p[i].s;

		  for(j=0;j<m_LEFT_Flow->Harris->cols;j++)
		    {
		      float x;
		      /*		      fprintf(stderr,"%d %d : %f\n",j,i,
					      p_LeftHarrisImage[j]);  */
		      x = (p_LeftHarrisImage[j] +128.0);
		      x = x < 0 ? 255.0 : x;
		      fprintf(fp,"%c",(unsigned char)( x < 255.0 ? x :255.0));
		    }
		}

	      if (fp!=0)
		fclose(fp);

	      counter++;
	    }

	}
    }

  if (m_ProcessOPFRightCamera)
    {
      MMXLucas(m_RIGHT_OPF_m,m_RIGHT_Flow);
      if (m_ProcessHarrisRightCamera)
	{
	  m_RIGHT_Harris_Flow->tau = m_Harris_k;
	  MMXHarris(m_RIGHT_OPF_m,m_RIGHT_Harris_Flow);
	  FilteringHarrisPoint(m_RIGHT_Harris_Flow,m_ImageOfTiles[1]);
	}
    }
  return 0;
}

int HRP2OpticalFlowProcess::CreateMatchingBetween2ConsequentViews(ImageOfTiles_t *anIOTAtT,
								  ImageOfTiles_t *anIOTAtTpdT,
								  MMXFlow *aFlow)
{
  if ((anIOTAtT==0) || (anIOTAtTpdT==0))
    return -1;

  int i=0,j=0,offset=0,k=0,l=0;
  int PossibleTarget[2];
  Tile_t *aTile;
  Tile_t *aCandidateTile;
  MMXMatrix *vY,*vX;
  MM_F_32 * vX_p, * vY_p;

  vX = aFlow->V[0];
  vY = aFlow->V[1];
  m_SetOfMatchingPoint.clear();

  for(j=0;j<anIOTAtT->Height;j++)
    {
      offset = j*anIOTAtT->Width;
      for(i=0;i<anIOTAtT->Width;i++)
	{
	  aTile = &anIOTAtT->Tiles[offset+i];

	  for(k=0;k<aTile->CurrentNbOfPoints;k++)
	    {
	      int locx, locy, Refx, Refy;
	      float Value;

	      /* Takes a Harris descriptor location */
	      locx = aTile->TilePoints[k].Coordinates[0];
	      locy = aTile->TilePoints[k].Coordinates[1];
	      Value = aTile->TilePoints[k].Value;

	      vX_p = (MM_F_32 *)vX->p[locy].s;
	      vY_p = (MM_F_32 *)vY->p[locy].s;

	      /* Add the optical flow */
	      Refx = locx + (int)vX_p[locx];
	      Refy = locy + (int)vY_p[locx];

	      if ((Refx >= 0) && (Refx<aFlow->Width) && (Refy>=0) && (Refy<aFlow->Height))
		{
		  /* Check if the new point can be match with an other Harris detector
		     in the current set of images. */
		  PossibleTarget[0] = Refx / anIOTAtTpdT->TilingDownSize;
		  PossibleTarget[1] = Refy / anIOTAtTpdT->TilingDownSize;

		  aCandidateTile = &anIOTAtTpdT->Tiles[PossibleTarget[1]*anIOTAtTpdT->Width + PossibleTarget[0]];


		  /* No candidate in the targeted tile */
		  if (aCandidateTile->CurrentNbOfPoints!=0)
		    {

		      for(l=0;l<aCandidateTile->CurrentNbOfPoints;l++)
			{

			  TilePoint_t *aTP = &aCandidateTile->TilePoints[l];
			  float ratio = aTP->Value/Value;

			  /* In the candidate tile and with a Harris value
			     similar to the origin. */
			  if ((ratio < 1.05) && (ratio>0.95))
			  {
			    MatchingPoint_t *aMT = new MatchingPoint_t;

			    /* Insert the matching */
			    aMT->LeftCoordinates[0] = locx +4;
			    aMT->LeftCoordinates[1] = locy +4;

			    aMT->RightCoordinates[0] = aTP->Coordinates[0] +4;
			    aMT->RightCoordinates[1] = aTP->Coordinates[1] +4;

			    aMT->Probability = 1.0;
			    m_SetOfMatchingPoint.insert(aMT);

			  }
			}
		    }
		}

	    }
	}
    }
  if (m_Verbosity>=2)
    fprintf(stderr,"Nb of matching points : %d %d %d %d\n",m_SetOfMatchingPoint.size(),i,j,k);
  return 0;
}

int HRP2OpticalFlowProcess::FilteringHarrisPoint(MMXFlow *aFlow, ImageOfTiles_t *anImageOfTiles)
{
  int i,j,k;
  float LocalThreshold = m_HarrisThreshold;
  TilePoint_t *aTP;
  MM_F_32 *p_HarrisImage;
  int localcounter = 0;

  for (i=0;i<anImageOfTiles->Width*anImageOfTiles->Height;i++)
    anImageOfTiles->Tiles[i].CurrentNbOfPoints = 0;

  /*! List of sorted Harris Points */
  multiset <TilePoint_t *,TilePoint_lt> SortedHarrisPoints;

  for(i=0;i<aFlow->Harris->rows;i++)
    {

      p_HarrisImage = (MM_F_32 *)aFlow->Harris->p[i].s;
      for(j=0;j<aFlow->Harris->cols;j++)
	{
	  if (p_HarrisImage[j]>LocalThreshold)
	    {
	      aTP = &anImageOfTiles->TilePointsCandidates[localcounter++];
	      aTP->Coordinates[0] = j;
	      aTP->Coordinates[1] = i;
	      aTP->Value = p_HarrisImage[j];
	      SortedHarrisPoints.insert(aTP);
	    }
	}
    }

  if (m_Verbosity>=2)
    fprintf(stderr,"Nb of Sorted Harris points: %d\n",
	    SortedHarrisPoints.size());

  HarrisFilterFeaturePoints(&SortedHarrisPoints,anImageOfTiles,LocalThreshold);

  CreateMatchingBetween2ConsequentViews(m_PreviousImageOfTiles, anImageOfTiles, aFlow);

  HarrisCopyImageOfTiles(anImageOfTiles, &m_PreviousImageOfTiles);
#if 0
  {
    CvSize ImgSize;
    ImgSize.width = aFlow->Harris->cols;
    ImgSize.height = aFlow->Harris->rows;
    IplImage *SrcImg = cvCreateImage(ImgSize,8,1);
    CvMemStorage *storage = cvCreateMemStorage(0);
    CvSeq *Lines = 0;
    int m,Twidth;
    Twidth = anImageOfTiles->Width;

    for(j=0;j<anImageOfTiles->Height;j++)
      {
	for(i=0;i<anImageOfTiles->Width;i++)
	  {

	    for(m = 0;m<anImageOfTiles->Tiles[Twidth*j+i].CurrentNbOfPoints;m++)
	      {
		cvSet(SrcImg, i,j,255);
	      }
	  }
      }
    Lines = cvHoughLines2(SrcImg, storage, CV_HOUGH_STANDARD,1, CV_PI/180.0, 150, 0, 0);


  }
#endif
  if (m_DumpingLevel>=1)
    {
      static unsigned int counter = 0;
      char Buffer[1024];
      bzero(Buffer,1024);
      if (anImageOfTiles==m_ImageOfTiles[0])
	sprintf(Buffer,"/tmp/HarrisPointsLEFT_%06d.ppm",counter);
      else
	sprintf(Buffer,"/tmp/HarrisPointsRIGHT_%06d.ppm",counter);

      FILE *fp=0;
      int lwidth, lheight, Twidth, Theight, MNFBT,l,m;

      fp = fopen(Buffer,"w");
      if (fp!=0)
	{
	  MM_F_32 * p_Image = 0;
	  lwidth = m_LEFT_OPF_m->cols;
	  lheight = m_LEFT_OPF_m->rows;
	  Twidth = anImageOfTiles->Width;
	  MNFBT = anImageOfTiles->MaxNbOfFeaturesByTile;
	  unsigned char DHarris = 0;

	  fprintf(fp,"P6\n%d %d\n255\n",
		  lwidth-8,
		  lheight-8);

	  for(j=4;j<lheight-4;j++)
	    {
	      if (!DHarris)
		p_Image = (MM_F_32 *)m_LEFT_OPF_m->p[j].s;
	      else
		p_Image = (MM_F_32 *)aFlow->Harris->p[j-4].s;

	      for(i=4;i<lwidth-4;i++)
		{
		  unsigned char IsHarrisPt,R,G,B;
		  IsHarrisPt = 0;
		  R = B= 0; G = 255;
		  k = (i-4) / anImageOfTiles->TilingDownSize;
		  l = (j-4) / anImageOfTiles->TilingDownSize;

		  for(m = 0;m<anImageOfTiles->Tiles[Twidth*l+k].CurrentNbOfPoints;m++)
		    {
		      if ((anImageOfTiles->Tiles[Twidth*l+k].TilePoints[m].Coordinates[0]+4 == i) &&
			  (anImageOfTiles->Tiles[Twidth*l+k].TilePoints[m].Coordinates[1]+4 == j))
			{
			  IsHarrisPt = 1;
			  break;
			}
		    }

		  if (!IsHarrisPt)
		    {
		      if (!DHarris)
			{
			  unsigned char tmp;
			  tmp = (unsigned char)(p_Image[i]*255.0);
			  if (tmp<200)
			    if (tmp>55)
			      { R=0;G=0;B =0;}
			    else
			      { R= 255;G=B=0;}
			  else
			    { R=G=B=250;}


			}
		      else
			{
			  float x;
			  x = p_Image[i-4];
			  x = x >255.0 ? 255 : x;
			  x = x < 0.0 ? 0.0 : x;
			  R = G = B = (unsigned char)x;
			}

		      if ((i%m_ImageOfTiles[0]->TilingDownSize==0) ||
			  (j%m_ImageOfTiles[0]->TilingDownSize==0))
			{
			  R = B = 0;
			  G = 0;
			}

		    }
		  fprintf(fp,"%c%c%c",R,G,B);
		}
	    }
	  fclose(fp);
	}
      counter ++;
    }

  return 0;
}

int HRP2OpticalFlowProcess::ComputesHarrisMatchingPoints()
{
  int r=0;
  bzero(m_HarrisMatchingPoint,m_MaxNbOfHarrisMatchingPoint*sizeof(MatchingPoint_t));
  r = HarrisMatching(m_LEFT_Flow->Harris, m_LEFT_Harris_PreviousImage,
		     m_LEFT_OPF_m,  m_LEFT_PreviousImage,
		     20.0,15, 10,
		     -15,0, m_HarrisMatchingPoint,m_MaxNbOfHarrisMatchingPoint,10E15);
  ODEBUG("HRP2OpticalFlowProcess::ComputesHarrisMatchingPoints() : " << r );
  return 0;
}

int HRP2OpticalFlowProcess::AllocateOpticalFlowStructures()
{
  int i;
  /* Memory allocation for the optical flow computed using SSE */


  FreeOpticalFlowStructures();

  /* Left Image */
  m_LEFT_OPF_m = MMXMatrixAlloc(MMT_F_32,m_Width,m_Height);

  m_LEFT_Flow =  new MMXFlow;
  MMXSetToZeroFlow(m_LEFT_Flow);
  MMXInitFlow(m_LEFT_Flow,m_Width,m_Height);
  m_LEFT_Flow->tau = m_tau;

  /* Right Image */
  m_RIGHT_OPF_m = MMXMatrixAlloc(MMT_F_32,m_Width,m_Height);

  m_RIGHT_Flow = new MMXFlow;
  MMXSetToZeroFlow(m_RIGHT_Flow);
  MMXInitFlow(m_RIGHT_Flow,m_Width,m_Height);
  m_RIGHT_Flow->tau = m_tau;


  m_LEFT_Harris_Flow =  new MMXFlow;
  MMXSetToZeroFlow(m_LEFT_Harris_Flow);
  MMXInitFlow(m_LEFT_Harris_Flow,m_Width,m_Height);

  m_RIGHT_Harris_Flow =  new MMXFlow;
  MMXSetToZeroFlow(m_RIGHT_Harris_Flow);
  MMXInitFlow(m_RIGHT_Harris_Flow,m_Width,m_Height);


  m_LEFT_PreviousImage = MMXMatrixAlloc(MMT_F_32, m_Width, m_Height);
  m_LEFT_Harris_PreviousImage = MMXMatrixAlloc(MMT_F_32, m_Width-8, m_Height-8);

  /*
   * Initialization of the Tiles for the harris descriptors:
   * Size of the Image: 4 times less than the original.
   * Number of possible points per Tile : 5.
   * Separation space between points: 4.
   * Total number of points : 100.
   *
   */

  m_ImageOfTiles[0]  = HarrisAllocateTiles(m_LEFT_Flow->Harris->cols,m_LEFT_Flow->Harris->rows,
					   m_SizeOfTheImage,
					   m_NbOfPossiblePtPerTile,
					   m_SeparationSpace,
					   m_TotalNbOfPoints);
  m_ImageOfTiles[1]  = HarrisAllocateTiles(m_RIGHT_Flow->Harris->cols,m_RIGHT_Flow->Harris->rows,
					   m_SizeOfTheImage,
					   m_NbOfPossiblePtPerTile,
					   m_SeparationSpace,
					   m_TotalNbOfPoints);

  /*
   * Initialization of the Tiles for the harris descriptors:
   * Size of the Image: 4 times less than the original.
   * Number of possible points per Tile : 5.
   * Separation space between points: 4.
   * Total number of points : 100.
   *
   */
  m_PreviousImageOfTiles = HarrisAllocateTiles(m_LEFT_Flow->Harris->cols,m_LEFT_Flow->Harris->rows,
					       m_SizeOfTheImage,
					       m_NbOfPossiblePtPerTile,
					       m_SeparationSpace,
					       m_TotalNbOfPoints);

  return 0;
}

int HRP2OpticalFlowProcess::FreeOpticalFlowStructures()
{

  if (m_LEFT_OPF_m!=0)
    delete m_LEFT_OPF_m;

  if (m_LEFT_Flow!=0)
    {
      MMXFreeFlow(m_LEFT_Flow);
      //      delete m_LEFT_Flow;
    }
  if (m_RIGHT_OPF_m!=0)
    delete m_RIGHT_OPF_m;

  if (m_RIGHT_Flow!=0)
    {
      MMXFreeFlow(m_LEFT_Flow);
      //      delete m_RIGHT_Flow;
    }

  if (m_LEFT_Harris_Flow!=0)
    {
      MMXFreeFlow(m_LEFT_Harris_Flow);
      //      delete m_LEFT_Flow;
    }

  if (m_RIGHT_Harris_Flow!=0)
    {
      MMXFreeFlow(m_RIGHT_Harris_Flow);
      //      delete m_LEFT_Flow;
    }


  if (m_LEFT_PreviousImage!=0)
    delete m_LEFT_PreviousImage;

  if (m_LEFT_Harris_PreviousImage!=0)
    delete m_LEFT_Harris_PreviousImage;

  for(int i=0;i<2;i++)
    HarrisFreeTile(m_ImageOfTiles[i]);

  SetToZeroOpticalFlowStructures();

  return 0;
}

int HRP2OpticalFlowProcess::InitializeTheProcess()
{
  m_Width = m_InputImages[0].Width;
  m_Height = m_InputImages[0].Height;

   CvSize		sim;
  sim.width = m_Width;
  sim.height = m_Height;
  if (m_eigImage!=0)
    {
      delete m_eigImage->imageDataOrigin;
      delete m_eigImage;
    }
  m_eigImage = cvCreateImage(sim,IPL_DEPTH_32F,1);
  if (m_tempImage!=0)
    {
      delete m_tempImage->imageDataOrigin;
      delete m_tempImage;
    }
  m_tempImage = cvCreateImage(sim,IPL_DEPTH_32F,1);

  m_IplInputImage = cvCreateImage(sim,IPL_DEPTH_8U,1);
  AllocateOpticalFlowStructures();

  return 0;
}

int HRP2OpticalFlowProcess::RealizeTheProcess()
{
  int i,j;
  ODEBUG("m_Computing " << (int)m_Computing);
  if (!m_Computing)
    return 0;

  ODEBUG("Start the process.");

  if (m_Verbosity)
    fprintf(stderr,"HRP2OpticalFlowProcess:RealizeTheProcess\n");

  for(i=0;i<m_InputImages[0].Height;i++)
    {
       for(j=0;j<m_InputImages[0].Width;j++)
	 {
	   ((MM_F_32 *)m_LEFT_OPF_m->p[i].s)[j] = (float)(((unsigned char *)m_InputImages[0].Image)
	     [i*m_InputImages[0].Width + j])/255.0;
	   ((MM_F_32 *)m_RIGHT_OPF_m->p[i].s) [j] = (float)(((unsigned char *)m_InputImages[1].Image)
	     [i*m_InputImages[1].Width + j])/255.0;
	 }
    }

  if (m_HarrisMethod==HARRIS_MMX)
    {
      ComputesOpticalFlowAndHarris();
      SaveLastImages();
    }
  else if (m_HarrisMethod==HARRIS_OPENCV)

    HarrisWithOpenCV();

  else if (m_HarrisMethod==HARRIS_MMX_AND_OPENCV)
    {
      ComputesOpticalFlowAndHarris();
      SaveLastImages();
      HarrisWithOpenCV();
    }

  ODEBUG("End the process.");
  return 0;
}

int HRP2OpticalFlowProcess::GetFlowStructure(MMXFlow **aFlow, int aCameraID, int HarrisOrOPF)
{
  if (aFlow==0)
    return -1;

  if (aCameraID==LowLevelVisionServer::CAMERA_LEFT)
    {
      if (HarrisOrOPF==FLOW_STRUCTURE_OPF)
	*aFlow = m_LEFT_Flow;
      else if (HarrisOrOPF==FLOW_STRUCTURE_HARRIS)
	*aFlow = m_LEFT_Harris_Flow;
      return 0;
    }
  else if (aCameraID==LowLevelVisionServer::CAMERA_RIGHT)
    {
      if (HarrisOrOPF==FLOW_STRUCTURE_OPF)
	*aFlow = m_RIGHT_Flow;
      else if (HarrisOrOPF==FLOW_STRUCTURE_HARRIS)
	*aFlow = m_RIGHT_Harris_Flow;

      return 0;
    }
  return -1;

}

int HRP2OpticalFlowProcess::HarrisWithOpenCV()
{
  CvPoint2D32f cornersg[100];
  int cornerCount = 100;
  double qualityLevel = 0.1;	// Quality criterion
  double minDistance  = 9;	// Minimal distance between two points
  int i;

  FromEPBMToIPL(m_InputImages[1],EXISTING_IPL,m_IplInputImage);
  cvGoodFeaturesToTrack(m_IplInputImage,m_eigImage, m_tempImage, cornersg, &cornerCount, qualityLevel, minDistance);
  ODEBUG("Corner Count : " << cornerCount);

  if (m_DumpingLevel>=3)
    {
      static IplImage * res[3]= {0,0,0};
      static int counter = 0;

      if (res[0]==0)
	{
	  CvSize		sim;
	  sim.width = m_Width;
	  sim.height = m_Height;

	  res[0] = cvCreateImage(sim,8,3);
	}

      for(i=0;i<m_Width*m_Height;i++)
	{
	  res[0]->imageData[i*3] =
	    res[0]->imageData[i*3+1] =
	    res[0]->imageData[i*3+2] =
	    m_IplInputImage->imageData[i];

	}
      for(i=0;i<cornerCount;i++)
	{
	  int x,y;
	  x = (int)cornersg[i].x;
	  y = (int)cornersg[i].y;
	  res[0]->imageData[y*3*m_Width+x*3+0] = 0;
	  res[0]->imageData[y*3*m_Width+x*3+1] = 0;
	  res[0]->imageData[y*3*m_Width+x*3+2] = 255;
	}


      char FileName[1024];
      bzero(FileName,1024);
      sprintf(FileName,"/tmp/HarrisDetectorWithOpenCV_%06d",counter++);
      ToEPBMFile(res,FileName);
    }
  return 0;
}
int HRP2OpticalFlowProcess::CleanUpTheProcess()
{
  AllocateOpticalFlowStructures();
  return 0;
}

int HRP2OpticalFlowProcess::SetInputImages(EPBM InputImages[3])
{
  int i;

  for(i=0;i<3;i++)
    m_InputImages[i] = InputImages[i];

  return 0;
}

int HRP2OpticalFlowProcess::UpdateAMotionEvaluationProcess(HRP2MotionEvaluationProcess *aMEP)
{
  if (aMEP==0)
    return -1;

  aMEP->UpdateMatchingPointsWithHarris(m_SetOfMatchingPoint);
  return 0;

}

int HRP2OpticalFlowProcess::GetHarrisDetectorMethod()
{
  return m_HarrisMethod;
}

int HRP2OpticalFlowProcess::SetHarrisDetectorMethod(int aHarrisMethod)
{
  if ((aHarrisMethod!=HARRIS_OPENCV) &&
      (aHarrisMethod!=HARRIS_MMX))
    return -1;

  m_HarrisMethod = aHarrisMethod;

  return 0;
}

int HRP2OpticalFlowProcess::SetModeDump(int aMode)
{
  if ((aMode>=0) && (aMode<3))
    {
      m_DumpingLevel = aMode;
      return 0;
    }

  return -1;
}

int HRP2OpticalFlowProcess::GetModeDump()
{
  return m_DumpingLevel;
}


int HRP2OpticalFlowProcess::SetParameter(string aParameter, string aValue)
{
  int r;

  r = HRP2VisionBasicProcess::SetParameter(aParameter,aValue);

  ODEBUG("SetParameter: " << aParameter << " : " << aValue );
  if (aParameter=="dump_mode")
    {
      r = atoi(aValue.c_str());
      if (r!=-1)
	m_DumpingLevel = r;
    }

  if (aParameter=="RatioHarrisImage")
    {
      r = atoi(aValue.c_str());
      if (r!=-1)
	m_SizeOfTheImage = r;
    }

  if (aParameter=="NbOfPossiblePtPerTile")
    {
      r = atoi(aValue.c_str());
      if (r!=-1)
	m_NbOfPossiblePtPerTile = r;
    }

  if (aParameter == "TotalNbOfPoints")
    {
      r = atoi(aValue.c_str());
      if (r!=-1)
	m_TotalNbOfPoints = r;

    }

  if (aParameter == "tau")
    {
      double lf;
      lf = atof(aValue.c_str());
      m_tau = lf;
    }

  if (aParameter =="Harris_k")
    {
      double lf;
      lf = atof(aValue.c_str());
      m_Harris_k = lf;
    }

  if (aParameter=="HarrisThreshold")
    {
      double lf;
      lf = atof(aValue.c_str());
      m_HarrisThreshold = lf;
      ODEBUG3("HarrisThreshold: " << m_HarrisThreshold);
    }
  return 0;
}

