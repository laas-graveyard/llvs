/** @doc This object implements a visual process
    performing visual motion evualation.

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
extern "C" 
{
#include <range.h>
#include <rangeopen.h>
}

#include <OpticalFlowProcess.h>
#include <MotionEvaluationProcess.h>
#include <set>
#include <TU/Vector++.h>

// Debug macros
#include "Debug.h"

using namespace TU;

HRP2MotionEvaluationProcess::HRP2MotionEvaluationProcess() : HRP2VisionBasicProcess()
{
  m_ProcessName = "Motion Evaluation Process";
  m_Verbosity = 0;
  memset(&m_ranges[0],0,sizeof(RANGE));
  memset(&m_ranges[1],0,sizeof(RANGE));
  m_LastRangeIndex = 1;
}

HRP2MotionEvaluationProcess::~HRP2MotionEvaluationProcess()
{
 
}


int HRP2MotionEvaluationProcess::InitializeTheProcess()
{
  /* Set to zero the two range data */
  range_free(&m_ranges[0]);
  range_free(&m_ranges[1]); 

  m_LastRangeIndex = 1;
  return 0;
}

int HRP2MotionEvaluationProcess::UpdateRangeData(RANGE & arng)
{
  if (m_LastRangeIndex==-1)
    m_LastRangeIndex = 1;
  
  if (m_LastRangeIndex == 0)
    m_LastRangeIndex = 1;
  else 
    m_LastRangeIndex = 0;

  ODEBUG("MotionEvolutionProcess:UpdateRangeData: from origine: " << arng.DotCount);
  DuplicateRangeWithoutReallocation(arng,m_ranges[m_LastRangeIndex]);
  ODEBUG("MotionEvolutionProcess:UpdateRangeData: in the copy: " << m_ranges[m_LastRangeIndex].DotCount );
  return 0;
}

int HRP2MotionEvaluationProcess::UpdateMatchingPointsWithHarris(multiset <MatchingPoint_t *, MatchingPoint_lt> aSetOfMatchingPoints)
{
  ODEBUG( "HRP2MotionEvaluationProcess::UpdateMatchingPointsWithHarris " <<
    aSetOfMatchingPoints.size());
  m_SetOfHarrisMatchingPoints.clear();
  m_SetOfHarrisMatchingPoints = aSetOfMatchingPoints;
  ODEBUG( "HRP2MotionEvaluationProcess::UpdateMatchingPointsWithHarris : Copied pts : " <<
    aSetOfMatchingPoints.size());
  return 0;
}

int HRP2MotionEvaluationProcess::WithMatched3DHarrisPoints()
{
  /* Create the two clouds of 3 points using range data and harris points */
  multiset <MatchingPoint_t *, MatchingPoint_lt>::iterator it_SMP;
  Vector<float> *CenterOfGravity[2] = {0,0};
  int RangeIndex;
  int k=0;
  Matrix<float> *lClouds[2] = {0,0};
  Matrix<float> *W=0, B(3,3), Rotation(3,3), U(3,3), V(3,3), tmp;

  ODEBUG("Beginning of HRP2MotionEvaluationProcess");
  /* Remove elements with no corresponding 3D point */
  ODEBUG( "Before checking with the disparity map: Number of point belonging to the set of harris points " 
       << m_SetOfHarrisMatchingPoints.size() 
       << endl
       << "Number of points in the range data set " 
       << m_LastRangeIndex << " " 
       << m_ranges[0].DotCount << " " 
       << m_ranges[1].DotCount << " " 
       );

  for (int i=0;i<2;i++)
    {
      if (m_ranges[i].Map==0)
	return -1;
    }

  it_SMP = m_SetOfHarrisMatchingPoints.begin();
  k=0;
  while(it_SMP!=m_SetOfHarrisMatchingPoints.end())
    {
      MatchingPoint_t *aMP;

      aMP=*it_SMP;
      
      if (aMP!=0)
	{
	  PixelData *pmap;
	  ODEBUG(  "Current nb of points: " 
		   << m_SetOfHarrisMatchingPoints.size() );

	  for(int i=0;i<2;i++)
	    {
	      if (m_LastRangeIndex==0)
		RangeIndex = i;
	      else 
		RangeIndex = (i==0) ? 1:0;
	      
	      int x,y;
	      if (i==0)
		{
		  x = aMP->LeftCoordinates[0];
		  y = aMP->LeftCoordinates[1];
		}
	      else 
		{
		  x = aMP->RightCoordinates[0];
		  y = aMP->RightCoordinates[1];
		}
	      ODEBUG( "Point " << k << " = (" << x << "," << y << ")"); 
	      pmap = m_ranges[RangeIndex].Map[m_ranges[RangeIndex].Width * y + x];
	      if (pmap==0)
		{
		  m_SetOfHarrisMatchingPoints.erase(it_SMP);
		  break;
		}
	      
	    }
	  k++;
	}

      it_SMP++;
    }
  ODEBUG( "After checking with the disparity map: Number of point belonging to the set of harris points " 
       << m_SetOfHarrisMatchingPoints.size());


  for(int j=0;j<2;j++)
    {
      lClouds[j] =  new Matrix<float> (3,m_SetOfHarrisMatchingPoints.size());
      CenterOfGravity[j] = new Vector<float> (3);

      for(int i=0;i<3;i++)
	(*CenterOfGravity[j])[i]=0.0;
    }
  W = new Matrix<float> (m_SetOfHarrisMatchingPoints.size(),m_SetOfHarrisMatchingPoints.size());

  for(unsigned int j=0;j<m_SetOfHarrisMatchingPoints.size();j++)
    {
      for(unsigned int i=0; i < m_SetOfHarrisMatchingPoints.size();i++)
	{
	  if (i==j)
	    (*W)[j][i] = 1.0;
	  else 
	    (*W)[j][i] = 0.0;
	}
    }
  ODEBUG( "Creates the clouds ");
  it_SMP = m_SetOfHarrisMatchingPoints.begin();
  k=0;
  while(it_SMP!=m_SetOfHarrisMatchingPoints.end())
    {
      MatchingPoint_t *aMP;

      aMP=*it_SMP;
      
      if (aMP!=0)
	{
	  PixelData *pmap;
	  for(int i=0;i<2;i++)
	    {
	      if (m_LastRangeIndex==0)
		RangeIndex = i;
	      else 
		RangeIndex = (i==0) ? 1:0;
	      
	      int x,y;
	      if (i==0)
		{
		  x = aMP->LeftCoordinates[0];
		  y = aMP->LeftCoordinates[1];
		}
	      else 
		{
		  x = aMP->RightCoordinates[0];
		  y = aMP->RightCoordinates[1];
		}
	      ODEBUG( x << " " << y );

	      pmap = m_ranges[RangeIndex].Map[m_ranges[RangeIndex].Width * y + x];
	      
	      for(int l=0;l<3;l++)
		{
		  if (pmap!=0)
		    {
		      (*lClouds[i])[l][k] = (float)pmap->Dot[l];
		      (*CenterOfGravity[i])[l] += (float)(pmap->Dot[l]) ;
		      ODEBUG( pmap->Dot[l] );
		    }
		}
	      
	    }
	  k++;
	}

      it_SMP++;
    }
  
  ODEBUG( "Before computing the mean for both clouds of points " );
  /* Computes the mean for both clouds of points */
  for(int j=0;j<2;j++)    
    for(int i=0;i<3;i++)
      (*CenterOfGravity[j])[i] /= (float) m_SetOfHarrisMatchingPoints.size();
    
  ODEBUG( "Computes the translation " );

  /* Compute the translation */
   for(int i=0;i<3;i++)
    m_Translation[i] = (*CenterOfGravity[0])[i] - (*CenterOfGravity[1])[i];

  ODEBUG( "Suppress the translation for both clouds " );
  /* Suppress the translation for both clouds */  
  for(int j=0;j<2;j++)
    {
      for(int l =0;l<3;l++)
	{
	  for(unsigned int i=0;i<(lClouds[j])->ncol();i++)
	      (*lClouds[j])[l][i] -= (*CenterOfGravity[j])[l];
	}
    }

  ODEBUG( "Computes the matrix to inverse " );
  tmp = *W * lClouds[0]->trns();
  B = (*lClouds[1]) * tmp;
  
  ODEBUG( "Computes the SVD decomposition " );
  SVDecomposition<float> svd(B);

  ODEBUG( "Computes the rotation matrix " );  
  Rotation = svd.Ut() * svd.Vt().trns();

  if (Rotation.det()<0.0)
    {
      Matrix <float> I(3,3);
      for(int j=0;j<3;j++)
	for(int i=0;i<3;i++)
	  if (i==j)
	    I[i][j] = -1;
      Rotation = svd.Ut() * I * svd.Vt().trns();

    }

  //  if (m_Verbosity>=1)
    {
      static float Pos[3] = { 0.0,0.0,0.0};
      
      if ((!isnan(m_Translation[0])) &&
	  (!isnan(m_Translation[1])) &&
	  (!isnan(m_Translation[2])))
	{
	  Pos[0] += -m_Translation[2]/1000.0;
	  Pos[1] += -m_Translation[0]/1000.0;
	  Pos[2] += m_Translation[1]/1000.0;  
	  cerr << Pos[0] << " " << Pos[1] << " " << Pos[2];
	  cerr << endl;
	}

#if 0
      for(int i=0;i<3;i++)
	{
	  for(int j=0;j<3;j++)
	    {
	      cerr << Rotation[i][j] << " " ;
	    }
	  cerr << endl;
	}
#endif
    }	
  ODEBUG( "Deletion " );
  for(int j=0;j<2;j++)
    {
      delete lClouds[j];
      delete CenterOfGravity[j];
    }
  delete W;

  ODEBUG("End of HRP2MotionEvaluationProcess");
  return 0;
}

int HRP2MotionEvaluationProcess::RealizeTheProcess()
{
  ODEBUG3("m_Computing "<< (int)m_Computing);
  if (!m_Computing)
    return 0;

  ODEBUG("Start the process");
  WithMatched3DHarrisPoints();
  ODEBUG("End the process");
  return 0;
}

int HRP2MotionEvaluationProcess::CleanUpTheProcess()
{
  
  return 0;
}



/* This method is build upon range_duplicate2 */
int HRP2MotionEvaluationProcess::DuplicateRangeWithoutReallocation(RANGE & orig, RANGE & copy)
{
  RangeCount	i0;
  PixelData	**pmo=0, **pmc=0, *ppc=0;
  DOUBLE64	*pdc=0, *pnc=0, *pcc=0, *prc=0;
  UCHAR8	*pcolc=0;
  REGION	*prlo=0, *prlc=0;
  unsigned char identical=1;

  /* range_create() */
  if (m_Verbosity>=4)
    {
      cerr <<  "copy.PixelCount : " << copy.PixelCount << endl
	   <<  "copy.DotCount   : " << copy.DotCount   << endl
	   <<  "copy.NormCount  : " << copy.NormCount  << endl
	   <<  "copy.CvecCount  : " << copy.CvecCount  << endl
	   <<  "copy.RhoCount   : " << copy.RhoCount   << endl
	   <<  "copy.RegionCount: " << copy.RegionCount << endl
	   <<  "copy.ColorCount : " << copy.ColorCount << endl
	   <<  "copy.Height     : " << copy.Height << endl
	   <<  "copy.Width      : " << copy.Width << endl;
      
      cerr <<  "orig.PixelCount : " << orig.PixelCount << endl
	   <<  "orig.DotCount   : " << orig.DotCount   << endl
	   <<  "orig.NormCount  : " << orig.NormCount  << endl
	   <<  "orig.CvecCount  : " << orig.CvecCount  << endl
	   <<  "orig.RhoCount   : " << orig.RhoCount   << endl
	   <<  "orig.RegionCount: " << orig.RegionCount << endl
	   <<  "orig.ColorCount : " << orig.ColorCount << endl
	   <<  "orig.Height     : " << orig.Height << endl
	   <<  "orig.Width      : " << orig.Width << endl;
    }

  if ((copy.Height!=orig.Height) ||
      (copy.Width!=orig.Width)) 
    identical = 0;
  

  if (!identical)
    {
      
      if (m_Verbosity>=2) 
	{
	  cerr << "HRP2MotionEvaluationProcess::DuplicateRangeWithoutReallocation(): Allocate the copy" << endl;
	  
	  if (orig.Label & RANGE_LABEL_DOT)
	    cerr << "Label : RANGE_LABEL_DOT " << endl;
	  
	  if (orig.Label & RANGE_LABEL_COLOR)
	    cerr << "Label : RANGE_LABEL_COLOR" << endl;
	}
      range_free(&copy);

      range_create(orig.Label, orig.Width, orig.Height,
		   orig.DataType, orig.MapType, orig.Coord,
		   &orig.PinholeParameter, &orig.LasMotion,
		   &copy, 0);
    }
  else
    {
      bzero(copy.Map,sizeof(PixelData *)*orig.MapSize);
    }


  /* Data のコピー */
  pmo = orig.Map;
  pmc = copy.Map;
  ppc = copy.PixelList;
  pdc = copy.DotList;
  pnc = copy.NormList;
  pcc = copy.CvecList;
  prc = copy.RhoList;
  pcolc = copy.ColorList;

  if (m_Verbosity>=4)
    {
      if (copy.Label & RANGE_LABEL_DOT)
	cerr << "COPY Label : RANGE_LABEL_DOT " << endl;
      
      if (copy.Label & RANGE_LABEL_COLOR)
	cerr << "COPY Label : RANGE_LABEL_COLOR" << endl;

    }

  for(i0 = 0; i0 < orig.MapSize; i0++, pmo++, pmc++){
    if(*pmo){
      ODEBUG( "Point in " << i0 % orig.Width << " " << i0 / orig.Width );

      ppc->Label = (*pmo)->Label;

      /* Dot のコピー */
      if(ppc->Label & RANGE_LABEL_DOT){
	memcpy(pdc, (*pmo)->Dot, (size_t)(sizeof(DOUBLE64) * 3));
	ppc->Dot = pdc;
	pdc += 3;
	copy.DotCount = orig.DotCount;
	ODEBUG( ppc->Dot[0] << " " << ppc->Dot[1] << " " << ppc->Dot[2] );
      }else{
	ppc->Dot = NULL;
      }

      /* Norm のコピー */
      if(ppc->Label & RANGE_LABEL_NORM){
	memcpy(pnc, (*pmo)->Norm, (size_t)(sizeof(DOUBLE64) * 3));
	ppc->Norm = pnc;
	pnc += 3;
	copy.NormCount = orig.NormCount;
      }else{
	ppc->Norm = NULL;
      }

      /* Cvec のコピー */
      if(ppc->Label & RANGE_LABEL_CVEC){
	memcpy(pcc, (*pmo)->Cvec, (size_t)(sizeof(DOUBLE64) * 3));
	ppc->Cvec = pcc;
	pcc += 3;
	copy.CvecCount = orig.CvecCount;
      }else{
	ppc->Cvec = NULL;
      }

      /* Rho のコピー */
      if(ppc->Label & RANGE_LABEL_RHO){
	memcpy(prc, (*pmo)->Rho, (size_t)(sizeof(DOUBLE64) * 2));
	ppc->Rho = prc;
	prc += 2;
	copy.RhoCount = orig.RhoCount;
      }else{
	ppc->Rho = NULL;
      }

      /* Surf のコピー */
      if(ppc->Label & RANGE_LABEL_SURFACE){
	ppc->Surf = (*pmo)->Surf;
      }else{
	ppc->Surf = 0;
      }

      /* Nreg のコピー */
      if(ppc->Label & RANGE_LABEL_REGION){
	ppc->Nreg = (*pmo)->Nreg;
	copy.RegionCount = orig.RegionCount;
      }else{
	ppc->Nreg = -1;
      }

      /* Color のコピー */
      if(ppc->Label & RANGE_LABEL_COLOR){
	memcpy(pcolc, (*pmo)->Color, (size_t)(sizeof(UCHAR8) * 3));
	ppc->Color = pcolc;
	pcolc += 3;
	copy.ColorCount = orig.ColorCount;
      }else{
	ppc->Color = NULL;
      }

      /* Map[i0] のセット */
      *pmc = ppc;
      ppc++;
    }
  }

  /* region のコピー */
  if(orig.Label & RANGE_LABEL_RLIST){
    prlo = orig.RegionList;

    if (!copy.RegionList)
      copy.RegionList = prlc =
	(REGION *)malloc((size_t)(sizeof(REGION) * orig.RegionCount));

    if(!(copy.RegionList)){
      /* RegionList の malloc 失敗 */
      /*
      fprintf(stderr, "Error:memory allocation failed(dup RegionList).\n");
      fprintf(stderr, "      Region Data not copied\n");
      */
      copy.RegionCount = 0;
      copy.Label &= ~RANGE_LABEL_RLIST;

      range_free(&copy);
      return 0;
    }

    for(i0 = 0; i0 < orig.RegionCount; i0++, prlc++, prlo++){
      memcpy(prlc, prlo, (size_t)(sizeof(REGION)));
      /* p だけ親領域へのポインタであるため、処理が変わる */
      if(prlo->p){
	prlc->p = &copy.RegionList[prlo->p->i];
      }else{
	prlc->p = NULL;
      }
    }
  }

  /* 内部 rotmat の初期化 */
  rng_initial_rotate_matrix(&copy, &copy.rotmat, 0);
  
  ODEBUG(  "copy.PixelCount : " << copy.PixelCount << endl
	   <<  "copy.DotCount   : " << copy.DotCount   << endl
	   <<  "copy.NormCount  : " << copy.NormCount  << endl
	   <<  "copy.CvecCount  : " << copy.CvecCount  << endl
	   <<  "copy.RhoCount   : " << copy.RhoCount   << endl
	   <<  "copy.RegionCount: " << copy.RegionCount << endl
	   <<  "copy.ColorCount : " << copy.ColorCount << endl
	   <<  "copy.Height     : " << copy.Height << endl
	   <<  "copy.Width      : " << copy.Width );
	
  return 0;
}

