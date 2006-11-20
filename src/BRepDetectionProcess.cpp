/** @doc This object implements a visual process
    to build boundary representation.
    
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

#include "BRepDetectionProcess.h"
#include "BRep.h"

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "HPR2BRepDetectionProcess:" << x << endl
#define ODEBUG3_CONT(x) cerr << x 

#if 0
#define ODEBUG(x) cerr << "HPR2BRepDetectionProcess:" <<  x << endl
#define ODEBUG_CONT(x) cerr << "HPR2BRepDetectionProcess:" <<  x << endl
#else
#define ODEBUG(x) 
#define ODEBUG_CONT(x) 
#endif

extern "C" {
#include "epbmtobrep.h"
}

HRP2BRepDetectionProcess::HRP2BRepDetectionProcess()
{
  m_ProcessName = "Boundary-Representation Detection Process";
  for(int i=0;i<2;i++)
    m_ImagesOnWhichToProcess[i]=1;
  m_ImagesOnWhichToProcess[2]=0;
}

HRP2BRepDetectionProcess::~HRP2BRepDetectionProcess()
{
}

void HRP2BRepDetectionProcess::SetImageOnWhichToProcess(int ImagesOnWhichToProcess[3])
{
  for(int i=0;i<3;i++)
    m_ImagesOnWhichToProcess[i] = ImagesOnWhichToProcess[i];

}

int HRP2BRepDetectionProcess::GetImagesOnWhichToProcess(int ImagesOnWhichToProcess[3])
{
  for(int i=0;i<3;i++)
    ImagesOnWhichToProcess[i] = m_ImagesOnWhichToProcess[i];
  return 0;
}

int HRP2BRepDetectionProcess::InitializeTheProcess()
{
  epbm_minitialize(m_Edge,3);
  brep_minitialize(m_BRep,3);
  epbmtobrep_init_options(&m_opt);
  m_opt.param.label &= ~(STEP1X_METHODS_MASK);
  m_opt.param.label |= STEP1X_LAPLACIAN_MASK;
  m_opt.param.eth=20;

  string aParameter,aValue;
  aParameter = "Threshold";
  char Buffer[124];
  sprintf(Buffer,"%d",(int)m_opt.param.eth);
  aValue = Buffer;
  HRP2VisionBasicProcess::SetParameter(aParameter, aValue);
  return 0;
}


int HRP2BRepDetectionProcess::RealizeTheProcess()
{
  if (!m_Computing)
    return 0;
    
  int EAt_orig = m_opt.param.eth;

  for(int i=0;i<3;i++)
    {
      if (m_ImagesOnWhichToProcess[i]!=0)
	{
	  m_opt.param.eth = EAt_orig;
	  epbmtobrep(&m_InputImage[i],&m_BRep[i],&m_opt, 0, 0, 0);
	}
    }
  //  brep_msave("output.brp",&m_BRep[0],3,0);
		     
  return 0;
}

int HRP2BRepDetectionProcess::CleanUpTheProcess()
{
  epbm_mfree(m_Edge,3);
  brep_mfree(m_BRep,3);
  return 0;
}


int HRP2BRepDetectionProcess::SetInputImages(EPBM lInputImages[3])
{
  for(int i=0;i<3;i++)
    m_InputImage[i] = lInputImages[i];
  return 0;
}

int HRP2BRepDetectionProcess::BuildBrepVar(CBREPSeq_var & aBrepVar)
{
  int NbOfImagesToProcess = 0;

  for(int i=0;i<3;i++)
    if (m_ImagesOnWhichToProcess[i]!=0)
      NbOfImagesToProcess++;
    
  aBrepVar = new CBREPSeq;
  aBrepVar->length(NbOfImagesToProcess);
  
  for(int i=0,k=0;i<NbOfImagesToProcess;i++)
    if (m_ImagesOnWhichToProcess[i]!=0)
      FromBREPToCorbaBREP(aBrepVar[k++],i);

  return NbOfImagesToProcess;
}

int HRP2BRepDetectionProcess::FromBREPToCorbaBREP(CBREP &aBREP_var, int anImage)
{
  if ((anImage<0) && (anImage>3))
    return 0;

  /* Simple field */
  aBREP_var.label = m_BRep[anImage].label;
  aBREP_var.nregion = m_BRep[anImage].nregion;
  aBREP_var.nbound = m_BRep[anImage].nbound;
  aBREP_var.nsegment = m_BRep[anImage].nsegment;
  aBREP_var.npoint = m_BRep[anImage].npoint;
  aBREP_var.ncnet = m_BRep[anImage].ncnet;
  aBREP_var.colstart = m_BRep[anImage].colstart;
  aBREP_var.rowstart = m_BRep[anImage].rowstart;
  aBREP_var.ncol = m_BRep[anImage].ncol;
  aBREP_var.nrow = m_BRep[anImage].nrow;  
  aBREP_var.CameraID = anImage;

  aBREP_var.region.length(aBREP_var.nregion);
  for(int i=0;i<aBREP_var.nregion;i++)
    {
      FromBREPRegionToCorbaBREPRegion(&m_BRep[anImage].region[i],aBREP_var.region[i]);
    }

  aBREP_var.bound.length(aBREP_var.nbound);
  for(int i=0;i<aBREP_var.nbound;i++)
    {
      FromBREPBoundaryToCorbaBREPBoundary(&m_BRep[anImage].bound[i],aBREP_var.bound[i]);
    }

  aBREP_var.cnet.length(aBREP_var.ncnet);
  for(int i=0;i<aBREP_var.ncnet;i++)
    {
      FromBREPCnetToCorbaBREPCnet(&m_BRep[anImage].cnet[i],aBREP_var.cnet[i]);
    }
  
  
  return 0;
}

int HRP2BRepDetectionProcess::FromBREPPointToCorbaBREPPoint(BREP_Point *abrepp, CBREP_Point &aBREP_Point_var)
{
  aBREP_Point_var.label = abrepp->label;
  aBREP_Point_var.n = abrepp->n;
  aBREP_Point_var.row = abrepp->row;
  aBREP_Point_var.col = abrepp->col;
  //  cout << "  " <<aBREP_Point_var.row << " " << aBREP_Point_var.col << " " << endl;
  aBREP_Point_var.rowd = abrepp->rowd;
  aBREP_Point_var.cold = abrepp->cold;
  aBREP_Point_var.Xr = abrepp->X;
  aBREP_Point_var.Yr = abrepp->Y;
  aBREP_Point_var.x = abrepp->x;
  aBREP_Point_var.y = abrepp->y;
  aBREP_Point_var.z = abrepp->z;
  aBREP_Point_var.normals = abrepp->normal;
  
  for(int i=0;i<3;i++)
    {
      aBREP_Point_var.Normal[i] = abrepp->Normal[i];
      aBREP_Point_var.Tangent[i] = abrepp->Tangent[i];
    }

  aBREP_Point_var.curvature = abrepp->curvature;
  aBREP_Point_var.error = abrepp->error;

  aBREP_Point_var.red = abrepp->red;
  aBREP_Point_var.green = abrepp->green;
  aBREP_Point_var.blue = abrepp->blue;

  return 0;
}

int HRP2BRepDetectionProcess::FromBREPCnetToCorbaBREPCnet(BREP_Cnet * aBREP_Cnet, CBREP_Cnet &aBREP_Cnet_var)
{
  aBREP_Cnet_var.label = aBREP_Cnet->label;
  aBREP_Cnet_var.n = aBREP_Cnet->n;
  aBREP_Cnet_var.mapcol = aBREP_Cnet->mapcol;
  aBREP_Cnet_var.maprow = aBREP_Cnet->maprow;
  for(int i=0;i<3;i++)
    {
      aBREP_Cnet_var.Position[i] = aBREP_Cnet->Position[i];
      aBREP_Cnet_var.Normal[i] = aBREP_Cnet->Normal[i];
    }
  aBREP_Cnet_var.red = aBREP_Cnet->red;
  aBREP_Cnet_var.green = aBREP_Cnet->green;
  aBREP_Cnet_var.blue = aBREP_Cnet->blue;
  

  return 0;
}

int HRP2BRepDetectionProcess::FromBREPRegionToCorbaBREPRegion(BREP_Region * aBREP_Region, CBREP_Region &aBREP_Region_var)
{
  aBREP_Region_var.label = aBREP_Region->label;
  aBREP_Region_var.n = aBREP_Region->n;
  aBREP_Region_var.nbound = aBREP_Region->nbound;
  aBREP_Region_var.nsegment = aBREP_Region->nsegment;
  aBREP_Region_var.npoint = aBREP_Region->npoint;
  aBREP_Region_var.ncnet = aBREP_Region->ncnet;
  aBREP_Region_var.area = aBREP_Region->area;
  aBREP_Region_var.width = aBREP_Region->width;
  aBREP_Region_var.type = aBREP_Region->type;
  aBREP_Region_var.intensity = aBREP_Region->intensity;
  aBREP_Region_var.std_deviation  = aBREP_Region->std_deviation;
  
  aBREP_Region_var.bound.length(aBREP_Region->nbound);
  for(int i=0;i<aBREP_Region->nbound;i++)
    FromBREPBoundaryToCorbaBREPBoundary(aBREP_Region->bound+i, aBREP_Region_var.bound[i]);

  aBREP_Region_var.segment.length(aBREP_Region->nsegment);
  for(int i=0;i<aBREP_Region->nsegment;i++)
    FromBREPSegmentToCorbaBREPSegment(aBREP_Region->segment+i, aBREP_Region_var.segment[i]);

  aBREP_Region_var.point.length(aBREP_Region->npoint);
  for(int i=0;i<aBREP_Region->npoint;i++)
    FromBREPPointToCorbaBREPPoint(aBREP_Region->point+i, aBREP_Region_var.point[i]);

  aBREP_Region_var.cnet.length(aBREP_Region->ncnet);
  for(int i=0;i<aBREP_Region->ncnet;i++)
    FromBREPCnetToCorbaBREPCnet(aBREP_Region->cnet+i, aBREP_Region_var.cnet[i]);
  
  return 0;
}

int HRP2BRepDetectionProcess::FromBREPBoundaryToCorbaBREPBoundary(BREP_Bound * aBREP_Boundary, CBREP_Bound &aBREP_Boundary_var)
{
  aBREP_Boundary_var.label = aBREP_Boundary->label;
  aBREP_Boundary_var.n = aBREP_Boundary->n;
  aBREP_Boundary_var.nsegment = aBREP_Boundary->nsegment;
  aBREP_Boundary_var.npoint = aBREP_Boundary->npoint;

  aBREP_Boundary_var.segment.length(aBREP_Boundary->nsegment);
  for(int i=0;i<aBREP_Boundary->nsegment;i++)
    FromBREPSegmentToCorbaBREPSegment(aBREP_Boundary->segment+i, aBREP_Boundary_var.segment[i]);

  aBREP_Boundary_var.point.length(aBREP_Boundary->npoint);
  for(int i=0;i<aBREP_Boundary->npoint;i++)
    FromBREPPointToCorbaBREPPoint(aBREP_Boundary->point+i, aBREP_Boundary_var.point[i]);
  
  return 0;
}

int HRP2BRepDetectionProcess::FromBREPSegmentToCorbaBREPSegment(BREP_Segment * aBREP_Segment, CBREP_Segment &aBREP_Segment_var)
{
  aBREP_Segment_var.label = aBREP_Segment->label;
  aBREP_Segment_var.n = aBREP_Segment->n;
  aBREP_Segment_var.npoint = aBREP_Segment->npoint;

  aBREP_Segment_var.curvature = aBREP_Segment->curvature;
  aBREP_Segment_var.error = aBREP_Segment->error;
  aBREP_Segment_var.intensity_sigma = aBREP_Segment->intensity_sigma;

  aBREP_Segment_var.point.length(aBREP_Segment->npoint);
  for(int i=0;i<aBREP_Segment->npoint;i++)
    FromBREPPointToCorbaBREPPoint(aBREP_Segment->point+i, aBREP_Segment_var.point[i]);

  aBREP_Segment_var.red = aBREP_Segment->red;
  aBREP_Segment_var.green = aBREP_Segment->green;
  aBREP_Segment_var.blue = aBREP_Segment->blue;
  
  return 0;
}


int HRP2BRepDetectionProcess::SetParameter(string aParameter, string aValue)
{
  int r;

  r = HRP2VisionBasicProcess::SetParameter(aParameter, aValue);
  
  if (aParameter=="Threshold")
    {
      m_opt.param.eth = atoi(aValue.c_str());
    }
  return 0;
}
