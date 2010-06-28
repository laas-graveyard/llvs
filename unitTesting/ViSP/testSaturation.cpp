/** @doc This file implements test for vispConvertImage.cpp


   Copyright (c) 2010, 
   @author Stephane EMBARKI,
   
   JRL-Japan, CNRS/AIST

   See license file for information on license.
*/



#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

#include "llvsConfig.h"

#if (LLVS_HAVE_VISP>0  && LLVS_HAVE_NMBT>0)

#include <visp/vpConfig.h>



//visp
#include <visp/vpColVector.h>
#include <visp/vpPlot.h>


// LLVS
#include "ViSP/ComputeControlLawProcess.h"

using namespace std;


int main(void)
{

#ifdef VISP_HAVE_LIBPLOTTER
  	//Create a window with two graphics
	vpPlot G(2);

	//The first graphic contains 1 curve and the second graphic contains 2 curves
	G.initGraph(0,3);

	//The first curve in the ith graphic is green
	G.setColor(0,0,0,55000,0);
	G.setColor(0,1,0,0,55000);
	G.setColor(0,2,55000,0,0);

	//For the ith graphic : along the x axis the expected values are between 0 and 200 and the step is 1
	//For the ith graphic : along the y axis the expected values are between 0 and 10 and the step is 0.1
	G.initRange(0,0,1,0.01,-0.4,0.6,0.01);
	G.setLegend(0, 0, "X before saturation");
	G.setLegend( 0, 1, "Y before saturation");
	G.setLegend( 0, 2, "Z before saturation");

	G.initGraph(1,3);
	G.setColor(1,0,0,55000,0);
	G.setColor(1,1,0,0,55000);
	G.setColor(1,2,55000,0,0);
	G.initRange(1,0,1,0.01,-0.2,0.2,0.01);
  	G.setLegend( 1, 0, "X after saturation");
	G.setLegend( 1, 1, "Y after saturation");
	G.setLegend( 1, 2, "Z after saturation");
  

	HRP2ComputeControlLawProcess * CCLProcess;
	CCLProcess = new HRP2ComputeControlLawProcess;
	
	CCLProcess->pSetParameter("VEL_MAX","0.2:0.1:0.1");

	vpColVector V0(3);

	V0[0]=0.4;
	V0[1]=-0.3;
	V0[2]=0.2;

	vpColVector RawVel(3);
	double SatVel[3];


	for(int i=0;i<100;++i)
	  {
	    double t=0.01*i;
	    RawVel[0]=V0[0]*exp(-2*t);
	    RawVel[1]=V0[1]+0.8*t;
	    RawVel[2]=V0[2]-0.4*t;

	    CCLProcess->VelocitySaturation(RawVel,SatVel);


	    G.plot(0,0,t,RawVel[0]);
	    G.plot(0,1,t,RawVel[1]);
	    G.plot(0,2,t,RawVel[2]);
	    G.plot(1,0,t,SatVel[0]);
	    G.plot(1,1,t,SatVel[1]);
	    G.plot(1,2,t,SatVel[2]);

	  }
	
#else

std::cout <<std::endl<<"Need ViSP with libplotter to execute this example"<<std::endl;
#endif


  return 0;
}


#else

int main(void)
{
  std::cout <<std::endl<<"Need ViSP and NMBT to execute this example"<<std::endl;

  return 0;

}
#endif
