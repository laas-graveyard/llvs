/** @doc This file implements the access to IEEE 1394 cameras
    through the dc library.

   CVS Information:
   $Id$
   $Author$
   $Date$
   $Revision$
   $Source$
   $Log$

   Copyright (c) 2003-2006, 
   @author Claire Dune,
   2010
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
#include <vector>
#include <sstream>

using namespace std;


#if LLVS_HAVE_VISP && LLVS_HAVE_NMBT

#include "ModelTracker/nmbtTrackingProcess.h"


#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "HPR2Tracker:" << x << endl
#define ODEBUG3_CONT(x) cerr << x 

#if 0
#define ODEBUG(x) cerr << "HPR2Tracker:" <<  x << endl
#define ODEBUG_CONT(x) cerr << "HPR2Tracker:" <<  x << endl
#else
#define ODEBUG(x) 
#define ODEBUG_CONT(x) 
#endif



// test the parameters functions
int main(void)
{
  cout <<" -------------------------- " << endl;
  cout << endl; 
  cout <<" Test NMBT tracking class"    << endl;
  cout <<"  parameter input/output test"    << endl;
  cout << endl; 
  cout <<" ---------------------------" << endl;

  
  // create the path to the box
  char* homePath;
  homePath = getenv ("HOME");
  string defaultPath ( "data/model/WoodenBox/WoodenBox.wrl");
  ostringstream tmp_stream;
  tmp_stream<< homePath << "/"<<defaultPath;
  cout << "Path :" << tmp_stream.str()  <<endl;

  // create a default object tracker
  HRP2nmbtTrackingProcess tracker;
  
  // get process name
  string name = tracker.GetName();
  cout <<endl<<"1."<<endl << "Name of the process : " << name << endl;  
 
  cout << endl<<"2."<<endl <<"Add parameters"<<endl;
  // set parameter, one listed and one unknown
  tracker.SetParameter("VPME_MASK_SIZE","5");
  tracker.SetParameter("FALSE_PARAM","-450"); 
  tracker.SetParameter("PATH_MODEL",tmp_stream.str()); 
  tracker.SetParameter("TRAC_LAMBDA","0.4"); 
 
  // get the parameters  
  vector<string> parameters, values; 
  tracker.GetParametersAndValues(parameters, values);
  int nbParam(parameters.size());  
  
  cout << endl<<"Print all  parameters"<<endl
       <<"Num \tParam \t\tValue" <<endl; 
  for (int i = 0;i< nbParam;++i)
    {
      cout << i<<"\t"
	   << parameters[i] <<"\t"
	   << values[i]<<endl;
      
    }
  
  cout << endl<<"3."<< endl
       << "convert a string into a double"<<endl;
  double d;
  string ds("5");
  std::istringstream i(ds);
  i >> d;
     

  cout <<"convert " << ds << " to " << d << endl;
  cout << endl;  
  cout <<" -------------------------- " << endl; 
  cout << endl; 
  cout <<"    .....End of the test    "    << endl;
  cout << endl; 
  cout <<" ---------------------------" << endl;
  // everything went well
  return 0;
}


#else

int main (void)
{
  cout <<"NMBT not found -> cannot test the parameters!\n";
  return 0;
}

#endif
