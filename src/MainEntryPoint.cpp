#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <signal.h>
#include <getopt.h>

#ifdef __ORBIX__
#include <OBE/CORBA.h>
#include <OBE/CosNaming.h>
#endif

#ifdef OMNIORB4
#include <omniORB4/CORBA.h>
#endif


#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "LLVS::MainEntryPoint:" << x << endl

#if 1
#define ODEBUG(x) cerr << "LLVS::MainEntryPoint:" <<  x << endl
#else
#define ODEBUG(x) 
#endif

#include "LowLevelVisionServer.h"

pthread_t MainThread;
LowLevelVisionServer *GlobalVisionServer=0;

void * LLVSThread(void *arg)
{
  LowLevelVisionServer *aVS = (LowLevelVisionServer *)arg;
  
  if (aVS!=0)
    {
      double current_time;
      double mean=0;

      
      // Start to process.
      aVS->StartMainProcess();

      // Current strategy:
      // If the Vision system read a file and or a directory,
      // or takes information from the frame-grabber 
      // it is in stand-alone mode.
      // In simulation mode, everything is trigger through
      // the SetImage method.
      if (aVS->GetInputMode()==LowLevelVisionSystem::SIMULATION)
	pthread_exit(0);

      // Vision Main loop.
      while(1)
	{
	  aVS->ApplyingProcess();
	  //	  usleep(1);
	  //      sleep(1);
	}
    }
  return 0;
}


void SIGINT_handler(int asig)
{
  //  cout << "Went through SIGINT_handler : "<< asig << endl;
  if((GlobalVisionServer!=0) && (pthread_self()==MainThread))
    {
      /* Stop the processes */
      GlobalVisionServer->StopMainProcess();
      cout << "Stopped the processes "<< endl;

      GlobalVisionServer->RecordImagesOnDisk(0);
      
      /* Delete the object. */
      delete GlobalVisionServer;
      cout << "delete VisionServer "<< endl;
      
      GlobalVisionServer=0;
      exit(0);
    }
}

int main(int argc, char * argv[])
{
  LowLevelVisionServer * aVS=0;
  static struct option long_options[]= 
    {
      {"fg",1,0,0},          /* 0 - Framegrabber */
      {"files",1,0,0},       /* 1 - Files */
      {"sim",0,0,0},         /* 2 - Simulator */
      {"single",1,0,0},      /* 3 - Single File */
      {"caldir",1,0,0},      /* 4 - Calibration directory */
      {"disp_par",1,0,0},    /* 5 - Disparity parameter */
      {"disp_val",1,0,0},    /* 6 - Disparity value */
      {"check_entry",0,0,0}, /* 7 - Check entries of the module */
      {"fgsize",1,0,0},      /* 8 - Grabbing size Width (only for framegrabbing) */
      {"opf_par",1,0,0},     /* 9 - Optical Flow parameter */
      {"opf_val",1,0,0},     /* 10 - Optical Flow value */
      {"NameService",1,0,0}, /* 11 - Name Service information */
      {"synchro",1,0,0},     /* 12 - Synchro mode */
      {"rbtcalibdir",1,0,0}, /* 13 - Robot To vision calibration directory */
      {"ffii_par",1,0,0},    /* 14 - Find Features in image parameter */
      {"ffii_val",1,0,0},    /* 15 - Find Features in image value */
      {0,0,0,0}
    };
  LowLevelVisionSystem::InputMode InputType=LowLevelVisionSystem::FRAMEGRABBER;
  LowLevelVisionSystem::SynchroMode SynchroType=LowLevelVisionSystem::SYNCHRO_TRIGGER;
  string filename;
  string calibdir;
  string rbtvisiondir;
  int Verbosemode=0;
  vector <string> DisparityParameters;
  vector <string> DisparityValueParameters;
  vector <string> OPFParameters;
  vector <string> OPFValueParameters;
  vector <string> FFIIParameters; /* Find Feature In Images parameters. */
  vector <string> FFIIValueParameters; /* Find Feature In Images parameters' values. */

  // Parameters for the size of the Frame Grabbing.
  string FGSize;
  int FGWidth=0, FGHeight=0;
  unsigned char bFGSize = 0;

  /* Dump the entry of the module if necessary , no by default */
  unsigned char CheckEntry=0;

  /* String related to the Name Service */
  string NameServiceString("NameService=");
  
  for(int i=0;i<argc;i++)
    cout << argv[i] << endl;
  // Parse the entry parameters.
  while(1)
    {
      int c;
      int this_option_optind = optind ? optind : 1;
      int option_index = 0;
      
      c = getopt_long(argc,argv,"v:",long_options, &option_index);
      if (c==-1)
	break;

      switch(c)
	{
	case 0:
	  if (option_index==0) 
	    {
	      InputType = LowLevelVisionSystem::FRAMEGRABBER;
	      
	    }
	  if (option_index==1) 
	    {
	      InputType = LowLevelVisionSystem::FILES;
	      filename = optarg;
	    }
	  if (option_index==3)
	    {
	      InputType = LowLevelVisionSystem::FILESINGLE;
	      filename = optarg;
	    }
	  if (option_index==2)
	    InputType = LowLevelVisionSystem::SIMULATION;

	  if (option_index==4)
	    {
	      calibdir = optarg;
	    }
	  if (option_index==5)
	    {
	      string tmp = optarg;
	      DisparityParameters.insert(DisparityParameters.end(),tmp);
	    }
	  if (option_index==6)
	    {
	      string tmp = optarg;
	      DisparityValueParameters.insert(DisparityValueParameters.end(),tmp);
	    }
	  if (option_index==7)
	    {
	      CheckEntry = 1;
	    }
	  if (option_index==8)
	    {
	      string SWidth, SHeight;
	      unsigned int pos;
	      FGSize = optarg;
	      pos = FGSize.find("x",0);
	      if (pos!=string::npos)
		{
		  SWidth =  FGSize.substr(0,pos);
		  SHeight = FGSize.substr(pos+1,FGSize.length()-pos-1);
		  bFGSize = 1;
		  FGWidth = atoi(SWidth.c_str());
		  FGHeight = atoi(SHeight.c_str());
		  cerr << "SWidth " << FGWidth << " " << " SHeight " << FGHeight << endl;
		}
	    }
	  if (option_index==9)
	    {
	      string tmp = optarg;
	      OPFParameters.insert(OPFParameters.end(),tmp);
	    }

	  if (option_index==10)
	    {
	      string tmp = optarg;
	      OPFValueParameters.insert(OPFValueParameters.end(),tmp);
	    }
	  if (option_index==11)
	    {
	      NameServiceString += optarg;
	    }
	  if (option_index==12) 
	    {
	      string tmp = optarg;
	      if (tmp=="flow")
		SynchroType = LowLevelVisionSystem::SYNCHRO_FLOW;
	      else if (tmp=="trigger")
		SynchroType = LowLevelVisionSystem::SYNCHRO_TRIGGER;
	    }
	  if (option_index==13)
	    {
	      rbtvisiondir = optarg;
	    }
	  if (option_index==14)
	    {
	      string tmp = optarg;
	      FFIIParameters.insert(FFIIParameters.end(),tmp);
	    }
	  if (option_index==15)
	    {
	      string tmp = optarg;
	      FFIIValueParameters.insert(FFIIValueParameters.end(),tmp);
	    }
	  break;
	case 'v':
	  if (optarg!=0)
	    {
	      Verbosemode = atoi(optarg);
	      if (Verbosemode>=1)
		cout << "Verbose level : " << Verbosemode << endl;
	    }
	  else 
	    cerr << "Error - usage for option -v : -v x\n\tx is the verbosity level" << endl;
	  break;
	}	

    }


  try 
    {
      int largc = 3;
      char largv[3][1024] = { "LLVS","-ORBInitRef","NameService=corbaloc:iiop:localhost:5005/NameService"};
      char *lpargv[3];

      lpargv[0] = largv[0];
      lpargv[1] = largv[1];
      lpargv[2] = largv[2];
      if (NameServiceString.size()!=12)
	{
	  strcpy(largv[2],NameServiceString.c_str());
	}

      ODEBUG("Flag 0 " << largc << " " << lpargv[0] << " " << lpargv[1] << " " << lpargv[2]);
      CORBA::ORB_var orb = CORBA::ORB_init(largc, lpargv);
      CORBA::Object_var obj = orb->resolve_initial_references("RootPOA");

      ODEBUG("Flag 1");

      PortableServer::POA_var poa = PortableServer::POA::_narrow(obj);
      
      ODEBUG("Flag 1.5");
      aVS = new LowLevelVisionServer(InputType,SynchroType,filename,orb,Verbosemode,calibdir);
      ODEBUG("Flag 1.7");
      aVS->SetRobotVisionCalibrationDirectory(rbtvisiondir);
      poa->activate_object(aVS);
      ODEBUG("Flag 2");
      if (aVS!=0)
	{
	  GlobalVisionServer= aVS;
	  signal(SIGINT,SIGINT_handler);
	  signal(SIGSEGV,SIGINT_handler);
	  MainThread = pthread_self();

	  LowLevelVisionSystem_var aVSref = aVS->_this();

	  ODEBUG("Flag 3");
	  CORBA::String_var x;
	  x = orb->object_to_string(aVSref);
	  
	  if (aVS->GetVerboseMode()>=2)
	    cerr << x << "\n";
	  
	  if( !aVS->bindObjectToName(aVSref) )
	    {
	      cout << "The server did not succeed to connect to a Name Service," << endl
		   << "thus it will run in a stand-alone mode" << endl;
	    }

	  /* Initialize the calibration directory */
	  aVS->SetCalibrationDirectory(calibdir);
	  aVS->DisplayProcessesNames();

	  /* Initialize the disparity parameters */
	  if (DisparityParameters.size()!=DisparityValueParameters.size())
	    {
	      fprintf(stderr,"The disparity parameters difference of size between parameters and values! %d %d\n",
		      DisparityParameters.size(), DisparityValueParameters.size());
	    }
	  else
	    {
	      for(unsigned int i=0;i<DisparityParameters.size();i++)
		{
		  string ProcessName = "Disparity Map (VVV)";
		  aVS->SetAProcessParameterAndValue(ProcessName,
						    DisparityParameters[i],
						    DisparityValueParameters[i]);
		  
		}
	    }

	  ODEBUG3( OPFParameters.size() << " " << OPFValueParameters.size() );
	  /* Initialize the Optical flow parameters */
	  if (OPFParameters.size()!=OPFValueParameters.size())
	    {
	      fprintf(stderr,"The optical flow parameters difference of size between parameters and values! %d %d\n",
		      OPFParameters.size(), OPFValueParameters.size());
	    }
	  else
	    {
	      for(unsigned int i=0;i<OPFParameters.size();i++)
		{
		  string ProcessName = "Optical Flow and Harris detector Process";
		  aVS->SetAProcessParameterAndValue(ProcessName,
						    OPFParameters[i],
						    OPFValueParameters[i]);
		  
		}
	    }

	  ODEBUG3( FFIIParameters.size() << " " << FFIIValueParameters.size() );
	  /* Initialize the Find Features In Image parameters */
	  if (FFIIParameters.size()!=FFIIValueParameters.size())
	    {
	      fprintf(stderr,"The Find Feature In Image parameters difference of size between parameters and values! %d %d\n",
		      FFIIParameters.size(), FFIIValueParameters.size());
	    }
	  else
	    {
	      for(unsigned int i=0;i<FFIIParameters.size();i++)
		{
		  string ProcessName = "Find Features in image";
		  aVS->SetAProcessParameterAndValue(ProcessName,
						    FFIIParameters[i],
						    FFIIValueParameters[i]);
		  
		}
	    }

	  if (((InputType==LowLevelVisionSystem::FRAMEGRABBER) ||
	       (InputType==LowLevelVisionSystem::FILESINGLE) ||
	       (InputType==LowLevelVisionSystem::FILES))
	      && (bFGSize))
	    aVS->SetImagesGrabbedSize(FGWidth,FGHeight);
	  
	  aVS->SetCheckEntry(CheckEntry);
	  aVS->_remove_ref();
		  
	  PortableServer::POAManager_var pman = poa->the_POAManager();
	  pman->activate();

	  /* Thread creation */
	  {
	    pthread_attr_t Thread_Attr;
	    pthread_t aThread;

	    pthread_attr_init(&Thread_Attr);
	    pthread_create(&aThread, &Thread_Attr,LLVSThread, (void *)aVS);
	  }

	  orb->run();
	}


    }

  catch(CORBA::SystemException&) 
    {
      cerr << "MainEntryPoint::Caught CORBA::SystemException." << endl;

    }
  catch(CORBA::Exception&) 
    {
      cerr << "MainEntryPoint::Caught CORBA::Exception." << endl;
	    
    }
  catch(...) {
    cerr << NameServiceString << endl;
    cerr << "MainEntryPoint::Caught unknown exception." << endl;
	  
  }

 
  return 0;

}


