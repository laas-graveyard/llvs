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

// Debug macros
#include <llvs/tools/Debug.h>
#include "LowLevelVisionServer.h"

using namespace llvs;
pthread_t MainThread;
LowLevelVisionServer *GlobalVisionServer = NULL;
PortableServer::POA_var poa;
PortableServer::ObjectId_var GlobalVisionServerID;
pthread_t LLVSThread_id;
pthread_mutex_t ExitApplicationMutex;
bool ServerBinded=false;

void * LLVSThread(void *arg)
{
  LowLevelVisionServer *aVS = (LowLevelVisionServer *)arg;

  double current_time;
  double mean=0;

  ODEBUG("LLVS thread: " << pthread_self());
  // Start to process.
  aVS->StartMainProcess();

  // Vision Main loop.
  for(;;)
    {
      aVS->ApplyingProcess();
    }
  return 0;
}


void SIGINT_handler(int signal_code)
{
  ODEBUG("Went through SIGINT_handler : "<< signal_code << " " << pthread_self() << " " << MainThread);
  if(pthread_self() == MainThread)
    {
      int rc = pthread_mutex_trylock(&ExitApplicationMutex);
      if(rc == EBUSY)
	{
	  // Trigger the application exit process
	  ODEBUG("Trigger application exit process");
	  pthread_mutex_unlock(&ExitApplicationMutex);
	}
      else
	{
	  ODEBUG("Normal exit failed. Exiting immediatly.");
	  exit(-1);
	}
    }
  else
    {
      ODEBUG("Go out from "<< pthread_self());
      pthread_exit(0);
    }
}

int main(int argc, char * argv[])
{
  // Init exit mutex
  pthread_mutex_init(&ExitApplicationMutex, NULL);
  pthread_mutex_lock(&ExitApplicationMutex);

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

  int largc = 1;
  char largv[1][14] = { "LLVS"};
  char *lpargv[1];
  lpargv[0] = largv[0];
  CORBA::ORB_var orb;
  CORBA::Object_var obj;

  try 
    {
      orb = CORBA::ORB_init(largc, lpargv);
      obj = orb->resolve_initial_references("RootPOA");

      ODEBUG("Flag 1");

      poa = PortableServer::POA::_narrow(obj);

      ODEBUG("Flag 1.5");
    }
  catch(CORBA::SystemException&) 
    {
      cerr << "MainEntryPoint::Caught CORBA::SystemException." << endl;
      exit(-1);

    }
  catch(CORBA::Exception&) 
    {
      cerr << "MainEntryPoint::Caught CORBA::Exception." << endl;
      exit(-1);

    }
  catch(...) {
    cerr << NameServiceString << endl;
    cerr << "MainEntryPoint::Caught unknown exception." << endl;
    exit(-1);
  }

  try
    {
      aVS = new LowLevelVisionServer(InputType,SynchroType,filename,orb,Verbosemode,calibdir);
    }
  //FIXME: We may choose an uniform way to throw exception to
  //       avoid this kind of copy/paste
  catch( const char* msg )
    {
      ODEBUG("LowLevelVisionServer could not be instantiated");
      ODEBUG3("Reason:" << msg);
      ODEBUG3("Stopping now...");
      orb->destroy();
      exit(-1);
    }
#if (LLVS_HAVE_VVV>0)
  catch( std::exception )
    {
      ODEBUG("LowLevelVisionServer could not be instantiated");
      ODEBUG3("Reason:" << msg);
      ODEBUG3("Stopping now...");
      orb->destroy();
      exit(-1);
    }
#endif
  ODEBUG("Flag 1.7");
  aVS->SetRobotVisionCalibrationDirectory(rbtvisiondir);
  GlobalVisionServerID = poa->activate_object(aVS);
  aVS->_remove_ref();
  ODEBUG("Flag 2");
  if (aVS!=0)
    {
      GlobalVisionServer= aVS;
      signal(SIGINT,SIGINT_handler);
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
      else 
	ServerBinded =true;

      /* Initialize the calibration directory */
      aVS->SetCalibrationDirectory(calibdir);
      //aVS->DisplayProcessesNames();

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

      ODEBUG( OPFParameters.size() << " " << OPFValueParameters.size() );
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

      ODEBUG( FFIIParameters.size() << " " << FFIIValueParameters.size() );
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

      aVS->SetCheckEntry(CheckEntry);

      PortableServer::POAManager_var pman = poa->the_POAManager();
      pman->activate();

      /* Thread creation */
      {
	pthread_attr_t Thread_Attr;

	pthread_attr_init(&Thread_Attr);
	pthread_create(&LLVSThread_id, &Thread_Attr,LLVSThread, (void *)aVS);

	ODEBUG("OmniORB thread:" << pthread_self());
      }
#ifdef __ORBIX__
      orb->run()
#endif
	}

  // Wait for handler signal to quit the application
  pthread_mutex_lock(&ExitApplicationMutex);
  pthread_mutex_unlock(&ExitApplicationMutex);

  /* Stop the processes */
  ODEBUG("Stop global vision server");
  GlobalVisionServer->StopMainProcess();

  /* Stop llvs motor loop */
  ODEBUG("Kill vision main loop");
  pthread_kill(LLVSThread_id, SIGINT);
  ODEBUG("Wait for end of process");
  pthread_join(LLVSThread_id, 0);

  /* Clean Stop the processes */
  ODEBUG("Clean up the frame grabbing ");
  GlobalVisionServer->CleanUpGrabbing();
  ODEBUG("Record images on disk");
  GlobalVisionServer->RecordImagesOnDisk(0);

  // Disconnect the object from CORBA
  ODEBUG("Deactivate LLVS");
  poa->deactivate_object(GlobalVisionServerID.in());

  cout << "LLVS exited normally." << endl;
  return 0;
}


