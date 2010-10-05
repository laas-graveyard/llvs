/** @doc This object implements the CORBA server providing
    Low Level Vision on the HRP-2 Vision processor.


    Copyright (c) 2003-2006,
    @author Olivier Stasse

    JRL-Japan, CNRS/AIST

    All rights reserved.

    See License.txt for more information on the license applied to this file.
*/
#include <iostream>

#include <cstdio>
#include <cstdlib>

#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <signal.h>
#include <getopt.h>

#include <boost/algorithm/string.hpp>
#include <boost/assign.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>

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
LowLevelVisionServer* GlobalVisionServer = 0;
PortableServer::POA_var poa;
PortableServer::ObjectId_var GlobalVisionServerID;
pthread_t LLVSThread_id;
pthread_mutex_t ExitApplicationMutex;
bool ServerBinded = false;

namespace po = boost::program_options;


LowLevelVisionSystem::InputMode toInputMode (const std::string& mode);
LowLevelVisionSystem::SynchroMode toSynchroMode (const std::string& mode);

void* LLVSThread(void *arg);
void SIGINT_handler(int signal_code);


/// \brief Convert a string representing an input mode to
/// the corresponding value of the InputMode enum.
LowLevelVisionSystem::InputMode toInputMode (const std::string& mode)
{
  static const std::map<std::string, LowLevelVisionSystem::InputMode> mapping
    = boost::assign::map_list_of
    ("firewire", LowLevelVisionSystem::FRAMEGRABBER)
    ("simulation", LowLevelVisionSystem::SIMULATION)
    ("file", LowLevelVisionSystem::FILESINGLE)
    ("files", LowLevelVisionSystem::FILES);

  typedef std::pair<std::string, LowLevelVisionSystem::InputMode> element_t;
  BOOST_FOREACH (element_t e, mapping)
    if (boost::iequals (e.first, mode))
      return e.second;
  throw "invalid input mode";
}

/// \brief Convert a string representing an synchro mode to
/// the corresponding value of the SynchroMode enum.
LowLevelVisionSystem::SynchroMode toSynchroMode (const std::string& mode)
{
  static const std::map<std::string, LowLevelVisionSystem::SynchroMode> mapping
    = boost::assign::map_list_of
    ("flow", LowLevelVisionSystem::SYNCHRO_FLOW)
    ("trigger", LowLevelVisionSystem::SYNCHRO_TRIGGER);

  typedef std::pair<std::string, LowLevelVisionSystem::SynchroMode> element_t;
  BOOST_FOREACH (element_t e, mapping)
    if (boost::iequals (e.first, mode))
      return e.second;
  throw "invalid synchro mode";
}




void* LLVSThread(void *arg)
{
  LowLevelVisionServer *aVS = (LowLevelVisionServer *)arg;

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
  ODEBUG ("Went through SIGINT_handler : "
	  << signal_code << " " << pthread_self() << " " << MainThread);
  if (pthread_self () == MainThread)
    {
      int rc = pthread_mutex_trylock (&ExitApplicationMutex);
      if(rc == EBUSY)
	{
	  // Trigger the application exit process
	  ODEBUG ("Trigger application exit process");
	  pthread_mutex_unlock (&ExitApplicationMutex);
	}
      else
	{
	  ODEBUG ("Normal exit failed. Exiting immediatly.");
	  exit (-1);
	}
    }
  else
    {
      ODEBUG("Go out from "<< pthread_self());
      pthread_exit(0);
    }
}

/// \brief This gathers the options values passed to the server.
struct Options
{
  /// \brief Input used for frame-grabbing (FireWire, OpenHRP
  /// simulation or disk).
  std::string inputType;

  /// \brief Synchronization mode (flow or trigger).
  std::string synchroType;

  std::string filename;
  std::string calibrationDirectory;
  std::string robotToVisionDirectory;

  unsigned frameGrabbingWidth;
  unsigned frameGrabbingHeight;

  /// CORBA name-service string.
  std::string nameService;

  bool checkEntry;

  /// \brief server global verbosity level.
  unsigned verbosity;
};

int main (int argc, char* argv[])
{
  // Init exit mutex
  pthread_mutex_init (&ExitApplicationMutex, 0);
  pthread_mutex_lock (&ExitApplicationMutex);

  LowLevelVisionServer* aVS = 0;

  Options options;
  po::options_description desc ("Allowed options");

  desc.add_options ()
    ("help,h", "produce help message")


    ("input,i",
     po::value<std::string> (&options.inputType)->default_value
     ("firewire"),
     "set the input mode (firewire, simulation, file, files)")

    ("synchro,s",
     po::value<std::string> (&options.synchroType)->default_value
     ("flow"),
     "set the frame-grabbing synchronization mode (trigger or flow)")


    ("width",
     po::value<unsigned> (&options.frameGrabbingWidth)->default_value
     (0),
     "set the image width for grabbed images")

    ("height",
     po::value<unsigned> (&options.frameGrabbingHeight)->default_value
     (0),
     "set the image height for grabbed images")


    ("calibration-directory",
     po::value<std::string> (&options.calibrationDirectory)->default_value
     (""),
     "set the calibration directory")

    ("rbt-vision-directory",
     po::value<std::string> (&options.robotToVisionDirectory)->default_value
     (""),
     "set the robot to vision directory")


    ("name-service,n",
     po::value<std::string> (&options.nameService)->default_value
     ("NameService="),
     "CORBA name-service string")


    ("check-entry",
     po::value<bool> (&options.checkEntry)->default_value (false),
     "FIXME")


    ("verbosity,v",
     po::value<unsigned> (&options.verbosity)->default_value (0),
     "set the verbosity level")
    ;

  po::variables_map vm;
  try
    {
      po::store (po::parse_command_line (argc, argv, desc), vm);
      po::notify (vm);
    }
  catch (po::error& error)
    {
      std::cerr << "Error while parsing argument: "
		<< error.what () << std::endl;
      return 1;
    }

  if (vm.count ("help"))
    {
      std::cout << desc << "\n";
      std::cout << "This program starts the low-level vision server.\n"
		<< "By default, it will start grabbing FireWire.\n"
		<< "\n"
		<< "Example:\n"
		<< "llvs-server"
		<< std::endl;
      return 0;
    }

  int largc = 1;
  char largv[1][14] = {"LLVS"};
  char* lpargv[1];
  lpargv[0] = largv[0];
  CORBA::ORB_var orb;
  CORBA::Object_var obj;

  std::cout << "Starting llvs-server..." << std::endl;

  try
    {
      orb = CORBA::ORB_init (largc, lpargv);
      obj = orb->resolve_initial_references("RootPOA");

      ODEBUG("Flag 1");

      poa = PortableServer::POA::_narrow(obj);

      ODEBUG("Flag 1.5");
    }
  catch(CORBA::SystemException&)
    {
      std::cerr << "MainEntryPoint::Caught CORBA::SystemException." << std::endl;
      return -1;

    }
  catch(CORBA::Exception&)
    {
      std::cerr << "MainEntryPoint::Caught CORBA::Exception." << std::endl;
      return -1;

    }
  catch(...) {
    std::cerr
      << "CORBA name-service string: " << options.nameService << std::endl
      << "MainEntryPoint::Caught unknown exception." << std::endl;
    return -1;
  }

  try
    {
      ODEBUG("Try to instanciate LLVS");
      aVS = new LowLevelVisionServer
	(toInputMode (options.inputType),
	 toSynchroMode (options.synchroType),
	 options.filename,
	 orb,
	 options.verbosity,
	 options.calibrationDirectory);
      ODEBUG3("LLVS is launched ");
    }
  //FIXME: We may choose an uniform way to throw exception to
  //       avoid this kind of copy/paste
  catch (const char* msg)
    {
      ODEBUG3 ("LowLevelVisionServer could not be instantiated");
      ODEBUG3 ("Reason:" << msg);
      ODEBUG3 ("Stopping now...");
      orb->destroy ();
      exit(-1);
    }
#if (LLVS_HAVE_VVV>0)
  catch (std::exception)
    {
      ODEBUG ("LowLevelVisionServer could not be instantiated");
      ODEBUG3 ("Reason:" << msg);
      ODEBUG3 ("Stopping now...");
      orb->destroy ();
      exit (-1);
    }
#endif
  ODEBUG("Flag 1.7");
  aVS->SetRobotVisionCalibrationDirectory (options.robotToVisionDirectory);
  GlobalVisionServerID = poa->activate_object(aVS);
  aVS->_remove_ref ();
  ODEBUG("Flag 2");
  if (aVS)
    {
      GlobalVisionServer = aVS;
      signal (SIGINT, SIGINT_handler);
      MainThread = pthread_self ();

      LowLevelVisionSystem_var aVSref = aVS->_this ();

      ODEBUG("Flag 3");
      CORBA::String_var x;
      x = orb->object_to_string (aVSref);

      if (aVS->GetVerboseMode() >= 2)
	std::cerr << x << "\n";

      if (!aVS->bindObjectToName (aVSref))
	{
	  std::cout
	    << "The server did not succeed to connect to a Name Service,\n"
	    << "thus it will run in a stand-alone mode" << std::endl;

	}
      else
	ServerBinded =true;

      /* Initialize the calibration directory */
      aVS->SetCalibrationDirectory (options.calibrationDirectory);
      aVS->SetCheckEntry(options.checkEntry);

      PortableServer::POAManager_var pman = poa->the_POAManager ();
      pman->activate ();

      /* Thread creation */
      {
	pthread_attr_t Thread_Attr;

	pthread_attr_init (&Thread_Attr);
	pthread_create (&LLVSThread_id, &Thread_Attr, LLVSThread, (void *)aVS);
	ODEBUG ("OmniORB thread:" << pthread_self ());
      }
#ifdef __ORBIX__
      orb->run()
#endif
	}

  // Wait for handler signal to quit the application
  pthread_mutex_lock (&ExitApplicationMutex);
  pthread_mutex_unlock (&ExitApplicationMutex);

  /* Stop the processes */
  ODEBUG ("Stop global vision server");
  GlobalVisionServer->StopMainProcess ();

  /* Stop llvs motor loop */
  ODEBUG ("Kill vision main loop");
  pthread_kill (LLVSThread_id, SIGINT);
  ODEBUG ("Wait for end of process");
  pthread_join (LLVSThread_id, 0);

  /* Clean Stop the processes */
  ODEBUG ("Clean up the frame grabbing ");
  GlobalVisionServer->CleanUpGrabbing ();
  ODEBUG ("Record images on disk");
  GlobalVisionServer->RecordImagesOnDisk (0);

  // Disconnect the object from CORBA
  ODEBUG ("Deactivate LLVS");
  poa->deactivate_object (GlobalVisionServerID.in ());

  std::cout << "LLVS exited normally." << std::endl;
  return 0;
}


