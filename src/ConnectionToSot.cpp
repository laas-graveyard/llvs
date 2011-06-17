/** @doc This object implements the CORBA server providing
    Low Level Vision on the HRP-2 Vision processor.


    Copyright (c) 2003-2010,
    @author Olivier Stasse

    JRL-Japan, CNRS/AIST

    All rights reserved.
    See license.txt for more information
*/
#include <iostream>
#include <fstream>
#include "LowLevelVisionServer.h"
#include "ConnectionToSot.h"

// Debug macros
#include <llvs/tools/Debug.h>

void * ConnectionToSotThread(void *arg)
{
  llvs::ConnectionToSot *aCST = (llvs::ConnectionToSot *)arg;

  if (aCST!=0)
    {
      while(!aCST->GetEndOfThreadLoop())
	{
#if 0
	  double waistposition[3];
	  double waistattitude[3];
	    
	  aCST->ReadWaistSignals(waistposition,
				 waistattitude);

	  usleep(23000);
	  ODEBUG3("Starting again. ( " 
		 << waistposition[0] << " , "
		 << waistposition[1] << " , "
		 << waistposition[2] << " ) ( "
		 << waistattitude[0] << " , "
		 << waistattitude[1] << " , "
		 << waistattitude[2] << " ) "); 
#else
#if 0
	  double headprpy[6];
	  double waistprpy[6];

	  aCST->ReadHeadRPYSignals(headprpy);
	  aCST->ReadWaistRPYSignals(waistprpy);
#endif 
	  usleep(5000);
#if 0
	  ODEBUG3("headprpy ( " 
		 << headprpy[0] << " , "
		 << headprpy[1] << " , "
		 << headprpy[2] << " ) ( "
		 << headprpy[3] << " , "
		 << headprpy[4] << " , "
		 << headprpy[5] << " ) "); 
	  
	  ODEBUG3("waistprpy ( " 
		 << waistprpy[0] << " , "
		 << waistprpy[1] << " , "
		 << waistprpy[2] << " ) ( "
		 << waistprpy[3] << " , "
		 << waistprpy[4] << " , "
		 << waistprpy[5] << " ) "); 
#endif	  

#endif
	}
    }
  ODEBUG("Went out of the thread.");
  return 0;
}

using namespace llvs;

ConnectionToSot::ConnectionToSot(LowLevelVisionServer *aLLVS)
{
  m_LLVS = aLLVS;
  m_EndOfThreadLoop = false;

  m_CircularBuffer = new double[33*7*3600];
  if (m_CircularBuffer!=0)
    ODEBUG3("Mem alloc done.");

  m_CircularBufferIndex = 0;
  m_CircularBufferIndexMax = 33*3600;
}

void ConnectionToSot::DumpCircularBuffer(string aFilename)
{
  ODEBUG("Dumping data: "<<m_CircularBufferIndexMax);
  ofstream aof;
  aof.open((const char *)aFilename.c_str(),ofstream::out);
  aof.precision(40);
  for(unsigned int i=0;i<m_CircularBufferIndexMax;i++)
    {
      aof << m_CircularBuffer[i]<< " ";
      if ((i%7==0) &&(i!=0))
	aof <<endl;
    }
  aof.close();
}

ConnectionToSot::~ConnectionToSot()
{
  delete [] m_CircularBuffer ;

}

bool ConnectionToSot::GetEndOfThreadLoop() const 
{
  return m_EndOfThreadLoop;
}

void ConnectionToSot::StartThreadOnConnectionSot()
{
  pthread_t aThread;
  m_EndOfThreadLoop = false;
  /* Thread creation */
  if (0) {
    pthread_attr_t Thread_Attr;
    
    pthread_attr_init(&Thread_Attr);
    pthread_create(&aThread, &Thread_Attr,ConnectionToSotThread, (void *)this);
    
    ODEBUG("OmniORB thread:" << pthread_self());
  }

}

void ConnectionToSot::StopThreadOnConnectionSot()
{
  m_EndOfThreadLoop = true;
}


bool ConnectionToSot::SetCorbaReference()
{
  std::vector<std::string> lServiceName,lServiceKind;
  lServiceName.resize(2);
  lServiceKind.resize(2);
  lServiceName[0]= "sot";lServiceName[1]= "coshell";
  lServiceKind[0]="context";lServiceKind[1]="servant";
  
  CORBA::Object_ptr obj = m_LLVS->getObjectReference(lServiceName,lServiceKind);
  ODEBUG( "Able to get the reference for :" << lServiceName[0] << " " 
	  << lServiceKind[0] );

  if (CORBA::is_nil(obj))
   {
      cerr << "Unable to find object: " << lServiceName[0] << " " << lServiceKind[0] <<endl;
      return false;
    }

  ODEBUG( "Before narrowing");
  
  try
    {
      m_SOT_Server_Command = nsCorba::SOT_Server_Command::_narrow(obj);
    }
  catch(...)
    {
       cerr << "Unable to narrow :" << lServiceName[0]
	   << " " << lServiceKind[0] 
	   << "CORBAReference : "<< m_SOT_Server_Command;
      return false;
    }

  if (CORBA::is_nil(m_SOT_Server_Command))
    {
      ODEBUG3("Unable to narrow :" << lServiceName[0]
	      << " " << lServiceKind[0] 
	      << "CORBAReference : "<< m_SOT_Server_Command);
      exit(0);
    }
  ODEBUG( "After narrowing");
  
  return true;
}

void ConnectionToSot::WriteVelocityReference(double velref[3])
{
  ODEBUG("Enter WriteVelocityReference ");

  try
    {
      nsCorba::DoubleSeq_var DSvelref;
      ODEBUG("Enter WriteVelocityReference 1 ");
      DSvelref = new nsCorba::DoubleSeq;
      DSvelref->length(3);
      ODEBUG("Enter WriteVelocityReference 2 ");
      for(unsigned int li=0;li<3;li++)
	DSvelref[li]= velref[li];
      ODEBUG("Enter WriteVelocityReference 3 " << m_VelRefSignalRank);
      m_SOT_Server_Command->writeOutputVectorSignal(m_VelRefSignalRank,
						    DSvelref);      

    }
  catch(...)
    {
      cout << "Unable to write signal of rank . " << m_VelRefSignalRank<< endl;
      
    }
  ODEBUG("Go out of WriteVelocityReference ");

}

void ConnectionToSot::WriteObjectCoG(double ObjectCoG[3])
{
  ODEBUG("Enter WriteObjectCoG ");

  try
    {
      nsCorba::DoubleSeq_var DSObjectCoG;
      ODEBUG("Enter WriteObjectCoG 1 ");
      DSObjectCoG = new nsCorba::DoubleSeq;
      DSObjectCoG->length(3);
      ODEBUG("Enter WriteObjectCoG 2 ");
      for(unsigned int li=0;li<3;li++)
	DSObjectCoG[li]= ObjectCoG[li];
      ODEBUG("Enter WriteObjectCoG 3 " << m_ObjectCoGSignalRank);
      m_SOT_Server_Command->writeOutputVectorSignal(m_ObjectCoGSignalRank,
						    DSObjectCoG);      

    }
  catch(...)
    {
      cout << "Unable to write signal of rank . " << m_ObjectCoGSignalRank<< endl;
      
    }
  ODEBUG("Go out of WriteObjectCoG");

}

void ConnectionToSot::ReadWaistSignals(double waistposition[3],
				       double waistattitude[3])
{
  ODEBUG("Enter ReadWaistSignals ");

  try
    {
      struct timeval ats;
	    

      nsCorba::DoubleSeq_var DSwaistpos, DSwaistatt;
      m_SOT_Server_Command->readInputVectorSignal(m_WaistPositionSignalRank,
						  DSwaistpos);
      
      m_SOT_Server_Command->readInputVectorSignal(m_WaistAttitudeSignalRank,
						  DSwaistatt);

      gettimeofday(&ats,0);
      m_CircularBuffer[m_CircularBufferIndex++] = (double)ats.tv_sec + 0.000001 * (double)ats.tv_usec;


      if (DSwaistpos->length()==3)
	{
	  for(unsigned li=0;li<3;li++)
	    {
	      waistposition[li] = 
		m_CircularBuffer[m_CircularBufferIndex++] = DSwaistpos[li];
	    }
	}

      if (DSwaistatt->length()==3)
	{
	  for(unsigned li=0;li<3;li++)
	    {
	      waistattitude[li] = 
		m_CircularBuffer[m_CircularBufferIndex++] = DSwaistatt[li];
	    }
	}

     
      if (m_CircularBufferIndex>=m_CircularBufferIndexMax)
	m_CircularBufferIndex = 0;

    }
  catch(...)
    {
      cout << "Unable to read waist signals. "<< endl;
      
    }
  ODEBUG("Go out of  ReadWaistSignals ");
}

void ConnectionToSot::ReadHeadRPYSignals(double headposerpy[6])
{
  ODEBUG("Enter ReadHeadRPYSignals ");

  try
    {
      nsCorba::DoubleSeq_var DShead;
      m_SOT_Server_Command->readInputVectorSignal(m_HeadPRPYSignalRank,
						  DShead);
      
      if (DShead->length()==6)
	{
	  for(unsigned int li=0;li<6;li++)
	    {
	      headposerpy[li]= DShead[li];
	    }
	}
      ODEBUG("Head Pose-RPY: " << 
	      DShead[0] << " " <<
	      DShead[1] << " " <<
	      DShead[2] << " " <<
	      DShead[3] << " " <<
	      DShead[4] << " " <<
	      DShead[5] );
    }
  catch(...)
    {
      cout << "Unable to read head rpy signals. "<< endl;
    }
  ODEBUG("Go out of ReadHeadRPYSignals ");
}


void ConnectionToSot::ReadWaistRPYSignals(double waistposerpy[6])
{
  ODEBUG("Enter ReadWaistRPYSignals ");

  try
    {

      nsCorba::DoubleSeq_var DSwaist;
      m_SOT_Server_Command->readInputVectorSignal(m_WaistPRPYSignalRank,
						  DSwaist);
      
      if (DSwaist->length()==6)
	for(unsigned int li=0;li<6;li++)
	  waistposerpy[li]= DSwaist[li];
    }
  catch(...)
    {
      cout << "Unable to read waist rpy signals. "<< endl;
      
    }
  ODEBUG("Go out of  ReadWaistRPYSignals ");
}

void ConnectionToSot::ReaddComRefSignals(double dcom[3])
{
  ODEBUG("Enter ReadComSignals ");

  try
    {
      nsCorba::DoubleSeq_var DSdcom;
      m_SOT_Server_Command->readInputVectorSignal(m_dComRefSignalRank,
						  DSdcom);
      
      if (DSdcom->length()==3)
	for(unsigned int li=0;li<3;li++)
	  dcom[li]= DSdcom[li];
      else 
	{
	  dcom[0] = 
	    dcom[1] = 
	    dcom[2] = 0.0;
	  double referencetime= DSdcom[0];

	  for(unsigned int li=0;li<DSdcom->length();li+=4)
	    {
	      if (referencetime < DSdcom[li])
		{
		  dcom[0] = DSdcom[li+1];
		  dcom[1] = DSdcom[li+2];
		  dcom[2] = DSdcom[li+3];
		}
	    }
	}
    }
  catch(...)
    {
      cout << "Unable to read com ref control signals. "<< endl;
      
    }
  ODEBUG("Go out of  ReaddComRefSignals ");
}

void ConnectionToSot::ReaddComRefSignals(vector<double> &dcomref)
{
  ReadDataRefSignal(m_dComRefSignalRank,dcomref);
}

void ConnectionToSot::ReaddComAttRefSignals(double dcomatt[3])
{
  try
    {
      nsCorba::DoubleSeq_var DSdcomatt;
      m_SOT_Server_Command->readInputVectorSignal(m_dComAttRefSignalRank,
						  DSdcomatt);
      
      if (DSdcomatt->length()==3)
	for(unsigned int li=0;li<3;li++)
	  dcomatt[li]= DSdcomatt[li];
      else 
	{
	  dcomatt[0] = 
	    dcomatt[1] = 
	    dcomatt[2] = 0.0;
	  double referencetime= DSdcomatt[0];

	  for(unsigned int li=0;li<DSdcomatt->length();li+=4)
	    {
	      if (referencetime < DSdcomatt[li])
		{
		  dcomatt[0] = DSdcomatt[li+1];
		  dcomatt[1] = DSdcomatt[li+2];
		  dcomatt[2] = DSdcomatt[li+3];
		}
	    }
	}
    }
  catch(...)
    {
      cout << "Unable to read com att ref control signals. "<< endl;
      
    } 
}

void ConnectionToSot::ReaddComAttRefSignals(vector<double> & dcomattref)
{
  ReadDataRefSignal(m_dComAttRefSignalRank,dcomattref);
}

void ConnectionToSot::ReadDataRefSignal(CORBA::Long & refToSignal,
					vector<double> &dataref)
{
  ODEBUG("Enter ReadDataRefSignals ");

  try
    {
      nsCorba::DoubleSeq_var DSdataref;
      m_SOT_Server_Command->readInputVectorSignal(refToSignal,
						  DSdataref);
      
      if (DSdataref->length()!=dataref.size())
	dataref.resize(DSdataref->length());
      
      for(unsigned int li=0;li<DSdataref->length();li++)
	dataref[li]= DSdataref[li];

      cout << "refToSignal: " << refToSignal
	   << " Size of dataref: " << DSdataref->length()
	   << " " << endl;
      for(unsigned int li=0;li<DSdataref->length();li++)
	cout << " " << DSdataref[li];
      cout << endl;
    }
  catch(...)
    {
      cout << "Unable to read dataref for control signal :"
	   << refToSignal << " ."
	   << endl;
      exit(1);
    }
  ODEBUG("Go out of  ReaddatarefSignals ");
}

void ConnectionToSot::ReadActivationCompensationSignal(double &value)
{
  ODEBUG("Enter ReadDataRefSignals ");

  try
    {
      nsCorba::DoubleSeq_var DSdataref;
      m_SOT_Server_Command->readInputVectorSignal(m_compensationActivationSignalRank,
						  DSdataref);
      
      if (DSdataref->length()==1)
	value= DSdataref[0];
    }
  catch(...)
    {
      cout << "Unable to read dataref for control signal read activation compensation"
	   << endl;
      exit(1);
    }
  ODEBUG("Go out of  ReaddatarefSignals ");
  
}

void ConnectionToSot::ReadComAttitudeSignals(double comattitude[3])
{
  ODEBUG("Enter ReadComSignals ");

  try
    {
      nsCorba::DoubleSeq_var DScomattitude;
      m_SOT_Server_Command->readInputVectorSignal(m_ComAttitudeSignalRank,
						  DScomattitude);
      
      if (DScomattitude->length()==3)
	for(unsigned int li=0;li<3;li++)
	  comattitude[li]= DScomattitude[li];
    }
  catch(...)
    {
      cout << "Unable to read com attitude signals. "<< endl;
      
    }
  ODEBUG("Go out of  ReadComAttitudeSignals ");
}





bool ConnectionToSot::Init()
{
  bool status;
  status = SetCorbaReference();

  status = status & InitInputSignals();

  status = status & InitOutputSignals();

  status = status & InitExecuteCommands();

  ODEBUG("Status: " << status);
  if (!status)
    return status;

  ODEBUG("Test");
  if( CORBA::is_nil(m_SOT_Server_Command) ) 
    {
      cerr << "Failed to narrow the root naming context." << endl;
      return false;
    }
}

bool ConnectionToSot::InitInputSignals()
{
  string CstSignaux[8]={"waistpositionabsolute",
			"waistattitudeabsolute",
			"Head",
			"Waist",
                        "dComRef",
                        "comattitudeabsolute",
                        "dComAttRef",
                        "compensationActivation"};

  // Creating the input signals.
  for(unsigned int li=0;li<8;li++)
    {

      try{

	if (li==0)
	  m_WaistPositionSignalRank = m_SOT_Server_Command->createInputVectorSignal(CstSignaux[li].c_str());

	else if (li==1)
	  m_WaistAttitudeSignalRank = m_SOT_Server_Command->createInputVectorSignal(CstSignaux[li].c_str());

	else if (li==2)
	  m_HeadPRPYSignalRank = m_SOT_Server_Command->createInputVectorSignal(CstSignaux[li].c_str());

	else if (li==3)
	  m_WaistPRPYSignalRank = m_SOT_Server_Command->createInputVectorSignal(CstSignaux[li].c_str());

	else if (li==4)
	  m_dComRefSignalRank = m_SOT_Server_Command->createInputVectorSignal(CstSignaux[li].c_str());

	else if (li==5)
	  m_ComAttitudeSignalRank = m_SOT_Server_Command->createInputVectorSignal(CstSignaux[li].c_str());

	else if (li==6)
	  m_dComAttRefSignalRank = m_SOT_Server_Command->createInputVectorSignal(CstSignaux[li].c_str());

	else if (li==7)
	  m_compensationActivationSignalRank = 
	    m_SOT_Server_Command->createInputVectorSignal(CstSignaux[li].c_str());

      }
      catch(...)

	{
	  cerr << "Tried to create signal " << CstSignaux[li] << endl;
	  return false;
	}

      ODEBUG("Creation of signal: " << CstSignaux[li]);
    }
  return true;
}

bool ConnectionToSot::InitOutputSignals()
{
  string OutSignal[2] = {"VelRef",
                         "ObjectCoG"};

  for(unsigned int li=0;li<2;li++)
    {
      
      try
	{
	  if (li==0)
	    m_VelRefSignalRank = m_SOT_Server_Command->createOutputVectorSignal(OutSignal[li].c_str());
	  else if (li==1)
	    m_ObjectCoGSignalRank = m_SOT_Server_Command->createOutputVectorSignal(OutSignal[li].c_str());
	  
	  ODEBUG("Creation of signal: " << OutSignal[0] << " " << m_VelRefSignalRank);
	  
	  {
	    double velref[3] = {0.0,0.0,0.0};
	    WriteVelocityReference(velref);
	  }
	  
	}
      catch(...)
	{
	  cerr << "Tried to create signal " << OutSignal[li] << endl;
	}
    }
  
  ODEBUG("After creating the signals: " );
  
}


bool ConnectionToSot::InitExecuteCommands()
{
#if 0
#define NBCOMMANDS 4
  string SotCommand[4]= {
    "plug pg.waistpositionabsolute coshell.waistpositionabsolute",
    "plug pg.waistattitudeabsolute coshell.waistattitudeabsolute",
    "OpenHRP.periodicCall addSignal pg.waistpositionabsolute",
    "OpenHRP.periodicCall addSignal pg.waistattitudeabsolute"};
#else
#define NBCOMMANDS 23
  string SotCommand[NBCOMMANDS]= {
    "set coshell.compensationActivation [1](0.0)",
    "OpenHRP.periodicCall addSignal pg.comattitude",
    "OpenHRP.periodicCall addSignal pg.dcomattitude",
    "plug ffposition_from_pg.out coshell.waistpositionabsolute",
    "plug ffattitude_from_pg.out coshell.waistattitudeabsolute",
    "OpenHRP.periodicCall addSignal ffposition_from_pg.out",
    "OpenHRP.periodicCall addSignal ffattitude_from_pg.out",
    "new MatrixHomoToPoseRollPitchYaw dhhtp",
    "plug dyn.head dhhtp.in",
    "plug dhhtp.out coshell.Head",
    "new MatrixHomoToPoseRollPitchYaw dwhtp",
    "plug dyn.Waist dwhtp.in",
    "plug dwhtp.out coshell.Waist",
    "OpenHRP.periodicCall addSignal dhhtp.out",
    "OpenHRP.periodicCall addSignal dwhtp.out",
    "plug pg.dcomref coshell.dComRef",
    "plug pg.dcomattitude coshell.dComAttRef",
    "plug coshell.VelRef pg.velocitydes",
    "coshell.buffer dComRef 30",
    "coshell.buffer dComAttRef 30",
    "OpenHRP.periodicCall addSignal coshell.synchro",
    "plug pg.comattitude coshell.comattitudeabsolute",
    "plug OpenHRP.state coshell.position",
  };
#endif

  for(unsigned int li=0;li<NBCOMMANDS;li++)
    {

      try 
	{ 

	  nsCorba::StringStreamer_var CoshellOutput;
	  m_SOT_Server_Command->runAndRead(SotCommand[li].c_str(),CoshellOutput); 
	  ODEBUG3("Launched " << SotCommand[li].c_str());
	  string lans;
	  lans.resize(CoshellOutput->length());
	  for(unsigned int i=0;i<CoshellOutput->length();i++)
	    { lans[i]=CoshellOutput[i]; }
	  ODEBUG3("Out " << lans);

	}
      catch(...)
	{
	  ODEBUG3("Unable to contact the sot server.");
	  return false;
	}
    }
  
  ODEBUG("Launched all the commands");
  
  return true;
}
