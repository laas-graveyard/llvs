/** @doc This object implements the CORBA server providing
    Low Level Vision on the HRP-2 Vision processor.


    Copyright (c) 2003-2006,
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
	  double headprpy[6];
	  double waistprpy[6];
#if 0
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
  aof.open((char *)aFilename.c_str(),ofstream::out);
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
      struct timeval ats;
	    

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
      struct timeval ats;
	    

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
      struct timeval ats;
	    

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
      struct timeval ats;
	    

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

void ConnectionToSot::ReaddComRefSignals(double waistcom[3])
{
  ODEBUG("Enter ReadComSignals ");

  try
    {
      struct timeval ats;
	    

      nsCorba::DoubleSeq_var DSwaistcom;
      m_SOT_Server_Command->readInputVectorSignal(m_dComRefSignalRank,
						  DSwaistcom);
      
      if (DSwaistcom->length()==3)
	for(unsigned int li=0;li<3;li++)
	  waistcom[li]= DSwaistcom[li];
    }
  catch(...)
    {
      cout << "Unable to read com ref control signals. "<< endl;
      
    }
  ODEBUG("Go out of  ReaddComRefSignals ");
}

void ConnectionToSot::ReaddComRefSignals(vector<double> &dcomref)
{
  ODEBUG("Enter ReadComSignals ");

  try
    {
      struct timeval ats;
	    

      nsCorba::DoubleSeq_var DSdcomref;
      m_SOT_Server_Command->readInputVectorSignal(m_dComRefSignalRank,
						  DSdcomref);
      
      if (DSdcomref->length()!=dcomref.size())
	dcomref.resize(DSdcomref->length());
      
      for(unsigned int li=0;li<DSdcomref->length();li++)
	dcomref[li]= DSdcomref[li];

      cout << "Size of dcomref: " << DSdcomref->length()
	   << " " << endl;
      for(unsigned int li=0;li<DSdcomref->length();li++)
	cout << " " << DSdcomref[li];
      cout << endl;
    }
  catch(...)
    {
      cout << "Unable to read dcomref control signals. "<< endl;
      
    }
  ODEBUG("Go out of  ReaddComRefSignals ");
}

void ConnectionToSot::ReadComAttitudeSignals(double comattitude[3])
{
  ODEBUG("Enter ReadComSignals ");

  try
    {
      struct timeval ats;
	    
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

  ODEBUG("Status: " << status);
  if (!status)
    return status;

  ODEBUG("Test");
  if( CORBA::is_nil(m_SOT_Server_Command) ) 
    {
      cerr << "Failed to narrow the root naming context." << endl;
      return false;
    }

  string CstSignaux[6]={"waistpositionabsolute",
			"waistattitudeabsolute",
			"Head",
			"Waist",
                        "dComRef",
                        "comattitudeabsolute"};

  string OutSignal[2] = {"VelRef",
                         "ObjectCoG"};
  ODEBUG("Before creating the signals: " << status);
  for(unsigned int li=0;li<6;li++)
    {

      /*      
      aCS->length(CstSignaux[li].size());
      for(unsigned int j=0;j<CstSignaux[li].size();j++)
      aCS[j] = CstSignaux[li][j];*/

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
      }
      catch(...)
	{
	  cerr << "Tried to create signal " << CstSignaux[li] << endl;
	  //exit(-1);
	}

      ODEBUG("Creation of signal: " << CstSignaux[li]);
    }

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
	  //exit(-1);
	}
    }
  
  ODEBUG("After creating the signals: " );
  
#if 0
#define NBCOMMANDS 4
  string SotCommand[4]= {
    "plug pg.waistpositionabsolute coshell.waistpositionabsolute",
    "plug pg.waistattitudeabsolute coshell.waistattitudeabsolute",
    "OpenHRP.periodicCall addSignal pg.waistpositionabsolute",
    "OpenHRP.periodicCall addSignal pg.waistattitudeabsolute"};
#else
#define NBCOMMANDS 24
  string SotCommand[NBCOMMANDS]= {
    "coshell.buffer dComRef 30",
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
    "plug coshell.VelRef pg.velocitydes",
    "new Stack<vector> dComRefPTime",
    "plug pg.dcomref dComRefPTime.in1",
    "plug OpenHRP.time dComRefPTime.in2",
    "dComRefPTime.selec1 0 3",
    "dComRefPTime.selec2 0 1",
    "OpenHRP.periodicCall addSignal dComRefPTime.out",
    "OpenHRP.periodicCall addSignal coshell.synchro",
    "plug dComRefPTime.out coshell.dComRef",
    "OpenHRP.periodicCall addSignal pg.comattitude",
    "plug pg.comattitude coshell.comattitudeabsolute",
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
