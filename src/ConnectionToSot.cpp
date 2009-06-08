#include <iostream>
#include <fstream>
#include "LowLevelVisionServer.h"
#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "ConnectionToSot:" << x << endl

#if 0
#define ODEBUG(x) cerr << "ConnectionToSot:" <<  x << endl
#else
#define ODEBUG(x) 
#endif

#include "ConnectionToSot.h"


void * ConnectionToSotThread(void *arg)
{
  llvs::ConnectionToSot *aCST = (llvs::ConnectionToSot *)arg;

  if (aCST!=0)
    {
      while(!aCST->GetEndOfThreadLoop())
	{
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
		 << waistattitude[2] << " ) ( "); 
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
  delete m_CircularBuffer;

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
  {
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
  std::string lServiceName = "coshell", lServiceKind="";
  CORBA::Object_ptr obj = m_LLVS->getObjectReference(lServiceName,lServiceKind);
  ODEBUG( "Able to get the reference for :" << lServiceName << " " 
	  << lServiceKind );

  if (CORBA::is_nil(obj))
   {
      cerr << "Unable to find object: " << lServiceName << " " << lServiceKind <<endl;
      return false;
    }

  ODEBUG( "Before narrowing");
  
  try
    {
      m_SOT_Server_Command = SOT_Server_Command::_narrow(obj);
    }
  catch(...)
    {
      cerr << "Unable to narrow :" << lServiceName << " " << lServiceKind << "CORBAReference : "<< m_SOT_Server_Command;
      return false;
    }

  ODEBUG( "After narrowing");
  
  return true;
}

void ConnectionToSot::ReadWaistSignals(double waistposition[3],
				  double waistattitude[3])
{
  ODEBUG("Enter ReadWaistSignals ");

  try
    {
      struct timeval ats;
	    

      SOT_Server_Command::DoubleSeq_var DSwaistpos, DSwaistatt;
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
      cout << "Unable to connect to Sot. "<< endl;
      
    }
  ODEBUG("Go out of  ReadWaistSignals ");
}

bool ConnectionToSot::Init()
{
  bool status;
  status = SetCorbaReference();
  
  if (!status)
    return status;

  SOT_Server_Command::CharSeq_var aCS= new SOT_Server_Command::CharSeq;

  string CstSignaux[2]={"waistpositionabsolute","waistattitudeabsolute"};

  for(unsigned int li=0;li<2;li++)
    {
      
      aCS->length(CstSignaux[li].size());
      for(unsigned int j=0;j<CstSignaux[li].size();j++)
	aCS[j] = CstSignaux[li][j];
      
      if (li==0)
	m_WaistPositionSignalRank = m_SOT_Server_Command->createInputVectorSignal(aCS);
      else 
	m_WaistAttitudeSignalRank = m_SOT_Server_Command->createInputVectorSignal(aCS);
    }

  
#if 0
  string SotCommand[4]= {
    "plug pg.waistpositionabsolute coshell.waistpositionabsolute",
    "plug pg.waistattitudeabsolute coshell.waistattitudeabsolute",
    "OpenHRP.periodicCall addSignal pg.waistpositionabsolute",
    "OpenHRP.periodicCall addSignal pg.waistattitudeabsolute"};
#else
  string SotCommand[4]= {
    "plug ffposition_from_pg.out coshell.waistpositionabsolute",
    "plug ffattitude_from_pg.out coshell.waistattitudeabsolute",
    "OpenHRP.periodicCall addSignal ffposition_from_pg.out",
    "OpenHRP.periodicCall addSignal ffattitude_from_pg.out"};
#endif

  for(unsigned int li=0;li<4;li++)
    {

      aCS->length(SotCommand[li].size());
      for(unsigned int j=0;j<SotCommand[li].size();j++)
	aCS[j] = SotCommand[li][j];

      try 
	{ 

	  SOT_Server_Command::StringStreamer_var CoshellOutput;
	  m_SOT_Server_Command->runAndRead(aCS,CoshellOutput); 
	  
	  string lans;
	  lans.resize(CoshellOutput->length());
	  for(unsigned int i=0;i<CoshellOutput->length();i++)
	    { lans[i]=CoshellOutput[i]; }

	}
      catch(...)
	{
	  ODEBUG3("Unable to contact the sot server.");
	  return false;
	}
    }

  return true;
}
