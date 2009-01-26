#include <iostream>
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
      aCST->SetCorbaReference();
      aCST->Init();
      
      while(!aCST->GetEndOfThreadLoop())
	{
	  double waistposition[3];
	  double waistattitude[3];
	    
	  aCST->ReadWaistSignals(waistposition,
				 waistattitude);
	  usleep(33000);
	}
    }
  return 0;
}

using namespace llvs;

ConnectionToSot::ConnectionToSot(LowLevelVisionServer *aLLVS)
{
  m_LLVS = aLLVS;
  m_EndOfThreadLoop = false;
}

ConnectionToSot::~ConnectionToSot()
{

}

bool ConnectionToSot::GetEndOfThreadLoop() const 
{
  return m_EndOfThreadLoop;
}

void ConnectionToSot::StartThreadOnConnectionSot()
{
  pthread_t aThread;
  m_EndOfThreadLoop = true;
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
  m_EndOfThreadLoop = false;
}


bool ConnectionToSot::SetCorbaReference()
{
  std::string lServiceName = "coshell", lServiceKind="";
  CORBA::Object_ptr obj = m_LLVS->getObjectReference(lServiceName,lServiceKind);
  ODEBUG( "Able to get the reference for :" << lServiceName << " " 
	  << lServiceKind << "CORBAReference : "<< lCORBAReference);

  if (CORBA::is_nil(obj))
   {
      cerr << "Unable to find object: " << lServiceName << " " << lServiceKind <<endl;
      return false;
    }
  
  try
    {
      m_SOT_Server_Command = SOT_Server_Command::_narrow(obj);
    }
  catch(...)
    {
      cerr << "Unable to narrow :" << lServiceName << " " << lServiceKind << "CORBAReference : "<< m_SOT_Server_Command;
      return false;
    }

  
  return true;
}

void ConnectionToSot::ReadWaistSignals(double waistposition[3],
				  double waistattitude[3])
{
  try
    {
      SOT_Server_Command::DoubleSeq_var DSwaistpos, DSwaistatt;
      m_SOT_Server_Command->readInputVectorSignal(m_WaistPositionSignalRank,
						  DSwaistpos);
      
      m_SOT_Server_Command->readInputVectorSignal(m_WaistAttitudeSignalRank,
						  DSwaistatt);
      if (DSwaistpos->length()==3)
	{
	  for(unsigned li=0;li<3;li++)
	    {
	      waistposition[li] = DSwaistpos[li];
	      cout << waistposition[li] << " ";
	    }
	}

      if (DSwaistatt->length()==3)
	{
	  for(unsigned li=0;li<3;li++)
	    {
	      waistattitude[li] = DSwaistatt[li];
	      cout << waistattitude[li] << " ";
	    }
	}
    }
  catch(...)
    {
      cout << "Unable to connect to Sot. "<< endl;
      
    }
}

bool ConnectionToSot::Init()
{
  bool status;
  status = SetCorbaReference();
  
  if (!status)
    return status;

  SOT_Server_Command::CharSeq_var aCS= new SOT_Server_Command::CharSeq;

  string CstSignaux[2]={"waistposition","waistattitude"};

  for(unsigned int li=0;li<2;li++)
    {
      
      aCS->length(CstSignaux[li].size());
      for(unsigned int j=0;j<CstSignaux[li].size();j++)
	aCS[j] = CstSignaux[li][j];

      m_WaistPositionSignalRank = m_SOT_Server_Command->createInputVectorSignal(aCS);
    }

  
  string SotCommand[2]= {"plug pg.waistposition coshell.waistposition",
    "plug pg.waistattitude coshell.waistattitude"};

  for(unsigned int li=0;li<2;li++)
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
