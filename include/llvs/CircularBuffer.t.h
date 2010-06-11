/** @doc This template implements a protected circular buffer
    for data structure.

    Copyright (c) 2010,
    @author O. Stasse

*/

#ifndef _CIRCULAR_BUFFER_LLVS_H_
#define _CIRCULAR_BUFFER_LLVS_H_

#include <vector>
#include <pthread.h>
#include <string.h>
#include <iostream>

#include "errno.h"

#include "Debug.h"
#include "VisionBasicProcess.h"

template <class T>
class CircularBuffer: public HRP2VisionBasicProcess
{
  typedef struct 
  {
    T onedatum;
    pthread_mutex_t amutex;
  } ProtectedDatum;

 public:

  /*! Constructor 
    \param SizeOfCircularBuffer*/
  CircularBuffer(int SizeOfCircularBuffer);
  
  /*! Default destructor. */
  ~CircularBuffer();

  int SetDatum(T* aDatum);
  
  int ReadData(T &aDatum);

  int SaveData(T &aDatum);

 protected:
  /*! Initialize the process. */
  virtual int pInitializeTheProcess();

  /*! Realize the process */
  virtual int pRealizeTheProcess();
  
  /*! Cleanup the process */
  virtual int pCleanUpTheProcess();
  
  /*! vector of Data.*/
  std::vector<ProtectedDatum> m_CircularBuffer;

  T* m_Datum;

  unsigned int m_IndexBuffer;
};


template <class T>
CircularBuffer<T>::CircularBuffer(int SizeOfCircularBuffer)
{
  if (SizeOfCircularBuffer>0)
    m_CircularBuffer.resize(SizeOfCircularBuffer);

  for(unsigned int i=0;
      i<m_CircularBuffer.size();
      i++)
    {
      pthread_mutex_init(&m_CircularBuffer[i].amutex,NULL);
    }

  m_IndexBuffer = 0;
  m_Datum=0x0;
}

template <class T>
int CircularBuffer<T>::pInitializeTheProcess()
{
  m_IndexBuffer = 0;
  return 0;
}

template <class T>
int CircularBuffer<T>::pRealizeTheProcess()
{

  if (m_Datum!=0)
    SaveData(*m_Datum);

  return 0;
}

template <class T>
int CircularBuffer<T>::pCleanUpTheProcess()
{

  m_CircularBuffer.clear();

  return 0;
}
  
template <class T>
int CircularBuffer<T>::ReadData(T &aDatum)
{
  int r;
  int lIndexBuffer = m_IndexBuffer;

  if(lIndexBuffer>0)
    {
      if ((r=pthread_mutex_lock(&(m_CircularBuffer[lIndexBuffer-1].amutex)))<0)
	{
	  cerr << "Error while trying to lock the mutex in ReadData() " << r << endl;
	  cerr<< strerror(r)<<endl;
	  
	}
      aDatum=m_CircularBuffer[lIndexBuffer-1].onedatum;
      if ((r=pthread_mutex_unlock(&(m_CircularBuffer[lIndexBuffer-1].amutex)))<0)
      {
	cerr << "Error while trying to unlock the mutex in ReadData() " << r << endl;
	cerr<< strerror(r)<<endl;
	
      }
      return 0;
    }
  else
    {

      unsigned int lsize = m_CircularBuffer.size();
      if ((r=pthread_mutex_lock(&(m_CircularBuffer[lsize-1].amutex)))<0)
	{
	  cerr << "Error while trying to lock the mutex ReadData() " << r << endl;
	  cerr<< strerror(r)<<endl;
	  
	}
      aDatum=m_CircularBuffer[lsize-1].onedatum;
      if ((r=pthread_mutex_unlock(&(m_CircularBuffer[lsize-1].amutex)))<0)   
	{
	  cerr << "Error while trying to unlock the mutex in ReadData() " << r << endl;
	  cerr<< strerror(r)<<endl;
	  
	}
      return 0;
    }
  return -1;
  
}

template <class T>
int CircularBuffer<T>::SaveData(T &aDatum)
{
  ODEBUG("1 - Save data");

  int r;
  unsigned char lloop=1;
  do
    {
      r=pthread_mutex_trylock(&(m_CircularBuffer[m_IndexBuffer].amutex));
      if (r<0)
	{
	  if (r!=EBUSY)
	    {
	      cerr << "Error while trying to lock the mutex in SaveData " << m_IndexBuffer << endl;
	      cerr<< strerror(r)<<endl;      
	      return -1;
	    }
	}
      else
	lloop=0;
    }
  while(lloop);

  ODEBUG("1.25 - Save data");
  m_CircularBuffer[m_IndexBuffer].onedatum=aDatum;

  ODEBUG("1.5 - Save data");
  if ((r=pthread_mutex_unlock(&(m_CircularBuffer[m_IndexBuffer].amutex)))<0)
    {
      cerr << "Error while trying to unlock the mutex in SaveData " << m_IndexBuffer << endl;
      cerr<< strerror(r)<<endl;
    }

  m_IndexBuffer++;

  if (m_CircularBuffer.size()== m_IndexBuffer)
    m_IndexBuffer = 0;
  

  ODEBUG("1.75 - Save data");
  ODEBUG("2 - Save data");
  return 0;
}

template <class T>
int CircularBuffer<T>::SetDatum(T* aDatum)
{

  m_Datum=aDatum;
  return 0;
}

template <class T>
CircularBuffer<T>::~CircularBuffer()
{
}

#endif /* _CIRCULAR_BUFFER_LLVS_H_ */
