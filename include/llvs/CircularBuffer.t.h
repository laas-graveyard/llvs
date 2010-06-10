/** @doc This template implements a protected circular buffer
    for data structure.

   Copyright (c) 2010,
   @author O. Stasse

*/

#ifndef _CIRCULAR_BUFFER_LLVS_H_
#define _CIRCULAR_BUFFER_LLVS_H_

#include <vector>
#include <pthread.h>
#include "VisionBasicProcess.h"

template <class T>
class CircularBuffer: public HRP2VisionBasicProcess
{
  typedef struct 
  {
    T onedatum;
    pthread_mutex_t mutex;
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
      //m_CircularBuffer[i].mutex = PTHREAD_MUTEX_INITIALIZER;
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
  if(m_IndexBuffer>0)
    {
      pthread_mutex_lock(&(m_CircularBuffer[m_IndexBuffer-1].mutex));
      aDatum=m_CircularBuffer[m_IndexBuffer-1].onedatum;
      pthread_mutex_unlock(&(m_CircularBuffer[m_IndexBuffer-1].mutex));
      return 0;
    }
  else
    {

      unsigned int lsize = m_CircularBuffer.size();
      pthread_mutex_lock(&(m_CircularBuffer[m_IndexBuffer-1].mutex));
      aDatum=m_CircularBuffer[lsize-1].onedatum;
      pthread_mutex_unlock(&(m_CircularBuffer[m_IndexBuffer-1].mutex));
    }
    return -1;
  
}

template <class T>
int CircularBuffer<T>::SaveData(T &aDatum)
{

  pthread_mutex_lock(&(m_CircularBuffer[m_IndexBuffer-1].mutex));
  m_CircularBuffer[m_IndexBuffer].onedatum=aDatum;
  pthread_mutex_unlock(&(m_CircularBuffer[m_IndexBuffer-1].mutex));

  m_IndexBuffer++;

  if (m_CircularBuffer.size()==
      m_IndexBuffer)
    m_IndexBuffer = 0;
  
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
