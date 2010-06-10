/** @doc This template implements a protected circular buffer
    for data structure.

   Copyright (c) 2010,
   @author O. Stasse
*/

template <T>
CircularBuffer::CircularBuffer(int SizeOfCircularBuffer)
{
  if (SizeOfCircularBuffer>0)
    m_CircularBuffer.resize(SizeOfCircularBuffer);

  for(unsigned int i=0;
      i<m_CircularBuffer.size();
      i++)
    {
      m_CircurlarBuffer[i].m_OneDatumMutex = 
	PTHREAD_MUTEX_INITIALIZER;
    }

  m_IndexBuffer = 0;
  m_Datum=0x0;
}


#include <CircularBuffer.t.h>

template <T>
int CircularBuffer::pInitializeTheProcess()
{
  m_IndexBuffer = 0;

}

template <T>
int CircularBuffer::RealizeTheProcess()
{

  if (m_Datum!=0)
    SaveData(*m_Datum);

  return 0;
}

template <T>
int CircularBuffer::pCleanUpTheProcess()
{

  m_CircularBuffer.clear();

  return 0;
}
  
template <T>
int CircularBuffer::ReadData(T &aDatum)
{
  if(m_indexBuffer>0)
    {
      pthread_mutex_lock(m_CircularBuffer[m_IndexBuffer-1].mutex);
      T=m_CircularBuffer[m_IndexBuffer-1].onedatum;
      pthread_mutex_unlock(m_CircularBuffer[m_IndexBuffer-1].mutex);
      return 0;
    }
  else
    {

      unsigned int lsize = m_CircularBuffer.size();
      pthread_mutex_lock(m_CircularBuffer[m_IndexBuffer-1].mutex);
      T=m_CircularBuffer[lsize-1].onedatum;
      pthread_mutex_unlock(m_CircularBuffer[m_IndexBuffer-1].mutex);
    }
    return -1;
  
}

template <T>
int CircularBuffer::SaveData(T &aDatum)
{

  pthread_mutex_lock(m_CircularBuffer[m_IndexBuffer-1].mutex);
  m_CircularBuffer[m_IndexBuffer].onedatum=T;
  pthread_mutex_unlock(m_CircularBuffer[m_IndexBuffer-1].mutex);

  m_IndexBuffer++;

  if (m_CircularBuffer.size()==
      m_IndexBuffer)
    m_IndexBuffer = 0;
  
  return 0;
}

template <T>
int CircularBuffer::SetDatum(T* aDatum)
{

  m_Datum=aDatum;

}
