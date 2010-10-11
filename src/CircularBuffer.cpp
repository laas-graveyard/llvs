/** @doc This template implements a protected circular buffer
    for data structure.

   Copyright (c) 2010,
   @author O. Stasse
   See License.txt for more information on license.
*/

template <T>
CircularBuffer::CircularBuffer(int SizeOfCircularBuffer)
{
  if (SizeOfCircularBuffer>0)
    m_CircularBuffer.resize(SizeOfCircularBuffer);

  m_IndexBuffer = 0;
  m_Datum=0x0;
}


#include <CircularBuffer.h>

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
      T=m_CircularBuffer[m_IndexBuffer-1];
      return 0;
    }
  else
    {

      unsigned int lsize = m_CircularBuffer.size();
      T=m_CircularBuffer[lsize-1];
    }
    return -1;

}

template <T>
int CircularBuffer::SaveData(T &aDatum)
{

  m_CircularBuffer[m_IndexBuffer]=T;

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
