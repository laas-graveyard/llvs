/** @doc This template implements a protected circular buffer
    for data structure.

   Copyright (c) 2010,
   @author O. Stasse

*/

#ifdef _CIRCULAR_BUFFER_LLVS_H_
#define _CIRCULAR_BUFFER_LLVS_H_

#include <vector>
#include "VisionBasicProcess.h"

template <classname T>
CircularBuffer: public HRP2VisionBasicProcess
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

  unsigned int m_indexBuffer;
};


#endif /* _CIRCULAR_BUFFER_LLVS_H_ */
