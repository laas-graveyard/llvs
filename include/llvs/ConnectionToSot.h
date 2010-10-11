/** @doc Object to interact with the SoT.

   CVS Information:
   $Id$
   $Author$
   $Date$
   $Revision$
   $Source$
   $Log$

   Copyright (c) 2009
   @author Olivier Stasse

   JRL-Japan, CNRS/AIST

   All rights reserved.

   Redistribution and use in source and binary forms, with or without modification,
   are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the CNRS and AIST nor the names of its contributors
   may be used to endorse or promote products derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
   AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
   OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _CONNECTION_TO_SOT_H_
#define _CONNECTION_TO_SOT_H_

//#define _OLD_INTERFACE_H_
#ifdef OMNIORB4
#include <omniORB4/CORBA.h>

#ifdef _OLD_INTERFACE_H_
#include "ServerCommand.hh"
typedef hppCorbaServer::SOT_Server_Command_var sotCorbaInterface;
namespace nsCorba=hppCorbaServer;
#else
#include "server-command-corba.hh"
typedef CorbaServer::SOT_Server_Command_var sotCorbaInterface;
namespace nsCorba=CorbaServer;
#endif

#endif


namespace llvs
{
  class LowLevelVisionServer;
  class ConnectionToSot
    {

    public:

      /*! \brief Constructor */
      ConnectionToSot(LowLevelVisionServer * aLLVS);

      /*! \brief Destructor*/
      ~ConnectionToSot();

      /*! \brief Read waist values.*/
      void ReadWaistSignals(double waistposition[3],
			    double waistattitude[3]);


      /*! \brief Read head values.*/
      void ReadHeadRPYSignals(double headposerpy[6]);

      /*! \brief Read waist values.*/
      void ReadWaistRPYSignals(double headposerpy[6]);

      /*! \brief Read dcomref velocity values from pg .*/
      void ReaddComRefSignals(double waistcom[3]);

      /*! \brief Read a sequence of dcomref velocity values from pg .*/
      void ReaddComRefSignals(vector<double> &dcomref);

      /*! \brief Read com attitude values from pg .*/
      void ReadComAttitudeSignals(double comattitude[3]);

      /*! \brief Write waist velocity command.*/
      void WriteVelocityReference(double velref[3]);

      /*! \brief Write waist velocity command.*/
      void WriteObjectCoG(double velref[2]);

      /*! \brief Create the signals, and plkug them. */
      bool Init();

      /* ! Start the thread */
      void StartThreadOnConnectionSot();

      /* ! Stop the thread */
      void StopThreadOnConnectionSot();

      /*! Internal method to set corba reference */
      bool SetCorbaReference();

      /*! Dump a circular buffer. */
      void DumpCircularBuffer(string aFileName);

    private:

      /*! Pointer on LLVS server. */
      LowLevelVisionServer * m_LLVS;

      /*! Pointer on SoT server. */
      sotCorbaInterface m_SOT_Server_Command;

      /*! \brief Store the rank of waist position signal. */
      CORBA::Long m_WaistPositionSignalRank;

      /*! \brief Store the rank of waist attitude signal. */
      CORBA::Long m_WaistAttitudeSignalRank;

      /*! \brief Store the rank of head poserpy signal. */
      CORBA::Long m_HeadPRPYSignalRank;

      /*! \brief Store the rank of waist poserpy signal. */
      CORBA::Long m_WaistPRPYSignalRank;

      /*! \brief Store the rank of com attitude signal. */
      CORBA::Long m_ComAttitudeSignalRank;

      CORBA::Long m_VelRefSignalRank;
      CORBA::Long m_ObjectCoGSignalRank;

      /*! \brief Store the rank of waist control signal. */
      CORBA::Long m_dComRefSignalRank;

      /*! \brief Boolean value to go out of the thread */
      bool m_EndOfThreadLoop;

      /*! \brief Internal circular buffer */
      double * m_CircularBuffer;

      /*! \brief Index of the circular buffer. */
      unsigned int m_CircularBufferIndex;

      /*! \brief Upper limit of the circular buffer. */
      unsigned int m_CircularBufferIndexMax;

    protected:

    public:
      /* ! \brief To know if the thread should be stop or not */
      bool GetEndOfThreadLoop() const;

    };
};

#endif /* _CONNECTION_TO_SOT_H_ */
