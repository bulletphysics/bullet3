//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef PXPVDSDK_PXPVDTRANSPORT_H
#define PXPVDSDK_PXPVDTRANSPORT_H

/** \addtogroup pvd
@{
*/
#include "foundation/PxErrors.h"
#include "foundation/PxFlags.h"
#include "pvd/PxPvd.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief	PxPvdTransport is an interface representing the data transport mechanism.
This class defines all services associated with the transport: configuration, connection, reading, writing etc.
It is owned by the application, and can be realized as a file or a socket (using one-line PxDefault<...> methods in
PhysXExtensions) or in a custom implementation. This is a class that is intended for use by PVD, not by the
application, the application entry points are PxPvd and PvdClient.
*/

class PxPvdTransport
{
  public:
	// connect, isConnected, disconnect, read, write, flush

	/**
	Connects to the Visual Debugger application.
	return True if success
	*/
	virtual bool connect() = 0;

	/**
	Disconnects from the Visual Debugger application.
	If we are still connected, this will kill the entire debugger connection.
	*/
	virtual void disconnect() = 0;

	/**
	 *	Return if connection to PVD is created.
	 */
	virtual bool isConnected() = 0;

	/**
	 *	write bytes to the other endpoint of the connection. should lock before witre. If an error occurs
	 *	this connection will assume to be dead.
	 */
	virtual bool write(const uint8_t* inBytes, uint32_t inLength) = 0;

	/*
	    lock this transport and return it
	*/
	virtual PxPvdTransport& lock() = 0;

	/*
	    unlock this transport
	*/
	virtual void unlock() = 0;

	/**
	 *	send any data and block until we know it is at least on the wire.
	 */
	virtual void flush() = 0;

	/**
	 *	Return size of written data.
	 */
	virtual uint64_t getWrittenDataSize() = 0;

	virtual void release() = 0;

  protected:
	virtual ~PxPvdTransport()
	{
	}
};

/**
	\brief Create a default socket transport.
	\param host host address of the pvd application.
	\param port ip port used for pvd, should same as the port setting in pvd application.
	\param timeoutInMilliseconds timeout when connect to pvd host.
*/
PX_C_EXPORT PxPvdTransport* PX_CALL_CONV
PxDefaultPvdSocketTransportCreate(const char* host, int port, unsigned int timeoutInMilliseconds);

/**
	\brief Create a default file transport.
	\param name full path filename used save captured pvd data, or NULL for a fake/test file transport.
*/
PX_C_EXPORT PxPvdTransport* PX_CALL_CONV PxDefaultPvdFileTransportCreate(const char* name);

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif // PXPVDSDK_PXPVDTRANSPORT_H
