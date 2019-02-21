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
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.

#ifndef PXPVDSDK_PXPROFILEEVENTBUFFERCLIENTMANAGER_H
#define PXPVDSDK_PXPROFILEEVENTBUFFERCLIENTMANAGER_H

#include "PxProfileEventBufferClient.h"

namespace physx { namespace profile {
	
	/**
	\brief	Manager keep collections of PxProfileEventBufferClient clients. 

	@see PxProfileEventBufferClient
	*/
	class PxProfileEventBufferClientManager
	{
	protected:
		virtual ~PxProfileEventBufferClientManager(){}
	public:
		/**
		\brief Adds new client.
		\param inClient Client to add.
		*/
		virtual void addClient( PxProfileEventBufferClient& inClient ) = 0;

		/**
		\brief Removes a client.
		\param inClient Client to remove.
		*/
		virtual void removeClient( PxProfileEventBufferClient& inClient ) = 0;

		/**
		\brief Check if manager has clients.
		\return True if manager has added clients.
		*/
		virtual bool hasClients() const = 0;
	};

	/**
	\brief	Manager keep collections of PxProfileZoneClient clients. 

	@see PxProfileZoneClient
	*/
	class PxProfileZoneClientManager
	{
	protected:
		virtual ~PxProfileZoneClientManager(){}
	public:
		/**
		\brief Adds new client.
		\param inClient Client to add.
		*/
		virtual void addClient( PxProfileZoneClient& inClient ) = 0;

		/**
		\brief Removes a client.
		\param inClient Client to remove.
		*/
		virtual void removeClient( PxProfileZoneClient& inClient ) = 0;

		/**
		\brief Check if manager has clients.
		\return True if manager has added clients.
		*/
		virtual bool hasClients() const = 0;
	};
} }

#endif // PXPVDSDK_PXPROFILEEVENTBUFFERCLIENTMANAGER_H
