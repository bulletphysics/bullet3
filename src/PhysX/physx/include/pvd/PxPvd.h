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

#ifndef PXPVDSDK_PXPVD_H
#define PXPVDSDK_PXPVD_H

/** \addtogroup pvd
@{
*/
#include "foundation/PxFlags.h"
#include "foundation/PxProfiler.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxFoundation;
class PxPvdTransport;

/**
\brief types of instrumentation that PVD can do.
*/
struct PxPvdInstrumentationFlag
{
	enum Enum
	{
		/**
			\brief Send debugging information to PVD.

			This information is the actual object data of the rigid statics, shapes,
			articulations, etc.  Sending this information has a noticeable impact on
			performance and thus this flag should not be set if you want an accurate
			performance profile.
	     */
		eDEBUG   = 1 << 0,

		/**
			\brief Send profile information to PVD.

			This information populates PVD's profile view.  It has (at this time) negligible
			cost compared to Debug information and makes PVD *much* more useful so it is quite
			highly recommended.

			This flag works together with a PxCreatePhysics parameter.
			Using it allows the SDK to send profile events to PVD.
	    */
		ePROFILE = 1 << 1,

		/**
			\brief Send memory information to PVD.

			The PVD sdk side hooks into the Foundation memory controller and listens to
			allocation/deallocation events.  This has a noticable hit on the first frame,
			however, this data is somewhat compressed and the PhysX SDK doesn't allocate much
			once it hits a steady state.  This information also has a fairly negligible
			impact and thus is also highly recommended.

			This flag works together with a PxCreatePhysics parameter,
			trackOutstandingAllocations.  Using both of them together allows users to have
			an accurate view of the overall memory usage of the simulation at the cost of
			a hashtable lookup per allocation/deallocation.  Again, PhysX makes a best effort
			attempt not to allocate or deallocate during simulation so this hashtable lookup
			tends to have no effect past the first frame.

			Sending memory information without tracking outstanding allocations means that
			PVD will accurate information about the state of the memory system before the
			actual connection happened.
	    */
		eMEMORY  = 1 << 2,

		eALL     = (eDEBUG | ePROFILE | eMEMORY)
	};
};

/**
\brief Bitfield that contains a set of raised flags defined in PxPvdInstrumentationFlag.

@see PxPvdInstrumentationFlag
*/
typedef PxFlags<PxPvdInstrumentationFlag::Enum, uint8_t> PxPvdInstrumentationFlags;
PX_FLAGS_OPERATORS(PxPvdInstrumentationFlag::Enum, uint8_t)

/**
\brief PxPvd is the top-level class for the PVD framework, and the main customer interface for PVD
configuration.It is a singleton class, instantiated and owned by the application.
*/
class PxPvd : public physx::PxProfilerCallback
{
  public:
	/**
	Connects the SDK to the PhysX Visual Debugger application.
	\param transport transport for pvd captured data.
	\param flags Flags to set.
	return True if success
	*/
	virtual bool connect(PxPvdTransport& transport, PxPvdInstrumentationFlags flags) = 0;

	/**
	Disconnects the SDK from the PhysX Visual Debugger application.
	If we are still connected, this will kill the entire debugger connection.
	*/
	virtual void disconnect() = 0;

	/**
	 *	Return if connection to PVD is created.
	  \param useCachedStatus
	    1> When useCachedStaus is false, isConnected() checks the lowlevel network status.
	       This can be slow because it needs to lock the lowlevel network stream. If isConnected() is
	       called frequently, the expense of locking can be significant.
	    2> When useCachedStatus is true, isConnected() checks the highlevel cached status with atomic access.
	       It is faster than locking, but the status may be different from the lowlevel network with latency of up to
	       one frame.
	       The reason for this is that the cached status is changed inside socket listener, which is not
	       called immediately when the lowlevel connection status changes.
	 */
	virtual bool isConnected(bool useCachedStatus = true) = 0;

	/**
	returns the PVD data transport
	returns NULL if no transport is present.
	*/
	virtual PxPvdTransport* getTransport() = 0;

	/**
	Retrieves the PVD flags. See PxPvdInstrumentationFlags.
	*/
	virtual PxPvdInstrumentationFlags getInstrumentationFlags() = 0;

	/**
	\brief Releases the pvd instance.
	*/
	virtual void release() = 0;

  protected:
	virtual ~PxPvd()
	{
	}
};

/**
	\brief Create a pvd instance. 	
	\param foundation is the foundation instance that stores the allocator and error callbacks.
*/
PX_C_EXPORT PxPvd* PX_CALL_CONV PxCreatePvd(PxFoundation& foundation);

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif // PXPVDSDK_PXPVD_H
