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
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

        
#ifndef PXS_CONTACTMANAGER_H
#define PXS_CONTACTMANAGER_H

#include "PxvConfig.h"
#include "PxcNpWorkUnit.h"

namespace physx
{

class PxsContext;
class PxsRigidBody;
struct PxsCCDBody;
class PxsMaterialManager;
struct PxsCCDShape;

namespace Dy
{
	class DynamicsContext;
}

namespace Sc
{
	class ShapeInteraction;
}

enum PxsPairVisColor 
{

	eVIS_COLOR_SWEPTINTEGRATE_OFF = 0x000000,
	eVIS_COLOR_SWEPTINTEGRATE_SLOW = 0x404040,
	eVIS_COLOR_SWEPTINTEGRATE_CLEAR = 0x007f00,
	eVIS_COLOR_SWEPTINTEGRATE_IMPACT = 0x1680ff,
	eVIS_COLOR_SWEPTINTEGRATE_FAIL = 0x0000ff

};


/**
\brief Additional header structure for CCD contact data stream.
*/
struct PxsCCDContactHeader
{
	/**
	\brief Stream for next collision. The same pair can collide multiple times during multiple CCD passes.
	*/
	PxsCCDContactHeader* nextStream;	//4    //8
	/**
	\brief Size (in bytes) of the CCD contact stream (not including force buffer)
	*/
	PxU16 contactStreamSize;			//6    //10
	/**
	\brief Defines whether the stream is from a previous pass.
	
	It could happen that the stream can not get allocated because we run out of memory. In that case the current event should not use the stream
	from an event of the previous pass.
	*/
	PxU16 isFromPreviousPass;			//8    //12

	PxU8 pad[12 - sizeof(PxsCCDContactHeader*)];	//16
};

PX_COMPILE_TIME_ASSERT((sizeof(PxsCCDContactHeader) & 0xF) == 0);


class PxsContactManager
{
public:
											PxsContactManager(PxsContext* context, PxU32 index);
											~PxsContactManager();


	PX_FORCE_INLINE	void					setDisableStrongFriction(PxU32 d)	{ (!d)	? mNpUnit.flags &= ~PxcNpWorkUnitFlag::eDISABLE_STRONG_FRICTION 
																					: mNpUnit.flags |= PxcNpWorkUnitFlag::eDISABLE_STRONG_FRICTION; }

	PX_FORCE_INLINE	PxReal					getRestDistance()					const	{ return mNpUnit.restDistance;				}
	PX_FORCE_INLINE	void					setRestDistance(PxReal v)					{ mNpUnit.restDistance = v;					}

					void					destroy();

	PX_FORCE_INLINE	PxU8					getDominance0()						const	{ return mNpUnit.dominance0;				}
	PX_FORCE_INLINE	void					setDominance0(PxU8 v)						{ mNpUnit.dominance0 = v;					}

	PX_FORCE_INLINE	PxU8					getDominance1()						const	{ return mNpUnit.dominance1;				}
	PX_FORCE_INLINE	void					setDominance1(PxU8 v)						{ mNpUnit.dominance1 = v;					}

	PX_FORCE_INLINE		PxU16				getTouchStatus()									const	{ return PxU16(mNpUnit.statusFlags & PxcNpWorkUnitStatusFlag::eHAS_TOUCH); }
	PX_FORCE_INLINE		PxU16				touchStatusKnown()									const	{ return PxU16(mNpUnit.statusFlags & PxcNpWorkUnitStatusFlag::eTOUCH_KNOWN); }
	PX_FORCE_INLINE		PxI32				getTouchIdx()										const	{ return (mNpUnit.statusFlags& PxcNpWorkUnitStatusFlag::eHAS_TOUCH) ? 1 : (mNpUnit.statusFlags& PxcNpWorkUnitStatusFlag::eHAS_NO_TOUCH ? -1 : 0); }

	PX_FORCE_INLINE		PxU32				getIndex()											const	{ return mNpUnit.index;						}

	PX_FORCE_INLINE	PxU16					getHasCCDRetouch()					const	{ return PxU16(mNpUnit.statusFlags & PxcNpWorkUnitStatusFlag::eHAS_CCD_RETOUCH); }
	PX_FORCE_INLINE	void					clearCCDRetouch()							{ mNpUnit.statusFlags &= ~PxcNpWorkUnitStatusFlag::eHAS_CCD_RETOUCH; }
	PX_FORCE_INLINE	void					raiseCCDRetouch()							{ mNpUnit.statusFlags |= PxcNpWorkUnitStatusFlag::eHAS_CCD_RETOUCH; }



	// flags stuff - needs to be refactored

	PX_FORCE_INLINE		Ps::IntBool			isChangeable()										const	{ return Ps::IntBool(mFlags & PXS_CM_CHANGEABLE);		}
	PX_FORCE_INLINE		Ps::IntBool			getCCD()											const	{ return Ps::IntBool((mFlags & PXS_CM_CCD_LINEAR) && (mNpUnit.flags & PxcNpWorkUnitFlag::eDETECT_CCD_CONTACTS)); }
	PX_FORCE_INLINE		Ps::IntBool			getHadCCDContact()									const	{ return Ps::IntBool(mFlags & PXS_CM_CCD_CONTACT); }
	PX_FORCE_INLINE		void				setHadCCDContact()											{ mFlags |= PXS_CM_CCD_CONTACT; }
						void				setCCD(bool enable);
	PX_FORCE_INLINE		void				clearCCDContactInfo()										{ mFlags &= ~PXS_CM_CCD_CONTACT; mNpUnit.ccdContacts = NULL; }

	PX_FORCE_INLINE	PxcNpWorkUnit&			getWorkUnit()								{ return mNpUnit;		}
	PX_FORCE_INLINE	const PxcNpWorkUnit&	getWorkUnit()						const	{ return mNpUnit;		}

	PX_FORCE_INLINE	void*					getUserData()						const	{ return mShapeInteraction;		}
	
	// Setup solver-constraints
						void				resetCachedState();
						void				resetFrictionCachedState();

						Sc::ShapeInteraction*					getShapeInteraction() const { return mShapeInteraction; }

private:
						//KS - moving this up - we want to get at flags
					
						PxsRigidBody*			mRigidBody0;					//4		//8
						PxsRigidBody*			mRigidBody1;					//8		//16	
						PxU32					mFlags;							//20	//36
						Sc::ShapeInteraction*	mShapeInteraction;				//16	//32
						


						friend class PxsContext;
	// everything required for narrow phase to run
					PxcNpWorkUnit			mNpUnit;

	enum
	{
		PXS_CM_CHANGEABLE	= (1<<0),
		PXS_CM_CCD_LINEAR	= (1<<1),
		PXS_CM_CCD_CONTACT	= (1 << 2)
	};

	friend class Dy::DynamicsContext;
	friend struct PxsCCDPair;
	friend class PxsIslandManager;
	friend class PxsCCDContext;
	friend class Sc::ShapeInteraction;
};

}

#endif
