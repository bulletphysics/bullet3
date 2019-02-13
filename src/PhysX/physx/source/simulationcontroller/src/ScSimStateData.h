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

#ifndef PX_SIMSTATEDATA
#define PX_SIMSTATEDATA

#include "foundation/PxMemory.h"
#include "ScBodyCore.h"

namespace physx
{
namespace Sc
{
	struct Kinematic : public KinematicTransform
	{
		// The following members buffer the original body data to restore them when switching back to dynamic body
		// (for kinematics the corresponding LowLevel properties are set to predefined values)
		PxVec3			backupInverseInertia;			// The inverse of the body space inertia tensor
		PxReal			backupInvMass;					// The inverse of the body mass
		PxReal			backupLinearDamping;			// The velocity is scaled by (1.0f - this * dt) inside integrateVelocity() every substep.
		PxReal			backupAngularDamping;
		PxReal			backupMaxAngVelSq;				// The angular velocity's magnitude is clamped to this maximum value.
		PxReal			backupMaxLinVelSq;				// The angular velocity's magnitude is clamped to this maximum value	
	};
	PX_COMPILE_TIME_ASSERT(0 == (sizeof(Kinematic) & 0x0f));

	// Important: Struct is reset in setForcesToDefaults.

	enum VelocityModFlags
	{
		VMF_GRAVITY_DIRTY	= (1 << 0),
		VMF_ACC_DIRTY		= (1 << 1),
		VMF_VEL_DIRTY		= (1 << 2)
	};

	struct VelocityMod
	{
		PxVec3	linearPerSec;		// A request to change the linear velocity by this much each second. The velocity is changed by this * dt inside integrateVelocity().
		PxU8	flags;
		PxU8	pad0[3];
		PxVec3	angularPerSec;
		PxU8	pad1[3];
		PxU8	type;
		PxVec3	linearPerStep;		// A request to change the linear velocity by this much the next step. The velocity is changed inside updateForces().
		PxU32	pad2;
		PxVec3	angularPerStep;
		PxU32	pad3;

		PX_FORCE_INLINE	void					clear()													{ linearPerSec = angularPerSec = linearPerStep = angularPerStep = PxVec3(0.0f); }

		PX_FORCE_INLINE	void					clearPerStep()											{ linearPerStep = angularPerStep = PxVec3(0.0f); }

		PX_FORCE_INLINE const PxVec3&			getLinearVelModPerSec()							const	{ return linearPerSec;				}
		PX_FORCE_INLINE void					accumulateLinearVelModPerSec(const PxVec3& v)			{ linearPerSec += v;				}
		PX_FORCE_INLINE void					setLinearVelModPerSec(const PxVec3& v)					{ linearPerSec = v; }
		PX_FORCE_INLINE void					clearLinearVelModPerSec()								{ linearPerSec = PxVec3(0.0f);		}

		PX_FORCE_INLINE const PxVec3&			getLinearVelModPerStep()						const	{ return linearPerStep;				}
		PX_FORCE_INLINE void					accumulateLinearVelModPerStep(const PxVec3& v)			{ linearPerStep += v;				}
		PX_FORCE_INLINE void					clearLinearVelModPerStep()								{ linearPerStep = PxVec3(0.0f);		}

		PX_FORCE_INLINE const PxVec3&			getAngularVelModPerSec()						const	{ return angularPerSec;				}
		PX_FORCE_INLINE void					accumulateAngularVelModPerSec(const PxVec3& v)			{ angularPerSec += v;				}
		PX_FORCE_INLINE void					setAngularVelModPerSec(const PxVec3& v)					{ angularPerSec = v; }
		PX_FORCE_INLINE void					clearAngularVelModPerSec()								{ angularPerSec = PxVec3(0.0f);		}

		PX_FORCE_INLINE const PxVec3&			getAngularVelModPerStep()						const	{ return angularPerStep;			}
		PX_FORCE_INLINE void					accumulateAngularVelModPerStep(const PxVec3& v)			{ angularPerStep += v;				}
		PX_FORCE_INLINE void					clearAngularVelModPerStep()								{ angularPerStep = PxVec3(0.0f);	}

		PX_FORCE_INLINE void					notifyAddAcceleration()									{ flags |= VMF_ACC_DIRTY;			}
		PX_FORCE_INLINE void					notifyClearAcceleration()								{ flags |= VMF_ACC_DIRTY;			}
		PX_FORCE_INLINE void					notifyAddVelocity()										{ flags |= VMF_VEL_DIRTY;			}
		PX_FORCE_INLINE void					notifyClearVelocity()									{ flags |= VMF_VEL_DIRTY;			}
	};
	PX_COMPILE_TIME_ASSERT(sizeof(VelocityMod) == sizeof(Kinematic));


	// Structure to store data for kinematics (target pose etc.)
	// note: we do not delete this object for kinematics even if no target is set.
	struct SimStateData : public Ps::UserAllocated	// TODO: may want to optimize the allocation of this further.
	{
		PxU8 data[sizeof(Kinematic)];

		enum Enum
		{
			eVelMod=0,
			eKine
		};

		SimStateData(){}
		SimStateData(const PxU8 type)
		{
			PxMemZero(data, sizeof(Kinematic));
			Kinematic* kine = reinterpret_cast<Kinematic*>(data);
			kine->type = type;
		}

		PX_FORCE_INLINE PxU32 getType() const { const Kinematic* kine = reinterpret_cast<const Kinematic*>(data); return kine->type;}
		PX_FORCE_INLINE bool isKine() const {return eKine == getType();}
		PX_FORCE_INLINE bool isVelMod() const {return eVelMod == getType();}

		Kinematic* getKinematicData() { Kinematic* kine = reinterpret_cast<Kinematic*>(data); PX_ASSERT(eKine == kine->type);  return kine;}
		VelocityMod* getVelocityModData() { VelocityMod* velmod = reinterpret_cast<VelocityMod*>(data); PX_ASSERT(eVelMod == velmod->type); return velmod;}
		const Kinematic* getKinematicData() const { const Kinematic* kine = reinterpret_cast<const Kinematic*>(data); PX_ASSERT(eKine == kine->type);  return kine;}
		const VelocityMod* getVelocityModData() const { const VelocityMod* velmod = reinterpret_cast<const VelocityMod*>(data); PX_ASSERT(eVelMod == velmod->type); return velmod;}
	};

} // namespace Sc

}  // namespace physx

#endif //PX_SIMSTATEDATA
