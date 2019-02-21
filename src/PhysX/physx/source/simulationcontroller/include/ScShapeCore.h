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


#ifndef PX_PHYSICS_SCP_SHAPECORE
#define PX_PHYSICS_SCP_SHAPECORE

#include "PsUserAllocated.h"
#include "GuGeometryUnion.h"
#include "PxvGeometry.h"
#include "PsUtilities.h"
#include "PxFiltering.h"
#include "PxShape.h"

namespace physx
{
class PxShape;

namespace Sc
{
	class Scene;
	class RigidCore;
	class BodyCore;
	class ShapeSim;
	class MaterialCore;

	class ShapeCore : public Ps::UserAllocated
	{
	//= ATTENTION! =====================================================================================
	// Changing the data layout of this class breaks the binary serialization format.  See comments for 
	// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
	// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
	// accordingly.
	//==================================================================================================
	public:
// PX_SERIALIZATION
													ShapeCore(const PxEMPTY);
						void						exportExtraData(PxSerializationContext& stream);
						void						importExtraData(PxDeserializationContext& context);
						void						resolveReferences(PxDeserializationContext& context);
		static			void						getBinaryMetaData(PxOutputStream& stream);
		                void                        resolveMaterialReference(PxU32 materialTableIndex, PxU16 materialIndex);
//~PX_SERIALIZATION

													ShapeCore(const PxGeometry& geometry, 
															  PxShapeFlags shapeFlags,
															  const PxU16* materialIndices, 
															  PxU16 materialCount);

													~ShapeCore();

		PX_FORCE_INLINE	PxGeometryType::Enum		getGeometryType()							const	{ return mCore.geometry.getType();			}
						PxShape*					getPxShape();
						const PxShape*				getPxShape()								const;

		PX_FORCE_INLINE	const Gu::GeometryUnion&	getGeometryUnion()							const	{ return mCore.geometry;					}
		PX_FORCE_INLINE	const PxGeometry&			getGeometry()								const	{ return mCore.geometry.getGeometry();		}
						void						setGeometry(const PxGeometry& geom);

						PxU16						getNbMaterialIndices()						const;
						const PxU16*				getMaterialIndices()						const;
						void						setMaterialIndices(const PxU16* materialIndices, PxU16 materialIndexCount);

		PX_FORCE_INLINE	const PxTransform&			getShape2Actor()							const	{ return mCore.transform;					}
		PX_FORCE_INLINE	void						setShape2Actor(const PxTransform& s2b)				{ mCore.transform = s2b;					}
		
		PX_FORCE_INLINE	const PxFilterData&			getSimulationFilterData()					const	{ return mSimulationFilterData;				}
		PX_FORCE_INLINE	void						setSimulationFilterData(const PxFilterData& data)	{ mSimulationFilterData = data;				}

		// PT: this one doesn't need double buffering
		PX_FORCE_INLINE	const PxFilterData&			getQueryFilterData()						const	{ return mQueryFilterData;					}
		PX_FORCE_INLINE	void						setQueryFilterData(const PxFilterData& data)		{ mQueryFilterData = data;					}

		PX_FORCE_INLINE	PxReal						getContactOffset()							const	{ return mCore.contactOffset;				}
		PX_FORCE_INLINE	void						setContactOffset(PxReal offset)						{ mCore.contactOffset = offset;				}

		PX_FORCE_INLINE	PxReal						getRestOffset()								const	{ return mRestOffset;						}
		PX_FORCE_INLINE	void						setRestOffset(PxReal offset)						{ mRestOffset = offset;						}

		PX_FORCE_INLINE	PxReal						getTorsionalPatchRadius()					const	{ return mTorsionalRadius;					}
		PX_FORCE_INLINE	void						setTorsionalPatchRadius(PxReal tpr)					{ mTorsionalRadius = tpr;					}

		PX_FORCE_INLINE PxReal						getMinTorsionalPatchRadius()				const	{return mMinTorsionalPatchRadius;			}
		PX_FORCE_INLINE	void						setMinTorsionalPatchRadius(PxReal radius)			{ mMinTorsionalPatchRadius = radius;		}

		PX_FORCE_INLINE	PxShapeFlags				getFlags()									const	{ return PxShapeFlags(mCore.mShapeFlags);	}
		PX_FORCE_INLINE	void						setFlags(PxShapeFlags f)							{ mCore.mShapeFlags = f;					}

		PX_FORCE_INLINE const PxsShapeCore&			getCore()									const	{ return mCore;								}

		static PX_FORCE_INLINE ShapeCore&			getCore(PxsShapeCore& core)			
		{ 
			size_t offset = PX_OFFSET_OF(ShapeCore, mCore);
			return *reinterpret_cast<ShapeCore*>(reinterpret_cast<PxU8*>(&core) - offset); 
		}	

	protected:
						PxFilterData				mQueryFilterData;		// Query filter data PT: TODO: consider moving this to SceneQueryShapeData
						PxFilterData				mSimulationFilterData;	// Simulation filter data
						PxsShapeCore				PX_ALIGN(16, mCore);	
						PxReal						mRestOffset;			// same as the API property of the same name
						PxReal						mTorsionalRadius;
						PxReal						mMinTorsionalPatchRadius;
	};

} // namespace Sc


}

#endif
