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

#ifndef PX_PHYSICS_SCB_SHAPE
#define PX_PHYSICS_SCB_SHAPE

#include "NpMaterial.h"
#include "NpPhysics.h"
#include "ScbNpDeps.h"
#include "ScShapeCore.h"
#include "ScRigidCore.h"

#include "PsUtilities.h"

// PX_SERIALIZATION
#include "PxSerialFramework.h"
//~PX_SERIALIZATION

#include "ScbDefs.h"

namespace physx
{
#if PX_SUPPORT_PVD
	#define UPDATE_PVD_MATERIALS()									\
	if(getControlState() == ControlState::eIN_SCENE)				\
	    getScbScene()->getScenePvdClient().updateMaterials(this);
#else
	#define UPDATE_PVD_MATERIALS() {}
#endif

namespace Scb
{

class RigidObject;

struct ShapeBuffer
{
#ifndef USE_NEW_SYSTEM
	template <PxU32 I, PxU32 dummy> struct Fns {};  // TODO: make the base class traits visible
	typedef Sc::ShapeCore Core;
	typedef ShapeBuffer Buf;
#endif

	ShapeBuffer() : materialBufferIndex(0), materialCount(0) {}

#ifdef USE_NEW_SYSTEM
	PX_ALIGN(16, PxTransform) mShape2Actor;
	PxFilterData	mSimulationFilterData;
	PxReal			mContactOffset;
	PxReal			mRestOffset;
	PxShapeFlags	mFlags;
	PxReal			mTorsionalPatchRadius;
	PxReal			mMinTorsionalPatchRadius;
#else
	// PT: I think we start with "2" (instead of 0) because the two first bits are reserved
	// below, for geometry & materials.
	SCB_REGULAR_ATTRIBUTE_ALIGNED(2, PxTransform,	Shape2Actor, 16)
//	SCB_REGULAR_ATTRIBUTE(2, PxTransform,	Shape2Actor)
	SCB_REGULAR_ATTRIBUTE(3, PxFilterData,  SimulationFilterData)
	SCB_REGULAR_ATTRIBUTE(4, PxReal,		ContactOffset)
	SCB_REGULAR_ATTRIBUTE(5, PxReal,		RestOffset)
	SCB_REGULAR_ATTRIBUTE(6, PxShapeFlags,	Flags)
	SCB_REGULAR_ATTRIBUTE(7, PxReal,		TorsionalPatchRadius)
	SCB_REGULAR_ATTRIBUTE(8, PxReal,		MinTorsionalPatchRadius)
#endif
	Gu::GeometryUnion	geometry;

	union
	{
		PxU16			materialIndex;  // for single material shapes
		PxU32			materialBufferIndex;  // for multi material shapes
	};
	PxU16				materialCount;

	enum
	{
		BF_Geometry	= 1<<0,
		BF_Material	= 1<<1
	};
};

class Shape : public Base
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================

	typedef Sc::ShapeCore Core;
	typedef ShapeBuffer Buf;
public:
// PX_SERIALIZATION
										Shape(const PxEMPTY) :	Base(PxEmpty), mShape(PxEmpty)	{}
	static		void					getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION

	PX_INLINE							Shape(const PxGeometry& geometry,
											  PxShapeFlags shapeFlags,
											  const PxU16* materialIndices,
											  PxU16 materialCount,
											  bool isExclusive);

	PX_INLINE	PxGeometryType::Enum	getGeometryType() const;

	PX_INLINE	const PxGeometry&		getGeometry() const;
	PX_INLINE	const Gu::GeometryUnion&getGeometryUnion() const;
	PX_INLINE	Scb::ShapeBuffer*		setGeometry(const PxGeometry& geom);

	PX_INLINE	PxU16					getNbMaterials() const;
	PX_INLINE	PxMaterial*				getMaterial(PxU32 index) const;
	PX_INLINE	PxU32					getMaterials(PxMaterial** buffer, PxU32 bufferSize, PxU32 startIndex=0) const;
	PX_INLINE	bool					setMaterials(PxMaterial*const* materials, PxU16 materialCount);

#ifdef USE_NEW_SYSTEM
	template<class ValueType, class CoreType, class SCBType, class AccessType, int id>
	void setValueT(ValueType value, CoreType& core, SCBType& scb)
	{
		if(!scb.isBuffering())
		{
			const PxShapeFlags oldShapeFlags = core.getFlags();
			AccessType::setCore(core, value);

			// shared shapes return NULL. But shared shapes aren't mutable when attached to an actor, so no notification needed.
			Sc::RigidCore* rigidCore = NpShapeGetScRigidObjectFromScbSLOW(scb);
			if(rigidCore && scb.getControlState() != ControlState::eINSERT_PENDING)
				rigidCore->onShapeChange(core, Sc::ShapeChangeNotifyFlags(1<<id), oldShapeFlags);
#if PX_SUPPORT_PVD
			Scb::Scene* scene = scb.getScbSceneForAPI();	// shared shapes also return zero here
			if(scene && !scb.insertPending())
				 scene->getScenePvdClient().updatePvdProperties(&scb);
#endif
		}
		else
		{
			AccessType::setBuffered(scb, value);
			scb.markUpdated(1<<id);
		}
	}

	SCB_MEMBER(Shape, mShape, Shape2Actor, const PxTransform&, 2)
	SCB_MEMBER(Shape, mShape, SimulationFilterData, const PxFilterData&, 3)
	SCB_MEMBER(Shape, mShape, ContactOffset, PxReal, 4)
	SCB_MEMBER(Shape, mShape, RestOffset, PxReal, 5)
	SCB_MEMBER(Shape, mShape, Flags, PxShapeFlags, 6)
	SCB_MEMBER(Shape, mShape, TorsionalPatchRadius, PxReal, 7)
	SCB_MEMBER(Shape, mShape, MinTorsionalPatchRadius, PxReal, 8)
#else
	PX_INLINE	const PxTransform&		getShape2Actor() const							{ return read<Buf::BF_Shape2Actor>();			}
	PX_INLINE	void					setShape2Actor(const PxTransform& v)			{ write<Buf::BF_Shape2Actor>(v);				}

	PX_INLINE	PxFilterData			getSimulationFilterData() const					{ return read<Buf::BF_SimulationFilterData>();	}
	PX_INLINE	void					setSimulationFilterData(const PxFilterData& v)	{ write<Buf::BF_SimulationFilterData>(v);		}

	PX_INLINE	PxReal					getContactOffset() const						{ return read<Buf::BF_ContactOffset>();			}
	PX_INLINE	void					setContactOffset(PxReal v)						{ write<Buf::BF_ContactOffset>(v);				}

	PX_INLINE	PxReal					getRestOffset() const							{ return read<Buf::BF_RestOffset>();			}
	PX_INLINE	void					setRestOffset(PxReal v)							{ write<Buf::BF_RestOffset>(v); }

	PX_INLINE	PxReal					getTorsionalPatchRadius() const					{ return read<Buf::BF_TorsionalPatchRadius>(); }
	PX_INLINE	void					setTorsionalPatchRadius(PxReal v)				{ write<Buf::BF_TorsionalPatchRadius>(v); }

	PX_INLINE	PxReal					getMinTorsionalPatchRadius() const				{ return read<Buf::BF_MinTorsionalPatchRadius>(); }
	PX_INLINE	void					setMinTorsionalPatchRadius(PxReal v)			{ write<Buf::BF_MinTorsionalPatchRadius>(v); }

	PX_INLINE	PxShapeFlags			getFlags() const								{ return read<Buf::BF_Flags>();					}
	PX_INLINE	void					setFlags(PxShapeFlags v)						{ write<Buf::BF_Flags>(v);						}
#endif

	//---------------------------------------------------------------------------------
	// Data synchronization
	//---------------------------------------------------------------------------------
				void					syncState();

	//---------------------------------------------------------------------------------
	// Miscellaneous
	//---------------------------------------------------------------------------------
	PX_FORCE_INLINE const PxU16*		getScMaterialIndices()				const	{ return mShape.getMaterialIndices();	}  // Only use if you know what you're doing!

	PX_FORCE_INLINE Sc::ShapeCore&			getScShape()							{ return mShape;					}  // Only use if you know what you're doing!
	PX_FORCE_INLINE const Sc::ShapeCore&	getScShape()					const	{ return mShape;					}

	PX_FORCE_INLINE bool				isExclusive()						const	{ return getScbType() == ScbType::eSHAPE_EXCLUSIVE; }
	PX_FORCE_INLINE void				setControlStateIfExclusive(Scene* s, ControlState::Enum cs);  // for exclusive shapes

	template<bool sync> PX_FORCE_INLINE void checkUpdateOnRemove(Scene* s);

	static size_t getScOffset()	{ return reinterpret_cast<size_t>(&reinterpret_cast<Shape*>(0)->mShape);	}

private:
					bool				setMaterialsHelper(PxMaterial* const* materials, PxU16 materialCount);

					Sc::ShapeCore		mShape;

	PX_FORCE_INLINE	const Scb::ShapeBuffer*	getBufferedData()	const	{ return reinterpret_cast<const Scb::ShapeBuffer*>(getStream()); }
	PX_FORCE_INLINE	Scb::ShapeBuffer*		getBufferedData()			{ return reinterpret_cast<Scb::ShapeBuffer*>(getStream()); }

	PX_FORCE_INLINE	const PxU16* getMaterialBuffer(const Scb::Scene& scene, const Scb::ShapeBuffer& sb) const
	{
		if(sb.materialCount == 1)
			return &sb.materialIndex;
		else
			return scene.getShapeMaterialBuffer(sb.materialBufferIndex);
	}

#ifndef USE_NEW_SYSTEM
	//---------------------------------------------------------------------------------
	// Infrastructure for regular attributes
	//---------------------------------------------------------------------------------

	struct Access: public BufferedAccess<Buf, Core, Shape> 
	{
		template<typename Fns>
		static PX_FORCE_INLINE void write(Shape& base, Core& core, typename Fns::Arg v)
		{
			if(!base.isBuffering())
			{
				PxShapeFlags oldShapeFlags = core.getFlags();
				Fns::setCore(core, v);

				// shared shapes return NULL. But shared shapes aren't mutable when attached to an actor, so no notification needed.
				Sc::RigidCore* rigidCore = NpShapeGetScRigidObjectFromScbSLOW(base);
				if(rigidCore && base.getControlState() != ControlState::eINSERT_PENDING)
					rigidCore->onShapeChange(core, Sc::ShapeChangeNotifyFlags(Fns::flag), oldShapeFlags);
#if PX_SUPPORT_PVD
				Scb::Scene* scene = base.getScbSceneForAPI();	// shared shapes also return zero here
				if(scene && !base.insertPending())
					 scene->getScenePvdClient().updatePvdProperties(&base);
#endif
			}
			else
			{
				Fns::setBuffered(*reinterpret_cast<Buf*>(base.getStream()), v);
				base.markUpdated(Fns::flag);
			}
		}
	};

	template<PxU32 f> PX_FORCE_INLINE typename Buf::Fns<f,0>::Arg read() const		{	return Access::read<Buf::Fns<f,0> >(*this, mShape);	}
	template<PxU32 f> PX_FORCE_INLINE void write(typename Buf::Fns<f,0>::Arg v)		{	Access::write<Buf::Fns<f,0> >(*this, mShape, v);	}
	template<PxU32 f> PX_FORCE_INLINE void flush(const Buf& buf)					{	Access::flush<Buf::Fns<f,0> >(*this, mShape, buf);	}
#endif
};

PX_INLINE Shape::Shape(const PxGeometry& geometry,
					   PxShapeFlags shapeFlags,
					   const PxU16* materialIndices,
					   PxU16 materialCount,
					   bool isExclusive) :
	mShape(geometry, shapeFlags, materialIndices, materialCount)
{
	// paranoia: the notify flags in Sc have to match up
#ifdef USE_NEW_SYSTEM
	PX_COMPILE_TIME_ASSERT(PxU32(ShapeBuffer::BF_Geometry)	== PxU32(Sc::ShapeChangeNotifyFlag::eGEOMETRY));
	PX_COMPILE_TIME_ASSERT(PxU32(ShapeBuffer::BF_Material)	== PxU32(Sc::ShapeChangeNotifyFlag::eMATERIAL));
	PX_COMPILE_TIME_ASSERT(PxU32(BF_Shape2Actor)			== PxU32(Sc::ShapeChangeNotifyFlag::eSHAPE2BODY));
	PX_COMPILE_TIME_ASSERT(PxU32(BF_SimulationFilterData)	== PxU32(Sc::ShapeChangeNotifyFlag::eFILTERDATA));
	PX_COMPILE_TIME_ASSERT(PxU32(BF_ContactOffset)			== PxU32(Sc::ShapeChangeNotifyFlag::eCONTACTOFFSET));
	PX_COMPILE_TIME_ASSERT(PxU32(BF_RestOffset)				== PxU32(Sc::ShapeChangeNotifyFlag::eRESTOFFSET));
	PX_COMPILE_TIME_ASSERT(PxU32(BF_Flags)					== PxU32(Sc::ShapeChangeNotifyFlag::eFLAGS));
#else
	PX_COMPILE_TIME_ASSERT(PxU32(ShapeBuffer::BF_Geometry)				== PxU32(Sc::ShapeChangeNotifyFlag::eGEOMETRY));
	PX_COMPILE_TIME_ASSERT(PxU32(ShapeBuffer::BF_Material)				== PxU32(Sc::ShapeChangeNotifyFlag::eMATERIAL));
	PX_COMPILE_TIME_ASSERT(PxU32(ShapeBuffer::BF_Shape2Actor)			== PxU32(Sc::ShapeChangeNotifyFlag::eSHAPE2BODY));
	PX_COMPILE_TIME_ASSERT(PxU32(ShapeBuffer::BF_SimulationFilterData)	== PxU32(Sc::ShapeChangeNotifyFlag::eFILTERDATA));
	PX_COMPILE_TIME_ASSERT(PxU32(ShapeBuffer::BF_ContactOffset)			== PxU32(Sc::ShapeChangeNotifyFlag::eCONTACTOFFSET));
	PX_COMPILE_TIME_ASSERT(PxU32(ShapeBuffer::BF_RestOffset)			== PxU32(Sc::ShapeChangeNotifyFlag::eRESTOFFSET));
	PX_COMPILE_TIME_ASSERT(PxU32(ShapeBuffer::BF_Flags)					== PxU32(Sc::ShapeChangeNotifyFlag::eFLAGS));
#endif

	if(isExclusive)
		setScbType(ScbType::eSHAPE_EXCLUSIVE);
	else
		setScbType(ScbType::eSHAPE_SHARED);
}

PX_INLINE PxGeometryType::Enum Shape::getGeometryType() const
{
	return mShape.getGeometryType();
}

PX_INLINE const PxGeometry& Shape::getGeometry() const
{
	if(isBuffered(Buf::BF_Geometry))
		return getBufferedData()->geometry.getGeometry();
	else
		return mShape.getGeometry();
}

PX_INLINE const Gu::GeometryUnion& Shape::getGeometryUnion() const
{
	if(isBuffered(Buf::BF_Geometry))
		return getBufferedData()->geometry;
	else
		return mShape.getGeometryUnion();
}

PX_INLINE Scb::ShapeBuffer* Shape::setGeometry(const PxGeometry& geom)
{
	Scb::ShapeBuffer* shapeBuffer = NULL;
	if (!isBuffering())
	{
		Scb::Scene* sc = getScbScene();

		if(sc)
			sc->getScScene().unregisterShapeFromNphase(mShape);

		mShape.setGeometry(geom);

		if(sc)
			sc->getScScene().registerShapeInNphase(mShape);

		Sc::RigidCore* rigidCore = NpShapeGetScRigidObjectFromScbSLOW(*this);
		if(rigidCore)
			rigidCore->onShapeChange(mShape, Sc::ShapeChangeNotifyFlag::eGEOMETRY, PxShapeFlags());

#if PX_SUPPORT_PVD
		Scb::Scene* scbScene = getScbSceneForAPI();	
		if(scbScene)
			scbScene->getScenePvdClient().releaseAndRecreateGeometry(this);
#endif
	}
	else
	{
		markUpdated(Buf::BF_Geometry);
		shapeBuffer = getBufferedData();
		shapeBuffer->geometry.set(geom);
	}

	return shapeBuffer;
}

PX_INLINE PxU16 Shape::getNbMaterials() const
{
	if(isBuffered(Buf::BF_Material))
		return getBufferedData()->materialCount;
	else
		return mShape.getNbMaterialIndices();
}

PX_INLINE PxMaterial* Shape::getMaterial(PxU32 index) const
{
	PX_ASSERT(index < getNbMaterials());

	NpMaterialManager& matManager = NpPhysics::getInstance().getMaterialManager();
	if(isBuffered(Buf::BF_Material))
	{
		const PxU16* materialIndices = getMaterialBuffer(*getScbScene(), *getBufferedData());
		return matManager.getMaterial(materialIndices[index]);
	}
	else
	{
		PxU16 matTableIndex = mShape.getMaterialIndices()[index];
		return matManager.getMaterial(matTableIndex);
	}
}

PX_INLINE PxU32 Shape::getMaterials(PxMaterial** buffer, PxU32 bufferSize, PxU32 startIndex) const
{
	const PxU16* materialIndices;
	PxU32 matCount;
	NpMaterialManager& matManager = NpPhysics::getInstance().getMaterialManager();
	if(isBuffered(Buf::BF_Material))
	{
		// IMPORTANT:
		// As long as the material pointers get copied to a user buffer, this works fine.
		// Never give direct access to the internal material buffer because in the
		// double buffered case the pointer changes on resize.

		const Scb::ShapeBuffer*	PX_RESTRICT bufferedData = getBufferedData();

		materialIndices = getMaterialBuffer(*getScbScene(), *bufferedData);
		matCount = bufferedData->materialCount;
	}
	else
	{
		materialIndices = mShape.getMaterialIndices();
		matCount = mShape.getNbMaterialIndices();
	}

	// PT: this is copied from Cm::getArrayOfPointers(). We cannot use the Cm function here
	// because of the extra indirection needed to access the materials.
	PxU32 size = matCount;
	const PxU32 remainder = PxU32(PxMax<PxI32>(PxI32(size - startIndex), 0));
	const PxU32 writeCount = PxMin(remainder, bufferSize);
	materialIndices += startIndex;
	for(PxU32 i=0;i<writeCount;i++)
		buffer[i] = matManager.getMaterial(materialIndices[i]);

	return writeCount;
}

PX_INLINE bool Shape::setMaterials(PxMaterial* const* materials, PxU16 materialCount)
{
	if(!isBuffering())
	{
		bool ret = setMaterialsHelper(materials, materialCount);
		UPDATE_PVD_MATERIALS()
		return ret;
	}
	else
	{
		Scb::ShapeBuffer* PX_RESTRICT bufferedData = getBufferedData();

		PxU16* materialIndices;
		if(materialCount == 1)
			materialIndices = &bufferedData->materialIndex;
		else
		{
			PxU32 bufferIdx;
			materialIndices = getScbScene()->allocShapeMaterialBuffer(materialCount, bufferIdx);
			bufferedData->materialBufferIndex = bufferIdx;
		}
		bufferedData->materialCount = materialCount;

		NpMaterial::getMaterialIndices(materials, materialIndices, materialCount);
		
		markUpdated(Buf::BF_Material);

		return true;
	}
}

PX_FORCE_INLINE void Shape::setControlStateIfExclusive(Scene* s, ControlState::Enum cs)
{
	if(isExclusive())
	{
		setControlState(cs);
		setScbScene(s);
	}
}

template<bool sync>
PX_FORCE_INLINE void Shape::checkUpdateOnRemove(Scene* s)
{
	// special code to cover the case where a shape has a pending update and gets released. The following operations have to be done
	// before the ref-counter of the shape gets decremented because that could cause the shape to be deleted in which case it must not
	// be in the pending update list any longer.
	if(getControlFlags() & Scb::ControlFlag::eIS_UPDATED)
	{
		if(sync)
			syncState();
		s->removeShapeFromPendingUpdateList(*this);

		resetControlFlag(ControlFlag::eIS_UPDATED);
	}
}

}  // namespace Scb

}

#endif
