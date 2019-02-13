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

#ifndef PX_PHYSICS_SCB_DEFS
#define PX_PHYSICS_SCB_DEFS

#include "ScbBase.h"

//#define USE_NEW_SYSTEM

#ifdef USE_NEW_SYSTEM
namespace physx
{
namespace Scb
{
	template<class ValueType, class CoreType, class SCBType, class AccessType, int id>
	void setValueT(ValueType value, CoreType& core, SCBType& scb)
	{
		if(!scb.isBuffering())
		{
			AccessType::setCore(core, value);
#if PX_SUPPORT_PVD
			if(scb.getControlState() == ControlState::eIN_SCENE)
			{
				Scb::Scene* scene = scb.getScbScene();
				PX_ASSERT(scene);
				scene->getScenePvdClient().updatePvdProperties(&scb);
			}
#endif
		}
		else
		{
			AccessType::setBuffered(scb, value);
			scb.markUpdated(1<<id);
		}
	}

	template<class ValueType, class CoreType, class SCBType, class AccessType, int id>
	ValueType getValueT(const CoreType& core, const SCBType& scb)
	{
		if(!scb.isBuffered(1<<id))
			return AccessType::getCore(core);
		else
			return AccessType::getBuffered(scb);
	}

	template<class CoreType, class SCBType, class AccessType, int id>
	void syncT(CoreType& core, SCBType& scb)
	{
		if(scb.isBuffered(1<<id))
			AccessType::setCore(core, AccessType::getBuffered(scb));
	}

	// PT: TODO:
	// - revisit the sync function, maybe ask for core/buffer params
	// - revisit the need for explicit BF_XXX flags. I think it's now mainly to skip sync functions in syncState() really.
	#define SCB_MEMBER(_scb, _getCore, _name, _type, _id)																			\
		enum { BF_##_name = 1<<(_id) };																								\
		PX_INLINE	void	setBuffered##_name(_type v)		{ reinterpret_cast<Buf*>(getStream())->m##_name = v;			}		\
		PX_INLINE	_type	getBuffered##_name()	const	{ return reinterpret_cast<const Buf*>(getStream())->m##_name;	}		\
		struct Fns_##_name																											\
		{																															\
			static PX_INLINE	void	setCore(Core& core, _type v)	{ core.set##_name(v);				}						\
			static PX_INLINE	void	setBuffered(_scb& scb, _type v)	{ scb.setBuffered##_name(v);		}						\
			static PX_INLINE	_type	getCore(const Core& core)		{ return core.get##_name();			}						\
			static PX_INLINE	_type	getBuffered(const _scb& scb)	{ return scb.getBuffered##_name();	}						\
		};																															\
		PX_INLINE	void	set##_name(_type v)		{ setValueT<_type, Core, _scb, Fns_##_name, _id>(v, _getCore, *this);		}	\
		PX_INLINE	_type	get##_name()	const	{ return getValueT<_type, Core, _scb, Fns_##_name, _id>(_getCore, *this);	}	\
		PX_INLINE	void	sync##_name()			{ syncT<Core, _scb, Fns_##_name, _id>(_getCore, *this);						}
}
}
#else

// a Regular attribute of type T is one for which 
// * the SC method takes a single argument of type ArgType<T> (defined below)
// * Scb either passes that argument through, or dumps it in a buffer to flush later. 
// * PVD is notified when the variable changes
//
// For each such, we can define static methods to read and write the core and buffered variables, 
// and capture the buffering logic in the BufferedAccess class.
//
// The dummy arg is necessary here because ISO permits partial specialization of member templates
// but not full specialization.
//
// putting just accessors and mutators here allows us to change the behavior just by varying the
// BufferAccess template (e.g. to compile without buffering), and also to size-reduce that template 
// by passing function pointers if necessary

#define SCB_REGULAR_ATTRIBUTE(_val, _type, _name)											\
enum { BF_##_name = 1<<(_val) };															\
_type m##_name;																				\
template<PxU32 Dummy> struct Fns<1<<(_val),Dummy>											\
{																							\
	typedef typename ArgType<_type>::Type Arg;												\
	enum { flag = 1<<(_val) };																\
	static PX_FORCE_INLINE Arg getBuffered(const Buf& buf) { return Arg(buf.m##_name);}		\
	static PX_FORCE_INLINE void setBuffered(Buf& buf, Arg v) { buf.m##_name = v;}			\
	static PX_FORCE_INLINE Arg getCore(const Core& core) { return Arg(core.get##_name());}	\
	static PX_FORCE_INLINE void setCore(Core& core, Arg v) { core.set##_name(v);}			\
};	

#define SCB_REGULAR_ATTRIBUTE_ALIGNED(_val, _type, _name, _alignment)						\
enum { BF_##_name = 1<<(_val) };															\
PX_ALIGN(_alignment, _type) m##_name;														\
template<PxU32 Dummy> struct Fns<1<<(_val),Dummy>											\
{																							\
	typedef typename ArgType<_type>::Type Arg;												\
	enum { flag = 1<<(_val) };																\
	static PX_FORCE_INLINE Arg getBuffered(const Buf& buf) { return buf.m##_name;}			\
	static PX_FORCE_INLINE void setBuffered(Buf& buf, Arg v) { buf.m##_name = v;}			\
	static PX_FORCE_INLINE Arg getCore(const Core& core) { return core.get##_name();}		\
	static PX_FORCE_INLINE void setCore(Core& core, Arg v) { core.set##_name(v);}			\
};

namespace physx
{
namespace Scb
{
class Scene;

template<typename T> struct ArgType		{ typedef T Type; };
template<> struct ArgType<PxVec3>		{ typedef const PxVec3& Type; };
template<> struct ArgType<PxTransform>	{ typedef const PxTransform& Type; };
template<> struct ArgType<PxQuat>		{ typedef const PxQuat& Type; };
template<> struct ArgType<PxPlane>		{ typedef const PxPlane& Type; };
template<> struct ArgType<PxFilterData>	{ typedef const PxFilterData& Type; };

// TODO: should be able to size-reduce this if necessary by just generating one set per
// arg type instead of one per arg, by passing function pointers to the accessors/mutators/flag
// instead of instancing per type.

template<class Buf, class Core, class ScbClass, class BaseClass=Scb::Base>  // BaseClass: introduced to have Scb::Body use custom location for storing buffered property flags
struct BufferedAccess
{
	template<typename Fns>
	static PX_FORCE_INLINE typename Fns::Arg read(const BaseClass& base, const Core& core)
	{
		/*return base.isBuffered(Fns::flag) ? Fns::getBuffered(*reinterpret_cast<const Buf*>(base.getStream())) 
										  : Fns::getCore(core);*/

		if (base.isBuffered(Fns::flag))
		{
			return Fns::getBuffered(*reinterpret_cast<const Buf*>(base.getStream()));
		}
		return Fns::getCore(core);

	}

	template<typename Fns>
	static PX_FORCE_INLINE void write(BaseClass& base, Core& core, typename Fns::Arg v)
	{
		if(!base.isBuffering())
		{
			Fns::setCore(core, v);
#if PX_SUPPORT_PVD
			if(base.getControlState() == ControlState::eIN_SCENE)
			{
				Scb::Scene* scene = base.getScbScene();
				PX_ASSERT(scene);
				scene->getScenePvdClient().updatePvdProperties(static_cast<ScbClass*>(&base));
			}
#endif
		}
		else
		{
			Fns::setBuffered(*reinterpret_cast<Buf*>(base.getStream()), v);
			base.markUpdated(Fns::flag);
		}
	}

	template<typename Fns> 
	static PX_FORCE_INLINE void flush(const BaseClass& base, Core& core, const Buf& buf)
	{
		if(base.isBuffered(Fns::flag))
			Fns::setCore(core, Fns::getBuffered(buf));
	}
};

}
}
#endif

#endif
