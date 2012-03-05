/*
Physics Effects Copyright(C) 2010 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#ifndef _SCE_PFX_SHAPE_H
#define _SCE_PFX_SHAPE_H

#include "../base/pfx_common.h"
#include "../base/pfx_vec_utils.h"
#include "pfx_box.h"
#include "pfx_sphere.h"
#include "pfx_capsule.h"
#include "pfx_cylinder.h"
#include "pfx_tri_mesh.h"
#include "pfx_large_tri_mesh.h"

namespace sce {
namespace PhysicsEffects {

enum ePfxShapeType
{
	kPfxShapeSphere = 0,
	kPfxShapeBox,		
	kPfxShapeCapsule,	
	kPfxShapeCylinder,	
	kPfxShapeConvexMesh,
	kPfxShapeLargeTriMesh,
	kPfxShapeReserved0,
	kPfxShapeReserved1,
	kPfxShapeReserved2,
	kPfxShapeUser0,
	kPfxShapeUser1,
	kPfxShapeUser2,
	kPfxShapeCount // =12
};

class SCE_PFX_ALIGNED(16) PfxShape
{
friend class PfxCollidable;

private:
	union {
		PfxFloat  m_vecDataF[3];
		PfxUInt32 m_vecDataI[3];
		void	  *m_vecDataPtr[2];
	};
	PfxUInt8 m_type;
	SCE_PFX_PADDING(1,3)
	PfxFloat m_offsetPosition[3];
	PfxFloat m_offsetOrientation[4];
	PfxUInt32 m_contactFilterSelf;
	PfxUInt32 m_contactFilterTarget;
	SCE_PFX_PADDING(2,12)

public:
	inline void reset();
	
	// Shape
	inline void setBox(PfxBox SCE_VECTORMATH_AOS_VECTOR_ARG box);
	inline void setCapsule(PfxCapsule capsule);
	inline void setCylinder(PfxCylinder cylinder);
	inline void setSphere(PfxSphere sphere);
	inline void setConvexMesh(const PfxConvexMesh *convexMesh);
	inline void setLargeTriMesh(const PfxLargeTriMesh *largeMesh);

	inline PfxUInt8			getType() const;
	inline PfxBox			getBox()const ;
	inline PfxCapsule		getCapsule() const;
	inline PfxCylinder		getCylinder() const;
	inline PfxSphere		getSphere() const;
	inline const PfxConvexMesh*   getConvexMesh() const;
	inline const PfxLargeTriMesh* getLargeTriMesh() const;

	// Offset
	inline void setOffsetTransform(const PfxTransform3 & xfrm);
	inline void setOffsetOrientation(const PfxQuat SCE_VECTORMATH_AOS_VECTOR_ARG rot) {return pfxStoreQuat(rot,m_offsetOrientation);}
	inline void setOffsetPosition(const PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG pos) {return pfxStoreVector3(pos,m_offsetPosition);}

	inline PfxTransform3	getOffsetTransform() const;
	inline PfxVector3		getOffsetPosition() const {return pfxReadVector3(m_offsetPosition);}
	inline PfxQuat			getOffsetOrientation() const {return pfxReadQuat(m_offsetOrientation);}

	// Raw data access
	inline void setDataFloat(int i,PfxFloat v) {m_vecDataF[i]=v;}
	inline void setDataInteger(int i,PfxUInt32 v) {m_vecDataI[i]=v;}

	inline PfxFloat getDataFloat(int i) const {return m_vecDataF[i];}
	inline PfxUInt32 getDataInteger(int i) const {return m_vecDataI[i];}

	// Contact Filter
	PfxUInt32	getContactFilterSelf() const {return m_contactFilterSelf;}
	void		setContactFilterSelf(PfxUInt32 filter) {m_contactFilterSelf = filter;}

	PfxUInt32	getContactFilterTarget() const {return m_contactFilterTarget;}
	void		setContactFilterTarget(PfxUInt32 filter) {m_contactFilterTarget = filter;}
	
	// Bouding Volume
	void getAabb(PfxVector3 &aabbMin,PfxVector3 &aabbMax) const;
};

#include "pfx_shape_implementation.h"

} // namespace PhysicsEffects
} // namespace sce

#endif // _SCE_PFX_SHAPE_H
