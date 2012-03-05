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

#ifndef _SCE_PFX_SUB_DATA_H
#define _SCE_PFX_SUB_DATA_H

namespace sce {
namespace PhysicsEffects {

struct PfxSubData {
	enum {
		NONE = 0,
		MESH_INFO
	};

	union {
		struct {
			PfxUInt8  m_type;
			SCE_PFX_PADDING(1,1)

			struct {
				PfxUInt8  islandId;
				PfxUInt8  facetId;
				PfxUInt16 s;
				PfxUInt16 t;

			} m_facetLocal;
		};
		PfxUInt32 param[2];
	};

	PfxSubData()
	{
		param[0] = param[1] = 0;
	}

	void  setIslandId(PfxUInt8 i) {m_facetLocal.islandId = i;}
	void  setFacetId(PfxUInt8 i) {m_facetLocal.facetId = i;}
	void  setFacetLocalS(PfxFloat s) {m_facetLocal.s = (PfxUInt16)(s * 65535.0f);}
	void  setFacetLocalT(PfxFloat t) {m_facetLocal.t = (PfxUInt16)(t * 65535.0f);}

	PfxUInt8 getIslandId() {return m_facetLocal.islandId;}
	PfxUInt8 getFacetId() {return m_facetLocal.facetId;}
	PfxFloat getFacetLocalS() {return m_facetLocal.s / 65535.0f;}
	PfxFloat getFacetLocalT() {return m_facetLocal.t / 65535.0f;}
};

} //namespace PhysicsEffects
} //namespace sce
#endif // _SCE_PFX_SUB_DATA_H
