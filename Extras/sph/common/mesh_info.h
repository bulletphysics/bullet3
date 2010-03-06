/*
  FLUIDS v.1 - SPH Fluid Simulator for CPU and GPU
  Copyright (C) 2009. Rama Hoetzlein, http://www.rchoetzlein.com

  ZLib license
  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

#ifndef DEF_MESH_INFO
	#define DEF_MESH_INFO

	#include "common_defs.h"
	//#include "mint_config.h"	
	typedef signed int		xref;

	#define MAX_MFORMAT		10
	#define MAX_BFORMAT		5

	// CM - Connected mesh
	struct FaceCM {
		xref		e1, e2, e3, e4;
		xref		v1, v2, v3, v4;
	};
	struct EdgeCM {
		xref		v1, v2;
		xref		f1, f2;
	};
	struct VertCM {
		hList		elist;
		hList		flist;
		float		x, y, z;
	};
	// FVF - Face-vertex-face mesh
	struct FaceFVF {
		xref		v1, v2, v3, v4;
	};
	struct VertFVF {
		hList		flist;
		float		x, y, z;
	};

	// Extra attributes
	struct AttrPos {
		float	x, y, z;
	};
	struct AttrClr {
		DWORD	clr;
	};
	struct AttrNorm {
		float	nx, ny, nz;
	};
	struct AttrTex {
		float	tu, tv;
	};

	class MeshInfo {
	public:				
		enum MFormat {			// Mesh format
			UDef = 0,
			VV = 1,				//   Vertex-Vertex
			FV = 2,				//   Face-Vertex
			FVF = 3,
			WE = 4,				//   Winged-Edge
			CM = 5				//   Connected-Mesh
		};
		enum BFormat {			// Buffer format
			BVert = 0,
			BEdge = 1,
			BFace = 2,
		};
		enum AFormat {			// Extra Attribute formats
			APos = 0,
			AClr = 1,
			ANorm = 2,
			ATex = 3
		};
		static int BufSize ( MFormat m, BFormat b )			{ return miBufSize[(int) m][(int) b]; }
		static int miBufSize [MAX_MFORMAT][MAX_BFORMAT];
	};

#endif


