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

#ifndef DEF_MESH
	#define DEF_MESH

	#include <vector>
	#include <gl/glut.h>

	#include "geomx.h"
	#include "mesh_info.h"	
	#include "vector.h"

	//#include "mfile.h"

	//#define MESH_DEBUG

	#ifdef MESH_DEBUG
		#define VERT_DELTA		10000
		#define EDGE_DELTA		20000
		#define FACE_DELTA		30000
	#else
		#define VERT_DELTA		0
		#define EDGE_DELTA		0
		#define FACE_DELTA		0
	#endif

	#define PLY_UINT			0
	#define PLY_INT				1
	#define PLY_FLOAT			2
	#define PLY_LIST			3
	#define PLY_VERTS			4
	#define PLY_FACES			5

	struct PlyProperty {
		char						type;
		std::string					name;
	};
	struct PlyElement {
		int							num;
		char						type;		// 0 = vert, 1 = face
		std::vector<PlyProperty>	prop_list;
	};

	class Mesh : public GeomX, public MeshInfo {
	public:
		Mesh ();	

		//virtual objType GetType ()			{ return 'mesh'; }
		
		// Distributed functions		
		//virtual void onUpdate ( objData dat, mint::Event* e );
		//void UpdateMesh ();

		// Generic functions
		void InitStatic ();
		void DrawGL ( float* viewmat );
		void DrawFaceGL ( float* viewmat );
		void Measure ();
		Mesh& operator= ( Mesh& op2 );

		// Load PLY mesh
		void LoadPly ( char* fname, float s );
		void AddPlyElement ( char typ, int n );
		void AddPlyProperty ( char typ, std::string name ); 
		void LoadPlyVerts ();
		void LoadPlyFaces ();
		int FindPlyElem ( char typ );
		int FindPlyProp ( int elem, std::string name );

		// Vertex, Face, Edge functions
		xref AddVert (float x, float y, float z )		{ return (this->*m_AddVertFunc) (x, y, z); }
		xref AddFaceFast (xref v1, xref v2, xref v3 )	{ return (this->*m_AddFaceFast3Func) (v1, v2, v3); }
		xref AddFaceFast (xref v1, xref v2, xref v3, xref v4 )	{ return (this->*m_AddFaceFast4Func) (v1, v2, v3, v4); }
		xref (Mesh::*m_AddVertFunc) (float x, float y, float z);
		xref (Mesh::*m_AddFaceFast3Func) (xref v1, xref v2, xref v3);
		xref (Mesh::*m_AddFaceFast4Func) (xref v1, xref v2, xref v3, xref v4);
		
		int NumVert ()	{ return NumElem ( m_Vbuf ); }
		int NumEdge ()	{ return NumElem ( m_Ebuf ); }
		int NumFace ()	{ return NumElem ( m_Fbuf ); }
		
		void IncFace ( int n )		{ m_CurrF += n; }
		void DebugHeap ();

		// FVF - Face-Vertex-Face Mesh
		void CreateFVF ();
		void ClearFVF ();
		void SetFuncFVF ();
		xref AddVertFVF ( float x, float y, float z );
		xref AddFaceFast3FVF ( xref v1, xref v2, xref v3 );
		xref AddFaceFast4FVF ( xref v1, xref v2, xref v3, xref v4 );
		VertFVF* GetVertFVF ( int n )		{ return (VertFVF*) (mBuf[m_Vbuf].data + n*mBuf[m_Vbuf].stride); }
		FaceFVF* GetFaceFVF ( int n )		{ return (FaceFVF*) (mBuf[m_Fbuf].data + n*mBuf[m_Fbuf].stride); }
		void* GetExtraFVF ( VertFVF* v )	{ return ((char*) v + miBufSize[(int) FVF][BVert]); }
		void ComputeNormalsFVF ();
		void SetNormalFVF ( int n, Vector3DF norm );
		void SetColorFVF ( int n, DWORD clr );
		void SmoothFVF ( int iter );
		void DebugFVF ();
		void DrawVertsFVF ( float* viewmat, int a, int b );
		void DrawFacesFVF ( float* viewmat, int a, int b );	

		// CM - Connected Mesh
		void CreateCM ();
		void SetFuncCM ();
		xref AddVertCM ( float x, float y, float z );
		xref AddFaceFast3CM ( xref v1, xref v2, xref v3 );
		xref AddFaceFast4CM ( xref v1, xref v2, xref v3, xref v4 );
		xref AddEdgeCM ( xref v1, xref v2 );
		xref FindEdgeCM ( xref v1, xref v2 );	
		VertCM* GetVertCM ( int n )		{ return (VertCM*) (mBuf[m_Vbuf].data + n*mBuf[m_Vbuf].stride); }
		EdgeCM* GetEdgeCM ( int n )		{ return (EdgeCM*) (mBuf[m_Ebuf].data + n*mBuf[m_Ebuf].stride); }
		FaceCM* GetFaceCM ( int n )		{ return (FaceCM*) (mBuf[m_Fbuf].data + n*mBuf[m_Fbuf].stride); }
		void* GetExtraCM ( VertCM* v )	{ return ((char*) v + miBufSize[(int) CM][BVert] ); }
		void DebugCM ();
		void DrawVertsCM ( float* viewmat, int a, int b );
		void DrawFacesCM ( float* viewmat, int a, int b );	
		void DrawEdgesCM ( float* viewmat, int a, int b );	

		MFormat GetMeshBufs ( char& v, char& e, char& f )	{ v = m_Vbuf; e = m_Ebuf; f = m_Fbuf; return m_Mform; }

	protected:
		MFormat		m_Mform;			// Mesh format
		char		m_Vbuf;
		char		m_Ebuf;
		char		m_Fbuf;

		int			m_CurrF;

		std::vector< PlyElement* >	m_Ply;
		//File		m_File;
		int			m_PlyCurrElem;

		static bool		mbInitStatic;

		Vector3DF	mT;
	};

#endif

