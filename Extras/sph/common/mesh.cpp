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

#include <conio.h>
#include <vector.h>
#include "mesh.h"
#include "mdebug.h"
//#include "mfile.h"

//#include "event.h"

#include <gl/glut.h>

#define DEGtoRAD	(3.141592/180.0)

bool Mesh::mbInitStatic = false;
int Mesh::miBufSize[MAX_MFORMAT][MAX_BFORMAT];

Mesh::Mesh ()
{
	debug.SendToConsole ( true );

	m_Mform = MFormat::UDef;
	m_CurrF = 0;
	m_Vbuf = BUF_UNDEF;
	m_Ebuf = BUF_UNDEF;
	m_Fbuf = BUF_UNDEF;
}

Mesh& Mesh::operator= ( Mesh& src )
{
	CopyBuffers ( src );
	CopyAttributes ( src );
	m_Mform = src.GetMeshBufs ( m_Vbuf, m_Ebuf, m_Fbuf );
	return *this;
}

void Mesh::InitStatic ()
{
	mbInitStatic = true;
	miBufSize[ (int) FVF ][ (int) BVert ] =	sizeof ( VertFVF );
	miBufSize[ (int) FVF ][ (int) BFace ] =	sizeof ( FaceFVF );
	miBufSize[ (int) FVF ][ (int) BEdge ] =	0;
	miBufSize[ (int) CM  ][ (int) BVert ] =	sizeof ( VertCM );
	miBufSize[ (int) CM  ][ (int) BEdge ] =	sizeof ( EdgeCM );
	miBufSize[ (int) CM  ][ (int) BFace ] =	sizeof ( FaceCM );

}

/*void Mesh::onUpdate ( objData dat, mint::Event* e )
{
	uchar		dtype;
	hval		num;
	hval		max;
	long		size;
	ushort		stride;
	int			num_buf;

	switch ( dat ) {
	case 'full':
		ClearBuffers ();
		ClearAttributes ();
				
		m_Mform = (MFormat) e->getUChar ();
		m_Vbuf = e->getUChar ();
		m_Ebuf = e->getUChar ();
		m_Fbuf = e->getUChar ();
		num_buf = e->getInt ();
		for (int b=0; b < num_buf; b++) {
			dtype = e->getUChar ();
			stride = e->getUShort ();
			num = e->getInt ();
			max = e->getInt ();
			AddBuffer ( dtype, stride, max );
			e->getMem ( mBuf[b].data, num * mBuf[b].stride );
			mBuf[b].num = num;
		}

		mHeapNum = e->getInt ();
		mHeapMax = e->getInt ();
		mHeapFree = e->getInt ();
		mHeap = new hval[mHeapMax];
		e->getMem ( (char*) mHeap, mHeapNum * sizeof(hval) );
		break;
	};
}

void Mesh::UpdateMesh ()
{
	int s = GetSize () + (mBuf.size()+1)*4*sizeof(int);
	mint::Event* e = updateStart ( 'full', s );	

	e->attachUChar ( (char) m_Mform ) ;
	e->attachUChar ( (char) m_Vbuf );
	e->attachUChar ( (char) m_Ebuf );
	e->attachUChar ( (char) m_Fbuf );
	e->attachInt ( mBuf.size() );
	for (int b=0; b < mBuf.size(); b++) {
		e->attachUChar ( mBuf[b].dtype );
		e->attachUShort ( mBuf[b].stride );
		e->attachInt ( mBuf[b].num );
		e->attachInt ( mBuf[b].max );		
		e->attachMem ( mBuf[b].data, mBuf[b].num * mBuf[b].stride );
	}
	e->attachInt ( mHeapNum );
	e->attachInt ( mHeapMax );
	e->attachInt ( mHeapFree );
	e->attachMem ( (char*) mHeap, mHeapNum * sizeof(hval) );

 	updateEnd ( e );
}
*/

//------------------------------------------------------------------ FVF - Face-vertex-face Mesh
void Mesh::CreateFVF () 
{
	if ( !mbInitStatic ) InitStatic ();
	SetFuncFVF ();

	m_Mform = FVF;
	m_Vbuf = AddBuffer ( (uchar) BVert, BufSize( FVF, BVert ), 64 );
	m_Fbuf = AddBuffer ( (uchar) BFace, BufSize( FVF, BFace ), 64 );
	AddAttribute ( m_Vbuf, "pos", sizeof ( AttrPos ), false );
	AddAttribute ( m_Vbuf, "norm", sizeof ( AttrNorm ) );

	AddHeap ( 128 );
}

void Mesh::ClearFVF ()
{
	ResetBuffer ( 0, mBuf[0].max );
	ResetBuffer ( 1, mBuf[0].max );

	ResetHeap ();
}

void Mesh::SmoothFVF ( int iter )
{
	Vector3DF norm, side;
	int cnt;
	FaceFVF* f;
	VertFVF *v1, *v2, *v3;
	AttrPos* face_pos;
	face_pos = new AttrPos[ NumFace() ];

	for (int j=0; j < iter; j++) {
		// Compute centroid of all faces
		for (int n=0; n < NumFace(); n++) {
			f = GetFaceFVF ( n );
			v1 = GetVertFVF(f->v1);	v2 = GetVertFVF(f->v2);	v3 = GetVertFVF(f->v3);
			face_pos[n].x = ( v1->x + v2->x + v3->x ) / 3.0;
			face_pos[n].y = ( v1->y + v2->y + v3->y ) / 3.0;
			face_pos[n].z = ( v1->z + v2->z + v3->z ) / 3.0;
		}
		// Compute new vertex positions
		int cnt;
		Vector3DF vec;
		for (int n=0; n < NumVert(); n++) {
			v1 = GetVertFVF ( n );
			vec.Set (0,0,0);
			hval* fptr = mHeap + v1->flist.pos;	
			for (int j=0; j < v1->flist.cnt; j++) {
				vec.x += face_pos[ (*fptr) ].x;
				vec.y += face_pos[ (*fptr) ].y;
				vec.z += face_pos[ (*fptr) ].z;
				fptr++;
			}
			v1->x = vec.x / (float) v1->flist.cnt;
			v1->y = vec.y / (float) v1->flist.cnt;
			v1->z = vec.z / (float) v1->flist.cnt;
		}
	}
	delete face_pos;
}

void Mesh::SetNormalFVF ( int n, Vector3DF norm )
{
	VertFVF* v1;
	AttrNorm* vn;
	int noff = GetAttrOffset ( "norm" );
	v1 = GetVertFVF ( n );
	vn = (AttrNorm* ) ((char*) v1 + noff );
	vn->nx = norm.x;
	vn->ny = norm.y;
	vn->nz = norm.z;
}


void Mesh::SetColorFVF ( int n, DWORD clr )
{
	VertFVF* v1;
	AttrClr* vc;
	int coff = GetAttrOffset ( "color" );
	if ( coff == -1 ) return;
	v1 = GetVertFVF ( n );
	vc = (AttrClr* ) ((char*) v1 + coff );
	vc->clr = clr;
}
void Mesh::ComputeNormalsFVF ()
{
	Vector3DF norm, side;
	FaceFVF* f;
	VertFVF *v1, *v2, *v3, *v4;
	AttrNorm* vn;
	AttrNorm* face_norms;
	face_norms = new AttrNorm[ NumFace() ];

	// Clear vertex normals
	int noff = GetAttrOffset ( "norm" );
	for (int n=0; n < NumVert(); n++) {
		v1 = GetVertFVF ( n );
		vn = (AttrNorm*) ((char*) v1 + noff);
		vn->nx = 0;
		vn->ny = 0;
		vn->nz = 0;
	}

	// Compute normals of all faces
	for (int n=0; n < NumFace(); n++) {
		f = GetFaceFVF ( n );
		v1 = GetVertFVF(f->v1);	v2 = GetVertFVF(f->v2);	v3 = GetVertFVF(f->v3);
		side = Vector3DF ( v2->x, v2->y, v2->z );
		side -= Vector3DF ( v1->x, v1->y, v1->z );
		side.Normalize ();
		norm = Vector3DF ( v3->x, v3->y, v3->z );
		norm -= Vector3DF ( v1->x, v1->y, v1->z );
		norm.Normalize ();
		norm.Cross ( side );
		face_norms[n].nx = norm.x;
		face_norms[n].ny = norm.y;
		face_norms[n].nz = norm.z;
		vn = (AttrNorm*) ((char*) v1 + noff);	vn->nx += norm.x; vn->ny += norm.y;	vn->nz += norm.z;
		vn = (AttrNorm*) ((char*) v2 + noff);	vn->nx += norm.x; vn->ny += norm.y;	vn->nz += norm.z;
		vn = (AttrNorm*) ((char*) v3 + noff);	vn->nx += norm.x; vn->ny += norm.y;	vn->nz += norm.z;
		if ( f->v4 != -1 ) {
			v4 = GetVertFVF(f->v4);
			vn = (AttrNorm*) ((char*) v4 + noff);	vn->nx += norm.x; vn->ny += norm.y;	vn->nz += norm.z;
		}
	}

	// Normalize vertex normals
	Vector3DF vec;
	for (int n=0; n < NumVert(); n++) {
		v1 = GetVertFVF ( n );
		vn = (AttrNorm*) ((char*) v1 + noff);
		vec.Set ( vn->nx, vn->ny, vn->nz );
		vec.Normalize ();
		vn->nx = vec.x;
		vn->ny = vec.y;
		vn->nz = vec.z;
	}
	
	// Compute normal of a vertex from surrounding faces (slow method)
	/*int cnt;
	for (int n=0; n < NumVert(); n++) {
		v1 = GetVertFVF ( n );
		vn = (VertNorm*) GetExtraFVF ( v1 );
		cnt = 0;
		vn->nx = 0; vn->ny = 0; vn->nz = 0;
		hval* fptr = mHeap + v1->flist.pos;	
		for (int j=0; j < v1->flist.cnt; j++) {
			vn->nx += face_norms[ (*fptr) ].nx;
			vn->ny += face_norms[ (*fptr) ].ny;
			vn->nz += face_norms[ (*fptr) ].nz;
			cnt++;
			fptr++;
		}
		vn->nx /= (float) cnt;
		vn->ny /= (float) cnt;
		vn->nz /= (float) cnt;
	}*/

	delete face_norms;
}

void Mesh::SetFuncFVF ()
{
	m_AddVertFunc = &Mesh::AddVertFVF;
	m_AddFaceFast3Func = &Mesh::AddFaceFast3FVF;
	m_AddFaceFast4Func = &Mesh::AddFaceFast4FVF;
}

xref Mesh::AddFaceFast3FVF ( xref v1, xref v2, xref v3 )
{
	xref fNdx;	
	FaceFVF* f = (FaceFVF*) AddElem ( m_Fbuf, fNdx );
	f->v1 = v1;	f->v2 = v2;	f->v3 = v3; f->v4 = -1;
	AddRef ( fNdx, GetVertFVF(v1)->flist, FACE_DELTA );	
	AddRef ( fNdx, GetVertFVF(v2)->flist, FACE_DELTA );
	AddRef ( fNdx, GetVertFVF(v3)->flist, FACE_DELTA );
	return fNdx;
}
xref Mesh::AddFaceFast4FVF ( xref v1, xref v2, xref v3, xref v4 )
{
	xref fNdx;	
	FaceFVF* f = (FaceFVF*) AddElem ( m_Fbuf, fNdx );
	f->v1 = v1;	f->v2 = v2;	f->v3 = v3; f->v4 = v4;
	AddRef ( fNdx, GetVertFVF(v1)->flist, FACE_DELTA );	
	AddRef ( fNdx, GetVertFVF(v2)->flist, FACE_DELTA );
	AddRef ( fNdx, GetVertFVF(v3)->flist, FACE_DELTA );
	AddRef ( fNdx, GetVertFVF(v4)->flist, FACE_DELTA );
	return fNdx;
}
xref Mesh::AddVertFVF ( float x, float y, float z )
{
	xref ndx;
	VertFVF* v = (VertFVF*) AddElem ( m_Vbuf, ndx );
	v->x = x; v->y = y; v->z = z;
	ClearRefs ( v->flist );
	return ndx;
}

void Mesh::DebugFVF ()
{
	int n;
	int j;	
	VertFVF* v;
	FaceFVF* f;
	debug.Printf ( "-- MESH --\n");
	
	debug.Printf ( "-- verts\n" );
	for (n=0; n < NumVert(); n++) {
		v = GetVertFVF(n);
		debug.Printf ( "%d: (%2.1f,%2.1f,%2.1f) f:%d %d{", n, v->x, v->y, v->z, v->flist.cnt, v->flist.pos);
		if ( v->flist.cnt > 0 ) {
			for (j=0; j < v->flist.cnt; j++) 
				debug.Printf ( "%d ", *(mHeap+v->flist.pos+j) - FACE_DELTA );
		}
		debug.Printf ( "}\n" );	
	}
	debug.Printf ( "-- faces\n" );	
	for (n=0; n < NumFace(); n++) {
		f = GetFaceFVF(n);
		debug.Printf ( "%d: v:%d %d %d\n", n, f->v1, f->v2, f->v3);
	}

	DebugHeap ();
	
	debug.Printf ("\n\n");
	//_getch();
}


//------------------------------------------------------------------ CM - Connected Mesh
// Create Connected Mesh (CM)
void Mesh::CreateCM () 
{
	if ( !mbInitStatic ) InitStatic ();
	SetFuncCM ();

	m_Mform = CM;
	m_Vbuf = AddBuffer ( (uchar) BVert, BufSize( CM, BVert ), 64 );
	m_Ebuf = AddBuffer ( (uchar) BEdge, BufSize ( CM, BEdge), 64 );
	m_Fbuf = AddBuffer ( (uchar) BFace, BufSize ( CM, BFace), 64 );
	AddAttribute ( m_Vbuf, "pos", sizeof(AttrPos), false );
	AddAttribute ( m_Vbuf, "norm", sizeof(AttrNorm) );

	AddHeap ( 128 );
}

void Mesh::SetFuncCM ()
{
	m_AddVertFunc = &Mesh::AddVertCM;
	m_AddFaceFast3Func = &Mesh::AddFaceFast3CM;
	m_AddFaceFast4Func = &Mesh::AddFaceFast4CM;
}


xref Mesh::AddVertCM ( float x, float y, float z )
{
	xref ndx;
	VertCM* v = (VertCM*) AddElem ( m_Vbuf, ndx );
	v->x = x; v->y = y; v->z = z;
	ClearRefs ( v->elist );
	ClearRefs ( v->flist );
	return ndx;
}

xref Mesh::AddFaceFast3CM ( xref v1, xref v2, xref v3 )
{
	xref fNdx;	
	FaceCM* f = (FaceCM*) AddElem ( m_Fbuf, fNdx );
	f->v1 = v1;	f->v2 = v2;	f->v3 = v3; f->v4 = -1;
	xref eNdx;
	eNdx = AddEdgeCM ( f->v1, f->v2 );		f->e1 = eNdx;
	eNdx = AddEdgeCM ( f->v2, f->v3 );		f->e2 = eNdx;
	eNdx = AddEdgeCM ( f->v3, f->v1 );		f->e3 = eNdx;
	AddRef ( fNdx, GetVertCM(v1)->flist, FACE_DELTA );	
	AddRef ( fNdx, GetVertCM(v2)->flist, FACE_DELTA );
	AddRef ( fNdx, GetVertCM(v3)->flist, FACE_DELTA );
	return fNdx;
}
xref Mesh::AddFaceFast4CM ( xref v1, xref v2, xref v3, xref v4 )
{
	xref fNdx;	
	FaceCM* f = (FaceCM*) AddElem ( m_Fbuf, fNdx );
	f->v1 = v1;	f->v2 = v2;	f->v3 = v3; f->v4 = v4;
	xref eNdx;
	eNdx = AddEdgeCM ( f->v1, f->v2 );		f->e1 = eNdx;
	eNdx = AddEdgeCM ( f->v2, f->v3 );		f->e2 = eNdx;
	eNdx = AddEdgeCM ( f->v3, f->v4 );		f->e3 = eNdx;
	eNdx = AddEdgeCM ( f->v4, f->v1 );		f->e4 = eNdx;
	AddRef ( fNdx, GetVertCM(v1)->flist, FACE_DELTA );	
	AddRef ( fNdx, GetVertCM(v2)->flist, FACE_DELTA );
	AddRef ( fNdx, GetVertCM(v3)->flist, FACE_DELTA );
	AddRef ( fNdx, GetVertCM(v4)->flist, FACE_DELTA );
	return fNdx;
}

xref Mesh::FindEdgeCM ( xref v1, xref v2 )
{	
	EdgeCM *pE;
	VertCM *pV1;
	pV1 = GetVertCM(v1);	
	if ( pV1->elist.cnt == 0 ) return -1;
	hval* e = mHeap + pV1->elist.pos;
	#ifdef MESH_DEBUG
		for (int n=0; n < pV1->elist.cnt; n++) {
			pE = GetEdgeCM( (*e)-EDGE_DELTA );
			if ( pE->v1 == v2 || pE->v2 == v2 ) return (*e)-EDGE_DELTA;
			e++;
		}
	#else	
		for (int n=0; n < pV1->elist.cnt; n++) {
			pE = GetEdgeCM( *e );
			if ( pE->v1 == v2 || pE->v2 == v2 ) return *e;
			e++;
		}
	#endif
	return -1;
}

xref Mesh::AddEdgeCM ( xref v1, xref v2 )
{	
	xref eNdx = FindEdgeCM ( v1, v2 );
	EdgeCM* e = GetEdgeCM(eNdx);
	if ( eNdx == -1 ) {
		e = (EdgeCM*) AddElem ( m_Ebuf, eNdx );
		e->f1 = 0;
		e->f2 = 0;
		e->v1 = v1;
		e->v2 = v2;		
		AddRef ( eNdx, GetVertCM(v1)->elist, EDGE_DELTA );
		AddRef ( eNdx, GetVertCM(v2)->elist, EDGE_DELTA );		
	}
	return eNdx;
}

void Mesh::DebugCM ()
{
	int n;
	int j;	
	VertCM* v;
	EdgeCM* e;
	FaceCM* f;
	debug.Printf ( "-- MESH --\n");
	
	debug.Printf ( "-- verts\n" );
	for (n=0; n < NumVert(); n++) {
		v = GetVertCM(n);
		debug.Printf ( "%d: (%2.1f,%2.1f,%2.1f) e:%d %d{", n, v->x, v->y, v->z, v->elist.cnt, v->elist.pos);
		if ( v->elist.cnt > 0 ) { 
			for (j=0; j < v->elist.cnt; j++) 
				debug.Printf ( "%d ", *(mHeap+v->elist.pos+j) - EDGE_DELTA );
		}
		debug.Printf ( "}, f:%d %d{", v->flist.cnt, v->flist.pos);
		if ( v->flist.cnt > 0 ) {
			for (j=0; j < v->flist.cnt; j++) 
				debug.Printf ( "%d ", *(mHeap+v->flist.pos+j) - FACE_DELTA );
		}
		debug.Printf ( "}\n" );	
	}

	debug.Printf ( "-- edges\n" );	
	for (n=0; n < NumEdge(); n++) {
		e = GetEdgeCM (n);
		debug.Printf ( "%d: v:%d %d, f:%d %d\n", n, e->v1, e->v2, e->f1, e->f2 );		
	}

	debug.Printf ( "-- faces\n" );	
	for (n=0; n < NumFace(); n++) {
		f = GetFaceCM(n);
		debug.Printf ( "%d: v:%d %d %d, e:%d %d %d\n", n, f->v1, f->v2, f->v3, f->e1, f->e2, f->e3 );		
	}


	hval* pVal = mHeap;
	debug.Printf ( "-- heap (size: %d, max: %d, free: %04d)\n", mHeapNum, mHeapMax, mHeapFree );
	for (n=0; n < mHeapNum; n++) {
		if ( (n % 8) == 0 ) debug.Printf ( "\n[%04d] ", n );
		#ifdef MESH_DEBUG
			if ( *pVal == 0 ) {
				debug.Printf ( "00000 ");
			} else if (  *pVal == (hval) 0xFFFF ) {
				debug.Printf ( "----- ");		
			} else if ( *pVal >= VERT_DELTA && *pVal < EDGE_DELTA ) {
				debug.Printf ( "v%04d ", *pVal - VERT_DELTA );
			} else if ( *pVal >= EDGE_DELTA && *pVal < FACE_DELTA ) {
				debug.Printf ( "e%04d ", *pVal - EDGE_DELTA );
			} else if ( *pVal >= FACE_DELTA ) {
				debug.Printf ( "f%04d ", *pVal - FACE_DELTA );
			} else {
				debug.Printf ( "H%04d ", (int) *pVal );
			}			
		#else
			debug.Printf ( "%05d ", (int) *pVal );
		#endif
		pVal++;
	}
	
	debug.Printf ("\n\n");
	//_getch();
}

void Mesh::DebugHeap ()
{
	hval* pVal = mHeap;
	debug.Printf ( "-- heap (size: %d, max: %d, free: %04d)\n", mHeapNum, mHeapMax, mHeapFree );
	for (int n=0; n < mHeapNum; n++) {
		if ( (n % 8) == 0 ) debug.Printf ( "\n[%04d] ", n );
		#ifdef MESH_DEBUG
			if ( *pVal == 0 ) {
				debug.Printf ( "00000 ");
			} else if (  *pVal == (hval) 0xFFFF ) {
				debug.Printf ( "----- ");		
			} else if ( *pVal >= VERT_DELTA && *pVal < EDGE_DELTA ) {
				debug.Printf ( "v%04d ", *pVal - VERT_DELTA );
			} else if ( *pVal >= EDGE_DELTA && *pVal < FACE_DELTA ) {
				debug.Printf ( "e%04d ", *pVal - EDGE_DELTA );
			} else if ( *pVal >= FACE_DELTA ) {
				debug.Printf ( "f%04d ", *pVal - FACE_DELTA );
			} else {
				debug.Printf ( "H%04d ", (int) *pVal );
			}			
		#else
			debug.Printf ( "%05d ", (int) *pVal );
		#endif
		pVal++;
	}
}

void Mesh::DrawVertsCM ( float* viewmat, int a, int b )
{
	VertCM* v;

	glColor3f (1,0,0);
	glLoadMatrixf ( viewmat );
	glTranslatef ( mT.x, mT.y, mT.z );
	glBegin ( GL_POINTS );
	for (int n = a; n <= b; n++) {	
		v = GetVertCM (n);
		glVertex3f ( v->x, v->y, v->z );
		//glCallList ( m_GLObj );		
	}
	glEnd ();
}

void Mesh::DrawVertsFVF ( float* viewmat, int a, int b )
{
	VertFVF* v;
	glColor3f (1,0,0);
	glLoadMatrixf ( viewmat );
	glTranslatef ( mT.x, mT.y, mT.z );
	glBegin ( GL_POINTS );
	for (int n = a; n <= b; n++) {	
		v = GetVertFVF (n);
		glVertex3f ( v->x, v->y, v->z );
		//glCallList ( m_GLObj );		
	}
	glEnd ();
}

void Mesh::DrawFacesCM ( float* viewmat, int a, int b )
{
	FaceCM* f;
	VertCM* v;
	AttrNorm* vn;
	int noff = GetAttrOffset ( "norm" );
	glLoadMatrixf ( viewmat );
	glTranslatef ( mT.x, mT.y, mT.z );
	GLenum dm = GL_TRIANGLES;
	glBegin ( dm );
	f = GetFaceCM ( a );
	for (int n = a; n <= b; n++) {
		if ( f->v4 == -1 ) {
			if ( dm != GL_TRIANGLES ) {	glEnd (); glBegin ( GL_TRIANGLES ); dm = GL_TRIANGLES; }
			v = GetVertCM(f->v1);	vn = (AttrNorm*) ((char*) v + noff);
			glNormal3f ( vn->nx, vn->ny, vn->nz );	glVertex3f ( v->x, v->y, v->z );
			v = GetVertCM(f->v2);	vn = (AttrNorm*) ((char*) v + noff);
			glNormal3f ( vn->nx, vn->ny, vn->nz );	glVertex3f ( v->x, v->y, v->z );
			v = GetVertCM(f->v3);	vn = (AttrNorm*) ((char*) v + noff);
			glNormal3f ( vn->nx, vn->ny, vn->nz );	glVertex3f ( v->x, v->y, v->z );
		} else {
			if ( dm != GL_QUADS )	{ glEnd (); glBegin ( GL_QUADS ); dm = GL_QUADS; }
			v = GetVertCM(f->v1);	vn = (AttrNorm*) ((char*) v + noff);
			glNormal3f ( vn->nx, vn->ny, vn->nz );	glVertex3f ( v->x, v->y, v->z );
			v = GetVertCM(f->v2);	vn = (AttrNorm*) ((char*) v + noff);
			glNormal3f ( vn->nx, vn->ny, vn->nz );	glVertex3f ( v->x, v->y, v->z );
			v = GetVertCM(f->v3);	vn = (AttrNorm*) ((char*) v + noff);
			glNormal3f ( vn->nx, vn->ny, vn->nz );	glVertex3f ( v->x, v->y, v->z );
			v = GetVertCM(f->v4);	vn = (AttrNorm*) ((char*) v + noff);
			glNormal3f ( vn->nx, vn->ny, vn->nz );	glVertex3f ( v->x, v->y, v->z );
		}
		f++;
	}	
	glEnd ();
}

void Mesh::DrawFacesFVF ( float* viewmat, int a, int b )
{
	FaceFVF* f;
	VertFVF* v;
	AttrNorm* vn;
	AttrClr* vc;
	int noff = GetAttrOffset ( "norm" );
	int coff = GetAttrOffset ( "color" );
	coff = -1;

	//glLoadMatrixf ( viewmat );
	//glTranslatef ( mT.x, mT.y, mT.z );
	GLenum dm = GL_TRIANGLES;
	glBegin ( dm );
	f = GetFaceFVF ( a );
	for (int n = a; n <= b; n++) {
		if ( f->v4 == -1 ) {
			if ( dm != GL_TRIANGLES ) {	glEnd (); glBegin ( GL_TRIANGLES ); dm = GL_TRIANGLES; }
			v = GetVertFVF(f->v1);	vn = (AttrNorm*) ((char*) v + noff);	vc = (AttrClr*) ((char*) v +coff);
			if ( coff != -1 ) glColor4f ( RED(vc->clr), GRN(vc->clr), BLUE(vc->clr), ALPH(vc->clr) );
			glNormal3f ( vn->nx, vn->ny, vn->nz );	glVertex3f ( v->x, v->y, v->z );
			
			v = GetVertFVF(f->v2);	vn = (AttrNorm*) ((char*) v + noff);	vc = (AttrClr*) ((char*) v +coff);
			if ( coff != -1 ) glColor4f ( RED(vc->clr), GRN(vc->clr), BLUE(vc->clr), ALPH(vc->clr) );
			glNormal3f ( vn->nx, vn->ny, vn->nz );	glVertex3f ( v->x, v->y, v->z );
			
			v = GetVertFVF(f->v3);	vn = (AttrNorm*) ((char*) v + noff);	vc = (AttrClr*) ((char*) v +coff);
			if ( coff != -1 ) glColor4f ( RED(vc->clr), GRN(vc->clr), BLUE(vc->clr), ALPH(vc->clr) );
			glNormal3f ( vn->nx, vn->ny, vn->nz );	glVertex3f ( v->x, v->y, v->z );
		} else {
			if ( dm != GL_QUADS )	{ glEnd (); glBegin ( GL_QUADS ); dm = GL_QUADS; }
			v = GetVertFVF(f->v1);	vn = (AttrNorm*) ((char*) v + noff);	vc = (AttrClr*) ((char*) v +coff);
			if ( coff != -1 ) glColor4f ( RED(vc->clr), GRN(vc->clr), BLUE(vc->clr), ALPH(vc->clr) );
			glNormal3f ( vn->nx, vn->ny, vn->nz );	glVertex3f ( v->x, v->y, v->z );
			
			v = GetVertFVF(f->v2);	vn = (AttrNorm*) ((char*) v + noff);	vc = (AttrClr*) ((char*) v +coff);
			if ( coff != -1 ) glColor4f ( RED(vc->clr), GRN(vc->clr), BLUE(vc->clr), ALPH(vc->clr) );
			glNormal3f ( vn->nx, vn->ny, vn->nz );	glVertex3f ( v->x, v->y, v->z );
			
			v = GetVertFVF(f->v3);	vn = (AttrNorm*) ((char*) v + noff);	vc = (AttrClr*) ((char*) v +coff);
			if ( coff != -1 ) glColor4f ( RED(vc->clr), GRN(vc->clr), BLUE(vc->clr), ALPH(vc->clr) );
			glNormal3f ( vn->nx, vn->ny, vn->nz );	glVertex3f ( v->x, v->y, v->z );
			
			v = GetVertFVF(f->v4);	vn = (AttrNorm*) ((char*) v + noff);	vc = (AttrClr*) ((char*) v +coff);
			if ( coff != -1 ) glColor4f ( RED(vc->clr), GRN(vc->clr), BLUE(vc->clr), ALPH(vc->clr) );
			glNormal3f ( vn->nx, vn->ny, vn->nz );	glVertex3f ( v->x, v->y, v->z );
		}
		f++;
	}	
	glEnd ();
}

void Mesh::DrawEdgesCM ( float* viewmat, int a, int b )
{
	EdgeCM* e;
	
	glLoadMatrixf ( viewmat );
	glTranslatef ( mT.x, mT.y, mT.z );
	glBegin ( GL_LINES );
	e = GetEdgeCM ( a );
	for (int n = a; n <= b; n++) {
		glVertex3f ( GetVertCM(e->v1)->x, GetVertCM(e->v1)->y, GetVertCM(e->v1)->z );
		glVertex3f ( GetVertCM(e->v2)->x, GetVertCM(e->v2)->y, GetVertCM(e->v2)->z );
		e++;
	}
	glEnd ();
}

void Mesh::DrawGL ( float* viewmat )
{
	mT.Set(0,0,0);
	
	switch ( m_Mform ) {
	case CM: {
		glDepthRange (0.001, 1.001);
		//glColor3f ( 1, 0, 0 );		DrawVertsCM ( viewmat, 0, NumVert()-1 );
		glColor3f ( .6, .6, .6 );	DrawFacesCM ( viewmat, 0, NumFace()-1 );
		//glDepthRange (0.0005, 1.0005);
		//glColor3f ( 1, 1, 1);		DrawEdgesCM ( viewmat, 0, NumEdge()-1 );	
		} break;
	case FVF: { 
		//glColor3f (1,0,0);			DrawVertsFVF ( viewmat, 0, NumVert()-1 );
		//glEnable (GL_LIGHTING);
		
		glPolygonMode ( GL_FRONT_AND_BACK, GL_FILL );
		glColor4f ( .9, .9, .9, 0.75 );	DrawFacesFVF ( viewmat, 0, NumFace()-1 );

		/*glDisable (GL_LIGHTING );
		glDepthRange (0.000, 1.00);
		glPolygonMode ( GL_FRONT_AND_BACK, GL_LINE );
		glLineWidth ( 3 );
		glColor4f ( 0, 0, 0, 1.0 );	DrawFacesFVF ( viewmat, 0, NumFace()-1 );		
		glEnable ( GL_LIGHTING );

		glLineWidth ( 1);

		glDepthRange (0.0, 1.0);
		glPolygonMode ( GL_FRONT_AND_BACK, GL_FILL );*/

		} break;
	}
}

void Mesh::DrawFaceGL ( float* viewmat )
{
	mT.Set (0,0,0);
	if ( m_CurrF < 0 ) m_CurrF = NumFace()-1;
	if ( m_CurrF >= NumFace() ) m_CurrF = 0;
	
	switch ( m_Mform ) {
	case FVF: 
		glDepthRange (0.0, 1.0); glColor3f (1.0, 1.0, 1.0 );	
		DrawFacesFVF ( viewmat, m_CurrF, m_CurrF );
		break;
	case CM:
		glDepthRange (0.0, 1.0); glColor3f (1.0, 1.0, 1.0 );	
		DrawFacesCM ( viewmat, m_CurrF, m_CurrF );
		break;
	};
}

void Mesh::Measure ()
{
	hval* pCurr = mHeap + mHeapFree;	
	int vs, es, fs, hs, hm, as, frees = 0;
	vs = NumVert(); if ( vs !=0 ) vs *= GetStride(m_Vbuf);
	es = NumEdge(); if ( es !=0 ) es *= GetStride(m_Ebuf);
	fs = NumFace(); if ( fs !=0 ) fs *= GetStride(m_Fbuf);
	hs = mHeapNum*sizeof(hval);
	hm = mHeapMax*sizeof(hval);
	
	while ( pCurr != mHeap-1 ) {
		frees += *(pCurr);
		pCurr = mHeap + * (hpos*) (pCurr + FPOS);
	}
	frees *= sizeof(hval);
	as = 0;
	if ( m_Vbuf!=-1 ) as += mBuf[m_Vbuf].max * GetStride(m_Vbuf);
	if ( m_Fbuf!=-1 ) as += mBuf[m_Fbuf].max * GetStride(m_Fbuf);
	if ( m_Ebuf!=-1 ) as += mBuf[m_Ebuf].max * GetStride(m_Ebuf);
	as += hm;

	debug.Printf ( "NumVert:     %07.1fk (%d)\n", vs/1000.0, NumVert() );
	debug.Printf ( "NumFace:     %07.1fk (%d)\n", fs/1000.0, NumFace() );
	debug.Printf ( "NumEdge:     %07.1fk (%d)\n", es/1000.0, NumEdge() );
	debug.Printf ( "Heap Size:   %07.1fk (%d)\n", hs/1000.0, mHeapNum );
	debug.Printf ( "Free Size:   %07.1fk\n", frees/1000.0 );
	debug.Printf ( "Heap Used:   %07.1fk (%5.1f%%)\n", (hs-frees)/1000.0, (hs-frees)*100.0/(vs+es+fs+hs-frees) );
	debug.Printf ( "Heap Max:    %07.1fk\n", hm/1000.0 );	
	debug.Printf ( "Total Used:  %07.1fk\n", (vs+es+fs+hs-frees)/1000.0 );
	debug.Printf ( "Total Alloc: %07.1fk\n", as/1000.0 );
	debug.Printf ( "Fragmentation: %f%%\n", (hm-(hs-frees))*100.0 / hm );
}


/*int Mesh::GetIndex ( int b, void* v )
{
	if ( v == 0x0 ) return -1;
	return ((char*) v - (char*) mBuf[b].data) / mBuf[b].stride;
}*/

int Mesh::FindPlyElem ( char typ )
{
	for (int n=0; n < m_Ply.size(); n++) {
		if ( m_Ply[n]->type == typ ) return n;
	}
	return -1;
}

int Mesh::FindPlyProp ( int elem, std::string name )
{
	for (int n=0; n < m_Ply[elem]->prop_list.size(); n++) {
		if ( m_Ply[elem]->prop_list[n].name.compare ( name)==0 )
			return n;
	}
	return -1;
}

void Mesh::LoadPly ( char* fname, float s  )
{
/*	int m_PlyCnt;
	float m_PlyData[40];
	char buf[1000];
	char bword[1000];
	std::string word;
	Buffer b(1000);
	int vnum, fnum, elem, cnt;
	char typ;

	if ( m_Mform == MFormat::UDef )
		CreateFVF ();

	m_File.Open ( fname, FILE_READ | FILE_SEQUENTIAL );	
	if ( !m_File.Valid() ) {
		error.PrintF ( "mesh", "Could not find file: %s\n", fname );
		error.Exit ();
	}
	
	// Read header
	m_File.ReadLine ( buf, 1000 );
	b.ReadWord ( buf, bword ); word = bword;
	if ( word.compare("ply" )!=0 ) {
		error.PrintF ( "Not a ply file. %s\n", fname );
		error.Exit ();
	}

	debug.Printf ( "Reading PLY.\n" ); 
	while ( m_File.ReadLine ( buf, 1000 ) == FILE_STATUS_OK ) {
		b.ReadWord ( buf, bword );
		word = bword;
		if ( word.compare("comment" )!=0 ) {
			if ( word.compare("end_header")==0 ) break;
			if ( word.compare("property")==0 ) {
				b.ReadWord ( buf, bword );
				word = bword;
				if ( word.compare("float")==0 ) typ = PLY_FLOAT;
				if ( word.compare("float16")==0 ) typ = PLY_FLOAT;
				if ( word.compare("float32")==0 ) typ = PLY_FLOAT;
				if ( word.compare("int8")==0 ) typ = PLY_INT;
				if ( word.compare("uint8")==0 ) typ = PLY_UINT;
				if ( word.compare("list")==0) {
					typ = PLY_LIST;
					b.ReadWord ( buf, bword );
					b.ReadWord ( buf, bword );
				}
				b.ReadWord ( buf, bword );
				word = bword;
				AddPlyProperty ( typ, word );
			}
			if ( word.compare("element" )==0 ) {
				b.ReadWord ( buf, bword);	word = bword;
				if ( word.compare("vertex")==0 ) {
					b.ReadWord ( buf, bword);
					vnum = atoi ( bword );
					debug.Printf ( "  Verts: %d\n", vnum );
					AddPlyElement ( PLY_VERTS, vnum );
				}
				if ( word.compare("face")==0 ) {
					b.ReadWord ( buf, bword);
					fnum = atoi ( bword );
					debug.Printf ( "  Faces: %d\n", fnum );
					AddPlyElement ( PLY_FACES, fnum );
				}
			}
		}		
	}

	// Read data
	int xi, yi, zi;
	debug.Printf ( "  Reading verts..\n" );	
	elem = FindPlyElem ( PLY_VERTS );
	xi = FindPlyProp ( elem, "x" );
	yi = FindPlyProp ( elem, "y" );
	zi = FindPlyProp ( elem, "z" );
	if ( elem == -1 || xi == -1 || yi == -1 || zi == -1 ) {
		debug.Printf ( "ERROR: Vertex data not found.\n" );
		exit(-1);
	}
	for (int n=0; n < m_Ply[elem]->num; n++) {
		m_File.ReadLine ( buf, 1000 );
		for (int j=0; j < m_Ply[elem]->prop_list.size(); j++) {
			b.ReadWord ( buf, bword );
			m_PlyData[ j ] = atof ( bword );
		}
		AddVert ( m_PlyData[xi]*s, m_PlyData[zi]*s, m_PlyData[yi]*s );
	}

	debug.Printf ( "  Reading faces..\n" );
	elem = FindPlyElem ( PLY_FACES );
	xi = FindPlyProp ( elem, "vertex_indices" );
	if ( elem == -1 || xi == -1 ) {
		debug.Printf ( "ERROR: Face data not found.\n" );
		exit(-1);
	}
	for (int n=0; n < m_Ply[elem]->num; n++) {
		m_File.ReadLine ( buf, 1000 );
		m_PlyCnt = 0;
		for (int j=0; j < m_Ply[elem]->prop_list.size(); j++) {
			if ( m_Ply[elem]->prop_list[j].type == PLY_LIST ) {
				b.ReadWord ( buf, bword );
				cnt = atoi ( bword );	
				m_PlyData[ m_PlyCnt++ ] = cnt;
				for (int c =0; c < cnt; c++) {
					b.ReadWord ( buf, bword );
					m_PlyData[ m_PlyCnt++ ] = atof ( bword );
				}
			} else {
				b.ReadWord ( buf, bword );
				m_PlyData[ m_PlyCnt++ ] = atof ( bword );
			}
		}
		if ( m_PlyData[xi] == 3 ) {
			//debug.Printf ( "    Face: %d, %d, %d\n", (int) m_PlyData[xi+1], (int) m_PlyData[xi+2], (int) m_PlyData[xi+3] );
			AddFaceFast ( (int) m_PlyData[xi+1], (int) m_PlyData[xi+2], (int) m_PlyData[xi+3] );
		}
		
		if ( m_PlyData[xi] == 4 ) {
			//debug.Printf ( "    Face: %d, %d, %d, %d\n", (int) m_PlyData[xi+1], (int) m_PlyData[xi+2], (int) m_PlyData[xi+3], (int) m_PlyData[xi+4]);
			AddFaceFast ( (int) m_PlyData[xi+1], (int) m_PlyData[xi+2], (int) m_PlyData[xi+3], (int) m_PlyData[xi+4] );
		}
	}

	Measure ();
	ComputeNormalsFVF ();		// !-- should be abstracted
*/

	//	UpdateMesh ();
}

void Mesh::AddPlyElement ( char typ, int n )
{
	debug.Printf ( "  Element: %d, %d\n", typ, n );
	PlyElement* p = new PlyElement;
	p->num = n;
	p->type = typ;
	p->prop_list.clear ();
	m_PlyCurrElem = m_Ply.size();
	m_Ply.push_back ( p );
}

void Mesh::AddPlyProperty ( char typ, std::string name )
{
	debug.Printf ( "  Property: %d, %s\n", typ, name.c_str() );
	PlyProperty p;
	p.name = name;
	p.type = typ;
	m_Ply [ m_PlyCurrElem ]->prop_list.push_back ( p );
}