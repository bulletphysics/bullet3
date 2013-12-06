/*
CDTestFramework http://codercorner.com
Copyright (c) 2007-2008 Pierre Terdiman,  pierre@codercorner.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#include "stdafx.h"
#include "Terrain.h"

inline float NxAngle(const Point& v0, const Point& v1)
	{
	float cos = v0|v1;					// |v0|*|v1|*Cos(Angle)
	float sin = (v0^v1).Magnitude();	// |v0|*|v1|*Sin(Angle)
	return ::atan2(sin, cos);
	}

static float computeAngle(const Point* verts, const udword* refs, udword vref)
{
	udword e0=0,e2=0;
	if(vref==refs[0])
	{
		e0 = 2;
		e2 = 1;
	}
	else if(vref==refs[1])
	{
		e0 = 2;
		e2 = 0;
	}
	else if(vref==refs[2])
	{
		e0 = 0;
		e2 = 1;
	}
	else
	{
		assert(0);
	}
	Point edge0 = verts[refs[e0]] - verts[vref];
	Point edge1 = verts[refs[e2]] - verts[vref];

	return NxAngle(edge0, edge1);
}
static bool buildSmoothNormals(
	udword nbTris, udword nbVerts,
	const Point* verts,
	const udword* dFaces, const uword* wFaces,
	Point* normals,
	bool flip)
	{
	// Checkings
	if(!verts || !normals || !nbTris || !nbVerts)	return false;

	// Get correct destination buffers
	// - if available, write directly to user-provided buffers
	// - else get some ram and keep track of it
	Point* FNormals = new Point[nbTris];
	if(!FNormals) return false;

	// Compute face normals
	udword c = (flip!=0);
	for(udword i=0;i<nbTris;i++)
		{
		udword Ref0 = dFaces ? dFaces[i*3+0]   : wFaces ? wFaces[i*3+0]   : 0;
		udword Ref1 = dFaces ? dFaces[i*3+1+c] : wFaces ? wFaces[i*3+1+c] : 1;
		udword Ref2 = dFaces ? dFaces[i*3+2-c] : wFaces ? wFaces[i*3+2-c] : 2;

		FNormals[i] = (verts[Ref2]-verts[Ref0])^(verts[Ref1] - verts[Ref0]);
		assert(!FNormals[i].IsZero());
		FNormals[i].Normalize();
		}

	// Compute vertex normals
	memset(normals, 0, nbVerts*sizeof(Point));

	Point* TmpNormals = new Point[nbVerts];
	memset(TmpNormals, 0, nbVerts*sizeof(Point));
	for(udword i=0;i<nbTris;i++)
		{
		udword Ref[3];
		Ref[0] = dFaces ? dFaces[i*3+0] : wFaces ? wFaces[i*3+0] : 0;
		Ref[1] = dFaces ? dFaces[i*3+1] : wFaces ? wFaces[i*3+1] : 1;
		Ref[2] = dFaces ? dFaces[i*3+2] : wFaces ? wFaces[i*3+2] : 2;

		for(udword j=0;j<3;j++)
			{
			if(TmpNormals[Ref[j]].IsZero())
				TmpNormals[Ref[j]] = FNormals[i];
			}
		}

	for(udword i=0;i<nbTris;i++)
		{
		udword Ref[3];
		Ref[0] = dFaces ? dFaces[i*3+0] : wFaces ? wFaces[i*3+0] : 0;
		Ref[1] = dFaces ? dFaces[i*3+1] : wFaces ? wFaces[i*3+1] : 1;
		Ref[2] = dFaces ? dFaces[i*3+2] : wFaces ? wFaces[i*3+2] : 2;

		normals[Ref[0]] += FNormals[i] * computeAngle(verts, Ref, Ref[0]);
		normals[Ref[1]] += FNormals[i] * computeAngle(verts, Ref, Ref[1]);
		normals[Ref[2]] += FNormals[i] * computeAngle(verts, Ref, Ref[2]);
		}

	// Normalize vertex normals
	for(udword i=0;i<nbVerts;i++)
		{
		if(normals[i].IsZero())
			normals[i] = TmpNormals[i];
		assert(!normals[i].IsZero());
		normals[i].Normalize();
		}

	DELETEARRAY(TmpNormals);
	DELETEARRAY(FNormals);

	return true;
	}




#define ONE_OVER_RAND_MAX	(1.0f / float(RAND_MAX))
#define RAND_MAX_OVER_TWO   (RAND_MAX / 2 + 1)

TerrainData::TerrainData() :
	size	(0),
	nbVerts	(0),
	nbFaces	(0),
	offset	(0.0f),
	width	(0.0f),
	chaos	(0.0f),
	verts	(NULL),
	colors	(NULL),
	normals	(NULL),
	faces	(NULL)
{
}

TerrainData::~TerrainData()
{
	release();
}

void TerrainData::release()
{
	DELETEARRAY(normals);
	DELETEARRAY(faces);
	DELETEARRAY(colors);
	DELETEARRAY(verts);
}

void TerrainData::init(udword s, float o, float w, float c, bool flat, const Point* pos)
	{
	release();

	size	= s;
	offset	= o;
	width	= w;
	chaos	= c;
	nbVerts = size*size;
	nbFaces	= (size-1)*(size-1)*2;

	////////

	// Initialize terrain vertices
	verts = new Point[nbVerts];
	for(udword y=0;y<size;y++)
		{
		for(udword x=0;x<size;x++)
			{
			verts[x+y*size] = Point(float(x)-(float(size-1)*0.5f), 0.0f, float(y)-(float(size-1)*0.5f)) * width;
			}
		}

	// Initialize terrain colors
		{
		colors = new Point[nbVerts];
		for(udword y=0;y<size;y++)
			{
			for(udword x=0;x<size;x++)
				{
				colors[x+y*size] = Point(0.5f, 0.4f, 0.2f);
				}
			}
		}

	// Initialize terrain faces
	faces = new udword[nbFaces*3];
	udword k = 0;
	for(udword j=0;j<size-1;j++)
		{
		for(udword i=0;i<size-1;i++)
			{
			// Create first triangle
			faces[k++] = i   + j*size;
			faces[k++] = i   + (j+1)*size;
			faces[k++] = i+1 + (j+1)*size;

			// Create second triangle
			faces[k++] = i   + j*size;
			faces[k++] = i+1 + (j+1)*size;
			faces[k++] = i+1 + j*size;
			}
		}

	struct Local
		{
		static void _Compute(bool* done, Point* field, udword x0, udword y0, udword currentSize, float value, udword initSize)
			{
			// Compute new size
			currentSize>>=1;
			if(!currentSize) return;

			// Compute new heights
			float v0 = (value * float(rand()-RAND_MAX_OVER_TWO) * ONE_OVER_RAND_MAX);
			float v1 = (value * float(rand()-RAND_MAX_OVER_TWO) * ONE_OVER_RAND_MAX);
			float v2 = (value * float(rand()-RAND_MAX_OVER_TWO) * ONE_OVER_RAND_MAX);
			float v3 = (value * float(rand()-RAND_MAX_OVER_TWO) * ONE_OVER_RAND_MAX);
			float v4 = (value * float(rand()-RAND_MAX_OVER_TWO) * ONE_OVER_RAND_MAX);

			udword x1 = (x0+currentSize)				% initSize;
			udword x2 = (x0+currentSize+currentSize)	% initSize;
			udword y1 = (y0+currentSize)				% initSize;
			udword y2 = (y0+currentSize+currentSize)	% initSize;

			if(!done[x1 + y0*initSize])	field[x1 + y0*initSize].y = v0 + 0.5f * (field[x0 + y0*initSize].y + field[x2 + y0*initSize].y);
			if(!done[x0 + y1*initSize])	field[x0 + y1*initSize].y = v1 + 0.5f * (field[x0 + y0*initSize].y + field[x0 + y2*initSize].y);
			if(!done[x2 + y1*initSize])	field[x2 + y1*initSize].y = v2 + 0.5f * (field[x2 + y0*initSize].y + field[x2 + y2*initSize].y);
			if(!done[x1 + y2*initSize])	field[x1 + y2*initSize].y = v3 + 0.5f * (field[x0 + y2*initSize].y + field[x2 + y2*initSize].y);
			if(!done[x1 + y1*initSize])	field[x1 + y1*initSize].y = v4 + 0.5f * (field[x0 + y1*initSize].y + field[x2 + y1*initSize].y);

			done[x1 + y0*initSize] = true;
			done[x0 + y1*initSize] = true;
			done[x2 + y1*initSize] = true;
			done[x1 + y2*initSize] = true;
			done[x1 + y1*initSize] = true;

			// Recurse through 4 corners
			value *= 0.5f;
			_Compute(done, field, x0,	y0,	currentSize, value, initSize);
			_Compute(done, field, x0,	y1,	currentSize, value, initSize);
			_Compute(done, field, x1,	y0,	currentSize, value, initSize);
			_Compute(done, field, x1,	y1,	currentSize, value, initSize);
			}
		};

	// Fractalize
	srand(42);
	bool* done = new bool[nbVerts];
	memset(done,0,nbVerts*sizeof(bool));
	verts[0].y = 10.0f;
	verts[size-1].y = 10.0f;
	verts[size*(size-1)].y = 10.0f;
	verts[nbVerts-1].y = 10.0f;
	Local::_Compute(done, verts, 0, 0, size, chaos, size);
	for(udword i=0;i<nbVerts;i++)	verts[i].y += offset;
	delete[] done;
	done=NULL;

	// Create a flat area in our terrain
	if(flat)
		{
		udword a = ((size)/2) - ((size)/8);
		udword b = ((size)/2) + ((size)/8);
		for(udword y=a;y<b;y++)
			{
			for(udword x=a;x<b;x++)
				{
				verts[x+y*size].y = 0.0f;
				colors[x+y*size].x = 0.3f;
				colors[x+y*size].y = 0.3f;
				colors[x+y*size].z = 0.3f;
				}
			}
		}

	if(pos)
		{
		for(udword y=0;y<size;y++)
			for(udword x=0;x<size;x++)
				verts[x+y*size] += *pos;
		}

	// Build vertex normals
	normals = new Point[nbVerts];
	buildSmoothNormals(nbFaces, nbVerts, verts, faces, NULL, normals, true);
	}

static void renderTerrain(const TerrainData& terrain, bool addWireframe)
{
	float* pVertList = new float[terrain.nbFaces*3*3];
	float* pNormList = new float[terrain.nbFaces*3*3];
	float* pColorList = new float[terrain.nbFaces*4*3];

	const udword* faces = terrain.faces;
	const Point* colors = terrain.colors;
	const Point* normals = terrain.normals;
	const Point* verts = terrain.verts;

	int vertIndex = 0;
	int normIndex = 0;
	int colorIndex = 0;
	for(int i=0;i<(int)terrain.nbFaces;i++)
	{
		for(int j=0;j<3;j++)
		{
			pVertList[vertIndex++] = verts[faces[i*3+j]].x;
			pVertList[vertIndex++] = verts[faces[i*3+j]].y;
			pVertList[vertIndex++] = verts[faces[i*3+j]].z;

			pNormList[normIndex++] = normals[faces[i*3+j]].x;
			pNormList[normIndex++] = normals[faces[i*3+j]].y;
			pNormList[normIndex++] = normals[faces[i*3+j]].z;

			pColorList[colorIndex++] = colors[faces[i*3+j]].x;
			pColorList[colorIndex++] = colors[faces[i*3+j]].y;
			pColorList[colorIndex++] = colors[faces[i*3+j]].z;
			pColorList[colorIndex++] = 1.0f;
		}
	}

    glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT, 0, pVertList);
    glEnableClientState(GL_NORMAL_ARRAY);
	glNormalPointer(GL_FLOAT, 0, pNormList);
    glColorPointer(4,GL_FLOAT, 0, pColorList);
    glEnableClientState(GL_COLOR_ARRAY);
    glDrawArrays(GL_TRIANGLES, 0, terrain.nbFaces*3);

	if(addWireframe)
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

		glVertexPointer(3,GL_FLOAT, 0, pVertList);
		glNormalPointer(GL_FLOAT, 0, pNormList);

		glDisableClientState(GL_COLOR_ARRAY);

		glLineWidth(2.0f);

		glColor4f(0.2f, 0.2f, 0.6f, 1.0f);
		glDrawArrays(GL_TRIANGLES, 0, terrain.nbFaces*3);

		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);	
	}

	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);

	delete[] pVertList;
	delete[] pNormList;
	delete[] pColorList;
}

static void renderTerrainTriangles(const TerrainData& terrain, udword nbTriangles, const udword* indices)
{
	float* pVertList = new float[nbTriangles*3*3];
	float* pNormList = new float[nbTriangles*3*3];

	const udword* faces = terrain.faces;
	const Point* normals = terrain.normals;
	const Point* verts = terrain.verts;

	int vertIndex = 0;
	int normIndex = 0;
	for(int i=0;i<(int)nbTriangles;i++)
	{
		udword index = *indices++;

		for(int j=0;j<3;j++)
		{
			pVertList[vertIndex++] = verts[faces[index*3+j]].x;
			pVertList[vertIndex++] = verts[faces[index*3+j]].y;
			pVertList[vertIndex++] = verts[faces[index*3+j]].z;

			pNormList[normIndex++] = normals[faces[index*3+j]].x;
			pNormList[normIndex++] = normals[faces[index*3+j]].y;
			pNormList[normIndex++] = normals[faces[index*3+j]].z;
		}
	}

    glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT, 0, pVertList);
    glEnableClientState(GL_NORMAL_ARRAY);
	glNormalPointer(GL_FLOAT, 0, pNormList);
	glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
	glDrawArrays(GL_TRIANGLES, 0, nbTriangles*3);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);

	delete[] pVertList;
	delete[] pNormList;
}







static TerrainData* gTerrainData = NULL;
static Model* gOpcodeModel = null;
static MeshInterface gMeshInterface;

const Model* GetTerrainModel()
{
	return gOpcodeModel;
}

void ReleaseTerrain()
{
	DELETESINGLE(gOpcodeModel);
	DELETESINGLE(gTerrainData);
}

void CreateTerrain()
{
	/////////////
	#define TERRAIN_SIZE	64
	#define TERRAIN_CHAOS	30.0f

	#define TERRAIN_OFFSET	-10.0f
	#define TERRAIN_WIDTH	2.0f

	ReleaseTerrain();

	gTerrainData = new TerrainData;
	gTerrainData->init(TERRAIN_SIZE, TERRAIN_OFFSET, TERRAIN_WIDTH, TERRAIN_CHAOS);

	// Build OPCODE model

	gMeshInterface.SetNbTriangles(gTerrainData->nbFaces);
	gMeshInterface.SetNbVertices(gTerrainData->nbVerts);
	gMeshInterface.SetPointers((const IndexedTriangle*)gTerrainData->faces, gTerrainData->verts);

	OPCODECREATE Create;
	Create.mIMesh			= &gMeshInterface;
	Create.mSettings.mLimit	= 1;
	Create.mSettings.mRules	= SPLIT_SPLATTER_POINTS|SPLIT_GEOM_CENTER;
	Create.mNoLeaf			= true;
	Create.mQuantized		= true;
	Create.mKeepOriginal	= false;
	Create.mCanRemap		= false;

	gOpcodeModel = new Model;
	if(!gOpcodeModel->Build(Create))
	{
	}
}

void RenderTerrain()
{
	if(gTerrainData)
		renderTerrain(*gTerrainData, true);
}

void RenderTerrainTriangles(udword nbTriangles, const udword* indices)
{
	if(gTerrainData)
		renderTerrainTriangles(*gTerrainData, nbTriangles, indices);
}
