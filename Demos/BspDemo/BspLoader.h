/*
===========================================================================
Copyright (C) 1999-2005 Id Software, Inc.

This file is part of Quake III Arena source code.

Quake III Arena source code is free software; you can redistribute it
and/or modify it under the terms of the GNU bteral Public License as
published by the Free Software Foundation; either version 2 of the License,
or (at your option) any later version.

Quake III Arena source code is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU bteral Public License for more details.

You should have received a copy of the GNU bteral Public License
along with Foobar; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
===========================================================================
*/



#ifndef BSP_LOADER_H
#define BSP_LOADER_H

#include "LinearMath/btAlignedObjectArray.h"

#define	BSPMAXTOKEN	1024
#define	BSPMAX_KEY				32
#define	BSPMAX_VALUE			1024
#define	BSPCONTENTS_SOLID			1
#define	BSPCONTENTS_AREAPORTAL		0x8000
#define	BSPLUMP_ENTITIES		0
#define	BSPLUMP_SHADERS		1
#define	BSPLUMP_PLANES			2
#define	BSPLUMP_NODES			3
#define	BSPLUMP_LEAFS			4
#define	BSPLUMP_LEAFSURFACES	5
#define	BSPLUMP_LEAFBRUSHES	6
#define	LUMP_MODELS			7
#define	LUMP_BRUSHES		8
#define	LUMP_BRUSHSIDES		9
#define	LUMP_DRAWVERTS		10
#define	LUMP_DRAWINDEXES	11
#define	LUMP_SURFACES		13
#define	LUMP_LIGHTMAPS		14
#define	LUMP_LIGHTGRID		15
#define	LUMP_VISIBILITY		16
#define	HEADER_LUMPS		17
#define	MAX_QPATH		64



typedef struct {
	int		fileofs, filelen;
} BSPLump;

typedef float BSPVector3[3];

typedef struct {
	int			ident;
	int			version;
	
	BSPLump		lumps[HEADER_LUMPS];
} BSPHeader;


typedef struct {
	float		mins[3], maxs[3];
	int			firstSurface, numSurfaces;
	int			firstBrush, numBrushes;
} BSPModel;

typedef struct {
	char		shader[MAX_QPATH];
	int			surfaceFlags;
	int			contentFlags;
} BSPShader;

typedef struct {
	float		normal[3];
	float		dist;
} BSPPlane;

typedef struct {
	int			planeNum;
	int			children[2];
	int			mins[3];
	int			maxs[3];
} BSPNode;

typedef struct {
	int			cluster;	
	int			area;
	
	int			mins[3];	
	int			maxs[3];
	
	int			firstLeafSurface;
	int			numLeafSurfaces;
	
	int			firstLeafBrush;
	int			numLeafBrushes;
} BSPLeaf;

typedef struct {
	int			planeNum;	
	int			shaderNum;
} BSPBrushSide;

typedef struct {
	int			firstSide;
	int			numSides;
	int			shaderNum;	
} BSPBrush;




typedef struct BSPPair {
	struct BSPPair	*next;
	char	*key;
	char	*value;
} BSPKeyValuePair;

typedef struct {
	BSPVector3		origin;
	struct bspbrush_s	*brushes;
	struct parseMesh_s	*patches;
	int			firstDrawSurf;
	BSPKeyValuePair		*epairs;
} BSPEntity;

typedef enum {
	MST_BAD,
		MST_PLANAR,
		MST_PATCH,
		MST_TRIANGLE_SOUP,
		MST_FLARE
} BSPMapSurface;

typedef struct {
	int			shaderNum;
	int			fogNum;
	int			surfaceType;
	
	int			firstVert;
	int			numVerts;
	
	int			firstIndex;
	int			numIndexes;
	
	int			lightmapNum;
	int			lightmapX, lightmapY;
	int			lightmapWidth, lightmapHeight;
	
	BSPVector3		lightmapOrigin;
	BSPVector3		lightmapVecs[3];
	
	int			patchWidth;
	int			patchHeight;
} BSPSurface;



///GPL code from IdSofware to parse a Quake 3 BSP file
///check that your platform define __BIG_ENDIAN__ correctly (in BspLoader.cpp)
class BspLoader
{
	int m_Endianness;

	public:

		BspLoader();

		bool	loadBSPFile( void* memoryBuffer);

		const char* getValueForKey( const BSPEntity *ent, const char *key ) const;

		bool	getVectorForKey( const BSPEntity *ent, const char *key, BSPVector3 vec );
		
		float	getFloatForKey( const BSPEntity *ent, const char *key );

		void parseEntities( void );

		bool findVectorByName(float* outvec,const char* name);

		const BSPEntity * getEntityByValue( const char* name, const char* value);


	protected:

		void parseFromMemory (char *buffer, int size);
		


		bool isEndOfScript (bool crossline);

		bool getToken (bool crossline);

		char *copystring(const char *s);
	
		void stripTrailing( char *e );

		BSPKeyValuePair * parseEpair( void );

		bool	parseEntity( void );

		short   isLittleShort (short l);
		int    isLittleLong (int l);
		float	isLittleFloat (float l);

		int    isBigLong (int l);
		short   isBigShort (short l);
		float	isBigFloat (float l);

		void swapBlock( int *block, int sizeOfBlock );

		int copyLump( BSPHeader	*header, int lump, void *dest, int size );

		void swapBSPFile( void );
		
	

	
	public: //easier for conversion
		int			m_num_entities;
		btAlignedObjectArray<BSPEntity>	m_entities;
		
		int			m_nummodels;
		btAlignedObjectArray<BSPModel>	m_dmodels;

		int			m_numShaders;
		btAlignedObjectArray<BSPShader>	m_dshaders;

		int			m_entdatasize;
		btAlignedObjectArray<char>		m_dentdata;

		int			m_numleafs;
		btAlignedObjectArray<BSPLeaf>		m_dleafs;

		int			m_numplanes;
		btAlignedObjectArray<BSPPlane>	m_dplanes;

		int			m_numnodes;
		btAlignedObjectArray<BSPNode>		m_dnodes;

		int			m_numleafsurfaces;
		btAlignedObjectArray<int>			m_dleafsurfaces;

		int			m_numleafbrushes;
		btAlignedObjectArray<int>			m_dleafbrushes;

		int			m_numbrushes;
		btAlignedObjectArray<BSPBrush>	m_dbrushes;

		int			m_numbrushsides;
		btAlignedObjectArray<BSPBrushSide>	m_dbrushsides;

		int			m_numLightBytes;
		btAlignedObjectArray<unsigned char>		m_lightBytes;

		int			m_numGridPoints;
		btAlignedObjectArray<unsigned char>		m_gridData;

		int			m_numVisBytes;
		btAlignedObjectArray<unsigned char>		m_visBytes;

		
		int			m_numDrawIndexes;
		btAlignedObjectArray<int>			m_drawIndexes;

		int			m_numDrawSurfaces;
		btAlignedObjectArray<BSPSurface>	m_drawSurfaces;

		enum
		{
			BSP_LITTLE_ENDIAN  = 0,
			BSP_BIG_ENDIAN    =  1
		};

		//returns machines big endian / little endian
		//
		int getMachineEndianness();

		inline int	machineEndianness()
		{
			return m_Endianness;
		}

};

#endif //BSP_LOADER_H
