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
#ifndef TERRAIN_H
#define TERRAIN_H

	class TerrainData
	{
		public:
					TerrainData();
					~TerrainData();

		void		init(udword size, float offset, float width, float chaos, bool flat=false, const Point* pos=NULL);
		void		release();

		udword		size;
		udword		nbVerts;
		udword		nbFaces;
		float		offset;
		float		width;
		float		chaos;
		Point*		verts;
		Point*		colors;
		Point*		normals;
		udword*		faces;
	};

	void			CreateTerrain();
	void			ReleaseTerrain();

	void			RenderTerrain();
	void			RenderTerrainTriangles(udword nbTriangles, const udword* indices);

	const Model*	GetTerrainModel();

#endif
