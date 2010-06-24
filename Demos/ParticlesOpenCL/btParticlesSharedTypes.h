/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006 - 2009 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#ifndef BT_SPHERES_GRID_DEMO_SHARED_TYPES
#define BT_SPHERES_GRID_DEMO_SHARED_TYPES

struct btSimParams
{
	float m_gravity[4];
	float m_worldMin[4];
	float m_worldMax[4];
	float m_cellSize[4];
	int   m_gridSize[4];

	float m_particleRad;
	float m_globalDamping;
	float m_boundaryDamping;
	float m_collisionDamping;

	float m_spring;
	float m_shear;
	float m_attraction;
	float m_dummy;
};

struct btInt2
{
	int x;
	int y;
};

struct btInt4
{
	int x;
	int y;
	int z;
	int w;
};



#endif

