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
#include "OBBMeshQuery.h"
#include "Terrain.h"
#include "IceHelpers.h"
#include "RenderingHelpers.h"
#include "Camera.h"
#include "GLFontRenderer.h"

OBBMeshQuery::OBBMeshQuery() :
	mBar			(null),
	mAngleX			(0.0f),
	mAngleY			(0.0f),
	mAngleZ			(0.0f),
	mDist			(0.0f),
	mValidHit		(false)
{
}

OBBMeshQuery::~OBBMeshQuery()
{
}

void OBBMeshQuery::Init()
{
	mBox.mCenter = Point(0.0f, 0.0f, 0.0f);
	mBox.mExtents = Point(1.0f, 1.0f, 1.0f);
	mBox.mRot.Identity();
}

void OBBMeshQuery::Release()
{
	Deselect();
}

void OBBMeshQuery::Select()
{
	// Create a tweak bar
	{
		mBar = TwNewBar("OBBMeshQuery");
		TwAddVarRW(mBar, "Extents.x", TW_TYPE_FLOAT, &mBox.mExtents.x, " min=0.01 max=200.0 step=0.05 group='Extents' ");
		TwAddVarRW(mBar, "Extents.y", TW_TYPE_FLOAT, &mBox.mExtents.y, " min=0.01 max=200.0 step=0.05 group='Extents' ");
		TwAddVarRW(mBar, "Extents.z", TW_TYPE_FLOAT, &mBox.mExtents.z, " min=0.01 max=200.0 step=0.05 group='Extents' ");
		TwAddVarRW(mBar, "Rot.x", TW_TYPE_FLOAT, &mAngleX, " min=0.0 max=6.28 step=0.01 group='Rotation' ");
		TwAddVarRW(mBar, "Rot.y", TW_TYPE_FLOAT, &mAngleY, " min=0.0 max=6.28 step=0.01 group='Rotation' ");
		TwAddVarRW(mBar, "Rot.z", TW_TYPE_FLOAT, &mAngleZ, " min=0.0 max=6.28 step=0.01 group='Rotation' ");

		mSettings.AddToTweakBar(mBar);

//		mProfiler.AddToTweakBar(mBar);
	}
}

void OBBMeshQuery::Deselect()
{
	if(mBar)
	{
		TwDeleteBar(mBar);
		mBar = null;
	}
}

void OBBMeshQuery::PerformTest()
{
	RenderTerrain();

	Matrix3x3 MX,MY,MZ;
	RotX(MX, mAngleX);
	RotY(MY, mAngleY);
	RotY(MZ, mAngleZ);
	mBox.mRot = MX * MY * MZ;

	DrawOBB(mBox);

	const Model* TM = GetTerrainModel();
	if(TM)
	{
		OBBCollider Collider;
		mSettings.SetupCollider(Collider);

		mProfiler.Start();
		bool Status = Collider.Collide(mCache, mBox, *TM, null, null);
		mProfiler.End();
		mProfiler.Accum();

		if(Status)
		{
			if(Collider.GetContactStatus())
			{
				udword NbTris = Collider.GetNbTouchedPrimitives();
				const udword* Indices = Collider.GetTouchedPrimitives();

				RenderTerrainTriangles(NbTris, Indices);
			}
		}
	}

	// Raycast hit
	if(mValidHit)
	{
		Point wp = mLocalHit + mBox.mCenter;
		DrawLine(wp, wp + Point(1.0f, 0.0f, 0.0f), Point(1,0,0), 1.0f);
		DrawLine(wp, wp + Point(0.0f, 1.0f, 0.0f), Point(0,1,0), 1.0f);
		DrawLine(wp, wp + Point(0.0f, 0.0f, 1.0f), Point(0,0,1), 1.0f);
	}

	char Buffer[4096];
	sprintf(Buffer, "OBB-mesh query = %5.1f us (%d cycles)\n", mProfiler.mMsTime, mProfiler.mCycles);
	GLFontRenderer::print(10.0f, 10.0f, 0.02f, Buffer);
}

void OBBMeshQuery::KeyboardCallback(unsigned char key, int x, int y)
{
}

void OBBMeshQuery::MouseCallback(int button, int state, int x, int y)
{
	mValidHit = false;
	if(!button && !state)
	{
		Point Dir = ComputeWorldRay(x, y);

		float d;
		Point hit;
		if(RayOBB(GetCameraPos(), Dir, mBox, d, hit))
		{
			mValidHit = true;
			mDist = d;
			mLocalHit = hit - mBox.mCenter;
		}
	}
}

void OBBMeshQuery::MotionCallback(int x, int y)
{
	if(mValidHit)
	{
		Point Dir = ComputeWorldRay(x, y);
		mBox.mCenter = GetCameraPos() + Dir*mDist - mLocalHit;
	}
}


