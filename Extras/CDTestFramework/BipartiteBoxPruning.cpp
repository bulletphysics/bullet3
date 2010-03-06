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
#include "BipartiteBoxPruning.h"
#include "RenderingHelpers.h"
#include "GLFontRenderer.h"

BipartiteBoxPruningTest::BipartiteBoxPruningTest() :
	mBar			(null),
	mNbBoxes		(0),
	mBoxes			(null),
	mBoxPtrs		(null),
	mBoxTime		(null),
	mSpeed			(0.0f),
	mAmplitude		(100.0f)
{
}

BipartiteBoxPruningTest::~BipartiteBoxPruningTest()
{
	DELETEARRAY(mBoxTime);
	DELETEARRAY(mBoxPtrs);
	DELETEARRAY(mBoxes);
}

void BipartiteBoxPruningTest::Init()
{
	mNbBoxes	= 1024;
	mBoxes = new AABB[mNbBoxes];
	mBoxPtrs = new const AABB*[mNbBoxes];
	mBoxTime = new float[mNbBoxes];
	for(udword i=0;i<mNbBoxes;i++)
	{
		Point Center, Extents;

		Center.x = (UnitRandomFloat()-0.5f) * 100.0f;
		Center.y = (UnitRandomFloat()-0.5f) * 10.0f;
		Center.z = (UnitRandomFloat()-0.5f) * 100.0f;
		Extents.x = 2.0f + UnitRandomFloat() * 2.0f;
		Extents.y = 2.0f + UnitRandomFloat() * 2.0f;
		Extents.z = 2.0f + UnitRandomFloat() * 2.0f;

		mBoxes[i].SetCenterExtents(Center, Extents);
		mBoxPtrs[i] = &mBoxes[i];

		mBoxTime[i] = 2000.0f*UnitRandomFloat();
	}
}

void BipartiteBoxPruningTest::Release()
{
	DELETEARRAY(mBoxTime);
	DELETEARRAY(mBoxes);
}

void BipartiteBoxPruningTest::Select()
{
	// Create a tweak bar
	{
		mBar = TwNewBar("BipartiteBoxPruning");
		TwAddVarRW(mBar, "Speed", TW_TYPE_FLOAT, &mSpeed, " min=0.0 max=0.01 step=0.00001");
		TwAddVarRW(mBar, "Amplitude", TW_TYPE_FLOAT, &mAmplitude, " min=10.0 max=200.0 step=0.1");
	}
}

void BipartiteBoxPruningTest::Deselect()
{
	if(mBar)
	{
		TwDeleteBar(mBar);
		mBar = null;
	}
}

bool BipartiteBoxPruningTest::UpdateBoxes()
{
	for(udword i=0;i<mNbBoxes;i++)
	{
		mBoxTime[i] += mSpeed;

		Point Center,Extents;
		mBoxes[i].GetExtents(Extents);

		Center.x = cosf(mBoxTime[i]*2.17f)*mAmplitude + sinf(mBoxTime[i])*mAmplitude*0.5f;
		Center.y = cosf(mBoxTime[i]*1.38f)*mAmplitude + sinf(mBoxTime[i]*mAmplitude);
		Center.z = sinf(mBoxTime[i]*0.777f)*mAmplitude;

		mBoxes[i].SetCenterExtents(Center, Extents);
	}
	return true;
}

void BipartiteBoxPruningTest::PerformTest()
{
	UpdateBoxes();

	// We pretend that half the boxes belong to first group, and the other half to the second group.
	udword Nb0 = mNbBoxes/2;
	udword Nb1 = mNbBoxes - Nb0;

	mPairs.ResetPairs();
	mProfiler.Start();
	BipartiteBoxPruning(Nb0, mBoxPtrs, Nb1, mBoxPtrs+Nb0, mPairs, Axes(AXES_XZY));
	mProfiler.End();
	mProfiler.Accum();

//	printf("%d pairs colliding\r     ", mPairs.GetNbPairs());

	bool* Flags = (bool*)_alloca(sizeof(bool)*mNbBoxes);
	ZeroMemory(Flags, sizeof(bool)*mNbBoxes);
	const Pair* P = mPairs.GetPairs();
	for(udword i=0;i<mPairs.GetNbPairs();i++)
	{
		// A colliding pair is (i,j) where 0 <= i < Nb0 and 0 <= j < Nb1
		Flags[P[i].id0] = true;
		Flags[P[i].id1+Nb0] = true;
	}

	// Render boxes
	OBB CurrentBox;
	CurrentBox.mRot.Identity();
	for(udword i=0;i<mNbBoxes;i++)
	{
		if(Flags[i])	glColor3f(1.0f, 0.0f, 0.0f);
		else
		{
			if(i<Nb0)
				glColor3f(0.0f, 1.0f, 0.0f);
			else
				glColor3f(0.0f, 0.0f, 1.0f);
		}
		mBoxes[i].GetCenter(CurrentBox.mCenter);
		mBoxes[i].GetExtents(CurrentBox.mExtents);
		DrawOBB(CurrentBox);
	}

	char Buffer[4096];
	sprintf(Buffer, "BipartiteBoxPruning - %5.1f us (%d cycles) - %d pairs\n", mProfiler.mMsTime, mProfiler.mCycles, mPairs.GetNbPairs());
	GLFontRenderer::print(10.0f, 10.0f, 0.02f, Buffer);
}

void BipartiteBoxPruningTest::KeyboardCallback(unsigned char key, int x, int y)
{
}

void BipartiteBoxPruningTest::MouseCallback(int button, int state, int x, int y)
{
}

void BipartiteBoxPruningTest::MotionCallback(int x, int y)
{
}
