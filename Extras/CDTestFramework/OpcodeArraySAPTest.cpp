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
#include "OpcodeArraySAPTest.h"
#include "RenderingHelpers.h"
#include "GLFontRenderer.h"

static udword gNbCreatedPairs;
static udword gNbDeletedPairs;
static udword gTotalNbPairs;

static void* CBData = (void*)0x12345678;
static void* PairUserData = (void*)0xDeadDead;

static void* CreatePairCB(const void* object0, const void* object1, void* user_data)
{
	assert(user_data==CBData);

	gNbCreatedPairs++;
	return PairUserData;
}

static void DeletePairCB(const void* object0, const void* object1, void* user_data, void* pair_user_data)
{
	assert(user_data==CBData);
	assert(pair_user_data==PairUserData);

	gNbDeletedPairs++;
}

OpcodeArraySAPTest::OpcodeArraySAPTest(int numBoxes) :
	mBar			(null),
	mNbBoxes		(numBoxes),
	mBoxes			(null),
	mHandles		(null),
	mBoxTime		(null),
	mAmplitude		(100.0f)
{
}

OpcodeArraySAPTest::~OpcodeArraySAPTest()
{
	Release();
}

void OpcodeArraySAPTest::Init()
{
	m_firstTime = true;

	SRand(0);
	mBoxes = new AABB[mNbBoxes];
	mBoxTime = new float[mNbBoxes];
	mHandles = new void*[mNbBoxes];
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

		mBoxTime[i] = 2000.0f*UnitRandomFloat();
	}

	UpdateBoxes(mNbBoxes);

	for(udword i=0;i<mNbBoxes;i++)
	{
		// It is mandatory to pass a valid pointer as a first parameter. This is supposed to be the game object
		// associated with the AABB. In this small example I just pass a pointer to the SAP itself.
		mHandles[i] = (void*)mASAP.AddObject(&mASAP, i, mBoxes[i]);
	}

	udword MyNbInitialPairs = mASAP.DumpPairs(CreatePairCB, DeletePairCB, CBData);
	gTotalNbPairs = MyNbInitialPairs;
}

void OpcodeArraySAPTest::Release()
{
	DELETEARRAY(mHandles);
	DELETEARRAY(mBoxTime);
	DELETEARRAY(mBoxes);
}

extern float	objectSpeed;

void OpcodeArraySAPTest::Select()
{
	// Create a tweak bar
	{
		mBar = TwNewBar("OpcodeArraySAPTest");
		TwAddVarRW(mBar, "Speed", TW_TYPE_FLOAT, &objectSpeed, " min=0.0 max=0.01 step=0.00001");
		TwAddVarRW(mBar, "Amplitude", TW_TYPE_FLOAT, &mAmplitude, " min=10.0 max=200.0 step=0.1");
	}
}

void OpcodeArraySAPTest::Deselect()
{
	if(mBar)
	{
		TwDeleteBar(mBar);
		mBar = null;
	}
}

bool OpcodeArraySAPTest::UpdateBoxes(int numBoxes)
{
	for(int i=0;i<numBoxes;i++)
	{
		mBoxTime[i] += objectSpeed;

		Point Center,Extents;
		mBoxes[i].GetExtents(Extents);

		Center.x = cosf(mBoxTime[i]*2.17f)*mAmplitude + sinf(mBoxTime[i])*mAmplitude*0.5f;
		Center.y = cosf(mBoxTime[i]*1.38f)*mAmplitude + sinf(mBoxTime[i]*mAmplitude);
		Center.z = sinf(mBoxTime[i]*0.777f)*mAmplitude;

		mBoxes[i].SetCenterExtents(Center, Extents);
	}
	return true;
}

extern int	percentUpdate;
extern bool	enableDraw;

void OpcodeArraySAPTest::PerformTest()
{
	int numBoxes = (mNbBoxes*percentUpdate)/100;
	if (m_firstTime)
	{
		numBoxes = mNbBoxes;
		m_firstTime = false;
	}

	mProfiler.Start();
	UpdateBoxes(numBoxes);

	gNbCreatedPairs = 0;
	gNbDeletedPairs = 0;

	for(int i=0;i<numBoxes;i++)
	{
		mASAP.UpdateObject((udword)mHandles[i], mBoxes[i]);
	}

	ASAP_Pair* Pairs;
	udword NbPairs = mASAP.DumpPairs(CreatePairCB, DeletePairCB, CBData, &Pairs);

	gTotalNbPairs += gNbCreatedPairs;
	gTotalNbPairs -= gNbDeletedPairs;
	

	mProfiler.End();
	mProfiler.Accum();

//	printf("%d pairs colliding\r     ", mPairs.GetNbPairs());

	bool* Flags = (bool*)_alloca(sizeof(bool)*mNbBoxes);
	ZeroMemory(Flags, sizeof(bool)*mNbBoxes);
	for(udword i=0;i<NbPairs;i++)
	{
		Flags[Pairs[i].id0] = true;
		Flags[Pairs[i].id1] = true;
	}

	// Render boxes
	if(enableDraw)
		{
		OBB CurrentBox;
		CurrentBox.mRot.Identity();
		for(udword i=0;i<mNbBoxes;i++)
		{
			if(Flags[i])	glColor3f(1.0f, 0.0f, 0.0f);
			else			glColor3f(0.0f, 1.0f, 0.0f);
			mBoxes[i].GetCenter(CurrentBox.mCenter);
			mBoxes[i].GetExtents(CurrentBox.mExtents);
			DrawOBB(CurrentBox);
		}
		}

	char Buffer[4096];
	sprintf(Buffer, "OpcodeArraySAPTest:  %5.1f us (%d cycles) : %d pairs\n", mProfiler.mMsTime, mProfiler.mCycles, NbPairs);
	GLFontRenderer::print(10.0f, 10.0f, 0.02f, Buffer);
}

void OpcodeArraySAPTest::KeyboardCallback(unsigned char key, int x, int y)
{
}

void OpcodeArraySAPTest::MouseCallback(int button, int state, int x, int y)
{
}

void OpcodeArraySAPTest::MotionCallback(int x, int y)
{
}
