/*
BulletSAPCompleteBoxPruningTest, Copyright (c) 2008 Erwin Coumans
Part of:
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

//This file was added by Erwin Coumans, to test Bullet SAP performance

#include "stdafx.h"
#include "BulletSAPCompleteBoxPruningTest.h"
#include "RenderingHelpers.h"
#include "GLFontRenderer.h"
#include "btBulletCollisionCommon.h"

int numParts =2;

BulletSAPCompleteBoxPruningTest::BulletSAPCompleteBoxPruningTest(int numBoxes,int method) :
	mBar			(null),
	mNbBoxes		(numBoxes),
	mBoxes			(null),
	mBoxPtrs		(null),
	mBoxTime		(null),
	mSpeed			(0.005f),
	mAmplitude		(100.0f)
{
	btVector3 aabbMin(-200,-200,-200);
	btVector3 aabbMax(200,200,200);

	int maxNumBoxes = numBoxes;

	switch (method)
	{
	case 1:
		m_broadphase = new btAxisSweep3(aabbMin,aabbMax,maxNumBoxes);
		methodname	=	"btAxisSweep3";
		break;
	case 2:
		m_broadphase = new btAxisSweep3(aabbMin,aabbMax,maxNumBoxes,new btNullPairCache());
		methodname	=	"btAxisSweep3+btNullPairCache";
		break;
	case 3:
		m_broadphase = new btAxisSweep3(aabbMin,aabbMax,maxNumBoxes,new btSortedOverlappingPairCache());
		methodname	=	"btAxisSweep3+btSortedOverlappingPairCache";
		break;
	case 4:
		m_broadphase = new btSimpleBroadphase(maxNumBoxes,new btSortedOverlappingPairCache());
		methodname	=	"btSimpleBroadphase+btSortedOverlappingPairCache";
		break;
	case 5:
		m_broadphase = new btSimpleBroadphase(maxNumBoxes,new btNullPairCache());
		methodname	=	"btSimpleBroadphase+btNullPairCache";
		break;

	case 6:
		{
		methodname	=	"btMultiSapBroadphase";
			btMultiSapBroadphase* multiSap = new btMultiSapBroadphase(maxNumBoxes);
			m_broadphase = multiSap;

			btVector3 tmpAabbMin,tmpAabbMax;
	
			float numP = (float) numParts;

			for (int i=0;i<numParts;i++)
			{
				tmpAabbMin[0] = aabbMin[0] + i*(aabbMax[0]-aabbMin[0])/numP;
				tmpAabbMax[0] = aabbMin[0] + (i+1)*(aabbMax[0]-aabbMin[0])/numP;

				for (int j=0;j<numParts;j++)
				{
					tmpAabbMin[1] = aabbMin[1] + j*(aabbMax[1]-aabbMin[1])/numP;
					tmpAabbMax[1] = aabbMin[1] + (j+1)*(aabbMax[1]-aabbMin[1])/numP;

					for (int k=0;k<numParts;k++)
					{
						tmpAabbMin[2] = aabbMin[2] + k*(aabbMax[2]-aabbMin[2])/numP;
						tmpAabbMax[2] = aabbMin[2] + (k+1)*(aabbMax[2]-aabbMin[2])/numP;

						btAxisSweep3* childBp = new btAxisSweep3(tmpAabbMin,tmpAabbMax,maxNumBoxes,multiSap->getOverlappingPairCache());
						multiSap->getBroadphaseArray().push_back(childBp);
					}
				}
			}
		
	//		btAxisSweep3* childBp = new btAxisSweep3(aabbMin,aabbMax,maxNumBoxes,multiSap->getOverlappingPairCache());
	//	multiSap->getBroadphaseArray().push_back(childBp);
			multiSap->buildTree(aabbMin,aabbMax);
	
		}
		break;

	default:
		{
			m_broadphase = new btAxisSweep3(aabbMin,aabbMax,numBoxes,new btNullPairCache());
			methodname	=	"btAxisSweep3+btNullPairCache";
		}
	}
}

BulletSAPCompleteBoxPruningTest::~BulletSAPCompleteBoxPruningTest()
{
	DELETEARRAY(mBoxTime);
	DELETEARRAY(mBoxPtrs);
	DELETEARRAY(mBoxes);
	delete m_broadphase;
}

void BulletSAPCompleteBoxPruningTest::Init()
{
	m_firstTime = true;
	SRand(0);


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
		btVector3	aabbMin(Center.x-Extents.x,Center.y-Extents.y,Center.z-Extents.z);
		btVector3	aabbMax(Center.x+Extents.x,Center.y+Extents.y,Center.z+Extents.z);
		int shapeType =0;
		void* userPtr = 0;
		btBroadphaseProxy* proxy = m_broadphase->createProxy(aabbMin,aabbMax,shapeType,(void*)i,1,1,0,0);//m_dispatcher);
		m_proxies.push_back( proxy );

		mBoxTime[i] = 2000.0f*UnitRandomFloat();
	}
}

void BulletSAPCompleteBoxPruningTest::Release()
{
	DELETEARRAY(mBoxTime);
	DELETEARRAY(mBoxes);
}

void BulletSAPCompleteBoxPruningTest::Select()
{
	// Create a tweak bar
	{
		mBar = TwNewBar("OPC_CompleteBoxPruning");
		TwAddVarRW(mBar, "Speed", TW_TYPE_FLOAT, &mSpeed, " min=0.0 max=0.01 step=0.00001");
		TwAddVarRW(mBar, "Amplitude", TW_TYPE_FLOAT, &mAmplitude, " min=10.0 max=200.0 step=0.1");
	}
	printf("SubMethod: %s\r\n",methodname);
}

void BulletSAPCompleteBoxPruningTest::Deselect()
{
	if(mBar)
	{
		TwDeleteBar(mBar);
		mBar = null;
	}
}

bool BulletSAPCompleteBoxPruningTest::UpdateBoxes(int numBoxes)
{
	static bool once=true;

	for(udword i=0;i<numBoxes;i++)
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
extern int doTree;

void BulletSAPCompleteBoxPruningTest::PerformTest()
{
	int numUpdatedBoxes = mNbBoxes/*/10*/;
	if (m_firstTime)
	{
		numUpdatedBoxes = mNbBoxes;
		m_firstTime = false;
	}

	mProfiler.Start();
	UpdateBoxes(numUpdatedBoxes);

	mPairs.ResetPairs();
	
	//CompleteBoxPruning(mNbBoxes, mBoxPtrs, mPairs, Axes(AXES_XZY));
	///add batch query?
	

	for (int i=0;i<numUpdatedBoxes;i++)
	{
		Point Center;
		Point Extents;
		mBoxPtrs[i]->GetCenter(Center);
		mBoxPtrs[i]->GetExtents(Extents);
		btVector3	aabbMin(Center.x-Extents.x,Center.y-Extents.y,Center.z-Extents.z);
		btVector3	aabbMax(Center.x+Extents.x,Center.y+Extents.y,Center.z+Extents.z);
		m_broadphase->setAabb(m_proxies[i],aabbMin,aabbMax,0);//m_dispatcher);
	}

	m_broadphase->calculateOverlappingPairs(0);

	

	mProfiler.End();
	mProfiler.Accum();

//	printf("%d pairs colliding\r     ", mPairs.GetNbPairs());

	bool* Flags = (bool*)_alloca(sizeof(bool)*mNbBoxes);
	ZeroMemory(Flags, sizeof(bool)*mNbBoxes);

	btOverlappingPairCache* pairCache = m_broadphase->getOverlappingPairCache();
	const btBroadphasePair* pairPtr = pairCache->getOverlappingPairArrayPtr();

	for(udword i=0;i<pairCache->getNumOverlappingPairs();i++)
	{
//		Flags[pairPtr[i].m_pProxy0->getUid()-1] = true;
//		Flags[pairPtr[i].m_pProxy1->getUid()-1] = true;
		Flags[int(pairPtr[i].m_pProxy0->m_clientObject)] = true;
		Flags[int(pairPtr[i].m_pProxy1->m_clientObject)] = true;
	}
	

	btVector3 aabbMin(-200,-200,-200);
	btVector3 aabbMax(200,200,200);

	btVector3 tmpAabbMin,tmpAabbMax;

	glDisable(GL_DEPTH_TEST);


			float numP = (float) numParts;
	
			for (int i=0;i<numParts;i++)
			{
				tmpAabbMin[0] = aabbMin[0] + i*(aabbMax[0]-aabbMin[0])/numP;
				tmpAabbMax[0] = aabbMin[0] + (i+1)*(aabbMax[0]-aabbMin[0])/numP;

				for (int j=0;j<numParts;j++)
				{
					tmpAabbMin[1] = aabbMin[1] + j*(aabbMax[1]-aabbMin[1])/numP;
					tmpAabbMax[1] = aabbMin[1] + (j+1)*(aabbMax[1]-aabbMin[1])/numP;

					for (int k=0;k<numParts;k++)
					{
						tmpAabbMin[2] = aabbMin[2] + k*(aabbMax[2]-aabbMin[2])/numP;
						tmpAabbMax[2] = aabbMin[2] + (k+1)*(aabbMax[2]-aabbMin[2])/numP;

			
				OBB CurrentBox;
				CurrentBox.mRot.Identity();
				
				{
					Point mmin(tmpAabbMin[0],tmpAabbMin[1],tmpAabbMin[2]);
					Point mmax(tmpAabbMax[0],tmpAabbMax[1],tmpAabbMax[2]);

					glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
					glEnable(GL_BLEND);
					glColor4f(i, j,k,0.2);//1.0f, 0.0f);
					CurrentBox.mCenter = (mmin+mmax)*0.5;
					CurrentBox.mExtents = (mmax-mmin)*0.5;
				
					DrawOBB(CurrentBox);

				}
			}
		}

	}

		

	glEnable(GL_DEPTH_TEST);

	glDisable(GL_BLEND);

	// Render boxes
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

	char Buffer[4096];
	sprintf(Buffer, "CompleteBoxPruning: %5.1f us (%d cycles) : %d pairs\n", mProfiler.mMsTime, mProfiler.mCycles, 
		m_broadphase->getOverlappingPairCache()->getNumOverlappingPairs());

//	m_broadphase)->printStats();

	GLFontRenderer::print(10.0f, 10.0f, 0.02f, Buffer);
}

void BulletSAPCompleteBoxPruningTest::KeyboardCallback(unsigned char key, int x, int y)
{
}

void BulletSAPCompleteBoxPruningTest::MouseCallback(int button, int state, int x, int y)
{
}

void BulletSAPCompleteBoxPruningTest::MotionCallback(int x, int y)
{
}
