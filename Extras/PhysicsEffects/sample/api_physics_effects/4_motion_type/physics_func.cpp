/*
Physics Effects Copyright(C) 2010 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#include "physics_func.h"
#include "../common/perf_func.h"

///////////////////////////////////////////////////////////////////////////////
// Simulation Data

#define NUM_RIGIDBODIES 500
#define NUM_JOINTS    500
#define NUM_CONTACTS  4000

const float timeStep = 0.016f;
const float separateBias = 0.1f;
int iteration = 5;

//J ワールドサイズ
//E World size
PfxVector3 worldCenter(0.0f);
PfxVector3 worldExtent(500.0f);

//J 剛体
//E Rigid body
PfxRigidState states[NUM_RIGIDBODIES];
PfxRigidBody  bodies[NUM_RIGIDBODIES];
PfxCollidable collidables[NUM_RIGIDBODIES];
PfxSolverBody solverBodies[NUM_RIGIDBODIES];
int numRigidBodies = 0;

//J プロキシ
//E Proxies
PfxBroadphaseProxy proxies[NUM_RIGIDBODIES];

//J ジョイント
//E Joint
PfxConstraintPair jointPairs[NUM_JOINTS];
PfxJoint joints[NUM_JOINTS];
int numJoints = 0;

//J ペア
//E Pairs
unsigned int pairSwap;
unsigned int numPairs[2];
PfxBroadphasePair pairsBuff[2][NUM_CONTACTS];

//J コンタクト
//E Contacts
PfxContactManifold contacts[NUM_CONTACTS];
int numContacts;

PfxUInt32 contactIdPool[NUM_CONTACTS];
int numContactIdPool;

//J シミュレーションアイランド
//E Island generation
PfxIsland *island=NULL;
PfxUInt8 SCE_PFX_ALIGNED(16) islandBuff[32*NUM_RIGIDBODIES]; // Island buffer should be 32 * the number of rigid bodies.

//J スリープ制御
//E Sleep control
/*
	A sleeping object wakes up, when 
	* a new pair related to this rigid body is created
	* a pair releated to this rigid body is removed
	* a rigid body's velocity or position are updated
 */

//J スリープに入るカウント
//E Count to enter sleeping
const PfxUInt32 sleepCount = 180;

//J 速度が閾値以下ならばスリープカウントが増加
//E If velocity is under the following value, sleep count is increased.
const PfxFloat sleepVelocity = 0.1f;

//J 一時バッファ
//E Temporary buffers
#define POOL_BYTES (5*1024*1024)
unsigned char SCE_PFX_ALIGNED(128) poolBuff[POOL_BYTES];

//J 一時バッファ用スタックアロケータ
//E Stack allocator for temporary buffers
PfxHeapManager pool(poolBuff,POOL_BYTES);

// Keyframe
void updateKeyframe();
int keyframeId;

// Trigger
void updateTrigger();
int triggerId;

///////////////////////////////////////////////////////////////////////////////
// Simulation Function

int frame = 0;

void broadphase()
{
	pairSwap = 1-pairSwap;

	unsigned int &numPreviousPairs = numPairs[1-pairSwap];
	unsigned int &numCurrentPairs = numPairs[pairSwap];
	PfxBroadphasePair *previousPairs = pairsBuff[1-pairSwap];
	PfxBroadphasePair *currentPairs = pairsBuff[pairSwap];

	//J 剛体が最も分散している軸を見つける
	//E Find the axis along which all rigid bodies are most widely positioned
	int axis = 0;
	{
		PfxVector3 s(0.0f),s2(0.0f);
		for(int i=0;i<numRigidBodies;i++) {
			PfxVector3 c = states[i].getPosition();
			s += c;
			s2 += mulPerElem(c,c);
		}
		PfxVector3 v = s2 - mulPerElem(s,s) / (float)numRigidBodies;
		if(v[1] > v[0]) axis = 1;
		if(v[2] > v[axis]) axis = 2;
	}

	//J ブロードフェーズプロキシの更新
	//E Create broadpahse proxies
	{
		for(int i=0;i<numRigidBodies;i++) {
			pfxUpdateBroadphaseProxy(proxies[i],states[i],collidables[i],worldCenter,worldExtent,axis);
		}

		int workBytes = sizeof(PfxBroadphaseProxy) * numRigidBodies;
		void *workBuff = pool.allocate(workBytes);
				
		pfxParallelSort(proxies,numRigidBodies,workBuff,workBytes);

		pool.deallocate(workBuff);
	}

	//J 交差ペア探索
	//E Find overlapped pairs
	{
		PfxFindPairsParam findPairsParam;
		findPairsParam.pairBytes = pfxGetPairBytesOfFindPairs(NUM_CONTACTS);
		findPairsParam.pairBuff = pool.allocate(findPairsParam.pairBytes);
		findPairsParam.workBytes = pfxGetWorkBytesOfFindPairs(NUM_CONTACTS);
		findPairsParam.workBuff = pool.allocate(findPairsParam.workBytes);
		findPairsParam.proxies = proxies;
		findPairsParam.numProxies = numRigidBodies;
		findPairsParam.maxPairs = NUM_CONTACTS;
		findPairsParam.axis = axis;

		PfxFindPairsResult findPairsResult;

		int ret = pfxFindPairs(findPairsParam,findPairsResult);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxFindPairs failed %d\n",ret);
		
		pool.deallocate(findPairsParam.workBuff);

		//J 交差ペア合成
		//E Decompose overlapped pairs into 3 arrays
		PfxDecomposePairsParam decomposePairsParam;
		decomposePairsParam.pairBytes = pfxGetPairBytesOfDecomposePairs(numPreviousPairs,findPairsResult.numPairs);
		decomposePairsParam.pairBuff = pool.allocate(decomposePairsParam.pairBytes);
		decomposePairsParam.workBytes = pfxGetWorkBytesOfDecomposePairs(numPreviousPairs,findPairsResult.numPairs);
		decomposePairsParam.workBuff = pool.allocate(decomposePairsParam.workBytes);
		decomposePairsParam.previousPairs = previousPairs;
		decomposePairsParam.numPreviousPairs = numPreviousPairs;
		decomposePairsParam.currentPairs = findPairsResult.pairs; // Set pairs from pfxFindPairs()
		decomposePairsParam.numCurrentPairs = findPairsResult.numPairs; // Set the number of pairs from pfxFindPairs()

		PfxDecomposePairsResult decomposePairsResult;

		ret = pfxDecomposePairs(decomposePairsParam,decomposePairsResult);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxDecomposePairs failed %d\n",ret);

		pool.deallocate(decomposePairsParam.workBuff);

		PfxBroadphasePair *outNewPairs = decomposePairsResult.outNewPairs;
		PfxBroadphasePair *outKeepPairs = decomposePairsResult.outKeepPairs;
		PfxBroadphasePair *outRemovePairs = decomposePairsResult.outRemovePairs;
		PfxUInt32 numOutNewPairs = decomposePairsResult.numOutNewPairs;
		PfxUInt32 numOutKeepPairs = decomposePairsResult.numOutKeepPairs;
		PfxUInt32 numOutRemovePairs = decomposePairsResult.numOutRemovePairs;

		//J 廃棄ペアのコンタクトをプールに戻す
		//E Put removed contacts into the contact pool
		for(PfxUInt32 i=0;i<numOutRemovePairs;i++) {
			contactIdPool[numContactIdPool++] = pfxGetContactId(outRemovePairs[i]);

			//J 寝てる剛体を起こす
			//E Wake up sleeping rigid bodies
			PfxRigidState &stateA = states[pfxGetObjectIdA(outRemovePairs[i])];
			PfxRigidState &stateB = states[pfxGetObjectIdB(outRemovePairs[i])];
			if(stateA.isAsleep()) {
				stateA.wakeup();
			}
			if(stateB.isAsleep()) {
				stateB.wakeup();
			}
		}

		//J 新規ペアのコンタクトのリンクと初期化
		//E Add new contacts and initialize
		for(PfxUInt32 i=0;i<numOutNewPairs;i++) {
			int cId = 0;
			if(numContactIdPool > 0) {
				cId = contactIdPool[--numContactIdPool];
			}
			else {
				cId = numContacts++;
			}
			if(cId >= NUM_CONTACTS) {
				cId = 0;
			}
			SCE_PFX_ASSERT(cId < NUM_CONTACTS);
			pfxSetContactId(outNewPairs[i],cId);
			PfxContactManifold &contact = contacts[cId];
			contact.reset(pfxGetObjectIdA(outNewPairs[i]),pfxGetObjectIdB(outNewPairs[i]));

			//J 寝てる剛体を起こす
			//E Wake up sleeping rigid bodies
			PfxRigidState &stateA = states[pfxGetObjectIdA(outNewPairs[i])];
			PfxRigidState &stateB = states[pfxGetObjectIdB(outNewPairs[i])];
			if(stateA.isAsleep()) {
				stateA.wakeup();
			}
			if(stateB.isAsleep()) {
				stateB.wakeup();
			}
		}

		//J 新規ペアと維持ペアを合成
		//E Merge 'new' and 'keep' pairs
		numCurrentPairs = 0;
		for(PfxUInt32 i=0;i<numOutKeepPairs;i++) {
			currentPairs[numCurrentPairs++] = outKeepPairs[i];
		}
		for(PfxUInt32 i=0;i<numOutNewPairs;i++) {
			currentPairs[numCurrentPairs++] = outNewPairs[i];
		}
		
		pool.deallocate(decomposePairsParam.pairBuff);
		pool.deallocate(findPairsParam.pairBuff);
	}
	
	{
		int workBytes = sizeof(PfxBroadphasePair) * numCurrentPairs;
		void *workBuff = pool.allocate(workBytes);
		
		pfxParallelSort(currentPairs,numCurrentPairs,workBuff,workBytes);
		
		pool.deallocate(workBuff);
	}
}

void collision()
{
	unsigned int numCurrentPairs = numPairs[pairSwap];
	PfxBroadphasePair *currentPairs = pairsBuff[pairSwap];
	
	//J 衝突検出
	//E Detect collisions
	{
		PfxDetectCollisionParam param;
		param.contactPairs = currentPairs;
		param.numContactPairs = numCurrentPairs;
		param.offsetContactManifolds = contacts;
		param.offsetRigidStates = states;
		param.offsetCollidables = collidables;
		param.numRigidBodies = numRigidBodies;

		int ret = pfxDetectCollision(param);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxDetectCollision failed %d\n",ret);
	}

	//J リフレッシュ
	//E Refresh contacts
	{
		PfxRefreshContactsParam param;
		param.contactPairs = currentPairs;
		param.numContactPairs = numCurrentPairs;
		param.offsetContactManifolds = contacts;
		param.offsetRigidStates = states;
		param.numRigidBodies = numRigidBodies;

		int ret = pfxRefreshContacts(param);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxRefreshContacts failed %d\n",ret);
	}

	//J アイランド生成
	//E Create simulation islands
	{
		PfxGenerateIslandParam param;
		param.islandBuff = islandBuff;
		param.islandBytes = 32*NUM_RIGIDBODIES;
		param.pairs = currentPairs;
		param.numPairs = numCurrentPairs;
		param.numObjects = numRigidBodies;

		PfxGenerateIslandResult result;

		int ret = pfxGenerateIsland(param,result);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxGenerateIsland failed %d\n",ret);
		island = result.island;

		//J ジョイント分のペアを追加
		//E Add joint pairs to islands
		ret = pfxAppendPairs(island,jointPairs,numJoints);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxAppendPairs failed %d\n",ret);
	}
}

void constraintSolver()
{
	PfxPerfCounter pc;

	unsigned int numCurrentPairs = numPairs[pairSwap];
	PfxBroadphasePair *currentPairs = pairsBuff[pairSwap];

	pc.countBegin("setup solver bodies");
	{
		PfxSetupSolverBodiesParam param;
		param.states = states;
		param.bodies = bodies;
		param.solverBodies = solverBodies;
		param.numRigidBodies = numRigidBodies;
		
		int ret = pfxSetupSolverBodies(param);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxSetupSolverBodies failed %d\n",ret);
	}
	pc.countEnd();

	pc.countBegin("setup contact constraints");
	{
		PfxSetupContactConstraintsParam param;
		param.contactPairs = currentPairs;
		param.numContactPairs = numCurrentPairs;
		param.offsetContactManifolds = contacts;
		param.offsetRigidStates = states;
		param.offsetRigidBodies = bodies;
		param.offsetSolverBodies = solverBodies;
		param.numRigidBodies = numRigidBodies;
		param.timeStep = timeStep;
		param.separateBias = separateBias;
		
		int ret = pfxSetupContactConstraints(param);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxSetupJointConstraints failed %d\n",ret);
	}
	pc.countEnd();

	pc.countBegin("setup joint constraints");
	{
		PfxSetupJointConstraintsParam param;
		param.jointPairs = jointPairs;
		param.numJointPairs = numJoints;
		param.offsetJoints = joints;
		param.offsetRigidStates = states;
		param.offsetRigidBodies = bodies;
		param.offsetSolverBodies = solverBodies;
		param.numRigidBodies = numRigidBodies;
		param.timeStep = timeStep;

		for(int i=0;i<numJoints;i++) {
			pfxUpdateJointPairs(jointPairs[i],i,joints[i],states[joints[i].m_rigidBodyIdA],states[joints[i].m_rigidBodyIdB]);
		}

		int ret = pfxSetupJointConstraints(param);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxSetupJointConstraints failed %d\n",ret);
	}
	pc.countEnd();

	pc.countBegin("solve constraints");
	{
		PfxSolveConstraintsParam param;
		param.workBytes = pfxGetWorkBytesOfSolveConstraints(numRigidBodies,numCurrentPairs,numJoints);
		param.workBuff = pool.allocate(param.workBytes);
		param.contactPairs = currentPairs;
		param.numContactPairs = numCurrentPairs;
		param.offsetContactManifolds = contacts;
		param.jointPairs = jointPairs;
		param.numJointPairs = numJoints;
		param.offsetJoints = joints;
		param.offsetRigidStates = states;
		param.offsetSolverBodies = solverBodies;
		param.numRigidBodies = numRigidBodies;
		param.iteration = iteration;

		int ret = pfxSolveConstraints(param);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxSolveConstraints failed %d\n",ret);
		
		pool.deallocate(param.workBuff);
	}
	pc.countEnd();

	//pc.printCount();
}

void sleepOrWakeup()
{
	PfxFloat sleepVelSqr = sleepVelocity * sleepVelocity;

	for(PfxUInt32 i=0;i<(PfxUInt32)numRigidBodies;i++) {
		PfxRigidState &state = states[i];
		if(SCE_PFX_MOTION_MASK_CAN_SLEEP(state.getMotionType())) {
			PfxFloat linVelSqr = lengthSqr(state.getLinearVelocity());
			PfxFloat angVelSqr = lengthSqr(state.getAngularVelocity());

			if(state.isAwake()) {
				if( linVelSqr < sleepVelSqr && angVelSqr < sleepVelSqr ) {
					state.incrementSleepCount();
				}
				else {
					state.resetSleepCount();
				}
			}
		}
	}

	if(island) {
		for(PfxUInt32 i=0;i<pfxGetNumIslands(island);i++) {
			int numActive = 0;
			int numSleep = 0;
			int numCanSleep = 0;
			
			{
				PfxIslandUnit *islandUnit = pfxGetFirstUnitInIsland(island,(PfxUInt32)i);
				for(;islandUnit!=NULL;islandUnit=pfxGetNextUnitInIsland(islandUnit)) {
					if(!(SCE_PFX_MOTION_MASK_CAN_SLEEP(states[pfxGetUnitId(islandUnit)].getMotionType()))) continue;
					PfxRigidState &state = states[pfxGetUnitId(islandUnit)];
					if(state.isAsleep()) {
						numSleep++;
					}
					else {
						numActive++;
						if(state.getSleepCount() > sleepCount) {
							numCanSleep++;
						}
					}
				}
			}

			// Deactivate Island
			if(numCanSleep > 0 && numCanSleep == numActive + numSleep) {
				PfxIslandUnit *islandUnit = pfxGetFirstUnitInIsland(island,(PfxUInt32)i);
				for(;islandUnit!=NULL;islandUnit=pfxGetNextUnitInIsland(islandUnit)) {
					if(!(SCE_PFX_MOTION_MASK_CAN_SLEEP(states[pfxGetUnitId(islandUnit)].getMotionType()))) continue;
					states[pfxGetUnitId(islandUnit)].sleep();
				}
			}

			// Activate Island
			else if(numSleep > 0 && numActive > 0) {
				PfxIslandUnit *islandUnit = pfxGetFirstUnitInIsland(island,(PfxUInt32)i);
				for(;islandUnit!=NULL;islandUnit=pfxGetNextUnitInIsland(islandUnit)) {
					if(!(SCE_PFX_MOTION_MASK_CAN_SLEEP(states[pfxGetUnitId(islandUnit)].getMotionType()))) continue;
					states[pfxGetUnitId(islandUnit)].wakeup();
				}
			}
		}
	}
}

void integrate()
{
	PfxUpdateRigidStatesParam param;
	param.states = states;
	param.bodies = bodies;
	param.numRigidBodies = numRigidBodies;
	param.timeStep = timeStep;
	
	pfxUpdateRigidStates(param);
}

void physics_simulate()
{
	PfxPerfCounter pc;

	for(int i=1;i<numRigidBodies;i++) {
		pfxApplyExternalForce(states[i],bodies[i],bodies[i].getMass()*PfxVector3(0.0f,-9.8f,0.0f),PfxVector3(0.0f),timeStep);
	}
	
	perf_push_marker("broadphase");
	pc.countBegin("broadphase");
	broadphase();
	pc.countEnd();
	perf_pop_marker();
	
	perf_push_marker("collision");
	pc.countBegin("collision");
	collision();
	pc.countEnd();
	perf_pop_marker();
	
	perf_push_marker("solver");
	pc.countBegin("solver");
	constraintSolver();
	pc.countEnd();
	perf_pop_marker();
	
	perf_push_marker("sleepOrWakeup");
	pc.countBegin("sleepOrWakeup");
	sleepOrWakeup();
	pc.countEnd();
	perf_pop_marker();
	
	perf_push_marker("integrate");
	pc.countBegin("integrate");
	integrate();
	pc.countEnd();
	perf_pop_marker();
	
	frame++;
	
	if(frame%100 == 0) {
		float broadphaseTime = pc.getCountTime(0);
		float collisionTime  = pc.getCountTime(2);
		float solverTime     = pc.getCountTime(4);
		float integrateTime  = pc.getCountTime(6);
		SCE_PFX_PRINTF("frame %3d broadphase %.2f collision %.2f solver %.2f integrate %.2f | total %.2f\n",frame,
			broadphaseTime,collisionTime,solverTime,integrateTime,
			broadphaseTime+collisionTime+solverTime+integrateTime);
	}

	if(keyframeId>=0) updateKeyframe();
	if(triggerId>=0) updateTrigger();
}

///////////////////////////////////////////////////////////////////////////////
// Create Scene

int createBrick(int id,const PfxVector3 &pos,const PfxQuat &rot,const PfxVector3 &boxSize,PfxFloat mass)
{
	PfxBox box(boxSize);
	PfxShape shape;
	shape.reset();
	shape.setBox(box);
	collidables[id].reset();
	collidables[id].addShape(shape);
	collidables[id].finish();
	bodies[id].reset();
	bodies[id].setRestitution(0.0f);
	bodies[id].setMass(mass);
	bodies[id].setInertia(pfxCalcInertiaBox(boxSize,mass));
	states[id].reset();
	states[id].setPosition(pos);
	states[id].setOrientation(rot);
	states[id].setMotionType(kPfxMotionTypeActive);
	states[id].setUseSleep(1); // sleep mode ON
	states[id].setRigidBodyId(id);

	return id;
}

void createWall(const PfxVector3 &offsetPosition,int stackSize,const PfxVector3 &boxSize)
{
	PfxFloat bodyMass = 0.5f;

	PfxFloat diffX = boxSize[0] * 1.02f;
	PfxFloat diffY = boxSize[1] * 1.02f;
	PfxFloat diffZ = boxSize[2] * 1.02f;

	PfxFloat offset = -stackSize * (diffZ * 2.0f) * 0.5f;
	PfxVector3 pos(0.0f, diffY, 0.0f);

	while(stackSize) {
		for(int i=0;i<stackSize;i++) {
			pos[0] = offset + (PfxFloat)i * (diffX * 2.0f);
		
			createBrick(numRigidBodies++,offsetPosition+pos,PfxQuat::identity(),boxSize,bodyMass);
		}
		offset += diffX;
		pos[1] += (diffY * 2.0f);
		stackSize--;
	}
}

void createSceneKeyframe()
{
	// rotation bar
	int id = numRigidBodies++;
	PfxVector3 barSize(5.0f,0.25f,0.5f);
	PfxBox box(barSize);
	PfxShape shape;
	shape.reset();
	shape.setBox(box);
	collidables[id].reset();
	collidables[id].addShape(shape);
	collidables[id].finish();
	bodies[id].reset();
	bodies[id].setMass(5.0f);
	bodies[id].setInertia(pfxCalcInertiaBox(barSize,5.0f));
	states[id].reset();
	states[id].setPosition(PfxVector3(0.0f,barSize[1],0.0f));
	states[id].setMotionType(kPfxMotionTypeKeyframe);
	states[id].setRigidBodyId(id);
	keyframeId = id; // keep keyframe rigidbody
}

void updateKeyframe()
{
	PfxQuat ori = states[keyframeId].getOrientation();
	PfxQuat rot = PfxQuat::rotationY(0.02f);
	states[keyframeId].moveOrientation(rot*ori,timeStep);
}

void createSceneOneWay()
{
	PfxVector3 barSize(4.5f,0.25f,0.25f);
	int oneWayId = createBrick(numRigidBodies++,PfxVector3(-3.0f,7.0f,-5.0f),PfxQuat::identity(),barSize,15.0f);
	int activeId = createBrick(numRigidBodies++,PfxVector3( 3.0f,8.0f, 5.0f),PfxQuat::identity(),barSize,15.0f);
	states[oneWayId].setMotionType(kPfxMotionTypeOneWay);

	PfxBallJointInitParam jparam;
	jparam.anchorPoint = PfxVector3(-3.0f,7.0f,0.0f);

	int jointId1 = numJoints++;
	pfxInitializeBallJoint(joints[jointId1],states[0],states[oneWayId],jparam);
	pfxUpdateJointPairs(jointPairs[jointId1],jointId1,joints[jointId1],states[0],states[oneWayId]);

	jparam.anchorPoint = PfxVector3( 3.0f,8.0f,0.0f);

	int jointId2 = numJoints++;
	pfxInitializeBallJoint(joints[jointId2],states[0],states[activeId],jparam);
	pfxUpdateJointPairs(jointPairs[jointId2],jointId2,joints[jointId2],states[0],states[activeId]);

	createWall(PfxVector3(-3.0f,0.0f,0.0f),5,PfxVector3(0.5f));
	createWall(PfxVector3( 3.0f,0.0f,0.0f),5,PfxVector3(0.5f));
}

void createSceneTrigger()
{
	PfxVector3 wallSize(5.0f,5.0f,0.5f);
	
	triggerId = createBrick(numRigidBodies++,PfxVector3(0.0f,wallSize[1],0.0f),PfxQuat::identity(),wallSize,10.0f);
	states[triggerId].setMotionType(kPfxMotionTypeTrigger);

	PfxVector3 boxSize(0.5f);
	int oneWayId = createBrick(numRigidBodies++,PfxVector3(-3.0f,7.0f,-5.0f),PfxQuat::identity(),boxSize,15.0f);
	int activeId = createBrick(numRigidBodies++,PfxVector3( 3.0f,8.0f, 5.0f),PfxQuat::identity(),boxSize,15.0f);
	states[oneWayId].setMotionType(kPfxMotionTypeOneWay);

	PfxBallJointInitParam jparam;
	jparam.anchorPoint = PfxVector3(-3.0f,7.0f,0.0f);

	int jointId1 = numJoints++;
	pfxInitializeBallJoint(joints[jointId1],states[0],states[oneWayId],jparam);
	pfxUpdateJointPairs(jointPairs[jointId1],jointId1,joints[jointId1],states[0],states[oneWayId]);

	jparam.anchorPoint = PfxVector3( 3.0f,8.0f,0.0f);

	int jointId2 = numJoints++;
	pfxInitializeBallJoint(joints[jointId2],states[0],states[activeId],jparam);
	pfxUpdateJointPairs(jointPairs[jointId2],jointId2,joints[jointId2],states[0],states[activeId]);
}

void updateTrigger()
{
	for(PfxUInt32 i=0;i<numPairs[pairSwap];i++) {
		PfxBroadphasePair pair = pairsBuff[pairSwap][i];
		PfxContactManifold &contact = contacts[pfxGetContactId(pair)];
		
		if(pfxGetObjectIdA(pair) == triggerId) {
			if(contact.getNumContacts() > 0) {
				SCE_PFX_PRINTF("Find Contact %u\n",pfxGetObjectIdB(pair));
			}
		}
		if(pfxGetObjectIdB(pair) == triggerId) {
			if(contact.getNumContacts() > 0) {
				SCE_PFX_PRINTF("Find Contact %u\n",pfxGetObjectIdA(pair));
			}
		}
	}
}

void createSceneBoxGround()
{
	int id = numRigidBodies++;
	PfxBox box(150.0f,2.5f,150.0f);
	PfxShape shape;
	shape.reset();
	shape.setBox(box);
	collidables[id].reset();
	collidables[id].addShape(shape);
	collidables[id].finish();
	bodies[id].reset();
	states[id].reset();
	states[id].setPosition(PfxVector3(0.0f,-2.5f,0.0f));
	states[id].setMotionType(kPfxMotionTypeFixed);
	states[id].setRigidBodyId(id);
}

void physics_create_scene(int sceneId)
{
	const int numScenes = 3;
	int sid = sceneId % numScenes;
	
	numRigidBodies= 0;
	pairSwap = 0;
	numPairs[0] = 0;
	numPairs[1] = 0;
	numContacts = 0;
	numContactIdPool = 0;
	numJoints = 0;
	island = NULL;
	frame = 0;

	keyframeId = -1;
	triggerId = -1;
	
	switch(sid) {
		case 0:
		createSceneBoxGround();
		createSceneKeyframe();
		createWall(PfxVector3(0.0f,1.0f,0.0f),5,PfxVector3(0.5f));
		break;

		case 1:
		createSceneBoxGround();
		createSceneOneWay();
		break;

		case 2:
		createSceneBoxGround();
		createSceneTrigger();
		break;
	}

	SCE_PFX_PRINTF("----- Size of rigid body buffer ------\n");
	SCE_PFX_PRINTF("                    size *   num = total\n");
	SCE_PFX_PRINTF("PfxRigidState      %5d * %5d = %5d bytes\n",sizeof(PfxRigidState),numRigidBodies,sizeof(PfxRigidState)*numRigidBodies);
	SCE_PFX_PRINTF("PfxRigidBody       %5d * %5d = %5d bytes\n",sizeof(PfxRigidBody),numRigidBodies,sizeof(PfxRigidBody)*numRigidBodies);
	SCE_PFX_PRINTF("PfxCollidable      %5d * %5d = %5d bytes\n",sizeof(PfxCollidable),numRigidBodies,sizeof(PfxCollidable)*numRigidBodies);
	SCE_PFX_PRINTF("PfxJoint           %5d * %5d = %5d bytes\n",sizeof(PfxJoint),numJoints,sizeof(PfxJoint)*numJoints);
	SCE_PFX_PRINTF("PfxSolverBody      %5d * %5d = %5d bytes\n",sizeof(PfxSolverBody),numRigidBodies,sizeof(PfxSolverBody)*numRigidBodies);
	SCE_PFX_PRINTF("PfxBroadphaseProxy %5d * %5d = %5d bytes\n",sizeof(PfxBroadphaseProxy),numRigidBodies,sizeof(PfxBroadphaseProxy)*numRigidBodies);
	SCE_PFX_PRINTF("PfxContactManifold %5d * %5d = %5d bytes\n",sizeof(PfxContactManifold),NUM_CONTACTS,sizeof(PfxContactManifold)*NUM_CONTACTS);
	SCE_PFX_PRINTF("PfxBroadphasePair  %5d * %5d = %5d bytes\n",sizeof(PfxBroadphasePair),NUM_CONTACTS,sizeof(PfxBroadphasePair)*NUM_CONTACTS);

	int totalBytes = 
		(sizeof(PfxRigidState) + sizeof(PfxRigidBody) + sizeof(PfxCollidable) + sizeof(PfxSolverBody) + sizeof(PfxBroadphaseProxy)) * numRigidBodies +
		(sizeof(PfxContactManifold) + sizeof(PfxBroadphasePair)) * NUM_CONTACTS;
	SCE_PFX_PRINTF("----------------------------------------------------------\n");
	SCE_PFX_PRINTF("Total %5d bytes\n",totalBytes);
}

///////////////////////////////////////////////////////////////////////////////
// Initialize / Finalize Engine

bool physics_init()
{
	return true;
}

void physics_release()
{
}

///////////////////////////////////////////////////////////////////////////////
// Pick

PfxVector3 physics_pick_start(const PfxVector3 &p1,const PfxVector3 &p2)
{
	return PfxVector3(0.0f);
}

void physics_pick_update(const PfxVector3 &p)
{
}

void physics_pick_end()
{
}

///////////////////////////////////////////////////////////////////////////////
// Get Information

int physics_get_num_rigidbodies()
{
	return numRigidBodies;
}

const PfxRigidState& physics_get_state(int id)
{
	return states[id];
}

const PfxRigidBody& physics_get_body(int id)
{
	return bodies[id];
}

const PfxCollidable& physics_get_collidable(int id)
{
	return collidables[id];
}

int physics_get_num_contacts()
{
	return numPairs[pairSwap];
}

const PfxContactManifold &physics_get_contact(int id)
{
	return contacts[pfxGetConstraintId(pairsBuff[pairSwap][id])];
}

const PfxIsland* physics_get_islands()
{
	return island;
}
