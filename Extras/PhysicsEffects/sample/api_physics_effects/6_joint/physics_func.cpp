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
int iteration = 8;

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
PfxBroadphaseProxy proxies[6][NUM_RIGIDBODIES]; // shared by simulation and raycast

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
const PfxFloat sleepVelocity = 0.3f;

//J 一時バッファ
//E Temporary buffers
#define POOL_BYTES (5*1024*1024)
unsigned char SCE_PFX_ALIGNED(128) poolBuff[POOL_BYTES];

//J 一時バッファ用スタックアロケータ
//E Stack allocator for temporary buffers
PfxHeapManager pool(poolBuff,POOL_BYTES);

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
		//J レイキャストと共用するため、全ての軸に対するプロキシ配列を作成する
		//E To share with ray casting, create proxy arrays for all axis

		PfxUpdateBroadphaseProxiesParam param;
		param.workBytes = pfxGetWorkBytesOfUpdateBroadphaseProxies(numRigidBodies);
		param.workBuff = pool.allocate(param.workBytes,128);
		param.numRigidBodies = numRigidBodies;
		param.offsetRigidStates = states;
		param.offsetCollidables = collidables;
		param.proxiesX = proxies[0];
		param.proxiesY = proxies[1];
		param.proxiesZ = proxies[2];
		param.proxiesXb = proxies[3];
		param.proxiesYb = proxies[4];
		param.proxiesZb = proxies[5];
		param.worldCenter = worldCenter;
		param.worldExtent = worldExtent;

		PfxUpdateBroadphaseProxiesResult result;

		int ret = pfxUpdateBroadphaseProxies(param,result);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxUpdateBroadphaseProxies failed %d\n",ret);
		
		pool.deallocate(param.workBuff);
	}

	//J 交差ペア探索
	//E Find overlapped pairs
	{
		PfxFindPairsParam findPairsParam;
		findPairsParam.pairBytes = pfxGetPairBytesOfFindPairs(NUM_CONTACTS);
		findPairsParam.pairBuff = pool.allocate(findPairsParam.pairBytes);
		findPairsParam.workBytes = pfxGetWorkBytesOfFindPairs(NUM_CONTACTS);
		findPairsParam.workBuff = pool.allocate(findPairsParam.workBytes);
		findPairsParam.proxies = proxies[axis];
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
				SCE_PFX_PRINTF("wakeup %u\n",stateA.getRigidBodyId());
			}
			if(stateB.isAsleep()) {
				stateB.wakeup();
				SCE_PFX_PRINTF("wakeup %u\n",stateB.getRigidBodyId());
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
				SCE_PFX_PRINTF("wakeup %u\n",stateA.getRigidBodyId());
			}
			if(stateB.isAsleep()) {
				stateB.wakeup();
				SCE_PFX_PRINTF("wakeup %u\n",stateB.getRigidBodyId());
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
}

///////////////////////////////////////////////////////////////////////////////
// Create Scene

int createBrick(const PfxVector3 &pos,const PfxQuat &rot,const PfxVector3 &boxSize,PfxFloat mass)
{
	int id = numRigidBodies++;
	
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

void createSceneBasicJoints()
{
	PfxVector3 boxSize(0.25f,0.7f,1.0f);
	PfxFloat boxMass = 5.0f;
	PfxFloat diffX = boxSize[0]*20.0f;
	PfxFloat posX = -diffX * 2.5f;
	
	// Ball Joint
	{
		int idA = createBrick(PfxVector3(posX,3.0f,-1.0f),PfxQuat::identity(),boxSize,boxMass);
		int idB = createBrick(PfxVector3(posX,3.0f, 1.0f),PfxQuat::identity(),boxSize,boxMass);

		PfxRigidState &stateA = states[idA];
		PfxRigidState &stateB = states[idB];

		stateA.setMotionType(kPfxMotionTypeFixed);
		stateA.setContactFilterSelf(1);
		stateA.setContactFilterTarget(1);
		stateB.setContactFilterSelf(2);
		stateB.setContactFilterTarget(2);

		PfxBallJointInitParam jparam;
		jparam.anchorPoint = (stateA.getPosition() + stateB.getPosition())*0.5f;

		int jointId = numJoints++;

		pfxInitializeBallJoint(joints[jointId],stateA,stateB,jparam);
		pfxUpdateJointPairs(jointPairs[jointId],jointId,joints[jointId],stateA,stateB);
	}

	posX += diffX;

	// Swing-Twist Joint
	{
		int idA = createBrick(PfxVector3(posX,3.0f,-1.0f),PfxQuat::identity(),boxSize,boxMass);
		int idB = createBrick(PfxVector3(posX,3.0f, 1.0f),PfxQuat::identity(),boxSize,boxMass);

		PfxRigidState &stateA = states[idA];
		PfxRigidState &stateB = states[idB];

		stateA.setMotionType(kPfxMotionTypeFixed);
		stateA.setContactFilterSelf(1);
		stateA.setContactFilterTarget(1);
		stateB.setContactFilterSelf(2);
		stateB.setContactFilterTarget(2);

		PfxSwingTwistJointInitParam jparam;
		jparam.anchorPoint = (stateA.getPosition() + stateB.getPosition())*0.5f;
		jparam.twistAxis = PfxVector3(0.0f,0.0f,1.0f);

		int jointId = numJoints++;

		pfxInitializeSwingTwistJoint(joints[jointId],stateA,stateB,jparam);
		pfxUpdateJointPairs(jointPairs[jointId],jointId,joints[jointId],stateA,stateB);
	}

	posX += diffX;

	// Hinge Joint
	{
		int idA = createBrick(PfxVector3(posX,3.0f,-1.0f),PfxQuat::identity(),boxSize,boxMass);
		int idB = createBrick(PfxVector3(posX,3.0f, 1.0f),PfxQuat::identity(),boxSize,boxMass);

		PfxRigidState &stateA = states[idA];
		PfxRigidState &stateB = states[idB];

		stateA.setMotionType(kPfxMotionTypeFixed);
		stateA.setContactFilterSelf(1);
		stateA.setContactFilterTarget(1);
		stateB.setContactFilterSelf(2);
		stateB.setContactFilterTarget(2);

		PfxHingeJointInitParam jparam;
		jparam.anchorPoint = (stateA.getPosition() + stateB.getPosition())*0.5f;
		jparam.axis = PfxVector3(1.0f,0.0f,0.0f);
		jparam.lowerAngle = -0.5f;
		jparam.upperAngle =  0.5f;

		int jointId = numJoints++;

		pfxInitializeHingeJoint(joints[jointId],stateA,stateB,jparam);
		pfxUpdateJointPairs(jointPairs[jointId],jointId,joints[jointId],stateA,stateB);
	}

	posX += diffX;

	// Slider Joint
	{
		int idA = createBrick(PfxVector3(posX,3.0f,-1.0f),PfxQuat::identity(),boxSize,boxMass);
		int idB = createBrick(PfxVector3(posX,3.0f, 1.0f),PfxQuat::identity(),boxSize,boxMass);

		PfxRigidState &stateA = states[idA];
		PfxRigidState &stateB = states[idB];

		stateA.setMotionType(kPfxMotionTypeFixed);
		stateA.setContactFilterSelf(1);
		stateA.setContactFilterTarget(1);
		stateB.setContactFilterSelf(2);
		stateB.setContactFilterTarget(2);

		PfxSliderJointInitParam jparam;
		jparam.anchorPoint = (stateA.getPosition() + stateB.getPosition())*0.5f;
		jparam.direction = PfxVector3(0.0f,0.0f,1.0f);
		jparam.lowerDistance = 0.0f;
		jparam.upperDistance = 1.0f;

		int jointId = numJoints++;

		pfxInitializeSliderJoint(joints[jointId],stateA,stateB,jparam);
		pfxUpdateJointPairs(jointPairs[jointId],jointId,joints[jointId],stateA,stateB);
	}

	posX += diffX;

	// Fix Joint
	{
		int idA = createBrick(PfxVector3(posX,3.0f,-1.0f),PfxQuat::identity(),boxSize,boxMass);
		int idB = createBrick(PfxVector3(posX,3.0f, 1.0f),PfxQuat::identity(),boxSize,boxMass);

		PfxRigidState &stateA = states[idA];
		PfxRigidState &stateB = states[idB];

		stateA.setMotionType(kPfxMotionTypeFixed);
		stateA.setContactFilterSelf(1);
		stateA.setContactFilterTarget(1);
		stateB.setContactFilterSelf(2);
		stateB.setContactFilterTarget(2);

		PfxFixJointInitParam jparam;
		jparam.anchorPoint = (stateA.getPosition() + stateB.getPosition())*0.5f;

		int jointId = numJoints++;

		pfxInitializeFixJoint(joints[jointId],stateA,stateB,jparam);
		pfxUpdateJointPairs(jointPairs[jointId],jointId,joints[jointId],stateA,stateB);

		// Strengthen the joint
		joints[jointId].m_constraints[0].m_bias = 0.6f;
		joints[jointId].m_constraints[1].m_bias = 0.6f;
		joints[jointId].m_constraints[2].m_bias = 0.6f;
		joints[jointId].m_constraints[3].m_bias = 0.6f;
		joints[jointId].m_constraints[4].m_bias = 0.6f;
		joints[jointId].m_constraints[5].m_bias = 0.6f;
	}

	posX += diffX;

	// Universal Joint
	{
		int idA = createBrick(PfxVector3(posX,3.0f,-1.0f),PfxQuat::identity(),boxSize,boxMass);
		int idB = createBrick(PfxVector3(posX,3.0f, 1.0f),PfxQuat::identity(),boxSize,boxMass);

		PfxRigidState &stateA = states[idA];
		PfxRigidState &stateB = states[idB];

		stateA.setMotionType(kPfxMotionTypeFixed);
		stateA.setContactFilterSelf(1);
		stateA.setContactFilterTarget(1);
		stateB.setContactFilterSelf(2);
		stateB.setContactFilterTarget(2);

		PfxUniversalJointInitParam jparam;
		jparam.anchorPoint = (stateA.getPosition() + stateB.getPosition())*0.5f;
		jparam.axis = PfxVector3(0.0f,0.0f,1.0f);

		int jointId = numJoints++;

		pfxInitializeUniversalJoint(joints[jointId],stateA,stateB,jparam);
		pfxUpdateJointPairs(jointPairs[jointId],jointId,joints[jointId],stateA,stateB);
	}
}

void createSceneRagdoll(const PfxVector3 &offsetPosition,const PfxQuat &offsetOrientation)
{
	int head,torso,body,
		upperLegL,lowerLegL,upperArmL,lowerArmL,
		upperLegR,lowerLegR,upperArmR,lowerArmR;

	int ragdollJointId,numRagdollJoints = 0;
	
	PfxUInt32 contactFilterSelfA = 0x01;
	PfxUInt32 contactFilterSelfB = 0x02;
	PfxUInt32 contactFilterSelfC = 0x04;
	PfxUInt32 contactFilterSelfD = 0x08;
	PfxUInt32 contactFilterTargetA = 0x0d;
	PfxUInt32 contactFilterTargetB = 0x0a;
	PfxUInt32 contactFilterTargetC = 0x05;
	PfxUInt32 contactFilterTargetD = 0x0b;
	
	// Adjust inertia
	PfxFloat inertiaScale = 3.0f;

	// Head
	{
		int id = numRigidBodies++;
		PfxSphere sphere(0.3f);
		PfxShape shape;
		shape.reset();
		shape.setSphere(sphere);
		collidables[id].reset();
		collidables[id].addShape(shape);
		collidables[id].finish();
		bodies[id].reset();
		bodies[id].setMass(3.0f);
		bodies[id].setInertia(inertiaScale * pfxCalcInertiaSphere(0.3f,3.0f));
		states[id].reset();
		states[id].setPosition(offsetPosition + rotate(offsetOrientation,PfxVector3(0.0f,3.38433f,0.0f)));
		states[id].setOrientation(offsetOrientation);
		states[id].setMotionType(kPfxMotionTypeActive);
		states[id].setUseSleep(1);
		states[id].setRigidBodyId(id);
		states[id].setContactFilterSelf(contactFilterSelfB);
		states[id].setContactFilterTarget(contactFilterTargetB);
		head = id;
	}
	
	// Torso
	{
		int id = numRigidBodies++;
		PfxSphere sphere(0.35f);
		PfxShape shape;
		shape.reset();
		shape.setSphere(sphere);
		collidables[id].reset();
		collidables[id].addShape(shape);
		collidables[id].finish();
		bodies[id].reset();
		bodies[id].setMass(10.0f);
		bodies[id].setInertia(inertiaScale * pfxCalcInertiaSphere(0.35f,10.0f));
		states[id].reset();
		states[id].setPosition(offsetPosition + rotate(offsetOrientation,PfxVector3(0.0f,1.96820f,0.0f)));
		states[id].setOrientation(offsetOrientation);
		states[id].setMotionType(kPfxMotionTypeActive);
		states[id].setUseSleep(1);
		states[id].setRigidBodyId(id);
		states[id].setContactFilterSelf(contactFilterSelfB);
		states[id].setContactFilterTarget(contactFilterTargetB);
		torso = id;
	}

	// Body
	{
		int id = numRigidBodies++;
		PfxCapsule capsule(0.1f,0.38f);
		PfxShape shape;
		shape.reset();
		shape.setOffsetOrientation(PfxQuat(0.0f,0.0f,0.70711f,0.70711f));
		shape.setCapsule(capsule);
		collidables[id].reset();
		collidables[id].addShape(shape);
		collidables[id].finish();
		bodies[id].reset();
		bodies[id].setMass(8.0f);
		bodies[id].setInertia(inertiaScale * pfxCalcInertiaCylinderY(0.1f+0.38f,0.38f,8.0f));
		states[id].reset();
		states[id].setPosition(offsetPosition + rotate(offsetOrientation,PfxVector3(0.0f,2.66926f,0.0f)));
		states[id].setOrientation(offsetOrientation);
		states[id].setMotionType(kPfxMotionTypeActive);
		states[id].setUseSleep(1);
		states[id].setRigidBodyId(id);
		states[id].setContactFilterSelf(contactFilterSelfA);
		states[id].setContactFilterTarget(contactFilterTargetA);
		body = id;
	}

	// UpperLegL
	{
		int id = numRigidBodies++;
		PfxCapsule capsule(0.35f,0.15f);
		PfxShape shape;
		shape.reset();
		shape.setOffsetOrientation(PfxQuat(0.0f,0.0f,0.70711f,0.70711f));
		shape.setCapsule(capsule);
		collidables[id].reset();
		collidables[id].addShape(shape);
		collidables[id].finish();
		bodies[id].reset();
		bodies[id].setMass(8.0f);
		bodies[id].setInertia(inertiaScale * pfxCalcInertiaCylinderY(0.35f+0.15f,0.15f,8.0f));
		states[id].reset();
		states[id].setPosition(offsetPosition + rotate(offsetOrientation,PfxVector3(0.21f,1.34871f,0.0f)));
		states[id].setOrientation(offsetOrientation);
		states[id].setMotionType(kPfxMotionTypeActive);
		states[id].setUseSleep(1);
		states[id].setRigidBodyId(id);
		states[id].setContactFilterSelf(contactFilterSelfC);
		states[id].setContactFilterTarget(contactFilterTargetC);
		upperLegL = id;
	}

	// LowerLegL
	{
		int id = numRigidBodies++;
		PfxCapsule capsule(0.3f,0.15f);
		PfxShape shape;
		shape.reset();
		shape.setOffsetOrientation(PfxQuat(0.0f,0.0f,0.70711f,0.70711f));
		shape.setCapsule(capsule);
		collidables[id].reset();
		collidables[id].addShape(shape);
		collidables[id].finish();
		bodies[id].reset();
		bodies[id].setMass(4.0f);
		bodies[id].setInertia(inertiaScale * pfxCalcInertiaCylinderY(0.3f+0.15f,0.15f,4.0f));
		states[id].reset();
		states[id].setPosition(offsetPosition + rotate(offsetOrientation,PfxVector3(0.21f,0.59253f,0.0f)));
		states[id].setOrientation(offsetOrientation);
		states[id].setMotionType(kPfxMotionTypeActive);
		states[id].setUseSleep(1);
		states[id].setRigidBodyId(id);
		states[id].setContactFilterSelf(contactFilterSelfD);
		states[id].setContactFilterTarget(contactFilterTargetD);
		lowerLegL = id;
	}

	// UpperArmL
	{
		int id = numRigidBodies++;
		PfxCapsule capsule(0.25f,0.15f);
		PfxShape shape;
		shape.reset();
		shape.setOffsetOrientation(PfxQuat(0.0f,0.0f,0.70711f,0.70711f));
		shape.setCapsule(capsule);
		collidables[id].reset();
		collidables[id].addShape(shape);
		collidables[id].finish();
		bodies[id].reset();
		bodies[id].setMass(5.0f);
		bodies[id].setInertia(inertiaScale * pfxCalcInertiaCylinderY(0.25f+0.15f,0.15f,5.0f));
		states[id].reset();
		states[id].setPosition(offsetPosition + rotate(offsetOrientation,PfxVector3(0.72813f,2.87483f,0.0f)));
		states[id].setOrientation(offsetOrientation * PfxQuat(0.0f,0.0f,0.70711f,0.70711f));
		states[id].setMotionType(kPfxMotionTypeActive);
		states[id].setUseSleep(1);
		states[id].setRigidBodyId(id);
		states[id].setContactFilterSelf(contactFilterSelfB);
		states[id].setContactFilterTarget(contactFilterTargetB);
		upperArmL = id;
	}

	// LowerArmL
	{
		int id = numRigidBodies++;
		PfxCapsule capsule(0.225f,0.15f);
		PfxShape shape;
		shape.reset();
		shape.setOffsetOrientation(PfxQuat(0.0f,0.0f,0.70711f,0.70711f));
		shape.setCapsule(capsule);
		collidables[id].reset();
		collidables[id].addShape(shape);
		collidables[id].finish();
		bodies[id].reset();
		bodies[id].setMass(3.0f);
		bodies[id].setInertia(inertiaScale * pfxCalcInertiaCylinderY(0.225f+0.15f,0.15f,3.0f));
		states[id].reset();
		states[id].setPosition(offsetPosition + rotate(offsetOrientation,PfxVector3(1.42931f,2.87483f,0.0f)));
		states[id].setOrientation(offsetOrientation * PfxQuat(0.0f,0.0f,0.70711f,0.70711f));
		states[id].setMotionType(kPfxMotionTypeActive);
		states[id].setUseSleep(1);
		states[id].setRigidBodyId(id);
		states[id].setContactFilterSelf(contactFilterSelfC);
		states[id].setContactFilterTarget(contactFilterTargetC);
		lowerArmL = id;
	}

	// UpperLegR
	{
		int id = numRigidBodies++;
		PfxCapsule capsule(0.35f,0.15f);
		PfxShape shape;
		shape.reset();
		shape.setOffsetOrientation(PfxQuat(0.0f,0.0f,0.70711f,0.70711f));
		shape.setCapsule(capsule);
		collidables[id].reset();
		collidables[id].addShape(shape);
		collidables[id].finish();
		bodies[id].reset();
		bodies[id].setMass(8.0f);
		bodies[id].setInertia(inertiaScale * pfxCalcInertiaCylinderY(0.35f+0.15f,0.15f,8.0f));
		states[id].reset();
		states[id].setPosition(offsetPosition + rotate(offsetOrientation,PfxVector3(-0.21f,1.34871f,0.0f)));
		states[id].setOrientation(offsetOrientation);
		states[id].setMotionType(kPfxMotionTypeActive);
		states[id].setUseSleep(1);
		states[id].setRigidBodyId(id);
		states[id].setContactFilterSelf(contactFilterSelfC);
		states[id].setContactFilterTarget(contactFilterTargetC);
		upperLegR = id;
	}

	// LowerLegR
	{
		int id = numRigidBodies++;
		PfxCapsule capsule(0.3f,0.15f);
		PfxShape shape;
		shape.reset();
		shape.setOffsetOrientation(PfxQuat(0.0f,0.0f,0.70711f,0.70711f));
		shape.setCapsule(capsule);
		collidables[id].reset();
		collidables[id].addShape(shape);
		collidables[id].finish();
		bodies[id].reset();
		bodies[id].setMass(4.0f);
		bodies[id].setInertia(inertiaScale * pfxCalcInertiaCylinderY(0.3f+0.15f,0.15f,4.0f));
		states[id].reset();
		states[id].setPosition(offsetPosition + rotate(offsetOrientation,PfxVector3(-0.21f,0.59253f,0.0f)));
		states[id].setOrientation(offsetOrientation);
		states[id].setMotionType(kPfxMotionTypeActive);
		states[id].setUseSleep(1);
		states[id].setRigidBodyId(id);
		states[id].setContactFilterSelf(contactFilterSelfD);
		states[id].setContactFilterTarget(contactFilterTargetD);
		lowerLegR = id;
	}

	// UpperArmR
	{
		int id = numRigidBodies++;
		PfxCapsule capsule(0.25f,0.15f);
		PfxShape shape;
		shape.reset();
		shape.setOffsetOrientation(PfxQuat(0.0f,0.0f,0.70711f,0.70711f));
		shape.setCapsule(capsule);
		collidables[id].reset();
		collidables[id].addShape(shape);
		collidables[id].finish();
		bodies[id].reset();
		bodies[id].setMass(5.0f);
		bodies[id].setInertia(inertiaScale * pfxCalcInertiaCylinderY(0.25f+0.15f,0.15f,5.0f));
		states[id].reset();
		states[id].setPosition(offsetPosition + rotate(offsetOrientation,PfxVector3(-0.72813f,2.87483f,0.0f)));
		states[id].setOrientation(offsetOrientation * PfxQuat(0.0f,0.0f,-0.70711f,0.70711f));
		states[id].setMotionType(kPfxMotionTypeActive);
		states[id].setUseSleep(1);
		states[id].setRigidBodyId(id);
		states[id].setContactFilterSelf(contactFilterSelfB);
		states[id].setContactFilterTarget(contactFilterTargetB);
		upperArmR = id;
	}

	// LowerArmR
	{
		int id = numRigidBodies++;
		PfxCapsule capsule(0.225f,0.15f);
		PfxShape shape;
		shape.reset();
		shape.setOffsetOrientation(PfxQuat(0.0f,0.0f,0.70711f,0.70711f));
		shape.setCapsule(capsule);
		collidables[id].reset();
		collidables[id].addShape(shape);
		collidables[id].finish();
		bodies[id].reset();
		bodies[id].setMass(3.0f);
		bodies[id].setInertia(inertiaScale * pfxCalcInertiaCylinderY(0.225f+0.15f,0.15f,3.0f));
		states[id].reset();
		states[id].setPosition(offsetPosition + rotate(offsetOrientation,PfxVector3(-1.42931f,2.87483f,0.0f)));
		states[id].setOrientation(offsetOrientation * PfxQuat(0.0f,0.0f,-0.70711f,0.70711f));
		states[id].setMotionType(kPfxMotionTypeActive);
		states[id].setUseSleep(1);
		states[id].setRigidBodyId(id);
		states[id].setContactFilterSelf(contactFilterSelfC);
		states[id].setContactFilterTarget(contactFilterTargetC);
		lowerArmR = id;
	}

	// Joint Torso-Body
	{
		PfxRigidState &stateA = states[torso];
		PfxRigidState &stateB = states[body];

		PfxSwingTwistJointInitParam jparam;
		jparam.anchorPoint = offsetPosition + rotate(offsetOrientation,PfxVector3(0.0f,2.26425f,0.0f));
		jparam.twistAxis = rotate(offsetOrientation,PfxVector3(0.0f,1.0f,0.0f));

		int jointId = numJoints++;

		pfxInitializeSwingTwistJoint(joints[jointId],stateA,stateB,jparam);
		pfxUpdateJointPairs(jointPairs[jointId],jointId,joints[jointId],stateA,stateB);

		ragdollJointId = jointId;
	}
	
	// Joint Body-Head
	{
		PfxRigidState &stateA = states[body];
		PfxRigidState &stateB = states[head];

		PfxSwingTwistJointInitParam jparam;
		jparam.anchorPoint = offsetPosition + rotate(offsetOrientation,PfxVector3(0.0f,3.13575f,0.0f));
		jparam.twistAxis = rotate(offsetOrientation,PfxVector3(0.0f,1.0f,0.0f));

		int jointId = numJoints++;

		pfxInitializeSwingTwistJoint(joints[jointId],stateA,stateB,jparam);
		pfxUpdateJointPairs(jointPairs[jointId],jointId,joints[jointId],stateA,stateB);
	}

	// Joint Body-UpperArmL
	{
		PfxRigidState &stateA = states[body];
		PfxRigidState &stateB = states[upperArmL];

		PfxSwingTwistJointInitParam jparam;
		jparam.anchorPoint = offsetPosition + rotate(offsetOrientation,PfxVector3(0.40038f,2.87192f,0.0f));
		jparam.twistAxis = rotate(offsetOrientation,PfxVector3(1.0f,0.0f,0.0f));
		jparam.swingLowerAngle = 0.0f;
		jparam.swingUpperAngle = 1.57f;

		int jointId = numJoints++;

		pfxInitializeSwingTwistJoint(joints[jointId],stateA,stateB,jparam);
		pfxUpdateJointPairs(jointPairs[jointId],jointId,joints[jointId],stateA,stateB);
	}

	// Joint UpperArmL-LowerArmL
	{
		PfxRigidState &stateA = states[upperArmL];
		PfxRigidState &stateB = states[lowerArmL];

		PfxHingeJointInitParam jparam;
		jparam.anchorPoint = offsetPosition + rotate(offsetOrientation,PfxVector3(1.08651f,2.87483f,0.00000f));
		jparam.axis = rotate(offsetOrientation,PfxVector3(0.0f,-1.0f,0.0f));
		jparam.lowerAngle = 0.0f;
		jparam.upperAngle = 1.57f;

		int jointId = numJoints++;

		pfxInitializeHingeJoint(joints[jointId],stateA,stateB,jparam);
		pfxUpdateJointPairs(jointPairs[jointId],jointId,joints[jointId],stateA,stateB);
	}

	// Joint Body-UpperArmR
	{
		PfxRigidState &stateA = states[body];
		PfxRigidState &stateB = states[upperArmR];

		PfxSwingTwistJointInitParam jparam;
		jparam.anchorPoint = offsetPosition + rotate(offsetOrientation,PfxVector3(-0.40360f,2.87499f,0.00000f));
		jparam.twistAxis = rotate(offsetOrientation,PfxVector3(-1.0f,0.0f,0.0f));
		jparam.swingLowerAngle = 0.0f;
		jparam.swingUpperAngle = 1.57f;

		int jointId = numJoints++;

		pfxInitializeSwingTwistJoint(joints[jointId],stateA,stateB,jparam);
		pfxUpdateJointPairs(jointPairs[jointId],jointId,joints[jointId],stateA,stateB);
	}

	// Joint UpperArmR-LowerArmR
	{
		PfxRigidState &stateA = states[upperArmR];
		PfxRigidState &stateB = states[lowerArmR];

		PfxHingeJointInitParam jparam;
		jparam.anchorPoint = offsetPosition + rotate(offsetOrientation,PfxVector3(-1.09407f,2.87483f,0.00000f));
		jparam.axis = rotate(offsetOrientation,PfxVector3(0.0f,-1.0f,0.0f));
		jparam.lowerAngle = 0.0f;
		jparam.upperAngle = 1.57f;

		int jointId = numJoints++;

		pfxInitializeHingeJoint(joints[jointId],stateA,stateB,jparam);
		pfxUpdateJointPairs(jointPairs[jointId],jointId,joints[jointId],stateA,stateB);
	}

	// Joint Torso-UpperLegL
	{
		PfxRigidState &stateA = states[torso];
		PfxRigidState &stateB = states[upperLegL];

		PfxUniversalJointInitParam jparam;
		jparam.anchorPoint = offsetPosition + rotate(offsetOrientation,PfxVector3(0.20993f,1.69661f,0.00000f));
		jparam.axis = rotate(offsetOrientation,PfxVector3(0.0f,-1.0f,0.0f));
		jparam.swing1LowerAngle = 0.0f;
		jparam.swing1UpperAngle = 0.52f;
		jparam.swing2LowerAngle = 0.0f;
		jparam.swing2UpperAngle = 1.57f;

		int jointId = numJoints++;

		pfxInitializeUniversalJoint(joints[jointId],stateA,stateB,jparam);
		pfxUpdateJointPairs(jointPairs[jointId],jointId,joints[jointId],stateA,stateB);
	}

	// Joint Torso-UpperLegR
	{
		PfxRigidState &stateA = states[torso];
		PfxRigidState &stateB = states[upperLegR];

		PfxUniversalJointInitParam jparam;
		jparam.anchorPoint = offsetPosition + rotate(offsetOrientation,PfxVector3(-0.21311f,1.69661f,0.00000f));
		jparam.axis = rotate(offsetOrientation,PfxVector3(0.0f,-1.0f,0.0f));
		jparam.swing1LowerAngle = 0.0f;
		jparam.swing1UpperAngle = 0.52f;
		jparam.swing2LowerAngle = 0.0f;
		jparam.swing2UpperAngle = 1.57f;

		int jointId = numJoints++;

		pfxInitializeUniversalJoint(joints[jointId],stateA,stateB,jparam);
		pfxUpdateJointPairs(jointPairs[jointId],jointId,joints[jointId],stateA,stateB);
	}

	// Joint UpperLegL-LowerLegL
	{
		PfxRigidState &stateA = states[upperLegL];
		PfxRigidState &stateB = states[lowerLegL];

		PfxHingeJointInitParam jparam;
		jparam.anchorPoint = offsetPosition + rotate(offsetOrientation,PfxVector3(0.21000f,0.97062f,0.00000f));
		jparam.axis = rotate(offsetOrientation,PfxVector3(1.0f,0.0f,0.0f));
		jparam.lowerAngle = 0.0f;
		jparam.upperAngle = 1.57f;

		int jointId = numJoints++;

		pfxInitializeHingeJoint(joints[jointId],stateA,stateB,jparam);
		pfxUpdateJointPairs(jointPairs[jointId],jointId,joints[jointId],stateA,stateB);
	}

	// Joint UpperLegR-LowerLegR
	{
		PfxRigidState &stateA = states[upperLegR];
		PfxRigidState &stateB = states[lowerLegR];

		PfxHingeJointInitParam jparam;
		jparam.anchorPoint = offsetPosition + rotate(offsetOrientation,PfxVector3(-0.21000f,0.97062f,0.00000f));
		jparam.axis = rotate(offsetOrientation,PfxVector3(1.0f,0.0f,0.0f));
		jparam.lowerAngle = 0.0f;
		jparam.upperAngle = 1.57f;

		int jointId = numJoints++;

		pfxInitializeHingeJoint(joints[jointId],stateA,stateB,jparam);
		pfxUpdateJointPairs(jointPairs[jointId],jointId,joints[jointId],stateA,stateB);
	}

	numRagdollJoints = numJoints - ragdollJointId;

	// Add angular damping to ragdoll joints
	for(int i=0;i<numRagdollJoints;i++) {
		joints[ragdollJointId+i].m_constraints[0].m_warmStarting = 1;
		joints[ragdollJointId+i].m_constraints[1].m_warmStarting = 1;
		joints[ragdollJointId+i].m_constraints[2].m_warmStarting = 1;

		joints[ragdollJointId+i].m_constraints[3].m_damping = 0.05f;
		joints[ragdollJointId+i].m_constraints[4].m_damping = 0.05f;
		joints[ragdollJointId+i].m_constraints[5].m_damping = 0.05f;
	}
}

void createSceneRagdolls()
{
	float stairHeight = 0.5f;
	for(int i=0;i<5;i++) {
		int stair = createBrick(
			PfxVector3(0.0f,stairHeight * ((float)i + 0.5f), - i * 0.5f),
			PfxQuat::identity(),
			PfxVector3(10.0f,0.25f,2.5f - i * 0.5f),
			1.0f);
		states[stair].setMotionType(kPfxMotionTypeFixed);
	}
	createSceneRagdoll(PfxVector3(-6.0f,3.0f,0.0f),PfxQuat::rotationX(-0.3f));
	createSceneRagdoll(PfxVector3(-3.0f,3.5f,0.0f),PfxQuat::rotationX(-0.1f));
	createSceneRagdoll(PfxVector3( 0.0f,3.0f,0.0f),PfxQuat::rotationX( 0.2f));
	createSceneRagdoll(PfxVector3( 3.0f,3.5f,0.0f),PfxQuat::rotationX(-0.2f));
	createSceneRagdoll(PfxVector3( 6.0f,3.0f,0.0f),PfxQuat::rotationX( 0.4f));
}

PfxShape shapes[6];

void createSceneComplexMachine()
{
	// pulley x2
	{
		PfxTransform3 capsuleTrans = PfxTransform3::rotationY(SCE_PFX_PI*0.5f);
		PfxTransform3 cylinderTrans1(PfxQuat::rotationY(SCE_PFX_PI*0.5f),PfxVector3(0.0f,0.0f,-1.5f));
		PfxTransform3 cylinderTrans2(PfxQuat::rotationY(SCE_PFX_PI*0.5f),PfxVector3(0.0f,0.0f, 1.5f));

		PfxShape shapeCapsule,shapeCylinder1,shapeCylinder2;
		shapeCapsule.reset();
		shapeCapsule.setCapsule(PfxCapsule(2.0f,2.0f));
		shapeCapsule.setOffsetTransform(capsuleTrans);
		shapeCylinder1.reset();
		shapeCylinder1.setCylinder(PfxCylinder(0.8f,3.0f));
		shapeCylinder1.setOffsetTransform(cylinderTrans1);
		shapeCylinder2.reset();
		shapeCylinder2.setCylinder(PfxCylinder(0.8f,3.0f));
		shapeCylinder2.setOffsetTransform(cylinderTrans2);

		{
			int id = numRigidBodies++;
			PfxUInt16 shapeIds[3] = {0,1,2};
			collidables[id].reset(shapes,shapeIds,3);
			collidables[id].addShape(shapeCapsule);
			collidables[id].addShape(shapeCylinder1);
			collidables[id].addShape(shapeCylinder2);
			collidables[id].finish();
			bodies[id].reset();
			bodies[id].setMass(10.0f);
			bodies[id].setInertia(pfxCalcInertiaCylinderZ(2.0f,5.0f,10.0f));
			states[id].reset();
			states[id].setPosition(PfxVector3(-6.0f,5.0f,0.0f));
			states[id].setMotionType(kPfxMotionTypeActive);
			states[id].setUseSleep(1);
			states[id].setRigidBodyId(id);

			int jointId = numJoints++;
			PfxHingeJointInitParam jparam;
			jparam.anchorPoint = states[id].getPosition();
			jparam.axis = PfxVector3(0.0f,0.0f,1.0f);
			pfxInitializeHingeJoint(joints[jointId],states[0],states[id],jparam);
			pfxUpdateJointPairs(jointPairs[jointId],jointId,joints[jointId],states[0],states[id]);
		}

		{
			int id = numRigidBodies++;
			PfxUInt16 shapeIds[3] = {3,4,5};
			collidables[id].reset(shapes,shapeIds,3);
			collidables[id].addShape(shapeCapsule);
			collidables[id].addShape(shapeCylinder1);
			collidables[id].addShape(shapeCylinder2);
			collidables[id].finish();
			bodies[id].reset();
			bodies[id].setMass(10.0f);
			bodies[id].setInertia(pfxCalcInertiaCylinderZ(2.0f,5.0f,10.0f));
			states[id].reset();
			states[id].setPosition(PfxVector3(6.0f,5.0f,0.0f));
			states[id].setMotionType(kPfxMotionTypeActive);
			states[id].setUseSleep(1);
			states[id].setRigidBodyId(id);

			int jointId = numJoints++;
			PfxHingeJointInitParam jparam;
			jparam.anchorPoint = states[id].getPosition();
			jparam.axis = PfxVector3(0.0f,0.0f,1.0f);
			pfxInitializeHingeJoint(joints[jointId],states[0],states[id],jparam);
			pfxUpdateJointPairs(jointPairs[jointId],jointId,joints[jointId],states[0],states[id]);
		}
	}

	// rope
	const PfxFloat ropeLen = 0.4f;
	const PfxFloat ropeRad = 0.3f;
	const int numRopes = 32;
	const PfxFloat len = numRopes * ropeLen * 2.0f;

	int firstRope=0,lastRope=0;

	{
		PfxShape shapeRope;
		shapeRope.reset();
		shapeRope.setCapsule(PfxCapsule(ropeLen,ropeRad));

		PfxFloat dLen = ropeLen * 2.0f;
		PfxVector3 pos(-len*0.5f+ropeLen,8.0f,0.0f);

		for(int i=0;i<numRopes;i++) {
			int id = numRigidBodies++;
			collidables[id].reset();
			collidables[id].addShape(shapeRope);
			collidables[id].finish();
			bodies[id].reset();
			bodies[id].setMass(0.5f);
			bodies[id].setInertia(pfxCalcInertiaCylinderX(0.4f,0.2f,2.0f));
			states[id].reset();
			states[id].setPosition(pos);
			states[id].setMotionType(kPfxMotionTypeActive);
			states[id].setUseSleep(1);
			states[id].setRigidBodyId(id);
			states[id].setContactFilterSelf(0x00000001);
			states[id].setContactFilterTarget(0xfffffffe);

			pos += PfxVector3(dLen,0.0f,0.0f);

			if(i>0) {
				PfxRigidState &stateA = states[id-1];
				PfxRigidState &stateB = states[id];
				int jointId = numJoints++;
				PfxSwingTwistJointInitParam jparam;
				jparam.twistLowerAngle = -0.5f;
				jparam.twistUpperAngle = 0.5f;
				jparam.anchorPoint = (stateA.getPosition() + stateB.getPosition())*0.5f;
				jparam.twistAxis = PfxVector3(1.0f,0.0f,0.0f);
				pfxInitializeSwingTwistJoint(joints[jointId],stateA,stateB,jparam);
				pfxUpdateJointPairs(jointPairs[jointId],jointId,joints[jointId],stateA,stateB);

				joints[jointId].m_constraints[3].m_damping = 0.1f;
			}

			if(i==0) {
				firstRope = id;
			}
			else if(i==numRopes-1) {
				lastRope = id;
			}
		}
	}

	const PfxFloat sphereRad = 2.0f;

	{
		PfxShape shapeSphere;
		shapeSphere.reset();
		shapeSphere.setSphere(PfxSphere(sphereRad));

		{
			PfxVector3 pos = states[firstRope].getPosition() - PfxVector3(ropeLen+sphereRad,0.0f,0.0f);

			int id2 = numRigidBodies++;
			collidables[id2].reset();
			collidables[id2].addShape(shapeSphere);
			collidables[id2].finish();
			bodies[id2].reset();
			bodies[id2].setMass(10.0f);
			bodies[id2].setInertia(pfxCalcInertiaSphere(sphereRad,15.0f));
			states[id2].reset();
			states[id2].setPosition(pos);
			states[id2].setMotionType(kPfxMotionTypeActive);
			states[id2].setUseSleep(1);
			states[id2].setRigidBodyId(id2);
			states[id2].setContactFilterSelf(0x00000001);
			states[id2].setContactFilterTarget(0xfffffffe);

			int jointId = numJoints++;
			PfxBallJointInitParam jparam;
			jparam.anchorPoint = states[firstRope].getPosition() - PfxVector3(ropeLen,0.0f,0.0f);
			pfxInitializeBallJoint(joints[jointId],states[id2],states[firstRope],jparam);
			pfxUpdateJointPairs(jointPairs[jointId],jointId,joints[jointId],states[id2],states[firstRope]);
		}

		{
			PfxVector3 pos = states[lastRope].getPosition() + PfxVector3(ropeLen+sphereRad,0.0f,0.0f);

			int id2 = numRigidBodies++;
			collidables[id2].reset();
			collidables[id2].addShape(shapeSphere);
			collidables[id2].finish();
			bodies[id2].reset();
			bodies[id2].setMass(2.0f);
			bodies[id2].setInertia(pfxCalcInertiaSphere(sphereRad,5.0f));
			states[id2].reset();
			states[id2].setPosition(pos);
			states[id2].setMotionType(kPfxMotionTypeActive);
			states[id2].setUseSleep(1);
			states[id2].setRigidBodyId(id2);
			states[id2].setContactFilterSelf(0x00000001);
			states[id2].setContactFilterTarget(0xfffffffe);

			int jointId = numJoints++;
			PfxBallJointInitParam jparam;
			jparam.anchorPoint = states[lastRope].getPosition() + PfxVector3(ropeLen,0.0f,0.0f);
			pfxInitializeBallJoint(joints[jointId],states[id2],states[lastRope],jparam);
			pfxUpdateJointPairs(jointPairs[jointId],jointId,joints[jointId],states[id2],states[lastRope]);
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
	
	//E Keep a first joint as a pick joint
	//J 最初のジョイントをピッキング用にキープ
	numJoints = 1;
	joints[0].m_active = 0;
	pfxSetActive(jointPairs[0],false);

	switch(sid) {
		case 0: // basic joints
		createSceneBoxGround();
		createSceneBasicJoints();
		break;
		
		case 1: // ragdolls
		createSceneBoxGround();
		createSceneRagdolls();
		break;

		case 2: // complex machine
		createSceneBoxGround();
		createSceneComplexMachine();
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

inline PfxVector3 calcLocalCoord(const PfxRigidState &state,const PfxVector3 &coord)
{
	return rotate(conj(state.getOrientation()),(coord - state.getPosition()));
}

PfxVector3 physics_pick_start(const PfxVector3 &p1,const PfxVector3 &p2)
{
	PfxJoint &pickJoint = joints[0];
	
	PfxRayCastParam param;
	param.offsetRigidStates = states;
	param.offsetCollidables = collidables;
	param.proxiesX = proxies[0];
	param.proxiesY = proxies[1];
	param.proxiesZ = proxies[2];
	param.proxiesXb = proxies[3];
	param.proxiesYb = proxies[4];
	param.proxiesZb = proxies[5];
	param.numProxies = numRigidBodies;
	param.rangeCenter = worldCenter;
	param.rangeExtent = worldExtent;
	
	PfxRayInput pickRay;
	PfxRayOutput pickOut;
	
	pickRay.m_contactFilterSelf = pickRay.m_contactFilterTarget = 0xffffffff;
	pickRay.m_startPosition = p1;
	pickRay.m_direction = p2-p1;
	pickRay.m_facetMode = SCE_PFX_RAY_FACET_MODE_FRONT_ONLY;
	pfxCastSingleRay(pickRay,pickOut,param);
	
	if(pickOut.m_contactFlag) {
		PfxRigidState &stateA = states[0];
		PfxRigidState &stateB = states[pickOut.m_objectId];
		PfxRigidBody &bodyB = bodies[pickOut.m_objectId];

		/* Connect a picked rigid body and the ground with a ball joint.
		 * Weaken pick power by adjusting a m_maxImpulse parameter so that impulse doesn't exceeds m_maxImpulse.
		 * It is needed, because constraint power is too strong.
		 */
		
		PfxBallJointInitParam jparam;
		jparam.anchorPoint = pickOut.m_contactPoint;
		
		pfxInitializeBallJoint(pickJoint,stateA,stateB,jparam);
		pickJoint.m_constraints[0].m_maxImpulse = bodyB.getMass() * 2.0f;
		pickJoint.m_constraints[1].m_maxImpulse = bodyB.getMass() * 2.0f;
		pickJoint.m_constraints[2].m_maxImpulse = bodyB.getMass() * 2.0f;
		
		pfxUpdateJointPairs(jointPairs[0],0,pickJoint,stateA,stateB);
		
		SCE_PFX_PRINTF("pick objId %d ",pickOut.m_objectId);
		if(pickOut.m_subData.m_type == PfxSubData::MESH_INFO) {
			SCE_PFX_PRINTF("mesh islandId %d facetId %d",pickOut.m_subData.getIslandId(),pickOut.m_subData.getFacetId());
		}
		SCE_PFX_PRINTF("\n");
		
		return pickOut.m_contactPoint;
	}

	return PfxVector3(0.0f);
}

void physics_pick_update(const PfxVector3 &p)
{
	PfxJoint &pickJoint = joints[0];
	
	if(pickJoint.m_active>0) {
		pickJoint.m_anchorA = calcLocalCoord(states[pickJoint.m_rigidBodyIdA],p);
		if(states[pickJoint.m_rigidBodyIdB].isAsleep()) {
			states[pickJoint.m_rigidBodyIdB].wakeup();
		}
	}
}

void physics_pick_end()
{
	PfxJoint &pickJoint = joints[0];
	
	if(pickJoint.m_active>0) {
		pickJoint.m_active = 0;
	}
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
