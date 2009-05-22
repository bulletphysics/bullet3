/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006, 2007 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------   C o n s t r a i n t   s o l v e r    d e m o   ----------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------


extern "C"
{

void BT_GPU_PREF(clearAccumulationOfLambdaDt(float* lambdaDtBox, int numConstraints, int numContPoints));
void BT_GPU_PREF(collisionWithWallBox3D(void* trans,void* vel,void* angVel,btCudaPartProps pProp,	btCudaBoxProps gProp,int numObjs,float dt));
void BT_GPU_PREF(collisionBatchResolutionBox3D(void* constraints,int *batch,int numConstraints,void *trans,void *vel,
											void *angularVel,float *lambdaDtBox,float *positionConstraint,void* normal,void* contact,
											btCudaPartProps pProp,int iBatch,float dt));

void BT_GPU_PREF(integrVel(float* pForceTorqueDamp, float* pInvInertiaMass, void* pVel, void* pAngVel, float timeStep, unsigned int numBodies));
void BT_GPU_PREF(integrTrans(void* trans, void* vel, void* angVel, float timeStep, int numBodies));


}   // extern "C"

//----------------------------------------------------------------------------------------
