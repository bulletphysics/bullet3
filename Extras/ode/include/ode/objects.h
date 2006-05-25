/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

#ifndef _ODE_OBJECTS_H_
#define _ODE_OBJECTS_H_

#include <ode/common.h>
#include <ode/mass.h>
#include <ode/contact.h>

#ifdef __cplusplus
extern "C" {
#endif

/* world */

dWorldID dWorldCreate(void);
void dWorldDestroy (dWorldID);

void dWorldSetGravity (dWorldID, dReal x, dReal y, dReal z);
void dWorldGetGravity (dWorldID, dVector3 gravity);
void dWorldSetERP (dWorldID, dReal erp);
dReal dWorldGetERP (dWorldID);
void dWorldSetCFM (dWorldID, dReal cfm);
dReal dWorldGetCFM (dWorldID);
void dWorldStep (dWorldID, dReal stepsize);
void dWorldImpulseToForce (dWorldID, dReal stepsize,
			   dReal ix, dReal iy, dReal iz, dVector3 force);

/* World QuickStep functions */

void dWorldQuickStep (dWorldID w, dReal stepsize);
void dWorldSetQuickStepNumIterations (dWorldID, int num);
int dWorldGetQuickStepNumIterations (dWorldID);
void dWorldSetQuickStepW (dWorldID, dReal param);
dReal dWorldGetQuickStepW (dWorldID);

/* World contact parameter functions */

void dWorldSetContactMaxCorrectingVel (dWorldID, dReal vel);
dReal dWorldGetContactMaxCorrectingVel (dWorldID);
void dWorldSetContactSurfaceLayer (dWorldID, dReal depth);
dReal dWorldGetContactSurfaceLayer (dWorldID);

/* StepFast1 functions */

void dWorldStepFast1(dWorldID, dReal stepsize, int maxiterations);
void dWorldSetAutoEnableDepthSF1(dWorldID, int autoEnableDepth);
int dWorldGetAutoEnableDepthSF1(dWorldID);

/* Auto-disable functions */

dReal dWorldGetAutoDisableLinearThreshold (dWorldID);
void  dWorldSetAutoDisableLinearThreshold (dWorldID, dReal linear_threshold);
dReal dWorldGetAutoDisableAngularThreshold (dWorldID);
void  dWorldSetAutoDisableAngularThreshold (dWorldID, dReal angular_threshold);
int   dWorldGetAutoDisableSteps (dWorldID);
void  dWorldSetAutoDisableSteps (dWorldID, int steps);
dReal dWorldGetAutoDisableTime (dWorldID);
void  dWorldSetAutoDisableTime (dWorldID, dReal time);
int   dWorldGetAutoDisableFlag (dWorldID);
void  dWorldSetAutoDisableFlag (dWorldID, int do_auto_disable);

dReal dBodyGetAutoDisableLinearThreshold (dBodyID);
void  dBodySetAutoDisableLinearThreshold (dBodyID, dReal linear_threshold);
dReal dBodyGetAutoDisableAngularThreshold (dBodyID);
void  dBodySetAutoDisableAngularThreshold (dBodyID, dReal angular_threshold);
int   dBodyGetAutoDisableSteps (dBodyID);
void  dBodySetAutoDisableSteps (dBodyID, int steps);
dReal dBodyGetAutoDisableTime (dBodyID);
void  dBodySetAutoDisableTime (dBodyID, dReal time);
int   dBodyGetAutoDisableFlag (dBodyID);
void  dBodySetAutoDisableFlag (dBodyID, int do_auto_disable);
void  dBodySetAutoDisableDefaults (dBodyID);

/* bodies */

dBodyID dBodyCreate (dWorldID);
void dBodyDestroy (dBodyID);

void  dBodySetData (dBodyID, void *data);
void *dBodyGetData (dBodyID);

void dBodySetPosition   (dBodyID, dReal x, dReal y, dReal z);
void dBodySetRotation   (dBodyID, const dMatrix3 R);
void dBodySetQuaternion (dBodyID, const dQuaternion q);
void dBodySetLinearVel  (dBodyID, dReal x, dReal y, dReal z);
void dBodySetAngularVel (dBodyID, dReal x, dReal y, dReal z);
const dReal * dBodyGetPosition   (dBodyID);
const dReal * dBodyGetRotation   (dBodyID);	/* ptr to 4x3 rot matrix */
const dReal * dBodyGetQuaternion (dBodyID);
const dReal * dBodyGetLinearVel  (dBodyID);
const dReal * dBodyGetAngularVel (dBodyID);

void dBodySetMass (dBodyID, const dMass *mass);
void dBodyGetMass (dBodyID, dMass *mass);

void dBodyAddForce            (dBodyID, dReal fx, dReal fy, dReal fz);
void dBodyAddTorque           (dBodyID, dReal fx, dReal fy, dReal fz);
void dBodyAddRelForce         (dBodyID, dReal fx, dReal fy, dReal fz);
void dBodyAddRelTorque        (dBodyID, dReal fx, dReal fy, dReal fz);
void dBodyAddForceAtPos       (dBodyID, dReal fx, dReal fy, dReal fz,
			                dReal px, dReal py, dReal pz);
void dBodyAddForceAtRelPos    (dBodyID, dReal fx, dReal fy, dReal fz,
			                dReal px, dReal py, dReal pz);
void dBodyAddRelForceAtPos    (dBodyID, dReal fx, dReal fy, dReal fz,
			                dReal px, dReal py, dReal pz);
void dBodyAddRelForceAtRelPos (dBodyID, dReal fx, dReal fy, dReal fz,
			                dReal px, dReal py, dReal pz);

const dReal * dBodyGetForce   (dBodyID);
const dReal * dBodyGetTorque  (dBodyID);
void dBodySetForce  (dBodyID b, dReal x, dReal y, dReal z);
void dBodySetTorque (dBodyID b, dReal x, dReal y, dReal z);

void dBodyGetRelPointPos    (dBodyID, dReal px, dReal py, dReal pz,
			     dVector3 result);
void dBodyGetRelPointVel    (dBodyID, dReal px, dReal py, dReal pz,
			     dVector3 result);
void dBodyGetPointVel       (dBodyID, dReal px, dReal py, dReal pz,
			     dVector3 result);
void dBodyGetPosRelPoint    (dBodyID, dReal px, dReal py, dReal pz,
			     dVector3 result);
void dBodyVectorToWorld     (dBodyID, dReal px, dReal py, dReal pz,
			     dVector3 result);
void dBodyVectorFromWorld   (dBodyID, dReal px, dReal py, dReal pz,
			     dVector3 result);

void dBodySetFiniteRotationMode (dBodyID, int mode);
void dBodySetFiniteRotationAxis (dBodyID, dReal x, dReal y, dReal z);

int dBodyGetFiniteRotationMode (dBodyID);
void dBodyGetFiniteRotationAxis (dBodyID, dVector3 result);

int dBodyGetNumJoints (dBodyID b);
dJointID dBodyGetJoint (dBodyID, int index);

void dBodyEnable (dBodyID);
void dBodyDisable (dBodyID);
int dBodyIsEnabled (dBodyID);

void dBodySetGravityMode (dBodyID b, int mode);
int dBodyGetGravityMode (dBodyID b);


/* joints */

dJointID dJointCreateBall (dWorldID, dJointGroupID);
dJointID dJointCreateHinge (dWorldID, dJointGroupID);
dJointID dJointCreateSlider (dWorldID, dJointGroupID);
dJointID dJointCreateContact (dWorldID, dJointGroupID, const dContact *);
dJointID dJointCreateHinge2 (dWorldID, dJointGroupID);
dJointID dJointCreateUniversal (dWorldID, dJointGroupID);
dJointID dJointCreateFixed (dWorldID, dJointGroupID);
dJointID dJointCreateNull (dWorldID, dJointGroupID);
dJointID dJointCreateAMotor (dWorldID, dJointGroupID);

void dJointDestroy (dJointID);

dJointGroupID dJointGroupCreate (int max_size);
void dJointGroupDestroy (dJointGroupID);
void dJointGroupEmpty (dJointGroupID);

void dJointAttach (dJointID, dBodyID body1, dBodyID body2);
void dJointSetData (dJointID, void *data);
void *dJointGetData (dJointID);
int dJointGetType (dJointID);
dBodyID dJointGetBody (dJointID, int index);

void dJointSetFeedback (dJointID, dJointFeedback *);
dJointFeedback *dJointGetFeedback (dJointID);

void dJointSetBallAnchor (dJointID, dReal x, dReal y, dReal z);
void dJointSetBallAnchor2 (dJointID, dReal x, dReal y, dReal z);
void dJointSetHingeAnchor (dJointID, dReal x, dReal y, dReal z);
void dJointSetHingeAnchorDelta (dJointID, dReal x, dReal y, dReal z, dReal ax, dReal ay, dReal az);
void dJointSetHingeAxis (dJointID, dReal x, dReal y, dReal z);
void dJointSetHingeParam (dJointID, int parameter, dReal value);
void dJointAddHingeTorque(dJointID joint, dReal torque);
void dJointSetSliderAxis (dJointID, dReal x, dReal y, dReal z);
void dJointSetSliderAxisDelta (dJointID, dReal x, dReal y, dReal z, dReal ax, dReal ay, dReal az);
void dJointSetSliderParam (dJointID, int parameter, dReal value);
void dJointAddSliderForce(dJointID joint, dReal force);
void dJointSetHinge2Anchor (dJointID, dReal x, dReal y, dReal z);
void dJointSetHinge2Axis1 (dJointID, dReal x, dReal y, dReal z);
void dJointSetHinge2Axis2 (dJointID, dReal x, dReal y, dReal z);
void dJointSetHinge2Param (dJointID, int parameter, dReal value);
void dJointAddHinge2Torques(dJointID joint, dReal torque1, dReal torque2);
void dJointSetUniversalAnchor (dJointID, dReal x, dReal y, dReal z);
void dJointSetUniversalAxis1 (dJointID, dReal x, dReal y, dReal z);
void dJointSetUniversalAxis2 (dJointID, dReal x, dReal y, dReal z);
void dJointSetUniversalParam (dJointID, int parameter, dReal value);
void dJointAddUniversalTorques(dJointID joint, dReal torque1, dReal torque2);
void dJointSetFixed (dJointID);
void dJointSetAMotorNumAxes (dJointID, int num);
void dJointSetAMotorAxis (dJointID, int anum, int rel,
			  dReal x, dReal y, dReal z);
void dJointSetAMotorAngle (dJointID, int anum, dReal angle);
void dJointSetAMotorParam (dJointID, int parameter, dReal value);
void dJointSetAMotorMode (dJointID, int mode);
void dJointAddAMotorTorques (dJointID, dReal torque1, dReal torque2, dReal torque3);

void dJointGetBallAnchor (dJointID, dVector3 result);
void dJointGetBallAnchor2 (dJointID, dVector3 result);
void dJointGetHingeAnchor (dJointID, dVector3 result);
void dJointGetHingeAnchor2 (dJointID, dVector3 result);
void dJointGetHingeAxis (dJointID, dVector3 result);
dReal dJointGetHingeParam (dJointID, int parameter);
dReal dJointGetHingeAngle (dJointID);
dReal dJointGetHingeAngleRate (dJointID);
dReal dJointGetSliderPosition (dJointID);
dReal dJointGetSliderPositionRate (dJointID);
void dJointGetSliderAxis (dJointID, dVector3 result);
dReal dJointGetSliderParam (dJointID, int parameter);
void dJointGetHinge2Anchor (dJointID, dVector3 result);
void dJointGetHinge2Anchor2 (dJointID, dVector3 result);
void dJointGetHinge2Axis1 (dJointID, dVector3 result);
void dJointGetHinge2Axis2 (dJointID, dVector3 result);
dReal dJointGetHinge2Param (dJointID, int parameter);
dReal dJointGetHinge2Angle1 (dJointID);
dReal dJointGetHinge2Angle1Rate (dJointID);
dReal dJointGetHinge2Angle2Rate (dJointID);
void dJointGetUniversalAnchor (dJointID, dVector3 result);
void dJointGetUniversalAnchor2 (dJointID, dVector3 result);
void dJointGetUniversalAxis1 (dJointID, dVector3 result);
void dJointGetUniversalAxis2 (dJointID, dVector3 result);
dReal dJointGetUniversalParam (dJointID, int parameter);
dReal dJointGetUniversalAngle1 (dJointID);
dReal dJointGetUniversalAngle2 (dJointID);
dReal dJointGetUniversalAngle1Rate (dJointID);
dReal dJointGetUniversalAngle2Rate (dJointID);
int dJointGetAMotorNumAxes (dJointID);
void dJointGetAMotorAxis (dJointID, int anum, dVector3 result);
int dJointGetAMotorAxisRel (dJointID, int anum);
dReal dJointGetAMotorAngle (dJointID, int anum);
dReal dJointGetAMotorAngleRate (dJointID, int anum);
dReal dJointGetAMotorParam (dJointID, int parameter);
int dJointGetAMotorMode (dJointID);

dJointID dConnectingJoint (dBodyID, dBodyID);
int dConnectingJointList (dBodyID, dBodyID, dJointID*);
int dAreConnected (dBodyID, dBodyID);
int dAreConnectedExcluding (dBodyID, dBodyID, int joint_type);


#ifdef __cplusplus
}
#endif

#endif
