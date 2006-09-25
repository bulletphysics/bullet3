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

///Bullet Continuous Collision Detection and Physics Engine:





#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
//#include <bullet/bullet.h>
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/BroadphaseCollision/btAxisSweep3.h"


#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"

#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"



#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCappedCylinder dsDrawCappedCylinderD
#endif


// some constants

#define NUM 100			// max number of objects
#define DENSITY (5.0)		// density of all objects
#define GPB 3			// maximum number of geometries per body
#define MAX_CONTACTS 4		// maximum number of contact points per body


// dynamics and collision objects

struct MyObject {
  dBodyID body;			// the body
  CollisionObject	collider; //the collisionShape
};

static int num=0;		// number of objects in simulation
static int nextobj=0;		// next object to recycle if num==NUM
static dWorldID world;

CollisionWorld*	collisionWorld = 0;

static MyObject obj[NUM];
static dJointGroupID contactgroup;
static int selected = -1;	// selected object
static int show_aabb = 0;	// show geom AABBs?
static int show_contacts = 0;	// show contact points?
static int random_pos = 1;	// drop objects from random position?
static int write_world = 0;



SimdTransform	GetTransformFromOde(const dReal* pos,const dReal* rot)
{
	SimdTransform trans;
	trans.setIdentity();

// rot is pointer to object's rotation matrix, 4*3 format!
	
	SimdMatrix3x3 orn(rot[0],rot[1],rot[2],
		rot[4],rot[5],rot[6],
		rot[8],rot[9],rot[10]);

	trans.setOrigin(SimdVector3(pos[0],pos[1],pos[2]));
	trans.setBasis(orn);

  return trans;
}

void	GetOdeFromTransform(const SimdTransform& trans,dReal* pos,dReal* rot)
{
	pos[0] = trans.getOrigin().x();
	pos[1] = trans.getOrigin().y();
	pos[2] = trans.getOrigin().z();

	rot[0] = trans.getBasis()[0][0];
	rot[1] = trans.getBasis()[0][1];
	rot[2] = trans.getBasis()[0][2];

	rot[4] = trans.getBasis()[1][0];
	rot[5] = trans.getBasis()[1][1];
	rot[6] = trans.getBasis()[1][2];

	rot[8] = trans.getBasis()[2][0];
	rot[9] = trans.getBasis()[2][1];
	rot[10] = trans.getBasis()[2][2];

}



// start simulation - set viewpoint

static void start()
{
  static float xyz[3] = {2.1640f,-1.3079f,1.7600f};
  static float hpr[3] = {125.5000f,-17.0000f,0.0000f};
  dsSetViewpoint (xyz,hpr);
  printf ("To drop another object, press:\n");
  printf ("   b for box.\n");
  printf ("   s for sphere.\n");
  printf ("   c for cylinder.\n");
  printf ("   x for a composite object.\n");
  printf ("To select an object, press space.\n");
  printf ("To disable the selected object, press d.\n");
  printf ("To enable the selected object, press e.\n");
  printf ("To toggle showing the geom AABBs, press a.\n");
  printf ("To toggle showing the contact points, press t.\n");
  printf ("To toggle dropping from random position/orientation, press r.\n");
  printf ("To save the current state to 'state.dif', press 1.\n");
}


char locase (char c)
{
  if (c >= 'A' && c <= 'Z') return c - ('a'-'A');
  else return c;
}


// called when a key pressed

static void command (int cmd)
{
  size_t i;
  int j,k;
  dReal sides[3];
  dMass m;

  cmd = locase (cmd);
  if (cmd == 'b' || cmd == 's' || cmd == 'c'
      /* || cmd == 'l' */) {
    if (num < NUM) {
      i = num;
      num++;
    }
    else {
      i = nextobj;
      nextobj++;
      if (nextobj >= num) nextobj = 0;

      // destroy the body and geoms for slot i
      dBodyDestroy (obj[i].body);
	  collisionWorld->RemoveCollisionObject(&obj[i].collider);
	  obj[i].collider.m_broadphaseHandle = (BroadphaseProxy*)(-1);


    //todo: destroy collider
    }

    obj[i].body = dBodyCreate (world);
    for (k=0; k<3; k++) sides[k] = dRandReal()*0.2+0.1;

    dMatrix3 R;
    if (random_pos) {
      dBodySetPosition (obj[i].body,
			dRandReal()*2-1,dRandReal()*2-1,dRandReal()+2);
      dRFromAxisAndAngle (R,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
			  dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
    }
    else {
      dReal maxheight = 0;
      for (k=0; k<num; k++) {
	const dReal *pos = dBodyGetPosition (obj[k].body);
	if (pos[2] > maxheight) maxheight = pos[2];
      }
      dBodySetPosition (obj[i].body, 0,0,maxheight+0.3);
      dRFromAxisAndAngle (R,0,0,1,dRandReal()*10.0-5.0);
    }
    dBodySetRotation (obj[i].body,R);
    dBodySetData (obj[i].body,(void*) i);

    if (cmd == 'b') {
      dMassSetBox (&m,DENSITY,sides[0],sides[1],sides[2]);
	  obj[i].collider.m_collisionShape = new BoxShape(SimdVector3(0.5*sides[0],0.5*sides[1],0.5*sides[2]));
	  obj[i].collider.m_worldTransform = GetTransformFromOde(dBodyGetPosition(obj[i].body),dBodyGetRotation(obj[i].body));
	  collisionWorld->AddCollisionObject(&obj[i].collider);
	  obj[i].collider.m_userPointer = obj[i].body;

    }
    else if (cmd == 'c') {
      sides[0] *= 0.2;
	  sides[1] *= 0.2;
	  sides[2] *= 0.2;
      dMassSetCappedCylinder (&m,DENSITY,3,sides[0],sides[1]);
		obj[i].collider.m_collisionShape = new CylinderShapeZ(SimdVector3(sides[0],sides[1],sides[1]));
	  obj[i].collider.m_worldTransform = GetTransformFromOde(dBodyGetPosition(obj[i].body),dBodyGetRotation(obj[i].body));
	  collisionWorld->AddCollisionObject(&obj[i].collider);
	  obj[i].collider.m_userPointer = obj[i].body;
      //obj[i].geom[0] = dCreateCCylinder (space,sides[0],sides[1]);
    }
/*
    // cylinder option not yet implemented
    else if (cmd == 'l') {
      sides[1] *= 0.5;
      dMassSetCappedCylinder (&m,DENSITY,3,sides[0],sides[1]);
      obj[i].geom[0] = dCreateCylinder (space,sides[0],sides[1]);
    }
*/
    else if (cmd == 's') {
      sides[0] *= 0.5;
      dMassSetSphere (&m,DENSITY,sides[0]);
	  obj[i].collider.m_collisionShape = new SphereShape(sides[0]);
	  

	  obj[i].collider.m_worldTransform = GetTransformFromOde(dBodyGetPosition(obj[i].body),dBodyGetRotation(obj[i].body));
	  collisionWorld->AddCollisionObject(&obj[i].collider);
	  obj[i].collider.m_userPointer = obj[i].body;

      //obj[i].geom[0] = dCreateSphere (space,sides[0]);
    }
	else if (cmd == 'x') {

		
		
	}

	for (k=0; k < GPB; k++) {
		//if (obj[i].geom[k]) 
		//	  dGeomSetBody (obj[i].geom[k],obj[i].body);
	}

	dBodySetMass (obj[i].body,&m);
  }

  if (cmd == ' ') {
	  selected++;
	  if (selected >= num) selected = 0;
	  if (selected < 0) selected = 0;
  }
  else if (cmd == 'd' && selected >= 0 && selected < num) {
	  dBodyDisable (obj[selected].body);
  }
  else if (cmd == 'e' && selected >= 0 && selected < num) {
	  dBodyEnable (obj[selected].body);
  }
  else if (cmd == 'a') {
	  show_aabb ^= 1;
  }
  else if (cmd == 't') {
	  show_contacts ^= 1;
  }
  else if (cmd == 'r') {
	  random_pos ^= 1;
  }
  else if (cmd == '1') {
	  write_world = 1;
  }
}


// draw a geom

void drawGeom (CollisionObject& collider)//, const dReal *pos, const dReal *R, int show_aabb)
{
	dReal pos[4];
	dReal R[16];

	GetOdeFromTransform(collider.m_worldTransform,&pos[0],&R[0]);

  int i;
	
  if (!collider.m_collisionShape) return;
  
  int type = collider.m_collisionShape->GetShapeType();
  
  if (type == BOX_SHAPE_PROXYTYPE) {
    dVector3 sides;
    BoxShape* boxShape = static_cast<BoxShape*>(collider.m_collisionShape);
	sides[0] = 2.f*boxShape->GetHalfExtents().x();
	sides[1] = 2.f*boxShape->GetHalfExtents().y();
	sides[2] = 2.f*boxShape->GetHalfExtents().z();
	///boxshape already has margins 'inside'
    dsDrawBox (pos,R,sides);

  }
  else if (type == SPHERE_SHAPE_PROXYTYPE) {
    SphereShape* sphereShape = static_cast<SphereShape*>(collider.m_collisionShape);
	dReal radius = sphereShape->GetMargin();
	
    dsDrawSphere (pos,R,radius);

  }

  else if (type == CYLINDER_SHAPE_PROXYTYPE) {
    
	CylinderShapeZ* cylinder = static_cast<CylinderShapeZ*>(collider.m_collisionShape);
	dReal radius = cylinder->GetHalfExtents()[0];
	dReal length = 2.f*cylinder->GetHalfExtents()[1];
	radius += cylinder->GetMargin();
	length += 2.f*cylinder->GetMargin();

    //dGeomCCylinderGetParams (g,&radius,&length);
    dsDrawCylinder (pos,R,length,radius);
  }
/*
  // cylinder option not yet implemented
  else if (type == dCylinderClass) {
    dReal radius,length;
    dGeomCylinderGetParams (g,&radius,&length);
    dsDrawCylinder (pos,R,length,radius);
  }

  else if (type == dGeomTransformClass) {
    dGeomID g2 = dGeomTransformGetGeom (g);
    const dReal *pos2 = dGeomGetPosition (g2);
    const dReal *R2 = dGeomGetRotation (g2);
    dVector3 actual_pos;
    dMatrix3 actual_R;
    dMULTIPLY0_331 (actual_pos,R,pos2);
    actual_pos[0] += pos[0];
    actual_pos[1] += pos[1];
    actual_pos[2] += pos[2];
    dMULTIPLY0_333 (actual_R,R,R2);
    drawGeom (g2,actual_pos,actual_R,0);
  }

  if (show_aabb) {
    // draw the bounding box for this geom
    dReal aabb[6];
    dGeomGetAABB (g,aabb);
    dVector3 bbpos;
    for (i=0; i<3; i++) bbpos[i] = 0.5*(aabb[i*2] + aabb[i*2+1]);
    dVector3 bbsides;
    for (i=0; i<3; i++) bbsides[i] = aabb[i*2+1] - aabb[i*2];
    dMatrix3 RI;
    dRSetIdentity (RI);
    dsSetColorAlpha (1,0,0,0.5);
    dsDrawBox (bbpos,RI,bbsides);
  }
*/
}


// simulation loop

static void simLoop (int pause)
{
  dsSetColor (0,0,2);
  //dSpaceCollide (space,0,&nearCallback);
  collisionWorld->PerformDiscreteCollisionDetection();
  //now the collisionWorld contains all contact points... just copy them over to ODE and that's it

  for (int i=0;i<collisionWorld->GetDispatcher()->GetNumManifolds();i++)
  {
	  PersistentManifold* manifold = collisionWorld->GetDispatcher()->GetManifoldByIndexInternal(i);
	  CollisionObject* obj0 = static_cast<CollisionObject*>(manifold->GetBody0());
	  CollisionObject* obj1 = static_cast<CollisionObject*>(manifold->GetBody1());
	  
	  //RefreshContactPoints will update and/or remove existing contactpoints from previous frames
	  manifold->RefreshContactPoints(obj0->m_worldTransform,obj1->m_worldTransform);
      for (int j=0;j<manifold->GetNumContacts();j++)
	  {
		  ManifoldPoint& pt = manifold->GetContactPoint(j);
		  if (pt.GetDistance()<0.f)
		  {
			//report point to ODE

			dContact contact;
				contact.surface.mode = dContactBounce | dContactSoftCFM;
				contact.surface.mu = 10.f;//dInfinity;
				contact.surface.mu2 = 0;
				contact.surface.bounce = 0.1;
				contact.surface.bounce_vel = 0.1;
				contact.surface.soft_cfm = 0.01;
				contact.geom.depth = -pt.GetDistance();
				

				contact.geom.normal[0] = pt.m_normalWorldOnB.x();
				contact.geom.normal[1] = pt.m_normalWorldOnB.y();
				contact.geom.normal[2] = pt.m_normalWorldOnB.z();
				//contact.geom.g1 does it really need this?
				contact.geom.g1 = 0;
				contact.geom.g2 = 0;
				contact.geom.pos[0] = pt.GetPositionWorldOnB().x();
				contact.geom.pos[1] = pt.GetPositionWorldOnB().y();
				contact.geom.pos[2] = pt.GetPositionWorldOnB().z();

				contact.fdir1[0] = 0.f;
				contact.fdir1[1] = 0.f;
				contact.fdir1[2] = 0.f;
				
			    
				dJointID c = dJointCreateContact (world,contactgroup,&contact);
				dBodyID b1 = (dBodyID)obj0->m_userPointer;
				dBodyID b2 = (dBodyID)obj1->m_userPointer;
				dJointAttach (c,b1,b2);
				if (show_contacts) 
				{
					 dMatrix3 RI;
					dRSetIdentity (RI);
					const dReal ss[3] = {0.02,0.02,0.02};
					dsDrawBox (contact.geom.pos,RI,ss);
				}

		  }





	  }
  }


  
  if (!pause) dWorldQuickStep (world,0.05);

  if (write_world) {
    FILE *f = fopen ("state.dif","wt");
    if (f) {
      dWorldExportDIF (world,f,"X");
      fclose (f);
    }
    write_world = 0;
  }
  
  // remove all contact joints
  dJointGroupEmpty (contactgroup);

  dsSetColor (1,1,0);
  dsSetTexture (DS_WOOD);
  for (int i=0; i<num; i++) {
    for (int j=0; j < GPB; j++) {
      if (i==selected) {
	dsSetColor (0,0.7,1);
      }
      else if (! dBodyIsEnabled (obj[i].body)) {
	dsSetColor (1,0.8,0);
      }
      else {
	dsSetColor (1,1,0);
      }
      
	  //sync transform
		obj[i].collider.m_worldTransform = GetTransformFromOde(dBodyGetPosition(obj[i].body),dBodyGetRotation(obj[i].body));
		drawGeom (obj[i].collider);//,0,0,show_aabb);
    }
  }
}


int main (int argc, char **argv)
{
  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = &command;
  fn.stop = 0;
  fn.path_to_textures = "../../drawstuff/textures";

  // create world

  world = dWorldCreate();
  //space = dHashSpaceCreate (0);

  float bpsize = 1000.f;
  AxisSweep3 broadphase(SimdVector3(-bpsize,-bpsize,-bpsize),SimdVector3(bpsize,bpsize,bpsize));
  //SimpleBroadphase broadphase;

  CollisionDispatcher dispatcher;
  collisionWorld = new CollisionWorld(&dispatcher,&broadphase);

  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,0,0,-0.5);
  dWorldSetCFM (world,1e-5);
  dWorldSetERP(world,0.2f);
  dWorldSetAutoDisableFlag (world,1);
  dWorldSetContactMaxCorrectingVel (world,0.5);
  dWorldSetContactSurfaceLayer (world,0.001);
  //dCreatePlane (space,0,0,1,0);
  
  CollisionObject groundPlane;
  groundPlane.m_worldTransform.setIdentity();
  groundPlane.m_collisionShape = new BoxShape(SimdVector3(50,50,0.04));
   groundPlane.m_collisionShape->SetMargin(0.005f);
  collisionWorld->AddCollisionObject(&groundPlane);
  groundPlane.m_userPointer = 0;

  memset (obj,0,sizeof(obj));

  // run simulation
  dsSimulationLoop (argc,argv,352,288,&fn);

  dJointGroupDestroy (contactgroup);
  
  delete collisionWorld;
  dWorldDestroy (world);

  return 0;
}
