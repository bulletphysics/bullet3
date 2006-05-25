/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
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

// TriMesh test by Erwin de Vries

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <CollisionShapes/BoxShape.h>
#include <CollisionShapes/CylinderShape.h>
#include <CollisionShapes/ConvexHullShape.h>
#include <CollisionShapes/ConeShape.h>
#include "BulletOdeCollide.h"


#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCappedCylinder dsDrawCappedCylinderD
#define dsDrawLine dsDrawLineD
#define dsDrawTriangle dsDrawTriangleD
#endif


// some constants

#define NUM 200			// max number of objects
#define DENSITY (5.0)		// density of all objects
#define GPB 3			// maximum number of geometries per body
#define MAX_CONTACTS 40		// maximum number of contact points per body


// dynamics and collision objects

struct MyObject {
  dBodyID body;			// the body
  dGeomID geom[GPB];		// geometries representing this body
};

static int num=0;		// number of objects in simulation
static int nextobj=0;		// next object to recycle if num==NUM
static dWorldID world;
static dSpaceID space;
static MyObject obj[NUM];
static dJointGroupID contactgroup;
static int selected = -1;	// selected object
static int show_aabb = 0;	// show geom AABBs?
static int show_contacts = 0;	// show contact points?
static int random_pos = 1;	// drop objects from random position?

#define VertexCount 5
#define IndexCount 12

static dVector3 Size;
static dVector3 Vertices[VertexCount];
static int Indices[IndexCount];

static dGeomID TriMesh=0;
static dGeomID Ray;

const int ConvexVertexCount = 4;
const int ConvexIndexCount = 4 * 3;


float ConvexVertices[ConvexVertexCount*3] = {
	REAL(-1./2.),REAL(-sqrt(3.)/6.), REAL(-sqrt(2./3.)/4.),
	REAL(1./2.),REAL(-sqrt(3.)/6.),REAL(-sqrt(2./3.)/4.),
	REAL(0.),REAL(2.*sqrt(3.)/6.),REAL(-sqrt(2./3.)/4.),
	REAL(0.),REAL(0.),REAL(3.*sqrt(2./3.)/4.)
	};

int ConvexIndices[ConvexIndexCount / 3][3] = {
	{2,1,0},
	{1,3,0},
	{2,3,1},
	{0,3,2}
};


// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  int i;
  // if (o1->body && o2->body) return;

  // exit without doing anything if the two bodies are connected by a joint
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;

  dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
  for (i=0; i<MAX_CONTACTS; i++) {
    contact[i].surface.mode = dContactBounce | dContactSoftCFM;
    contact[i].surface.mu = dInfinity;
    contact[i].surface.mu2 = 0;
    contact[i].surface.bounce = 0.1;
    contact[i].surface.bounce_vel = 0.1;
    contact[i].surface.soft_cfm = 0.01;
  }
  if (int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,
			   sizeof(dContact))) {
    dMatrix3 RI;
    dRSetIdentity (RI);
    const dReal ss[3] = {0.02,0.02,0.02};
    for (i=0; i<numc; i++) {
		if (dGeomGetClass(o1) == dRayClass || dGeomGetClass(o2) == dRayClass){
			dMatrix3 Rotation;
			dRSetIdentity(Rotation);
			dsDrawSphere(contact[i].geom.pos, Rotation, REAL(0.01));
			
			dVector3 End;
			End[0] = contact[i].geom.pos[0] + (contact[i].geom.normal[0] * contact[i].geom.depth);
			End[1] = contact[i].geom.pos[1] + (contact[i].geom.normal[1] * contact[i].geom.depth);
			End[2] = contact[i].geom.pos[2] + (contact[i].geom.normal[2] * contact[i].geom.depth);
			End[3] = contact[i].geom.pos[3] + (contact[i].geom.normal[3] * contact[i].geom.depth);
			
			dsDrawLine(contact[i].geom.pos, End);
			continue;
		}
		
      dJointID c = dJointCreateContact (world,contactgroup,contact+i);
      dJointAttach (c,b1,b2);
      if (show_contacts) dsDrawBox (contact[i].geom.pos,RI,ss);
    }
  }
}


// start simulation - set viewpoint

static void start()
{
  static float xyz[3] = {2.1640f,-1.3079f,1.7600f};
  static float hpr[3] = {125.5000f,-17.0000f,0.0000f};
  dsSetViewpoint (xyz,hpr);
  printf ("To drop another object, press:\n");
  printf ("   1 for bullet gjk cylinder.\n");
  printf ("   2 for bullet gjk box.\n");
  printf ("   3 for bullet gjk cone.\n");
  printf ("   4 for bullet gjk convex hull\n");
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
}


char locase (char c)
{
  if (c >= 'A' && c <= 'Z') return c - ('a'-'A');
  else return c;
}


// called when a key pressed

static void command (int cmd)
{
  int i,j,k;
  dReal sides[3];
  dMass m;

  cmd = locase (cmd);
  if (cmd == 'b' || cmd == 's' || cmd == 'c' || cmd == 'x'
      || cmd == '1' || cmd == '2' || cmd == '3' || cmd == '4' ) {

//	  /* || cmd == 'l' */) {
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
      for (k=0; k < GPB; k++) {
	if (obj[i].geom[k]) dGeomDestroy (obj[i].geom[k]);
      }
      memset (&obj[i],0,sizeof(obj[i]));
    }

    obj[i].body = dBodyCreate (world);
    for (k=0; k<3; k++) sides[k] = dRandReal()*0.5+0.1;

    dMatrix3 R;
    if (random_pos) {
      dBodySetPosition (obj[i].body,
			dRandReal()*2-1,dRandReal()*2-1,dRandReal()+1);
      dRFromAxisAndAngle (R,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
			  dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
    }
    else {
      dReal maxheight = 0;
      for (k=0; k<num; k++) {
	const dReal *pos = dBodyGetPosition (obj[k].body);
	if (pos[2] > maxheight) maxheight = pos[2];
      }
      dBodySetPosition (obj[i].body, 0,0,maxheight+1);
      dRFromAxisAndAngle (R,0,0,1,dRandReal()*10.0-5.0);
    }
    dBodySetRotation (obj[i].body,R);
    dBodySetData (obj[i].body,(void*) i);

    if (cmd == 'b') {
      dMassSetBox (&m,DENSITY,sides[0],sides[1],sides[2]);
      obj[i].geom[0] = dCreateBox (space,sides[0],sides[1],sides[2]);
    }
    else if (cmd == 'c') {
      sides[0] *= 0.5;
      dMassSetCappedCylinder (&m,DENSITY,3,sides[0],sides[1]);
      obj[i].geom[0] = dCreateCCylinder (space,sides[0],sides[1]);
    }
    else if (cmd == '1') {
      sides[1] *= 0.5;
      dMassSetCylinder (&m,DENSITY,3,sides[0],sides[1]);
//      obj[i].geom[0] = dCreateCylinder (space,sides[0],sides[1]);
	  SimdVector3 boxHalfExtents(0.5*sides[0],0.5*sides[0],0.5*sides[1]);
	  CollisionShape* boxShape = new CylinderShapeZ(boxHalfExtents);
		boxShape->SetMargin(0.004f);

      obj[i].geom[0] = dCreateConvex(space,boxShape);
 
	}
	 else if (cmd == '2') {
      dMassSetBox (&m,DENSITY,sides[0],sides[1],sides[2]);
	  
	  SimdVector3 boxHalfExtents(0.5f*sides[0],0.5f*sides[1],0.5f*sides[2]);
	  CollisionShape* boxShape = new BoxShape(boxHalfExtents);
		boxShape->SetMargin(0.004f);

      obj[i].geom[0] = dCreateConvex(space,boxShape);
    }

	 else if (cmd == '3') {
      sides[1] *= 0.5;
      dMassSetCylinder (&m,DENSITY,3,sides[0],sides[1]);
//      obj[i].geom[0] = dCreateCylinder (space,sides[0],sides[1]);
	  CollisionShape* boxShape = new ConeShape(0.5*sides[0],0.5*sides[2]);
		boxShape->SetMargin(0.004f);
      obj[i].geom[0] = dCreateConvex(space,boxShape);
 
	}
	 // cylinder option not yet implemented
    else if (cmd == '4') {
 dMassSetBox (&m,DENSITY,sides[0],sides[1],sides[2]);
		ConvexHullShape* hullShape = new ConvexHullShape(0,0);
		for (int k=0;k<ConvexVertexCount;k++)
		{
			hullShape->AddPoint(SimdPoint3(ConvexVertices[k*3],ConvexVertices[k*3+1],ConvexVertices[k*3+2]));
		}

		hullShape->SetMargin(0.01f);
		hullShape->setLocalScaling(SimdVector3(sides[0],sides[1],sides[2]));


      obj[i].geom[0] = dCreateConvex(space,hullShape);
 
	}


    else if (cmd == 's') {
      sides[0] *= 0.5;
      dMassSetSphere (&m,DENSITY,sides[0]);
      obj[i].geom[0] = dCreateSphere (space,sides[0]);
    }
    else if (cmd == 'x') {
      dGeomID g2[GPB];		// encapsulated geometries
      dReal dpos[GPB][3];	// delta-positions for encapsulated geometries

      // start accumulating masses for the encapsulated geometries
      dMass m2;
      dMassSetZero (&m);

      // set random delta positions
      for (j=0; j<GPB; j++) {
	for (k=0; k<3; k++) dpos[j][k] = dRandReal()*0.3-0.15;
      }

      for (k=0; k<GPB; k++) {
	obj[i].geom[k] = dCreateGeomTransform (space);
	dGeomTransformSetCleanup (obj[i].geom[k],1);
	if (k==0) {
	  dReal radius = dRandReal()*0.25+0.05;
	  g2[k] = dCreateSphere (0,radius);
	  dMassSetSphere (&m2,DENSITY,radius);
	}
	else if (k==1) {
	  g2[k] = dCreateBox (0,sides[0],sides[1],sides[2]);
	  dMassSetBox (&m2,DENSITY,sides[0],sides[1],sides[2]);
	}
	else {
	  dReal radius = dRandReal()*0.1+0.05;
	  dReal length = dRandReal()*1.0+0.1;
	  g2[k] = dCreateCCylinder (0,radius,length);
	  dMassSetCappedCylinder (&m2,DENSITY,3,radius,length);
	}
	dGeomTransformSetGeom (obj[i].geom[k],g2[k]);

	// set the transformation (adjust the mass too)
	dGeomSetPosition (g2[k],dpos[k][0],dpos[k][1],dpos[k][2]);
	dMassTranslate (&m2,dpos[k][0],dpos[k][1],dpos[k][2]);
	dMatrix3 Rtx;
	dRFromAxisAndAngle (Rtx,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
			    dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
	dGeomSetRotation (g2[k],Rtx);
	dMassRotate (&m2,Rtx);

	// add to the total mass
	dMassAdd (&m,&m2);
      }

      // move all encapsulated objects so that the center of mass is (0,0,0)
      for (k=0; k<2; k++) {
	dGeomSetPosition (g2[k],
			  dpos[k][0]-m.c[0],
			  dpos[k][1]-m.c[1],
			  dpos[k][2]-m.c[2]);
      }
      dMassTranslate (&m,-m.c[0],-m.c[1],-m.c[2]);
    }

    for (k=0; k < GPB; k++) {
      if (obj[i].geom[k]) dGeomSetBody (obj[i].geom[k],obj[i].body);
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
}


// draw a geom

void drawGeom (dGeomID g, const dReal *pos, const dReal *R, int show_aabb)
{
  if (!g) return;
  if (!pos) pos = dGeomGetPosition (g);
  if (!R) R = dGeomGetRotation (g);

  int type = dGeomGetClass (g);
  if (type == dBoxClass) {
    dVector3 sides;
    dGeomBoxGetLengths (g,sides);
    dsDrawBox (pos,R,sides);
  }
    else if (type == dConvexClass) {
    dVector3 sides;
	dGeomConvexGetLengths(g,sides); 
    
	CollisionShape* convexShape = GetCollisionShapeFromConvex(g);
	switch (convexShape->GetShapeType())
	{
	case BOX_SHAPE_PROXYTYPE:
		{
			dsDrawBox (pos,R,sides);
			break;
		}
	case CYLINDER_SHAPE_PROXYTYPE:
		{
			dsDrawCylinder (pos,R,sides[2],0.5*sides[0]);//length,radius);
			break;
		}
	case CONE_SHAPE_PROXYTYPE:
		{
			dsDrawCone(pos,R,sides[2],0.5f*sides[0]);
			break;
		}

		case CONVEX_HULL_SHAPE_PROXYTYPE:
		{
		//	dsDrawCylinder2(pos,R,0.5*sides[2],0.5*sides[0],0.5*sides[1]);

		
			{
				int* Indices = (int*)::ConvexIndices;

        SimdVector3 scaling = convexShape->getLocalScaling();

	
          for (int ii = 0; ii < IndexCount / 3; ii++) {
            const dReal v[9] = { // explicit conversion from float to dReal
              ConvexVertices[Indices[ii * 3 + 0] * 3 + 0]*scaling[0],
              ConvexVertices[Indices[ii * 3 + 0] * 3 + 1]*scaling[1],
              ConvexVertices[Indices[ii * 3 + 0] * 3 + 2]*scaling[2],
              ConvexVertices[Indices[ii * 3 + 1] * 3 + 0]*scaling[0],
              ConvexVertices[Indices[ii * 3 + 1] * 3 + 1]*scaling[1],
              ConvexVertices[Indices[ii * 3 + 1] * 3 + 2]*scaling[2],
              ConvexVertices[Indices[ii * 3 + 2] * 3 + 0]*scaling[0],
              ConvexVertices[Indices[ii * 3 + 2] * 3 + 1]*scaling[1],
              ConvexVertices[Indices[ii * 3 + 2] * 3 + 2]*scaling[2]
            };
            dsDrawTriangle(pos, R, &v[0], &v[3], &v[6], 1);
		  }
			};	 
	
		};
	default:
		{
		}

	}
	}


  else if (type == dSphereClass) {
    dsDrawSphere (pos,R,dGeomSphereGetRadius (g));
  }
  else if (type == dCCylinderClass) {
    dReal radius,length;
    dGeomCCylinderGetParams (g,&radius,&length);
    dsDrawCappedCylinder (pos,R,length,radius);
  }
/*
  // cylinder option not yet implemented
  else if (type == dCylinderClass) {
    dReal radius,length;
    dGeomCylinderGetParams (g,&radius,&length);
    dsDrawCylinder (pos,R,length,radius);
  }
*/
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
    for (int i=0; i<3; i++) bbpos[i] = 0.5*(aabb[i*2] + aabb[i*2+1]);
    dVector3 bbsides;
    for (int j=0; j<3; j++) bbsides[j] = aabb[j*2+1] - aabb[j*2];
    dMatrix3 RI;
    dRSetIdentity (RI);
    dsSetColorAlpha (1,0,0,0.5);
    dsDrawBox (bbpos,RI,bbsides);
  }
}


// simulation loop

static void simLoop (int pause)
{
  dsSetColor (0,0,2);
  dSpaceCollide (space,0,&nearCallback);
  if (!pause) dWorldStep (world,0.05);
  //if (!pause) dWorldStepFast (world,0.05, 1);

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
	dsSetColor (1,0,0);
      }
      else {
	dsSetColor (1,1,0);
      }
      drawGeom (obj[i].geom[j],0,0,show_aabb);
    }
  }

  /*{
    for (int i = 1; i < IndexCount; i++) {
      dsDrawLine(Vertices[Indices[i - 1]], Vertices[Indices[i]]);
    }
  }*/

  if (TriMesh)
  {const dReal* Pos = dGeomGetPosition(TriMesh);
  const dReal* Rot = dGeomGetRotation(TriMesh);

  {for (int i = 0; i < IndexCount / 3; i++){
    const dVector3& v0 = Vertices[Indices[i * 3 + 0]];
	const dVector3& v1 = Vertices[Indices[i * 3 + 1]];
	const dVector3& v2 = Vertices[Indices[i * 3 + 2]];
	dsDrawTriangle(Pos, Rot, (dReal*)&v0, (dReal*)&v1, (dReal*)&v2, 0);
  }}}

  if (Ray){
	  dVector3 Origin, Direction;
	  dGeomRayGet(Ray, Origin, Direction);
	  
	  dReal Length = dGeomRayGetLength(Ray);
	  
	  dVector3 End;
	  End[0] = Origin[0] + (Direction[0] * Length);
	  End[1] = Origin[1] + (Direction[1] * Length);
	  End[2] = Origin[2] + (Direction[2] * Length);
	  End[3] = Origin[3] + (Direction[3] * Length);
	  
	  dsDrawLine(Origin, End);
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

  space = dSimpleSpaceCreate(0);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,0,0,-0.5);
  dWorldSetCFM (world,1e-5);
  //dCreatePlane (space,0,0,1,0);
  memset (obj,0,sizeof(obj));
  
  Size[0] = 5.0f;
  Size[1] = 5.0f;
  Size[2] = 2.5f;
  
  Vertices[0][0] = -Size[0];
  Vertices[0][1] = -Size[1];
  Vertices[0][2] = Size[2];
  
  Vertices[1][0] = Size[0];
  Vertices[1][1] = -Size[1];
  Vertices[1][2] = Size[2];
  
  Vertices[2][0] = Size[0];
  Vertices[2][1] = Size[1];
  Vertices[2][2] = Size[2];
  
  Vertices[3][0] = -Size[0];
  Vertices[3][1] = Size[1];
  Vertices[3][2] = Size[2];
  
  Vertices[4][0] = 0;
  Vertices[4][1] = 0;
  Vertices[4][2] = 0;
  
  Indices[0] = 0;
  Indices[1] = 1;
  Indices[2] = 4;
  
  Indices[3] = 1;
  Indices[4] = 2;
  Indices[5] = 4;
  
  Indices[6] = 2;
  Indices[7] = 3;
  Indices[8] = 4;
  
  Indices[9] = 3;
  Indices[10] = 0;
  Indices[11] = 4;

  /*dTriMeshDataID Data = dGeomTriMeshDataCreate();

  dGeomTriMeshDataBuildSimple(Data, (dReal*)Vertices, VertexCount, Indices, IndexCount);
  
  TriMesh = dCreateTriMesh(space, Data, 0, 0, 0);
  */

  assert(0);
  //todo: create a Bullet BvhTriangleMeshShape instead of opcode trimesh
  //dGeomSetPosition(TriMesh, 0, 0, 1.0);
  
  Ray = dCreateRay(space, 0.9);
  dVector3 Origin, Direction;
  Origin[0] = 0.0;
  Origin[1] = 0;
  Origin[2] = 0.5;
  Origin[3] = 0;
  
  Direction[0] = 0;
  Direction[1] = 1.1f;
  Direction[2] = -1;
  Direction[3] = 0;
  dNormalize3(Direction);
  
  dGeomRaySet(Ray, Origin[0], Origin[1], Origin[2], Direction[0], Direction[1], Direction[2]);
  
  // run simulation
  dsSimulationLoop (argc,argv,352,288,&fn);

  dJointGroupDestroy (contactgroup);
  dSpaceDestroy (space);
  dWorldDestroy (world);

  return 0;
}
