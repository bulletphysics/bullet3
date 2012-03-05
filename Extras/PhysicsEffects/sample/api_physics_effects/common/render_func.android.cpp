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

#include "render_func.h"

#include "box.h"
#include "sphere.h"
#include "cylinder.h"

#include <EGL/egl.h>
#include <GLES/gl.h>

// local variables
static int screen_width,screen_height;
static PfxMatrix4 g_pMat,g_vMat;
static PfxVector3 g_viewPos,g_lightPos,g_viewTgt;
static float lightRadius,lightRadX,lightRadY;
static float viewRadius,viewRadX,viewRadY,viewHeight;

static unsigned short *box_wire_idx;
static unsigned short *sphere_wire_idx;
static unsigned short *cylinder_wire_idx;

#define MAX_MESH 5

static struct MeshBuff {
	float *vtx;
	float *nml;
	int numVtx;
	unsigned short *idx;
	unsigned short *wire_idx;
	int numIdx;
} meshBuff[MAX_MESH];
int numMesh;

void render_init()
{
	screen_width = DISPLAY_WIDTH;
	screen_height = DISPLAY_HEIGHT;

	// initalize matrix
	g_pMat = PfxMatrix4::perspective(3.1415f/4.0f, (float)screen_width/(float)screen_height,0.1f, 1000.0f);

	// initalize parameters
	lightRadius = 40.0f;
	lightRadX = -0.6f;
	lightRadY = 0.6f;
	viewRadius = 40.0f;
	viewRadX = -0.01f;
	viewRadY = 0.0f;
	viewHeight = 1.0f;

	g_viewTgt = PfxVector3(0.0f,viewHeight,0.0f);

	box_wire_idx = new unsigned short [NUM_BOX_IDX*2];
	sphere_wire_idx = new unsigned short [NUM_SPHERE_IDX*2];
	cylinder_wire_idx = new unsigned short [NUM_CYLINDER_IDX*2];

	for(int i=0;i<NUM_BOX_IDX/3;i++) {
		box_wire_idx[i*6  ] = box_idx[i*3  ];
		box_wire_idx[i*6+1] = box_idx[i*3+1];
		box_wire_idx[i*6+2] = box_idx[i*3+1];
		box_wire_idx[i*6+3] = box_idx[i*3+2];
		box_wire_idx[i*6+4] = box_idx[i*3+2];
		box_wire_idx[i*6+5] = box_idx[i*3  ];
	}

	for(int i=0;i<NUM_SPHERE_IDX/3;i++) {
		sphere_wire_idx[i*6  ] = sphere_idx[i*3  ];
		sphere_wire_idx[i*6+1] = sphere_idx[i*3+1];
		sphere_wire_idx[i*6+2] = sphere_idx[i*3+1];
		sphere_wire_idx[i*6+3] = sphere_idx[i*3+2];
		sphere_wire_idx[i*6+4] = sphere_idx[i*3+2];
		sphere_wire_idx[i*6+5] = sphere_idx[i*3  ];
	}

	for(int i=0;i<NUM_CYLINDER_IDX/3;i++) {
		cylinder_wire_idx[i*6  ] = cylinder_idx[i*3  ];
		cylinder_wire_idx[i*6+1] = cylinder_idx[i*3+1];
		cylinder_wire_idx[i*6+2] = cylinder_idx[i*3+1];
		cylinder_wire_idx[i*6+3] = cylinder_idx[i*3+2];
		cylinder_wire_idx[i*6+4] = cylinder_idx[i*3+2];
		cylinder_wire_idx[i*6+5] = cylinder_idx[i*3  ];
	}
	
	numMesh = 0;
}

void render_release()
{
	delete [] box_wire_idx;
	delete [] sphere_wire_idx;
	delete [] cylinder_wire_idx;
	
	if(numMesh > 0) {
		for(int i=0;i<numMesh;i++) {
			if(meshBuff[i].vtx) delete [] meshBuff[i].vtx;
			if(meshBuff[i].nml) delete [] meshBuff[i].nml;
			if(meshBuff[i].idx) delete [] meshBuff[i].idx;
			if(meshBuff[i].wire_idx) delete [] meshBuff[i].wire_idx;
		}
	}
}

void render_begin()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glFrontFace(GL_CCW);
    glDepthFunc(GL_LESS);
	glCullFace(GL_BACK);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMultMatrixf((GLfloat*)&g_pMat);

	// create view matrix
	g_viewPos = 
		PfxMatrix3::rotationY(viewRadY) * 
		PfxMatrix3::rotationX(viewRadX) * 
		PfxVector3(0,0,viewRadius);

	g_lightPos = 
		PfxMatrix3::rotationY(lightRadY) * 
		PfxMatrix3::rotationX(lightRadX) * 
		PfxVector3(0,0,lightRadius);

	PfxMatrix4 viewMtx = PfxMatrix4::lookAt(PfxPoint3(g_viewTgt + g_viewPos),PfxPoint3(g_viewTgt),PfxVector3(0,1,0));

	g_vMat = g_pMat * viewMtx;

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrixf((GLfloat*)&viewMtx);
}

void render_end()
{
}

void render_debug_begin()
{
	glDepthMask(GL_FALSE);
}

void render_debug_end()
{
	glDepthMask(GL_TRUE);
}

void render_get_view_angle(float &angleX,float &angleY,float &radius)
{
	angleX = viewRadX;
	angleY = viewRadY;
	radius = viewRadius;
}

void render_set_view_angle(float angleX,float angleY,float radius)
{
	viewRadX   = angleX;
	viewRadY   = angleY;
	viewRadius = radius;
}

void render_sphere(
	const PfxTransform3 &transform,
	const PfxVector3 &color,
	const PfxFloatInVec &radius)
{
	PfxMatrix4 wMtx = PfxMatrix4(transform) * PfxMatrix4::scale(PfxVector3(radius));

	glPushMatrix();
	glMultMatrixf((GLfloat*)&wMtx);
	
	glEnableClientState(GL_VERTEX_ARRAY);
	
	glVertexPointer(3,GL_FLOAT,24,sphere_vtx);

	glColor4f(color[0], color[1], color[2], 1.0);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0f,1.0f);
	glDrawElements(GL_TRIANGLES,NUM_SPHERE_IDX,GL_UNSIGNED_SHORT,sphere_idx);
	glDisable(GL_POLYGON_OFFSET_FILL);

	glColor4f(0.0f,0.0f,0.0f, 1.0);
	glDrawElements(GL_LINES,NUM_SPHERE_IDX*2,GL_UNSIGNED_SHORT,sphere_wire_idx);
	
	glDisableClientState(GL_VERTEX_ARRAY);

	glPopMatrix();
}

void render_box(
	const PfxTransform3 &transform,
	const PfxVector3 &color,
	const PfxVector3 &halfExtent)
{
	PfxMatrix4 wMtx = PfxMatrix4(transform) * PfxMatrix4::scale(2.0f*halfExtent);

	glPushMatrix();
	glMultMatrixf((GLfloat*)&wMtx);

	glEnableClientState(GL_VERTEX_ARRAY);
	
	glVertexPointer(3,GL_FLOAT,24,box_vtx);

	glColor4f(color[0], color[1], color[2], 1.0);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0f,1.0f);
	glDrawElements(GL_TRIANGLES,NUM_BOX_IDX,GL_UNSIGNED_SHORT,box_idx);
	glDisable(GL_POLYGON_OFFSET_FILL);

	glColor4f(0.0f,0.0f,0.0f, 1.0);
	glDrawElements(GL_LINES,NUM_BOX_IDX*2,GL_UNSIGNED_SHORT,box_wire_idx);
	
	glDisableClientState(GL_VERTEX_ARRAY);
	
	glPopMatrix();
}

void render_cylinder(
	const PfxTransform3 &transform,
	const PfxVector3 &color,
	const PfxFloatInVec &radius,
	const PfxFloatInVec &halfLength)
{
	PfxVector3 scale(halfLength,radius,radius);

	PfxMatrix4 wMtx = PfxMatrix4(transform) * PfxMatrix4::scale(scale);

	glPushMatrix();
	glMultMatrixf((GLfloat*)&wMtx);
	
	glEnableClientState(GL_VERTEX_ARRAY);

	glVertexPointer(3,GL_FLOAT,24,cylinder_vtx);

	glColor4f(color[0], color[1], color[2], 1.0);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0f,1.0f);
	glDrawElements(GL_TRIANGLES,NUM_CYLINDER_IDX,GL_UNSIGNED_SHORT,cylinder_idx);
	glDisable(GL_POLYGON_OFFSET_FILL);

	glColor4f(0.0f,0.0f,0.0f,1.0);
	glDrawElements(GL_LINES,NUM_CYLINDER_IDX*2,GL_UNSIGNED_SHORT,cylinder_wire_idx);
	
	glDisableClientState(GL_VERTEX_ARRAY);
	
	glPopMatrix();
}

void render_capsule(
	const PfxTransform3 &transform,
	const PfxVector3 &color,
	const PfxFloatInVec &radius,
	const PfxFloatInVec &halfLength)
{
	PfxTransform3 tr1 = PfxTransform3::translation(PfxVector3(-halfLength,0.0f,0.0f));
	PfxTransform3 tr2 = PfxTransform3::translation(PfxVector3(halfLength,0.0f,0.0f));

	render_sphere(transform*tr1,color,radius);
	render_sphere(transform*tr2,color,radius);

	render_cylinder(transform,color,radius,halfLength);
}

int render_init_mesh(
	const float *vtx,unsigned int vtxStrideBytes,
	const float *nml,unsigned int nmlStrideBytes,
	const unsigned short *tri,unsigned int triStrideBytes,
	int numVtx,int numTri)
{
	assert(numMesh<MAX_MESH);
	
	MeshBuff &buff = meshBuff[numMesh++];
	buff.vtx = new float [3*numVtx];
	buff.nml = new float [3*numVtx];
	buff.idx = new unsigned short [numTri*3];
	buff.wire_idx = new unsigned short [numTri*6];
	buff.numIdx = numTri*3;
	buff.numVtx = numVtx;

	for(int i=0;i<numVtx;i++) {
		const float *v = (float*)((uintptr_t)vtx + vtxStrideBytes * i);
		const float *n = (float*)((uintptr_t)nml + nmlStrideBytes * i);
		buff.vtx[i*3  ] = v[0];
		buff.vtx[i*3+1] = v[1];
		buff.vtx[i*3+2] = v[2];
		buff.nml[i*3  ] = n[0];
		buff.nml[i*3+1] = n[1];
		buff.nml[i*3+2] = n[2];
	}

	for(int i=0;i<numTri;i++) {
		const unsigned short *idx = (unsigned short*)((uintptr_t)tri + triStrideBytes * i);
		buff.idx[i*3  ] = idx[0];
		buff.idx[i*3+1] = idx[1];
		buff.idx[i*3+2] = idx[2];
		buff.wire_idx[i*6  ] = buff.idx[i*3  ];
		buff.wire_idx[i*6+1] = buff.idx[i*3+1];
		buff.wire_idx[i*6+2] = buff.idx[i*3+1];
		buff.wire_idx[i*6+3] = buff.idx[i*3+2];
		buff.wire_idx[i*6+4] = buff.idx[i*3+2];
		buff.wire_idx[i*6+5] = buff.idx[i*3  ];
	}

	return numMesh-1;
}

void render_mesh(
	const PfxTransform3 &transform,
	const PfxVector3 &color,
	int meshId)
{
	assert(meshId>=0&&meshId<MAX_MESH);
	
	MeshBuff &buff = meshBuff[meshId];

	PfxMatrix4 wMtx = PfxMatrix4(transform);

	glPushMatrix();
	glMultMatrixf((GLfloat*)&wMtx);

	glEnableClientState(GL_VERTEX_ARRAY);
	
	glVertexPointer(3,GL_FLOAT,0,buff.vtx);

	glColor4f(color[0], color[1], color[2], 1.0);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0f,1.0f);
	glDrawElements(GL_TRIANGLES,buff.numIdx,GL_UNSIGNED_SHORT,buff.idx);
	glDisable(GL_POLYGON_OFFSET_FILL);

	glColor4f(0.0f,0.0f,0.0f,1.0);
	glDrawElements(GL_LINES,buff.numIdx*2,GL_UNSIGNED_SHORT,buff.wire_idx);
	
	glDisableClientState(GL_VERTEX_ARRAY);
	
	glPopMatrix();
}

void render_resize(int width,int height)
{
	glViewport(0,0,width,height);
	g_pMat = PfxMatrix4::perspective(3.1415f/4.0f, (float)width/(float)height,0.1f, 1000.0f);
	screen_width = width;
	screen_height = height;
}

void render_debug_point(
	const PfxVector3 &position,
	const PfxVector3 &color)
{
	glColor4f(color[0], color[1], color[2], 1.0);

	glPointSize(5.0f);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT,16,(float*)&position);
	glDrawArrays(GL_POINTS,0,1);
	glDisableClientState(GL_VERTEX_ARRAY);
	glPointSize(1.0f);
}

void render_debug_line(
	const PfxVector3 &position1,
	const PfxVector3 &position2,
	const PfxVector3 &color)
{
	glColor4f(color[0], color[1], color[2], 1.0);
	
	const PfxVector3 points[2] = {
		position1,
		position2,
	};
	
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT,16,(float*)points);
	glDrawArrays(GL_LINES,0,2);
	glDisableClientState(GL_VERTEX_ARRAY);
}

void render_debug_box(
	const PfxVector3 &center,
	const PfxVector3 &extent,
	const PfxVector3 &color)
{
	const PfxVector3 points[8] = {
		center + mulPerElem(PfxVector3(-1,-1,-1),extent),
		center + mulPerElem(PfxVector3(-1,-1, 1),extent),
		center + mulPerElem(PfxVector3( 1,-1, 1),extent),
		center + mulPerElem(PfxVector3( 1,-1,-1),extent),
		center + mulPerElem(PfxVector3(-1, 1,-1),extent),
		center + mulPerElem(PfxVector3(-1, 1, 1),extent),
		center + mulPerElem(PfxVector3( 1, 1, 1),extent),
		center + mulPerElem(PfxVector3( 1, 1,-1),extent),
	};
	
	const unsigned short indices[] = {
		0,1,1,2,2,3,3,0,4,5,5,6,6,7,7,4,0,4,1,5,2,6,3,7,
	};
	
	glColor4f(color[0], color[1], color[2], 1.0);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT,16,(float*)points);
	glDrawElements(GL_LINES,24,GL_UNSIGNED_SHORT,indices);
	glDisableClientState(GL_VERTEX_ARRAY);
}

PfxVector3 render_get_world_position(const PfxVector3 &screenPos)
{
	PfxMatrix4 mvp,mvpInv;
	mvp = g_vMat;
	mvpInv = inverse(mvp);

	PfxVector4 wp(screenPos,1.0f);

	wp[0] /= (0.5f * (float)screen_width);
	wp[1] /= (0.5f * (float)screen_height);

	float w =	mvpInv[0][3] * wp[0] +  
				mvpInv[1][3] * wp[1] +  
				mvpInv[2][3] * wp[2] +  
				mvpInv[3][3];

	wp = mvpInv * wp;
	wp /= w;

	return wp.getXYZ();
}

PfxVector3 render_get_screen_position(const PfxVector3 &worldPos)
{
	PfxVector4 sp(worldPos,1.0f);

	PfxMatrix4 mvp;
	mvp = g_vMat;

	sp = mvp * sp;
	sp /= sp[3];
	sp[0] *= (0.5f * (float)screen_width);
	sp[1] *= (0.5f * (float)screen_height);

	return sp.getXYZ();
}

void render_get_screent_size(int &width,int &height)
{
	width = screen_width;
	height = screen_height;
}

void render_get_view_target(PfxVector3 &targetPos)
{
	targetPos = g_viewTgt;
}

void render_set_view_target(const PfxVector3 &targetPos)
{
	g_viewTgt = targetPos;
}

void render_get_view_radius(float &radius)
{
	radius = viewRadius;
}

void render_set_view_radius(float radius)
{
	viewRadius = radius;
}
