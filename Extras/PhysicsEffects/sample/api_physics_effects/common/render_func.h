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

#ifndef __RENDER_FUNC_H__
#define __RENDER_FUNC_H__

#include "common.h"
#include "physics_effects.h"

using namespace sce::PhysicsEffects;

#define	DISPLAY_WIDTH			640
#define	DISPLAY_HEIGHT			480

///////////////////////////////////////////////////////////////////////////////
// Draw Primitives

void render_init();
void render_release();
void render_begin();
void render_end();

void render_sphere(
	const PfxTransform3 &transform,
	const PfxVector3 &color,
	const PfxFloatInVec &radius);

void render_box(
	const PfxTransform3 &transform,
	const PfxVector3 &color,
	const PfxVector3 &halfExtent);

void render_cylinder(
	const PfxTransform3 &transform,
	const PfxVector3 &color,
	const PfxFloatInVec &radius,
	const PfxFloatInVec &halfLength);

void render_capsule(
	const PfxTransform3 &transform,
	const PfxVector3 &color,
	const PfxFloatInVec &radius,
	const PfxFloatInVec &halfLength);

int render_init_mesh(
	const float *vtx,unsigned int vtxStrideBytes,
	const float *nml,unsigned int nmlStrideBytes,
	const unsigned short *tri,unsigned int triStrideBytes,
	int numVtx,int numTri);

void render_mesh(
	const PfxTransform3 &transform,
	const PfxVector3 &color,
	int meshId);

///////////////////////////////////////////////////////////////////////////////
// Debug Drawing

void render_debug_begin();
void render_debug_end();

void render_debug_point(
	const PfxVector3 &position,
	const PfxVector3 &color);

void render_debug_line(
	const PfxVector3 &position1,
	const PfxVector3 &position2,
	const PfxVector3 &color);

void render_debug_box(
	const PfxVector3 &center,
	const PfxVector3 &extent,
	const PfxVector3 &color);

///////////////////////////////////////////////////////////////////////////////
// Render Parameter

void render_get_view_angle(float &angleX,float &angleY,float &radius);
void render_set_view_angle(float angleX,float angleY,float radius);
void render_resize(int width,int height);

void render_get_view_target(PfxVector3 &targetPos);
void render_set_view_target(const PfxVector3 &targetPos);
void render_get_view_radius(float &radius);
void render_set_view_radius(float radius);

void render_get_screent_size(int &width,int &height);

PfxVector3 render_get_world_position(const PfxVector3 &screenPos);
PfxVector3 render_get_screen_position(const PfxVector3 &worldPos);

#endif /* __RENDER_FUNC_H__ */
