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

#ifndef _SCE_PFX_BASE_LEVEL_INCLUDE_H
#define _SCE_PFX_BASE_LEVEL_INCLUDE_H

///////////////////////////////////////////////////////////////////////////////
// Physics Effects Base Level Headers

#include "base/pfx_common.h"
#include "base/pfx_perf_counter.h"

#include "rigidbody/pfx_rigid_state.h"
#include "rigidbody/pfx_rigid_body.h"

#include "broadphase/pfx_broadphase_pair.h"
#include "broadphase/pfx_broadphase_proxy.h"
#include "broadphase/pfx_update_broadphase_proxy.h"

#include "collision/pfx_collidable.h"
#include "collision/pfx_shape_iterator.h"
#include "collision/pfx_contact_manifold.h"

#include "solver/pfx_constraint_pair.h"
#include "solver/pfx_contact_constraint.h"
#include "solver/pfx_joint_ball.h"
#include "solver/pfx_joint_swing_twist.h"
#include "solver/pfx_joint_hinge.h"
#include "solver/pfx_joint_slider.h"
#include "solver/pfx_joint_fix.h"
#include "solver/pfx_joint_universal.h"

#include "solver/pfx_integrate.h"

#include "sort/pfx_sort.h"

#endif // _SCE_PFX_BASE_LEVEL_INCLUDE_H