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

#ifndef _SCE_PFX_LOW_LEVEL_INCLUDE_H
#define _SCE_PFX_LOW_LEVEL_INCLUDE_H

///////////////////////////////////////////////////////////////////////////////
// Physics Effects Low Level Headers

// Include base level headers
#include "../base_level/pfx_base_level_include.h"

// Include low level headers
#include "broadphase/pfx_broadphase.h"

#include "collision/pfx_collision_detection.h"
#include "collision/pfx_refresh_contacts.h"
#include "collision/pfx_batched_ray_cast.h"
#include "collision/pfx_ray_cast.h"
#include "collision/pfx_island_generation.h"

#include "solver/pfx_constraint_solver.h"
#include "solver/pfx_update_rigid_states.h"

#include "sort/pfx_parallel_sort.h"

// ARA begin insert new code 
#ifdef USE_PTHREADS
#include "task/pfx_pthreads.h"
#endif
// ARA end

#endif // _SCE_PFX_LOW_LEVEL_INCLUDE_H
