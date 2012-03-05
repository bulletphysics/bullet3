/*
Physics Effects Copyright(C) 2011 Sony Computer Entertainment Inc.
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

#include "physics_effects.h"
#include "physics_func.h"
#include "../common/perf_func.h"

static int frameCount = 0;
static int sceneId = 2;

int main()
{

	perf_init();
	physics_init();

	physics_create_scene(sceneId);

	frameCount = 0;

	//createScene();

	while(frameCount<600) {
		physics_simulate();
		perf_sync();
	}

	SCE_PFX_PRINTF("program complete\n");

	return 0;
}
