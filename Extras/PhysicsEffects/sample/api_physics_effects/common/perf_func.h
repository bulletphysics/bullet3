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

#ifndef __PERF_FUNC_H__
#define __PERF_FUNC_H__

void perf_init();
void perf_release();
void perf_push_marker(char *str);
void perf_pop_marker();
void perf_sync();

#endif /* __PERF_FUNC_H__ */
