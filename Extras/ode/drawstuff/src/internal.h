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

/* functions supplied and used by the platform specific code */

#ifndef __DS_INTERNAL_H
#define __DS_INTERNAL_H

#include "drawstuff/drawstuff.h"


// supplied by platform specific code

void dsPlatformSimLoop (int window_width, int window_height,
			dsFunctions *fn, int initial_pause);


// used by platform specific code

void dsStartGraphics (int width, int height, dsFunctions *fn);
void dsDrawFrame (int width, int height, dsFunctions *fn, int pause);
void dsStopGraphics();
void dsMotion (int mode, int deltax, int deltay);

int dsGetShadows();
void dsSetShadows (int a);

int dsGetTextures();
void dsSetTextures (int a);

#endif
