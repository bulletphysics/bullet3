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

/*

Draw Stuff
----------

this is a library for rendering simple 3D objects in a virtual environment.

NOTES
-----

in the virtual world, the z axis is "up" and z=0 is the floor.

the user is able to click+drag in the main window to move the camera:
  * left button - pan and tilt.
  * right button - forward and sideways.
  * left + right button (or middle button) - sideways and up.

*/


#ifndef __DRAWSTUFF_H__
#define __DRAWSTUFF_H__

#ifdef __cplusplus
extern "C" {
#endif


#include <drawstuff/version.h>


/* texture numbers */
#define DS_NONE   0	/* uses the current color instead of a texture */
#define DS_WOOD   1


typedef struct dsFunctions {
  int version;			/* put DS_VERSION here */
  /* version 1 data */
  void (*start)();		/* called before sim loop starts */
  void (*step) (int pause);	/* called before every frame */
  void (*command) (int cmd);	/* called if a command key is pressed */
  void (*stop)();		/* called after sim loop exits */
  /* version 2 data */
  char *path_to_textures;	/* if nonzero, path to texture files */
} dsFunctions;


/* the main() function should fill in the dsFunctions structure then
 * call this.
 */
void dsSimulationLoop (int argc, char **argv,
		       int window_width, int window_height,
		       struct dsFunctions *fn);

/* these functions display an error message then exit. they take arguments
 * in the same way as printf(), except you do not have to add a terminating
 * '\n'. Debug() tries to dump core or start the debugger.
 */
void dsError (char *msg, ...);
void dsDebug (char *msg, ...);

/* dsPrint() prints out a message. it takes arguments in the same way as
 * printf() (i.e. you must add a '\n' at the end of every line).
 */
void dsPrint (char *msg, ...);

/* set and get the camera position. xyz is the cameria position (x,y,z).
 * hpr contains heading, pitch and roll numbers in degrees. heading=0
 * points along the x axis, pitch=0 is looking towards the horizon, and
 * roll 0 is "unrotated".
 */
void dsSetViewpoint (float xyz[3], float hpr[3]);
void dsGetViewpoint (float xyz[3], float hpr[3]);

/* stop the simulation loop. calling this from within dsSimulationLoop()
 * will cause it to exit and return to the caller. it is the same as if the
 * user used the exit command. using this outside the loop will have no
 * effect.
 */
void dsStop();

/* change the way objects are drawn. these changes will apply to all further
 * dsDrawXXX() functions. the texture number must be a DS_xxx texture
 * constant. the red, green, and blue number are between 0 and 1.
 * alpha is between 0 and 1 - if alpha is not specified it's assubed to be 1.
 * the current texture is colored according to the current color.
 * at the start of each frame, the texture is reset to none and the color is
 * reset to white.
 */
void dsSetTexture (int texture_number);
void dsSetColor (float red, float green, float blue);
void dsSetColorAlpha (float red, float green, float blue, float alpha);

/* draw objects.
 *   - pos[] is the x,y,z of the center of the object.
 *   - R[] is a 3x3 rotation matrix for the object, stored by row like this:
 *        [ R11 R12 R13 0 ]
 *        [ R21 R22 R23 0 ]
 *        [ R31 R32 R33 0 ]
 *   - sides[] is an array of x,y,z side lengths.
 *   - all cylinders are aligned along the z axis.
 */
void dsDrawBox (const float pos[3], const float R[12], const float sides[3]);
void dsDrawSphere (const float pos[3], const float R[12], float radius);
void dsDrawTriangle (const float pos[3], const float R[12],
		     const float *v0, const float *v1, const float *v2, int solid);
void dsDrawCylinder (const float pos[3], const float R[12],
		     float length, float radius);
void dsDrawCone(const float pos[3], const float R[12],
		     float length, float radius);

void dsDrawCylinder2 (const float pos[3], const float R[12],
		     float length, float radius,float radius2);



void dsDrawCappedCylinder (const float pos[3], const float R[12],
			   float length, float radius);
void dsDrawLine (const float pos1[3], const float pos2[3]);

/* these drawing functions are identical to the ones above, except they take
 * double arrays for `pos' and `R'.
 */
void dsDrawBoxD (const double pos[3], const double R[12],
		 const double sides[3]);
void dsDrawSphereD (const double pos[3], const double R[12],
		    const float radius);
void dsDrawTriangleD (const double pos[3], const double R[12],
		      const double *v0, const double *v1, const double *v2, int solid);
void dsDrawCylinderD (const double pos[3], const double R[12],
		      float length, float radius);
void dsDrawCappedCylinderD (const double pos[3], const double R[12],
			    float length, float radius);
void dsDrawLineD (const double pos1[3], const double pos2[3]);

/* Set the drawn quality of the objects. Higher numbers are higher quality,
 * but slower to draw. This must be set before the first objects are drawn to
 * be effective.
 */
void dsSetSphereQuality (int n);		/* default = 1 */
void dsSetCappedCylinderQuality (int n);	/* default = 3 */


/* closing bracket for extern "C" */
#ifdef __cplusplus
}
#endif

#endif
