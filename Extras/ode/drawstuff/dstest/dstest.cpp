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

#include <stdio.h>
#include <math.h>
#include <drawstuff/drawstuff.h>


#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif


void start()
{
  // adjust the starting viewpoint a bit
  float xyz[3],hpr[3];
  dsGetViewpoint (xyz,hpr);
  hpr[0] += 7;
  dsSetViewpoint (xyz,hpr);
}


void simLoop (int pause)
{
  float pos[3];
  float R[12];
  static float a = 0;

  if (!pause) a += 0.02f;
  if (a > (2*M_PI)) a -= (float) (2*M_PI);
  float ca = (float) cos(a);
  float sa = (float) sin(a);

  dsSetTexture (DS_WOOD);

  float b = (a > M_PI) ? (2*(a-(float)M_PI)) : a*2;
  pos[0] = -0.3f;
  pos[1] = 0;
  pos[2] = (float) (0.1f*(2*M_PI*b - b*b) + 0.65f);
  R[0] = ca; R[1] = 0; R[2] = -sa;
  R[4] = 0;  R[5] = 1; R[6] = 0;
  R[8] = sa; R[9] = 0; R[10] = ca;
  dsSetColor (1,0.8f,0.6f);
  dsDrawSphere (pos,R,0.3f);

  dsSetTexture (DS_NONE);

  pos[0] = -0.2f;
  pos[1] = 0.8f;
  pos[2] = 0.4f;
  R[0] = ca; R[1] = -sa; R[2] = 0;
  R[4] = sa; R[5] = ca;  R[6] = 0;
  R[8] = 0;  R[9] = 0;	 R[10] = 1;
  float sides[3] = {0.1f,0.4f,0.8f};
  dsSetColor (0.6f,0.6f,1);
  dsDrawBox (pos,R,sides);

  dsSetTexture (DS_WOOD);

  float r = 0.3f;		      // cylinder radius
  float d = (float)cos(a*2) * 0.4f;
  float cd = (float)cos(-d/r);
  float sd = (float)sin(-d/r);
  pos[0] = -0.2f;
  pos[1] = -1 + d;
  pos[2] = 0.3f;
  R[0] = 0;   R[1] = 0;  R[2] = -1;
  R[4] = -sd; R[5] = cd; R[6] =  0;
  R[8] =  cd; R[9] = sd; R[10] = 0;
  dsSetColor (0.4f,1,1);
  dsDrawCylinder (pos,R,0.8f,r);

  pos[0] = 0;
  pos[1] = 0;
  pos[2] = 0.2f;
  R[0] = 0; R[1] = sa; R[2] = -ca;
  R[4] = 0; R[5] = ca; R[6] = sa;
  R[8] = 1; R[9] = 0;  R[10] = 0;
  dsSetColor (1,0.9f,0.2f);
  dsDrawCappedCylinder (pos,R,0.8f,0.2f);
}


void command (int cmd)
{
  dsPrint ("received command %d (`%c')\n",cmd,cmd);
}


int main (int argc, char **argv)
{
  // setup pointers to callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = command;
  fn.stop = 0;
  fn.path_to_textures = 0;	// uses default

  // run simulation
  dsSimulationLoop (argc,argv,400,400,&fn);

  return 0;
}
