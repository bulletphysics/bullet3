
/* This demo has been modified to use the Bullet C-API.
 The C-API is minimal, and will develop based on developer feedback.
 The C++ API is recommended, and compatible with the C-API.
*/

/* Copyright (c) Mark J. Kilgard, 1994, 1997.  */

/* This program is freely distributable without licensing fees 
   and is provided without guarantee or warrantee expressed or 
   implied. This program is -not- in the public domain. */

/* Example for PC game developers to show how to *combine* texturing,
   reflections, and projected shadows all in real-time with OpenGL.
   Robust reflections use stenciling.  Robust projected shadows
   use both stenciling and polygon offset.  PC game programmers
   should realize that neither stenciling nor polygon offset are 
   supported by Direct3D, so these real-time rendering algorithms
   are only really viable with OpenGL. 
   
   The program has modes for disabling the stenciling and polygon
   offset uses.  It is worth running this example with these features
   toggled off so you can see the sort of artifacts that result.
   
   Notice that the floor texturing, reflections, and shadowing
   all co-exist properly. */

/* When you run this program:  Left mouse button controls the
   view.  Middle mouse button controls light position (left &
   right rotates light around dino; up & down moves light
   position up and down).  Right mouse button pops up menu. */

/* Check out the comments in the "redraw" routine to see how the
   reflection blending and surface stenciling is done.  You can
   also see in "redraw" how the projected shadows are rendered,
   including the use of stenciling and polygon offset. */

/* This program is derived from glutdino.c */

/* Compile: cc -o dinoshade dinoshade.c -lglut -lGLU -lGL -lXmu -lXext -lX11 -lm */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>       /* for cos(), sin(), and sqrt() */
#ifdef WIN32//for glut.h
#include <windows.h>
#endif

#ifndef WIN32
#ifndef CALLBACK
#define CALLBACK
#endif
#endif

//think different
#if defined(__APPLE__) && !defined (VMDMESA)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#define GLVOIDPTR GLvoid(*)()
#else
#include <GL/glut.h>
#include <GL/glext.h>
#define GLVOIDPTR void(CALLBACK*)()
#endif

/* Some <math.h> files do not define M_PI... */
#ifndef M_PI
#define M_PI 3.14159265
#endif


//#include "../../include/Bullet-C-Api.h"
#include "Bullet-C-Api.h"



plPhysicsSdkHandle	physicsSdk=0;
plDynamicsWorldHandle	dynamicsWorld=0;
plRigidBodyHandle floorRigidBody;
plRigidBodyHandle dinoRigidBody;

/* Variable controlling various rendering modes. */
static int stencilReflection = 1, stencilShadow = 1, offsetShadow = 1;
static int renderShadow = 1, renderDinosaur = 1, renderReflection = 1;
static int linearFiltering = 0, useMipmaps = 0, useTexture = 1;
static int reportSpeed = 0;
static int animation = 1;
static GLboolean lightSwitch = GL_TRUE;
static int directionalLight = 1;
static int forceExtension = 0;

/* Time varying or user-controled variables. */
static float jump = 0.0;
static float lightAngle = 0.0, lightHeight = 20;
GLfloat angle = -150;   /* in degrees */
GLfloat angle2 = 30;   /* in degrees */

int moving, startx, starty;
int lightMoving = 0, lightStartX, lightStartY;

enum {
  MISSING, EXTENSION, ONE_DOT_ONE
};
int polygonOffsetVersion;

static GLdouble bodyWidth = 3.0;
/* *INDENT-OFF* */
static GLfloat body[][2] = { {0, 3}, {1, 1}, {5, 1}, {8, 4}, {10, 4}, {11, 5},
  {11, 11.5}, {13, 12}, {13, 13}, {10, 13.5}, {13, 14}, {13, 15}, {11, 16},
  {8, 16}, {7, 15}, {7, 13}, {8, 12}, {7, 11}, {6, 6}, {4, 3}, {3, 2},
  {1, 2} };
static GLfloat arm[][2] = { {8, 10}, {9, 9}, {10, 9}, {13, 8}, {14, 9}, {16, 9},
  {15, 9.5}, {16, 10}, {15, 10}, {15.5, 11}, {14.5, 10}, {14, 11}, {14, 10},
  {13, 9}, {11, 11}, {9, 11} };
static GLfloat leg[][2] = { {8, 6}, {8, 4}, {9, 3}, {9, 2}, {8, 1}, {8, 0.5}, {9, 0},
  {12, 0}, {10, 1}, {10, 2}, {12, 4}, {11, 6}, {10, 7}, {9, 7} };
static GLfloat eye[][2] = { {8.75, 15}, {9, 14.7}, {9.6, 14.7}, {10.1, 15},
  {9.6, 15.25}, {9, 15.25} };
static GLfloat lightPosition[4];
static GLfloat lightColor[] = {0.8, 1.0, 0.8, 1.0}; /* green-tinted */
static GLfloat skinColor[] = {0.1, 1.0, 0.1, 1.0}, eyeColor[] = {1.0, 0.2, 0.2, 1.0};
/* *INDENT-ON* */

/* Nice floor texture tiling pattern. */
static char *circles[] = {
  "....xxxx........",
  "..xxxxxxxx......",
  ".xxxxxxxxxx.....",
  ".xxx....xxx.....",
  "xxx......xxx....",
  "xxx......xxx....",
  "xxx......xxx....",
  "xxx......xxx....",
  ".xxx....xxx.....",
  ".xxxxxxxxxx.....",
  "..xxxxxxxx......",
  "....xxxx........",
  "................",
  "................",
  "................",
  "................",
};

static void
makeFloorTexture(void)
{
  GLubyte floorTexture[16][16][3];
  GLubyte *loc;
  int s, t;
  loc=0;

  /* Setup RGB image for the texture. */
  loc = (GLubyte*) floorTexture;
  for (t = 0; t < 16; t++) {
    for (s = 0; s < 16; s++) {
      if (circles[t][s] == 'x') {
	/* Nice green. */
        loc[0] = 0x1f;
        loc[1] = 0x8f;
        loc[2] = 0x1f;
      } else {
	/* Light gray. */
        loc[0] = 0xaa;
        loc[1] = 0xaa;
        loc[2] = 0xaa;
      }
      loc += 3;
    }
  }

  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  if (useMipmaps) {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
      GL_LINEAR_MIPMAP_LINEAR);
    gluBuild2DMipmaps(GL_TEXTURE_2D, 3, 16, 16,
      GL_RGB, GL_UNSIGNED_BYTE, floorTexture);
  } else {
    if (linearFiltering) {
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    } else {
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    }
    glTexImage2D(GL_TEXTURE_2D, 0, 3, 16, 16, 0,
      GL_RGB, GL_UNSIGNED_BYTE, floorTexture);
  }
}

enum {
  X, Y, Z, W
};
enum {
  A, B, C, D
};

/* Create a matrix that will project the desired shadow. */
void
shadowMatrix(GLfloat shadowMat[4][4],
  GLfloat groundplane[4],
  GLfloat lightpos[4])
{
  GLfloat dot;

  /* Find dot product between light position vector and ground plane normal. */
  dot = groundplane[X] * lightpos[X] +
    groundplane[Y] * lightpos[Y] +
    groundplane[Z] * lightpos[Z] +
    groundplane[W] * lightpos[W];

  shadowMat[0][0] = dot - lightpos[X] * groundplane[X];
  shadowMat[1][0] = 0.f - lightpos[X] * groundplane[Y];
  shadowMat[2][0] = 0.f - lightpos[X] * groundplane[Z];
  shadowMat[3][0] = 0.f - lightpos[X] * groundplane[W];

  shadowMat[X][1] = 0.f - lightpos[Y] * groundplane[X];
  shadowMat[1][1] = dot - lightpos[Y] * groundplane[Y];
  shadowMat[2][1] = 0.f - lightpos[Y] * groundplane[Z];
  shadowMat[3][1] = 0.f - lightpos[Y] * groundplane[W];

  shadowMat[X][2] = 0.f - lightpos[Z] * groundplane[X];
  shadowMat[1][2] = 0.f - lightpos[Z] * groundplane[Y];
  shadowMat[2][2] = dot - lightpos[Z] * groundplane[Z];
  shadowMat[3][2] = 0.f - lightpos[Z] * groundplane[W];

  shadowMat[X][3] = 0.f - lightpos[W] * groundplane[X];
  shadowMat[1][3] = 0.f - lightpos[W] * groundplane[Y];
  shadowMat[2][3] = 0.f - lightpos[W] * groundplane[Z];
  shadowMat[3][3] = dot - lightpos[W] * groundplane[W];

}

/* Find the plane equation given 3 points. */
void
findPlane(GLfloat plane[4],
  GLfloat v0[3], GLfloat v1[3], GLfloat v2[3])
{
  GLfloat vec0[3], vec1[3];

  /* Need 2 vectors to find cross product. */
  vec0[X] = v1[X] - v0[X];
  vec0[Y] = v1[Y] - v0[Y];
  vec0[Z] = v1[Z] - v0[Z];

  vec1[X] = v2[X] - v0[X];
  vec1[Y] = v2[Y] - v0[Y];
  vec1[Z] = v2[Z] - v0[Z];

  /* find cross product to get A, B, and C of plane equation */
  plane[A] = vec0[Y] * vec1[Z] - vec0[Z] * vec1[Y];
  plane[B] = -(vec0[X] * vec1[Z] - vec0[Z] * vec1[X]);
  plane[C] = vec0[X] * vec1[Y] - vec0[Y] * vec1[X];

  plane[D] = -(plane[A] * v0[X] + plane[B] * v0[Y] + plane[C] * v0[Z]);
}

void
extrudeSolidFromPolygon(GLfloat data[][2], unsigned int dataSize,
  GLdouble thickness, GLuint side, GLuint edge, GLuint whole)
{
  static GLUtriangulatorObj *tobj = NULL;
  GLdouble vertex[3], dx, dy, len;
  int i;
  int count = dataSize / (2 * sizeof(GLfloat));

  if (tobj == NULL) {
    tobj = gluNewTess();  /* create and initialize a GLU
                             polygon * * tesselation object */
    gluTessCallback(tobj, (GLenum)GLU_BEGIN, (GLVOIDPTR)glBegin);
    gluTessCallback(tobj, (GLenum)GLU_VERTEX, (GLVOIDPTR)glVertex2fv);  /* semi-tricky */
    gluTessCallback(tobj, (GLenum)GLU_END, (GLVOIDPTR)glEnd);
  }
  glNewList(side, GL_COMPILE);
  glShadeModel(GL_SMOOTH);  /* smooth minimizes seeing
                               tessellation */
  gluBeginPolygon(tobj);
  for (i = 0; i < count; i++) {
    vertex[0] = data[i][0];
    vertex[1] = data[i][1];
    vertex[2] = 0;
    gluTessVertex(tobj, vertex, data[i]);
  }
  gluEndPolygon(tobj);
  glEndList();
  glNewList(edge, GL_COMPILE);
  glShadeModel(GL_FLAT);  /* flat shade keeps angular hands
                             from being "smoothed" */
  glBegin(GL_QUAD_STRIP);
  for (i = 0; i <= count; i++) {
    /* mod function handles closing the edge */
    glVertex3f(data[i % count][0], data[i % count][1], 0.0);
    glVertex3f(data[i % count][0], data[i % count][1], thickness);
    /* Calculate a unit normal by dividing by Euclidean
       distance. We * could be lazy and use
       glEnable(GL_NORMALIZE) so we could pass in * arbitrary
       normals for a very slight performance hit. */
    dx = data[(i + 1) % count][1] - data[i % count][1];
    dy = data[i % count][0] - data[(i + 1) % count][0];
    len = sqrt(dx * dx + dy * dy);
    glNormal3f(dx / len, dy / len, 0.0);
  }
  glEnd();
  glEndList();
  glNewList(whole, GL_COMPILE);
  glFrontFace(GL_CW);
  glCallList(edge);
  glNormal3f(0.0, 0.0, -1.0);  /* constant normal for side */
  glCallList(side);
  glPushMatrix();
  glTranslatef(0.0, 0.0, thickness);
  glFrontFace(GL_CCW);
  glNormal3f(0.0, 0.0, 1.0);  /* opposite normal for other side */
  glCallList(side);
  glPopMatrix();
  glEndList();
}

/* Enumerants for refering to display lists. */
typedef enum {
  RESERVED, BODY_SIDE, BODY_EDGE, BODY_WHOLE, ARM_SIDE, ARM_EDGE, ARM_WHOLE,
  LEG_SIDE, LEG_EDGE, LEG_WHOLE, EYE_SIDE, EYE_EDGE, EYE_WHOLE
} displayLists;

static void
makeDinosaur(void)
{
  extrudeSolidFromPolygon(body, sizeof(body), bodyWidth,
    BODY_SIDE, BODY_EDGE, BODY_WHOLE);
  extrudeSolidFromPolygon(arm, sizeof(arm), bodyWidth / 4,
    ARM_SIDE, ARM_EDGE, ARM_WHOLE);
  extrudeSolidFromPolygon(leg, sizeof(leg), bodyWidth / 2,
    LEG_SIDE, LEG_EDGE, LEG_WHOLE);
  extrudeSolidFromPolygon(eye, sizeof(eye), bodyWidth + 0.2,
    EYE_SIDE, EYE_EDGE, EYE_WHOLE);
}

static void
drawDinosaur(void)

{
	plReal matrix[16];

	glPushMatrix();
  /* Translate the dinosaur to be at (0,8,0). */

	plGetOpenGLMatrix(dinoRigidBody,matrix);
//	plGetPosition(dinoRigidBody,dinoWorldPos);
 // glTranslatef(-8, 0, -bodyWidth / 2);
  //glTranslatef(0.0, jump, 0.0);
//	glTranslatef(dinoWorldPos[0],dinoWorldPos[1],dinoWorldPos[2]);

#ifdef BT_USE_DOUBLE_PRECISION
	glMultMatrixd(matrix);
#else
	glMultMatrixf(matrix);
#endif
//	glutSolidCube(15);
	glTranslatef(-8.5, -8.5, 0);

  glMaterialfv(GL_FRONT, GL_DIFFUSE, skinColor);
  glCallList(BODY_WHOLE);
  glTranslatef(0.0, 0.0, bodyWidth);
  glCallList(ARM_WHOLE);
  glCallList(LEG_WHOLE);
  glTranslatef(0.0, 0.0, -bodyWidth - bodyWidth / 4);
  glCallList(ARM_WHOLE);
  glTranslatef(0.0, 0.0, -bodyWidth / 4);
  glCallList(LEG_WHOLE);
  glTranslatef(0.0, 0.0, bodyWidth / 2 - 0.1);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, eyeColor);
  glCallList(EYE_WHOLE);
  
	
  glPopMatrix();
}

static GLfloat floorVertices[4][3] = {
  { -20.0, 0.0, 20.0 },
  { 20.0, 0.0, 20.0 },
  { 20.0, 0.0, -20.0 },
  { -20.0, 0.0, -20.0 },
};

/* Draw a floor (possibly textured). */
static void
drawFloor(void)
{
  glDisable(GL_LIGHTING);

  if (useTexture) {
    glEnable(GL_TEXTURE_2D);
  }

  glBegin(GL_QUADS);
    glTexCoord2f(0.0, 0.0);
    glVertex3fv(floorVertices[0]);
    glTexCoord2f(0.0, 16.0);
    glVertex3fv(floorVertices[1]);
    glTexCoord2f(16.0, 16.0);
    glVertex3fv(floorVertices[2]);
    glTexCoord2f(16.0, 0.0);
    glVertex3fv(floorVertices[3]);
  glEnd();

  if (useTexture) {
    glDisable(GL_TEXTURE_2D);
  }

  glEnable(GL_LIGHTING);
}

static GLfloat floorPlane[4];
static GLfloat floorShadow[4][4];

static void
redraw(void)
{
  int start = 0, end = 0 ;

  if (reportSpeed) {
    start = glutGet(GLUT_ELAPSED_TIME);
  }

  /* Clear; default stencil clears to zero. */
  if ((stencilReflection && renderReflection) || (stencilShadow && renderShadow)) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
  } else {
    /* Avoid clearing stencil when not using it. */
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  }

  /* Reposition the light source. */
  lightPosition[0] = 12*cos(lightAngle);
  lightPosition[1] = lightHeight;
  lightPosition[2] = 12*sin(lightAngle);
  if (directionalLight) {
    lightPosition[3] = 0.0;
  } else {
    lightPosition[3] = 1.0;
  }

  shadowMatrix(floorShadow, floorPlane, lightPosition);

  glPushMatrix();
    /* Perform scene rotations based on user mouse input. */
    glRotatef(angle2, 1.0, 0.0, 0.0);
    glRotatef(angle, 0.0, 1.0, 0.0);
     
    /* Tell GL new light source position. */
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

    if (renderReflection) {
      if (stencilReflection) {
        /* We can eliminate the visual "artifact" of seeing the "flipped"
  	   dinosaur underneath the floor by using stencil.  The idea is
	   draw the floor without color or depth update but so that 
	   a stencil value of one is where the floor will be.  Later when
	   rendering the dinosaur reflection, we will only update pixels
	   with a stencil value of 1 to make sure the reflection only
	   lives on the floor, not below the floor. */

        /* Don't update color or depth. */
        glDisable(GL_DEPTH_TEST);
        glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

        /* Draw 1 into the stencil buffer. */
        glEnable(GL_STENCIL_TEST);
        glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);
        glStencilFunc(GL_ALWAYS, 1, 0xffffffff);

        /* Now render floor; floor pixels just get their stencil set to 1. */
        drawFloor();

        /* Re-enable update of color and depth. */ 
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        glEnable(GL_DEPTH_TEST);

        /* Now, only render where stencil is set to 1. */
        glStencilFunc(GL_EQUAL, 1, 0xffffffff);  /* draw if ==1 */
        glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
      }

      glPushMatrix();

        /* The critical reflection step: Reflect dinosaur through the floor
           (the Y=0 plane) to make a relection. */
        glScalef(1.0, -1.0, 1.0);

	/* Reflect the light position. */
        glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

        /* To avoid our normals getting reversed and hence botched lighting
	   on the reflection, turn on normalize.  */
        glEnable(GL_NORMALIZE);
        glCullFace(GL_FRONT);

        /* Draw the reflected dinosaur. */
        drawDinosaur();

        /* Disable noramlize again and re-enable back face culling. */
        glDisable(GL_NORMALIZE);
        glCullFace(GL_BACK);

      glPopMatrix();

      /* Switch back to the unreflected light position. */
      glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

      if (stencilReflection) {
        glDisable(GL_STENCIL_TEST);
      }
    }

    /* Back face culling will get used to only draw either the top or the
       bottom floor.  This let's us get a floor with two distinct
       appearances.  The top floor surface is reflective and kind of red.
       The bottom floor surface is not reflective and blue. */

    /* Draw "bottom" of floor in blue. */
    glFrontFace(GL_CW);  /* Switch face orientation. */
    glColor4f(0.1, 0.1, 0.7, 1.0);
    drawFloor();
    glFrontFace(GL_CCW);

    if (renderShadow) {
      if (stencilShadow) {
	/* Draw the floor with stencil value 3.  This helps us only 
	   draw the shadow once per floor pixel (and only on the
	   floor pixels). */
        glEnable(GL_STENCIL_TEST);
        glStencilFunc(GL_ALWAYS, 3, 0xffffffff);
        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
      }
    }

    /* Draw "top" of floor.  Use blending to blend in reflection. */
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0.7, 0.0, 0.0, 0.3);
    glColor4f(1.0, 1.0, 1.0, 0.3);
    drawFloor();
    glDisable(GL_BLEND);

    if (renderDinosaur) {
      /* Draw "actual" dinosaur, not its reflection. */
      drawDinosaur();
    }

    if (renderShadow) {

      /* Render the projected shadow. */

      if (stencilShadow) {

        /* Now, only render where stencil is set above 2 (ie, 3 where
	   the top floor is).  Update stencil with 2 where the shadow
	   gets drawn so we don't redraw (and accidently reblend) the
	   shadow). */
        glStencilFunc(GL_LESS, 2, 0xffffffff);  /* draw if ==1 */
        glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);
      }

      /* To eliminate depth buffer artifacts, we use polygon offset
	 to raise the depth of the projected shadow slightly so
	 that it does not depth buffer alias with the floor. */
      if (offsetShadow) {
	switch (polygonOffsetVersion) {
	case EXTENSION:
#ifdef GL_VERSION_1_1
	case ONE_DOT_ONE:
          glEnable(GL_POLYGON_OFFSET_FILL);
	  break;
#endif
	case MISSING:
	  /* Oh well. */
	  break;
	}
      }

      /* Render 50% black shadow color on top of whatever the
         floor appareance is. */
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glDisable(GL_LIGHTING);  /* Force the 50% black. */
      glColor4f(0.0, 0.0, 0.0, 0.5);

      glPushMatrix();
	/* Project the shadow. */
        glMultMatrixf((GLfloat *) floorShadow);
        drawDinosaur();
      glPopMatrix();

      glDisable(GL_BLEND);
      glEnable(GL_LIGHTING);

      if (offsetShadow) {
	switch (polygonOffsetVersion) {
#ifdef GL_VERSION_1_1
	case ONE_DOT_ONE:
          glDisable(GL_POLYGON_OFFSET_FILL);
	  break;
#endif
	case MISSING:
	  /* Oh well. */
	  break;
	}
      }
      if (stencilShadow) {
        glDisable(GL_STENCIL_TEST);
      }
    }

    glPushMatrix();
    glDisable(GL_LIGHTING);
    glColor3f(1.0, 1.0, 0.0);
    if (directionalLight) {
      /* Draw an arrowhead. */
      glDisable(GL_CULL_FACE);
      glTranslatef(lightPosition[0], lightPosition[1], lightPosition[2]);
      glRotatef(lightAngle * -180.0 / M_PI, 0, 1, 0);
      glRotatef(atan(lightHeight/12) * 180.0 / M_PI, 0, 0, 1);
      glBegin(GL_TRIANGLE_FAN);
	glVertex3f(0, 0, 0);
	glVertex3f(2, 1, 1);
	glVertex3f(2, -1, 1);
	glVertex3f(2, -1, -1);
	glVertex3f(2, 1, -1);
	glVertex3f(2, 1, 1);
      glEnd();
      /* Draw a white line from light direction. */
      glColor3f(1.0, 1.0, 1.0);
      glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(5, 0, 0);
      glEnd();
      glEnable(GL_CULL_FACE);
    } else {
      /* Draw a yellow ball at the light source. */
      glTranslatef(lightPosition[0], lightPosition[1], lightPosition[2]);
      glutSolidSphere(1.0, 5, 5);
    }
    glEnable(GL_LIGHTING);
    glPopMatrix();

  glPopMatrix();

  if (reportSpeed) {
    glFinish();
    end = glutGet(GLUT_ELAPSED_TIME);
    printf("Speed %.3g frames/sec (%d ms)\n", 1000.0/(end-start), end-start);
  }

  glutSwapBuffers();
}

/* ARGSUSED2 */
static void
mouse(int button, int state, int x, int y)
{
  if (button == GLUT_LEFT_BUTTON) {
    if (state == GLUT_DOWN) {
      moving = 1;
      startx = x;
      starty = y;
    }
    if (state == GLUT_UP) {
      moving = 0;
    }
  }
  if (button == GLUT_MIDDLE_BUTTON) {
    if (state == GLUT_DOWN) {
      lightMoving = 1;
      lightStartX = x;
      lightStartY = y;
    }
    if (state == GLUT_UP) {
      lightMoving = 0;
    }
  }
}

/* ARGSUSED1 */
static void
motion(int x, int y)
{
  if (moving) {
    angle = angle + (x - startx);
    angle2 = angle2 + (y - starty);
    startx = x;
    starty = y;
    glutPostRedisplay();
  }
  if (0){//lightMoving) {
    lightAngle += (x - lightStartX)/40.0;
    lightHeight += (lightStartY - y)/20.0;
    lightStartX = x;
    lightStartY = y;
    glutPostRedisplay();
  }
}

/* Advance time varying state when idle callback registered. */
static void
idle(void)
{
  static float time = 0.0;
  static float prevtime = 0.0;
  float dtime;
  prevtime = time;

  time = glutGet(GLUT_ELAPSED_TIME) / 500.0;
  dtime = time - prevtime;

  jump = 4.0 * fabs(sin(time)*0.5);
  if (!lightMoving) {
    lightAngle = time;
  }

  if (dynamicsWorld)
		plStepSimulation(dynamicsWorld,dtime);
  
  glutPostRedisplay();
}

enum {
  M_NONE, M_MOTION, M_LIGHT, M_TEXTURE, M_SHADOWS, M_REFLECTION, M_DINOSAUR,
  M_STENCIL_REFLECTION, M_STENCIL_SHADOW, M_OFFSET_SHADOW,
  M_POSITIONAL, M_DIRECTIONAL, M_PERFORMANCE
};

static void
controlLights(int value)
{
  switch (value) {
  case M_NONE:
    return;
  case M_MOTION:
    animation = 1 - animation;
    if (animation) {
      glutIdleFunc(idle);
    } else {
      glutIdleFunc(NULL);
    }
    break;
  case M_LIGHT:
    lightSwitch = !lightSwitch;
    if (lightSwitch) {
      glEnable(GL_LIGHT0);
    } else {
      glDisable(GL_LIGHT0);
    }
    break;
  case M_TEXTURE:
    useTexture = !useTexture;
    break;
  case M_SHADOWS:
    renderShadow = 1 - renderShadow;
    break;
  case M_REFLECTION:
    renderReflection = 1 - renderReflection;
    break;
  case M_DINOSAUR:
    renderDinosaur = 1 - renderDinosaur;
    break;
  case M_STENCIL_REFLECTION:
    stencilReflection = 1 - stencilReflection;
    break;
  case M_STENCIL_SHADOW:
    stencilShadow = 1 - stencilShadow;
    break;
  case M_OFFSET_SHADOW:
    offsetShadow = 1 - offsetShadow;
    break;
  case M_POSITIONAL:
    directionalLight = 0;
    break;
  case M_DIRECTIONAL:
    directionalLight = 1;
    break;
  case M_PERFORMANCE:
    reportSpeed = 1 - reportSpeed;
    break;
  }
  glutPostRedisplay();
}

/* When not visible, stop animating.  Restart when visible again. */
static void 
visible(int vis)
{
  if (vis == GLUT_VISIBLE) {
    if (animation)
      glutIdleFunc(idle);
  } else {
    if (!animation)
      glutIdleFunc(NULL);
  }
}

/* Press any key to redraw; good when motion stopped and
   performance reporting on. */
/* ARGSUSED */
static void
key(unsigned char c, int x, int y)
{
  if (c == 27) {
    exit(0);  /* IRIS GLism, Escape quits. */
  }
  glutPostRedisplay();
}

/* Press any key to redraw; good when motion stopped and
   performance reporting on. */
/* ARGSUSED */
static void
special(int k, int x, int y)
{
  glutPostRedisplay();
}

static int
supportsOneDotOne(void)
{
  const char *version;
  int major, minor;

  version = (char *) glGetString(GL_VERSION);
  if (sscanf(version, "%d.%d", &major, &minor) == 2)
    return ((major > 1) || (major >= 1 && minor >= 1));
  return 0;            /* OpenGL version string malformed! */
}


int
main(int argc, char **argv)
{
  int i;
	plCollisionShapeHandle floorShape;
	plCollisionShapeHandle dinoShape,dinoChildShape;
	plVector3 floorPos,childPos;
	plVector3 dinoPos;
	plQuaternion childOrn,dinoOrient;
	
	void* user_data=NULL;
	
	physicsSdk = plNewBulletSdk();
	dynamicsWorld = plCreateDynamicsWorld(physicsSdk);

	//create ground plane

	floorShape = plNewConvexHullShape();
	
	for (i=0;i<4;i++)
	{
//		floorVertices
		plAddVertex(floorShape,floorVertices[i][0],floorVertices[i][1],floorVertices[i][2]);
	}
	

	floorShape = plNewBoxShape(120,0,120);

	floorRigidBody = plCreateRigidBody(user_data,0.f,floorShape);
	floorPos[0] = 0;
	floorPos[1] = 0;
	floorPos[2] = 0;

	plSetPosition(floorRigidBody,floorPos);
	plAddRigidBody(dynamicsWorld,floorRigidBody);

	//create dino rigidbody
	dinoChildShape = plNewBoxShape(8.5,8.5,8.5);
	dinoShape = plNewCompoundShape();
	childPos[0] = 0;
	childPos[1] = 0;
	childPos[2] = 0;
	childOrn[0] = 0;
	childOrn[1] = 0;
	childOrn[2] = 0;
	childOrn[3] = 1;

	plAddChildShape(dinoShape,dinoChildShape,childPos,childOrn);

	dinoPos[0] = -10; dinoPos[1] = 28; dinoPos[2] = 0;
	dinoRigidBody = plCreateRigidBody(0,1.0,dinoShape);
	plSetPosition(dinoRigidBody,dinoPos);
	plSetEuler(0,0,3.15*0.20,dinoOrient);
	plSetOrientation(dinoRigidBody,dinoOrient);

	plAddRigidBody(dynamicsWorld,dinoRigidBody);
	
	  printf("BulletDino\n");
  glutInit(&argc, argv);

  for (i=1; i<argc; i++) {
    if (!strcmp("-linear", argv[i])) {
      linearFiltering = 1;
    } else if (!strcmp("-mipmap", argv[i])) {
      useMipmaps = 1;
    } else if (!strcmp("-ext", argv[i])) {
      forceExtension = 1;
    }
  }

  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_STENCIL | GLUT_MULTISAMPLE);

#if 0
  /* In GLUT 4.0, you'll be able to do this an be sure to
     get 2 bits of stencil if the machine has it for you. */
  glutInitDisplayString("samples stencil>=2 rgb double depth");
#endif

  glutCreateWindow("Shadowy Leapin' Lizards");

  if (glutGet(GLUT_WINDOW_STENCIL_SIZE) <= 1) {
    printf("dinoshade: Sorry, I need at least 2 bits of stencil.\n");
    exit(1);
  }

  /* Register GLUT callbacks. */
  glutDisplayFunc(redraw);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  glutVisibilityFunc(visible);
  glutKeyboardFunc(key);
  glutSpecialFunc(special);

  glutCreateMenu(controlLights);

  glutAddMenuEntry("Toggle motion", M_MOTION);
  glutAddMenuEntry("-----------------------", M_NONE);
  glutAddMenuEntry("Toggle light", M_LIGHT);
  glutAddMenuEntry("Toggle texture", M_TEXTURE);
  glutAddMenuEntry("Toggle shadows", M_SHADOWS);
  glutAddMenuEntry("Toggle reflection", M_REFLECTION);
  glutAddMenuEntry("Toggle dinosaur", M_DINOSAUR);
  glutAddMenuEntry("-----------------------", M_NONE);
  glutAddMenuEntry("Toggle reflection stenciling", M_STENCIL_REFLECTION);
  glutAddMenuEntry("Toggle shadow stenciling", M_STENCIL_SHADOW);
  glutAddMenuEntry("Toggle shadow offset", M_OFFSET_SHADOW);
  glutAddMenuEntry("----------------------", M_NONE);
  glutAddMenuEntry("Positional light", M_POSITIONAL);
  glutAddMenuEntry("Directional light", M_DIRECTIONAL);
  glutAddMenuEntry("-----------------------", M_NONE);
  glutAddMenuEntry("Toggle performance", M_PERFORMANCE);
  glutAttachMenu(GLUT_RIGHT_BUTTON);
  makeDinosaur();

#ifdef GL_VERSION_1_1
  if (supportsOneDotOne() && !forceExtension) {
    polygonOffsetVersion = ONE_DOT_ONE;
    glPolygonOffset(-2.0, -1.0);
  } else
#endif
  {
    {
      polygonOffsetVersion = MISSING;
      printf("\ndinoshine: Missing polygon offset.\n");
      printf("           Expect shadow depth aliasing artifacts.\n\n");
    }
  }

  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_TEXTURE_2D);
  glLineWidth(3.0);

  glMatrixMode(GL_PROJECTION);
  gluPerspective( /* field of view in degree */ 40.0,
  /* aspect ratio */ 1.0,
    /* Z near */ 20.0, /* Z far */ 100.0);
  glMatrixMode(GL_MODELVIEW);
  gluLookAt(0.0, 8.0, 60.0,  /* eye is at (0,0,30) */
    0.0, 8.0, 0.0,      /* center is at (0,0,0) */
    0.0, 1.0, 0.);      /* up is in postivie Y direction */

  glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, 1);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);
  glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 0.1);
  glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.05);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHTING);

  makeFloorTexture();

  /* Setup floor plane for projected shadow calculations. */
  findPlane(floorPlane, floorVertices[1], floorVertices[2], floorVertices[3]);

  glutMainLoop();

  plDeleteDynamicsWorld(dynamicsWorld);
  plDeletePhysicsSdk(physicsSdk);


  return 0;             /* ANSI C requires main to return int. */
}
