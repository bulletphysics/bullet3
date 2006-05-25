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

simple graphics.

the following command line flags can be used (typically under unix)
	-notex		Do not use any textures
	-noshadow[s]	Do not draw any shadows
	-pause		Start the simulation paused

TODO
----

manage openGL state changes better

*/

#ifdef WIN32
#include <windows.h>
#endif

#include <ode/config.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "drawstuff/drawstuff.h"
#include "internal.h"

//***************************************************************************
// misc

#ifdef WIN32
#define DEFAULT_PATH_TO_TEXTURES "..\\textures\\"
#else
#define DEFAULT_PATH_TO_TEXTURES "../textures/"
#endif

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

// constants to convert degrees to radians and the reverse
#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD (M_PI/180.0)

// light vector. LIGHTZ is implicitly 1
#define LIGHTX (1.0f)
#define LIGHTY (0.4f)

// ground and sky
#define SHADOW_INTENSITY (0.65f)
#define GROUND_R (0.5f) 	// ground color for when there's no texture
#define GROUND_G (0.5f)
#define GROUND_B (0.3f)

const float ground_scale = 1.0f/1.0f;	// ground texture scale (1/size)
const float ground_ofsx = 0.5;		// offset of ground texture
const float ground_ofsy = 0.5;
const float sky_scale = 1.0f/4.0f;	// sky texture scale (1/size)
const float sky_height = 1.0f;		// sky height above viewpoint

//***************************************************************************
// misc mathematics stuff

#define dCROSS(a,op,b,c) \
  (a)[0] op ((b)[1]*(c)[2] - (b)[2]*(c)[1]); \
  (a)[1] op ((b)[2]*(c)[0] - (b)[0]*(c)[2]); \
  (a)[2] op ((b)[0]*(c)[1] - (b)[1]*(c)[0]);


inline float dDOT (const float *a, const float *b)
  { return ((a)[0]*(b)[0] + (a)[1]*(b)[1] + (a)[2]*(b)[2]); }


static void normalizeVector3 (float v[3])
{
  float len = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
  if (len <= 0.0f) {
    v[0] = 1;
    v[1] = 0;
    v[2] = 0;
  }
  else {
    len = 1.0f / (float)sqrt(len);
    v[0] *= len;
    v[1] *= len;
    v[2] *= len;
  }
}

//***************************************************************************
// PPM image object

typedef unsigned char byte;

class Image {
  int image_width,image_height;
  byte *image_data;
public:
  Image (char *filename);
  // load from PPM file
  ~Image();
  int width() { return image_width; }
  int height() { return image_height; }
  byte *data() { return image_data; }
};


// skip over whitespace and comments in a stream.

static void skipWhiteSpace (char *filename, FILE *f)
{
  int c,d;
  for(;;) {
    c = fgetc(f);
    if (c==EOF) dsError ("unexpected end of file in \"%s\"",filename);

    // skip comments
    if (c == '#') {
      do {
	d = fgetc(f);
	if (d==EOF) dsError ("unexpected end of file in \"%s\"",filename);
      } while (d != '\n');
      continue;
    }

    if (c > ' ') {
      ungetc (c,f);
      return;
    }
  }
}


// read a number from a stream, this return 0 if there is none (that's okay
// because 0 is a bad value for all PPM numbers anyway).

static int readNumber (char *filename, FILE *f)
{
  int c,n=0;
  for(;;) {
    c = fgetc(f);
    if (c==EOF) dsError ("unexpected end of file in \"%s\"",filename);
    if (c >= '0' && c <= '9') n = n*10 + (c - '0');
    else {
      ungetc (c,f);
      return n;
    }
  }
}


Image::Image (char *filename)
{
  FILE *f = fopen (filename,"rb");
  if (!f) dsError ("Can't open image file `%s'",filename);

  // read in header
  if (fgetc(f) != 'P' || fgetc(f) != '6')
    dsError ("image file \"%s\" is not a binary PPM (no P6 header)",filename);
  skipWhiteSpace (filename,f);

  // read in image parameters
  image_width = readNumber (filename,f);
  skipWhiteSpace (filename,f);
  image_height = readNumber (filename,f);
  skipWhiteSpace (filename,f);
  int max_value = readNumber (filename,f);

  // check values
  if (image_width < 1 || image_height < 1)
    dsError ("bad image file \"%s\"",filename);
  if (max_value != 255)
    dsError ("image file \"%s\" must have color range of 255",filename);

  // read either nothing, LF (10), or CR,LF (13,10)
  int c = fgetc(f);
  if (c == 10) {
    // LF
  }
  else if (c == 13) {
    // CR
    c = fgetc(f);
    if (c != 10) ungetc (c,f);
  }
  else ungetc (c,f);

  // read in rest of data
  image_data = new byte [image_width*image_height*3];
  if (fread (image_data,image_width*image_height*3,1,f) != 1)
    dsError ("Can not read data from image file `%s'",filename);
  fclose (f);
}


Image::~Image()
{
  delete[] image_data;
}

//***************************************************************************
// Texture object.

class Texture {
  Image *image;
  GLuint name;
public:
  Texture (char *filename);
  ~Texture();
  void bind (int modulate);
};


Texture::Texture (char *filename)
{
  image = new Image (filename);
  glGenTextures (1,&name);
  glBindTexture (GL_TEXTURE_2D,name);

  // set pixel unpacking mode
  glPixelStorei (GL_UNPACK_SWAP_BYTES, 0);
  glPixelStorei (GL_UNPACK_ROW_LENGTH, 0);
  glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei (GL_UNPACK_SKIP_ROWS, 0);
  glPixelStorei (GL_UNPACK_SKIP_PIXELS, 0);

  // glTexImage2D (GL_TEXTURE_2D, 0, 3, image->width(), image->height(), 0,
  //		   GL_RGB, GL_UNSIGNED_BYTE, image->data());
  gluBuild2DMipmaps (GL_TEXTURE_2D, 3, image->width(), image->height(),
		     GL_RGB, GL_UNSIGNED_BYTE, image->data());

  // set texture parameters - will these also be bound to the texture???
  glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

  glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
		   GL_LINEAR_MIPMAP_LINEAR);

  glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
}


Texture::~Texture()
{
  delete image;
  glDeleteTextures (1,&name);
}


void Texture::bind (int modulate)
{
  glBindTexture (GL_TEXTURE_2D,name);
  glTexEnvi (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE,
	     modulate ? GL_MODULATE : GL_DECAL);
}

//***************************************************************************
// the current drawing state (for when the user's step function is drawing)

static float color[4] = {0,0,0,0};	// current r,g,b,alpha color
static int tnum = 0;			// current texture number

//***************************************************************************
// OpenGL utility stuff

static void setCamera (float x, float y, float z, float h, float p, float r)
{
  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity();
  glRotatef (90, 0,0,1);
  glRotatef (90, 0,1,0);
  glRotatef (r, 1,0,0);
  glRotatef (p, 0,1,0);
  glRotatef (-h, 0,0,1);
  glTranslatef (-x,-y,-z);
}


// sets the material color, not the light color

static void setColor (float r, float g, float b, float alpha)
{
  GLfloat light_ambient[4],light_diffuse[4],light_specular[4];
  light_ambient[0] = r*0.3f;
  light_ambient[1] = g*0.3f;
  light_ambient[2] = b*0.3f;
  light_ambient[3] = alpha;
  light_diffuse[0] = r*0.7f;
  light_diffuse[1] = g*0.7f;
  light_diffuse[2] = b*0.7f;
  light_diffuse[3] = alpha;
  light_specular[0] = r*0.2f;
  light_specular[1] = g*0.2f;
  light_specular[2] = b*0.2f;
  light_specular[3] = alpha;
  glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, light_ambient);
  glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, light_diffuse);
  glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, light_specular);
  glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 5.0f);
}


static void setTransform (const float pos[3], const float R[12])
{
  GLfloat matrix[16];
  matrix[0]=R[0];
  matrix[1]=R[4];
  matrix[2]=R[8];
  matrix[3]=0;
  matrix[4]=R[1];
  matrix[5]=R[5];
  matrix[6]=R[9];
  matrix[7]=0;
  matrix[8]=R[2];
  matrix[9]=R[6];
  matrix[10]=R[10];
  matrix[11]=0;
  matrix[12]=pos[0];
  matrix[13]=pos[1];
  matrix[14]=pos[2];
  matrix[15]=1;
  glPushMatrix();
  glMultMatrixf (matrix);
}


// set shadow projection transform

static void setShadowTransform()
{
  GLfloat matrix[16];
  for (int i=0; i<16; i++) matrix[i] = 0;
  matrix[0]=1;
  matrix[5]=1;
  matrix[8]=-LIGHTX;
  matrix[9]=-LIGHTY;
  matrix[15]=1;
  glPushMatrix();
  glMultMatrixf (matrix);
}


static void drawBox (const float sides[3])
{
  float lx = sides[0]*0.5f;
  float ly = sides[1]*0.5f;
  float lz = sides[2]*0.5f;

  // sides
  glBegin (GL_TRIANGLE_STRIP);
  glNormal3f (-1,0,0);
  glVertex3f (-lx,-ly,-lz);
  glVertex3f (-lx,-ly,lz);
  glVertex3f (-lx,ly,-lz);
  glVertex3f (-lx,ly,lz);
  glNormal3f (0,1,0);
  glVertex3f (lx,ly,-lz);
  glVertex3f (lx,ly,lz);
  glNormal3f (1,0,0);
  glVertex3f (lx,-ly,-lz);
  glVertex3f (lx,-ly,lz);
  glNormal3f (0,-1,0);
  glVertex3f (-lx,-ly,-lz);
  glVertex3f (-lx,-ly,lz);
  glEnd();

  // top face
  glBegin (GL_TRIANGLE_FAN);
  glNormal3f (0,0,1);
  glVertex3f (-lx,-ly,lz);
  glVertex3f (lx,-ly,lz);
  glVertex3f (lx,ly,lz);
  glVertex3f (-lx,ly,lz);
  glEnd();

  // bottom face
  glBegin (GL_TRIANGLE_FAN);
  glNormal3f (0,0,-1);
  glVertex3f (-lx,-ly,-lz);
  glVertex3f (-lx,ly,-lz);
  glVertex3f (lx,ly,-lz);
  glVertex3f (lx,-ly,-lz);
  glEnd();
}


// This is recursively subdivides a triangular area (vertices p1,p2,p3) into
// smaller triangles, and then draws the triangles. All triangle vertices are
// normalized to a distance of 1.0 from the origin (p1,p2,p3 are assumed
// to be already normalized). Note this is not super-fast because it draws
// triangles rather than triangle strips.

static void drawPatch (float p1[3], float p2[3], float p3[3], int level)
{
  int i;
  if (level > 0) {
    float q1[3],q2[3],q3[3];		 // sub-vertices
    for (i=0; i<3; i++) {
      q1[i] = 0.5f*(p1[i]+p2[i]);
      q2[i] = 0.5f*(p2[i]+p3[i]);
      q3[i] = 0.5f*(p3[i]+p1[i]);
    }
    float length1 = (float)(1.0/sqrt(q1[0]*q1[0]+q1[1]*q1[1]+q1[2]*q1[2]));
    float length2 = (float)(1.0/sqrt(q2[0]*q2[0]+q2[1]*q2[1]+q2[2]*q2[2]));
    float length3 = (float)(1.0/sqrt(q3[0]*q3[0]+q3[1]*q3[1]+q3[2]*q3[2]));
    for (i=0; i<3; i++) {
      q1[i] *= length1;
      q2[i] *= length2;
      q3[i] *= length3;
    }
    drawPatch (p1,q1,q3,level-1);
    drawPatch (q1,p2,q2,level-1);
    drawPatch (q1,q2,q3,level-1);
    drawPatch (q3,q2,p3,level-1);
  }
  else {
    glNormal3f (p1[0],p1[1],p1[2]);
    glVertex3f (p1[0],p1[1],p1[2]);
    glNormal3f (p2[0],p2[1],p2[2]);
    glVertex3f (p2[0],p2[1],p2[2]);
    glNormal3f (p3[0],p3[1],p3[2]);
    glVertex3f (p3[0],p3[1],p3[2]);
  }
}


// draw a sphere of radius 1

static int sphere_quality = 1;

static void drawSphere()
{
  // icosahedron data for an icosahedron of radius 1.0
# define ICX 0.525731112119133606f
# define ICZ 0.850650808352039932f
  static GLfloat idata[12][3] = {
    {-ICX, 0, ICZ},
    {ICX, 0, ICZ},
    {-ICX, 0, -ICZ},
    {ICX, 0, -ICZ},
    {0, ICZ, ICX},
    {0, ICZ, -ICX},
    {0, -ICZ, ICX},
    {0, -ICZ, -ICX},
    {ICZ, ICX, 0},
    {-ICZ, ICX, 0},
    {ICZ, -ICX, 0},
    {-ICZ, -ICX, 0}
  };

  static int index[20][3] = {
    {0, 4, 1},	  {0, 9, 4},
    {9, 5, 4},	  {4, 5, 8},
    {4, 8, 1},	  {8, 10, 1},
    {8, 3, 10},   {5, 3, 8},
    {5, 2, 3},	  {2, 7, 3},
    {7, 10, 3},   {7, 6, 10},
    {7, 11, 6},   {11, 0, 6},
    {0, 1, 6},	  {6, 1, 10},
    {9, 0, 11},   {9, 11, 2},
    {9, 2, 5},	  {7, 2, 11},
  };

  static GLuint listnum = 0;
  if (listnum==0) {
    listnum = glGenLists (1);
    glNewList (listnum,GL_COMPILE);
    glBegin (GL_TRIANGLES);
    for (int i=0; i<20; i++) {
      drawPatch (&idata[index[i][2]][0],
		  &idata[index[i][1]][0],
		 &idata[index[i][0]][0],
		 sphere_quality);
    }
    glEnd();
    glEndList();
  }
  glCallList (listnum);
}


static void drawSphereShadow (float px, float py, float pz, float radius)
{
  // calculate shadow constants based on light vector
  static int init=0;
  static float len2,len1,scale;
  if (!init) {
    len2 = LIGHTX*LIGHTX + LIGHTY*LIGHTY;
    len1 = 1.0f/(float)sqrt(len2);
    scale = (float) sqrt(len2 + 1);
    init = 1;
  }

  // map sphere center to ground plane based on light vector
  px -= LIGHTX*pz;
  py -= LIGHTY*pz;

  const float kx = 0.96592582628907f;
  const float ky = 0.25881904510252f;
  float x=radius, y=0;

  glBegin (GL_TRIANGLE_FAN);
  for (int i=0; i<24; i++) {
    // for all points on circle, scale to elongated rotated shadow and draw
    float x2 = (LIGHTX*x*scale - LIGHTY*y)*len1 + px;
    float y2 = (LIGHTY*x*scale + LIGHTX*y)*len1 + py;
    glTexCoord2f (x2*ground_scale+ground_ofsx,y2*ground_scale+ground_ofsy);
    glVertex3f (x2,y2,0);

    // rotate [x,y] vector
    float xtmp = kx*x - ky*y;
    y = ky*x + kx*y;
    x = xtmp;
  }
  glEnd();
}


static void drawTriangle (const float *v0, const float *v1, const float *v2, int solid)
{
  float u[3],v[3],normal[3];
  u[0] = v1[0] - v0[0];
  u[1] = v1[1] - v0[1];
  u[2] = v1[2] - v0[2];
  v[0] = v2[0] - v0[0];
  v[1] = v2[1] - v0[1];
  v[2] = v2[2] - v0[2];
  dCROSS (normal,=,u,v);
  normalizeVector3 (normal);

  glBegin(solid ? GL_TRIANGLES : GL_LINE_STRIP);
  glNormal3fv (normal);
  glVertex3fv (v0);
  glVertex3fv (v1);
  glVertex3fv (v2);
  glEnd();
}

static void drawTriangleD (const double *v0, const double *v1, const double *v2, int solid)
{
  float u[3],v[3],normal[3];
  u[0] = float( v1[0] - v0[0] );
  u[1] = float( v1[1] - v0[1] );
  u[2] = float( v1[2] - v0[2] );
  v[0] = float( v2[0] - v0[0] );
  v[1] = float( v2[1] - v0[1] );
  v[2] = float( v2[2] - v0[2] );
  dCROSS (normal,=,u,v);
  normalizeVector3 (normal);

  glBegin(solid ? GL_TRIANGLES : GL_LINE_STRIP);
  glNormal3fv (normal);
  glVertex3dv (v0);
  glVertex3dv (v1);
  glVertex3dv (v2);
  glEnd();
}


// draw a capped cylinder of length l and radius r, aligned along the x axis

static int capped_cylinder_quality = 3;

static void drawCappedCylinder (float l, float r)
{
  int i,j;
  float tmp,nx,ny,nz,start_nx,start_ny,a,ca,sa;
  // number of sides to the cylinder (divisible by 4):
  const int n = capped_cylinder_quality*4;

  l *= 0.5;
  a = float(M_PI*2.0)/float(n);
  sa = (float) sin(a);
  ca = (float) cos(a);

  // draw cylinder body
  ny=1; nz=0;		  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_STRIP);
  for (i=0; i<=n; i++) {
    glNormal3d (ny,nz,0);
    glVertex3d (ny*r,nz*r,l);
    glNormal3d (ny,nz,0);
    glVertex3d (ny*r,nz*r,-l);
    // rotate ny,nz
    tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  // draw first cylinder cap
  start_nx = 0;
  start_ny = 1;
  for (j=0; j<(n/4); j++) {
    // get start_n2 = rotated start_n
    float start_nx2 =  ca*start_nx + sa*start_ny;
    float start_ny2 = -sa*start_nx + ca*start_ny;
    // get n=start_n and n2=start_n2
    nx = start_nx; ny = start_ny; nz = 0;
    float nx2 = start_nx2, ny2 = start_ny2, nz2 = 0;
    glBegin (GL_TRIANGLE_STRIP);
    for (i=0; i<=n; i++) {
      glNormal3d (ny2,nz2,nx2);
      glVertex3d (ny2*r,nz2*r,l+nx2*r);
      glNormal3d (ny,nz,nx);
      glVertex3d (ny*r,nz*r,l+nx*r);
      // rotate n,n2
      tmp = ca*ny - sa*nz;
      nz = sa*ny + ca*nz;
      ny = tmp;
      tmp = ca*ny2- sa*nz2;
      nz2 = sa*ny2 + ca*nz2;
      ny2 = tmp;
    }
    glEnd();
    start_nx = start_nx2;
    start_ny = start_ny2;
  }

  // draw second cylinder cap
  start_nx = 0;
  start_ny = 1;
  for (j=0; j<(n/4); j++) {
    // get start_n2 = rotated start_n
    float start_nx2 = ca*start_nx - sa*start_ny;
    float start_ny2 = sa*start_nx + ca*start_ny;
    // get n=start_n and n2=start_n2
    nx = start_nx; ny = start_ny; nz = 0;
    float nx2 = start_nx2, ny2 = start_ny2, nz2 = 0;
    glBegin (GL_TRIANGLE_STRIP);
    for (i=0; i<=n; i++) {
      glNormal3d (ny,nz,nx);
      glVertex3d (ny*r,nz*r,-l+nx*r);
      glNormal3d (ny2,nz2,nx2);
      glVertex3d (ny2*r,nz2*r,-l+nx2*r);
      // rotate n,n2
      tmp = ca*ny - sa*nz;
      nz = sa*ny + ca*nz;
      ny = tmp;
      tmp = ca*ny2- sa*nz2;
      nz2 = sa*ny2 + ca*nz2;
      ny2 = tmp;
    }
    glEnd();
    start_nx = start_nx2;
    start_ny = start_ny2;
  }

  glPopMatrix();
}

// draw a cylinder of length l and radius r, aligned along the z axis

static void drawCylinder2 (float l, float radius1, float radius2,float zoffset)
{
  int i;
  float tmp,ny,nz,a,ca,sa;
  const int n = 24;	// number of sides to the cylinder (divisible by 4)

  l *= 0.5;
  a = float(M_PI*2.0)/float(n);
  sa = (float) sin(a);
  ca = (float) cos(a);

  // draw cylinder body
  ny=1; nz=0;		  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_STRIP);
  for (i=0; i<=n; i++) {
    glNormal3d (ny,nz,0);
    glVertex3d (ny*radius1,nz*radius1,l+zoffset);
    glNormal3d (ny,nz,0);
    glVertex3d (ny*radius2,nz*radius2,-l+zoffset);
    // rotate ny,nz
    tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  // draw top cap
  glShadeModel (GL_FLAT);
  ny=1; nz=0;		  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_FAN);
  glNormal3d (0,0,1);
  glVertex3d (0,0,l+zoffset);
  for (i=0; i<=n; i++) {
    if (i==1 || i==n/2+1)
      setColor (color[0]*0.75f,color[1]*0.75f,color[2]*0.75f,color[3]);
    glNormal3d (0,0,1);
    glVertex3d (ny*radius1,nz*radius1,l+zoffset);
    if (i==1 || i==n/2+1)
      setColor (color[0],color[1],color[2],color[3]);

    // rotate ny,nz
    tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  // draw bottom cap
  ny=1; nz=0;		  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_FAN);
  glNormal3d (0,0,-1);
  glVertex3d (0,0,-l+zoffset);
  for (i=0; i<=n; i++) {
    if (i==1 || i==n/2+1)
      setColor (color[0]*0.75f,color[1]*0.75f,color[2]*0.75f,color[3]);
    glNormal3d (0,0,-1);
    glVertex3d (ny*radius2,nz*radius2,-l+zoffset);
    if (i==1 || i==n/2+1)
      setColor (color[0],color[1],color[2],color[3]);

    // rotate ny,nz
    tmp = ca*ny + sa*nz;
    nz = -sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();
}

static void drawCone(float l, float r, float zoffset)
{
	drawCylinder2(l,0,r,zoffset);
}



// draw a cylinder of length l and radius r, aligned along the z axis

static void drawCylinder (float l, float r, float zoffset)
{
	drawCylinder2(l,r,r,zoffset);

}

//***************************************************************************
// motion model

// current camera position and orientation
static float view_xyz[3];	// position x,y,z
static float view_hpr[3];	// heading, pitch, roll (degrees)


// initialize the above variables

static void initMotionModel()
{
  view_xyz[0] = 2;
  view_xyz[1] = 0;
  view_xyz[2] = 1;
  view_hpr[0] = 180;
  view_hpr[1] = 0;
  view_hpr[2] = 0;
}


static void wrapCameraAngles()
{
  for (int i=0; i<3; i++) {
    while (view_hpr[i] > 180) view_hpr[i] -= 360;
    while (view_hpr[i] < -180) view_hpr[i] += 360;
  }
}


// call this to update the current camera position. the bits in `mode' say
// if the left (1), middle (2) or right (4) mouse button is pressed, and
// (deltax,deltay) is the amount by which the mouse pointer has moved.

void dsMotion (int mode, int deltax, int deltay)
{
  float side = 0.01f * float(deltax);
  float fwd = (mode==4) ? (0.01f * float(deltay)) : 0.0f;
  float s = (float) sin (view_hpr[0]*DEG_TO_RAD);
  float c = (float) cos (view_hpr[0]*DEG_TO_RAD);

  if (mode==1) {
    view_hpr[0] += float (deltax) * 0.5f;
    view_hpr[1] += float (deltay) * 0.5f;
  }
  else {
    view_xyz[0] += -s*side + c*fwd;
    view_xyz[1] += c*side + s*fwd;
    if (mode==2 || mode==5) view_xyz[2] += 0.01f * float(deltay);
  }
  wrapCameraAngles();
}

//***************************************************************************
// drawing loop stuff

// the current state:
//    0 = uninitialized
//    1 = dsSimulationLoop() called
//    2 = dsDrawFrame() called
static int current_state = 0;

// textures and shadows
static int use_textures=1;		// 1 if textures to be drawn
static int use_shadows=1;		// 1 if shadows to be drawn
static Texture *sky_texture = 0;
static Texture *ground_texture = 0;
static Texture *wood_texture = 0;


#ifndef macintosh

void dsStartGraphics (int width, int height, dsFunctions *fn)
{
  char *prefix = DEFAULT_PATH_TO_TEXTURES;
  if (fn->version >= 2 && fn->path_to_textures) prefix = fn->path_to_textures;
  char *s = (char*) alloca (strlen(prefix) + 20);

  strcpy (s,prefix);
  strcat (s,"/sky.ppm");
  sky_texture = new Texture (s);

  strcpy (s,prefix);
  strcat (s,"/ground.ppm");
  ground_texture = new Texture (s);

  strcpy (s,prefix);
  strcat (s,"/wood.ppm");
  wood_texture = new Texture (s);
}

#else // macintosh

void dsStartGraphics (int width, int height, dsFunctions *fn)
{
   // All examples build into the same dir
   char *prefix = "::::drawstuff:textures";
   char *s = (char*) alloca (strlen(prefix) + 20);

   strcpy (s,prefix);
   strcat (s,":sky.ppm");
   sky_texture = new Texture (s);

   strcpy (s,prefix);
   strcat (s,":ground.ppm");
   ground_texture = new Texture (s);

   strcpy (s,prefix);
   strcat (s,":wood.ppm");
   wood_texture = new Texture (s);
}

#endif


void dsStopGraphics()
{
  delete sky_texture;
  delete ground_texture;
  delete wood_texture;
  sky_texture = 0;
  ground_texture = 0;
  wood_texture = 0;
}


static void drawSky (float view_xyz[3])
{
  glDisable (GL_LIGHTING);
  if (use_textures) {
    glEnable (GL_TEXTURE_2D);
    sky_texture->bind (0);
  }
  else {
    glDisable (GL_TEXTURE_2D);
    glColor3f (0,0.5,1.0);
  }

  // make sure sky depth is as far back as possible
  glShadeModel (GL_FLAT);
  glEnable (GL_DEPTH_TEST);
  glDepthFunc (GL_LEQUAL);
  glDepthRange (1,1);

  const float ssize = 1000.0f;
  static float offset = 0.0f;

  float x = ssize*sky_scale;
  float z = view_xyz[2] + sky_height;

  glBegin (GL_QUADS);
  glNormal3f (0,0,-1);
  glTexCoord2f (-x+offset,-x+offset);
  glVertex3f (-ssize+view_xyz[0],-ssize+view_xyz[1],z);
  glTexCoord2f (-x+offset,x+offset);
  glVertex3f (-ssize+view_xyz[0],ssize+view_xyz[1],z);
  glTexCoord2f (x+offset,x+offset);
  glVertex3f (ssize+view_xyz[0],ssize+view_xyz[1],z);
  glTexCoord2f (x+offset,-x+offset);
  glVertex3f (ssize+view_xyz[0],-ssize+view_xyz[1],z);
  glEnd();

  offset = offset + 0.002f;
  if (offset > 1) offset -= 1;

  glDepthFunc (GL_LESS);
  glDepthRange (0,1);
}


static void drawGround()
{
  glDisable (GL_LIGHTING);
  glShadeModel (GL_FLAT);
  glEnable (GL_DEPTH_TEST);
  glDepthFunc (GL_LESS);
  // glDepthRange (1,1);

  if (use_textures) {
    glEnable (GL_TEXTURE_2D);
    ground_texture->bind (0);
  }
  else {
    glDisable (GL_TEXTURE_2D);
    glColor3f (GROUND_R,GROUND_G,GROUND_B);
  }

  // ground fog seems to cause problems with TNT2 under windows
  /*
  GLfloat fogColor[4] = {0.5, 0.5, 0.5, 1};
  glEnable (GL_FOG);
  glFogi (GL_FOG_MODE, GL_EXP2);
  glFogfv (GL_FOG_COLOR, fogColor);
  glFogf (GL_FOG_DENSITY, 0.05f);
  glHint (GL_FOG_HINT, GL_NICEST); // GL_DONT_CARE);
  glFogf (GL_FOG_START, 1.0);
  glFogf (GL_FOG_END, 5.0);
  */

  const float gsize = 100.0f;
  const float offset = 0; // -0.001f; ... polygon offsetting doesn't work well

  glBegin (GL_QUADS);
  glNormal3f (0,0,1);
  glTexCoord2f (-gsize*ground_scale + ground_ofsx,
		-gsize*ground_scale + ground_ofsy);
  glVertex3f (-gsize,-gsize,offset);
  glTexCoord2f (gsize*ground_scale + ground_ofsx,
		-gsize*ground_scale + ground_ofsy);
  glVertex3f (gsize,-gsize,offset);
  glTexCoord2f (gsize*ground_scale + ground_ofsx,
		gsize*ground_scale + ground_ofsy);
  glVertex3f (gsize,gsize,offset);
  glTexCoord2f (-gsize*ground_scale + ground_ofsx,
		gsize*ground_scale + ground_ofsy);
  glVertex3f (-gsize,gsize,offset);
  glEnd();

  glDisable (GL_FOG);
}


static void drawPyramidGrid()
{
  // setup stuff
  glEnable (GL_LIGHTING);
  glDisable (GL_TEXTURE_2D);
  glShadeModel (GL_FLAT);
  glEnable (GL_DEPTH_TEST);
  glDepthFunc (GL_LESS);

  // draw the pyramid grid
  for (int i=-1; i<=1; i++) {
    for (int j=-1; j<=1; j++) {
      glPushMatrix();
      glTranslatef ((float)i,(float)j,(float)0);
      if (i==1 && j==0) setColor (1,0,0,1);
      else if (i==0 && j==1) setColor (0,0,1,1);
      else setColor (1,1,0,1);
      const float k = 0.03f;
      glBegin (GL_TRIANGLE_FAN);
      glNormal3f (0,-1,1);
      glVertex3f (0,0,k);
      glVertex3f (-k,-k,0);
      glVertex3f ( k,-k,0);
      glNormal3f (1,0,1);
      glVertex3f ( k, k,0);
      glNormal3f (0,1,1);
      glVertex3f (-k, k,0);
      glNormal3f (-1,0,1);
      glVertex3f (-k,-k,0);
      glEnd();
      glPopMatrix();
    }
  }
}


void dsDrawFrame (int width, int height, dsFunctions *fn, int pause)
{
  if (current_state < 1) dsDebug ("internal error");
  current_state = 2;

  // setup stuff
  glEnable (GL_LIGHTING);
  glEnable (GL_LIGHT0);
  glDisable (GL_TEXTURE_2D);
  glDisable (GL_TEXTURE_GEN_S);
  glDisable (GL_TEXTURE_GEN_T);
  glShadeModel (GL_FLAT);
  glEnable (GL_DEPTH_TEST);
  glDepthFunc (GL_LESS);
  glEnable (GL_CULL_FACE);
  glCullFace (GL_BACK);
  glFrontFace (GL_CCW);

  // setup viewport
  glViewport (0,0,width,height);
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity();
  const float vnear = 0.1f;
  const float vfar = 100.0f;
  const float k = 0.8f;     // view scale, 1 = +/- 45 degrees
  if (width >= height) {
    float k2 = float(height)/float(width);
    glFrustum (-vnear*k,vnear*k,-vnear*k*k2,vnear*k*k2,vnear,vfar);
  }
  else {
    float k2 = float(width)/float(height);
    glFrustum (-vnear*k*k2,vnear*k*k2,-vnear*k,vnear*k,vnear,vfar);
  }

  // setup lights. it makes a difference whether this is done in the
  // GL_PROJECTION matrix mode (lights are scene relative) or the
  // GL_MODELVIEW matrix mode (lights are camera relative, bad!).
  static GLfloat light_ambient[] = { 0.5, 0.5, 0.5, 1.0 };
  static GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
  static GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
  glLightfv (GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv (GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv (GL_LIGHT0, GL_SPECULAR, light_specular);
  glColor3f (1.0, 1.0, 1.0);

  // clear the window
  glClearColor (0.5,0.5,0.5,0);
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // snapshot camera position (in MS Windows it is changed by the GUI thread)
  float view2_xyz[3];
  float view2_hpr[3];
  memcpy (view2_xyz,view_xyz,sizeof(float)*3);
  memcpy (view2_hpr,view_hpr,sizeof(float)*3);

  // go to GL_MODELVIEW matrix mode and set the camera
  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity();
  setCamera (view2_xyz[0],view2_xyz[1],view2_xyz[2],
	     view2_hpr[0],view2_hpr[1],view2_hpr[2]);

  // set the light position (for some reason we have to do this in model view.
  static GLfloat light_position[] = { LIGHTX, LIGHTY, 1.0, 0.0 };
  glLightfv (GL_LIGHT0, GL_POSITION, light_position);

  // draw the background (ground, sky etc)
  drawSky (view2_xyz);
  drawGround();

  // draw the little markers on the ground
  drawPyramidGrid();

  // leave openGL in a known state - flat shaded white, no textures
  glEnable (GL_LIGHTING);
  glDisable (GL_TEXTURE_2D);
  glShadeModel (GL_FLAT);
  glEnable (GL_DEPTH_TEST);
  glDepthFunc (GL_LESS);
  glColor3f (1,1,1);
  setColor (1,1,1,1);

  // draw the rest of the objects. set drawing state first.
  color[0] = 1;
  color[1] = 1;
  color[2] = 1;
  color[3] = 1;
  tnum = 0;
  if (fn->step) fn->step (pause);
}


int dsGetShadows()
{
  return use_shadows;
}


void dsSetShadows (int a)
{
  use_shadows = (a != 0);
}


int dsGetTextures()
{
  return use_textures;
}


void dsSetTextures (int a)
{
  use_textures = (a != 0);
}

//***************************************************************************
// C interface

// sets lighting and texture modes, sets current color
static void setupDrawingMode()
{
  glEnable (GL_LIGHTING);
  if (tnum) {
    if (use_textures) {
      glEnable (GL_TEXTURE_2D);
      wood_texture->bind (1);
      glEnable (GL_TEXTURE_GEN_S);
      glEnable (GL_TEXTURE_GEN_T);
      glTexGeni (GL_S,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
      glTexGeni (GL_T,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
      static GLfloat s_params[4] = {1.0f,1.0f,0.0f,1};
      static GLfloat t_params[4] = {0.817f,-0.817f,0.817f,1};
      glTexGenfv (GL_S,GL_OBJECT_PLANE,s_params);
      glTexGenfv (GL_T,GL_OBJECT_PLANE,t_params);
    }
    else {
      glDisable (GL_TEXTURE_2D);
    }
  }
  else {
    glDisable (GL_TEXTURE_2D);
  }
  setColor (color[0],color[1],color[2],color[3]);

  if (color[3] < 1) {
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
  }
  else {
    glDisable (GL_BLEND);
  }
}


static void setShadowDrawingMode()
{
  glDisable (GL_LIGHTING);
  if (use_textures) {
    glEnable (GL_TEXTURE_2D);
    ground_texture->bind (1);
    glColor3f (SHADOW_INTENSITY,SHADOW_INTENSITY,SHADOW_INTENSITY);
    glEnable (GL_TEXTURE_2D);
    glEnable (GL_TEXTURE_GEN_S);
    glEnable (GL_TEXTURE_GEN_T);
    glTexGeni (GL_S,GL_TEXTURE_GEN_MODE,GL_EYE_LINEAR);
    glTexGeni (GL_T,GL_TEXTURE_GEN_MODE,GL_EYE_LINEAR);
    static GLfloat s_params[4] = {ground_scale,0,0,ground_ofsx};
    static GLfloat t_params[4] = {0,ground_scale,0,ground_ofsy};
    glTexGenfv (GL_S,GL_EYE_PLANE,s_params);
    glTexGenfv (GL_T,GL_EYE_PLANE,t_params);
  }
  else {
    glDisable (GL_TEXTURE_2D);
    glColor3f (GROUND_R*SHADOW_INTENSITY,GROUND_G*SHADOW_INTENSITY,
	       GROUND_B*SHADOW_INTENSITY);
  }
  glDepthRange (0,0.9999);
}


extern "C" void dsSimulationLoop (int argc, char **argv,
				  int window_width, int window_height,
				  dsFunctions *fn)
{
  if (current_state != 0) dsError ("dsSimulationLoop() called more than once");
  current_state = 1;

  // look for flags that apply to us
  int initial_pause = 0;
  for (int i=1; i<argc; i++) {
    if (strcmp(argv[i],"-notex")==0) use_textures = 0;
    if (strcmp(argv[i],"-noshadow")==0) use_shadows = 0;
    if (strcmp(argv[i],"-noshadows")==0) use_shadows = 0;
    if (strcmp(argv[i],"-pause")==0) initial_pause = 1;
  }

  if (fn->version > DS_VERSION)
    dsDebug ("bad version number in dsFunctions structure");

  initMotionModel();
  dsPlatformSimLoop (window_width,window_height,fn,initial_pause);

  current_state = 0;
}


extern "C" void dsSetViewpoint (float xyz[3], float hpr[3])
{
  if (current_state < 1) dsError ("dsSetViewpoint() called before simulation started");
  if (xyz) {
    view_xyz[0] = xyz[0];
    view_xyz[1] = xyz[1];
    view_xyz[2] = xyz[2];
  }
  if (hpr) {
    view_hpr[0] = hpr[0];
    view_hpr[1] = hpr[1];
    view_hpr[2] = hpr[2];
    wrapCameraAngles();
  }
}


extern "C" void dsGetViewpoint (float xyz[3], float hpr[3])
{
  if (current_state < 1) dsError ("dsGetViewpoint() called before simulation started");
  if (xyz) {
    xyz[0] = view_xyz[0];
    xyz[1] = view_xyz[1];
    xyz[2] = view_xyz[2];
  }
  if (hpr) {
    hpr[0] = view_hpr[0];
    hpr[1] = view_hpr[1];
    hpr[2] = view_hpr[2];
  }
}


extern "C" void dsSetTexture (int texture_number)
{
  if (current_state != 2) dsError ("drawing function called outside simulation loop");
  tnum = texture_number;
}


extern "C" void dsSetColor (float red, float green, float blue)
{
  if (current_state != 2) dsError ("drawing function called outside simulation loop");
  color[0] = red;
  color[1] = green;
  color[2] = blue;
  color[3] = 1;
}


extern "C" void dsSetColorAlpha (float red, float green, float blue,
				 float alpha)
{
  if (current_state != 2) dsError ("drawing function called outside simulation loop");
  color[0] = red;
  color[1] = green;
  color[2] = blue;
  color[3] = alpha;
}


extern "C" void dsDrawBox (const float pos[3], const float R[12],
			   const float sides[3])
{
  if (current_state != 2) dsError ("drawing function called outside simulation loop");
  setupDrawingMode();
  glShadeModel (GL_FLAT);
  setTransform (pos,R);
  drawBox (sides);
  glPopMatrix();

  if (use_shadows) {
    setShadowDrawingMode();
    setShadowTransform();
    setTransform (pos,R);
    drawBox (sides);
    glPopMatrix();
    glPopMatrix();
    glDepthRange (0,1);
  }
}


extern "C" void dsDrawSphere (const float pos[3], const float R[12],
			      float radius)
{
  if (current_state != 2) dsError ("drawing function called outside simulation loop");
  setupDrawingMode();
  glEnable (GL_NORMALIZE);
  glShadeModel (GL_SMOOTH);
  setTransform (pos,R);
  glScaled (radius,radius,radius);
  drawSphere();
  glPopMatrix();
  glDisable (GL_NORMALIZE);

  // draw shadows
  if (use_shadows) {
    glDisable (GL_LIGHTING);
    if (use_textures) {
      ground_texture->bind (1);
      glEnable (GL_TEXTURE_2D);
      glDisable (GL_TEXTURE_GEN_S);
      glDisable (GL_TEXTURE_GEN_T);
      glColor3f (SHADOW_INTENSITY,SHADOW_INTENSITY,SHADOW_INTENSITY);
    }
    else {
      glDisable (GL_TEXTURE_2D);
      glColor3f (GROUND_R*SHADOW_INTENSITY,GROUND_G*SHADOW_INTENSITY,
		 GROUND_B*SHADOW_INTENSITY);
    }
    glShadeModel (GL_FLAT);
    glDepthRange (0,0.9999);
    drawSphereShadow (pos[0],pos[1],pos[2],radius);
    glDepthRange (0,1);
  }
}


extern "C" void dsDrawTriangle (const float pos[3], const float R[12],
				const float *v0, const float *v1,
				const float *v2, int solid)
{
  if (current_state != 2) dsError ("drawing function called outside simulation loop");
  setupDrawingMode();
  glShadeModel (GL_FLAT);
  setTransform (pos,R);
  drawTriangle (v0, v1, v2, solid);
  glPopMatrix();
}

void dsDrawCone(const float pos[3], const float R[12],
		     float length, float radius)
{

	 if (current_state != 2) dsError ("drawing function called outside simulation loop");
  setupDrawingMode();
  glShadeModel (GL_SMOOTH);
  setTransform (pos,R);
  drawCone(length,radius,0);
  glPopMatrix();

  if (use_shadows) {
    setShadowDrawingMode();
    setShadowTransform();
    setTransform (pos,R);
    drawCone (length,radius,0);
    glPopMatrix();
    glPopMatrix();
    glDepthRange (0,1);
  }
}
void dsDrawCylinder2 (const float pos[3], const float R[12],
		     float length, float radius1,float radius2)
{
	 if (current_state != 2) dsError ("drawing function called outside simulation loop");
  setupDrawingMode();
  glShadeModel (GL_SMOOTH);
  setTransform (pos,R);
  drawCylinder2 (length,radius1,radius2,0);
  glPopMatrix();

  if (use_shadows) {
    setShadowDrawingMode();
    setShadowTransform();
    setTransform (pos,R);
    drawCylinder2 (length,radius1,radius2,0);
    glPopMatrix();
    glPopMatrix();
    glDepthRange (0,1);
  }
}


extern "C" void dsDrawCylinder (const float pos[3], const float R[12],
				float length, float radius)
{
  if (current_state != 2) dsError ("drawing function called outside simulation loop");
  setupDrawingMode();
  glShadeModel (GL_SMOOTH);
  setTransform (pos,R);
  drawCylinder (length,radius,0);
  glPopMatrix();

  if (use_shadows) {
    setShadowDrawingMode();
    setShadowTransform();
    setTransform (pos,R);
    drawCylinder (length,radius,0);
    glPopMatrix();
    glPopMatrix();
    glDepthRange (0,1);
  }
}


extern "C" void dsDrawCappedCylinder (const float pos[3], const float R[12],
				      float length, float radius)
{
  if (current_state != 2) dsError ("drawing function called outside simulation loop");
  setupDrawingMode();
  glShadeModel (GL_SMOOTH);
  setTransform (pos,R);
  drawCappedCylinder (length,radius);
  glPopMatrix();

  if (use_shadows) {
    setShadowDrawingMode();
    setShadowTransform();
    setTransform (pos,R);
    drawCappedCylinder (length,radius);
    glPopMatrix();
    glPopMatrix();
    glDepthRange (0,1);
  }
}


void dsDrawLine (const float pos1[3], const float pos2[3])
{
  setupDrawingMode();
  glColor3f (color[0],color[1],color[2]);
  glDisable (GL_LIGHTING);
  glLineWidth (2);
  glShadeModel (GL_FLAT);
  glBegin (GL_LINES);
  glVertex3f (pos1[0],pos1[1],pos1[2]);
  glVertex3f (pos2[0],pos2[1],pos2[2]);
  glEnd();
}


void dsDrawBoxD (const double pos[3], const double R[12],
		 const double sides[3])
{
  int i;
  float pos2[3],R2[12],fsides[3];
  for (i=0; i<3; i++) pos2[i]=(float)pos[i];
  for (i=0; i<12; i++) R2[i]=(float)R[i];
  for (i=0; i<3; i++) fsides[i]=(float)sides[i];
  dsDrawBox (pos2,R2,fsides);
}


void dsDrawSphereD (const double pos[3], const double R[12], float radius)
{
  int i;
  float pos2[3],R2[12];
  for (i=0; i<3; i++) pos2[i]=(float)pos[i];
  for (i=0; i<12; i++) R2[i]=(float)R[i];
  dsDrawSphere (pos2,R2,radius);
}


void dsDrawTriangleD (const double pos[3], const double R[12],
				 const double *v0, const double *v1,
				 const double *v2, int solid)
{
  int i;
  float pos2[3],R2[12];
  for (i=0; i<3; i++) pos2[i]=(float)pos[i];
  for (i=0; i<12; i++) R2[i]=(float)R[i];

  setupDrawingMode();
  glShadeModel (GL_FLAT);
  setTransform (pos2,R2);
  drawTriangleD (v0, v1, v2, solid);
  glPopMatrix();
}


void dsDrawCylinderD (const double pos[3], const double R[12],
		      float length, float radius)
{
  int i;
  float pos2[3],R2[12];
  for (i=0; i<3; i++) pos2[i]=(float)pos[i];
  for (i=0; i<12; i++) R2[i]=(float)R[i];
  dsDrawCylinder (pos2,R2,length,radius);
}


void dsDrawCappedCylinderD (const double pos[3], const double R[12],
			    float length, float radius)
{
  int i;
  float pos2[3],R2[12];
  for (i=0; i<3; i++) pos2[i]=(float)pos[i];
  for (i=0; i<12; i++) R2[i]=(float)R[i];
  dsDrawCappedCylinder (pos2,R2,length,radius);
}


void dsDrawLineD (const double _pos1[3], const double _pos2[3])
{
  int i;
  float pos1[3],pos2[3];
  for (i=0; i<3; i++) pos1[i]=(float)_pos1[i];
  for (i=0; i<3; i++) pos2[i]=(float)_pos2[i];
  dsDrawLine (pos1,pos2);
}


void dsSetSphereQuality (int n)
{
  sphere_quality = n;
}


void dsSetCappedCylinderQuality (int n)
{
  capped_cylinder_quality = n;
}
