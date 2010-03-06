/*
  FLUIDS v.1 - SPH Fluid Simulator for CPU and GPU
  Copyright (C) 2009. Rama Hoetzlein, http://www.rchoetzlein.com

  ZLib license
  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

#include "common_defs.h"
#include "gl_helper.h"

#include <math.h>


// Shadow Light
float light_proj[16];
float light_x, light_y, light_z;
float light_tox, light_toy, light_toz;
float light_mfov;

// Fonts
void *font = GLUT_BITMAP_8_BY_13;
void *fonts[] = {GLUT_BITMAP_9_BY_15,
				 GLUT_BITMAP_TIMES_ROMAN_10,
				GLUT_BITMAP_TIMES_ROMAN_24};
// Timing
mint::Time	tm_last;
int			tm_cnt;
float		tm_fps;

GLuint glSphere = 65535;
float  glRadius = 0.0;

void setSphereRadius ( float r )
{
	if ( glRadius == r ) return;
	glRadius = r;

	// GL sphere
	if ( glSphere != 65535 ) glDeleteLists ( glSphere, 1 );
	glSphere = glGenLists ( 1 );
	float x, y, z, x1, y1, z1;	
	glNewList ( glSphere, GL_COMPILE );
		glBegin ( GL_TRIANGLE_STRIP );
		for ( float tilt=-90; tilt <= 90; tilt += 10.0) {
			for ( float ang=0; ang <= 360; ang += 30.0) {
				x = sin ( ang*DEGtoRAD) * cos ( tilt*DEGtoRAD );
				y = cos ( ang*DEGtoRAD) * cos ( tilt*DEGtoRAD );
				z = sin ( tilt*DEGtoRAD ) ;
				x1 = sin ( ang*DEGtoRAD) * cos ( (tilt+10.0)*DEGtoRAD ) ;
				y1 = cos ( ang*DEGtoRAD) * cos ( (tilt+10.0)*DEGtoRAD ) ;
				z1 = sin ( (tilt+10.0)*DEGtoRAD );
				glNormal3f ( x, y, z );		glVertex3f ( x*r, y*r, z*r );		
				glNormal3f ( x1, y1, z1 );	glVertex3f ( x1*r, y1*r, z1*r );
			}
		}
		glEnd ();
	glEndList ();
}

void drawSphere ()
{
	if ( glRadius == 0.0 ) setSphereRadius ( 1.0 );
	glCallList ( glSphere );
}

// Check if there have been any openGL problems
void checkOpenGL ()
{
	GLenum errCode = glGetError();
	if (errCode != GL_NO_ERROR) {
		const GLubyte* errString = gluErrorString(errCode);
		fprintf( stderr, "OpenGL error: %s\n", errString );
	}
}

void drawText ( int x, int y, char* msg)
{
  int len, i;
  glRasterPos2f(x, y);
  len = (int) strlen(msg);
  for (i = 0; i < len; i++) 
    glutBitmapCharacter(font, msg[i]);  
}

void drawGrid ()
{
	glColor3f ( 0.3, 0.3, 0.3 );
	glBegin ( GL_LINES );
	for (float x=-40; x<=40.0; x+=10.0 ) {
		glVertex3f ( x, -40.0, 0 );
		glVertex3f ( x,  40.0, 0 );
	}
	for (float y=-40; y<=40.0; y+=10.0 ) {
		glVertex3f ( -40.0, y, 0 );
		glVertex3f (  40.0, y, 0 );
	}
	glEnd ();
}

void measureFPS ()
{
	// Measure FPS
	mint::Time tm_elaps;	
	if ( ++tm_cnt > 5 ) {		
		tm_elaps.SetSystemTime ( ACC_NSEC );			// get current sytem time - accurate to 1 ns
		tm_elaps = tm_elaps - tm_last;					// get elapsed time from 5 frames ago
		tm_fps = 5.0 * 1000.0 / tm_elaps.GetMSec ();	// compute fps
		tm_cnt = 0;										// reset frame counter
		tm_last.SetSystemTime ( ACC_NSEC );
	}
}

void checkFrameBuffers ()
{                                                            
	GLenum status;                                             
	status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);  
	switch(status) {                                          
	case GL_FRAMEBUFFER_COMPLETE_EXT: printf ( "FBO complete\n" ); break;                                                
	case GL_FRAMEBUFFER_UNSUPPORTED_EXT: printf ( "FBO format unsupported\n"); break;                                                
	default:  printf ( "Unknown FBO error\n");
	}
}

void disableShadows ()
	{
		glDisable ( GL_TEXTURE_2D );		
		
		glActiveTextureARB( GL_TEXTURE1_ARB );
		glBindTexture ( GL_TEXTURE_2D, 0 );
		glDisable ( GL_TEXTURE_GEN_S );
		glDisable ( GL_TEXTURE_GEN_T );
		glDisable ( GL_TEXTURE_GEN_R );
		glDisable ( GL_TEXTURE_GEN_Q );	
		
		glActiveTextureARB( GL_TEXTURE2_ARB );
		glBindTexture ( GL_TEXTURE_2D, 0 );		
		glDisable ( GL_TEXTURE_GEN_S );
		glDisable ( GL_TEXTURE_GEN_T );
		glDisable ( GL_TEXTURE_GEN_R );
		glDisable ( GL_TEXTURE_GEN_Q );	
	}

#ifdef USE_SHADOWS
	// Materials & Textures
	GLuint shadow1_id = 0;			// display buffer shadows
	GLuint shadow2_id = 0;			// display buffer shadows

	// Frame buffer
	GLuint frameBufferObject = 0;	// frame buffer shadows

	void createFrameBuffer () 
	{
		//Generate the frame buffer object
		glGenFramebuffersEXT (1, &frameBufferObject);  
		glBindFramebufferEXT (GL_FRAMEBUFFER_EXT, frameBufferObject);		// Turn on frame buffer object
		glFramebufferTexture2DEXT (GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, shadow1_id, 0);
		glDrawBuffer (GL_NONE);							// Set Draw & ReadBuffer to none since we're rendering depth only
		glReadBuffer (GL_NONE);
		checkFrameBuffers ();					// Check completeness of frame buffer object (no need for stencil and depth attachement)
		glBindFramebufferEXT (GL_FRAMEBUFFER_EXT, 0);	// Turn off frame buffer object
	}

	void createShadowTextures ()
	{
		// Create depth texture maps
		glActiveTextureARB( GL_TEXTURE1_ARB );
		glGenTextures( 1, &shadow1_id );	
		glBindTexture ( GL_TEXTURE_2D, shadow1_id );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
		
		//-- sets region outside shadow to 0
		//glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER_ARB );
		//glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER_ARB  );
		
		//-- sets region outside shadow to 1 (border edge color)
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );				

		glTexParameteri ( GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE_ARB, GL_COMPARE_R_TO_TEXTURE_ARB);
		glTexParameteri ( GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC_ARB, GL_LEQUAL);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		glTexImage2D ( GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24_ARB, TEX_SIZE, TEX_SIZE, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, 0);

		glActiveTextureARB( GL_TEXTURE2_ARB );
		glGenTextures( 1, &shadow2_id );
		glBindTexture ( GL_TEXTURE_2D, shadow2_id );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
		//glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER_ARB );
		//glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER_ARB  );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE  );
		glTexParameteri ( GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE_ARB, GL_COMPARE_R_TO_TEXTURE_ARB);
		glTexParameteri ( GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC_ARB, GL_LEQUAL);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		glTexImage2D ( GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24_ARB, TEX_SIZE, TEX_SIZE, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, 0);
	}

	void computeLightMatrix ( int n, int tx, int ty )
	{
		int lnum = n;
		// Construct projective texturing matrix

		// S - light bias matrix
		glMatrixMode ( GL_MODELVIEW );
		glLoadIdentity ();
		glTranslatef ( 0.5, 0.5, 0.5 );
		glScalef ( 0.5, 0.5, 0.5 );
		// Plight - light projection matrix
		gluPerspective ( light_mfov*2.0, float(tx) / ty, LIGHT_NEAR, LIGHT_FAR );
		// L^-1 - light view inverse matrix
		gluLookAt ( light_x, light_y, light_z, light_tox, light_toy, light_toz, 0, 0, 1);
		glPushMatrix ();
		glGetFloatv ( GL_MODELVIEW_MATRIX, light_proj );
		glPopMatrix ();

	}

	void renderDepthMap_Clear ( float wx, float wy )
	{
		glBindFramebufferEXT (GL_FRAMEBUFFER_EXT, frameBufferObject);
		glFramebufferTexture2DEXT (GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, shadow1_id, 0);
		glActiveTextureARB( GL_TEXTURE1_ARB );			// TEXTURE1 = shadow map stage	
		glViewport (1, 1, TEX_SIZE-2, TEX_SIZE-2);		// Note: Avoid artifact cause by drawing into border pixels
		glClear ( GL_DEPTH_BUFFER_BIT );	
		glBindFramebufferEXT (GL_FRAMEBUFFER_EXT, 0);
		glViewport ( 0, 0, (GLsizei) wx, (GLsizei) wy );
	}

	void renderDepthMap_FrameBuffer ( int n, float wx, float wy )
	{
		float vmat[16];

		computeLightMatrix ( n, TEX_SIZE, TEX_SIZE );	

		glBindFramebufferEXT (GL_FRAMEBUFFER_EXT, frameBufferObject);
		
		if ( n == 0 )	{	
			glFramebufferTexture2DEXT (GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, shadow1_id, 0);
		} else {		
			glFramebufferTexture2DEXT (GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, shadow2_id, 0);
		}
		
		if ( n == 0 )	glActiveTextureARB( GL_TEXTURE1_ARB );			// TEXTURE1 = shadow map stage	
		else			glActiveTextureARB( GL_TEXTURE2_ARB );			// TEXTURE2 = shadow map stage				
		
		glViewport (1, 1, TEX_SIZE-2, TEX_SIZE-2);		// Note: Avoid artifact cause by drawing into border pixels
		glClear ( GL_DEPTH_BUFFER_BIT );	
		glLoadIdentity();

		// Plight - projection matrix of light
		glMatrixMode ( GL_PROJECTION );			// Setup projection for depth-map rendering
		glLoadIdentity ();
		gluPerspective ( light_mfov*2.0, float(TEX_SIZE) / TEX_SIZE, LIGHT_NEAR, LIGHT_FAR );

		// L^-1 - light view matrix (gluLookAt computes inverse)
		glMatrixMode ( GL_MODELVIEW);			// Setup view for depth-map rendering
		glLoadIdentity ();
		gluLookAt ( light_x, light_y, light_z, light_tox, light_toy, light_toz, 0, 0, 1);
		glPushMatrix ();						// Save view matrix for later
		glGetFloatv ( GL_MODELVIEW_MATRIX, vmat );
		glPopMatrix ();

		glDisable ( GL_LIGHTING );
		glColor4f ( 1, 1, 1, 1 );
		glShadeModel (GL_FLAT);					// No shading (faster)
		
		glEnable ( GL_CULL_FACE );
		glCullFace ( GL_FRONT );	
		
		glEnable ( GL_POLYGON_OFFSET_FILL );
		glPolygonOffset ( 50.0, 0.1 );			// Depth bias
		
		drawScene ( &vmat[0], false );				// Draw scene. 

		glDisable ( GL_POLYGON_OFFSET_FILL );	
		glBindFramebufferEXT (GL_FRAMEBUFFER_EXT, 0);
		glViewport ( 0, 0, (GLsizei) wx, (GLsizei) wy );

		//glCullFace (GL_BACK);					// Restore render states
		//glBindTexture ( GL_TEXTURE_2D, 0);
	}

	void renderShadowStage ( int n, float* vmat )
	{
		GLfloat pos[4];
		GLfloat row[4];

		computeLightMatrix ( n, TEX_SIZE, TEX_SIZE );
		if ( n == 0 ) {
			glActiveTextureARB( GL_TEXTURE1_ARB );			// TEXTURE1 = shadow map stage #1
		} else {
			glActiveTextureARB( GL_TEXTURE2_ARB );			// TEXTURE2 = shadow map stage #2
		}
		glEnable ( GL_TEXTURE_2D );   
		if ( n == 0 )	glBindTexture ( GL_TEXTURE_2D, shadow1_id );
		else			glBindTexture ( GL_TEXTURE_2D, shadow2_id );	

		glMatrixMode( GL_MODELVIEW );	
		glLoadMatrixf ( vmat );

		row[0] = light_proj[0]; row[1] = light_proj[4]; row[2] = light_proj[8]; row[3] = light_proj[12];
		glTexGenfv(GL_S, GL_EYE_PLANE, &row[0] );
		row[0] = light_proj[1]; row[1] = light_proj[5]; row[2] = light_proj[9]; row[3] = light_proj[13];
		glTexGenfv(GL_T, GL_EYE_PLANE, &row[0] );
		row[0] = light_proj[2]; row[1] = light_proj[6]; row[2] = light_proj[10]; row[3] = light_proj[14];
		glTexGenfv(GL_R, GL_EYE_PLANE, &row[0] );
		row[0] = light_proj[3]; row[1] = light_proj[7]; row[2] = light_proj[11]; row[3] = light_proj[15];
		glTexGenfv(GL_Q, GL_EYE_PLANE, &row[0] );
		glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
		glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
		glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
		glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
		glEnable(GL_TEXTURE_GEN_S);    
		glEnable(GL_TEXTURE_GEN_T);
		glEnable(GL_TEXTURE_GEN_R);
		glEnable(GL_TEXTURE_GEN_Q);	

		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_COMBINE );
		
		glTexEnvi(GL_TEXTURE_ENV, GL_COMBINE_RGB, GL_INTERPOLATE ) ;
		glTexEnvi(GL_TEXTURE_ENV, GL_SOURCE0_RGB, GL_TEXTURE ) ;
		glTexEnvi(GL_TEXTURE_ENV, GL_SOURCE1_RGB, GL_PREVIOUS ) ;
		glTexEnvi(GL_TEXTURE_ENV, GL_SOURCE2_RGB, GL_CONSTANT ) ;
			
		pos[0] = 0.20;
		pos[1] = 0.20;
		pos[2] = 0.20;
		pos[3] = 0.20;
		glTexEnvfv (GL_TEXTURE_ENV, GL_TEXTURE_ENV_COLOR, &pos[0] );
	}

	void renderShadows ( float* vmat )
	{
		GLfloat pos[4];
		
		renderShadowStage ( 0, vmat );
	//	renderShadowStage ( 1, vmat );		

		glActiveTextureARB( GL_TEXTURE0_ARB );			// Render Tex 0 - Base render
		glDisable ( GL_TEXTURE_GEN_S );
		glDisable ( GL_TEXTURE_GEN_T );
		glDisable ( GL_TEXTURE_GEN_R );
		glDisable ( GL_TEXTURE_GEN_Q );	
		glTexEnvi ( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );
		glEnable ( GL_LIGHTING );
		glLightModeli (GL_LIGHT_MODEL_COLOR_CONTROL_EXT, GL_SEPARATE_SPECULAR_COLOR_EXT);
		
		glEnable ( GL_LIGHT0 );	
		pos[0] = light_x; pos[1] = light_y; pos[2] = light_z; pos[3] = 1.0;	
		glLightfv ( GL_LIGHT0, GL_POSITION, &pos[0] );
	
		/* glEnable ( GL_LIGHT1 );
		pos[0] = light[1].x; pos[1] = light[1].y; pos[2] = light[1].z; pos[3] = 1.0;	
		glLightfv ( GL_LIGHT1, GL_POSITION, &pos[0] );*/			
	}

	

	void setShadowLight ( float fx, float fy, float fz, float tx, float ty, float tz, float fov )
	{
		light_x = fx;
		light_y = fy;
		light_z = fz;
		light_tox = tx;
		light_toy = ty;
		light_toz = tz;
		light_mfov = fov;
	}

	void setShadowLightColor ( float dr, float dg, float db, float sr, float sg, float sb )
	{
		GLfloat amb[4] = {0.0,0.0,0.0,1};
		GLfloat dif[4];
		GLfloat spec[4];
		GLfloat pos[4] = {0.0,0.0,0.0, 100.0};

		glEnable(GL_LIGHT0);    
		dif[0] = dr; dif[1] = dg; dif[2] = db; dif[3] = 1;	
		spec[0] = sr; spec[1] = sg; spec[2] = sb; spec[3] = 1;
		glLightfv(GL_LIGHT0, GL_AMBIENT, &amb[0] );
		glLightfv(GL_LIGHT0, GL_DIFFUSE, &dif[0] );
		glLightfv(GL_LIGHT0, GL_SPECULAR, &spec[0] );
	}

#endif
