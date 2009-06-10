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

#include <time.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "common_defs.h"

#ifdef BUILD_CUDA
	#include "fluid_system_host.cuh"	
#endif
#include "fluid_system.h"
#include "gl_helper.h"

#ifdef _MSC_VER						// Windows
	#include <gl/glut.h>
#else								// Linux
	#include <GL/glut.h>	
#endif

bool bTiming = false;
bool bRec = false;
int mFrame = 0;

// Globals
FluidSystem			psys;

float window_width  = 1024;
float window_height = 768;

Vector3DF	cam_from, cam_angs, cam_to;			// Camera stuff
Vector3DF	obj_from, obj_angs, obj_dang;
Vector3DF	light[2], light_to[2];				// Light stuff
float		light_fov, cam_fov;	

int		psys_rate = 0;							// Particle stuff
int		psys_freq = 1;
int		psys_demo = 0;
int		psys_nmax = 4096;

bool	bHelp = false;						// Toggles
int		iShade = 1;			
int		iClrMode = 0;
bool	bPntDraw = false;
bool    bPause = false;

// View matricies
float view_matrix[16];					// View matrix (V)
float model_matrix[16];					// Model matrix (M)
float proj_matrix[16];					// Projective matrix

// Mouse control
#define DRAG_OFF		0				// mouse states
#define DRAG_LEFT		1
#define DRAG_RIGHT		2
int		last_x = -1, last_y = -1;		// mouse vars
int		mode = 0;
int		dragging = 0;
int		psel;

GLuint	screen_id;
GLuint	depth_id;


// Different things we can move around
#define MODE_CAM		0
#define MODE_CAM_TO		1
#define MODE_OBJ		2
#define MODE_OBJPOS		3
#define MODE_OBJGRP		4
#define MODE_LIGHTPOS	5

#define MODE_DOF		6

GLuint screenBufferObject;
GLuint depthBufferObject;
GLuint envid;

void drawScene ( float* viewmat, bool bShade )
{
	if ( iShade <= 1 && bShade ) {		
		glEnable ( GL_LIGHT0 );
		GLfloat diff[4];
		GLfloat spec[4];
		GLfloat shininess = 60.0;
		
		diff[0] = 0.8f; diff[1] = 0.8f; diff[2] = 0.8f; diff[3] = 1.0f;
		spec[0] = 1.0f; spec[1] = 1.0f; spec[2] = 1.0f; spec[3] = 1.0f;
		glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, &diff[0]);
		glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, &spec[0]);
		glMaterialfv (GL_FRONT_AND_BACK, GL_SHININESS, &shininess);		
		glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );

		glColor3f ( 1, 1, 1 );
		glLoadMatrixf ( viewmat );
		glBegin ( GL_QUADS );
		glNormal3f ( 0, 0, 1 );
		glVertex3f ( -1000, -1000, 0.0 );
		glVertex3f ( 1000, -1000, 0.0 );
		glVertex3f ( 1000, 1000, 0.0 );
		glVertex3f ( -1000, 1000, 0.0 );
		glEnd ();
		glBegin ( GL_LINES );
		for (float n=-100; n <= 100; n += 20.0 ) {
			glVertex3f ( -100, n, 0.1 );
			glVertex3f ( 100, n, 0.1 );
			glVertex3f ( n, -100, 0.1 );
			glVertex3f ( n,  100, 0.1 );
		}
		glEnd ();

		psys.Draw ( &viewmat[0], 0.8 );				// Draw particles		

	} else {
		glDisable ( GL_LIGHTING );
		psys.Draw ( &viewmat[0], 0.55 );			// Draw particles
	}
}

void draw2D ()
{
	
	mint::Time start, stop;

	#ifdef USE_SHADOWS
		disableShadows ();
	#endif
	glDisable ( GL_LIGHTING );  
	glDisable ( GL_DEPTH_TEST );

	glMatrixMode ( GL_PROJECTION );
	glLoadIdentity ();  
	glScalef ( 2.0/window_width, -2.0/window_height, 1 );		// Setup view (0,0) to (800,600)
	glTranslatef ( -window_width/2.0, -window_height/2, 0.0);

	glMatrixMode ( GL_MODELVIEW );
	glLoadIdentity ();
	glPushMatrix (); 
	glGetFloatv ( GL_MODELVIEW_MATRIX, view_matrix ); 
	glPopMatrix (); 

	char disp[200];
	glColor4f ( 1.0, 1.0, 1.0, 1.0 );

	strcpy ( disp, "Press H for help." );		drawText ( 10, 20, disp );  

	if ( bHelp ) {	

		if ( psys.GetToggle ( USE_CUDA ) ) {
			sprintf ( disp,	"Kernel:  USING CUDA (GPU)" );				drawText ( 20, 40,  disp );	
		} else {
			sprintf ( disp,	"Kernel:  USING CPU" );				drawText ( 20, 40,  disp );
		}		

		sprintf ( disp,	"KEYBOARD" );						drawText ( 20, 60,  disp );
		sprintf ( disp,	"[ ]    Next/Prev Demo" );			drawText ( 20, 70,  disp );
		sprintf ( disp,	"N M    Adjust Max Particles" );	drawText ( 20, 80,  disp );
		sprintf ( disp,	"space  Pause" );					drawText ( 20, 90,  disp );
		sprintf ( disp,	"S      Shading mode" );			drawText ( 20, 100,  disp );	
		sprintf ( disp,	"G      Toggle CUDA vs CPU" );		drawText ( 20, 110,  disp );	
		sprintf ( disp,	"< >    Change emitter rate" );		drawText ( 20, 120,  disp );	
		sprintf ( disp,	"C      Move camera /w mouse" );	drawText ( 20, 130,  disp );	
		sprintf ( disp,	"I      Move emitter /w mouse" );	drawText ( 20, 140,  disp );	
		sprintf ( disp,	"O      Change emitter angle" );	drawText ( 20, 150,  disp );	
		sprintf ( disp,	"L      Move light /w mouse" );				drawText ( 20, 160,  disp );			
		sprintf ( disp,	"X      Draw velocity/pressure/color" );	drawText ( 20, 170,  disp );

		Vector3DF vol = psys.GetVec(SPH_VOLMAX);
		vol -= psys.GetVec(SPH_VOLMIN);
		sprintf ( disp,	"Volume Size:           %3.5f %3.2f %3.2f", vol.x, vol.y, vol.z );	drawText ( 20, 190,  disp );
		sprintf ( disp,	"Time Step (dt):        %3.5f", psys.GetDT () );					drawText ( 20, 200,  disp );
		sprintf ( disp,	"Num Particles:         %d", psys.NumPoints() );					drawText ( 20, 210,  disp );		
		sprintf ( disp,	"Simulation Scale:      %3.5f", psys.GetParam(SPH_SIMSIZE) );		drawText ( 20, 220,  disp );
		sprintf ( disp,	"Simulation Size (m):   %3.5f", psys.GetParam(SPH_SIMSCALE) );		drawText ( 20, 230,  disp );
		sprintf ( disp,	"Smooth Radius (m):     %3.3f", psys.GetParam(SPH_SMOOTHRADIUS) );	drawText ( 20, 240,  disp );
		sprintf ( disp,	"Particle Radius (m):   %3.3f", psys.GetParam(SPH_PRADIUS) );		drawText ( 20, 250,  disp );
		sprintf ( disp,	"Particle Mass (kg):    %0.8f", psys.GetParam(SPH_PMASS) );			drawText ( 20, 260,  disp );
		sprintf ( disp,	"Rest Density (kg/m^3): %3.3f", psys.GetParam(SPH_RESTDENSITY) );	drawText ( 20, 270,  disp );
		sprintf ( disp,	"Viscosity:             %3.3f", psys.GetParam(SPH_VISC) );			drawText ( 20, 280,  disp );
		sprintf ( disp,	"Internal Stiffness:    %3.3f", psys.GetParam(SPH_INTSTIFF) );		drawText ( 20, 290,  disp );
		sprintf ( disp,	"Boundary Stiffness:    %6.0f", psys.GetParam(SPH_EXTSTIFF) );		drawText ( 20, 300,  disp );
		sprintf ( disp,	"Boundary Dampening:    %4.3f", psys.GetParam(SPH_EXTDAMP) );		drawText ( 20, 310,  disp );
		sprintf ( disp,	"Speed Limiting:        %4.3f", psys.GetParam(SPH_LIMIT) );			drawText ( 20, 320,  disp );
		vol = psys.GetVec ( PLANE_GRAV_DIR );
		sprintf ( disp,	"Gravity:               %3.2f %3.2f %3.2f", vol.x, vol.y, vol.z );	drawText ( 20, 330,  disp );
	}
}

void computeFromPositions ()
{
	cam_from.x = cam_to.x + sin( cam_angs.x * DEGtoRAD) * sin( cam_angs.y * DEGtoRAD) * cam_angs.z;
	cam_from.y = cam_to.y + -cos( cam_angs.x * DEGtoRAD) * sin( cam_angs.y * DEGtoRAD) * cam_angs.z;
	cam_from.z = cam_to.z + cos( cam_angs.y * DEGtoRAD) * cam_angs.z;	
}

void computeProjection ()
{
	// ---- Create projection matrix for eye-coordinate transformations 
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluPerspective ( cam_fov, window_width / ( float ) window_height, 10.0, 800.0 );
	glPushMatrix (); 
	glGetFloatv ( GL_MODELVIEW_MATRIX, proj_matrix ); 
	glPopMatrix ();
}

void computeView ()
{
	glMatrixMode ( GL_MODELVIEW );
	glLoadIdentity ();
	gluLookAt ( cam_from.x, cam_from.y, cam_from.z, cam_to.x, cam_to.y, cam_to.z, 0, 0, 1 );
	glPushMatrix (); 
	glGetFloatv ( GL_MODELVIEW_MATRIX, view_matrix ); 
	glPopMatrix ();
}	

int frame;

void display () 
{
	mint::Time start, stop;	

//	iso = sin(frame*0.01f );
	
	// Do simulation!
	if ( !bPause ) psys.Run ();

	frame++;
	measureFPS ();

	glEnable ( GL_DEPTH_TEST );

	// Render depth map shadows
	start.SetSystemTime ( ACC_NSEC );
	disableShadows ();
	#ifdef USE_SHADOWS
		if ( iShade==1 ) {
			renderDepthMap_FrameBuffer ( 0, window_width, window_height );
		} else {
			renderDepthMap_Clear ( window_width, window_height );		
		}
	#endif	

	// Clear frame buffer
	if ( iShade<=1 ) 	glClearColor( 0.29, 0.29, 0.29, 1.0 );
	else				glClearColor ( 0, 0, 0, 0 );
	glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	glDisable ( GL_CULL_FACE );
	glShadeModel ( GL_SMOOTH );

	// Compute camera view
	computeFromPositions ();
	computeProjection ();
	computeView ();		

	// Draw Shadows (if on)
	#ifdef USE_SHADOWS	
		if ( iShade==1 )	renderShadows ( view_matrix );			
	#endif

	// Draw 3D	
	start.SetSystemTime ( ACC_NSEC );
	glEnable ( GL_LIGHTING );  
	glLoadMatrixf ( view_matrix );	
	drawScene ( view_matrix, true );
	if ( bTiming) { stop.SetSystemTime ( ACC_NSEC ); stop = stop - start; printf ( "SCENE: %s\n", stop.GetReadableTime().c_str() ); }

	// Draw 2D overlay
	draw2D ();
 
	// Swap buffers
	glutSwapBuffers();  
	glutPostRedisplay();
}

void reshape ( int width, int height ) 
{
  // set window height and width
  window_width  = (float) width;
  window_height = (float) height;
  glViewport( 0, 0, width, height );  
}

void UpdateEmit ()
{	
	obj_from = psys.GetVec ( EMIT_POS );
	obj_angs = psys.GetVec ( EMIT_ANG );
	obj_dang = psys.GetVec ( EMIT_RATE );
}


void keyboard_func ( unsigned char key, int x, int y )
{
	switch( key ) {
	case 'M': case 'm': {
		psys_nmax *= 2;
		if ( psys_nmax > 65535 ) psys_nmax = 65535;		
		psys.SPH_CreateExample ( psys_demo, psys_nmax );
		} break;
	case 'N': case 'n': {
		psys_nmax /= 2;
		if ( psys_nmax < 64 ) psys_nmax = 64;		
		psys.SPH_CreateExample ( psys_demo, psys_nmax );
		} break;
	case '0':
		UpdateEmit ();
		psys_freq++;	
		psys.SetVec ( EMIT_RATE, Vector3DF(psys_freq, psys_rate, 0) );
		break;  
	case '9':
		UpdateEmit ();
		psys_freq--;  if ( psys_freq < 0 ) psys_freq = 0;
		psys.SetVec ( EMIT_RATE, Vector3DF(psys_freq, psys_rate, 0) );
		break;
	case '.': case '>':
		UpdateEmit ();
		if ( ++psys_rate > 100 ) psys_rate = 100;
		psys.SetVec ( EMIT_RATE, Vector3DF(psys_freq, psys_rate, 0) );
		break;
	case ',': case '<':
		UpdateEmit ();
		if ( --psys_rate < 0 ) psys_rate = 0;
		psys.SetVec ( EMIT_RATE, Vector3DF(psys_freq, psys_rate, 0) );
	break;
	case 'g': case 'G':	psys.Toggle ( USE_CUDA );	break;
	case 'f': case 'F':	mode = MODE_DOF;	break;

	case 'z': case 'Z':	mode = MODE_CAM_TO;	break;
	case 'c': case 'C':	mode = MODE_CAM;	break; 
	case 'h': case 'H':	bHelp = !bHelp; break;
	case 'i': case 'I':	
		UpdateEmit ();
		mode = MODE_OBJPOS;	
		break;
	case 'o': case 'O':	
		UpdateEmit ();
		mode = MODE_OBJ;
		break;  
	case 'x': case 'X':
		if ( ++iClrMode > 2) iClrMode = 0;
		psys.SetParam ( CLR_MODE, iClrMode );
		break;
	case 'l': case 'L':	mode = MODE_LIGHTPOS;	break;
	case 'd': case 'D': {
		int d = psys.GetParam ( PNT_DRAWMODE ) + 1;
		if ( d > 2 ) d = 0;
		psys.SetParam ( PNT_DRAWMODE, d );
		} break;	
	case 's': case 'S':	if ( ++iShade > 2 ) iShade = 0;		break;
	case 27:			    exit( 0 ); break;
	
	case '`':
		bRec = !bRec; break;

	case ' ':		
		//psys.Run (); ptris.Rebuild (); break;
		bPause = !bPause;	break;

	case '\'': case ';':	psys.SPH_CreateExample ( psys_demo, psys_nmax ); break;
	case 'r': case 'R':		psys.SPH_CreateExample ( psys_demo, psys_nmax ); break;  
	case '[':
		psys_demo--;
		if (psys_demo < 0 ) psys_demo = 10;
		psys.SPH_CreateExample ( psys_demo, psys_nmax );
		UpdateEmit ();
		break;
	case ']':
		psys_demo++;
		if (psys_demo > 10 ) psys_demo = 0;
		psys.SPH_CreateExample ( psys_demo, psys_nmax );
		UpdateEmit ();
		break;  
	default:
	break;
  }
}


void mouse_click_func ( int button, int state, int x, int y )
{
  if( state == GLUT_DOWN ) {
    if ( button == GLUT_LEFT_BUTTON )		dragging = DRAG_LEFT;
    else if ( button == GLUT_RIGHT_BUTTON ) dragging = DRAG_RIGHT;	
    last_x = x;
    last_y = y;	
  } else {
    dragging = DRAG_OFF;
  }
}

void mouse_move_func ( int x, int y )
{
	int dx = x - last_x;
	int dy = y - last_y;

	switch ( mode ) {
	case MODE_CAM:
		if ( dragging == DRAG_LEFT ) {
			cam_angs.x += dx;
			cam_angs.y += dy;
			if ( cam_angs.x >= 360.0 )	cam_angs.x -= 360.0;
			if ( cam_angs.x < 0 )		cam_angs.x += 360.0;
			if ( cam_angs.y >= 180.0 )	cam_angs.y = 180.0;
			if ( cam_angs.y <= -180.0 )	cam_angs.y = -180.0;
			printf ( "Cam Ang: %f %f %f\n", cam_angs.x, cam_angs.y, cam_angs.z );
			printf ( "Cam To:  %f %f %f\n", cam_to.x, cam_to.y, cam_to.z );
			printf ( "Cam FOV: %f\n", cam_fov);
		} else if ( dragging == DRAG_RIGHT ) {
			cam_angs.z += dy*.15;
			if ( cam_angs.z < 0)		cam_angs.z = 0;
			printf ( "Cam Ang: %f %f %f\n", cam_angs.x, cam_angs.y, cam_angs.z );
			printf ( "Cam To:  %f %f %f\n", cam_to.x, cam_to.y, cam_to.z );
			printf ( "Cam FOV: %f\n", cam_fov );
		}
		break;
	case MODE_CAM_TO:
		if ( dragging == DRAG_LEFT ) {
			cam_to.x += dx;
			cam_to.y += dy;			
		} else if ( dragging == DRAG_RIGHT ) {
			cam_to.z += dy*.05;
			if ( cam_to.z < 0) 	cam_to.z = 0;
		}
		break;	
	case MODE_OBJ:
		if ( dragging == DRAG_LEFT ) {
			obj_angs.x -= dx*0.1;
			obj_angs.y += dy*0.1;
			printf ( "Obj Angs:  %f %f %f\n", obj_angs.x, obj_angs.y, obj_angs.z );
			//force_x += dx*.1;
			//force_y += dy*.1;
		} else if (dragging == DRAG_RIGHT) {
			obj_angs.z -= dy*.005;			
			printf ( "Obj Angs:  %f %f %f\n", obj_angs.x, obj_angs.y, obj_angs.z );
		}
		psys.SetVec ( EMIT_ANG, Vector3DF ( obj_angs.x, obj_angs.y, obj_angs.z ) );
		break;
	case MODE_OBJPOS:
		if ( dragging == DRAG_LEFT ) {
			obj_from.x -= dx*.1;
			obj_from.y += dy*.1;
			printf ( "Obj:  %f %f %f\n", obj_from.x, obj_from.y, obj_from.z );
		} else if (dragging == DRAG_RIGHT) {
			obj_from.z -= dy*.1;
			printf ( "Obj:  %f %f %f\n", obj_from.x, obj_from.y, obj_from.z );
		}
		psys.SetVec ( EMIT_POS, Vector3DF ( obj_from.x, obj_from.y, obj_from.z ) );
		//psys.setPos ( obj_x, obj_y, obj_z, obj_ang, obj_tilt, obj_dist );
		break;
	case MODE_LIGHTPOS:
		if ( dragging == DRAG_LEFT ) {
			light[0].x -= dx*.1;
			light[0].y += dy*.1;		
			printf ( "Light: %f %f %f\n", light[0].x, light[0].y, light[0].z );
		} else if (dragging == DRAG_RIGHT) {
			light[0].z -= dy*.1;			
			printf ( "Light: %f %f %f\n", light[0].x, light[0].y, light[0].z );
		}	
		#ifdef USE_SHADOWS
			setShadowLight ( light[0].x, light[0].y, light[0].z, light_to[0].x, light_to[0].y, light_to[0].z, light_fov );
		#endif
		break;
	}

	if ( x < 10 || y < 10 || x > 1000 || y > 700 ) {
		glutWarpPointer ( 1024/2, 768/2 );
		last_x = 1024/2;
		last_y = 768/2;
	} else {
		last_x = x;
		last_y = y;
	}
}


void idle_func ()
{
}

void init ()
{
	
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);	
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);	

	srand ( time ( 0x0 ) );

	glClearColor( 0.49, 0.49, 0.49, 1.0 );
	glShadeModel( GL_SMOOTH );

	glEnable ( GL_COLOR_MATERIAL );
	glEnable (GL_DEPTH_TEST);  
	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
	glDepthMask ( 1 );
	glEnable ( GL_TEXTURE_2D );

	// callbacks
	glutDisplayFunc( display );
	glutReshapeFunc( reshape );
	glutKeyboardFunc( keyboard_func );
	glutMouseFunc( mouse_click_func );  
	glutMotionFunc( mouse_move_func );
	glutIdleFunc( idle_func );
	glutSetCursor ( GLUT_CURSOR_NONE );

	cam_angs.x = 29;		cam_angs.y = 75;		cam_angs.z = 80.0;
	cam_to.x = 0;		cam_to.y = 0;		cam_to.z = 5;
	cam_fov = 35.0;

	light[0].x = 39;		light[0].y = -60;	light[0].z = 43;
	light_to[0].x = 0;	light_to[0].y = 0;	light_to[0].z = 0;

	light[1].x = 15;		light[1].y = -5;	light[1].z = 145;	
	light_to[1].x = 0;	light_to[1].y = 0;	light_to[1].z = 0;  

	light_fov = 45;

	#ifdef USE_SHADOWS
		createShadowTextures();
		createFrameBuffer ();
		setShadowLight ( light[0].x, light[0].y, light[0].z, light_to[0].x, light_to[0].y, light_to[0].z, light_fov );
		setShadowLightColor ( .7, .7, .7, 0.2, 0.2, 0.2 );		
	#endif

	obj_from.x = 0;		obj_from.y = 0;		obj_from.z = 20;		// emitter
	obj_angs.x = 118.7;	obj_angs.y = 200;	obj_angs.z = 1.0;
	obj_dang.x = 1;	obj_dang.y = 1;		obj_dang.z = 0;

	psys.Initialize ( BFLUID, psys_nmax );
	psys.SPH_CreateExample ( 0, psys_nmax );
	psys.SetVec ( EMIT_ANG, Vector3DF ( obj_angs.x, obj_angs.y, obj_angs.z ) );
	psys.SetVec ( EMIT_POS, Vector3DF ( obj_from.x, obj_from.y, obj_from.z ) );

	psys.SetParam ( PNT_DRAWMODE, int(bPntDraw ? 1:0) );
	psys.SetParam ( CLR_MODE, iClrMode );	
}


int main ( int argc, char **argv )
{
	#ifdef BUILD_CUDA
		// Initialize CUDA
		cudaInit( argc, argv );
	#endif

	// set up the window
	glutInit( &argc, &argv[0] ); 
	glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
	glutInitWindowPosition( 100, 100 );
	glutInitWindowSize( (int) window_width, (int) window_height );
	glutCreateWindow ( "Fluids v.1 (c) 2008, R. Hoetzlein (ZLib)" );

//	glutFullScreen ();
 
	// initialize parameters
	init();

	// wait for something to happen
	glutMainLoop();

  return 0;
}

extern "C" {
	void btCuda_exit(int val)
	{
		fprintf(stderr, "Press ENTER key to terminate the program\n");
		getchar();
		exit(val);
	}
}