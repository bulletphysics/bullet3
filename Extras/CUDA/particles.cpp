/*
 * Copyright 1993-2007 NVIDIA Corporation.  All rights reserved.
 *
 * NOTICE TO USER:   
 *
 * This source code is subject to NVIDIA ownership rights under U.S. and 
 * international Copyright laws.  
 *
 * NVIDIA MAKES NO REPRESENTATION ABOUT THE SUITABILITY OF THIS SOURCE 
 * CODE FOR ANY PURPOSE.  IT IS PROVIDED "AS IS" WITHOUT EXPRESS OR 
 * IMPLIED WARRANTY OF ANY KIND.  NVIDIA DISCLAIMS ALL WARRANTIES WITH 
 * REGARD TO THIS SOURCE CODE, INCLUDING ALL IMPLIED WARRANTIES OF 
 * MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE.   
 * IN NO EVENT SHALL NVIDIA BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL, 
 * OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS 
 * OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE 
 * OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE 
 * OR PERFORMANCE OF THIS SOURCE CODE.  
 *
 * U.S. Government End Users.  This source code is a "commercial item" as 
 * that term is defined at 48 C.F.R. 2.101 (OCT 1995), consisting  of 
 * "commercial computer software" and "commercial computer software 
 * documentation" as such terms are used in 48 C.F.R. 12.212 (SEPT 1995) 
 * and is provided to the U.S. Government only as a commercial end item.  
 * Consistent with 48 C.F.R.12.212 and 48 C.F.R. 227.7202-1 through 
 * 227.7202-4 (JUNE 1995), all U.S. Government End Users acquire the 
 * source code with only those rights set forth herein.
 */

/*
    Particle system example with collisions using uniform grid
*/

#include <cstdlib>
#include <cstdio>
#include <algorithm>
#include <math.h>
//#include <cutil.h>

#include <GL/glew.h>

#if defined(__APPLE__) || defined(MACOSX)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "particleSystem.h"
#include "render_particles.h"
#include "paramgl.h"

// view params
int ox, oy;
int buttonState = 0;
float camera_trans[] = {0, 0, -3};
float camera_rot[]   = {0, 0, 0};
float camera_trans_lag[] = {0, 0, -3};
float camera_rot_lag[] = {0, 0, 0};
const float inertia = 0.1;
ParticleRenderer::DisplayMode displayMode = ParticleRenderer::PARTICLE_SPHERES;



int mode = 0;
bool displayEnabled = true;
bool bPause = false;
bool displaySliders = false;
bool wireframe = false;

enum { M_VIEW = 0, M_MOVE };

uint numParticles = 0;
uint3 gridSize;
int numIterations = 0; // run until exit

// simulation parameters
float timestep = 0.5f;
float damping = 1.0f;
float gravity = 0.0003f;
int iterations = 1;
int ballr = 10;

float collideSpring = 0.5f;;
float collideDamping = 0.02f;;
float collideShear = 0.1f;
float collideAttraction = 0.0f;

ParticleSystem *psystem = 0;

// fps

ParticleRenderer *renderer = 0;

float modelView[16];

ParamListGL *params;

extern "C" void cudaInit(int argc, char **argv);

void init(int numParticles, uint3 gridSize)
{
    psystem = new ParticleSystem(numParticles, gridSize); 
    psystem->reset(ParticleSystem::CONFIG_GRID);

    renderer = new ParticleRenderer;
    renderer->setParticleRadius(psystem->getParticleRadius());
    renderer->setColorBuffer(psystem->getColorBuffer());

    
}

void initGL()
{  
    glewInit();
    if (!glewIsSupported("GL_VERSION_2_0 GL_VERSION_1_5 GL_ARB_multitexture GL_ARB_vertex_buffer_object")) {
        fprintf(stderr, "Required OpenGL extensions missing.");
        exit(-1);
    }

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.25, 0.25, 0.25, 1.0);

    glutReportErrors();
}


void display()
{
    
    // update the simulation
    if (!bPause)
    {
        psystem->setIterations(iterations);
        psystem->setDamping(damping);
        psystem->setGravity(-gravity);
        psystem->setCollideSpring(collideSpring);
        psystem->setCollideDamping(collideDamping);
        psystem->setCollideShear(collideShear);
        psystem->setCollideAttraction(collideAttraction);

        psystem->update(timestep); 
        renderer->setVertexBuffer(psystem->getCurrentReadBuffer(), psystem->getNumParticles());
		float* posArray = psystem->getArray(ParticleSystem::POSITION);
		renderer->setPositions(posArray,psystem->getNumParticles());
    }

    // render
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  

    // view transform
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    for (int c = 0; c < 3; ++c)
    {
        camera_trans_lag[c] += (camera_trans[c] - camera_trans_lag[c]) * inertia;
        camera_rot_lag[c] += (camera_rot[c] - camera_rot_lag[c]) * inertia;
    }
    glTranslatef(camera_trans_lag[0], camera_trans_lag[1], camera_trans_lag[2]);
    glRotatef(camera_rot_lag[0], 1.0, 0.0, 0.0);
    glRotatef(camera_rot_lag[1], 0.0, 1.0, 0.0);

    glGetFloatv(GL_MODELVIEW_MATRIX, modelView);

    // cube
    glColor3f(1.0, 1.0, 1.0);
    glutWireCube(2.0);

    // collider
    glPushMatrix();
    float4 p = psystem->getColliderPos();
    glTranslatef(p.x, p.y, p.z);
    glColor3f(1.0, 0.0, 0.0);
    glutSolidSphere(psystem->getColliderRadius(), 20, 10);
    glPopMatrix();

    if (displayEnabled)
    {
        renderer->display(displayMode);
    }

    if (displaySliders) {
        glDisable(GL_DEPTH_TEST);
        glBlendFunc(GL_ONE_MINUS_DST_COLOR, GL_ZERO); // invert color
        glEnable(GL_BLEND);
        params->Render(0, 0);
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
    }

	psystem->debugDraw();
    
    glutSwapBuffers();

	{
		 char fps[256];
		 //float ifps = 1.f / (cutGetAverageTimerValue(timer) / 1000.f);
		 switch (psystem->getSimulationMode())
		 {
		 case ParticleSystem::SIMULATION_CUDA:
			 {
				sprintf(fps, "CUDA particles (%d particles)", numParticles);  
				break;
			 }
		case ParticleSystem::SIMULATION_BULLET_CPU:
			 {
				 sprintf(fps, "Bullet btCudaBroadphase (%d btSphereShapes)", numParticles);  
				 break;
			 }
		 default:
			 {
				 sprintf(fps, "Unknown simulation mode");  
			 }
		 }
		 glutSetWindowTitle(fps);
	}

    glutReportErrors();
}

void reshape(int w, int h)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (float) w / (float) h, 0.1, 10.0);

    glMatrixMode(GL_MODELVIEW);
    glViewport(0, 0, w, h);

    renderer->setWindowSize(w, h);
    renderer->setFOV(60.0);
}

void mouse(int button, int state, int x, int y)
{
    int mods;

    if (state == GLUT_DOWN)
        buttonState |= 1<<button;
    else if (state == GLUT_UP)
        buttonState = 0;

    mods = glutGetModifiers();
    if (mods & GLUT_ACTIVE_SHIFT) {
        buttonState = 2;
    } else if (mods & GLUT_ACTIVE_CTRL) {
        buttonState = 3;
    }

    ox = x; oy = y;

    if (displaySliders) {
        if (params->Mouse(x, y, button, state)) {
            glutPostRedisplay();
            return;
        }
    }

    glutPostRedisplay();
}

// transfrom vector by matrix
void xform(float *v, float *r, GLfloat *m)
{
  r[0] = v[0]*m[0] + v[1]*m[4] + v[2]*m[8] + m[12];
  r[1] = v[0]*m[1] + v[1]*m[5] + v[2]*m[9] + m[13];
  r[2] = v[0]*m[2] + v[1]*m[6] + v[2]*m[10] + m[14];
}

// transform vector by transpose of matrix
void ixform(float *v, float *r, GLfloat *m)
{
  r[0] = v[0]*m[0] + v[1]*m[1] + v[2]*m[2];
  r[1] = v[0]*m[4] + v[1]*m[5] + v[2]*m[6];
  r[2] = v[0]*m[8] + v[1]*m[9] + v[2]*m[10];
}

void ixformPoint(float *v, float *r, GLfloat *m)
{
    float x[4];
    x[0] = v[0] - m[12];
    x[1] = v[1] - m[13];
    x[2] = v[2] - m[14];
    x[3] = 1.0f;
    ixform(x, r, m);
}

void motion(int x, int y)
{
    float dx, dy;
    dx = x - ox;
    dy = y - oy;

    if (displaySliders) {
        if (params->Motion(x, y)) {
            ox = x; oy = y;
            glutPostRedisplay();
            return;
        }
    }

    switch(mode) 
    {
    case M_VIEW:
        if (buttonState == 3) {
            // left+middle = zoom
            camera_trans[2] += (dy / 100.0) * 0.5 * fabs(camera_trans[2]);
        } 
        else if (buttonState & 2) {
            // middle = translate
            camera_trans[0] += dx / 100.0;
            camera_trans[1] -= dy / 100.0;
        }
        else if (buttonState & 1) {
            // left = rotate
            camera_rot[0] += dy / 5.0;
            camera_rot[1] += dx / 5.0;
        }
        break;

    case M_MOVE:
        {
            float translateSpeed = 0.003f;
            float4 p = psystem->getColliderPos();
            if (buttonState==1) {
                float v[3], r[3];
                v[0] = dx*translateSpeed;
                v[1] = -dy*translateSpeed;
                v[2] = 0.0f;
                ixform(v, r, modelView);
                p.x += r[0];
                p.y += r[1];
                p.z += r[2];
            } else if (buttonState==2) {
                float v[3], r[3];
                v[0] = 0.0f;
                v[1] = 0.0f;
                v[2] = dy*translateSpeed;
                ixform(v, r, modelView);
                p.x += r[0];
                p.y += r[1];
                p.z += r[2];
            }
            psystem->setColliderPos(p);
        }
        break;
    }

    ox = x; oy = y;
    glutPostRedisplay();
}

inline float frand()
{
    return rand() / (float) RAND_MAX;
}

// commented out to remove unused parameter warnings in Linux
void key(unsigned char key, int /*x*/, int /*y*/)
{
    switch (key) 
    {
    case ' ':
        bPause = !bPause;
        break;
    case 13:
        psystem->update(timestep); 
        renderer->setVertexBuffer(psystem->getCurrentReadBuffer(), psystem->getNumParticles());
        break;
    case '\033':
    case 'q':
        exit(0);
        break;
    case 'v':
        mode = M_VIEW;
        break;
    case 'm':
        mode = M_MOVE;
        break;
	case 's':
		psystem->setSimulationMode((ParticleSystem::SimulationMode) ((psystem->getSimulationMode() + 1) % ParticleSystem::SIMULATION_NUM_MODES));
        break;

    case 'p':
        displayMode = (ParticleRenderer::DisplayMode) ((displayMode + 1) % ParticleRenderer::PARTICLE_NUM_MODES);
        break;
    case 'd':
        psystem->dumpGrid();
        break;
    case 'u':
        psystem->dumpParticles(0, 1);
        break;

    case 'r':
        displayEnabled = !displayEnabled;
        break;

    case '1':
        psystem->reset(ParticleSystem::CONFIG_GRID);
        break;
    case '2':
        psystem->reset(ParticleSystem::CONFIG_RANDOM);
        break;
    case '3':
        {
            // inject a sphere of particles
            float pr = psystem->getParticleRadius();
            float tr = pr+(pr*2.0f)*ballr;
            float pos[4], vel[4];
            pos[0] = -1.0 + tr + frand()*(2.0f - tr*2.0f);
            pos[1] = 1.0f - tr;
            pos[2] = -1.0 + tr + frand()*(2.0f - tr*2.0f);
            pos[3] = 0.0f;
            vel[0] = vel[1] = vel[2] = vel[3] = 0.0f;
            psystem->addSphere(0, pos, vel, ballr, pr*2.0f);
        }
        break;
    case '4':
        {
            // shoot ball from camera
            float pr = psystem->getParticleRadius();
            float vel[4], velw[4], pos[4], posw[4];
            vel[0] = 0.0f;
            vel[1] = 0.0f;
            vel[2] = -0.05f;
            vel[3] = 0.0f;
            ixform(vel, velw, modelView);

            pos[0] = 0.0f;
            pos[1] = 0.0f;
            pos[2] = -2.5f;
            pos[3] = 1.0;
            ixformPoint(pos, posw, modelView);
            posw[3] = 0.0f;

            psystem->addSphere(0, posw, velw, ballr, pr*2.0f);
        }
        break;

    case 'w':
        wireframe = !wireframe;
        break;

    case 'h':
        displaySliders = !displaySliders;
        break;
    }

    glutPostRedisplay();
}

void special(int k, int x, int y)
{
    if (displaySliders) {
        params->Special(k, x, y);
    }
}

void idle(void)
{
    glutPostRedisplay();
}

void initParams()
{
    // create a new parameter list
    params = new ParamListGL("misc");
    params->AddParam(new Param<float>("time step", timestep, 0.0, 1.0, 0.01, &timestep));
    params->AddParam(new Param<int>("iterations", iterations, 0, 10, 1, &iterations));
    params->AddParam(new Param<float>("damping", damping, 0.0, 1.0, 0.001, &damping));
    params->AddParam(new Param<float>("gravity", gravity, 0.0, 0.001, 0.0001, &gravity));
    params->AddParam(new Param<int>("ball r", ballr, 1, 20, 1, &ballr));

    params->AddParam(new Param<float>("collide spring", collideSpring, 0.0, 1.0, 0.001, &collideSpring));
    params->AddParam(new Param<float>("collide damping", collideDamping, 0.0, 0.1, 0.001, &collideDamping));
    params->AddParam(new Param<float>("collide shear", collideShear, 0.0, 0.1, 0.001, &collideShear));
    params->AddParam(new Param<float>("collide attract", collideAttraction, 0.0, 0.1, 0.001, &collideAttraction));
}

void mainMenu(int i)
{
    key((unsigned char) i, 0, 0);
}

void initMenus()
{
    glutCreateMenu(mainMenu);
    glutAddMenuEntry("Reset block [1]", '1');
    glutAddMenuEntry("Reset random [2]", '2');
    glutAddMenuEntry("Add sphere [3]", '3');
    glutAddMenuEntry("View mode [v]", 'v');
    glutAddMenuEntry("Move cursor mode [m]", 'm');
    glutAddMenuEntry("Toggle point rendering [p]", 'p');
	glutAddMenuEntry("Toggle Bullet simulation[s]", 's');
    glutAddMenuEntry("Toggle animation [ ]", ' ');
    glutAddMenuEntry("Step animation [ret]", 13);
    glutAddMenuEntry("Toggle sliders [h]", 'h');
    glutAddMenuEntry("Quit (esc)", '\033');
    glutAttachMenu(GLUT_RIGHT_BUTTON);
}

////////////////////////////////////////////////////////////////////////////////
// Program main
////////////////////////////////////////////////////////////////////////////////
int
main(int argc, char** argv) 
{
    numParticles =1024;//1024;//64;//16380;//32768;
    uint gridDim = 64;
    numIterations = 0;

    gridSize.x = gridSize.y = gridSize.z = gridDim;
    printf("grid: %d x %d x %d = %d cells\n", gridSize.x, gridSize.y, gridSize.z, gridSize.x*gridSize.y*gridSize.z);

    bool benchmark = false;
    
    cudaInit(argc, argv);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize(640, 480);
    glutCreateWindow("CUDA particles");

    initGL();
    init(numParticles, gridSize);
    initParams();
    initMenus();

   
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutKeyboardFunc(key);
    glutSpecialFunc(special);
    glutIdleFunc(idle);

    glutMainLoop();


    if (psystem)
        delete psystem;

    return 0;
}
