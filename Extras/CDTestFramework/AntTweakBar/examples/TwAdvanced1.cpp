//	---------------------------------------------------------------------------
//
//	@file		TwAdvanced1.cpp
//	@brief		An example showing many features of AntTweakBar.
//				It also uses OpenGL and GLFW windowing system
//				but could be easily adapted to other frameworks.
//
//				AntTweakBar: http://www.antisphere.com/Wiki/tools:anttweakbar
//				OpenGL:		 http://www.opengl.org
//				GLFW:		 http://glfw.sourceforge.net
//	
//
//				This example draws a simple scene that can be re-tesselated 
//				interactively, and illuminated dynamically by an adjustable 
//				number of moving lights.
//
//
//	@author		Philippe Decaudin - http://www.antisphere.com
//	@date		2006/05/20
//
//	note:		TAB=4
//
//	Compilation:
//	http://www.antisphere.com/Wiki/tools:anttweakbar:examples#twadvanced1
//
//	---------------------------------------------------------------------------

#include <AntTweakBar.h>

#define GLFW_DLL	// use GLFW as a dynamically linked library
#include "glfw.h"

#include <cmath>
#include <iostream>
#ifndef _WIN32
#	define _snprintf snprintf
#endif
const float FLOAT_2PI = 6.283185307f;	// 2*PI


//	Light structure: embed light parameters
struct Light
{
	bool	Active;								// light On or Off
	float	Pos[4];								// light position (in homogeneous coordinates, ie. Pos[4]=1)
	float	Color[4];							// light color (no alpha, ie. Color[4]=1)
	float	Radius;								// radius of the light influence area 
	float   Dist0, Angle0, Height0, Speed0;		// light initial cylindrical coordinates and speed
	char	Name[4];							// light short name (will be named "1", "2", "3",...)
	enum	AnimMode { ANIM_FIXED, ANIM_BOUNCE, ANIM_ROTATE, ANIM_COMBINED };
	AnimMode Animation;							// light animation mode
};


//	Class that describes the scene and its methods
class Scene
{
public:
	bool	Wireframe;							// draw scene in wireframe or filled
	int		Subdiv;								// number of subdivisions used to tesselate the scene
	int		NumLights;							// number of dynamic lights
	float	BgColor0[3], BgColor1[3];			// top and bottom background colors
	float	Ambient;							// scene ambient factor 
	float	Reflection;							// ground plane reflection factor (0=no reflection, 1=full reflection)
	float	RotYAngle;							// rotation angle of the scene around its Y axis (in degree)
	enum	RotMode { ROT_OFF, ROT_CW, ROT_CCW };
	RotMode	Rotation;							// scene rotation mode (off, clockwise, counter-clockwise)

			Scene();							// constructor
			~Scene();							// destructor
	void	Init(bool changeLightPos);			// (re)intialize the scene
	void	Draw();								// draw scene
	void	Update(float time);					// move lights

private:
	void	CreateBar();						// create a tweak bar for lights
	// Some drawing subroutines
	void	DrawSubdivPlaneY(float xMin, float xMax, float y, float zMin, float zMax, int xSubdiv, int zSubdiv);
	void	DrawSubdivCylinderY(float xCenter, float yBottom, float zCenter, float height, float radiusBottom, float radiusTop, int sideSubdiv, int ySubdiv);
	void	DrawSubdivHaloZ(float x, float y, float z, float radius, int subdiv);
	void	DrawHalos(bool reflected);

	GLuint	objList, groundList, haloList;		// OpenGL display list IDs
	int		maxLights;							// maximum number of dynamic lights allowed by the graphic card
	Light *	lights;								// array of lights
	TwBar * lightsBar;							// pointer to the tweak bar for lights created by CreateBar()
};


// Constructor
Scene::Scene()
{
	// Set scene members.
	// The scene will be created by Scene::Init( )
	Wireframe = false;
	Subdiv = 20;
	NumLights = 0;
	BgColor0[0] = 0.9f;
	BgColor0[1] = 0.0f;
	BgColor0[2] = 0.0f;
	BgColor1[0] = 0.6f;
	BgColor1[1] = 0.6f;
	BgColor1[2] = 0.6f;
	Ambient = 0.3f;
	Reflection = 0.5f;
	RotYAngle = 0;
	Rotation = ROT_CCW;
	objList = 0;
	groundList = 0;
	haloList = 0;
	maxLights = 0;
	lights = NULL;
	lightsBar = NULL;
}


// Destructor
Scene::~Scene() 
{ 
	// delete all lights
	if( lights ) 
		delete[] lights;
}


//	Create the scene, and (re)initialize lights if changeLights is true
void Scene::Init(bool changeLights)
{
	// Get the max number of lights allowed by the graphic card
	glGetIntegerv(GL_MAX_LIGHTS, &maxLights);
	if( maxLights>16 )
		maxLights = 16;

	// Create the lights array
	if( lights==NULL && maxLights>0 )
	{
		lights = new Light[maxLights];
		NumLights = maxLights/2;	// default number of lights
		if( NumLights==0 )
			NumLights = 1;
		changeLights = true;		// force lights initialization

		// Create a tweak bar for lights
		CreateBar();
	}

	// (Re)initialize lights if needed (uses random values)
	if( changeLights )
		for(int i=0; i<maxLights; ++i)
		{
			lights[i].Dist0		= 0.5f*(float)rand()/RAND_MAX + 0.55f;
			lights[i].Angle0	= FLOAT_2PI*((float)rand()/RAND_MAX);
			lights[i].Height0	= FLOAT_2PI*(float)rand()/RAND_MAX;
			lights[i].Speed0	= 4.0f*(float)rand()/RAND_MAX - 2.0f;
			lights[i].Animation	= Light::ANIM_BOUNCE;
			lights[i].Radius	= (float)rand()/RAND_MAX+0.05f;
			lights[i].Color[0]	= (float)rand()/RAND_MAX;
			lights[i].Color[1]	= (float)rand()/RAND_MAX;
			lights[i].Color[2]	= (lights[i].Color[0]>lights[i].Color[1]) ? 1.0f-lights[i].Color[1] : 1.0f-lights[i].Color[0];
			lights[i].Color[3]	= 1;
			lights[i].Active	= true;
		}

	// Initialize some OpenGL states
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_LIGHTING);
	glEnable(GL_CULL_FACE);
	glEnable(GL_NORMALIZE);
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);

	// Create objects display list using the current Subdiv parameter to control the tesselation
	if( objList>0 )
		glDeleteLists(objList, 1);		// delete previously created display list
	objList = glGenLists(1);
	glNewList(objList, GL_COMPILE);
	DrawSubdivCylinderY(-0.9f, 0, -0.9f, 1.4f, 0.15f, 0.12f, Subdiv/2+8, Subdiv);
	DrawSubdivCylinderY(+0.9f, 0, -0.9f, 1.4f, 0.15f, 0.12f, Subdiv/2+8, Subdiv);
	DrawSubdivCylinderY(+0.9f, 0, +0.9f, 1.4f, 0.15f, 0.12f, Subdiv/2+8, Subdiv);
	DrawSubdivCylinderY(-0.9f, 0, +0.9f, 1.4f, 0.15f, 0.12f, Subdiv/2+8, Subdiv);
	DrawSubdivCylinderY(0, 0, 0, 0.4f, 0.5f, 0.3f, Subdiv+16, Subdiv/8+1);
	DrawSubdivCylinderY(0, 0.4f, 0, 0.05f, 0.3f, 0.0f, Subdiv+16, Subdiv/16+1);
	glEndList();

	// Create ground display list
	if( groundList>0 )
		glDeleteLists(groundList, 1);	// delete previously created display list
	groundList = glGenLists(1);
	glNewList(groundList, GL_COMPILE);
	DrawSubdivPlaneY(-1.2f, 1.2f, 0, -1.2f, 1.2f, (3*Subdiv)/2, (3*Subdiv)/2);
	glEndList();

	// Create display list to draw light halos
	if( haloList>0 )
		glDeleteLists(haloList, 1);		// delete previously created display list
	haloList = glGenLists(1);
	glNewList(haloList, GL_COMPILE);
	DrawSubdivHaloZ(0, 0, 0, 1, 32);
	glEndList();
}


//	Callback function associated to the 'Change lights' button of the lights tweak bar.
void TW_CALL ReinitCB(void *clientData)
{
	Scene *scene = static_cast<Scene *>(clientData); // scene pointer is stored in clientData
	scene->Init(true);								 // re-initialize the scene
}


//	Create a tweak bar for lights.
//	New enum type and struct type are defined and used by this bar.
void Scene::CreateBar()
{
	// Create a new tweak bar and change its label
	lightsBar = TwNewBar("Lights");					
	TwDefine(" Lights label='Lights TweakBar' ");

	// Add a variable of type int to control the number of lights
	TwAddVarRW(lightsBar, "NumLights", TW_TYPE_INT32, &NumLights, " label='Number of lights' keyIncr=l keyDecr=L ");
	// Set the NumLights min value (=0) and max value (depends on the user graphic card)
	char def[256];
	_snprintf(def, 255, "Lights/NumLights min=0 max=%d", maxLights);
	TwDefine(def);		 // min and max are defined using a defintion string

	// Add a button to re-initialize the lights; this button calls the ReinitCB callback function
	TwAddButton(lightsBar, "Reinit", ReinitCB, this, " label='Change lights' key=RETURN ");

	// Define a new enum type for the tweak bar
	TwEnumVal modeEV[] = // array used to describe the Scene::AnimMode enum values
	{
		{ Light::ANIM_FIXED,	"Fixed"		}, 
		{ Light::ANIM_BOUNCE,	"Bounce"	}, 
		{ Light::ANIM_ROTATE,	"Rotate"	}, 
		{ Light::ANIM_COMBINED,	"Combined"	}
	};
	TwType modeType = TwDefineEnum("Mode", modeEV, 4);	// create a new TwType associated to the enum defined by the modeEV array

	// Define a new struct type: light variables are embeded in this structure
	TwStructMember lightMembers[] =	// array used to describe tweakable variables of the Light structure
	{
		{ "Active",	   TW_TYPE_BOOLCPP, offsetof(Light, Active),	"" },			// Light::Active is a C++ boolean value
		{ "Color",	   TW_TYPE_COLOR4F, offsetof(Light, Color),		"noalpha" },	// Light::Color is represented by 4 floats, but alpha channel should be ignored
		{ "Radius",	   TW_TYPE_FLOAT,   offsetof(Light, Radius),	"min=0 max=4 step=0.02" },
		{ "Animation", modeType,        offsetof(Light, Animation),	"" },			// use the enum 'modeType' created before to tweak the Light::Animation variable
		{ "Speed",	   TW_TYPE_FLOAT,   offsetof(Light, Speed0),    "readonly" }	// Light::Speed is made read-only
	};
	TwType lightType = TwDefineStruct("Light", lightMembers, 5, sizeof(Light), NULL, NULL);	 // create a new TwType associated to the struct defined by the lightMembers array

	// Use the newly created 'lightType' to add variables associated with lights
	for(int i=0; i<maxLights; ++i)	// Add 'maxLights' variables of type lightType; unused lights variables (over NumLights) will hidden by Scene::Update( )
	{
		_snprintf(lights[i].Name, 3, "%d", i+1);	// Create a name for each light ("1", "2", "3",...)
		TwAddVarRW(lightsBar, lights[i].Name, lightType, &lights[i], "group='Edit lights'"); // Add a lightType variable and group it into the 'Edit lights' group
	}
}


//	Move lights
void Scene::Update(float time)
{
	float horizSpeed, vertSpeed;
	for(int i=0; i<NumLights; ++i)
	{
		// Change light position according to its current animation mode

		if( lights[i].Animation==Light::ANIM_ROTATE || lights[i].Animation==Light::ANIM_COMBINED )
			horizSpeed = lights[i].Speed0;
		else
			horizSpeed = 0;

		if( lights[i].Animation==Light::ANIM_BOUNCE || lights[i].Animation==Light::ANIM_COMBINED )
			vertSpeed = 1;
		else
			vertSpeed = 0;

		lights[i].Pos[0] = lights[i].Dist0 * (float)cos(horizSpeed*time + lights[i].Angle0);
		lights[i].Pos[1] = (float)fabs(cos(vertSpeed*time + lights[i].Height0));
		lights[i].Pos[2] = lights[i].Dist0 * (float)sin(horizSpeed*time + lights[i].Angle0);
		lights[i].Pos[3] = 1;
	}
}


//	Activate OpenGL lights; hide unused lights in the Lights tweak bar; 
//	and draw the scene. The scene is reflected by the ground plane, so it is
//	drawn two times: first reflected, and second normal (unreflected).
void Scene::Draw()
{
	int i;
	char def[32];	// definition string used to show/hide tweakable lights
	for(i=0; i<maxLights; ++i)
	{
		strcpy(def, "Lights/");
		strcat(def, lights[i].Name); // name of the current light
		if( i<NumLights )
		{
			// Lights under NumLights are shown in the Lights tweak bar
			strcat(def, " show");

			// Tell OpenGL to enable or disable the light
			if( lights[i].Active )
				glEnable(GL_LIGHT0+i);
			else
				glDisable(GL_LIGHT0+i);

			// Update OpenGL light parameters (for the reflected scene)
			float reflectPos[4] = { lights[i].Pos[0], -lights[i].Pos[1], lights[i].Pos[2], lights[i].Pos[3] };
			glLightfv(GL_LIGHT0+i, GL_POSITION, reflectPos);
			glLightfv(GL_LIGHT0+i, GL_DIFFUSE, lights[i].Color);
			glLightf(GL_LIGHT0+i, GL_CONSTANT_ATTENUATION, 1);
			glLightf(GL_LIGHT0+i, GL_LINEAR_ATTENUATION, 0);
			glLightf(GL_LIGHT0+i, GL_QUADRATIC_ATTENUATION, 1.0f/(lights[i].Radius*lights[i].Radius));
		}
		else
		{
			// Lights over NumLights are hidden in the Lights tweak bar
			strcat(def, " hide");

			// Disable the OpenGL light
			glDisable(GL_LIGHT0+i);
			
		}

		// Show or hide the light variable in the Lights tweak bar
		TwDefine(def);
	}

	// Set global ambient and clear screen and depth buffer
	float ambient[4] = { Ambient*(BgColor0[0]+BgColor1[0])/2, Ambient*(BgColor0[1]+BgColor1[1])/2, Ambient*(BgColor0[2]+BgColor1[2])/2, 1 };
	glClearColor(ambient[0], ambient[1], ambient[2], 1);
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);

	// Rotate the scene
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glRotatef(RotYAngle, 0, 1, 0);

	// Draw the reflected scene
	glPolygonMode(GL_FRONT_AND_BACK, (Wireframe ? GL_LINE : GL_FILL));
	glCullFace(GL_FRONT);
	glPushMatrix();
	glScalef(1, -1, 1);
	glColor3f(1, 1, 1);
	glCallList(objList);
	DrawHalos(true);
	glPopMatrix();
	glCullFace(GL_BACK);

	// clear depth buffer again
	glClear(GL_DEPTH_BUFFER_BIT);

	// Draw the ground plane (using the Reflection parameter as transparency)
	glColor4f(1, 1, 1, 1.0f-Reflection);
	glCallList(groundList);

	// Draw the gradient background (requires to switch to screen-space normalized coordinates)
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glDisable(GL_LIGHTING);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glBegin(GL_QUADS);
		glColor3f(BgColor0[0], BgColor0[1], BgColor0[2]);
		glVertex3f(-1, -1, 0.9f); 
		glVertex3f(1, -1, 0.9f); 
		glColor3f(BgColor1[0], BgColor1[1], BgColor1[2]);
		glVertex3f(1, 1, 0.9f); 
		glVertex3f(-1, 1, 0.9f);
	glEnd();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glEnable(GL_LIGHTING);

	// Update light positions for unreflected scene
	for(i=0; i<NumLights; ++i)
		glLightfv(GL_LIGHT0+i, GL_POSITION, lights[i].Pos);

	// Draw the unreflected scene
	glPolygonMode(GL_FRONT_AND_BACK, (Wireframe ? GL_LINE : GL_FILL));
	glColor3f(1, 1, 1);
	glCallList(objList);
	DrawHalos(false);
}


//	Subroutine used to draw halos around light positions
void Scene::DrawHalos(bool reflected)
{
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE);
	glDepthMask(GL_FALSE);
	float prevAmbient[4];
	glGetFloatv(GL_LIGHT_MODEL_AMBIENT, prevAmbient);
	glPushMatrix();
	glLoadIdentity();
	if( reflected )
		glScalef(1, -1 ,1);
	float black[4] = {0, 0, 0, 1};
	float cr = (float)cos(FLOAT_2PI*RotYAngle/360.0f);
	float sr = (float)sin(FLOAT_2PI*RotYAngle/360.0f);
	for(int i=0; i<NumLights; ++i)
	{
		if( lights[i].Active )
			glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lights[i].Color);
		else
			glLightModelfv(GL_LIGHT_MODEL_AMBIENT, black);
		glPushMatrix();
		glTranslatef(cr*lights[i].Pos[0]+sr*lights[i].Pos[2], lights[i].Pos[1], -sr*lights[i].Pos[0]+cr*lights[i].Pos[2]);
		//glScalef(0.5f*lights[i].Radius, 0.5f*lights[i].Radius, 1);
		glScalef(0.05f, 0.05f, 1);
		glCallList(haloList);
		glPopMatrix();
	}
	glPopMatrix();
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, prevAmbient);
	glDepthMask(GL_TRUE);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}


//	Subroutine used to build the ground plane display list (mesh subdivision is adjustable)
void Scene::DrawSubdivPlaneY(float xMin, float xMax, float y, float zMin, float zMax, int xSubdiv, int zSubdiv)
{
	const float FLOAT_EPS = 1.0e-5f;
	float dx = (xMax-xMin)/xSubdiv;
	float dz = (zMax-zMin)/zSubdiv;
	glBegin(GL_QUADS);
	glNormal3f(0, -1, 0);
	for( float z=zMin; z<zMax-FLOAT_EPS; z+=dz )
		for( float x=xMin; x<xMax-FLOAT_EPS; x+=dx )
		{
			glVertex3f(x, y, z);
			glVertex3f(x, y, z+dz);
			glVertex3f(x+dx, y, z+dz);
			glVertex3f(x+dx, y, z);
		}
	glEnd();
}


//	Subroutine used to build objects display list (mesh subdivision is adjustable)
void Scene::DrawSubdivCylinderY(float xCenter, float yBottom, float zCenter, float height, float radiusBottom, float radiusTop, int sideSubdiv, int ySubdiv)
{
	float h0, h1, y0, y1, r0, r1, a0, a1, cosa0, sina0, cosa1, sina1;
	glBegin(GL_QUADS);
	glNormal3f(0, 1, 0);
	for( int j=0; j<ySubdiv; ++j )
		for( int i=0; i<sideSubdiv; ++i )
		{
			h0 = (float)j/ySubdiv;
			h1 = (float)(j+1)/ySubdiv;
			y0 = yBottom + h0*height;
			y1 = yBottom + h1*height;
			r0 = radiusBottom + h0*(radiusTop-radiusBottom);
			r1 = radiusBottom + h1*(radiusTop-radiusBottom);
			a0 = FLOAT_2PI*(float)i/sideSubdiv;
			a1 = FLOAT_2PI*(float)(i+1)/sideSubdiv;
			cosa0 = (float)cos(a0);
			sina0 = (float)sin(a0);
			cosa1 = (float)cos(a1);
			sina1 = (float)sin(a1);
			glNormal3f(cosa0, 0, sina0);
			glVertex3f(xCenter+r0*cosa0, y0, zCenter+r0*sina0);
			glNormal3f(cosa0, 0, sina0);
			glVertex3f(xCenter+r1*cosa0, y1, zCenter+r1*sina0);
			glNormal3f(cosa1, 0, sina1);
			glVertex3f(xCenter+r1*cosa1, y1, zCenter+r1*sina1);
			glNormal3f(cosa1, 0, sina1);
			glVertex3f(xCenter+r0*cosa1, y0, zCenter+r0*sina1);

		}
	glEnd();
}


//	Subroutine used to build halo display list
void Scene::DrawSubdivHaloZ(float x, float y, float z, float radius, int subdiv)
{
	glBegin(GL_TRIANGLE_FAN);
	glNormal3f(0, 0, 0);
	glColor4f(1, 1, 1, 1);
	glVertex3f(x, y, z);
	for( int i=0; i<=subdiv; ++i )
	{
		glColor4f(1, 1, 1, 0);
		glVertex3f(x+radius*(float)cos(FLOAT_2PI*(float)i/subdiv), x+radius*(float)sin(FLOAT_2PI*(float)i/subdiv), z);
	}
	glEnd();
}


//	Callback function called by GLFW when a mouse button is clicked
void GLFWCALL OnMouseButton(int glfwButton, int glfwAction)
{
	if( !TwEventMouseButtonGLFW(glfwButton, glfwAction) )	// Send event to AntTweakBar
	{
		// Event has not been handled by AntTweakBar
		// Do something if needed.
	}
}


//	Callback function called by GLFW when mouse has moved
void GLFWCALL OnMousePos(int mouseX, int mouseY)
{
	if( !TwEventMousePosGLFW(mouseX, mouseY) )	// Send event to AntTweakBar
	{
		// Event has not been handled by AntTweakBar
		// Do something if needed.
	}
}


//	Callback function called by GLFW on mouse wheel event
void GLFWCALL OnMouseWheel(int pos)
{
	if( !TwEventMouseWheelGLFW(pos) )	// Send event to AntTweakBar
	{
		// Event has not been handled by AntTweakBar
		// Do something if needed.
	}
}


//	Callback function called by GLFW on key event
void GLFWCALL OnKey(int glfwKey, int glfwAction)
{
	if( !TwEventKeyGLFW(glfwKey, glfwAction) )	// Send event to AntTweakBar
	{
		// Event has not been handled by AntTweakBar
		// Do something if needed.
	}
}


//	Callback function called by GLFW on char event
void GLFWCALL OnChar(int glfwChar, int glfwAction)
{
	if( !TwEventCharGLFW(glfwChar, glfwAction) )	// Send event to AntTweakBar
	{
		// Event has not been handled by AntTweakBar
		// Do something if needed.
	}
}


//	Callback function called by GLFW when window size changes
void GLFWCALL OnWindowSize(int width, int height)
{
	// Set OpenGL viewport and camera
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(40, (double)width/height, 1, 10);
	gluLookAt(-0.3,1,3.5, -0.3,0,0, 0,1,0);
	glTranslated(0, -0.3, 0);
	
	// Send the new window size to AntTweakBar
	TwWindowSize(width, height);
}


//	Callback function called when the 'Subdiv' variable value of the main tweak bar has changed.
void TW_CALL SetSubdivCB(const void *value, void *clientData)
{
	Scene *scene = static_cast<Scene *>(clientData);	// scene pointer is stored in clientData
	scene->Subdiv = *static_cast<const int *>(value);	// copy value to scene->Subdiv
	scene->Init(false);									// re-init scene with the new Subdiv parameter
}


//	Callback function called by the main tweak bar to get the 'Subdiv' value
void TW_CALL GetSubdivCB(void *value, void *clientData)
{
	Scene *scene = static_cast<Scene *>(clientData);	// scene pointer is stored in clientData
	*static_cast<int *>(value) = scene->Subdiv;			// copy scene->Subdiv to value
}


//	Main function
int main() 
{
	// Intialize GLFW	
	if( !glfwInit() )
	{
		// A fatal error occured
		std::cerr << "GLFW initialization failed" << std::endl;
		return 1;
	}

	// Create a window
	GLFWvidmode mode;
	glfwGetDesktopMode(&mode);
	if( !glfwOpenWindow(800, 600, mode.RedBits, mode.GreenBits, mode.BlueBits, 0, 16, 0, GLFW_WINDOW /* or GLFW_FULLSCREEN */) )
	{
		// A fatal error occured	
		std::cerr << "Cannot open GLFW window" << std::endl;
		glfwTerminate();
		return 1;
	}
	glfwEnable(GLFW_MOUSE_CURSOR);
	glfwEnable(GLFW_KEY_REPEAT);
	glfwSetWindowTitle("AntTweakBar example: TwAdvanced1");
	// Set GLFW event callbacks
	glfwSetWindowSizeCallback(OnWindowSize);
	glfwSetMouseButtonCallback(OnMouseButton);
	glfwSetMousePosCallback(OnMousePos);
	glfwSetMouseWheelCallback(OnMouseWheel);
	glfwSetKeyCallback(OnKey);
	glfwSetCharCallback(OnChar);

	// Initialize AntTweakBar
	if( !TwInit(TW_OPENGL, NULL) )
	{
		// A fatal error occured	
		std::cerr << "AntTweakBar initialization failed: " << TwGetLastError() << std::endl;
		glfwTerminate();
		return 1;
	}

	// Initialize the 3D scene
	Scene scene;
	scene.Init(true);


	// Create a tweak bar called 'Main' and change its refresh rate and font size
	TwBar *mainBar = TwNewBar("Main");
	TwDefine(" Main label='Main TweakBar' refresh=0.5 fontSize=3 ");

	// Add some variables to the Main tweak bar
	TwAddVarRW(mainBar, "Wireframe", TW_TYPE_BOOLCPP, &scene.Wireframe, " group='Display' key=w ");	// 'Wireframe' is put in the group 'Display' (which is then created)
	TwAddVarRW(mainBar, "BgTop", TW_TYPE_COLOR3F, &scene.BgColor1, " group='Background' ");			// 'BgTop' and 'BgBottom' are put in the group 'Background' (which is then created)
	TwAddVarRW(mainBar, "BgBottom", TW_TYPE_COLOR3F, &scene.BgColor0, " group='Background' ");
	TwDefine(" Main/Background group='Display' ");	// The group 'Background' of bar 'Main' is put in the group 'Display'
	TwAddVarCB(mainBar, "Subdiv", TW_TYPE_INT32, SetSubdivCB, GetSubdivCB, &scene, " group='Scene' label='Meshs subdivision' min=1 max=50 keyincr=s keyDecr=S ");
	TwAddVarRW(mainBar, "Ambient", TW_TYPE_FLOAT, &scene.Ambient, " label='Ambient factor' group='Scene' min=0 max=1 step=0.001 keyIncr=a keyDecr=A ");
	TwAddVarRW(mainBar, "Reflection", TW_TYPE_FLOAT, &scene.Reflection, " label='Reflection factor' group='Scene' min=0 max=1 step=0.001 keyIncr=r keyDecr=R ");

	// Create a new TwType called rotationType associated with the Scene::RotMode enum, and use it
	TwEnumVal rotationEV[] = { { Scene::ROT_OFF, "Stopped"}, { Scene::ROT_CW, "Clockwise" }, { Scene::ROT_CCW, "Counter-clockwise" } };
	TwType rotationType = TwDefineEnum( "Rotation Mode", rotationEV, 3 );
	TwAddVarRW(mainBar, "Rotation", rotationType, &scene.Rotation, " group='Scene' keyIncr=Backspace keyDecr=SHIFT+Backspace ");

	// Add a read-only float variable; its precision is 0 which means that the fractionnal part of the float value will not be displayed
	TwAddVarRO(mainBar, "RotYAngle", TW_TYPE_FLOAT, &scene.RotYAngle, " group='Scene' label='Rot angle (degree)' precision=0 ");

	// Initialize time
	float time = (float)glfwGetTime(), dt = 0; 	// Current time and elapsed time
	// Main loop (repeated while window is not closed and [ESC] is not pressed)
	while( glfwGetWindowParam(GLFW_OPENED) && !glfwGetKey(GLFW_KEY_ESC) )
	{
		// Compute elapsed time
		dt = (float)glfwGetTime() - time;
		if( dt>0.1f )
			dt = 0.1f;
		time += dt;

		// Rotate scene
		if( scene.Rotation==Scene::ROT_CW )
			scene.RotYAngle -= 5.0f*dt;
		else if( scene.Rotation==Scene::ROT_CCW )
			scene.RotYAngle += 5.0f*dt;

		// Move lights
		scene.Update(time);

		// Draw scene
		scene.Draw();

		// Draw tweak bars
		TwDraw();

		// Present frame buffer
		glfwSwapBuffers();
	}

	// Terminate AntTweakBar and GLFW
	TwTerminate();
	glfwTerminate();
	
	return 0;
}
