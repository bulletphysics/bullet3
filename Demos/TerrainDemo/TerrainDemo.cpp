
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006,2008 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "TerrainDemo.h"		// always include our own header first!

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"

#include "GLDebugDrawer.h"

#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"
#include "GLDebugFont.h"



// constants -------------------------------------------------------------------
static const float s_gravity			= 9.8;		// 9.8 m/s^2

static const int s_gridSize			= 64 + 1;  // must be (2^N) + 1
static const float s_gridSpacing		= 5.0;

static const float s_gridHeightScale		= 0.2;

// the singularity at the center of the radial model means we need a lot of
//   finely-spaced time steps to get the physics right.
// These numbers are probably too aggressive for a real game!
static const int s_requestedHz			= 180;
static const float s_engineTimeStep		= 1.0 / s_requestedHz;

// delta phase: radians per second
static const float s_deltaPhase			= 0.25 * 2.0 * SIMD_PI;

// what type of terrain is generated?
enum eTerrainModel {
	eRadial			= 1,	// deterministic
	eFractal		= 2	// random
};


typedef unsigned char byte_t;



////////////////////////////////////////////////////////////////////////////////
//
//	static helper methods
//
//	Only used within this file (helpers and terrain generation, etc)
//
////////////////////////////////////////////////////////////////////////////////

static const char *
getTerrainTypeName
(
eTerrainModel model
)
{
	switch (model) {
	case eRadial:
		return "Radial";

	case eFractal:
		return "Fractal";

	default:
		btAssert(!"bad terrain model type");
	}

	return NULL;
}



static const char *
getDataTypeName
(
PHY_ScalarType type
)
{
	switch (type) {
	case PHY_UCHAR:
		return "UnsignedChar";

	case PHY_SHORT:
		return "Short";

	case PHY_FLOAT:
		return "Float";

	default:
		btAssert(!"bad heightfield data type");
	}

	return NULL;
}



static const char *
getUpAxisName
(
int axis
)
{
	switch (axis) {
	case 0:
		return "X";

	case 1:
		return "Y";

	case 2:
		return "Z";

	default:
		btAssert(!"bad up axis");
	}

	return NULL;
}



static btVector3
getUpVector
(
int upAxis,
btScalar regularValue,
btScalar upValue
)
{
	btAssert(upAxis >= 0 && upAxis <= 2 && "bad up axis");

	btVector3 v(regularValue, regularValue, regularValue);
	v[upAxis] = upValue;

	return v;
}



// TODO: it would probably cleaner to have a struct per data type, so
// 	you could lookup byte sizes, conversion functions, etc.
static int getByteSize
(
PHY_ScalarType type
)
{
	int size = 0;

	switch (type) {
	case PHY_FLOAT:
		size = sizeof(float);
		break;

	case PHY_UCHAR:
		size = sizeof(unsigned char);
		break;

	case PHY_SHORT:
		size = sizeof(short);
		break;

	default:
		btAssert(!"Bad heightfield data type");
	}

	return size;
}



static float
convertToFloat
(
const byte_t * p,
PHY_ScalarType type
)
{
	btAssert(p);

	switch (type) {
	case PHY_FLOAT:
		{
			float * pf = (float *) p;
			return *pf;
		}

	case PHY_UCHAR:
		{
			unsigned char * pu = (unsigned char *) p;
			return ((*pu) * s_gridHeightScale);
		}

	case PHY_SHORT:
		{
			short * ps = (short *) p;
			return ((*ps) * s_gridHeightScale);
		}

	default:
		btAssert(!"bad type");
	}

	return 0;
}



static float
getGridHeight
(
byte_t * grid,
int i,
int j,
PHY_ScalarType type
)
{
	btAssert(grid);
	btAssert(i >= 0 && i < s_gridSize);
	btAssert(j >= 0 && j < s_gridSize);

	int bpe = getByteSize(type);
	btAssert(bpe > 0 && "bad bytes per element");

	int idx = (j * s_gridSize) + i;
	long offset = ((long) bpe) * idx;

	byte_t * p = grid + offset;

	return convertToFloat(p, type);
}



static void
convertFromFloat
(
byte_t * p,
float value,
PHY_ScalarType type
)
{
	btAssert(p && "null");

	switch (type) {
	case PHY_FLOAT:
		{
			float * pf = (float *) p;
			*pf = value;
		}
		break;

	case PHY_UCHAR:
		{
			unsigned char * pu = (unsigned char *) p;
			*pu = (unsigned char) (value / s_gridHeightScale);
		}
		break;

	case PHY_SHORT:
		{
			short * ps = (short *) p;
			*ps = (short) (value / s_gridHeightScale);
		}
		break;

	default:
		btAssert(!"bad type");
	}
}



// creates a radially-varying heightfield
static void
setRadial
(
byte_t * grid,
int bytesPerElement,
PHY_ScalarType type,
float phase = 0.0
)
{
	btAssert(grid);
	btAssert(bytesPerElement > 0);

	// min/max
	float period = 0.5 / s_gridSpacing;
	float floor = 0.0;
	float min_r = 3.0 * sqrt(s_gridSpacing);
	float magnitude = 50.0 * sqrt(s_gridSpacing);

	// pick a base_phase such that phase = 0 results in max height
	//   (this way, if you create a heightfield with phase = 0,
	//    you can rely on the min/max heights that result)
	float base_phase = (0.5 * SIMD_PI) - (period * min_r);
	phase += base_phase;

	// center of grid
	float cx = 0.5 * s_gridSize * s_gridSpacing;
	float cy = cx;		// assume square grid
	byte_t * p = grid;
	for (int i = 0; i < s_gridSize; ++i) {
		float x = i * s_gridSpacing;
		for (int j = 0; j < s_gridSize; ++j) {
			float y = j * s_gridSpacing;

			float dx = x - cx;
			float dy = y - cy;

			float r = sqrt((dx * dx) + (dy * dy));

			float z = period;
			if (r < min_r) {
				r = min_r;
			}
			z = (1.0 / r) * sin(period * r + phase);
			if (z > period) {
				z = period;
			} else if (z < -period) {
				z = -period;
			}
			z = floor + magnitude * z;

			convertFromFloat(p, z, type);
			p += bytesPerElement;
		}
	}
}



static float
randomHeight
(
int step
)
{
	return (0.33 * s_gridSpacing * s_gridSize * step * (rand() - (0.5 * RAND_MAX))) / (1.0 * RAND_MAX * s_gridSize);
}



static void
dumpGrid
(
const byte_t * grid,
int bytesPerElement,
PHY_ScalarType type,
int max
)
{
	//std::cerr << "Grid:\n";

	char buffer[32];

	for (int j = 0; j < max; ++j) {
		for (int i = 0; i < max; ++i) {
			long offset = j * s_gridSize + i;
			float z = convertToFloat(grid + offset * bytesPerElement, type);
			sprintf(buffer, "%6.2f", z);
			//std::cerr << "  " << buffer;
		}
		//std::cerr << "\n";
	}
}



static void
updateHeight
(
byte_t * p,
float new_val,
PHY_ScalarType type
)
{
	float old_val = convertToFloat(p, type);
	if (!old_val) {
		convertFromFloat(p, new_val, type);
	}
}



// creates a random, fractal heightfield
static void
setFractal
(
byte_t * grid,
int bytesPerElement,
PHY_ScalarType type,
int step
)
{
	btAssert(grid);
	btAssert(bytesPerElement > 0);
	btAssert(step > 0);
	btAssert(step < s_gridSize);

	int newStep = step / 2;
//	std::cerr << "Computing grid with step = " << step << ": before\n";
//	dumpGrid(grid, bytesPerElement, type, step + 1);

	// special case: starting (must set four corners)
	if (s_gridSize - 1 == step) {
		// pick a non-zero (possibly negative) base elevation for testing
		float base = randomHeight(step / 2);

		convertFromFloat(grid, base, type);
		convertFromFloat(grid + step * bytesPerElement, base, type);
		convertFromFloat(grid + step * s_gridSize * bytesPerElement, base, type);
		convertFromFloat(grid + (step * s_gridSize + step) * bytesPerElement, base, type);
	}

	// determine elevation of each corner
	float c00 = convertToFloat(grid, type);
	float c01 = convertToFloat(grid + step * bytesPerElement, type);
	float c10 = convertToFloat(grid + (step * s_gridSize) * bytesPerElement, type);
	float c11 = convertToFloat(grid + (step * s_gridSize + step) * bytesPerElement, type);

	// set top middle
	updateHeight(grid + newStep * bytesPerElement, 0.5 * (c00 + c01) + randomHeight(step), type);

	// set left middle
	updateHeight(grid + (newStep * s_gridSize) * bytesPerElement, 0.5 * (c00 + c10) + randomHeight(step), type);

	// set right middle
	updateHeight(grid + (newStep * s_gridSize + step) * bytesPerElement, 0.5 * (c01 + c11) + randomHeight(step), type);

	// set bottom middle
	updateHeight(grid + (step * s_gridSize + newStep) * bytesPerElement, 0.5 * (c10 + c11) + randomHeight(step), type);

	// set middle
	updateHeight(grid + (newStep * s_gridSize + newStep) * bytesPerElement, 0.25 * (c00 + c01 + c10 + c11) + randomHeight(step), type);

//	std::cerr << "Computing grid with step = " << step << ": after\n";
//	dumpGrid(grid, bytesPerElement, type, step + 1);

	// terminate?
	if (newStep < 2) {
		return;
	}

	// recurse
	setFractal(grid, bytesPerElement, type, newStep);
	setFractal(grid + newStep * bytesPerElement, bytesPerElement, type, newStep);
	setFractal(grid + (newStep * s_gridSize) * bytesPerElement, bytesPerElement, type, newStep);
	setFractal(grid + ((newStep * s_gridSize) + newStep) * bytesPerElement, bytesPerElement, type, newStep);
}



static byte_t *
getRawHeightfieldData
(
eTerrainModel model,
PHY_ScalarType type,
btScalar& minHeight,
btScalar& maxHeight
)
{
//	std::cerr << "\nRegenerating terrain\n";
//	std::cerr << "  model = " << model << "\n";
//	std::cerr << "  type = " << type << "\n";

	long nElements = ((long) s_gridSize) * s_gridSize;
//	std::cerr << "  nElements = " << nElements << "\n";

	int bytesPerElement = getByteSize(type);
//	std::cerr << "  bytesPerElement = " << bytesPerElement << "\n";
	btAssert(bytesPerElement > 0 && "bad bytes per element");

	long nBytes = nElements * bytesPerElement;
//	std::cerr << "  nBytes = " << nBytes << "\n";
	byte_t * raw = new byte_t[nBytes];
	btAssert(raw && "out of memory");

	// reseed randomization every 30 seconds
//	srand(time(NULL) / 30);

	// populate based on model
	switch (model) {
	case eRadial:
		setRadial(raw, bytesPerElement, type);
		break;

	case eFractal:
		for (int i = 0; i < nBytes; i++)
		{
			raw[i] = 0;
		}
		setFractal(raw, bytesPerElement, type, s_gridSize - 1);
		break;

	default:
		btAssert(!"bad model type");
	}

	if (0) {
		// inside if(0) so it keeps compiling but isn't
		// 	exercised and doesn't cause warnings
//		std::cerr << "final grid:\n";
		dumpGrid(raw, bytesPerElement, type, s_gridSize - 1);
	}

	// find min/max
	for (int i = 0; i < s_gridSize; ++i) {
		for (int j = 0; j < s_gridSize; ++j) {
			float z = getGridHeight(raw, i, j, type);
//			std::cerr << "i=" << i << ", j=" << j << ": z=" << z << "\n";

			// update min/max
			if (!i && !j) {
				minHeight = z;
				maxHeight = z;
			} else {
				if (z < minHeight) {
					minHeight = z;
				}
				if (z > maxHeight) {
					maxHeight = z;
				}
			}
		}
	}

	if (maxHeight < -minHeight) {
		maxHeight = -minHeight;
	}
	if (minHeight > -maxHeight) {
		minHeight = -maxHeight;
	}

//	std::cerr << "  minHeight = " << minHeight << "\n";
//	std::cerr << "  maxHeight = " << maxHeight << "\n";

	return raw;
}



////////////////////////////////////////////////////////////////////////////////
//
//	TerrainDemo class
//
////////////////////////////////////////////////////////////////////////////////

/// class that demonstrates the btHeightfieldTerrainShape object
class TerrainDemo : public GlutDemoApplication {
public:
	// constructor, destructor ---------------------------------------------
	TerrainDemo(void);
	~TerrainDemo(void);

	virtual void initPhysics() {}

	// public class methods ------------------------------------------------
	void initialize(void);

	// DemoApplication class interface methods -----------------------------
	void clientMoveAndDisplay(void);
	void keyboardCallback(unsigned char key, int x, int y);
	void renderme(void);

private:
	// private helper methods ----------------------------------------------
	void resetPhysics(void);
	void clearWorld(void);

	// private data members ------------------------------------------------
	btDefaultCollisionConfiguration *	m_collisionConfiguration;
	btCollisionDispatcher *			m_dispatcher;
	btAxisSweep3 *				m_overlappingPairCache;
	btSequentialImpulseConstraintSolver *	m_constraintSolver;
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
	int					m_upAxis;
	PHY_ScalarType				m_type;
	eTerrainModel				m_model;
	byte_t *				m_rawHeightfieldData;
	btScalar				m_minHeight;
	btScalar				m_maxHeight;
	float					m_phase;	// for dynamics
	bool					m_isDynamic;
};



TerrainDemo::TerrainDemo(void)
:
m_collisionConfiguration(NULL),
m_dispatcher(NULL),
m_overlappingPairCache(NULL),
m_constraintSolver(NULL),
m_upAxis(1),
m_type(PHY_FLOAT),
m_model(eFractal),
m_rawHeightfieldData(NULL),
m_phase(0.0),
m_isDynamic(true)
{
}



TerrainDemo::~TerrainDemo(void)
{
	clearWorld();

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_constraintSolver;

	//delete broadphase
	delete m_overlappingPairCache;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;

}



////////////////////////////////////////////////////////////////////////////////
//
//	TerrainDemo -- public class methods
//
////////////////////////////////////////////////////////////////////////////////

/// one-time class and physics initialization
void TerrainDemo::initialize(void)
{
//	std::cerr << "initializing...\n";

	// set up basic state
	m_upAxis = 1;		// start with Y-axis as "up"
	m_type = PHY_FLOAT;
	m_model = eRadial;//eFractal;
	m_isDynamic = true;

	// set up the physics world
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	m_overlappingPairCache = new btAxisSweep3(worldMin,worldMax);
	m_constraintSolver = new btSequentialImpulseConstraintSolver();
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_constraintSolver,m_collisionConfiguration);

	// initialize axis- or type-dependent physics from here
	this->resetPhysics();
}



////////////////////////////////////////////////////////////////////////////////
//
//	TerrainDemo -- DemoApplication class interface methods
//
////////////////////////////////////////////////////////////////////////////////

void TerrainDemo::clientMoveAndDisplay(void)
{
	// elapsed time
	float us = getDeltaTimeMicroseconds();
	float seconds = 1.0e-6 * us;

	// we'll carefully iterate through each time step so we can update
	//   the dynamic model if necessary
	long nStepsPerIteration = 1;
	while (seconds > 1.0e-6) {
		float dt = nStepsPerIteration * s_engineTimeStep;
		if (dt > seconds) {
			dt = seconds;
		}
		seconds -= dt;
	//	std::cerr << "  Stepping through " << dt << " seconds\n";

		// if dynamic and radial, go ahead and update the field
		if (m_rawHeightfieldData && m_isDynamic && eRadial == m_model) {
			m_phase += s_deltaPhase * dt;
			if (m_phase > 2.0 * SIMD_PI) {
				m_phase -= 2.0 * SIMD_PI;
			}
			int bpe = getByteSize(m_type);
			btAssert(bpe > 0 && "Bad bytes per element");
			setRadial(m_rawHeightfieldData, bpe, m_type, m_phase);
		}

		if (m_dynamicsWorld) {
			m_dynamicsWorld->stepSimulation(dt,
			    nStepsPerIteration + 1, s_engineTimeStep);
		}
	}

	// okay, render
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	renderme();
	glFlush();
	glutSwapBuffers();
}


static PHY_ScalarType nextType (PHY_ScalarType type)
{
	switch (type)
	{
	case PHY_FLOAT:
		return PHY_SHORT;
	break;
	case PHY_SHORT:
		return PHY_UCHAR;
	break;
	case PHY_UCHAR:
		return PHY_FLOAT;
	break;
	}
	btAssert (0);
	return PHY_FLOAT;
}

void TerrainDemo::keyboardCallback(unsigned char key, int x, int y) {

	if (',' == key) {
		// increment model
		m_model = (eFractal == m_model) ? eRadial : eFractal;
		this->resetPhysics();
	}
	if ('/' == key) {
		// increment type
		m_type = nextType(m_type);
		this->resetPhysics();
	}
	if ('\\' == key) {
		// increment axis
		m_upAxis++;
		if (m_upAxis > 2) {
			m_upAxis = 0;
		}
		this->resetPhysics();
	}
	if ('[' == key) {
		// toggle dynamics
		m_isDynamic = !m_isDynamic;
	}

	// let demo base class handle!
	DemoApplication::keyboardCallback(key, x, y);
}



static void doPrint(int x,int& y,int dy,const char * text)
{
	GLDebugDrawString(x,y, text);
	y += dy;
}



/// override the default display just so we can overlay a bit more text
void TerrainDemo::renderme(void)
{
	// give base class a shot
	DemoApplication::renderme();

	// overlay any debug information
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	// switch to orthographic
	setOrthographicProjection();

	// we'll draw on the right top of the screen
	const int lineWidth = 200;
	const int lineHeight = 16;
	char buffer[256];

	int xStart = m_glutScreenWidth - lineWidth;
	int yStart = lineHeight;

	sprintf(buffer, "Terrain Type: %s", getTerrainTypeName(m_model));
	doPrint(xStart, yStart, lineHeight, buffer);
	doPrint(xStart, yStart, lineHeight, "Press ',' to cycle terrain types");
	doPrint(xStart, yStart, lineHeight, "");

	sprintf(buffer, "Data Type: %s", getDataTypeName(m_type));
	doPrint(xStart, yStart, lineHeight, buffer);
	doPrint(xStart, yStart, lineHeight, "Press '/' to cycle data types");
	doPrint(xStart, yStart, lineHeight, "");

	sprintf(buffer, "'up' axis: %s", getUpAxisName(m_upAxis));
	doPrint(xStart, yStart, lineHeight, buffer);
	doPrint(xStart, yStart, lineHeight, "Press '\\' to cycle 'up' axes");
	doPrint(xStart, yStart, lineHeight, "");

	if (eRadial == m_model) {
		sprintf(buffer, "Dynamic: %s", m_isDynamic ? "yes" : "no");
		doPrint(xStart, yStart, lineHeight, buffer);
		doPrint(xStart, yStart, lineHeight, "Press '[' to toggle dynamics");
	}
}



////////////////////////////////////////////////////////////////////////////////
//
//	TerrainDemo -- private helper methods
//
////////////////////////////////////////////////////////////////////////////////

/// called whenever key terrain attribute is changed
void TerrainDemo::resetPhysics(void)
{
	// remove old heightfield
	clearWorld();

	// reset gravity to point in appropriate direction
	m_dynamicsWorld->setGravity(getUpVector(m_upAxis, 0.0, -s_gravity));

	// get new heightfield of appropriate type
	m_rawHeightfieldData =
	    getRawHeightfieldData(m_model, m_type, m_minHeight, m_maxHeight);
	btAssert(m_rawHeightfieldData && "failed to create raw heightfield");

	bool flipQuadEdges = false;
	btHeightfieldTerrainShape * heightfieldShape =
	    new btHeightfieldTerrainShape(s_gridSize, s_gridSize,
					  m_rawHeightfieldData,
					  s_gridHeightScale,
					  m_minHeight, m_maxHeight,
					  m_upAxis, m_type, flipQuadEdges);
	btAssert(heightfieldShape && "null heightfield");

	// scale the shape
	btVector3 localScaling = getUpVector(m_upAxis, s_gridSpacing, 1.0);
	heightfieldShape->setLocalScaling(localScaling);

	// stash this shape away
	m_collisionShapes.push_back(heightfieldShape);

	// set origin to middle of heightfield
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0,-20,0));

	// create ground object
	float mass = 0.0;
	localCreateRigidBody(mass, tr, heightfieldShape);
}


/// removes all objects and shapes from the world
void TerrainDemo::clearWorld(void)
{
	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	// delete raw heightfield data
	delete m_rawHeightfieldData;
	m_rawHeightfieldData = NULL;
}



////////////////////////////////////////////////////////////////////////////////
//
//	TerrainDemo -- public API (exposed in header)
//
////////////////////////////////////////////////////////////////////////////////

/// creates an object that demonstrates terrain
GlutDemoApplication * btCreateTerrainDemo(void)
{
	TerrainDemo * demo = new TerrainDemo;
	btAssert(demo && "out of memory");

	demo->initialize();

	return demo;
}

