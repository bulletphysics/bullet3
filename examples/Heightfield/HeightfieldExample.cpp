
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006,2008 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "HeightfieldExample.h"		// always include our own header first!

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../MultiThreadedDemo/CommonRigidBodyMTBase.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include "../OpenGLWindow/GLInstanceGraphicsShape.h"
#include "../Utils/b3BulletDefaultFileIO.h"
#include "../Importers/ImportURDFDemo/urdfStringSplit.h"
#include "stb_image/stb_image.h"

// constants -------------------------------------------------------------------
static const btScalar s_gravity = 9.8;		// 9.8 m/s^2

static int s_gridSize = 16 + 1;  // must be (2^N) + 1
static btScalar s_gridSpacing = 0.5;
static btScalar s_gridHeightScale = 0.02;

// the singularity at the center of the radial model means we need a lot of
//   finely-spaced time steps to get the physics right.
// These numbers are probably too aggressive for a real game!

// delta phase: radians per second
static const btScalar s_deltaPhase = 0.25 * 2.0 * SIMD_PI;

// what type of terrain is generated?
enum eTerrainModel {
	eRadial = 0,	// deterministic
	eFractal = 1,	// random
	eCSVFile = 2,//csv file used in DeepLoco for example
	eImageFile = 3,//terrain from png/jpg files, asset from https://www.beamng.com/threads/tutorial-adding-heightmap-roads-using-blender.16356/

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
    case eCSVFile:
        return "DeepLocoCSV";
    case eImageFile:
        return "Image";
	default:
		btAssert(!"bad terrain model type");
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
		size = sizeof(btScalar);
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



static btScalar
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
		btScalar * pf = (btScalar *)p;
		return *pf;
	}

	case PHY_UCHAR:
	{
		unsigned char * pu = (unsigned char *)p;
		return ((*pu) * s_gridHeightScale);
	}

	case PHY_SHORT:
	{
		short * ps = (short *)p;
		return ((*ps) * s_gridHeightScale);
	}

	default:
		btAssert(!"bad type");
	}

	return 0;
}



static btScalar
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
	long offset = ((long)bpe) * idx;

	byte_t * p = grid + offset;

	return convertToFloat(p, type);
}



static void
convertFromFloat
(
	byte_t * p,
	btScalar value,
	PHY_ScalarType type
)
{
	btAssert(p && "null");

	switch (type) {
	case PHY_FLOAT:
	{
		btScalar * pf = (btScalar *)p;
		*pf = value;
	}
	break;

	case PHY_UCHAR:
	{
		unsigned char * pu = (unsigned char *)p;
		*pu = (unsigned char)(value / s_gridHeightScale);
	}
	break;

	case PHY_SHORT:
	{
		short * ps = (short *)p;
		*ps = (short)(value / s_gridHeightScale);
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
	btScalar phase = 0.0
)
{
	btAssert(grid);
	btAssert(bytesPerElement > 0);

	// min/max
	btScalar period = 0.5 / s_gridSpacing;
	btScalar floor = 0.0;
	btScalar min_r = 3.0 * btSqrt(s_gridSpacing);
	btScalar magnitude = 5.0 * btSqrt(s_gridSpacing);

	// pick a base_phase such that phase = 0 results in max height
	//   (this way, if you create a heightfield with phase = 0,
	//    you can rely on the min/max heights that result)
	btScalar base_phase = (0.5 * SIMD_PI) - (period * min_r);
	phase += base_phase;

	// center of grid
	btScalar cx = 0.5 * s_gridSize * s_gridSpacing;
	btScalar cy = cx;		// assume square grid
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
			}
			else if (z < -period) {
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


#if 0
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
#endif


static void
updateHeight
(
	byte_t * p,
	btScalar new_val,
	PHY_ScalarType type
)
{
	btScalar old_val = convertToFloat(p, type);
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
		btScalar base = randomHeight(step / 2);

		convertFromFloat(grid, base, type);
		convertFromFloat(grid + step * bytesPerElement, base, type);
		convertFromFloat(grid + step * s_gridSize * bytesPerElement, base, type);
		convertFromFloat(grid + (step * s_gridSize + step) * bytesPerElement, base, type);
	}

	// determine elevation of each corner
	btScalar c00 = convertToFloat(grid, type);
	btScalar c01 = convertToFloat(grid + step * bytesPerElement, type);
	btScalar c10 = convertToFloat(grid + (step * s_gridSize) * bytesPerElement, type);
	btScalar c11 = convertToFloat(grid + (step * s_gridSize + step) * bytesPerElement, type);

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


#define MYLINELENGTH 16*32768

static byte_t *
getRawHeightfieldData
(
	eTerrainModel model,
	PHY_ScalarType type,
	btScalar& minHeight,
	btScalar& maxHeight
)
{

    if (model==eImageFile)
    {

        b3BulletDefaultFileIO fileIO;
        char relativeFileName[1024];
        int found = fileIO.findFile("heightmaps/wm_height_out.png", relativeFileName, 1024);
        

        b3AlignedObjectArray<char> buffer;
        buffer.reserve(1024);
        int fileId = fileIO.fileOpen(relativeFileName,"rb");
        if (fileId>=0)
        {
            int size = fileIO.getFileSize(fileId);
            if (size>0)
            {
                buffer.resize(size);
                int actual = fileIO.fileRead(fileId,&buffer[0],size);
                if (actual != size)
                {
                    b3Warning("STL filesize mismatch!\n");
                    buffer.resize(0);
                }
            }
            fileIO.fileClose(fileId);
        }

        if (buffer.size())
        {
            int width, height,n;

            unsigned char* image = stbi_load_from_memory((const unsigned char*)&buffer[0], buffer.size(), &width, &height, &n, 3);
            if (image)
            {
                printf("width=%d, height=%d at %d channels\n", width,height, n);
                s_gridSize = width;
                s_gridSpacing = 0.2;
                s_gridHeightScale = 0.2;
                fileIO.fileClose(fileId);
                long nElements = ((long)s_gridSize) * s_gridSize;
                //	std::cerr << "  nElements = " << nElements << "\n";

                int bytesPerElement = getByteSize(type);
                //	std::cerr << "  bytesPerElement = " << bytesPerElement << "\n";
                btAssert(bytesPerElement > 0 && "bad bytes per element");

                long nBytes = nElements * bytesPerElement;
                //	std::cerr << "  nBytes = " << nBytes << "\n";
                byte_t * raw = new byte_t[nBytes];
                btAssert(raw && "out of memory");

                byte_t * p = raw;
                
				for (int j = 0; j < width; ++j)
                {
                    
					for (int i = 0; i < width; ++i)
                    {
						float x = i * s_gridSpacing;
                        float y = j * s_gridSpacing;
						float heightScaling = (14. / 256.);
						float z = double(image[(width - 1 - i) * 3 + width*j * 3]) * heightScaling;
                        convertFromFloat(p, z, type);
						// update min/max
						if (!i && !j) {
							minHeight = z;
							maxHeight = z;
						}
						else {
							if (z < minHeight) {
								minHeight = z;
							}
							if (z > maxHeight) {
								maxHeight = z;
							}
						}

                        p += bytesPerElement;
                    }
                }
                return raw;

            }

        }





    }

    if (model==eCSVFile)
    {
        {
            b3BulletDefaultFileIO fileIO;
            char relativePath[1024];
            int found = fileIO.findFile("heightmaps/ground0.txt", relativePath, 1024);
            char lineBuffer[MYLINELENGTH];
            int slot = fileIO.fileOpen(relativePath, "r");
            int rows = 0;
            int cols=0;

            btAlignedObjectArray<double> allValues;
            if (slot>=0)
            {
                char* lineChar;
                while (lineChar = fileIO.readLine(slot, lineBuffer, MYLINELENGTH))
                {
                    rows=0;
                    char** values = urdfStrSplit(lineChar, ",");
                    if (values)
                    {
                        int index = 0;
                        char* value;
                        while (value = values[index++])
                        {
                            std::string strval(value);
                            double v;
                            if(sscanf(value, "%lf", &v) == 1)
                            {
                                //printf("strlen = %d\n", strval.length());
                                //printf("value[%d,%d]=%s or (%f)", cols,rows,value, v);
                                allValues.push_back(v);
                                rows++;
                            }
                        }
                    }
                    cols++;

                }
                printf("done, rows=%d, cols=%d\n", rows, cols);
                int width = rows-1;
                s_gridSize = rows;
                s_gridSpacing = 0.2;
                s_gridHeightScale = 0.2;
                fileIO.fileClose(slot);
                long nElements = ((long)s_gridSize) * s_gridSize;
                //	std::cerr << "  nElements = " << nElements << "\n";

                int bytesPerElement = getByteSize(type);
                //	std::cerr << "  bytesPerElement = " << bytesPerElement << "\n";
                btAssert(bytesPerElement > 0 && "bad bytes per element");

                long nBytes = nElements * bytesPerElement;
                //	std::cerr << "  nBytes = " << nBytes << "\n";
                byte_t * raw = new byte_t[nBytes];
                btAssert(raw && "out of memory");

                byte_t * p = raw;
                for (int i = 0; i < width; ++i)
                {
                    float x = i * s_gridSpacing;
                    for (int j = 0; j < width; ++j)
                    {
                        float y = j * s_gridSpacing;
                        float z = allValues[i+width*j];
                        convertFromFloat(p, z, type);
						// update min/max
						if (!i && !j) {
							minHeight = z;
							maxHeight = z;
						}
						else {
							if (z < minHeight) {
								minHeight = z;
							}
							if (z > maxHeight) {
								maxHeight = z;
							}
						}
                        p += bytesPerElement;
                    }
                }
                return raw;
            }
            printf("found=%d",found);
        }
    } else
    {
        if (model==eRadial)
        {
            s_gridSize = 16 + 1;  // must be (2^N) + 1
            s_gridSpacing = 0.5;
            s_gridHeightScale = 0.02;
        } else
        {
            s_gridSize = 256 + 1;  // must be (2^N) + 1
            s_gridSpacing = 0.5;
            s_gridHeightScale = 0.02;
        }
        //	std::cerr << "\nRegenerating terrain\n";
        //	std::cerr << "  model = " << model << "\n";
        //	std::cerr << "  type = " << type << "\n";

        long nElements = ((long)s_gridSize) * s_gridSize;
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

        //		std::cerr << "final grid:\n";
        //dumpGrid(raw, bytesPerElement, type, s_gridSize - 1);

        // find min/max
        for (int i = 0; i < s_gridSize; ++i) {
            for (int j = 0; j < s_gridSize; ++j) {
                btScalar z = getGridHeight(raw, i, j, type);
                //			std::cerr << "i=" << i << ", j=" << j << ": z=" << z << "\n";

                // update min/max
                if (!i && !j) {
                    minHeight = z;
                    maxHeight = z;
                }
                else {
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
	return 0;
}



////////////////////////////////////////////////////////////////////////////////
//
//	TerrainDemo class
//
////////////////////////////////////////////////////////////////////////////////

/// class that demonstrates the btHeightfieldTerrainShape object
class HeightfieldExample : public CommonRigidBodyMTBase//CommonRigidBodyBase
{
public:
	// constructor, destructor ---------------------------------------------
	HeightfieldExample(struct GUIHelperInterface* helper);
	virtual ~HeightfieldExample();

	virtual void initPhysics();

	// public class methods ------------------------------------------------

	void castRays();

	void stepSimulation(float deltaTime);

	void resetCamera()
	{
		float dist = 15;
		float pitch = -32;
		float yaw = 35;
		float targetPos[3] = { 0, 0, 0 };
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

private:
	// private helper methods ----------------------------------------------
	void resetPhysics(void);
	void clearWorld(void);

	// private data members ------------------------------------------------
	int					m_upAxis;
	PHY_ScalarType				m_type;
	eTerrainModel				m_model;
	byte_t *				m_rawHeightfieldData;
	btScalar				m_minHeight;
	btScalar				m_maxHeight;
	float					m_phase;	// for dynamics
	bool					m_isDynamic;
	btHeightfieldTerrainShape * m_heightfieldShape;
};


#define HEIGHTFIELD_TYPE_COUNT 4
eTerrainModel gHeightfieldType = eRadial;

void setHeightfieldTypeComboBoxCallback(int combobox, const char* item, void* userPointer)
{
	const char** items = static_cast<const char**>(userPointer);
	for (int i = 0; i < HEIGHTFIELD_TYPE_COUNT; ++i)
	{
		if (strcmp(item, items[i]) == 0)
		{
			gHeightfieldType = static_cast<eTerrainModel>(i);
			break;
		}
	}
}



HeightfieldExample::HeightfieldExample(struct GUIHelperInterface* helper)
	: CommonRigidBodyMTBase(helper),
	m_upAxis(1),
	m_type(PHY_FLOAT),
	m_model(eFractal),
	m_rawHeightfieldData(NULL),
	m_phase(0.0),
	m_isDynamic(true),
	m_heightfieldShape(0)
{
	{
		// create a combo box for selecting the solver type
		static const char* sHeightfieldTypeComboBoxItems[HEIGHTFIELD_TYPE_COUNT];
		for (int i = 0; i < HEIGHTFIELD_TYPE_COUNT; ++i)
		{
			eTerrainModel heightfieldType = static_cast<eTerrainModel>(i);
			sHeightfieldTypeComboBoxItems[i] = getTerrainTypeName(heightfieldType);
		}
		ComboBoxParams comboParams;
		comboParams.m_userPointer = sHeightfieldTypeComboBoxItems;
		comboParams.m_numItems = HEIGHTFIELD_TYPE_COUNT;
		comboParams.m_startItem = gHeightfieldType;
		comboParams.m_items = sHeightfieldTypeComboBoxItems;
		comboParams.m_callback = setHeightfieldTypeComboBoxCallback;
		m_guiHelper->getParameterInterface()->registerComboBox(comboParams);
	}
}



HeightfieldExample::~HeightfieldExample(void)
{
	clearWorld();


}


class MyTriangleCollector3 : public btTriangleCallback
{
public:
	btAlignedObjectArray<GLInstanceVertex>* m_pVerticesOut;
	btAlignedObjectArray<int>* m_pIndicesOut;

	MyTriangleCollector3()
	{
		m_pVerticesOut = 0;
		m_pIndicesOut = 0;
	}

	virtual void processTriangle(btVector3* tris, int partId, int triangleIndex)
	{
		for (int k = 0; k < 3; k++)
		{
			GLInstanceVertex v;
			v.xyzw[3] = 0;
			v.uv[0] = v.uv[1] = 0.5f;
			btVector3 normal = (tris[0] - tris[1]).cross(tris[0] - tris[2]);
			normal.safeNormalize();
			for (int l = 0; l < 3; l++)
			{
				v.xyzw[l] = tris[k][l];
				v.normal[l] = normal[l];
			}
			m_pIndicesOut->push_back(m_pVerticesOut->size());
			m_pVerticesOut->push_back(v);
		}
	}
};


#define NUMRAYS2 500
#define USE_PARALLEL_RAYCASTS 1

class btRaycastBar3
{
public:
	btVector3 source[NUMRAYS2];
	btVector3 dest[NUMRAYS2];
	btVector3 direction[NUMRAYS2];
	btVector3 hit[NUMRAYS2];
	btVector3 normal[NUMRAYS2];
	struct GUIHelperInterface* m_guiHelper;

	int frame_counter;
	int ms;
	int sum_ms;
	int sum_ms_samples;
	int min_ms;
	int max_ms;

#ifdef USE_BT_CLOCK
	btClock frame_timer;
#endif  //USE_BT_CLOCK

	btScalar dx;
	btScalar min_x;
	btScalar max_x;
	btScalar max_y;
	btScalar sign;

	btRaycastBar3()
	{
		m_guiHelper = 0;
		ms = 0;
		max_ms = 0;
		min_ms = 9999;
		sum_ms_samples = 0;
		sum_ms = 0;
	}

	btRaycastBar3(btScalar ray_length, btScalar z, btScalar max_y, struct GUIHelperInterface* guiHelper, int upAxisIndex)
	{

		m_guiHelper = guiHelper;
		frame_counter = 0;
		ms = 0;
		max_ms = 0;
		min_ms = 9999;
		sum_ms_samples = 0;
		sum_ms = 0;
		dx = 10.0;
		min_x = 0;
		max_x = 0;
		this->max_y = max_y;
		sign = 1.0;
		btScalar dalpha = 2 * SIMD_2_PI / NUMRAYS2;
		for (int i = 0; i < NUMRAYS2; i++)
		{
			btScalar alpha = dalpha * i;
			// rotate around by alpha degrees y
			btVector3 upAxis(0, 0, 0);
			upAxis[upAxisIndex] = 1;

			btQuaternion q(upAxis, alpha);
			direction[i] = btVector3(1.0, 0.0, 0.0);
			direction[i] = quatRotate(q, direction[i]);
			direction[i] = direction[i] * ray_length;

			if (upAxisIndex == 1)
			{
				source[i] = btVector3(min_x, max_y, z);
			}
			else
			{
				source[i] = btVector3(min_x, z, max_y);
			}
			dest[i] = source[i] + direction[i];
			dest[i][upAxisIndex] = -1000;
			normal[i] = btVector3(1.0, 0.0, 0.0);
		}
	}

	void move(btScalar dt)
	{
		if (dt > btScalar(1.0 / 60.0))
			dt = btScalar(1.0 / 60.0);
		for (int i = 0; i < NUMRAYS2; i++)
		{
			source[i][0] += dx * dt * sign;
			dest[i][0] += dx * dt * sign;
		}
		if (source[0][0] < min_x)
			sign = 1.0;
		else if (source[0][0] > max_x)
			sign = -1.0;
	}

	void castRays(btCollisionWorld* cw, int iBegin, int iEnd)
	{
		if (m_guiHelper==0)
			return;

		for (int i = iBegin; i < iEnd; ++i)
		{
			btCollisionWorld::ClosestRayResultCallback cb(source[i], dest[i]);

			{
				BT_PROFILE("cw->rayTest");
				//to disable raycast accelerator, uncomment next line
				//cb.m_flags |= btTriangleRaycastCallback::kF_DisableHeightfieldAccelerator;
				cw->rayTest(source[i], dest[i], cb);
			}
			if (cb.hasHit())
			{
				hit[i] = cb.m_hitPointWorld;
				normal[i] = cb.m_hitNormalWorld;
				normal[i].normalize();
			}
			else
			{
				hit[i] = dest[i];
				normal[i] = btVector3(1.0, 0.0, 0.0);
			}
		}
	}

	struct CastRaysLoopBody : public btIParallelForBody
	{
		btCollisionWorld* mWorld;
		btRaycastBar3* mRaycasts;

		CastRaysLoopBody(btCollisionWorld* cw, btRaycastBar3* rb) : mWorld(cw), mRaycasts(rb) {}

		void forLoop(int iBegin, int iEnd) const
		{
			mRaycasts->castRays(mWorld, iBegin, iEnd);
		}
	};

	void cast(btCollisionWorld* cw, bool multiThreading = false)
	{
		BT_PROFILE("cast");

#ifdef USE_BT_CLOCK
		frame_timer.reset();
#endif  //USE_BT_CLOCK

#ifdef BATCH_RAYCASTER
		if (!gBatchRaycaster)
			return;

		gBatchRaycaster->clearRays();
		for (int i = 0; i < NUMRAYS; i++)
		{
			gBatchRaycaster->addRay(source[i], dest[i]);
		}
		gBatchRaycaster->performBatchRaycast();
		for (int i = 0; i < gBatchRaycaster->getNumRays(); i++)
		{
			const SpuRaycastTaskWorkUnitOut& out = (*gBatchRaycaster)[i];
			hit[i].setInterpolate3(source[i], dest[i], out.hitFraction);
			normal[i] = out.hitNormal;
			normal[i].normalize();
		}
#else
#if USE_PARALLEL_RAYCASTS
		if (multiThreading)
		{
			CastRaysLoopBody rayLooper(cw, this);
			int grainSize = 20;  // number of raycasts per task
			btParallelFor(0, NUMRAYS2, grainSize, rayLooper);
		}
		else
#endif  // USE_PARALLEL_RAYCASTS
		{
			// single threaded
			castRays(cw, 0, NUMRAYS2);
		}
#ifdef USE_BT_CLOCK
		ms += frame_timer.getTimeMilliseconds();
#endif  //USE_BT_CLOCK
		frame_counter++;
		if (frame_counter > 50)
		{
			min_ms = ms < min_ms ? ms : min_ms;
			max_ms = ms > max_ms ? ms : max_ms;
			sum_ms += ms;
			sum_ms_samples++;
			btScalar mean_ms = (btScalar)sum_ms / (btScalar)sum_ms_samples;
			printf("%d rays in %d ms %d %d %f\n", NUMRAYS2 * frame_counter, ms, min_ms, max_ms, mean_ms);
			ms = 0;
			frame_counter = 0;
		}
#endif
	}

	void draw()
	{
		if (m_guiHelper)
		{
			btAlignedObjectArray<unsigned int> indices;
			btAlignedObjectArray<btVector3FloatData> points;

			float lineColor[4] = { 1, 0.4, .4, 1 };

			for (int i = 0; i < NUMRAYS2; i++)
			{
				btVector3FloatData s, h;
				for (int w = 0; w < 4; w++)
				{
					s.m_floats[w] = source[i][w];
					h.m_floats[w] = hit[i][w];
				}

				points.push_back(s);
				points.push_back(h);
				indices.push_back(indices.size());
				indices.push_back(indices.size());
			}

			m_guiHelper->getRenderInterface()->drawLines(&points[0].m_floats[0], lineColor, points.size(), sizeof(btVector3FloatData), &indices[0], indices.size(), 1);
		}

	}
};

static btRaycastBar3 raycastBar;

void HeightfieldExample::castRays()
{
#ifdef BT_THREADSAFE
	raycastBar.cast(m_dynamicsWorld, true);
#else
	raycastBar.cast(m_dynamicsWorld, false);
#endif
}

void HeightfieldExample::stepSimulation(float deltaTime)
{
	castRays();

	raycastBar.draw();

	// if dynamic and radial, go ahead and update the field
	if (m_rawHeightfieldData && m_isDynamic && eRadial == m_model && m_heightfieldShape)
	{
		btAlignedObjectArray<GLInstanceVertex> gfxVertices;
		btAlignedObjectArray<int> indices;
		int strideInBytes = 9 * sizeof(float);

		m_phase += s_deltaPhase * deltaTime;
		if (m_phase > 2.0 * SIMD_PI) {
			m_phase -= 2.0 * SIMD_PI;
		}
		int bpe = getByteSize(m_type);
		btAssert(bpe > 0 && "Bad bytes per element");
		setRadial(m_rawHeightfieldData, bpe, m_type, m_phase);

		MyTriangleCollector3  col;
		col.m_pVerticesOut = &gfxVertices;
		col.m_pIndicesOut = &indices;
		btVector3 aabbMin, aabbMax;
		for (int k = 0; k < 3; k++)
		{
			aabbMin[k] = -BT_LARGE_FLOAT;
			aabbMax[k] = BT_LARGE_FLOAT;
		}
		m_heightfieldShape->processAllTriangles(&col, aabbMin, aabbMax);
		if (gfxVertices.size() && indices.size())
		{
			m_guiHelper->getRenderInterface()->updateShape(m_heightfieldShape->getUserIndex(), &gfxVertices[0].xyzw[0], gfxVertices.size());
		}
	}

	if (m_model != gHeightfieldType)
	{
		m_model = gHeightfieldType;
		resetPhysics();
	}
	CommonRigidBodyMTBase::stepSimulation(deltaTime);
}

////////////////////////////////////////////////////////////////////////////////
//
//	TerrainDemo -- public class methods
//
////////////////////////////////////////////////////////////////////////////////

/// one-time class and physics initialization
void HeightfieldExample::initPhysics()
{
	//	std::cerr << "initializing...\n";

	createEmptyDynamicsWorld();
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	m_upAxis = 2;		// start with Y-axis as "up"
	m_guiHelper->setUpAxis(m_upAxis);

	raycastBar = btRaycastBar3(2500.0, 0, 2.0, m_guiHelper, m_upAxis);
	// set up basic state


	m_type = PHY_FLOAT;
	m_model = gHeightfieldType;
	m_isDynamic = true;

	// set up the physics world

	// initialize axis- or type-dependent physics from here
	this->resetPhysics();

}



////////////////////////////////////////////////////////////////////////////////
//
//	TerrainDemo -- private helper methods
//
////////////////////////////////////////////////////////////////////////////////

/// called whenever key terrain attribute is changed
void HeightfieldExample::resetPhysics(void)
{
	m_guiHelper->removeAllGraphicsInstances();

	// remove old heightfield
	clearWorld();

	// reset gravity to point in appropriate direction
	m_dynamicsWorld->setGravity(getUpVector(m_upAxis, 0.0, -s_gravity));

	// get new heightfield of appropriate type
	m_rawHeightfieldData =
		getRawHeightfieldData(m_model, m_type, m_minHeight, m_maxHeight);
	btAssert(m_rawHeightfieldData && "failed to create raw heightfield");

	bool flipQuadEdges = false;
	m_heightfieldShape =
		new btHeightfieldTerrainShape(s_gridSize, s_gridSize,
			m_rawHeightfieldData,
			s_gridHeightScale,
			m_minHeight, m_maxHeight,
			m_upAxis, m_type, flipQuadEdges);
	btAssert(m_heightfieldShape && "null heightfield");

	// set origin to middle of heightfield
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0, 0, -4));

	if (m_model== eImageFile)
	{
		
		b3BulletDefaultFileIO fileIO;
		char relativeFileName[1024];
		int found = fileIO.findFile("heightmaps/gimp_overlay_out.png", relativeFileName, 1024);

		b3AlignedObjectArray<char> buffer;
		buffer.reserve(1024);
		int fileId = fileIO.fileOpen(relativeFileName, "rb");
		if (fileId >= 0)
		{
			int size = fileIO.getFileSize(fileId);
			if (size>0)
			{
				buffer.resize(size);
				int actual = fileIO.fileRead(fileId, &buffer[0], size);
				if (actual != size)
				{
					b3Warning("STL filesize mismatch!\n");
					buffer.resize(0);
				}
			}
			fileIO.fileClose(fileId);
		}

		if (buffer.size())
		{
			int width, height, n;


			unsigned char* image = stbi_load_from_memory((const unsigned char*)&buffer[0], buffer.size(), &width, &height, &n, 3);
			if (image)
			{
				int texId = m_guiHelper->registerTexture(image, width, height);
				m_heightfieldShape->setUserIndex2(texId);
			}
		}
	}
	if (m_upAxis == 2)
		m_heightfieldShape->setFlipTriangleWinding(true);
	//buildAccelerator is optional, it may not support all features.
	m_heightfieldShape->buildAccelerator();

	// scale the shape
	btVector3 localScaling = getUpVector(m_upAxis, s_gridSpacing, 1.0);
	m_heightfieldShape->setLocalScaling(localScaling);

	// stash this shape away
	m_collisionShapes.push_back(m_heightfieldShape);



	// create ground object
	float mass = 0.0;
	btRigidBody* body = createRigidBody(mass, tr, m_heightfieldShape);
	double color[4]={1,1,1,1};

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	m_guiHelper->changeRGBAColor(body->getUserIndex(),color);
}


/// removes all objects and shapes from the world
void HeightfieldExample::clearWorld(void)
{
	if (m_dynamicsWorld)
	{
		//remove the rigidbodies from the dynamics world and delete them
		int i;
		for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
		{
			btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body && body->getMotionState())
			{
				delete body->getMotionState();
			}
			m_dynamicsWorld->removeCollisionObject(obj);
			delete obj;
		}

		//delete collision shapes
		for (int j = 0; j < m_collisionShapes.size(); j++)
		{
			btCollisionShape* shape = m_collisionShapes[j];
			delete shape;
		}
		m_collisionShapes.clear();

		// delete raw heightfield data
		delete[] m_rawHeightfieldData;
		m_rawHeightfieldData = NULL;
	}
}

CommonExampleInterface* HeightfieldExampleCreateFunc(CommonExampleOptions& options)
{
	return new HeightfieldExample(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(HeightfieldExampleCreateFunc)

