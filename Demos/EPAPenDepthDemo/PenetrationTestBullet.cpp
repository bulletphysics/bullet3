#include <stdio.h>
#include <GL/glut.h>

#include "btDiscreteCollisionDetectorInterface.h"
#include "btSimplexSolverInterface.h"
#include "btConvexHullShape.h"
#include "Solid3EpaPenetrationDepth.h"
#include "Solid3JohnsonSimplexSolver.h"
#include "btGjkPairDetector.h"
#include "btMinkowskiPenetrationDepthSolver.h"
#include "EpaPenetrationDepthSolver.h"

static bool gRefMode = false;
static int gMethod = 0;
static const float gDisp = 0.01f;
static const float gCamSpeed = 0.1f;
static btVector3 Eye(3.0616338f, 1.1985892f, 2.5769043f);
static btVector3 Dir(-0.66853905,-0.14004262,-0.73037237);
static btVector3 N;
static int mx = 0;
static int my = 0;

static void DrawLine(const btVector3& p0, const btVector3& p1, const btVector3& color, float line_width)
{
	glDisable(GL_LIGHTING);
	glLineWidth(line_width);
	glColor4f(color.x(), color.y(), color.z(), 1.0f);
	btVector3 tmp[] = {p0, p1};
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, sizeof(btVector3), &tmp[0].x());
	glDrawArrays(GL_LINES, 0, 2);
	glDisableClientState(GL_VERTEX_ARRAY);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glEnable(GL_LIGHTING);
}

void DrawTriangle(const btVector3& p0, const btVector3& p1, const btVector3& p2, const btVector3& color)
{
//	glDisable(GL_LIGHTING);
	glColor4f(color.x(), color.y(), color.z(), 1.0f);
	btVector3 tmp[] = {p0, p1, p2};
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, sizeof(btVector3), &tmp[0].x());
	glDrawArrays(GL_TRIANGLES, 0, 3);
	glDisableClientState(GL_VERTEX_ARRAY);
//	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
//	glEnable(GL_LIGHTING);
}

class MyPoly
{
	public:
			MyPoly() : mNbVerts(0), mIndices(NULL)	{}
			~MyPoly()								{ delete[]mIndices;	}

	short	mNbVerts;
	char*	mIndices;
	float	mPlane[4];
};

class MyConvex
{
	public:
				MyConvex();
				~MyConvex();

	bool		LoadFromFile(const char* filename);
	void		Render(bool only_wireframe, const btVector3& wire_color) const;
	void		Project(const btVector3& dir, float& min, float& max) const;

	int			mNbVerts;
	btVector3*	mVerts;
	int			mNbPolys;
	MyPoly*		mPolys;
	btTransform	mTransform;
};

MyConvex::MyConvex() :
	mNbVerts	(0),
	mVerts		(NULL),
	mNbPolys	(0),
	mPolys		(NULL)
{
	mTransform.setIdentity();
}

MyConvex::~MyConvex()
{
	delete[]mPolys;
	delete[]mVerts;
}

bool MyConvex::LoadFromFile(const char* filename)
{
	FILE* fp = fopen(filename, "rb");
	if(!fp)	return false;

	fread(&mNbVerts, sizeof(int), 1, fp);

	mVerts = new btVector3[mNbVerts];
	for(int i=0;i<mNbVerts;i++)
	{
		float vals[3];
		fread(vals, sizeof(float)*3, 1, fp);
		mVerts[i].setX(vals[0]);
		mVerts[i].setY(vals[1]);
		mVerts[i].setZ(vals[2]);
	}

	fread(&mNbPolys, sizeof(int), 1, fp);
	mPolys = new MyPoly[mNbPolys];

	for(int i=0;i<mNbPolys;i++)
	{
		fread(&mPolys[i].mNbVerts, sizeof(short), 1, fp);
		mPolys[i].mIndices = new char[mPolys[i].mNbVerts];
		fread(mPolys[i].mIndices, mPolys[i].mNbVerts, 1, fp);
		fread(mPolys[i].mPlane, sizeof(float)*4, 1, fp);
	}
	fclose(fp);
	return true;
}

void MyConvex::Render(bool only_wireframe, const btVector3& wire_color) const
{
	const float Scale = 1.0f;
	glPushMatrix();

	float glmat[16];	//4x4 column major matrix for OpenGL.
	mTransform.getOpenGLMatrix(glmat);
	glMultMatrixf(&(glmat[0]));

	if(!only_wireframe)
	{
		btVector3 color(0.0f, 0.5f, 1.0f);
		for(int i=0;i<mNbPolys;i++)
		{
			glNormal3f(mPolys[i].mPlane[0], mPolys[i].mPlane[1], mPolys[i].mPlane[2]);

			int NbTris = mPolys[i].mNbVerts-2;
			const btVector3& p0 = mVerts[mPolys[i].mIndices[0]]*Scale;
			for(int j=1;j<=NbTris;j++)
			{
				int k = (j+1)%mPolys[i].mNbVerts;

				const btVector3& p1 = mVerts[mPolys[i].mIndices[j]]*Scale;
				const btVector3& p2 = mVerts[mPolys[i].mIndices[k]]*Scale;

				DrawTriangle(p0, p1, p2, color);
			}
		}
	}

	{
		btVector3 color;
		if(only_wireframe)
			color = wire_color;
		else
			color = btVector3(0.0f, 0.0f, 0.0f);

		for(int i=0;i<mNbPolys;i++)
		{
			for(int j=0;j<mPolys[i].mNbVerts;j++)
			{
				int k = (j+1)%mPolys[i].mNbVerts;
				DrawLine(mVerts[mPolys[i].mIndices[j]]*Scale, mVerts[mPolys[i].mIndices[k]]*Scale, color, 1.0f);
			}
		}
	}

	glPopMatrix();
}

void MyConvex::Project(const btVector3& dir, float& min, float& max) const
{
	min = FLT_MAX;
	max = -FLT_MAX;
	for(int i=0;i<mNbVerts;i++)
	{
		btVector3 pt = mTransform * mVerts[i];
		float dp = pt.dot(dir);
		if(dp < min)	min = dp;
		if(dp > max)	max = dp;
	}
	if(min>max)
	{
		float tmp = min;
		min = max;
		max = tmp;
	}
}

static btVector3 gNormal;
static btVector3 gPoint;
static float gDepth;

	struct MyResult : public btDiscreteCollisionDetectorInterface::Result
	{
		virtual void setShapeIdentifiers(int partId0, int index0, int partId1, int index1)
		{
		}

		virtual void addContactPoint(const btVector3& normalOnBInWorld, const btVector3& pointInWorld, float depth)
		{
			gNormal	= normalOnBInWorld;
			gPoint	= pointInWorld;
			gDepth	= depth;
		}
	};

static bool TestEPA(const MyConvex& hull0, const MyConvex& hull1)
{
	static btSimplexSolverInterface simplexSolver;
//	static Solid3JohnsonSimplexSolver simplexSolver;
	simplexSolver.reset();

	btConvexHullShape convexA((float*)hull0.mVerts, hull0.mNbVerts, sizeof(btVector3));
	btConvexHullShape convexB((float*)hull1.mVerts, hull1.mNbVerts, sizeof(btVector3));

	static Solid3EpaPenetrationDepth Solver0;
	static EpaPenetrationDepthSolver Solver1;
	static btMinkowskiPenetrationDepthSolver Solver2;

	btConvexPenetrationDepthSolver* Solver;
			if(gMethod==0)	Solver = &Solver0;
	else	if(gMethod==1)	Solver = &Solver1;
	else					Solver = &Solver2;

	btGjkPairDetector GJK(&convexA, &convexB, &simplexSolver, Solver);
	GJK.setIgnoreMargin(true);
	convexA.setMargin(0.0f);
	convexB.setMargin(0.0f);

	btDiscreteCollisionDetectorInterface::ClosestPointInput input;
	input.m_transformA = hull0.mTransform;
	input.m_transformB = hull1.mTransform;

	MyResult output;
	GJK.getClosestPoints(input, output, 0);
	return true;
}

static bool TestSepAxis(const btVector3& sep_axis, const MyConvex& hull0, const MyConvex& hull1, float& depth)
{
	float Min0,Max0;
	float Min1,Max1;
	hull0.Project(sep_axis, Min0, Max0);
	hull1.Project(sep_axis, Min1, Max1);

	if(Max0<Min1 || Max1<Min0)
		return false;

	float d0 = Max0 - Min1;
	assert(d0>=0.0f);
	float d1 = Max1 - Min0;
	assert(d1>=0.0f);
	depth = d0<d1 ? d0:d1;
	return true;
}

inline bool IsAlmostZero(const btVector3& v)
{
	if(fabsf(v.x())>1e-6 || fabsf(v.y())>1e-6 || fabsf(v.z())>1e-6)	return false;
	return true;
}

static bool ReferenceCode(const MyConvex& hull0, const MyConvex& hull1, float& dmin, btVector3& sep)
{
	dmin = FLT_MAX;

	// Test normals from hull0
	for(int i=0;i<hull0.mNbPolys;i++)
	{
		btVector3 Normal(hull0.mPolys[i].mPlane[0], hull0.mPolys[i].mPlane[1], hull0.mPolys[i].mPlane[2]);

//		btVector3 WorldNormal = hull0.mTransform * Normal;
		btVector3 WorldNormal = hull0.mTransform.getBasis() * Normal;

		float d;
		if(!TestSepAxis(WorldNormal, hull0, hull1, d))
			return false;

		if(d<dmin)
		{
			dmin = d;
			sep = WorldNormal;
		}
	}

	// Test normals from hull1
	for(int i=0;i<hull1.mNbPolys;i++)
	{
		btVector3 Normal(hull1.mPolys[i].mPlane[0], hull1.mPolys[i].mPlane[1], hull1.mPolys[i].mPlane[2]);

//		btVector3 WorldNormal = hull1.mTransform * Normal;
		btVector3 WorldNormal = hull1.mTransform.getBasis() * Normal;

		float d;
		if(!TestSepAxis(WorldNormal, hull0, hull1, d))
			return false;

		if(d<dmin)
		{
			dmin = d;
			sep = WorldNormal;
		}
	}

	// Test edges
	for(int j=0;j<hull0.mNbPolys;j++)
	{
		const MyPoly& poly0 = hull0.mPolys[j];

		for(int i=0;i<hull1.mNbPolys;i++)
		{
			const MyPoly& poly1 = hull1.mPolys[i];

			for(int e0=0;e0<poly0.mNbVerts;e0++)
			{
				const btVector3& a = hull0.mVerts[poly0.mIndices[e0]];
				const btVector3& b = hull0.mVerts[poly0.mIndices[(e0+1)%poly0.mNbVerts]];

				btVector3 edge0 = a - b;
				btVector3 WorldEdge0 = hull0.mTransform.getBasis() * edge0;

				for(int e1=0;e1<poly1.mNbVerts;e1++)
				{
					const btVector3& c = hull1.mVerts[poly1.mIndices[e1]];
					const btVector3& d = hull1.mVerts[poly1.mIndices[(e1+1)%poly1.mNbVerts]];

					btVector3 edge1 = c - d;
					btVector3 WorldEdge1 = hull1.mTransform.getBasis() * edge1;

					btVector3 Cross = WorldEdge0.cross(WorldEdge1);
					if(!IsAlmostZero(Cross))
					{
						Cross = Cross.normalize();

						float d;
						if(!TestSepAxis(Cross, hull0, hull1, d))
							return false;

						if(d<dmin)
						{
							dmin = d;
							sep = Cross;
						}

					}
				}
			}
		}
	}



	btVector3 DeltaC = (hull1.mTransform * hull1.mTransform.getOrigin()) - (hull0.mTransform * hull0.mTransform.getOrigin());
	if((DeltaC.dot(sep))>0.0f)	sep = -sep;

	return true;
}



static MyConvex gConvex0;
static MyConvex gConvex1;

static void KeyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:	exit(0); break;

	case 'R':
	case 'r':
		gRefMode = !gRefMode;
		break;

	case ' ':
		gMethod++;
		if(gMethod==3)	gMethod=0;
		break;

	case '4':
		gConvex0.mTransform.setOrigin(gConvex0.mTransform.getOrigin() + btVector3(-gDisp,0,0));
		break;
	case '6':
		gConvex0.mTransform.setOrigin(gConvex0.mTransform.getOrigin() + btVector3(gDisp,0,0));
		break;
	case '8':
		gConvex0.mTransform.setOrigin(gConvex0.mTransform.getOrigin() + btVector3(0,gDisp,0));
		break;
	case '2':
		gConvex0.mTransform.setOrigin(gConvex0.mTransform.getOrigin() + btVector3(0,-gDisp,0));
		break;

	case 101:	Eye += Dir * gCamSpeed; break;
	case 103:	Eye -= Dir * gCamSpeed; break;
	case 100:	Eye -= N * gCamSpeed; break;
	case 102:	Eye += N * gCamSpeed; break;
	}
}

static void ArrowKeyCallback(int key, int x, int y)
{
	KeyboardCallback(key,x,y);
}
	
static void MouseCallback(int button, int state, int x, int y)
{
	mx = x;
	my = y;
}

static const float NxPiF32		= 3.141592653589793f;

float degToRad(float a)
	{
	return (float)0.01745329251994329547 * a;
	}

class NxQuat
	{
	public:
	NxQuat(){}

	NxQuat(const float angle, const btVector3 & axis)
	{
	x = axis.x();
	y = axis.y();
	z = axis.z();

	const float i_length =  1.0f / sqrtf( x*x + y*y + z*z );
	x = x * i_length;
	y = y * i_length;
	z = z * i_length;

	float Half = degToRad(angle * 0.5f);

	w = cosf(Half);
	const float sin_theta_over_two = sinf(Half );
	x = x * sin_theta_over_two;
	y = y * sin_theta_over_two;
	z = z * sin_theta_over_two;
	}

void NxQuat::multiply(const NxQuat& left, const btVector3& right)
	{
	float a,b,c,d;

	a = - left.x*right.x() - left.y*right.y() - left.z *right.z();
	b =   left.w*right.x() + left.y*right.z() - right.y()*left.z;
	c =   left.w*right.y() + left.z*right.x() - right.z()*left.x;
	d =   left.w*right.z() + left.x*right.y() - right.x()*left.y;

	w = a;
	x = b;
	y = c;
	z = d;
	}

void NxQuat::rotate(btVector3 & v) const
	{
	NxQuat myInverse;
	myInverse.x = -x;
	myInverse.y = -y;
	myInverse.z = -z;
	myInverse.w =  w;

	NxQuat left;
	left.multiply(*this,v);
	float vx = left.w*myInverse.x + myInverse.w*left.x + left.y*myInverse.z - myInverse.y*left.z;
	float vy = left.w*myInverse.y + myInverse.w*left.y + left.z*myInverse.x - myInverse.z*left.x;
	float vz = left.w*myInverse.z + myInverse.w*left.z + left.x*myInverse.y - myInverse.x*left.y;
	v.setValue(vx, vy, vz);
	}

    float x,y,z,w;
	};


static void MotionCallback(int x, int y)
{
	int dx = mx - x;
	int dy = my - y;
	
	Dir = Dir.normalize();
	N = Dir.cross(btVector3(0,1,0));

	NxQuat qx(NxPiF32 * dx * 20/ 180.0f, btVector3(0,1,0));
	qx.rotate(Dir);
	NxQuat qy(NxPiF32 * dy * 20/ 180.0f, N);
	qy.rotate(Dir);

	mx = x;
	my = y;
}

static void RenderCallback()
{
	// Clear buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
	// Setup camera
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0f, ((float)glutGet(GLUT_WINDOW_WIDTH))/((float)glutGet(GLUT_WINDOW_HEIGHT)), 1.0f, 10000.0f);
	gluLookAt(Eye.x(), Eye.y(), Eye.z(), Eye.x() + Dir.x(), Eye.y() + Dir.y(), Eye.z() + Dir.z(), 0.0f, 1.0f, 0.0f);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glEnable(GL_LIGHTING);

	TestEPA(gConvex0, gConvex1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	float RefDMin;
	btVector3 RefSep;
	bool RefResult = false;
	if(gRefMode)
		RefResult = ReferenceCode(gConvex0, gConvex1, RefDMin, RefSep);

//	DrawLine(gPoint, gPoint + gNormal*20.0f, btVector3(1,0,0), 2.0f);
//	printf("%f:  %f  %f  %f\n", gDepth, gNormal.x(), gNormal.y(), gNormal.z());

	btVector3 color(0,0,0);
	gConvex0.Render(false, color);
	gConvex1.Render(false, color);

	if(gDepth<0.0f)
	{
		btTransform Saved = gConvex0.mTransform;
		gConvex0.mTransform.setOrigin(gConvex0.mTransform.getOrigin() - btVector3(gNormal*gDepth));
		gConvex0.Render(true, btVector3(1.0f, 0.5f, 0.0f));
		gConvex0.mTransform = Saved;
	}
	else
	{
		DrawLine(gPoint, gPoint + gNormal, btVector3(0,1,0), 2.0f);
	}

	if(RefResult & gRefMode)
	{
		btTransform Saved = gConvex0.mTransform;
		gConvex0.mTransform.setOrigin(gConvex0.mTransform.getOrigin() + btVector3(RefSep*RefDMin));
		gConvex0.Render(true, btVector3(0.0f, 0.5f, 1.0f));
		gConvex0.mTransform = Saved;
	}

	glutSwapBuffers();	
}

static void ReshapeCallback(int width, int height)
{
	glViewport(0, 0, width, height);
}

static void IdleCallback()
{
	glutPostRedisplay();
}

int main(int argc, char** argv)
{
	// Initialize Glut
	glutInit(&argc, argv);
	glutInitWindowSize(512, 512);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	int mainHandle = glutCreateWindow("TestBullet");
	glutSetWindow(mainHandle);
	glutDisplayFunc(RenderCallback);
	glutReshapeFunc(ReshapeCallback);
	glutIdleFunc(IdleCallback);
	glutKeyboardFunc(KeyboardCallback);
	glutSpecialFunc(ArrowKeyCallback);
	glutMouseFunc(MouseCallback);
	glutMotionFunc(MotionCallback);
	MotionCallback(0,0);

	// Setup default render states
	glClearColor(0.3f, 0.4f, 0.5f, 1.0);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_CULL_FACE);

	// Setup lighting
	glEnable(GL_LIGHTING);
	float AmbientColor[] = { 0.0f, 0.1f, 0.2f, 0.0f };		glLightfv(GL_LIGHT0, GL_AMBIENT, AmbientColor);
	float DiffuseColor[] = { 1.0f, 1.0f, 1.0f, 0.0f };		glLightfv(GL_LIGHT0, GL_DIFFUSE, DiffuseColor);
	float SpecularColor[] = { 0.0f, 0.0f, 0.0f, 0.0f };		glLightfv(GL_LIGHT0, GL_SPECULAR, SpecularColor);
	float Position[] = { -10.0f, 1000.0f, -4.0f, 1.0f };	glLightfv(GL_LIGHT0, GL_POSITION, Position);
	glEnable(GL_LIGHT0);

	//
	bool Status = gConvex0.LoadFromFile("c:\\convex0.bin");
	if(!Status)
	{
		Status = gConvex0.LoadFromFile("convex0.bin");
		if(!Status)
		{
			printf("Failed to load object!\n");
			exit(0);
		}
	}
	Status = gConvex1.LoadFromFile("c:\\convex0.bin");
	if(!Status)
	{
		Status = gConvex1.LoadFromFile("convex0.bin");
		if(!Status)
		{
			printf("Failed to load object!\n");
			exit(0);
		}
	}

//	gConvex0.mTransform.setOrigin(btVector3(1.0f, 1.0f, 0.0f));
	gConvex0.mTransform.setOrigin(btVector3(0.20000069f, 0.95000005f, 0.0f));

	// Run
	glutMainLoop();

	return 0;
}
