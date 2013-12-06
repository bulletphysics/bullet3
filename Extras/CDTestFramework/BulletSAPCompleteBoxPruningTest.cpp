/*
BulletSAPCompleteBoxPruningTest, Copyright (c) 2008 Erwin Coumans
Part of:
CDTestFramework http://codercorner.com
Copyright (c) 2007-2008 Pierre Terdiman,  pierre@codercorner.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

//This file was added by Erwin Coumans, to test Bullet SAP performance

#include "stdafx.h"
#include "BulletSAPCompleteBoxPruningTest.h"
#include "RenderingHelpers.h"
#include "GLFontRenderer.h"
#include "btBulletCollisionCommon.h"
#include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"
#include "Camera.h"
#ifdef USE_CUDA_BROADPHASE
#include "../CUDA/btCudaBroadphase.h"
#endif
#include "LinearMath/btQuickprof.h"

int numParts =2;

bool	enableCulling	=	true;
bool	cullFarPlane	=	false;
bool	showCulling		=	false;
bool	enableOcclusion	=	false;
bool	showOcclusion	=	true;
int		visiblecount	=	0;

static bool sBulletProfilerToggle = false;

struct OcclusionBuffer
{
struct WriteOCL
	{
	static inline bool Process(btScalar& q,btScalar v) { if(q<v) q=v;return(false); }
	};
struct QueryOCL
	{
	static inline bool Process(btScalar& q,btScalar v) { return(q<=v); }
	};
bool							initialized;
btAlignedObjectArray<btScalar>	buffer;
int								sizes[2];
btScalar						scales[2];
btScalar						offsets[2];
btScalar						wtrs[16];
btVector3						eye;
btVector3						neardist;
btScalar						ocarea;
btScalar						qrarea;
GLuint							texture;
			OcclusionBuffer()
	{
	initialized=false;
	neardist=btVector3(2,2,2);
	ocarea=(btScalar)0;
	qrarea=(btScalar)0;
	}
void		setup(int w,int h)
	{
	initialized=true;
	sizes[0]=w;
	sizes[1]=h;
	scales[0]=w/2;
	scales[1]=h/2;
	offsets[0]=scales[0]+0.5;
	offsets[1]=scales[1]+0.5;
	glGenTextures(1,&texture);
	clear();
	}
void		clear()
	{
	buffer.resize(0);
	buffer.resize(sizes[0]*sizes[1],0);
	}
void		initialize()
	{
	if(!initialized)
		{
		setup(128,128);
		}
	GLint		v[4];
	GLdouble	m[16],p[16];
	glGetIntegerv(GL_VIEWPORT,v);
	glGetDoublev(GL_MODELVIEW_MATRIX,m);
	glGetDoublev(GL_PROJECTION_MATRIX,p);
	for(int i=0;i<16;++i) wtrs[i]=p[i];
	clear();
	}
void		drawBuffer(	btScalar l,btScalar t,
						btScalar r,btScalar b)
	{
	btAlignedObjectArray<GLubyte>	data;
	data.resize(buffer.size());
	for(int i=0;i<data.size();++i)
		{
		data[i]=int(32/buffer[i])%255;
		}
	glBindTexture(GL_TEXTURE_2D,texture);
	glDisable(GL_BLEND);
	glEnable(GL_TEXTURE_2D);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D,0,GL_LUMINANCE,sizes[0],sizes[1],0,GL_LUMINANCE,GL_UNSIGNED_BYTE,&data[0]);
	glBegin(GL_QUADS);
	glColor4ub(255,255,255,255);
	glTexCoord2f(0,0);glVertex2f(l,t);
	glTexCoord2f(1,0);glVertex2f(r,t);
	glTexCoord2f(1,1);glVertex2f(r,b);
	glTexCoord2f(0,1);glVertex2f(l,b);
	glEnd();
	glDisable(GL_TEXTURE_2D);
	}
btVector4	transform(const btVector3& x) const
	{
	btVector4	t;
	t[0]	=	x[0]*wtrs[0]+x[1]*wtrs[4]+x[2]*wtrs[8]+wtrs[12];
	t[1]	=	x[0]*wtrs[1]+x[1]*wtrs[5]+x[2]*wtrs[9]+wtrs[13];
	t[2]	=	x[0]*wtrs[2]+x[1]*wtrs[6]+x[2]*wtrs[10]+wtrs[14];
	t[3]	=	x[0]*wtrs[3]+x[1]*wtrs[7]+x[2]*wtrs[11]+wtrs[15];
	return(t);
	}
static bool	project(btVector4* p,int n)
	{
	for(int i=0;i<n;++i)
		{
		const btScalar	iw=1/p[i][3];
		p[i][2]=1/p[i][3];
		p[i][0]*=p[i][2];
		p[i][1]*=p[i][2];
		}
	return(true);
	}
template <const int NP>
static int	clip(const btVector4* pi,btVector4* po)
	{
	btScalar	s[NP];
	int			m=0;
	for(int i=0;i<NP;++i)
		{
		s[i]=pi[i][2]+pi[i][3];
		if(s[i]<0) m+=1<<i;
		}
	if(m==((1<<NP)-1)) return(0);
	if(m!=0)
		{
		int n=0;
		for(int i=NP-1,j=0;j<NP;i=j++)
			{
			const btVector4&	a=pi[i];
			const btVector4&	b=pi[j];
			const btScalar		t=s[i]/(a[3]+a[2]-b[3]-b[2]);
			if((t>0)&&(t<1))
				{
				po[n][0]	=	a[0]+(b[0]-a[0])*t;
				po[n][1]	=	a[1]+(b[1]-a[1])*t;
				po[n][2]	=	a[2]+(b[2]-a[2])*t;
				po[n][3]	=	a[3]+(b[3]-a[3])*t;
				++n;
				}
			if(s[j]>0) po[n++]=b;
			}
		return(n);
		}
	for(int i=0;i<NP;++i) po[i]=pi[i];
	return(NP);
	}
template <typename POLICY>
inline bool	draw(	const btVector4& a,
					const btVector4& b,
					const btVector4& c,
					const btScalar minarea)
	{
	const btScalar		a2=(b-a).cross(c-a)[2];
	if(a2>0)
		{
		if(a2<minarea)	return(true);
		const int		x[]={	(int)(a.x()*scales[0]+offsets[0]),
								(int)(b.x()*scales[0]+offsets[0]),
								(int)(c.x()*scales[0]+offsets[0])};
		const int		y[]={	(int)(a.y()*scales[1]+offsets[1]),
								(int)(b.y()*scales[1]+offsets[1]),
								(int)(c.y()*scales[1]+offsets[1])};
		const btScalar	z[]={	a.z(),b.z(),c.z()};
		const int		mix=btMax(0,btMin(x[0],btMin(x[1],x[2])));
		const int		mxx=btMin(sizes[0],1+btMax(x[0],btMax(x[1],x[2])));
		const int		miy=btMax(0,btMin(y[0],btMin(y[1],y[2])));
		const int		mxy=btMin(sizes[1],1+btMax(y[0],btMax(y[1],y[2])));
		const int		width=mxx-mix;
		const int		height=mxy-miy;
		if((width*height)>0)
			{
			const int		dx[]={	y[0]-y[1],
									y[1]-y[2],
									y[2]-y[0]};
			const int		dy[]={	x[1]-x[0]-dx[0]*width,
									x[2]-x[1]-dx[1]*width,
									x[0]-x[2]-dx[2]*width};
			const int		a=x[2]*y[0]+x[0]*y[1]-x[2]*y[1]-x[0]*y[2]+x[1]*y[2]-x[1]*y[0];
			const btScalar	ia=1/(btScalar)a;
			const btScalar	dzx=ia*(y[2]*(z[1]-z[0])+y[1]*(z[0]-z[2])+y[0]*(z[2]-z[1]));
			const btScalar	dzy=ia*(x[2]*(z[0]-z[1])+x[0]*(z[1]-z[2])+x[1]*(z[2]-z[0]))-(dzx*width);		
			int				c[]={	miy*x[1]+mix*y[0]-x[1]*y[0]-mix*y[1]+x[0]*y[1]-miy*x[0],
									miy*x[2]+mix*y[1]-x[2]*y[1]-mix*y[2]+x[1]*y[2]-miy*x[1],
									miy*x[0]+mix*y[2]-x[0]*y[2]-mix*y[0]+x[2]*y[0]-miy*x[2]};
			btScalar		v=ia*((z[2]*c[0])+(z[0]*c[1])+(z[1]*c[2]));
			btScalar*		scan=&buffer[miy*sizes[1]];
			for(int iy=miy;iy<mxy;++iy)
				{
				for(int ix=mix;ix<mxx;++ix)
					{
					if((c[0]>=0)&&(c[1]>=0)&&(c[2]>=0))
						{
						if(POLICY::Process(scan[ix],v)) return(true);
						}
					c[0]+=dx[0];c[1]+=dx[1];c[2]+=dx[2];v+=dzx;
					}
				c[0]+=dy[0];c[1]+=dy[1];c[2]+=dy[2];v+=dzy;
				scan+=sizes[0];
				}
			}
		}
	return(false);
	}
template <const int NP,typename POLICY>
inline bool	clipDraw(	const btVector4* p,
						btScalar minarea)
	{
	btVector4	o[NP*2];
	const int	n=clip<NP>(p,o);
	bool		earlyexit=false;
	project(o,n);
	for(int i=2;i<n;++i)
		{
		earlyexit|=draw<POLICY>(o[0],o[i-1],o[i],minarea);
		}
	return(earlyexit);
	}
void		appendOccluder(	const btVector3& a,
							const btVector3& b,
							const btVector3& c)
	{
	const btVector4	p[]={transform(a),transform(b),transform(c)};
	clipDraw<3,WriteOCL>(p,ocarea);
	}
void		appendOccluder(	const btVector3& a,
							const btVector3& b,
							const btVector3& c,
							const btVector3& d)
	{	
	const btVector4	p[]={transform(a),transform(b),transform(c),transform(d)};
	clipDraw<4,WriteOCL>(p,ocarea);
	}
void		appendOccluder( const btVector3& c,
							const btVector3& e)
	{
	const btVector4	x[]={	transform(btVector3(c[0]-e[0],c[1]-e[1],c[2]-e[2])),
							transform(btVector3(c[0]+e[0],c[1]-e[1],c[2]-e[2])),
							transform(btVector3(c[0]+e[0],c[1]+e[1],c[2]-e[2])),
							transform(btVector3(c[0]-e[0],c[1]+e[1],c[2]-e[2])),
							transform(btVector3(c[0]-e[0],c[1]-e[1],c[2]+e[2])),
							transform(btVector3(c[0]+e[0],c[1]-e[1],c[2]+e[2])),
							transform(btVector3(c[0]+e[0],c[1]+e[1],c[2]+e[2])),
							transform(btVector3(c[0]-e[0],c[1]+e[1],c[2]+e[2]))};
	static const int	d[]={	1,0,3,2,
								4,5,6,7,
								4,7,3,0,
								6,5,1,2,
								7,6,2,3,
								5,4,0,1};
	for(int i=0;i<(sizeof(d)/sizeof(d[0]));)
		{
		const btVector4	p[]={	x[d[i++]],
								x[d[i++]],
								x[d[i++]],
								x[d[i++]]};
		clipDraw<4,WriteOCL>(p,ocarea);
		}	
	}
inline bool	queryOccluder(	const btVector3& a,
							const btVector3& b,
							const btVector3& c)
	{	
	const btVector4	p[]={transform(a),transform(b),transform(c)};
	return(clipDraw<3,QueryOCL>(p,qrarea));
	}
inline bool	queryOccluder(	const btVector3& a,
							const btVector3& b,
							const btVector3& c,
							const btVector3& d)
	{
	const btVector4	p[]={transform(a),transform(b),transform(c),transform(d)};
	return(clipDraw<4,QueryOCL>(p,qrarea));
	}
inline bool	queryOccluder(	const btVector3& c,
							const btVector3& e)
	{
	const btVector4	x[]={	transform(btVector3(c[0]-e[0],c[1]-e[1],c[2]-e[2])),
							transform(btVector3(c[0]+e[0],c[1]-e[1],c[2]-e[2])),
							transform(btVector3(c[0]+e[0],c[1]+e[1],c[2]-e[2])),
							transform(btVector3(c[0]-e[0],c[1]+e[1],c[2]-e[2])),
							transform(btVector3(c[0]-e[0],c[1]-e[1],c[2]+e[2])),
							transform(btVector3(c[0]+e[0],c[1]-e[1],c[2]+e[2])),
							transform(btVector3(c[0]+e[0],c[1]+e[1],c[2]+e[2])),
							transform(btVector3(c[0]-e[0],c[1]+e[1],c[2]+e[2]))};
	for(int i=0;i<8;++i)
		{
		if((x[i][2]+x[i][3])<=0) return(true);
		}
	static const int	d[]={	1,0,3,2,
								4,5,6,7,
								4,7,3,0,
								6,5,1,2,
								7,6,2,3,
								5,4,0,1};
	for(int i=0;i<(sizeof(d)/sizeof(d[0]));)
		{
		const btVector4	p[]={	x[d[i++]],
								x[d[i++]],
								x[d[i++]],
								x[d[i++]]};
		if(clipDraw<4,QueryOCL>(p,qrarea)) return(true);
		}
	return(false);
	}
};

OcclusionBuffer		ocb;

BulletSAPCompleteBoxPruningTest::BulletSAPCompleteBoxPruningTest(int numBoxes,int method) :
	mBar			(null),
	mNbBoxes		(numBoxes),
	mBoxes			(null),
	mBoxPtrs		(null),
	mBoxTime		(null),
	mAmplitude		(100.0f),
	m_method(method)
{
	btVector3 aabbMin(-200,-200,-200);
	btVector3 aabbMax(200,200,200);

	int maxNumBoxes = numBoxes;
	m_isdbvt=false;
	bool disableRaycastAccelerator = true;
	switch (method)
	{
	case 1:
		m_broadphase = new btAxisSweep3(aabbMin,aabbMax,maxNumBoxes,0,disableRaycastAccelerator);
		methodname	=	"btAxisSweep3";
		break;
	case 2:
		m_broadphase = new btAxisSweep3(aabbMin,aabbMax,maxNumBoxes,new btNullPairCache(),disableRaycastAccelerator);
		methodname	=	"btAxisSweep3+btNullPairCache";
		break;
	case 3:
		m_broadphase = new btAxisSweep3(aabbMin,aabbMax,maxNumBoxes,new btSortedOverlappingPairCache(),disableRaycastAccelerator);
		methodname	=	"btAxisSweep3+btSortedOverlappingPairCache";
		break;
	case 4:
		m_broadphase = new btSimpleBroadphase(maxNumBoxes,new btSortedOverlappingPairCache());
		methodname	=	"btSimpleBroadphase+btSortedOverlappingPairCache";
		break;
	case 5:
		m_broadphase = new btSimpleBroadphase(maxNumBoxes,new btNullPairCache());
		methodname	=	"btSimpleBroadphase+btNullPairCache";
		break;

/*	case 6:
		{
		methodname	=	"btMultiSapBroadphase";
			btMultiSapBroadphase* multiSap = new btMultiSapBroadphase(maxNumBoxes);
			m_broadphase = multiSap;

			btVector3 tmpAabbMin,tmpAabbMax;
	
			float numP = (float) numParts;

			for (int i=0;i<numParts;i++)
			{
				tmpAabbMin[0] = aabbMin[0] + i*(aabbMax[0]-aabbMin[0])/numP;
				tmpAabbMax[0] = aabbMin[0] + (i+1)*(aabbMax[0]-aabbMin[0])/numP;

				for (int j=0;j<numParts;j++)
				{
					tmpAabbMin[1] = aabbMin[1] + j*(aabbMax[1]-aabbMin[1])/numP;
					tmpAabbMax[1] = aabbMin[1] + (j+1)*(aabbMax[1]-aabbMin[1])/numP;

					for (int k=0;k<numParts;k++)
					{
						tmpAabbMin[2] = aabbMin[2] + k*(aabbMax[2]-aabbMin[2])/numP;
						tmpAabbMax[2] = aabbMin[2] + (k+1)*(aabbMax[2]-aabbMin[2])/numP;

						btAxisSweep3* childBp = new btAxisSweep3(tmpAabbMin,tmpAabbMax,maxNumBoxes,multiSap->getOverlappingPairCache(),disableRaycastAccelerator);
						multiSap->getBroadphaseArray().push_back(childBp);
					}
				}
			}
		
	//		btAxisSweep3* childBp = new btAxisSweep3(aabbMin,aabbMax,maxNumBoxes,multiSap->getOverlappingPairCache());
	//	multiSap->getBroadphaseArray().push_back(childBp);
			multiSap->buildTree(aabbMin,aabbMax);
	
		}
		break;
		*/
	case	7:
		{
		btDbvtBroadphase*	pbp=new btDbvtBroadphase();
		m_broadphase			=	pbp;
		pbp->m_deferedcollide	=	true;	/* Faster initialization, set to false after.	*/ 
		m_isdbvt				=	true;
		methodname				=	"dynamic AABB tree, btDbvtBroadphase";
		}
		break;
	case 8:
//		m_broadphase = new btAxisSweep3(aabbMin,aabbMax,maxNumBoxes);
//		m_broadphase = new btSimpleBroadphase(maxNumBoxes,new btSortedOverlappingPairCache());
//		m_broadphase = new btCudaBroadphase(aabbMin, aabbMax, 8, 8, 8, 8192, 8192, 64, 16);
//		m_broadphase = new btCudaBroadphase(aabbMin, aabbMax, 12, 12, 12, 8192, 8192, 64, 16);
//		m_broadphase = new btCudaBroadphase(aabbMin, aabbMax, 16, 16, 16, 8192, 8192, 64, 16);
#ifdef USE_CUDA_BROADPHASE
		m_broadphase = new btCudaBroadphase(aabbMin, aabbMax, 24, 24, 24,maxNumBoxes , maxNumBoxes, 64, 16);
//		m_broadphase = new btCudaBroadphase(aabbMin, aabbMax, 32, 32, 32, 8192, 8192, 64, 16);
		methodname	=	"btCudaBroadphase";
		break;

	case 9:
		m_broadphase = new bt3DGridBroadphase(aabbMin, aabbMax, 24, 24, 24,maxNumBoxes , maxNumBoxes, 64, 16);
		methodname	=	"bt3DGridBroadphase";
		break;
#endif //USE_CUDA_BROADPHASE

	default:
		{

			btDbvtBroadphase*	pbp=new btDbvtBroadphase();
			m_broadphase			=	pbp;
			pbp->m_deferedcollide	=	true;	/* Faster initialization, set to false after.	*/ 
			m_isdbvt				=	true;
			methodname				=	"dynamic AABB tree, btDbvtBroadphase";

			//m_broadphase = new btAxisSweep3(aabbMin,aabbMax,numBoxes,new btNullPairCache());
			//methodname	=	"btAxisSweep3+btNullPairCache";
		}
	}
}

BulletSAPCompleteBoxPruningTest::~BulletSAPCompleteBoxPruningTest()
{
	DELETEARRAY(mBoxTime);
	DELETEARRAY(mBoxPtrs);
	DELETEARRAY(mBoxes);
	delete m_broadphase;
}

void BulletSAPCompleteBoxPruningTest::Init()
{
	btClock		clock;
	m_firstTime = true;	
	SRand(0);


	mBoxes = new AABB[mNbBoxes];
	mFlags	=	new bool[mNbBoxes];
	mBoxPtrs = new const AABB*[mNbBoxes];
	mBoxTime = new float[mNbBoxes];
	for(udword i=0;i<mNbBoxes;i++)
	{
		Point Center, Extents;

		Center.x = (UnitRandomFloat()-0.5f) * 100.0f;
		Center.y = (UnitRandomFloat()-0.5f) * 10.0f;
		Center.z = (UnitRandomFloat()-0.5f) * 100.0f;
		Extents.x = 2.0f + UnitRandomFloat() * 2.0f;
		Extents.y = 2.0f + UnitRandomFloat() * 2.0f;
		Extents.z = 2.0f + UnitRandomFloat() * 2.0f;

		mBoxes[i].SetCenterExtents(Center, Extents);
		mBoxPtrs[i] = &mBoxes[i];
		btVector3	aabbMin(Center.x-Extents.x,Center.y-Extents.y,Center.z-Extents.z);
		btVector3	aabbMax(Center.x+Extents.x,Center.y+Extents.y,Center.z+Extents.z);
		int shapeType =0;
		void* userPtr = 0;
		btBroadphaseProxy* proxy = m_broadphase->createProxy(aabbMin,aabbMax,shapeType,&mBoxes[i],1,1,0,0);//m_dispatcher);
		m_proxies.push_back( proxy );

		mBoxTime[i] = 2000.0f*UnitRandomFloat();
	}
	printf("Initialization of %s with %u boxes: %ums\r\n",methodname,mNbBoxes,clock.getTimeMilliseconds());
}

void BulletSAPCompleteBoxPruningTest::Release()
{
	DELETEARRAY(mBoxTime);
	DELETEARRAY(mBoxes);
}

extern int		doTree;
extern int		percentUpdate;
extern float	objectSpeed;
extern bool		enableDraw;

static void TW_CALL NormalMode(void* pdata)
{
btDbvtBroadphase*	pb=(btDbvtBroadphase*)pdata;
pb->m_deferedcollide	=	true;
}

static void TW_CALL SlowSpeedMode(void* pdata)
{
btDbvtBroadphase*	pb=(btDbvtBroadphase*)pdata;
pb->m_deferedcollide	=	false;
}

void BulletSAPCompleteBoxPruningTest::Select()
{
	// Create a tweak bar
	{
		mBar = TwNewBar("OPC_CompleteBoxPruning");
		TwAddVarRW(mBar, "Speed", TW_TYPE_FLOAT, &objectSpeed, " min=0.0 max=0.01 step=0.0001");
		TwAddVarRW(mBar, "Amplitude", TW_TYPE_FLOAT, &mAmplitude, " min=10.0 max=200.0 step=0.1");
		if(m_isdbvt)
			{
			btDbvtBroadphase*	pbp=(btDbvtBroadphase*)m_broadphase;
			TwAddVarRW(mBar, "Enable culling",TW_TYPE_BOOLCPP,&enableCulling,"");
			TwAddVarRW(mBar, "Enable occlusion",TW_TYPE_BOOLCPP,&enableOcclusion,"");
			TwAddVarRW(mBar, "Show culling",TW_TYPE_BOOLCPP,&showCulling,"");			
			TwAddVarRW(mBar, "Show occlusion",TW_TYPE_BOOLCPP,&showOcclusion,"");
			TwAddVarRW(mBar, "Cull far plane",TW_TYPE_BOOLCPP,&cullFarPlane,"");
			TwAddVarRW(mBar, "OC Min area",TW_TYPE_FLOAT,&ocb.ocarea,"min=0.0 max=1.0 step=0.001");
			TwAddVarRW(mBar, "QR Min area",TW_TYPE_FLOAT,&ocb.qrarea,"min=0.0 max=1.0 step=0.001");
			TwAddVarRW(mBar, "Dyn lkhd",TW_TYPE_INT32,&pbp->m_sets[0].m_lkhd,"min=-1 max=32");
			TwAddVarRW(mBar, "Fix lkhd",TW_TYPE_INT32,&pbp->m_sets[1].m_lkhd,"min=-1 max=32");
			TwAddVarRW(mBar, "Dyn opt/f(%)",TW_TYPE_INT32,&pbp->m_dupdates,"min=0 max=100");
			TwAddVarRW(mBar, "Fix opt/f(%)",TW_TYPE_INT32,&pbp->m_fupdates,"min=0 max=100");
			TwAddVarRW(mBar, "Cln opt/f(%)",TW_TYPE_INT32,&pbp->m_cupdates,"min=0 max=100");
			TwAddVarRW(mBar, "Prediction",TW_TYPE_FLOAT,&pbp->m_prediction,"min=0.0 max=2.0 step=0.1");
			TwAddVarRW(mBar, "Defered collide",TW_TYPE_BOOLCPP,&pbp->m_deferedcollide,"");
			TwAddVarRO(mBar, "Dyn leafs",TW_TYPE_INT32,&pbp->m_sets[0].m_leaves,"");
			TwAddVarRO(mBar, "Fix leafs",TW_TYPE_INT32,&pbp->m_sets[1].m_leaves,"");
			TwAddVarRO(mBar, "Updates ratio",TW_TYPE_FLOAT,&pbp->m_updates_ratio,"");
			TwAddVarRO(mBar, "Visible",TW_TYPE_INT32,&visiblecount,"");
			TwAddButton(mBar,"Normal mode",&NormalMode,m_broadphase,"");
			TwAddButton(mBar,"Slow speed mode",&SlowSpeedMode,m_broadphase,"");
			}
	}
	printf("SubMethod: %s\r\n",methodname);
}

void BulletSAPCompleteBoxPruningTest::Deselect()
{
	if(mBar)
	{
		TwDeleteBar(mBar);
		mBar = null;
	}
}

bool BulletSAPCompleteBoxPruningTest::UpdateBoxes(int numBoxes)
{
	static bool once=true;

	for(udword i=0;i<(udword)numBoxes;i++)
	{
		mBoxTime[i] += objectSpeed;

		Point Center,Extents;
		mBoxes[i].GetExtents(Extents);

		Center.x = cosf(mBoxTime[i]*2.17f)*mAmplitude + sinf(mBoxTime[i])*mAmplitude*0.5f;
		Center.y = cosf(mBoxTime[i]*1.38f)*mAmplitude + sinf(mBoxTime[i]*mAmplitude);
		Center.z = sinf(mBoxTime[i]*0.777f)*mAmplitude;

		mBoxes[i].SetCenterExtents(Center, Extents);
	}
	return true;
}

void BulletSAPCompleteBoxPruningTest::PerformTest()
{
	int numUpdatedBoxes = (mNbBoxes*percentUpdate)/100;
	if (m_firstTime)
	{
		numUpdatedBoxes = mNbBoxes;
	}
	mProfiler.Start();
	UpdateBoxes(numUpdatedBoxes);
	

	mPairs.ResetPairs();
	
	//CompleteBoxPruning(mNbBoxes, mBoxPtrs, mPairs, Axes(AXES_XZY));
	///add batch query?
	

	for (int i=0;i<numUpdatedBoxes;i++)
	{
		Point Center;
		Point Extents;
		mBoxPtrs[i]->GetCenter(Center);
		mBoxPtrs[i]->GetExtents(Extents);
		btVector3	aabbMin(Center.x-Extents.x,Center.y-Extents.y,Center.z-Extents.z);
		btVector3	aabbMax(Center.x+Extents.x,Center.y+Extents.y,Center.z+Extents.z);
		m_broadphase->setAabb(m_proxies[i],aabbMin,aabbMax,0);//m_dispatcher);
	}

#ifndef BT_NO_PROFILE
	if(sBulletProfilerToggle)
	{
		CProfileManager::Reset();
	}
#endif //BT_NO_PROFILE

	m_broadphase->calculateOverlappingPairs(0);

#ifndef BT_NO_PROFILE
	if(sBulletProfilerToggle)
	{
		CProfileManager::Increment_Frame_Counter();
		CProfileManager::dumpAll();
	}
#endif //BT_NO_PROFILE
	

	mProfiler.End();
	mProfiler.Accum();

	if (m_firstTime)
	{
		//initialization messes up timings
		m_firstTime = false;
		if(m_isdbvt)
			{
			((btDbvtBroadphase*)m_broadphase)->m_deferedcollide=false;
			}
		mProfiler.Reset();		
	}
	
	#if 0
		{
		int	missedpairs=0;
		for(int i=0;i<m_proxies.size();++i)
			{
			btDbvtProxy*	pa((btDbvtProxy*)m_proxies[i]);
			for(int j=i+1;j<m_proxies.size();++j)
				{
				btDbvtProxy*	pb((btDbvtProxy*)m_proxies[j]);				
				if(Intersect(pa->aabb,pb->aabb))
					{
					btDbvtProxy*	spa=pa;
					btDbvtProxy*	spb=pb;
					if(spa>spb) btSwap(spa,spb);
					if(!m_broadphase->getOverlappingPairCache()->findPair(spa,spb))
						{
						++missedpairs;
						printf("Cannot find %i,%i\r\n",i,j);
						}
					}
				}
			}
		if(missedpairs>0) printf("Missed pairs: %u\r\n",missedpairs);
		}
	#endif
	
//	printf("%d pairs colliding\r     ", mPairs.GetNbPairs());

	ZeroMemory(mFlags,sizeof(bool)*mNbBoxes);

	btOverlappingPairCache* pairCache = m_broadphase->getOverlappingPairCache();
	const btBroadphasePair* pairPtr = pairCache->getOverlappingPairArrayPtr();

	for(udword i=0;i<(udword)pairCache->getNumOverlappingPairs();i++)
	{
//		Flags[pairPtr[i].m_pProxy0->getUid()-1] = true;
//		Flags[pairPtr[i].m_pProxy1->getUid()-1] = true;
		int	j;
		j=((AABB*)pairPtr[i].m_pProxy0->m_clientObject)-mBoxes;
		mFlags[j] = true;
		j=((AABB*)pairPtr[i].m_pProxy1->m_clientObject)-mBoxes;
		mFlags[j] = true;
	}
	
	if(enableDraw)
		{
		btVector3 aabbMin(-200,-200,-200);
		btVector3 aabbMax(200,200,200);

		btVector3 tmpAabbMin,tmpAabbMax;	
		glDisable(GL_DEPTH_TEST);


				float numP = (float) numParts;
		
				for (int i=0;i<numParts;i++)
				{
					tmpAabbMin[0] = aabbMin[0] + i*(aabbMax[0]-aabbMin[0])/numP;
					tmpAabbMax[0] = aabbMin[0] + (i+1)*(aabbMax[0]-aabbMin[0])/numP;

					for (int j=0;j<numParts;j++)
					{
						tmpAabbMin[1] = aabbMin[1] + j*(aabbMax[1]-aabbMin[1])/numP;
						tmpAabbMax[1] = aabbMin[1] + (j+1)*(aabbMax[1]-aabbMin[1])/numP;

						for (int k=0;k<numParts;k++)
						{
							tmpAabbMin[2] = aabbMin[2] + k*(aabbMax[2]-aabbMin[2])/numP;
							tmpAabbMax[2] = aabbMin[2] + (k+1)*(aabbMax[2]-aabbMin[2])/numP;

				
					OBB CurrentBox;
					CurrentBox.mRot.Identity();
					
					{
						Point mmin(tmpAabbMin[0],tmpAabbMin[1],tmpAabbMin[2]);
						Point mmax(tmpAabbMax[0],tmpAabbMax[1],tmpAabbMax[2]);

						glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
						glEnable(GL_BLEND);
						glColor4f(i, j,k,0.2);//1.0f, 0.0f);
						CurrentBox.mCenter = (mmin+mmax)*0.5;
						CurrentBox.mExtents = (mmax-mmin)*0.5;
					
						DrawOBB(CurrentBox);

					}
				}
			}

		}

			

		glEnable(GL_DEPTH_TEST);
		//glDisable(GL_DEPTH_TEST);

		glDisable(GL_BLEND);
		Render();
		}
	
	char Buffer[4096];
	sprintf_s(Buffer, sizeof(Buffer), "Bullet %s: %5.1f us (%d cycles) : %d pairs\n", methodname, mProfiler.mMsTime, mProfiler.mCycles, 
			m_broadphase->getOverlappingPairCache()->getNumOverlappingPairs());

//	m_broadphase)->printStats();

	GLFontRenderer::print(10.0f, 10.0f, 0.02f, Buffer);
}

//
static void	DrawVolume(const btDbvtVolume& volume,const btVector3& color)
{
const btVector3	mins=volume.Mins();
const btVector3	maxs=volume.Maxs();
glColor3f(color.x(),color.y(),color.z());
glVertex3f(mins.x(),mins.y(),mins.z());
glVertex3f(maxs.x(),mins.y(),mins.z());

glVertex3f(maxs.x(),mins.y(),mins.z());
glVertex3f(maxs.x(),maxs.y(),mins.z());

glVertex3f(maxs.x(),maxs.y(),mins.z());
glVertex3f(mins.x(),maxs.y(),mins.z());

glVertex3f(mins.x(),maxs.y(),mins.z());
glVertex3f(mins.x(),mins.y(),mins.z());

glVertex3f(mins.x(),mins.y(),maxs.z());
glVertex3f(maxs.x(),mins.y(),maxs.z());

glVertex3f(maxs.x(),mins.y(),maxs.z());
glVertex3f(maxs.x(),maxs.y(),maxs.z());

glVertex3f(maxs.x(),maxs.y(),maxs.z());
glVertex3f(mins.x(),maxs.y(),maxs.z());

glVertex3f(mins.x(),maxs.y(),maxs.z());
glVertex3f(mins.x(),mins.y(),maxs.z());

glVertex3f(mins.x(),mins.y(),mins.z());
glVertex3f(mins.x(),mins.y(),maxs.z());

glVertex3f(maxs.x(),mins.y(),mins.z());
glVertex3f(maxs.x(),mins.y(),maxs.z());

glVertex3f(maxs.x(),maxs.y(),mins.z());
glVertex3f(maxs.x(),maxs.y(),maxs.z());

glVertex3f(mins.x(),maxs.y(),mins.z());
glVertex3f(mins.x(),maxs.y(),maxs.z());
}

//
void BulletSAPCompleteBoxPruningTest::RenderAll()
{
OBB CurrentBox;
CurrentBox.mRot.Identity();
for(udword i=0;i<mNbBoxes;i++)
	{
	if(mFlags[i])	glColor3f(1.0f, 0.0f, 0.0f);
	else			glColor3f(0.0f, 1.0f, 0.0f);
	mBoxes[i].GetCenter(CurrentBox.mCenter);
	mBoxes[i].GetExtents(CurrentBox.mExtents);
	DrawOBB(CurrentBox);
	}
}

//
void BulletSAPCompleteBoxPruningTest::Render()
{
visiblecount=mNbBoxes;
if((!m_isdbvt)||(!enableCulling))
	{	
	RenderAll();
	}
	else
	{
	btDbvtBroadphase*	pbp=(btDbvtBroadphase*)m_broadphase;
	const int			margin=0;
	const int			lc=margin;
	const int			rc=glutGet(GLUT_WINDOW_WIDTH)-(1+margin);
	const int			tc=margin;
	const int			bc=glutGet(GLUT_WINDOW_HEIGHT)-(1+margin);
	const btVector3		c00(ComputeWorldRay(lc,tc).x,
							ComputeWorldRay(lc,tc).y,
							ComputeWorldRay(lc,tc).z);
	const btVector3		c10(ComputeWorldRay(rc,tc).x,
							ComputeWorldRay(rc,tc).y,
							ComputeWorldRay(rc,tc).z);
	const btVector3		c01(ComputeWorldRay(lc,bc).x,
							ComputeWorldRay(lc,bc).y,
							ComputeWorldRay(lc,bc).z);
	const btVector3		c11(ComputeWorldRay(rc,bc).x,
							ComputeWorldRay(rc,bc).y,
							ComputeWorldRay(rc,bc).z);
	
	const btVector3	eye(GetCameraPos().x,GetCameraPos().y,GetCameraPos().z);
	const btVector3	dir(GetCameraDir().x,GetCameraDir().y,GetCameraDir().z);
	const btVector3	x00=eye+c00*100;
	const btVector3	x10=eye+c10*100;
	const btVector3	x01=eye+c01*100;
	const btVector3	x11=eye+c11*100;
	
	ocb.initialize();
	ocb.eye=eye;
	
	btVector3	planes_n[5];
	btScalar	planes_o[5];
	static const btScalar	farplane=200;
	static const int		nplanes=sizeof(planes_n)/sizeof(planes_n[0]);
	const int				acplanes=cullFarPlane?5:4;
	planes_n[0]	=	c01.cross(c00).normalized();
	planes_n[1]	=	c10.cross(c11).normalized();
	planes_n[2]	=	c00.cross(c10).normalized();
	planes_n[3]	=	c11.cross(c01).normalized();
	planes_n[4]	=	-dir;
	planes_o[4]	=	-(eye+dir*farplane).dot(planes_n[4]);
	for(int i=0;i<4;++i) planes_o[i]=-(eye.dot(planes_n[i]));
	
	struct	SceneRenderer : btDbvt::ICollide
		{
		int									drawn;
		BulletSAPCompleteBoxPruningTest*	self;
		OBB									box;
		OcclusionBuffer*					ocb;
				SceneRenderer()
			{
			drawn=0;
			box.mRot.Identity();
			}
		bool	Descent(const btDbvtNode* node)
			{
			return(ocb->queryOccluder(node->volume.Center(),node->volume.Extents()));
			}
		void	Process(const btDbvtNode* node,btScalar depth)
			{
			Process(node);
			}
		void	Process(const btDbvtNode* leaf)
			{	
			btBroadphaseProxy*	proxy=(btBroadphaseProxy*)leaf->data;
			int					i=((AABB*)proxy->m_clientObject)-self->mBoxes;
			if(self->mFlags[i])	glColor3f(1.0f, 0.0f, 0.0f);
			else				glColor3f(0.0f, 1.0f, 0.0f);
			self->mBoxes[i].GetCenter(box.mCenter);
			self->mBoxes[i].GetExtents(box.mExtents);
			DrawOBB(box);drawn++;
			if(ocb)
				{
				ocb->appendOccluder(btVector3(box.mCenter.x,box.mCenter.y,box.mCenter.z),
									btVector3(box.mExtents.x,box.mExtents.y,box.mExtents.z));
				}
			}
		}	srenderer;
	srenderer.self=this;
	srenderer.ocb=0;
	if(enableOcclusion)
		{
		srenderer.ocb=&ocb;
		btDbvt::collideOCL(pbp->m_sets[1].m_root,planes_n,planes_o,dir,acplanes,srenderer);
		btDbvt::collideOCL(pbp->m_sets[0].m_root,planes_n,planes_o,dir,acplanes,srenderer);		
		}
		else
		{
		btDbvt::collideKDOP(pbp->m_sets[1].m_root,planes_n,planes_o,acplanes,srenderer);
		btDbvt::collideKDOP(pbp->m_sets[0].m_root,planes_n,planes_o,acplanes,srenderer);		
		}
	visiblecount=srenderer.drawn;
	if(showOcclusion&&enableOcclusion)
		{
		const btScalar		ratio=((float)glutGet(GLUT_WINDOW_HEIGHT))/((float)glutGet(GLUT_WINDOW_WIDTH));	
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(-1,1,-1,1,-1,1);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		const float	mm[]={	1,0,0,0,
							0,1,0,0,
							0,0,0,1,
							0,0,0,1};
		glMultMatrixf(mm);
		glDisable(GL_DEPTH_TEST);	
		glDisable(GL_LIGHTING);
		const float size=0.6f;
		const float	orgx=0.3f;
		const float	orgy=0.25f;
		const float	left=orgx;
		const float	right=orgx+size;
		const float	top=orgy+size;
		const float	bottom=orgy;
		ocb.drawBuffer(left,bottom,right,top);
		}
	if(showCulling)
		{
		const btScalar		ratio=((float)glutGet(GLUT_WINDOW_HEIGHT))/((float)glutGet(GLUT_WINDOW_WIDTH));	
		static const float	scale=0.004;
		
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(-1,1,-1*ratio,1*ratio,-1,1);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		const float	mm[]={	1,0,0,0,
							0,0,1,0,
							0,1,0,0,
							0,0,0,1};
		glMultMatrixf(mm);
		glScalef(scale,scale,scale);
		
		glDisable(GL_DEPTH_TEST);	
		glDisable(GL_LIGHTING);	
		
		glBegin(GL_LINES);
		glColor4f(1,1,1,1);	
		
		struct	DebugRenderer : btDbvt::ICollide
		{
		OcclusionBuffer*	ocb;
		int					sid;
		bool	AllLeafs(const btDbvtNode* node)
			{
			Process(node);
			return(false);
			}
		bool	Descent(const btDbvtNode* node)
			{
			return(ocb->queryOccluder(node->volume.Center(),node->volume.Extents()));
			}
		void	Process(const btDbvtNode* node,btScalar depth)
			{
			Process(node);
			}
		void	Process(const btDbvtNode* node)
			{
			if(ocb)
				{
				ocb->appendOccluder(node->volume.Center(),node->volume.Extents());
				}
			if(sid>=0)
				{
				const float f=sid/1023.;
				DrawVolume(node->volume,btVector3(1,f,f));
				sid=(sid+1)%1024;
				}
				else
				{
				if(node->isinternal())
					DrawVolume(node->volume,btVector3(0,1,0));
					else
					DrawVolume(node->volume,btVector3(1,0,1));
				}
			}
		}	drenderer;
		if(enableOcclusion)
			{
			drenderer.ocb=&ocb;
			drenderer.sid=0;
			ocb.clear();
			btDbvt::collideOCL(pbp->m_sets[1].m_root,planes_n,planes_o,dir,acplanes,drenderer);
			btDbvt::collideOCL(pbp->m_sets[0].m_root,planes_n,planes_o,dir,acplanes,drenderer);			
			}
			else
			{
			drenderer.ocb=0;
			drenderer.sid=-1;
			btDbvt::collideKDOP(pbp->m_sets[1].m_root,planes_n,planes_o,acplanes,drenderer);
			btDbvt::collideKDOP(pbp->m_sets[0].m_root,planes_n,planes_o,acplanes,drenderer);			
			}
		glEnd();
		
		glBegin(GL_LINES);
		glColor4f(1,1,1,1);
		glVertex3f(eye.x(),eye.y(),eye.z());
		glVertex3f(x00.x(),x00.y(),x00.z());
		glVertex3f(eye.x(),eye.y(),eye.z());
		glVertex3f(x10.x(),x10.y(),x10.z());
		glVertex3f(eye.x(),eye.y(),eye.z());
		glVertex3f(x01.x(),x01.y(),x01.z());
		glVertex3f(eye.x(),eye.y(),eye.z());
		glVertex3f(x11.x(),x11.y(),x11.z());
		
		glVertex3f(x00.x(),x00.y(),x00.z());
		glVertex3f(x10.x(),x10.y(),x10.z());
		
		glVertex3f(x10.x(),x10.y(),x10.z());
		glVertex3f(x11.x(),x11.y(),x11.z());
		
		glVertex3f(x11.x(),x11.y(),x11.z());
		glVertex3f(x01.x(),x01.y(),x01.z());
		
		glVertex3f(x01.x(),x01.y(),x01.z());
		glVertex3f(x00.x(),x00.y(),x00.z());	
		glEnd();
		}
	}
}

void BulletSAPCompleteBoxPruningTest::KeyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
		case 'p':
		case 'P':
			sBulletProfilerToggle = !sBulletProfilerToggle;
			break;
		default : break;
	}
}

void BulletSAPCompleteBoxPruningTest::MouseCallback(int button, int state, int x, int y)
{
}

void BulletSAPCompleteBoxPruningTest::MotionCallback(int x, int y)
{
}
