/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
///btSoftBody implementation by Nathanael Presson

#include "btSoftBody.h"
#include "LinearMath/btQuickprof.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"

//
// btSymMatrix
//
template <typename T>
struct btSymMatrix
{
						btSymMatrix() : dim(0)					{}
						btSymMatrix(int n,const T& init=T())	{ resize(n,init); }
void					resize(int n,const T& init=T())			{ dim=n;store.resize((n*(n+1))/2,init); }
T&						operator()(int c,int r)
	{
	if(c>r) btSwap(c,r);
	btAssert(r<dim);
	return(store[(r*(r+1))/2+c]);
	}
const T&				operator()(int c,int r) const
	{
	if(c>r) btSwap(c,r);
	btAssert(r<dim);
	return(store[(r*(r+1))/2+c]);
	}
btAlignedObjectArray<T>	store;
int						dim;
};	


//
// Collision shape
//

///btSoftBodyCollisionShape is work-in-progress collision shape for softbodies
class btSoftBodyCollisionShape : public btConcaveShape
{
public:
	btSoftBody*						m_body;
	
	btSoftBodyCollisionShape(btSoftBody* backptr);

	virtual ~btSoftBodyCollisionShape();

	virtual void	processAllTriangles(btTriangleCallback* callback,const btVector3& aabbMin,const btVector3& aabbMax) const;

	///getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
	virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const
	{
	/* t should be identity, but better be safe than...fast? */ 
	const btVector3	mins=m_body->m_bounds[0];
	const btVector3	maxs=m_body->m_bounds[1];
	const btVector3	crns[]={t*btVector3(mins.x(),mins.y(),mins.z()),
							t*btVector3(maxs.x(),mins.y(),mins.z()),
							t*btVector3(maxs.x(),maxs.y(),mins.z()),
							t*btVector3(mins.x(),maxs.y(),mins.z()),
							t*btVector3(mins.x(),mins.y(),maxs.z()),
							t*btVector3(maxs.x(),mins.y(),maxs.z()),
							t*btVector3(maxs.x(),maxs.y(),maxs.z()),
							t*btVector3(mins.x(),maxs.y(),maxs.z())};
	aabbMin=aabbMax=crns[0];
	for(int i=1;i<8;++i)
		{
		aabbMin.setMin(crns[i]);
		aabbMax.setMax(crns[i]);
		}
	}

	virtual int		getShapeType() const
	{
		return SOFTBODY_SHAPE_PROXYTYPE;
	}
	virtual void	setLocalScaling(const btVector3& /*scaling*/)
	{		
		///na
		btAssert(0);
	}
	virtual const btVector3& getLocalScaling() const
	{
	static const btVector3 dummy(1,1,1);
	return dummy;
	}
	virtual void	calculateLocalInertia(btScalar /*mass*/,btVector3& /*inertia*/) const
	{
		///not yet
		btAssert(0);
	}
	virtual const char*	getName()const
	{
		return "SoftBody";
	}

};

btSoftBodyCollisionShape::btSoftBodyCollisionShape(btSoftBody* backptr)
{
m_body=backptr;
}

btSoftBodyCollisionShape::~btSoftBodyCollisionShape()
{

}

void	btSoftBodyCollisionShape::processAllTriangles(btTriangleCallback* /*callback*/,const btVector3& /*aabbMin*/,const btVector3& /*aabbMax*/) const
{
	//not yet
	btAssert(0);
}

//
// Helpers
//

//

//
template <typename T>
static inline void			ZeroInitialize(T& value)
{
static const T	zerodummy;
value=zerodummy;
}
//
template <typename T>
static inline bool			CompLess(const T& a,const T& b)
{ return(a<b); }
//
template <typename T>
static inline bool			CompGreater(const T& a,const T& b)
{ return(a>b); }
//
template <typename T>
static inline T				Lerp(const T& a,const T& b,btScalar t)
{ return(a+(b-a)*t); }
//
template <typename T>
static inline T				InvLerp(const T& a,const T& b,btScalar t)
{ return((b+a*t-b*t)/(a*b)); }
//
static inline btMatrix3x3	Lerp(	const btMatrix3x3& a,
									const btMatrix3x3& b,
									btScalar t)
{
btMatrix3x3	r;
r[0]=Lerp(a[0],b[0],t);
r[1]=Lerp(a[1],b[1],t);
r[2]=Lerp(a[2],b[2],t);
return(r);
}
//
template <typename T>
static inline T				Clamp(const T& x,const T& l,const T& h)
{ return(x<l?l:x>h?h:x); }
//
template <typename T>
static inline T				Sq(const T& x)
{ return(x*x); }
//
template <typename T>
static inline T				Cube(const T& x)
{ return(x*x*x); }
//
template <typename T>
static inline T				Sign(const T& x)
{ return((T)(x<0?-1:+1)); }
//
template <typename T>
static inline bool			SameSign(const T& x,const T& y)
{ return((x*y)>0); }
//
static inline btMatrix3x3	ScaleAlongAxis(const btVector3& a,btScalar s)
{
	const btScalar	xx=a.x()*a.x();
	const btScalar	yy=a.y()*a.y();
	const btScalar	zz=a.z()*a.z();
	const btScalar	xy=a.x()*a.y();
	const btScalar	yz=a.y()*a.z();
	const btScalar	zx=a.z()*a.x();
	btMatrix3x3		m;
	m[0]=btVector3(1-xx+xx*s,xy*s-xy,zx*s-zx);
	m[1]=btVector3(xy*s-xy,1-yy+yy*s,yz*s-yz);
	m[2]=btVector3(zx*s-zx,yz*s-yz,1-zz+zz*s);
	return(m);
}
//
static inline btMatrix3x3	Cross(const btVector3& v)
{
	btMatrix3x3	m;
	m[0]=btVector3(0,-v.z(),+v.y());
	m[1]=btVector3(+v.z(),0,-v.x());
	m[2]=btVector3(-v.y(),+v.x(),0);
	return(m);
}
//
static inline btMatrix3x3	Diagonal(btScalar x)
{
	btMatrix3x3	m;
	m[0]=btVector3(x,0,0);
	m[1]=btVector3(0,x,0);
	m[2]=btVector3(0,0,x);
	return(m);
}
//
static inline btMatrix3x3	Add(const btMatrix3x3& a,
	const btMatrix3x3& b)
{
	btMatrix3x3	r;
	for(int i=0;i<3;++i) r[i]=a[i]+b[i];
	return(r);
}
//
static inline btMatrix3x3	Sub(const btMatrix3x3& a,
	const btMatrix3x3& b)
{
	btMatrix3x3	r;
	for(int i=0;i<3;++i) r[i]=a[i]-b[i];
	return(r);
}
//
static inline btMatrix3x3	Mul(const btMatrix3x3& a,
	btScalar b)
{
	btMatrix3x3	r;
	for(int i=0;i<3;++i) r[i]=a[i]*b;
	return(r);
}
//
static inline btMatrix3x3	MassMatrix(btScalar im,const btMatrix3x3& iwi,const btVector3& r)
{
	const btMatrix3x3	cr=Cross(r);
	return(Sub(Diagonal(im),cr*iwi*cr));
}
//
static inline btMatrix3x3	ImpulseMatrix(	btScalar dt,
	btScalar ima,
	btScalar imb,
	const btMatrix3x3& iwi,
	const btVector3& r)
{
	return(	Diagonal(1/dt)*
		Add(Diagonal(ima),MassMatrix(imb,iwi,r)).inverse());
}
//
static inline void			PolarDecompose(	const btMatrix3x3& m,
	btMatrix3x3& q,
	btMatrix3x3& s)
{
	static const btScalar	half=(btScalar)0.5;
	static const btScalar	accuracy=(btScalar)0.00001;
	static const int		maxiterations=64;
	btScalar				det=m.determinant();
	if(!btFuzzyZero(det))
	{
		q=m;
		for(int i=0;i<maxiterations;++i)
		{
			q=Mul(Add(q,Mul(q.adjoint(),1/det).transpose()),half);
			const btScalar	ndet=q.determinant();
			if(Sq(ndet-det)>accuracy) det=ndet; else break;
		}
		/* Final orthogonalization	*/ 
		q[0]=q[0].normalized();
		q[2]=cross(q[0],q[1]).normalized();
		q[1]=cross(q[2],q[0]).normalized();
		/* Compute 'S'				*/ 
		s=q.transpose()*m;
	}
	else
	{
		q.setIdentity();
		s.setIdentity();
	}
}
//
static inline btVector3		ProjectOnAxis(	const btVector3& v,
	const btVector3& a)
{
	return(a*dot(v,a));
}
//
static inline btVector3		ProjectOnPlane(	const btVector3& v,
	const btVector3& a)
{
	return(v-ProjectOnAxis(v,a));
}
//
static inline void			ProjectOrigin(	const btVector3& a,
											const btVector3& b,
											btVector3& prj,
											btScalar& sqd)
{
const btVector3	d=b-a;
const btScalar	m2=d.length2();
if(m2>SIMD_EPSILON)
	{	
	const btScalar	t=Clamp<btScalar>(-dot(a,d)/m2,0,1);
	const btVector3	p=a+d*t;
	const btScalar	l2=p.length2();
	if(l2<sqd)
		{
		prj=p;
		sqd=l2;
		}
	}
}
//
static inline void			ProjectOrigin(	const btVector3& a,
											const btVector3& b,
											const btVector3& c,
											btVector3& prj,
											btScalar& sqd)
{
const btVector3&	q=cross(b-a,c-a);
const btScalar		m2=q.length2();
if(m2>SIMD_EPSILON)
	{
	const btVector3	n=q/btSqrt(m2);
	const btScalar	k=dot(a,n);
	const btScalar	k2=k*k;
	if(k2<sqd)
		{
		const btVector3	p=n*k;
		if(	(dot(cross(a-p,b-p),q)>0)&&
			(dot(cross(b-p,c-p),q)>0)&&
			(dot(cross(c-p,a-p),q)>0))
			{			
			prj=p;
			sqd=k2;
			}
			else
			{
			ProjectOrigin(a,b,prj,sqd);
			ProjectOrigin(b,c,prj,sqd);
			ProjectOrigin(c,a,prj,sqd);
			}
		}
	}
}
//
template <typename T>
static inline T				BaryEval(		const T& a,
											const T& b,
											const T& c,
											const btVector3& coord)
{
	return(a*coord.x()+b*coord.y()+c*coord.z());
}
//
static inline btVector3		BaryCoord(	const btVector3& a,
										const btVector3& b,
										const btVector3& c,
										const btVector3& p)
{
const btScalar	w[]={	cross(a-p,b-p).length(),
						cross(b-p,c-p).length(),
						cross(c-p,a-p).length()};
const btScalar	isum=1/(w[0]+w[1]+w[2]);
return(btVector3(w[1]*isum,w[2]*isum,w[0]*isum));
}

//
static btScalar				ImplicitSolve(	btSoftBody::ImplicitFn* fn,
											const btVector3& a,
											const btVector3& b,
											const btScalar accuracy,
											const int maxiterations=256)
{
btScalar	span[2]={0,1};
btScalar	values[2]={fn->Eval(a),fn->Eval(b)};
if(values[0]>values[1])
	{
	btSwap(span[0],span[1]);
	btSwap(values[0],values[1]);
	}
if(values[0]>-accuracy) return(-1);
if(values[1]<+accuracy) return(-1);
for(int i=0;i<maxiterations;++i)
	{
	const btScalar	t=Lerp(span[0],span[1],values[0]/(values[0]-values[1]));
	const btScalar	v=fn->Eval(Lerp(a,b,t));
	if((t<=0)||(t>=1))		break;
	if(btFabs(v)<accuracy)	return(t);
	if(v<0)
		{ span[0]=t;values[0]=v; }
		else
		{ span[1]=t;values[1]=v; }
	}
return(-1);
}

//
static inline btVector3		NormalizeAny(const btVector3& v)
{
	const btScalar l=v.length();
	if(l>SIMD_EPSILON)
		return(v/l);
	else
		return(btVector3(0,0,0));
}

//
static void					PointersToIndices(btSoftBody* psb)
{
#define	PTR2IDX(_p_,_b_)	reinterpret_cast<btSoftBody::Node*>((_p_)-(_b_))
	btSoftBody::Node*	base=&psb->m_nodes[0];
	int i,ni;

	for(i=0,ni=psb->m_nodes.size();i<ni;++i)
	{
		if(psb->m_nodes[i].m_leaf)
			{
			psb->m_nodes[i].m_leaf->data=*(void**)&i;
			}
	}
	for(i=0,ni=psb->m_links.size();i<ni;++i)
	{
		psb->m_links[i].m_n[0]=PTR2IDX(psb->m_links[i].m_n[0],base);
		psb->m_links[i].m_n[1]=PTR2IDX(psb->m_links[i].m_n[1],base);
	}
	for(i=0,ni=psb->m_faces.size();i<ni;++i)
	{
		psb->m_faces[i].m_n[0]=PTR2IDX(psb->m_faces[i].m_n[0],base);
		psb->m_faces[i].m_n[1]=PTR2IDX(psb->m_faces[i].m_n[1],base);
		psb->m_faces[i].m_n[2]=PTR2IDX(psb->m_faces[i].m_n[2],base);
		if(psb->m_faces[i].m_leaf)
			{
			psb->m_faces[i].m_leaf->data=*(void**)&i;
			}
	}
	for(i=0,ni=psb->m_anchors.size();i<ni;++i)
		{
		psb->m_anchors[i].m_node=PTR2IDX(psb->m_anchors[i].m_node,base);
		}
	for(i=0,ni=psb->m_notes.size();i<ni;++i)
		{
		for(int j=0;j<psb->m_notes[i].m_rank;++j)
			{
			psb->m_notes[i].m_nodes[j]=PTR2IDX(psb->m_notes[i].m_nodes[j],base);
			}
		}
#undef	PTR2IDX
}

//
static void					IndicesToPointers(btSoftBody* psb,const int* map=0)
{
#define	IDX2PTR(_p_,_b_)	map?(&(_b_)[map[(((char*)_p_)-(char*)0)]]):	\
								(&(_b_)[(((char*)_p_)-(char*)0)])
	btSoftBody::Node*	base=&psb->m_nodes[0];
	int i,ni;

	for(i=0,ni=psb->m_nodes.size();i<ni;++i)
	{
		if(psb->m_nodes[i].m_leaf)
			{
			psb->m_nodes[i].m_leaf->data=&psb->m_nodes[i];
			}
	}
	for(i=0,ni=psb->m_links.size();i<ni;++i)
	{
		psb->m_links[i].m_n[0]=IDX2PTR(psb->m_links[i].m_n[0],base);
		psb->m_links[i].m_n[1]=IDX2PTR(psb->m_links[i].m_n[1],base);
	}
	for(i=0,ni=psb->m_faces.size();i<ni;++i)
	{
		psb->m_faces[i].m_n[0]=IDX2PTR(psb->m_faces[i].m_n[0],base);
		psb->m_faces[i].m_n[1]=IDX2PTR(psb->m_faces[i].m_n[1],base);
		psb->m_faces[i].m_n[2]=IDX2PTR(psb->m_faces[i].m_n[2],base);
		if(psb->m_faces[i].m_leaf)
			{
			psb->m_faces[i].m_leaf->data=&psb->m_faces[i];
			}
	}
	for(i=0,ni=psb->m_anchors.size();i<ni;++i)
		{
		psb->m_anchors[i].m_node=IDX2PTR(psb->m_anchors[i].m_node,base);
		}
	for(i=0,ni=psb->m_notes.size();i<ni;++i)
		{
		for(int j=0;j<psb->m_notes[i].m_rank;++j)
			{
			psb->m_notes[i].m_nodes[j]=IDX2PTR(psb->m_notes[i].m_nodes[j],base);
			}
		}
#undef	IDX2PTR
}

//
static inline btDbvtVolume	VolumeOf(	const btSoftBody::Face& f,
											btScalar margin)
{
const btVector3*	pts[]={	&f.m_n[0]->m_x,
							&f.m_n[1]->m_x,
							&f.m_n[2]->m_x};
btDbvtVolume		vol=btDbvtVolume::FromPoints(pts,3);
vol.Expand(btVector3(margin,margin,margin));
return(vol);
}

//
static inline btScalar			AreaOf(		const btVector3& x0,
											const btVector3& x1,
											const btVector3& x2)
{
	const btVector3	a=x1-x0;
	const btVector3	b=x2-x0;
	const btVector3	cr=cross(a,b);
	const btScalar	area=cr.length();
	return(area);
}

//
static inline btScalar		VolumeOf(	const btVector3& x0,
										const btVector3& x1,
										const btVector3& x2,
										const btVector3& x3)
{
	const btVector3	a=x1-x0;
	const btVector3	b=x2-x0;
	const btVector3	c=x3-x0;
	return(dot(a,cross(b,c)));
}

//
static inline btScalar		RayTriangle(const btVector3& org,
										const btVector3& dir,
										const btVector3& a,
										const btVector3& b,
										const btVector3& c,
										btScalar maxt=SIMD_INFINITY)
{
	static const btScalar	ceps=-SIMD_EPSILON*10;
	static const btScalar	teps=SIMD_EPSILON*10;
	const btVector3			n=cross(b-a,c-a);
	const btScalar			d=dot(a,n);
	const btScalar			den=dot(dir,n);
	if(!btFuzzyZero(den))
	{
		const btScalar		num=dot(org,n)-d;
		const btScalar		t=-num/den;
		if((t>teps)&&(t<maxt))
		{
			const btVector3	hit=org+dir*t;
			if(	(dot(n,cross(a-hit,b-hit))>ceps)	&&			
				(dot(n,cross(b-hit,c-hit))>ceps)	&&
				(dot(n,cross(c-hit,a-hit))>ceps))
			{
				return(t);
			}
		}
	}
	return(-1);
}

//
// Private implementation
//

struct	RayCaster : btDbvt::ICollide
	{
	btVector3			o;
	btVector3			d;
	btScalar			mint;
	btSoftBody::Face*	face;
	int					tests;
			RayCaster(const btVector3& org,const btVector3& dir,btScalar mxt)
		{
		o		=	org;
		d		=	dir;
		mint	=	mxt;
		face	=	0;
		tests	=	0;
		}
	void	Process(const btDbvtNode* leaf)
		{
		btSoftBody::Face&	f=*(btSoftBody::Face*)leaf->data;
		const btScalar		t=RayTriangle(	o,d,
											f.m_n[0]->m_x,
											f.m_n[1]->m_x,
											f.m_n[2]->m_x,
											mint);
		if((t>0)&&(t<mint))
			{
			mint=t;
			face=&f;
			}
		++tests;
		}
	};


//
static int		RaycastInternal(const btSoftBody* psb,
	const btVector3& org,
	const btVector3& dir,
	btScalar& mint,
	btSoftBody::eFeature::_& feature,
	int& index,
	bool bcountonly)
{
	int	cnt=0;
	if(bcountonly||psb->m_fdbvt.empty())
		{/* Full search	*/ 
		for(int i=0,ni=psb->m_faces.size();i<ni;++i)
			{
			const btSoftBody::Face&	f=psb->m_faces[i];
			const btScalar			t=RayTriangle(	org,dir,
													f.m_n[0]->m_x,
													f.m_n[1]->m_x,
													f.m_n[2]->m_x,
													mint);
			if(t>0)
				{
				++cnt;
				if(!bcountonly)
					{
					feature=btSoftBody::eFeature::Face;
					index=i;
					mint=t;
					}
				}
			}
		}
		else
		{/* Use dbvt	*/ 
		RayCaster	collider(org,dir,mint);
		btDbvt::collideRAY(psb->m_fdbvt.m_root,org,dir,collider);
		if(collider.face)
			{
			mint=collider.mint;
			feature=btSoftBody::eFeature::Face;
			index=(int)(collider.face-&psb->m_faces[0]);
			cnt=1;
			}
		}
	return(cnt);
}

//
static void			InitializeFaceTree(btSoftBody* psb)
{
psb->m_fdbvt.clear();
for(int i=0;i<psb->m_faces.size();++i)
	{
	btSoftBody::Face&	f=psb->m_faces[i];
	f.m_leaf=psb->m_fdbvt.insert(VolumeOf(f,0),&f);
	}
}

//
static btVector3	EvaluateCom(btSoftBody* psb)
{
btVector3	com(0,0,0);
if(psb->m_pose.m_bframe)
	{
	for(int i=0,ni=psb->m_nodes.size();i<ni;++i)
		{
		com+=psb->m_nodes[i].m_x*psb->m_pose.m_wgh[i];
		}
	}
	return(com);
}

//
static void			EvaluateMedium(	const btSoftBody::btSoftBodyWorldInfo* wfi,
									const btVector3& x,
									btSoftBody::sMedium& medium)
{
	medium.m_velocity	=	btVector3(0,0,0);
	medium.m_pressure	=	0;
	medium.m_density	=	wfi->air_density;
	if(wfi->water_density>0)
	{
		const btScalar	depth=-(dot(x,wfi->water_normal)+wfi->water_offset);
		if(depth>0)
		{
			medium.m_density	=	wfi->water_density;
			medium.m_pressure	=	depth*wfi->water_density*wfi->m_gravity.length();
		}
	}
}
	
//
static bool			CheckContact(	btSoftBody* psb,
									btRigidBody* prb,
									const btVector3& x,
									btScalar margin,
									btSoftBody::sCti& cti)
{
	btVector3			nrm;
	btCollisionShape*	shp=prb->getCollisionShape();
	const btTransform&	wtr=prb->getInterpolationWorldTransform();
	btScalar			dst=psb->m_worldInfo->m_sparsesdf.Evaluate(	wtr.invXform(x),
																	shp,
																	nrm,
																	margin);
	if(dst<0)
	{
		cti.m_body		=	prb;
		cti.m_normal	=	wtr.getBasis()*nrm;
		cti.m_offset	=	-dot(	cti.m_normal,
									x-cti.m_normal*dst);
		return(true);
	}
	return(false);
}

//
static void			UpdateNormals(btSoftBody* psb)
{
	const btVector3	zv(0,0,0);
	int i,ni;

	for(i=0,ni=psb->m_nodes.size();i<ni;++i)
	{
		psb->m_nodes[i].m_n=zv;
	}
	for(i=0,ni=psb->m_faces.size();i<ni;++i)
	{
		btSoftBody::Face&	f=psb->m_faces[i];
		const btVector3		n=cross(f.m_n[1]->m_x-f.m_n[0]->m_x,
			f.m_n[2]->m_x-f.m_n[0]->m_x);
		f.m_normal=n.normalized();
		f.m_n[0]->m_n+=n;
		f.m_n[1]->m_n+=n;
		f.m_n[2]->m_n+=n;
	}
	for(i=0,ni=psb->m_nodes.size();i<ni;++i)
	{
		psb->m_nodes[i].m_n.normalize();
	}
}

//
static void			UpdateBounds(btSoftBody* psb)
{
	if(psb->m_ndbvt.m_root)
		{
		const btVector3&	mins=psb->m_ndbvt.m_root->volume.Mins();
		const btVector3&	maxs=psb->m_ndbvt.m_root->volume.Maxs();
		const btScalar		csm=psb->getCollisionShape()->getMargin();
		const btVector3		mrg=btVector3(	csm,
											csm,
											csm)*1; // ??? to investigate...
		psb->m_bounds[0]=mins-mrg;
		psb->m_bounds[1]=maxs+mrg;
			if(0!=psb->getBroadphaseHandle())
			{					
				psb->m_worldInfo->m_broadphase->setAabb(psb->getBroadphaseHandle(),
														psb->m_bounds[0],
														psb->m_bounds[1],
														psb->m_worldInfo->m_dispatcher);
			}
		}
		else
		{
		psb->m_bounds[0]=
		psb->m_bounds[1]=btVector3(0,0,0);
		}		
}


//
static void			UpdatePose(btSoftBody* psb)
{
	if(psb->m_pose.m_bframe)
	{
		btSoftBody::Pose&	pose=psb->m_pose;
		const btVector3		com=EvaluateCom(psb);
		/* Com			*/ 
		pose.m_com	=	com;
		/* Rotation		*/ 
		btMatrix3x3		Apq;
		const btScalar	eps=1/(btScalar)(100*psb->m_nodes.size());
		Apq[0]=Apq[1]=Apq[2]=btVector3(0,0,0);
		Apq[0].setX(eps);Apq[1].setY(eps*2);Apq[2].setZ(eps*3);
		for(int i=0,ni=psb->m_nodes.size();i<ni;++i)
		{
			const btVector3		a=pose.m_wgh[i]*(psb->m_nodes[i].m_x-com);
			const btVector3&	b=pose.m_pos[i];
			Apq[0]+=a.x()*b;
			Apq[1]+=a.y()*b;
			Apq[2]+=a.z()*b;
		}
		btMatrix3x3		r,s;
		PolarDecompose(Apq,r,s);
		pose.m_rot=r;
		pose.m_scl=pose.m_aqq*r.transpose()*Apq;
		if(psb->m_cfg.maxvolume>1)
			{
			const btScalar	idet=Clamp<btScalar>(	1/pose.m_scl.determinant(),
													1,psb->m_cfg.maxvolume);
			pose.m_scl=Mul(pose.m_scl,idet);
			}
		
	}
}

//
static void			UpdateConstants(btSoftBody* psb)
{
	int i,ni;

	/* Links		*/ 
	for(i=0,ni=psb->m_links.size();i<ni;++i)
	{
		btSoftBody::Link&		l=psb->m_links[i];
		btSoftBody::Material&	m=*l.m_material;
		l.m_rl	=	(l.m_n[0]->m_x-l.m_n[1]->m_x).length();
		l.m_c0	=	(l.m_n[0]->m_im+l.m_n[1]->m_im)/m.m_kLST;
		l.m_c1	=	l.m_rl*l.m_rl;
	}
	/* Faces		*/ 
	for(i=0,ni=psb->m_faces.size();i<ni;++i)
	{
		btSoftBody::Face&	f=psb->m_faces[i];
		f.m_ra	=	AreaOf(f.m_n[0]->m_x,f.m_n[1]->m_x,f.m_n[2]->m_x);
	}
	/* Area's		*/ 
	btAlignedObjectArray<int>	counts;
	counts.resize(psb->m_nodes.size(),0);
	for(i=0,ni=psb->m_nodes.size();i<ni;++i)
	{
		psb->m_nodes[i].m_area	=	0;
	}
	for(i=0,ni=psb->m_faces.size();i<ni;++i)
	{
		btSoftBody::Face&	f=psb->m_faces[i];
		for(int j=0;j<3;++j)
		{
			const int index=(int)(f.m_n[j]-&psb->m_nodes[0]);
			counts[index]++;
			f.m_n[j]->m_area+=btFabs(f.m_ra);
		}
	}
	for(i=0,ni=psb->m_nodes.size();i<ni;++i)
	{
		if(counts[i]>0)
			psb->m_nodes[i].m_area/=(btScalar)counts[i];
		else
			psb->m_nodes[i].m_area=0;
	}
}

//
static inline void	ApplyClampedForce(	btSoftBody::Node& n,
	const btVector3& f,
	btScalar dt)
{
	const btScalar	dtim=dt*n.m_im;
	if((f*dtim).length2()>n.m_v.length2())
	{/* Clamp	*/ 
		n.m_f-=ProjectOnAxis(n.m_v,f.normalized())/dtim;						
	}
	else
	{/* Apply	*/ 
		n.m_f+=f;
	}
}

//
static void			ApplyForces(btSoftBody* psb,btScalar dt)
{
	BT_PROFILE("SoftBody applyForces");
	int i,ni;
	const btScalar					kLF=psb->m_cfg.kLF;
	const btScalar					kDG=psb->m_cfg.kDG;
	const btScalar					kPR=psb->m_cfg.kPR;
	const btScalar					kVC=psb->m_cfg.kVC;
	const bool						as_lift=kLF>0;
	const bool						as_drag=kDG>0;
	const bool						as_pressure=kPR!=0;
	const bool						as_volume=kVC>0;
	const bool						as_aero=	as_lift		||
												as_drag		;
	const bool						as_vaero=	as_aero		&&
												(psb->m_cfg.aeromodel<btSoftBody::eAeroModel::F_TwoSided);
	const bool						as_faero=	as_aero		&&
												(psb->m_cfg.aeromodel>=btSoftBody::eAeroModel::F_TwoSided);
	const bool						use_medium=	as_aero;
	const bool						use_volume=	as_pressure	||
												as_volume	;
	btScalar						volume=0;
	btScalar						ivolumetp=0;
	btScalar						dvolumetv=0;
	btSoftBody::sMedium	medium;
	if(use_volume)
	{
		volume		=	psb->getVolume();
		ivolumetp	=	1/btFabs(volume)*kPR;
		dvolumetv	=	(psb->m_pose.m_volume-volume)*kVC;
	}
	/* Per vertex forces			*/ 
	for(i=0,ni=psb->m_nodes.size();i<ni;++i)
	{
		btSoftBody::Node&	n=psb->m_nodes[i];
		if(n.m_im>0)
		{
			if(use_medium)
			{
				EvaluateMedium(psb->m_worldInfo,n.m_x,medium);
				/* Aerodynamics			*/ 
				if(as_vaero)
				{				
					const btVector3	rel_v=n.m_v-medium.m_velocity;
					const btScalar	rel_v2=rel_v.length2();
					if(rel_v2>SIMD_EPSILON)
					{
						btVector3	nrm=n.m_n;
						/* Setup normal		*/ 
						switch(psb->m_cfg.aeromodel)
						{
						case	btSoftBody::eAeroModel::V_Point:
							nrm=NormalizeAny(rel_v);break;
						case	btSoftBody::eAeroModel::V_TwoSided:
							nrm*=(btScalar)(dot(nrm,rel_v)<0?-1:+1);break;
						}
						const btScalar	dvn=dot(rel_v,nrm);
						/* Compute forces	*/ 
						if(dvn>0)
						{
							btVector3		force(0,0,0);
							const btScalar	c0	=	n.m_area*dvn*rel_v2/2;
							const btScalar	c1	=	c0*medium.m_density;
							force	+=	nrm*(-c1*kLF);
							force	+=	rel_v.normalized()*(-c1*kDG);
							ApplyClampedForce(n,force,dt);
						}
					}
				}
			}
			/* Pressure				*/ 
			if(as_pressure)
			{
				n.m_f	+=	n.m_n*(n.m_area*ivolumetp);
			}
			/* Volume				*/ 
			if(as_volume)
			{
				n.m_f	+=	n.m_n*(n.m_area*dvolumetv);
			}
		}
	}
	/* Per face forces				*/ 
	for(i=0,ni=psb->m_faces.size();i<ni;++i)
	{
		btSoftBody::Face&	f=psb->m_faces[i];
		if(as_faero)
		{
			const btVector3	v=(f.m_n[0]->m_v+f.m_n[1]->m_v+f.m_n[2]->m_v)/3;
			const btVector3	x=(f.m_n[0]->m_x+f.m_n[1]->m_x+f.m_n[2]->m_x)/3;
			EvaluateMedium(psb->m_worldInfo,x,medium);
			const btVector3	rel_v=v-medium.m_velocity;
			const btScalar	rel_v2=rel_v.length2();
			if(rel_v2>SIMD_EPSILON)
			{
				btVector3	nrm=f.m_normal;
				/* Setup normal		*/ 
				switch(psb->m_cfg.aeromodel)
				{
				case	btSoftBody::eAeroModel::F_TwoSided:
					nrm*=(btScalar)(dot(nrm,rel_v)<0?-1:+1);break;
				}
				const btScalar	dvn=dot(rel_v,nrm);
				/* Compute forces	*/ 
				if(dvn>0)
				{
					btVector3		force(0,0,0);
					const btScalar	c0	=	f.m_ra*dvn*rel_v2;
					const btScalar	c1	=	c0*medium.m_density;
					force	+=	nrm*(-c1*kLF);
					force	+=	rel_v.normalized()*(-c1*kDG);
					force	/=	3;
					for(int j=0;j<3;++j) ApplyClampedForce(*f.m_n[j],force,dt);
				}
			}
		}
	}
}

//
static void			PSolve_Anchors(btSoftBody* psb,btScalar kst)
{
	const btScalar	kAHR=psb->m_cfg.kAHR*kst;
	const btScalar	dt=psb->m_sst.sdt;
	for(int i=0,ni=psb->m_anchors.size();i<ni;++i)
	{
		const btSoftBody::Anchor&	a=psb->m_anchors[i];
		const btTransform&			t=a.m_body->getInterpolationWorldTransform();
		btSoftBody::Node&			n=*a.m_node;
		const btVector3				wa=t*a.m_local;
		const btVector3				va=a.m_body->getVelocityInLocalPoint(a.m_c1)*dt;
		const btVector3				vb=n.m_x-n.m_q;
		const btVector3				vr=(va-vb)+(wa-n.m_x)*kAHR;
		const btVector3				impulse=a.m_c0*vr;
		n.m_x+=impulse*a.m_c2;
		a.m_body->applyImpulse(-impulse,a.m_c1);
	}
}

//
static void			PSolve_RContacts(btSoftBody* psb,btScalar kst)
{
	const btScalar	dt=psb->m_sst.sdt;
	const btScalar	mrg=psb->getCollisionShape()->getMargin();
	for(int i=0,ni=psb->m_rcontacts.size();i<ni;++i)
	{
		const btSoftBody::RContact&	c=psb->m_rcontacts[i];
		const btSoftBody::sCti&		cti=c.m_cti;	
		const btVector3		va=cti.m_body->getVelocityInLocalPoint(c.m_c1)*dt;
		const btVector3		vb=c.m_node->m_x-c.m_node->m_q;	
		const btVector3		vr=vb-va;
		const btScalar		dn=dot(vr,cti.m_normal);		
		if(dn<=SIMD_EPSILON)
		{
			const btScalar		dp=btMin(dot(c.m_node->m_x,cti.m_normal)+cti.m_offset,mrg);
			const btVector3		fv=vr-cti.m_normal*dn;
			const btVector3		impulse=c.m_c0*((vr-fv*c.m_c3+cti.m_normal*(dp*c.m_c4))*kst);
			c.m_node->m_x-=impulse*c.m_c2;
			c.m_cti.m_body->applyImpulse(impulse,c.m_c1);
		}
	}
}

//
static void			PSolve_SContacts(btSoftBody* psb,btScalar)
{
for(int i=0,ni=psb->m_scontacts.size();i<ni;++i)
	{
	const btSoftBody::SContact&	c=psb->m_scontacts[i];
	const btVector3&	nr=c.m_normal;
	btSoftBody::Node&	n=*c.m_node;
	btSoftBody::Face&	f=*c.m_face;
	const btVector3		p=BaryEval(	f.m_n[0]->m_x,
									f.m_n[1]->m_x,
									f.m_n[2]->m_x,
									c.m_weights);
	const btVector3		q=BaryEval(	f.m_n[0]->m_q,
									f.m_n[1]->m_q,
									f.m_n[2]->m_q,
									c.m_weights);											
	const btVector3		vr=(n.m_x-n.m_q)-(p-q);
	btVector3			corr(0,0,0);
	if(dot(vr,nr)<0)
		{
		const btScalar	j=c.m_margin-(dot(nr,n.m_x)-dot(nr,p));
		corr+=c.m_normal*j;
		}
	corr			-=	ProjectOnPlane(vr,nr)*c.m_friction;
	n.m_x			+=	corr*c.m_cfm[0];
	f.m_n[0]->m_x	-=	corr*(c.m_cfm[1]*c.m_weights.x());
	f.m_n[1]->m_x	-=	corr*(c.m_cfm[1]*c.m_weights.y());
	f.m_n[2]->m_x	-=	corr*(c.m_cfm[1]*c.m_weights.z());
	}
}

//
static void			PSolve_Links(btSoftBody* psb,btScalar kst)
{
	for(int i=0,ni=psb->m_links.size();i<ni;++i)
	{			
		btSoftBody::Link&	l=psb->m_links[i];
		if(l.m_c0>0)
		{
			btSoftBody::Node&	a=*l.m_n[0];
			btSoftBody::Node&	b=*l.m_n[1];
			const btVector3		del=b.m_x-a.m_x;
			const btScalar		len=del.length2();
			const btScalar		k=((l.m_c1-len)/(l.m_c0*(l.m_c1+len)))*kst;
			a.m_x-=del*(k*a.m_im);
			b.m_x+=del*(k*b.m_im);
		}
	}
}

//
static void			VSolve_Links(btSoftBody* psb,btScalar kst)
{
for(int i=0,ni=psb->m_links.size();i<ni;++i)
	{			
	btSoftBody::Link&	l=psb->m_links[i];
	btSoftBody::Node**	n=l.m_n;
	const btScalar		j=-dot(l.m_c3,n[0]->m_v-n[1]->m_v)*l.m_c2*kst;
	n[0]->m_v+=	l.m_c3*(j*n[0]->m_im);
	n[1]->m_v-=	l.m_c3*(j*n[1]->m_im);
	}
}

//
static void (* const VSolvers[])(btSoftBody*,btScalar)=	{
														VSolve_Links,
														};
												
//
static void (* const PSolvers[])(btSoftBody*,btScalar)=	{
														PSolve_Links,
														PSolve_Anchors,
														PSolve_RContacts,
														PSolve_SContacts,
														};

//
// btSoftBody
//

//
btSoftBody::btSoftBody(btSoftBody::btSoftBodyWorldInfo*	worldInfo,int node_count,  const btVector3* x,  const btScalar* m)
:m_worldInfo(worldInfo)
{	
	/* Init		*/ 
	m_internalType		=	CO_SOFT_BODY;
	m_cfg.aeromodel		=	eAeroModel::V_Point;
	m_cfg.kVCF			=	1;
	m_cfg.kDG			=	0;
	m_cfg.kLF			=	0;
	m_cfg.kDP			=	0;
	m_cfg.kPR			=	0;
	m_cfg.kVC			=	0;
	m_cfg.kDF			=	(btScalar)0.2;
	m_cfg.kMT			=	0;
	m_cfg.kCHR			=	(btScalar)1.0;
	m_cfg.kKHR			=	(btScalar)0.1;
	m_cfg.kSHR			=	(btScalar)1.0;
	m_cfg.kAHR			=	(btScalar)0.7;
	m_cfg.maxvolume		=	(btScalar)1;
	m_cfg.timescale		=	1;
	m_cfg.viterations	=	0;
	m_cfg.piterations	=	1;
	m_cfg.diterations	=	0;
	m_cfg.collisions	=	fCollision::Default;
	m_pose.m_bvolume	=	false;
	m_pose.m_bframe		=	false;
	m_pose.m_volume		=	0;
	m_pose.m_com		=	btVector3(0,0,0);
	m_pose.m_rot.setIdentity();
	m_pose.m_scl.setIdentity();
	m_tag				=	0;
	m_timeacc			=	0;
	m_bUpdateRtCst		=	true;
	m_bounds[0]			=	btVector3(0,0,0);
	m_bounds[1]			=	btVector3(0,0,0);
	m_worldTransform.setIdentity();
	setSolver(eSolverPresets::Positions);
	/* Default material	*/ 
	Material*	pm=appendMaterial();
	pm->m_kLST	=	1;
	pm->m_kAST	=	1;
	pm->m_kVST	=	1;
	pm->m_flags	=	fMaterial::Default;
	/* Collision shape	*/ 
	///for now, create a collision shape internally
	setCollisionShape(new btSoftBodyCollisionShape(this));	
	m_collisionShape->setMargin(0.25);
	/* Nodes			*/ 
	const btScalar		margin=getCollisionShape()->getMargin();
	m_nodes.resize(node_count);
	for(int i=0,ni=node_count;i<ni;++i)
	{	
		Node&	n=m_nodes[i];
		ZeroInitialize(n);
		n.m_x		=	x?*x++:btVector3(0,0,0);
		n.m_q		=	n.m_x;
		n.m_im		=	m?*m++:1;
		n.m_im		=	n.m_im>0?1/n.m_im:0;
		n.m_leaf	=	m_ndbvt.insert(btDbvtVolume::FromCR(n.m_x,margin),&n);
		n.m_material=	pm;
	}
	UpdateBounds(this);	


}

//
btSoftBody::~btSoftBody()
{
	//for now, delete the internal shape
	delete m_collisionShape;	
	for(int i=0;i<m_materials.size();++i) btAlignedFree(m_materials[i]);
}

//
bool			btSoftBody::checkLink(int node0,int node1) const
{
	return(checkLink(&m_nodes[node0],&m_nodes[node1]));
}

//
bool			btSoftBody::checkLink(const Node* node0,const Node* node1) const
{
	const Node*	n[]={node0,node1};
	for(int i=0,ni=m_links.size();i<ni;++i)
	{
		const Link&	l=m_links[i];
		if(	(l.m_n[0]==n[0]&&l.m_n[1]==n[1])||
			(l.m_n[0]==n[1]&&l.m_n[1]==n[0]))
		{
			return(true);
		}
	}
	return(false);
}

//
bool			btSoftBody::checkFace(int node0,int node1,int node2) const
{
	const Node*	n[]={	&m_nodes[node0],
		&m_nodes[node1],
		&m_nodes[node2]};
	for(int i=0,ni=m_faces.size();i<ni;++i)
	{
		const Face&	f=m_faces[i];
		int			c=0;
		for(int j=0;j<3;++j)
		{
			if(	(f.m_n[j]==n[0])||
				(f.m_n[j]==n[1])||
				(f.m_n[j]==n[2])) c|=1<<j; else break;
		}
		if(c==7) return(true);
	}
	return(false);
}

//
btSoftBody::Material*		btSoftBody::appendMaterial()
{
Material*	pm=new(btAlignedAlloc(sizeof(Material),16)) Material();
if(m_materials.size()>0)
	*pm=*m_materials[0];
	else
	ZeroInitialize(*pm);
m_materials.push_back(pm);
return(pm);
}

//
void			btSoftBody::appendNote(	const char* text,
										const btVector3& o,
										const btVector4& c,
										Node* n0,
										Node* n1,
										Node* n2,
										Node* n3)
{
Note	n;
ZeroInitialize(n);
n.m_rank		=	0;
n.m_text		=	text;
n.m_offset		=	o;
n.m_coords[0]	=	c.x();
n.m_coords[1]	=	c.y();
n.m_coords[2]	=	c.z();
n.m_coords[3]	=	c.w();
n.m_nodes[0]	=	n0;n.m_rank+=n0?1:0;
n.m_nodes[1]	=	n1;n.m_rank+=n1?1:0;
n.m_nodes[2]	=	n2;n.m_rank+=n2?1:0;
n.m_nodes[3]	=	n3;n.m_rank+=n3?1:0;
m_notes.push_back(n);
}

//
void			btSoftBody::appendNote(	const char* text,
										const btVector3& o,
										Node* feature)
{
appendNote(text,o,btVector4(1,0,0,0),feature);
}

//
void			btSoftBody::appendNote(	const char* text,
										const btVector3& o,
										Link* feature)
{
static const btScalar	w=1/(btScalar)2;
appendNote(text,o,btVector4(w,w,0,0),	feature->m_n[0],
										feature->m_n[1]);
}
								
//
void			btSoftBody::appendNote(	const char* text,
										const btVector3& o,
										Face* feature)
{
static const btScalar	w=1/(btScalar)3;
appendNote(text,o,btVector4(w,w,w,0),	feature->m_n[0],
										feature->m_n[1],
										feature->m_n[2]);
}

//
void			btSoftBody::appendNode(	const btVector3& x,btScalar m)
{
if(m_nodes.capacity()==m_nodes.size())
	{
	PointersToIndices(this);
	m_nodes.reserve(m_nodes.size()*2+1);
	IndicesToPointers(this);
	}
const btScalar	margin=getCollisionShape()->getMargin();
m_nodes.push_back(Node());
Node&			n=m_nodes[m_nodes.size()-1];
ZeroInitialize(n);
n.m_x			=	x;
n.m_q			=	n.m_x;
n.m_im			=	m>0?1/m:0;
n.m_material	=	m_materials[0];
n.m_leaf		=	m_ndbvt.insert(btDbvtVolume::FromCR(n.m_x,margin),&n);
}

//
void			btSoftBody::appendLink(int model,Material* mat)
{
Link	l;
if(model>=0)
	l=m_links[model];
	else
	{ ZeroInitialize(l);l.m_material=mat?mat:m_materials[0]; }
m_links.push_back(l);
}

//
void			btSoftBody::appendLink(	int node0,
									   int node1,
									   Material* mat,
									   bool bcheckexist)
{
	appendLink(&m_nodes[node0],&m_nodes[node1],mat,bcheckexist);
}

//
void			btSoftBody::appendLink(	Node* node0,
									   Node* node1,
									   Material* mat,
									   bool bcheckexist)
{
	if((!bcheckexist)||(!checkLink(node0,node1)))
	{
		appendLink(-1,mat);
		Link&	l=m_links[m_links.size()-1];
		l.m_n[0]		=	node0;
		l.m_n[1]		=	node1;
		l.m_rl			=	(l.m_n[0]->m_x-l.m_n[1]->m_x).length();
		m_bUpdateRtCst=true;
	}
}

//
void			btSoftBody::appendFace(int model,Material* mat)
{
Face	f;
if(model>=0)
	{ f=m_faces[model]; }
	else
	{ ZeroInitialize(f);f.m_material=mat?mat:m_materials[0]; }
m_faces.push_back(f);
}

//
void			btSoftBody::appendFace(int node0,int node1,int node2,Material* mat)
{
	appendFace(-1,mat);
	Face&	f=m_faces[m_faces.size()-1];
	f.m_n[0]	=	&m_nodes[node0];
	f.m_n[1]	=	&m_nodes[node1];
	f.m_n[2]	=	&m_nodes[node2];
	f.m_ra		=	AreaOf(	f.m_n[0]->m_x,
		f.m_n[1]->m_x,
		f.m_n[2]->m_x);	
	m_bUpdateRtCst=true;
}

//
void			btSoftBody::appendAnchor(int node,btRigidBody* body)
{
	Anchor	a;
	a.m_node			=	&m_nodes[node];
	a.m_body			=	body;
	a.m_local			=	body->getInterpolationWorldTransform().inverse()*a.m_node->m_x;
	a.m_node->m_battach	=	1;
	m_anchors.push_back(a);
}

//
void			btSoftBody::addForce(const btVector3& force)
{
	for(int i=0,ni=m_nodes.size();i<ni;++i) addForce(force,i);
}

//
void			btSoftBody::addForce(const btVector3& force,int node)
{
	Node&	n=m_nodes[node];
	if(n.m_im>0)
	{
		n.m_f	+=	force;
	}
}

//
void			btSoftBody::addVelocity(const btVector3& velocity)
{
	for(int i=0,ni=m_nodes.size();i<ni;++i) addVelocity(velocity,i);
}

//
void			btSoftBody::addVelocity(const btVector3& velocity,int node)
{
	Node&	n=m_nodes[node];
	if(n.m_im>0)
	{
		n.m_v	+=	velocity;
	}
}

//
void			btSoftBody::setMass(int node,btScalar mass)
{
	m_nodes[node].m_im=mass>0?1/mass:0;
	m_bUpdateRtCst=true;
}

//
btScalar		btSoftBody::getMass(int node) const
{
	return(m_nodes[node].m_im>0?1/m_nodes[node].m_im:0);
}

//
btScalar		btSoftBody::getTotalMass() const
{
	btScalar	mass=0;
	for(int i=0;i<m_nodes.size();++i)
	{
		mass+=getMass(i);
	}
	return(mass);
}

//
void			btSoftBody::setTotalMass(btScalar mass,bool fromfaces)
{
	int i;
	if(fromfaces)
	{


		for(i=0;i<m_nodes.size();++i)
		{
			m_nodes[i].m_im=0;
		}
		for(i=0;i<m_faces.size();++i)
		{
			const Face&		f=m_faces[i];
			const btScalar	twicearea=AreaOf(	f.m_n[0]->m_x,
												f.m_n[1]->m_x,
												f.m_n[2]->m_x);
			for(int j=0;j<3;++j)
			{
				f.m_n[j]->m_im+=twicearea;
			}
		}
		for(i=0;i<m_nodes.size();++i)
		{
			m_nodes[i].m_im=1/m_nodes[i].m_im;
		}
	}
	const btScalar	tm=getTotalMass();
	const btScalar	itm=1/tm;
	for( i=0;i<m_nodes.size();++i)
	{
		m_nodes[i].m_im/=itm*mass;
	}
	m_bUpdateRtCst=true;
}

//
void			btSoftBody::setTotalDensity(btScalar density)
{
	setTotalMass(getVolume()*density,true);
}

//
void			btSoftBody::transform(const btTransform& trs)
{
	const btScalar	margin=getCollisionShape()->getMargin();
	for(int i=0,ni=m_nodes.size();i<ni;++i)
	{
		Node&	n=m_nodes[i];
		n.m_x=trs*n.m_x;
		n.m_q=trs*n.m_q;
		n.m_n=trs.getBasis()*n.m_n;
		m_ndbvt.update(n.m_leaf,btDbvtVolume::FromCR(n.m_x,margin));
	}
	UpdateNormals(this);
	UpdateBounds(this);
	UpdateConstants(this);
}

//
void			btSoftBody::translate(const btVector3& trs)
{
btTransform	t;
t.setIdentity();
t.setOrigin(trs);
transform(t);
}

//
void			btSoftBody::rotate(	const btQuaternion& rot)
{
btTransform	t;
t.setIdentity();
t.setRotation(rot);
transform(t);
}

//
void			btSoftBody::scale(const btVector3& scl)
{
	const btScalar	margin=getCollisionShape()->getMargin();
	for(int i=0,ni=m_nodes.size();i<ni;++i)
	{
		Node&	n=m_nodes[i];
		n.m_x*=scl;
		n.m_q*=scl;
		m_ndbvt.update(n.m_leaf,btDbvtVolume::FromCR(n.m_x,margin));
	}
	UpdateNormals(this);
	UpdateBounds(this);
	UpdateConstants(this);
}

//
void			btSoftBody::setPose(bool bvolume,bool bframe)
{
	m_pose.m_bvolume	=	bvolume;
	m_pose.m_bframe		=	bframe;
	/* Weights		*/ 
	const btScalar	omass=getTotalMass();
	const btScalar	kmass=omass*m_nodes.size()*1000;
	btScalar		tmass=omass;
	m_pose.m_wgh.resize(m_nodes.size());
	int i,ni;

	for(i=0,ni=m_nodes.size();i<ni;++i)
	{
		if(m_nodes[i].m_im<=0) tmass+=kmass;
	}
	for(i=0,ni=m_nodes.size();i<ni;++i)
	{
		Node&	n=m_nodes[i];
		m_pose.m_wgh[i]=	n.m_im>0					?
							1/(m_nodes[i].m_im*tmass)	:
							kmass/tmass;
	}
	/* Pos		*/ 
	const btVector3	com=EvaluateCom(this);
	m_pose.m_pos.resize(m_nodes.size());
	for(i=0,ni=m_nodes.size();i<ni;++i)
	{
		m_pose.m_pos[i]=m_nodes[i].m_x-com;
	}
	m_pose.m_volume	=	bvolume?getVolume():0;
	m_pose.m_com	=	com;
	m_pose.m_rot.setIdentity();
	m_pose.m_scl.setIdentity();
	/* Aqq		*/ 
	m_pose.m_aqq[0]	=
	m_pose.m_aqq[1]	=
	m_pose.m_aqq[2]	=	btVector3(0,0,0);
	for(i=0,ni=m_nodes.size();i<ni;++i)
		{
		const btVector3&	q=m_pose.m_pos[i];
		const btVector3		mq=m_pose.m_wgh[i]*q;
		m_pose.m_aqq[0]+=mq.x()*q;
		m_pose.m_aqq[1]+=mq.y()*q;
		m_pose.m_aqq[2]+=mq.z()*q;
		}
	m_pose.m_aqq=m_pose.m_aqq.inverse();
	UpdateConstants(this);
}

//
btScalar		btSoftBody::getVolume() const
{
	btScalar	vol=0;
	if(m_nodes.size()>0)
	{
		const btVector3	org=m_nodes[0].m_x;
		for(int i=0,ni=m_faces.size();i<ni;++i)
		{
			const Face&	f=m_faces[i];
			vol+=dot(f.m_n[0]->m_x-org,cross(f.m_n[1]->m_x-org,f.m_n[2]->m_x-org));
		}
		vol/=(btScalar)6;
	}
	return(vol);
}

//
int				btSoftBody::generateBendingConstraints(int distance,Material* mat)
{
	if(distance>1)
	{
		/* Build graph	*/ 
		const int		n=m_nodes.size();
		const unsigned	inf=(~(unsigned)0)>>1;
		unsigned*		adj=new unsigned[n*n];
#define IDX(_x_,_y_)	((_y_)*n+(_x_))

		int i,j;

		for( j=0;j<n;++j)
		{
			for(int i=0;i<n;++i)
			{
				if(i!=j)	adj[IDX(i,j)]=adj[IDX(j,i)]=inf;
				else
					adj[IDX(i,j)]=adj[IDX(j,i)]=0;
			}
		}
		for(i=0;i<m_links.size();++i)
		{
			const int	ia=(int)(m_links[i].m_n[0]-&m_nodes[0]);
			const int	ib=(int)(m_links[i].m_n[1]-&m_nodes[0]);
			adj[IDX(ia,ib)]=1;
			adj[IDX(ib,ia)]=1;
		}
		for(int k=0;k<n;++k)
		{
			for(j=0;j<n;++j)
			{
				for(int i=j+1;i<n;++i)
				{
					const unsigned	sum=adj[IDX(i,k)]+adj[IDX(k,j)];
					if(adj[IDX(i,j)]>sum)
					{
						adj[IDX(i,j)]=adj[IDX(j,i)]=sum;
					}
				}
			}
		}
		/* Build links	*/ 
		int	nlinks=0;
		for(j=0;j<n;++j)
		{
			for(int i=j+1;i<n;++i)
			{
				if(adj[IDX(i,j)]==(unsigned)distance)
				{
					appendLink(i,j,mat);
					m_links[m_links.size()-1].m_bbending=1;
					++nlinks;
				}
			}
		}
		delete[] adj;		
		return(nlinks);
	}
	return(0);
}

//
void			btSoftBody::randomizeConstraints()
{
unsigned long	seed=243703;
#define NEXTRAND (seed=(1664525L*seed+1013904223L)&0xffffffff)
	int i,ni;


	for(i=0,ni=m_links.size();i<ni;++i)
	{
		btSwap(m_links[i],m_links[NEXTRAND%ni]);
	}
	for(i=0,ni=m_faces.size();i<ni;++i)
	{
		btSwap(m_faces[i],m_faces[NEXTRAND%ni]);
	}
#undef NEXTRAND
}

//
void			btSoftBody::refine(ImplicitFn* ifn,btScalar accurary,bool cut)
{
Node*			nbase = &m_nodes[0];
int					ncount(m_nodes.size());
btSymMatrix<int>	edges(ncount,-2);
int					newnodes=0;
int i,ni;

/* Filter out		*/ 
for(i=0;i<m_links.size();++i)
	{
	Link&	l=m_links[i];
	if(l.m_bbending)
		{
		if(!SameSign(ifn->Eval(l.m_n[0]->m_x),ifn->Eval(l.m_n[1]->m_x)))
			{
			btSwap(m_links[i],m_links[m_links.size()-1]);
			m_links.pop_back();--i;
			}
		}	
	}
/* Fill edges		*/ 
for(i=0;i<m_links.size();++i)
	{
	Link&	l=m_links[i];
	edges(int(l.m_n[0]-nbase),int(l.m_n[1]-nbase))=-1;
	}
for(i=0;i<m_faces.size();++i)
	{	
	Face&	f=m_faces[i];
	edges(int(f.m_n[0]-nbase),int(f.m_n[1]-nbase))=-1;
	edges(int(f.m_n[1]-nbase),int(f.m_n[2]-nbase))=-1;
	edges(int(f.m_n[2]-nbase),int(f.m_n[0]-nbase))=-1;
	}
/* Intersect		*/ 
for(i=0;i<ncount;++i)
	{
	for(int j=i+1;j<ncount;++j)
		{
		if(edges(i,j)==-1)
			{
			Node&			a=m_nodes[i];
			Node&			b=m_nodes[j];
			const btScalar	t=ImplicitSolve(ifn,a.m_x,b.m_x,accurary);
			if(t>0)
				{
				const btVector3	x=Lerp(a.m_x,b.m_x,t);
				const btVector3	v=Lerp(a.m_v,b.m_v,t);
				btScalar		m=0;
				if(a.m_im>0)
					{
					if(b.m_im>0)
						{
						const btScalar	ma=1/a.m_im;
						const btScalar	mb=1/b.m_im;
						const btScalar	mc=Lerp(ma,mb,t);
						const btScalar	f=(ma+mb)/(ma+mb+mc);
						a.m_im=1/(ma*f);
						b.m_im=1/(mb*f);
						m=mc*f;
						}
						else
						{ a.m_im/=0.5;m=1/a.m_im; }
					}
					else
					{
					if(b.m_im>0)
						{ b.m_im/=0.5;m=1/b.m_im; }
						else
						m=0;
					}
				appendNode(x,m);
				edges(i,j)=m_nodes.size()-1;
				m_nodes[edges(i,j)].m_v=v;
				++newnodes;
				}
			}
		}
	}
nbase=&m_nodes[0];
/* Refine links		*/ 
for(i=0,ni=m_links.size();i<ni;++i)
	{
	Link&		feat=m_links[i];
	const int	idx[]={	int(feat.m_n[0]-nbase),
						int(feat.m_n[1]-nbase)};
	if((idx[0]<ncount)&&(idx[1]<ncount))
		{
		const int ni=edges(idx[0],idx[1]);
		if(ni>0)
			{
			appendLink(i);
			Link*		pft[]={	&m_links[i],
								&m_links[m_links.size()-1]};			
			pft[0]->m_n[0]=&m_nodes[idx[0]];
			pft[0]->m_n[1]=&m_nodes[ni];
			pft[1]->m_n[0]=&m_nodes[ni];
			pft[1]->m_n[1]=&m_nodes[idx[1]];
			}
		}
	}
/* Refine faces		*/ 
for(i=0;i<m_faces.size();++i)
	{
	const Face&	feat=m_faces[i];
	const int	idx[]={	int(feat.m_n[0]-nbase),
						int(feat.m_n[1]-nbase),
						int(feat.m_n[2]-nbase)};
	for(int j=2,k=0;k<3;j=k++)
		{
		if((idx[j]<ncount)&&(idx[k]<ncount))
			{
			const int ni=edges(idx[j],idx[k]);
			if(ni>0)
				{
				appendFace(i);
				const int	l=(k+1)%3;
				Face*		pft[]={	&m_faces[i],
									&m_faces[m_faces.size()-1]};
				pft[0]->m_n[0]=&m_nodes[idx[l]];
				pft[0]->m_n[1]=&m_nodes[idx[j]];
				pft[0]->m_n[2]=&m_nodes[ni];
				pft[1]->m_n[0]=&m_nodes[ni];
				pft[1]->m_n[1]=&m_nodes[idx[k]];
				pft[1]->m_n[2]=&m_nodes[idx[l]];
				appendLink(ni,idx[l],pft[0]->m_material);
				--i;break;
				}
			}
		}
	}
/* Cut				*/ 
if(cut)
	{	
	btAlignedObjectArray<int>	cnodes;
	const int					pcount=ncount;
	ncount=m_nodes.size();
	cnodes.resize(ncount,0);
	/* Nodes		*/ 
	for(i=0;i<ncount;++i)
		{
		const btVector3	x=m_nodes[i].m_x;
		if((i>=pcount)||(btFabs(ifn->Eval(x))<accurary))
			{
			const btVector3	v=m_nodes[i].m_v;
			btScalar		m=getMass(i);
			if(m>0) { m*=0.5;m_nodes[i].m_im/=0.5; }
			appendNode(x,m);
			cnodes[i]=m_nodes.size()-1;
			m_nodes[cnodes[i]].m_v=v;
			}
		}
	nbase=&m_nodes[0];
	/* Links		*/ 
	for(i=0,ni=m_links.size();i<ni;++i)
		{
		const int		id[]={	int(m_links[i].m_n[0]-nbase),
								int(m_links[i].m_n[1]-nbase)};
		int				todetach=0;
		if(cnodes[id[0]]&&cnodes[id[1]])
			{
			appendLink(i);
			todetach=m_links.size()-1;
			}
		else
			{
			if((	(ifn->Eval(m_nodes[id[0]].m_x)<accurary)&&
					(ifn->Eval(m_nodes[id[1]].m_x)<accurary)))
				todetach=i;
			}
		if(todetach)
			{
			Link&	l=m_links[todetach];
			for(int j=0;j<2;++j)
				{
				int cn=cnodes[int(l.m_n[j]-nbase)];
				if(cn) l.m_n[j]=&m_nodes[cn];
				}			
			}
		}
	/* Faces		*/ 
	for( i=0,ni=m_faces.size();i<ni;++i)
		{
		Node**			n=	m_faces[i].m_n;
		if(	(ifn->Eval(n[0]->m_x)<accurary)&&
			(ifn->Eval(n[1]->m_x)<accurary)&&
			(ifn->Eval(n[2]->m_x)<accurary))
			{
			for(int j=0;j<3;++j)
				{
				int cn=cnodes[int(n[j]-nbase)];
				if(cn) n[j]=&m_nodes[cn];
				}
			}
		}
	/* Clean orphans	*/ 
	int							nnodes=m_nodes.size();
	btAlignedObjectArray<int>	ranks;
	btAlignedObjectArray<int>	todelete;
	ranks.resize(nnodes,0);
	for(i=0,ni=m_links.size();i<ni;++i)
		{
		for(int j=0;j<2;++j) ranks[int(m_links[i].m_n[j]-nbase)]++;
		}
	for(i=0,ni=m_faces.size();i<ni;++i)
		{
		for(int j=0;j<3;++j) ranks[int(m_faces[i].m_n[j]-nbase)]++;
		}
	for(i=0;i<m_links.size();++i)
		{
		const int	id[]={	int(m_links[i].m_n[0]-nbase),
							int(m_links[i].m_n[1]-nbase)};
		const bool	sg[]={	ranks[id[0]]==1,
							ranks[id[1]]==1};
		if(sg[0]||sg[1])
			{
			--ranks[id[0]];
			--ranks[id[1]];
			btSwap(m_links[i],m_links[m_links.size()-1]);
			m_links.pop_back();--i;
			}
		}
	#if 0	
	for(i=nnodes-1;i>=0;--i)
		{
		if(!ranks[i]) todelete.push_back(i);
		}	
	if(todelete.size())
		{		
		btAlignedObjectArray<int>&	map=ranks;
		for(int i=0;i<nnodes;++i) map[i]=i;
		PointersToIndices(this);
		for(int i=0,ni=todelete.size();i<ni;++i)
			{
			int		j=todelete[i];
			int&	a=map[j];
			int&	b=map[--nnodes];
			m_ndbvt.remove(m_nodes[a].m_leaf);m_nodes[a].m_leaf=0;
			btSwap(m_nodes[a],m_nodes[b]);
			j=a;a=b;b=j;			
			}
		IndicesToPointers(this,&map[0]);
		m_nodes.resize(nnodes);
		}
	#endif
	}
m_bUpdateRtCst=true;
}

static inline int		MatchEdge(	const btSoftBody::Node* a,
									const btSoftBody::Node* b,
									const btSoftBody::Node* ma,
									const btSoftBody::Node* mb)
{
if((a==ma)&&(b==mb)) return(0);
if((a==mb)&&(b==ma)) return(1);
return(-1);
}

//
bool			btSoftBody::cutLink(const Node* node0,const Node* node1,btScalar position)
{
return(cutLink(int(node0-&m_nodes[0]),int(node1-&m_nodes[0]),position));
}

//
bool			btSoftBody::cutLink(int node0,int node1,btScalar position)
{
bool			done=false;
const btVector3	d=m_nodes[node0].m_x-m_nodes[node1].m_x;
const btVector3	x=Lerp(m_nodes[node0].m_x,m_nodes[node1].m_x,position);
const btVector3	v=Lerp(m_nodes[node0].m_v,m_nodes[node1].m_v,position);
const btScalar	m=1;
appendNode(x,m);
appendNode(x,m);
Node*			pa=&m_nodes[node0];
Node*			pb=&m_nodes[node1];
Node*			pn[2]={	&m_nodes[m_nodes.size()-2],
						&m_nodes[m_nodes.size()-1]};
pn[0]->m_v=v;
pn[1]->m_v=v;
int i,ni;

for(i=0,ni=m_links.size();i<ni;++i)
	{
	const int mtch=MatchEdge(m_links[i].m_n[0],m_links[i].m_n[1],pa,pb);
	if(mtch!=-1)
		{
		appendLink(i);
		Link*	pft[]={&m_links[i],&m_links[m_links.size()-1]};
		pft[0]->m_n[1]=pn[mtch];
		pft[1]->m_n[0]=pn[1-mtch];
		done=true;
		}
	}
for(i=0,ni=m_faces.size();i<ni;++i)
	{
	for(int k=2,l=0;l<3;k=l++)
		{
		const int mtch=MatchEdge(m_faces[i].m_n[k],m_faces[i].m_n[l],pa,pb);
		if(mtch!=-1)
			{
			appendFace(i);
			Face*	pft[]={&m_faces[i],&m_faces[m_faces.size()-1]};
			pft[0]->m_n[l]=pn[mtch];
			pft[1]->m_n[k]=pn[1-mtch];
			appendLink(pn[0],pft[0]->m_n[(l+1)%3],pft[0]->m_material,true);
			appendLink(pn[1],pft[0]->m_n[(l+1)%3],pft[0]->m_material,true);
			}
		}
	}
if(!done)
	{
	m_ndbvt.remove(pn[0]->m_leaf);
	m_ndbvt.remove(pn[1]->m_leaf);
	m_nodes.pop_back();
	m_nodes.pop_back();
	}
return(done);
}

//
bool			btSoftBody::rayCast(const btVector3& org,
									const btVector3& dir,
									sRayCast& results,
									btScalar maxtime)
{
if(m_faces.size()&&m_fdbvt.empty()) InitializeFaceTree(this);
results.body	=	this;
results.time	=	maxtime;
results.feature	=	eFeature::None;
results.index	=	-1;
return(RaycastInternal(	this,org,dir,
						results.time,
						results.feature,
						results.index,false)!=0);
}

//
void			btSoftBody::setSolver(eSolverPresets::_ preset)
{
m_cfg.m_vsequence.clear();
m_cfg.m_psequence.clear();
m_cfg.m_dsequence.clear();
switch(preset)
	{
	case	eSolverPresets::Positions:
	m_cfg.m_psequence.push_back(ePSolver::Anchors);
	m_cfg.m_psequence.push_back(ePSolver::RContacts);
	m_cfg.m_psequence.push_back(ePSolver::SContacts);
	m_cfg.m_psequence.push_back(ePSolver::Linear);	
	break;	
	case	eSolverPresets::Velocities:
	m_cfg.m_vsequence.push_back(eVSolver::Linear);
	
	m_cfg.m_psequence.push_back(ePSolver::Anchors);
	m_cfg.m_psequence.push_back(ePSolver::RContacts);
	m_cfg.m_psequence.push_back(ePSolver::SContacts);
	
	m_cfg.m_dsequence.push_back(ePSolver::Linear);
	break;
	}
}

//
void			btSoftBody::predictMotion(btScalar dt)
{
	/* Update				*/ 
	if(m_bUpdateRtCst)
	{
		m_bUpdateRtCst=false;
		UpdateConstants(this);
		m_fdbvt.clear();
		if(m_cfg.collisions&fCollision::VF_SS)
			{
			InitializeFaceTree(this);			
			}
	}
		
	/* Prepare				*/ 
	m_sst.sdt		=	dt*m_cfg.timescale;
	m_sst.isdt		=	1/m_sst.sdt;
	m_sst.velmrg	=	m_sst.sdt*3;
	m_sst.radmrg	=	getCollisionShape()->getMargin();
	m_sst.updmrg	=	m_sst.radmrg*(btScalar)0.25;
	/* Forces				*/ 
	addVelocity(m_worldInfo->m_gravity*m_sst.sdt);
	ApplyForces(this,m_sst.sdt);	
	/* Integrate			*/ 
	for(int i=0,ni=m_nodes.size();i<ni;++i)
	{
		Node&	n=m_nodes[i];
		n.m_q	=	n.m_x;
		n.m_v	+=	n.m_f*n.m_im*m_sst.sdt;
		n.m_x	+=	n.m_v*m_sst.sdt;
		m_ndbvt.update(	n.m_leaf,
						btDbvtVolume::FromCR(n.m_x,m_sst.radmrg),
						n.m_v*m_sst.velmrg,
						m_sst.updmrg);
	}
	UpdateBounds(this);	
	/* Faces				*/ 
	if(!m_fdbvt.empty())
		{
		for(int i=0;i<m_faces.size();++i)
			{
			Face&			f=m_faces[i];
			const btVector3	v=(	f.m_n[0]->m_v+
								f.m_n[1]->m_v+
								f.m_n[2]->m_v)/3;
			m_fdbvt.update(	f.m_leaf,
							VolumeOf(f,m_sst.radmrg),
							v*m_sst.velmrg,
							m_sst.updmrg);
			}
		}
	/* Pose					*/ 
	UpdatePose(this);
	/* Match				*/ 
	if(m_pose.m_bframe&&(m_cfg.kMT>0))
		{
		for(int i=0,ni=m_nodes.size();i<ni;++i)
			{
			Node&	n=m_nodes[i];
			if(n.m_im>0)
				{
				const btVector3	x=m_pose.m_rot*m_pose.m_pos[i]+m_pose.m_com;
				n.m_x=Lerp(n.m_x,x,m_cfg.kMT);
				}
			}
		}
	/* Clear contacts		*/ 
	m_rcontacts.resize(0);
	m_scontacts.resize(0);
	/* Optimize dbvt's		*/ 
	m_ndbvt.optimizeIncremental(1);
	m_fdbvt.optimizeIncremental(1);
}

//
void			btSoftBody::solveConstraints()
{
	int i,ni;

/* Prepare links		*/ 
for(i=0,ni=m_links.size();i<ni;++i)
	{
	Link&	l=m_links[i];
	l.m_c3		=	l.m_n[1]->m_x-l.m_n[0]->m_x;
	l.m_c2		=	1/(l.m_c3.length2()*l.m_c0);
	}
/* Prepare anchors		*/ 
for(i=0,ni=m_anchors.size();i<ni;++i)
	{
	Anchor&			a=m_anchors[i];
	const btVector3	ra=a.m_body->getWorldTransform().getBasis()*a.m_local;
	a.m_c0	=	ImpulseMatrix(	m_sst.sdt,
								a.m_node->m_im,
								a.m_body->getInvMass(),
								a.m_body->getInvInertiaTensorWorld(),
								ra);
	a.m_c1	=	ra;
	a.m_c2	=	m_sst.sdt*a.m_node->m_im;
	a.m_body->activate();
	}
/* Solve velocities		*/ 
if(m_cfg.viterations>0)
	{	
	/* Solve			*/ 
	for(int isolve=0;isolve<m_cfg.viterations;++isolve)
		{
		for(int iseq=0;iseq<m_cfg.m_vsequence.size();++iseq)
			{
			VSolvers[m_cfg.m_vsequence[iseq]](this,1);
			}
		}
	/* Update			*/ 
	for(int i=0,ni=m_nodes.size();i<ni;++i)
		{
		Node&	n=m_nodes[i];
		n.m_x	=	n.m_q+n.m_v*m_sst.sdt;
		}
	}
/* Solve positions		*/ 
if(m_cfg.piterations>0)
	{
	for(int isolve=0;isolve<m_cfg.piterations;++isolve)
		{
		for(int iseq=0;iseq<m_cfg.m_psequence.size();++iseq)
			{
			PSolvers[m_cfg.m_psequence[iseq]](this,1);
			}
		}
	const btScalar	vc=m_sst.isdt*(1-m_cfg.kDP);
	for(int i=0,ni=m_nodes.size();i<ni;++i)
		{
		Node&	n=m_nodes[i];
		n.m_v	=	(n.m_x-n.m_q)*vc;
		n.m_f	=	btVector3(0,0,0);		
		}
	}
/* Solve drift			*/ 
if(m_cfg.diterations>0)
	{
	const btScalar	vcf=m_cfg.kVCF*m_sst.isdt;
	for(i=0,ni=m_nodes.size();i<ni;++i)
		{
		Node&	n=m_nodes[i];
		n.m_q	=	n.m_x;
		}
	for(int idrift=0;idrift<m_cfg.diterations;++idrift)
		{
		for(int iseq=0;iseq<m_cfg.m_dsequence.size();++iseq)
			{
			PSolvers[m_cfg.m_dsequence[iseq]](this,1);
			}
		}
	for(i=0,ni=m_nodes.size();i<ni;++i)
		{
		Node&	n=m_nodes[i];
		n.m_v	+=	(n.m_x-n.m_q)*vcf;
		}
	}
}

//
void			btSoftBody::staticSolve(int iterations)
{
for(int isolve=0;isolve<iterations;++isolve)
	{
	for(int iseq=0;iseq<m_cfg.m_psequence.size();++iseq)
		{
		PSolvers[m_cfg.m_psequence[iseq]](this,1);
		}
	}
}

//
void			btSoftBody::solveCommonConstraints(btSoftBody** /*bodies*/,int /*count*/,int /*iterations*/)
{
/// placeholder
}

//
void			btSoftBody::integrateMotion()
{
	/* Update			*/ 
	UpdateNormals(this);
}

//
void			btSoftBody::defaultCollisionHandler(btCollisionObject* pco)
{
switch(m_cfg.collisions&fCollision::RVSmask)
	{
	case	fCollision::SDF_RS:
		{
		struct	DoCollide : btDbvt::ICollide
			{
			void		Process(const btDbvtNode* leaf)
				{
				btSoftBody::Node*	node=(btSoftBody::Node*)leaf->data;
				DoNode(*node);
				}
			void		DoNode(btSoftBody::Node& n) const
				{				
				const btScalar	m=n.m_im>0?dynmargin:stamargin;
				btSoftBody::RContact		c;
				if(	(!n.m_battach)&&
					CheckContact(psb,prb,n.m_x,m,c.m_cti))
					{
					const btScalar	ima=n.m_im;
					const btScalar	imb=prb->getInvMass();
					const btScalar	ms=ima+imb;
					if(ms>0)
						{
						const btTransform&	wtr=prb->getInterpolationWorldTransform();
						const btMatrix3x3	iwi=prb->getInvInertiaTensorWorld();
						const btVector3		ra=n.m_x-wtr.getOrigin();
						const btVector3		va=prb->getVelocityInLocalPoint(ra)*psb->m_sst.sdt;
						const btVector3		vb=n.m_x-n.m_q;	
						const btVector3		vr=vb-va;
						const btScalar		dn=dot(vr,c.m_cti.m_normal);
						const btVector3		fv=vr-c.m_cti.m_normal*dn;
						const btScalar		fc=psb->m_cfg.kDF*prb->getFriction();
						c.m_node	=	&n;
						c.m_c0		=	ImpulseMatrix(psb->m_sst.sdt,ima,imb,iwi,ra);
						c.m_c1		=	ra;
						c.m_c2		=	ima*psb->m_sst.sdt;
						c.m_c3		=	fv.length2()<(btFabs(dn)*fc)?0:1-fc;
						c.m_c4		=	prb->isStaticOrKinematicObject()?psb->m_cfg.kKHR:psb->m_cfg.kCHR;
						psb->m_rcontacts.push_back(c);
						prb->activate();
						}
					}
				}
			btSoftBody*		psb;
			btRigidBody*	prb;
			btScalar		dynmargin;
			btScalar		stamargin;
			}				docollide;
		btRigidBody*		prb=btRigidBody::upcast(pco);
		const btTransform	wtr=prb->getInterpolationWorldTransform();
		const btTransform	ctr=prb->getWorldTransform();
		const btScalar		timemargin=(wtr.getOrigin()-ctr.getOrigin()).length();
		const btScalar		basemargin=getCollisionShape()->getMargin();
		btVector3			mins;
		btVector3			maxs;
		btDbvtVolume		volume;
		pco->getCollisionShape()->getAabb(	pco->getInterpolationWorldTransform(),
											mins,
											maxs);
		volume=btDbvtVolume::FromMM(mins,maxs);
		volume.Expand(btVector3(basemargin,basemargin,basemargin));		
		docollide.psb		=	this;
		docollide.prb		=	prb;
		docollide.dynmargin	=	basemargin+timemargin;
		docollide.stamargin	=	basemargin;
		btDbvt::collideTV(m_ndbvt.m_root,volume,docollide);
		}
	break;
	}
}

//
void			btSoftBody::defaultCollisionHandler(btSoftBody* psb)
{
const int cf=m_cfg.collisions&psb->m_cfg.collisions;
switch(cf&fCollision::SVSmask)
	{
	case	fCollision::VF_SS:
		{
		struct	DoCollide : btDbvt::ICollide
			{
			void		Process(const btDbvtNode* lnode,
								const btDbvtNode* lface)
				{
				btSoftBody::Node*	node=(btSoftBody::Node*)lnode->data;
				btSoftBody::Face*	face=(btSoftBody::Face*)lface->data;
				btVector3	o=node->m_x;
				btVector3	p;
				btScalar	d=SIMD_INFINITY;
				ProjectOrigin(	face->m_n[0]->m_x-o,
								face->m_n[1]->m_x-o,
								face->m_n[2]->m_x-o,
								p,d);
				const btScalar	m=mrg+(o-node->m_q).length()*2;
				if(d<(m*m))
					{
					const btSoftBody::Node*		n[]={face->m_n[0],face->m_n[1],face->m_n[2]};
					const btVector3	w=BaryCoord(n[0]->m_x,n[1]->m_x,n[2]->m_x,p+o);
					const btScalar	ma=node->m_im;
					btScalar		mb=BaryEval(n[0]->m_im,n[1]->m_im,n[2]->m_im,w);
					if(	(n[0]->m_im<=0)||
						(n[1]->m_im<=0)||
						(n[2]->m_im<=0))
						{
						mb=0;
						}
					const btScalar	ms=ma+mb;
					if(ms>0)
						{
						btSoftBody::SContact		c;
						c.m_normal		=	p/-btSqrt(d);
						c.m_margin		=	m;
						c.m_node		=	node;
						c.m_face		=	face;
						c.m_weights		=	w;
						c.m_friction	=	btMax(psb[0]->m_cfg.kDF,psb[1]->m_cfg.kDF);
						c.m_cfm[0]		=	ma/ms*psb[0]->m_cfg.kSHR;
						c.m_cfm[1]		=	mb/ms*psb[1]->m_cfg.kSHR;
						psb[0]->m_scontacts.push_back(c);
						}
					}	
				}
			btSoftBody*		psb[2];
			btScalar		mrg;
			}	docollide;
		/* common					*/ 
		docollide.mrg=	getCollisionShape()->getMargin()+
						psb->getCollisionShape()->getMargin();
		/* psb0 nodes vs psb1 faces	*/ 
		docollide.psb[0]=this;
		docollide.psb[1]=psb;
		btDbvt::collideTT(	docollide.psb[0]->m_ndbvt.m_root,
							docollide.psb[1]->m_fdbvt.m_root,
							docollide);
		/* psb1 nodes vs psb0 faces	*/ 
		docollide.psb[0]=psb;
		docollide.psb[1]=this;
		btDbvt::collideTT(	docollide.psb[0]->m_ndbvt.m_root,
							docollide.psb[1]->m_fdbvt.m_root,
							docollide);
		}
	break;
	}
}