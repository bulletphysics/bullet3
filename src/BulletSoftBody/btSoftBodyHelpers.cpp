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
///btSoftBodyHelpers.cpp by Nathanael Presson

#include "btSoftBody.h"
#include "btDbvt.h"
#include <stdio.h>
#include <string.h>
#include "btSoftBodyHelpers.h"
#include "LinearMath/btConvexHull.h"

//
static void				drawVertex(	btIDebugDraw* idraw,
									const btVector3& x,btScalar s,const btVector3& c)
	{
		idraw->drawLine(x-btVector3(s,0,0),x+btVector3(s,0,0),c);
		idraw->drawLine(x-btVector3(0,s,0),x+btVector3(0,s,0),c);
		idraw->drawLine(x-btVector3(0,0,s),x+btVector3(0,0,s),c);
	}

//
static void				drawBox(	btIDebugDraw* idraw,
									const btVector3& mins,
									const btVector3& maxs,
									const btVector3& color)
{
const btVector3	c[]={	btVector3(mins.x(),mins.y(),mins.z()),
						btVector3(maxs.x(),mins.y(),mins.z()),
						btVector3(maxs.x(),maxs.y(),mins.z()),
						btVector3(mins.x(),maxs.y(),mins.z()),
						btVector3(mins.x(),mins.y(),maxs.z()),
						btVector3(maxs.x(),mins.y(),maxs.z()),
						btVector3(maxs.x(),maxs.y(),maxs.z()),
						btVector3(mins.x(),maxs.y(),maxs.z())};
idraw->drawLine(c[0],c[1],color);idraw->drawLine(c[1],c[2],color);
idraw->drawLine(c[2],c[3],color);idraw->drawLine(c[3],c[0],color);
idraw->drawLine(c[4],c[5],color);idraw->drawLine(c[5],c[6],color);
idraw->drawLine(c[6],c[7],color);idraw->drawLine(c[7],c[4],color);
idraw->drawLine(c[0],c[4],color);idraw->drawLine(c[1],c[5],color);
idraw->drawLine(c[2],c[6],color);idraw->drawLine(c[3],c[7],color);
}

//
static void				drawTree(	btIDebugDraw* idraw,
									const btDbvt::Node* node,
									int depth,
									const btVector3& ncolor,
									const btVector3& lcolor,
									int mindepth,
									int maxdepth)
{
if(node)
	{
	if(node->isinternal()&&((depth<maxdepth)||(maxdepth<0)))
		{
		drawTree(idraw,node->childs[0],depth+1,ncolor,lcolor,mindepth,maxdepth);
		drawTree(idraw,node->childs[1],depth+1,ncolor,lcolor,mindepth,maxdepth);
		}
	if(depth>=mindepth)
		{
		const btScalar	scl=(btScalar)(node->isinternal()?1:1);
		const btVector3	mi=node->box.Center()-node->box.Extent()*scl;
		const btVector3	mx=node->box.Center()+node->box.Extent()*scl;
		drawBox(idraw,mi,mx,node->isleaf()?lcolor:ncolor);
		}
	}
}

//
#if 0
static btVector3		stresscolor(btScalar stress)
	{
	static const btVector3	spectrum[]=	{	btVector3(1,0,1),
											btVector3(0,0,1),
											btVector3(0,1,1),
											btVector3(0,1,0),
											btVector3(1,1,0),
											btVector3(1,0,0),
											btVector3(1,0,0)};
	static const int		ncolors=sizeof(spectrum)/sizeof(spectrum[0])-1;
	static const btScalar	one=1;
	stress=btMax<btScalar>(0,btMin<btScalar>(1,stress))*ncolors;
	const int				sel=(int)stress;
	const btScalar			frc=stress-sel;
	return(spectrum[sel]+(spectrum[sel+1]-spectrum[sel])*frc);
	}
#endif

//
void			btSoftBodyHelpers::Draw(	btSoftBody* psb,
					 btIDebugDraw* idraw,
					 int drawflags)
{
	const btScalar		scl=(btScalar)0.1;
	const btScalar		nscl=scl*5;
	const btVector3		scolor=btVector3(0,0,0);
	const btVector3		bcolor=btVector3(1,1,0);
	const btVector3		ncolor=btVector3(1,1,1);
	const btVector3		ccolor=btVector3(1,0,0);
	/* Nodes	*/ 
	if(0!=(drawflags&fDrawFlags::Nodes))
	{
		for(int i=0;i<psb->getNodes().size();++i)
		{
			const btSoftBody::Node&	n=psb->getNodes()[i];		
			idraw->drawLine(n.m_x-btVector3(scl,0,0),n.m_x+btVector3(scl,0,0),btVector3(1,0,0));
			idraw->drawLine(n.m_x-btVector3(0,scl,0),n.m_x+btVector3(0,scl,0),btVector3(0,1,0));
			idraw->drawLine(n.m_x-btVector3(0,0,scl),n.m_x+btVector3(0,0,scl),btVector3(0,0,1));
		}
	}
	/* Links	*/ 
	if(0!=(drawflags&fDrawFlags::Links))
	{
		for(int i=0;i<psb->getLinks().size();++i)
		{
			const btSoftBody::Link&	l=psb->getLinks()[i];
			switch(l.m_type)
			{
			case	btSoftBody::eLType::Structural:
				if(0!=(drawflags&fDrawFlags::SLinks)) idraw->drawLine(l.m_n[0]->m_x,l.m_n[1]->m_x,scolor);break;
			case	btSoftBody::eLType::Bending:
				if(0!=(drawflags&fDrawFlags::BLinks)) idraw->drawLine(l.m_n[0]->m_x,l.m_n[1]->m_x,bcolor);break;
			}
		}
	}
	/* Normals	*/ 
	if(0!=(drawflags&fDrawFlags::Normals))
	{
		for(int i=0;i<psb->getNodes().size();++i)
		{
			const btSoftBody::Node&	n=psb->getNodes()[i];
			const btVector3			d=n.m_n*nscl;
			idraw->drawLine(n.m_x,n.m_x+d,ncolor);
			idraw->drawLine(n.m_x,n.m_x-d,ncolor*0.5);
		}
	}
	/* Contacts	*/ 
	if(0!=(drawflags&fDrawFlags::Contacts))
	{
		static const btVector3		axis[]={btVector3(1,0,0),
			btVector3(0,1,0),
			btVector3(0,0,1)};
		for(int i=0;i<psb->m_rcontacts.size();++i)
		{		
			const btSoftBody::RContact&	c=psb->m_rcontacts[i];
			const btVector3				o=	c.m_node->m_x-c.m_cti.m_normal*
				(dot(c.m_node->m_x,c.m_cti.m_normal)+c.m_cti.m_offset);
			const btVector3				x=cross(c.m_cti.m_normal,axis[c.m_cti.m_normal.minAxis()]).normalized();
			const btVector3				y=cross(x,c.m_cti.m_normal).normalized();
			idraw->drawLine(o-x*nscl,o+x*nscl,ccolor);
			idraw->drawLine(o-y*nscl,o+y*nscl,ccolor);
			idraw->drawLine(o,o+c.m_cti.m_normal*nscl*3,btVector3(1,1,0));
		}
	}
	/* Anchors	*/ 
	if(0!=(drawflags&fDrawFlags::Anchors))
	{
		for(int i=0;i<psb->m_anchors.size();++i)
		{
			const btSoftBody::Anchor&	a=psb->m_anchors[i];
			const btVector3				q=a.m_body->getWorldTransform()*a.m_local;
			drawVertex(idraw,a.m_node->m_x,0.25,btVector3(1,0,0));
			drawVertex(idraw,q,0.25,btVector3(0,1,0));
			idraw->drawLine(a.m_node->m_x,q,btVector3(1,1,1));
		}
		for(int i=0;i<psb->getNodes().size();++i)
		{
			const btSoftBody::Node&	n=psb->getNodes()[i];		
			if(n.m_im<=0)
			{
				drawVertex(idraw,n.m_x,0.25,btVector3(1,0,0));
			}
		}
	}
	/* Faces	*/ 
	if(0!=(drawflags&fDrawFlags::Faces))
	{
		const btScalar	scl=(btScalar)0.7;
		const btScalar	alp=(btScalar)1;
		const btVector3	col(0,(btScalar)0.7,0);
		for(int i=0;i<psb->getFaces().size();++i)
		{
			const btSoftBody::Face&	f=psb->getFaces()[i];
			const btVector3			x[]={f.m_n[0]->m_x,f.m_n[1]->m_x,f.m_n[2]->m_x};
			const btVector3			c=(x[0]+x[1]+x[2])/3;
			idraw->drawTriangle((x[0]-c)*scl+c,
				(x[1]-c)*scl+c,
				(x[2]-c)*scl+c,
				f.m_n[0]->m_n,f.m_n[1]->m_n,f.m_n[2]->m_n,
				col,alp);
		}	
	}
}

//
void			btSoftBodyHelpers::DrawInfos(		btSoftBody* psb,
						  btIDebugDraw* idraw,
						  bool masses,
						  bool areas,
						  bool /*stress*/)
{
	for(int i=0;i<psb->getNodes().size();++i)
	{
		const btSoftBody::Node&	n=psb->getNodes()[i];
		char					text[2048]={0};
		char					buff[1024];
		if(masses)
		{
			sprintf(buff," M(%.2f)",1/n.m_im);
			strcat(text,buff);
		}
		if(areas)
		{
			sprintf(buff," A(%.2f)",n.m_area);
			strcat(text,buff);
		}
		if(text[0]) idraw->draw3dText(n.m_x,text);
	}
}

//
void			btSoftBodyHelpers::DrawNodeTree(	btSoftBody* psb,
													btIDebugDraw* idraw,
													int mindepth,
													int maxdepth)
{
drawTree(idraw,psb->m_ndbvt.m_root,0,btVector3(1,0,1),btVector3(1,1,1),mindepth,maxdepth);
}

//
void			btSoftBodyHelpers::DrawFaceTree(	btSoftBody* psb,
													btIDebugDraw* idraw,
													int mindepth,
													int maxdepth)
{
drawTree(idraw,psb->m_fdbvt.m_root,0,btVector3(0,1,0),btVector3(1,0,0),mindepth,maxdepth);
}

//
void			btSoftBodyHelpers::DrawFrame(		btSoftBody* psb,
						  btIDebugDraw* idraw)
{
	if(psb->m_pose.m_bframe)
	{
		static const btScalar	ascl=10;
		static const btScalar	nscl=(btScalar)0.1;
		const btVector3			com=psb->m_pose.m_com;
		const btMatrix3x3		trs=psb->m_pose.m_rot*psb->m_pose.m_scl;
		const btVector3			Xaxis=(trs*btVector3(1,0,0)).normalized();
		const btVector3			Yaxis=(trs*btVector3(0,1,0)).normalized();
		const btVector3			Zaxis=(trs*btVector3(0,0,1)).normalized();
		idraw->drawLine(com,com+Xaxis*ascl,btVector3(1,0,0));
		idraw->drawLine(com,com+Yaxis*ascl,btVector3(0,1,0));
		idraw->drawLine(com,com+Zaxis*ascl,btVector3(0,0,1));
		for(int i=0;i<psb->m_pose.m_pos.size();++i)
		{
			const btVector3	x=com+trs*psb->m_pose.m_pos[i];
			drawVertex(idraw,x,nscl,btVector3(1,0,1));
		}
		for(int i=0;i<psb->m_dfld.pts.size();++i)
			{
			const btVector3	x=com+trs*psb->m_dfld.pts[i];
			drawVertex(idraw,x,nscl*(btScalar)0.5,btVector3(0,0,1));
			}
	}
}

//
btSoftBody*		btSoftBodyHelpers::CreateRope(	btSoftBody::btSoftBodyWorldInfo& worldInfo, const btVector3& from,
						   const btVector3& to,
						   int res,
						   int fixeds)
{
	/* Create nodes	*/ 
	const int		r=res+2;
	btVector3*		x=new btVector3[r];
	btScalar*		m=new btScalar[r];
	for(int i=0;i<r;++i)
	{
		const btScalar	t=i/(btScalar)(r-1);
		x[i]=lerp(from,to,t);
		m[i]=1;
	}
	btSoftBody*		psb= new btSoftBody(&worldInfo,r,x,m);
	if(fixeds&1) psb->setMass(0,0);
	if(fixeds&2) psb->setMass(r-1,0);
	delete[] x;
	delete[] m;
	/* Create links	*/ 
	for(int i=1;i<r;++i)
	{
		psb->appendLink(i-1,i,1,btSoftBody::eLType::Structural);
	}
	/* Finished		*/ 
	return(psb);
}

//
btSoftBody*		btSoftBodyHelpers::CreatePatch(btSoftBody::btSoftBodyWorldInfo& worldInfo,const btVector3& corner00,
							const btVector3& corner10,
							const btVector3& corner01,
							const btVector3& corner11,
							int resx,
							int resy,
							int fixeds,
							bool gendiags)
{
#define IDX(_x_,_y_)	((_y_)*rx+(_x_))
	/* Create nodes	*/ 
	if((resx<2)||(resy<2)) return(0);
	const int	rx=resx;
	const int	ry=resy;
	const int	tot=rx*ry;
	btVector3*	x=new btVector3[tot];
	btScalar*	m=new btScalar[tot];
	for(int iy=0;iy<ry;++iy)
	{
		const btScalar	ty=iy/(btScalar)(ry-1);
		const btVector3	py0=lerp(corner00,corner01,ty);
		const btVector3	py1=lerp(corner10,corner11,ty);
		for(int ix=0;ix<rx;++ix)
		{
			const btScalar	tx=ix/(btScalar)(rx-1);
			x[IDX(ix,iy)]=lerp(py0,py1,tx);
			m[IDX(ix,iy)]=1;
		}
	}
	btSoftBody*		psb=new btSoftBody(&worldInfo,tot,x,m);
	if(fixeds&1)	psb->setMass(IDX(0,0),0);
	if(fixeds&2)	psb->setMass(IDX(rx-1,0),0);
	if(fixeds&4)	psb->setMass(IDX(0,ry-1),0);
	if(fixeds&8)	psb->setMass(IDX(rx-1,ry-1),0);
	delete[] x;
	delete[] m;
	/* Create links	and faces */ 
	for(int iy=0;iy<ry;++iy)
	{
		for(int ix=0;ix<rx;++ix)
		{
			const int	idx=IDX(ix,iy);
			const bool	mdx=(ix+1)<rx;
			const bool	mdy=(iy+1)<ry;
			if(mdx) psb->appendLink(idx,IDX(ix+1,iy),
				1,btSoftBody::eLType::Structural);
			if(mdy) psb->appendLink(idx,IDX(ix,iy+1),
				1,btSoftBody::eLType::Structural);			
			if(mdx&&mdy)
			{
				if((ix+iy)&1)
				{
					psb->appendFace(IDX(ix,iy),IDX(ix+1,iy),IDX(ix+1,iy+1));
					psb->appendFace(IDX(ix,iy),IDX(ix+1,iy+1),IDX(ix,iy+1));
					if(gendiags)
					{
						psb->appendLink(IDX(ix,iy),IDX(ix+1,iy+1),
							1,btSoftBody::eLType::Structural);
					}
				}
				else
				{
					psb->appendFace(IDX(ix,iy+1),IDX(ix,iy),IDX(ix+1,iy));
					psb->appendFace(IDX(ix,iy+1),IDX(ix+1,iy),IDX(ix+1,iy+1));
					if(gendiags)
					{
						psb->appendLink(IDX(ix+1,iy),IDX(ix,iy+1),
							1,btSoftBody::eLType::Structural);
					}
				}
			}
		}
	}
	/* Finished		*/ 
#undef IDX
	return(psb);
}

//
btSoftBody*		btSoftBodyHelpers::CreateEllipsoid(btSoftBody::btSoftBodyWorldInfo& worldInfo,const btVector3& center,
								const btVector3& radius,
								int res)
{
	struct	Hammersley
	{
		static void	Generate(btVector3* x,int n)
		{
			for(int i=0;i<n;i++)
			{
				btScalar	p=0.5,t=0;
				for(int j=i;j;p*=0.5,j>>=1) if(j&1) t+=p;
				btScalar	w=2*t-1;
				btScalar	a=(SIMD_PI+2*i*SIMD_PI)/n;
				btScalar	s=btSqrt(1-w*w);
				*x++=btVector3(s*btCos(a),s*btSin(a),w);
			}
		}
	};
	btAlignedObjectArray<btVector3>	vtx;
	vtx.resize(3+res);
	Hammersley::Generate(&vtx[0],vtx.size());
	for(int i=0;i<vtx.size();++i)
	{
		vtx[i]=vtx[i]*radius+center;
	}
	return(CreateFromConvexHull(worldInfo,&vtx[0],vtx.size()));
}



//
btSoftBody*		btSoftBodyHelpers::CreateFromTriMesh(btSoftBody::btSoftBodyWorldInfo& worldInfo,const btScalar*	vertices,
								  const int* triangles,
								  int ntriangles)
{
	int		maxidx=0;
	for(int i=0,ni=ntriangles*3;i<ni;++i)
	{
		maxidx=btMax(triangles[i],maxidx);
	}
	++maxidx;
	btAlignedObjectArray<bool>		chks;
	btAlignedObjectArray<btVector3>	vtx;
	chks.resize(maxidx*maxidx,false);
	vtx.resize(maxidx);
	for(int i=0,j=0,ni=maxidx*3;i<ni;++j,i+=3)
	{
		vtx[j]=btVector3(vertices[i],vertices[i+1],vertices[i+2]);
	}
	btSoftBody*		psb=new btSoftBody(&worldInfo,vtx.size(),&vtx[0],0);
	for(int i=0,ni=ntriangles*3;i<ni;i+=3)
	{
		const int idx[]={triangles[i],triangles[i+1],triangles[i+2]};
#define IDX(_x_,_y_) ((_y_)*maxidx+(_x_))
		for(int j=2,k=0;k<3;j=k++)
		{
			if(!chks[IDX(idx[j],idx[k])])
			{
				chks[IDX(idx[j],idx[k])]=true;
				chks[IDX(idx[k],idx[k])]=true;
				psb->appendLink(idx[j],idx[k],1,btSoftBody::eLType::Structural);
			}
		}
#undef IDX
		psb->appendFace(idx[0],idx[1],idx[2]);
	}
	psb->randomizeConstraints();
	return(psb);
}

//
btSoftBody*		btSoftBodyHelpers::CreateFromConvexHull(btSoftBody::btSoftBodyWorldInfo& worldInfo,	const btVector3* vertices,
									 int nvertices)
{
	HullDesc		hdsc(QF_TRIANGLES,nvertices,vertices);
	HullResult		hres;
	HullLibrary		hlib;/*??*/ 
	hdsc.mMaxVertices=nvertices;
	hlib.CreateConvexHull(hdsc,hres);
	btSoftBody*		psb=new btSoftBody(&worldInfo,(int)hres.mNumOutputVertices,
		&hres.m_OutputVertices[0],0);
	for(int i=0;i<(int)hres.mNumFaces;++i)
	{
		const int idx[]={	hres.m_Indices[i*3+0],
			hres.m_Indices[i*3+1],
			hres.m_Indices[i*3+2]};
		if(idx[0]<idx[1]) psb->appendLink(	idx[0],idx[1],
			1,btSoftBody::eLType::Structural);
		if(idx[1]<idx[2]) psb->appendLink(	idx[1],idx[2],
			1,btSoftBody::eLType::Structural);
		if(idx[2]<idx[0]) psb->appendLink(	idx[2],idx[0],
			1,btSoftBody::eLType::Structural);
		psb->appendFace(idx[0],idx[1],idx[2]);
	}
	hlib.ReleaseResult(hres);
	psb->randomizeConstraints();
	return(psb);
}


