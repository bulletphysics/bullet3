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

#ifndef _312AEEF3_52DA_4ff6_B804_FCF937182E46_
#define _312AEEF3_52DA_4ff6_B804_FCF937182E46_

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btPoint3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btIDebugDraw.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

//
// btSoftBody
//
struct	btSoftBody
	{
	//
	// Enumerations
	//

	/* eLType			*/ 
	struct eLType { enum _ {
		Structural,	/* Master constraints							*/ 
		Bending,	/* Secondary constraints						*/ 
	};};

	/* eAeroModel		*/ 
	struct eAeroModel { enum _ {
		V_Point,	/* Vertex normals are oriented toward velocity	*/ 
		V_TwoSided,	/* Vertex normals are fliped to match velocity	*/ 
		V_OneSided,	/* Vertex normals are taken as it is			*/ 
		F_TwoSided,	/* Face normals are fliped to match velocity	*/ 
		F_OneSided,	/* Face normals are taken as it is				*/ 
	};};
	
	//
	// Interfaces
	//

	/* ISoftBody		*/ 
	struct	ISoftBody
		{
		struct	sCti
			{
			btRigidBody*	m_body;		/* Rigid body			*/ 
			btVector3		m_normal;	/* Outward normal		*/ 
			btScalar		m_offset;	/* Offset from origin	*/ 
			};
		struct	sMedium
			{
			btVector3		m_velocity;	/* Velocity				*/ 
			btScalar		m_pressure;	/* Pressure				*/ 
			btScalar		m_density;	/* Density				*/ 
			};
		virtual void		Attach(btSoftBody*)
			{}
		virtual void		Detach(btSoftBody*)
			{ delete this; }
		virtual void		StartCollide(	const btVector3& /*minbounds*/,
											const btVector3& /*maxbounds*/)
			{}
		virtual bool		CheckContact	(const btVector3& /*position*/,
											sCti& /*contact*/)
			{ return(false); }
		virtual void		EndCollide()
			{}
		virtual void		EvaluateMedium(	const btVector3& /*position*/,
											sMedium& medium)
			{	medium.m_velocity=btVector3(0,0,0);
				medium.m_pressure=0;
				medium.m_density=0; }
		};
	
	//
	// Internal types
	//
	
	typedef btAlignedObjectArray<btScalar>	tScalarArray;
	typedef btAlignedObjectArray<btVector3>	tVector3Array;
	
	/* Base type	*/ 
	struct	Element
		{
		void*			m_tag;			// User data
		};
	/* Node			*/ 
	struct	Node : Element
		{
		btVector3		m_x;			// Position
		btVector3		m_q;			// Previous step position
		btVector3		m_v;			// Velocity
		btVector3		m_f;			// Force accumulator
		btVector3		m_n;			// Normal
		btScalar		m_im;			// 1/mass
		btScalar		m_area;			// Area
		int				m_battach:1;	// Attached
		};
	/* Link			*/ 
	struct	Link : Element
		{
		Node*			m_n[2];			// Node pointers
		btScalar		m_rl;			// Rest length
		btScalar		m_kST;			// Stiffness coefficient
		btScalar		m_c0;			// (ima+imb)*kLST
		btScalar		m_c1;			// rl^2
		eLType::_		m_type;			// Link type
		};
	/* Face			*/ 
	struct	Face : Element
		{
		Node*			m_n[3];			// Node pointers
		btVector3		m_normal;		// Normal
		btScalar		m_ra;			// Rest area
		};
	/* Contact		*/ 
	struct	Contact
		{
		ISoftBody::sCti	m_cti;			// Contact infos
		Node*			m_node;			// Owner node
		btMatrix3x3		m_c0;			// Impulse matrix
		btVector3		m_c1;			// Relative anchor
		btScalar		m_c2;			// ima*dt
		btScalar		m_c3;			// Friction
		};
	/* Anchor		*/ 
	struct	Anchor
		{
		Node*			m_node;			// Node pointer
		btVector3		m_local;		// Anchor position in body space
		btRigidBody*	m_body;			// Body
		btMatrix3x3		m_c0;			// Impulse matrix
		btVector3		m_c1;			// Relative anchor
		btScalar		m_c2;			// ima*dt
		};
	/* Pose			*/ 
	struct	Pose
		{
		bool			m_bvolume;		// Is valid
		bool			m_bframe;		// Is frame
		btScalar		m_volume;		// Rest volume
		tVector3Array	m_pos;			// Reference positions
		tScalarArray	m_wgh;			// Weights
		btVector3		m_com;			// COM
		btMatrix3x3		m_trs;			// Transform
		};	
	/* Config		*/ 
	struct	Config
		{
		eAeroModel::_	aeromodel;		// Aerodynamic model (default: V_Point)
		btScalar		kLST;			// Linear stiffness coefficient [0,1]
		btScalar		kDP;			// Damping coefficient [0,1]
		btScalar		kDG;			// Drag coefficient [0,+inf]
		btScalar		kLF;			// Lift coefficient [0,+inf]
		btScalar		kPR;			// Pressure coefficient [-inf,+inf]
		btScalar		kVC;			// Volume conversation coefficient [0,+inf]
		btScalar		kDF;			// Dynamic friction coefficient [0,1]
		btScalar		kMT;			// Pose matching coefficient [0,1]		
		btScalar		kSOR;			// SOR(w) [1,2] default 1, never use with solver!=Accurate
		btScalar		kCHR;			// Contacts hardness [0,1]
		btScalar		kAHR;			// Anchors hardness [0,1]
		btScalar		timescale;		// Time scale
		btScalar		timestep;		// Time step
		int				maxsteps;		// Maximum time steps
		int				iterations;		// Solver iterations
		bool			becollide;		// Enable external collisions
		bool			bscollide;		// Enable self collisions
		};
	
	//
	// Typedef's
	//
	
	typedef btAlignedObjectArray<Node>		tNodeArray;
	typedef btAlignedObjectArray<Link>		tLinkArray;
	typedef btAlignedObjectArray<Face>		tFaceArray;
	typedef btAlignedObjectArray<Anchor>	tAnchorArray;
	typedef btAlignedObjectArray<Contact>	tContactArray;
	
	//
	// Fields
	//
	
	Config				m_cfg;			// Configuration
	Pose				m_pose;			// Pose
	ISoftBody*			m_isb;			// ISoftBody
	void*				m_tag;			// User data
	tNodeArray			m_nodes;		// Nodes
	tLinkArray			m_links;		// Links
	tFaceArray			m_faces;		// Faces
	tAnchorArray		m_anchors;		// Anchors
	tContactArray		m_contacts;		// Contacts
	btScalar			m_timeacc;		// Time accumulator
	btVector3			m_bounds[2];	// Spatial bounds	
	bool				m_bUpdateRtCst;	// Update runtime constants
	
	//
	// Api
	//

	/* Create a soft body													*/ 
	static btSoftBody*	Create(	ISoftBody* isoftbody,
								int node_count,
								const btVector3* x=0,
								const btScalar* m=0);
	/* Delete a body														*/ 
	void				Delete();
	/* Check for existing link												*/ 
	bool				CheckLink(	int node0,
									int node1) const;
	bool				CheckLink(	const btSoftBody::Node* node0,
									const btSoftBody::Node* node1) const;
	/* Check for existring face												*/ 
	bool				CheckFace(	int node0,
									int node1,
									int node2) const;
	/* Append link															*/ 
	void				AppendLink(		int node0,
										int node1,
										btScalar kST,
										btSoftBody::eLType::_ type,
										bool bcheckexist=false);
	void				AppendLink(		btSoftBody::Node* node0,
										btSoftBody::Node* node1,
										btScalar kST,
										btSoftBody::eLType::_ type,
										bool bcheckexist=false);
	/* Append face															*/ 
	void				AppendFace(		int node0,
										int node1,
										int node2);
	/* Append anchor														*/ 
	void				AppendAnchor(	int node,
										btRigidBody* body);
	/* Add force (or gravity) to the entire body							*/ 
	void				AddForce(		const btVector3& force);
	/* Add force (or gravity) to a node of the body							*/ 
	void				AddForce(		const btVector3& force,
										int node);
	/* Add velocity to the entire body										*/ 
	void				AddVelocity(	const btVector3& velocity);
	/* Add velocity to a node of the body									*/ 
	void				AddVelocity(	const btVector3& velocity,
										int node);
	/* Set mass																*/ 
	void				SetMass(		int node,
										btScalar mass);
	/* Get mass																*/ 
	btScalar			GetMass(		int node) const;
	/* Get total mass														*/ 
	btScalar			GetTotalMass() const;
	/* Set total mass (weighted by previous masses)							*/ 
	void				SetTotalMass(	btScalar mass,
										bool fromfaces=false);
	/* Set total density													*/ 
	void				SetTotalDensity(btScalar density);
	/* Transform															*/ 
	void				Transform(		const btTransform& trs);
	/* Scale																*/ 
	void				Scale(			const btVector3& scl);
	/* Set current state as pose											*/ 
	void				SetPose(		bool bvolume,
										bool bframe);
	/* Return the volume													*/ 
	btScalar			GetVolume() const;
	/* Generate bending constraints based on distance in the adjency graph	*/ 
	int					GenerateBendingConstraints(	int distance,
													btScalar stiffness);
	/* Randomize constraints to reduce solver bias							*/ 
	void				RandomizeConstraints();
	/* Ray casting															*/ 
	btScalar			Raycast(		const btVector3& org,
										const btVector3& dir) const;
	/* Step 																*/ 
	void				Step(			btScalar dt);
	
	};
								
//
// Helpers
//

/* fDrawFlags															*/ 
struct	fDrawFlags { enum _ {
	Nodes		=	0x0001,
	SLinks		=	0x0002,
	BLinks		=	0x0004,
	Faces		=	0x0008,
	Tetras		=	0x0010,
	Normals		=	0x0020,
	Contacts	=	0x0040,
	Anchors		=	0x0080,
	/* presets	*/ 
	Links		=	SLinks+BLinks,
	Std			=	SLinks+Faces+Anchors,
	StdTetra	=	Std-Faces+Tetras,
};};

/* Draw body															*/ 
void			Draw(			btSoftBody* psb,
								btIDebugDraw* idraw,
								int drawflags=fDrawFlags::Std);
/* Draw body infos														*/ 
void			DrawInfos(		btSoftBody* psb,
								btIDebugDraw* idraw,
								bool masses,
								bool areas,
								bool stress);
/* Draw rigid frame														*/ 
void			DrawFrame(		btSoftBody* psb,
								btIDebugDraw* idraw);
/* Create a rope														*/ 
btSoftBody*		CreateRope(		btSoftBody::ISoftBody*	isoftbody,
								const btVector3& from,
								const btVector3& to,
								int res,
								int fixeds);
/* Create a patch														*/ 
btSoftBody*		CreatePatch(	btSoftBody::ISoftBody*	isoftbody,
								const btVector3& corner00,
								const btVector3& corner10,
								const btVector3& corner01,
								const btVector3& corner11,
								int resx,
								int resy,
								int fixeds,
								bool gendiags);
/* Create an ellipsoid													*/ 
btSoftBody*		CreateEllipsoid(btSoftBody::ISoftBody*	isoftbody,
								const btVector3& center,
								const btVector3& radius,
								int res);
/* Create from convex-hull												*/ 
btSoftBody*		CreateFromConvexHull(	btSoftBody::ISoftBody*	isoftbody,
										const btVector3* vertices,
										int nvertices);											
/* Create from trimesh													*/ 
btSoftBody*		CreateFromTriMesh(		btSoftBody::ISoftBody*	isoftbody,
										const btScalar*	vertices,
										const int* triangles,
										int ntriangles);

#endif