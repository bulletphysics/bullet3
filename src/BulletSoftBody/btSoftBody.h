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

#ifndef _BT_SOFT_BODY_H
#define _BT_SOFT_BODY_H

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btPoint3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btIDebugDraw.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

#include "BulletCollision/CollisionShapes/btConcaveShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
#include "btSparseSDF.h"
#include "btDbvt.h"

class btBroadphaseInterface;
class btCollisionDispatcher;

/// btSoftBody is work-in-progress
class	btSoftBody : public btCollisionObject
{
public:
	//
	// Enumerations
	//

	///eLType
	struct eLType { enum _ {
		Structural,	///Master constraints 
		Bending,	///Secondary constraints
	};};

	///eAeroModel 
	struct eAeroModel { enum _ {
		V_Point,	///Vertex normals are oriented toward velocity
		V_TwoSided,	///Vertex normals are fliped to match velocity	
		V_OneSided,	///Vertex normals are taken as it is	
		F_TwoSided,	///Face normals are fliped to match velocity
		F_OneSided,	///Face normals are taken as it is
	};};
	
	//
	// Flags
	//
	
	///fCollision
	struct fCollision { enum _ {
		RVSmask	=	0x000f,	///Rigid versus soft mask
		SDF_RS	=	0x0001,	///SDF base rigid vs soft
		
		SVSmask	=	0x00f0,	///Rigid versus soft mask		
		VF_SS	=	0x0010,	///Vertex vs face soft vs soft handling
		/* presets	*/ 
		Default	=	SDF_RS,
	};};
	
	//
	// API Types
	//
	
	/* sRayCast		*/ 
	struct sRayCast
	{
		int			face;	/// face
		btScalar	time;	/// time of impact (rayorg+raydir*time)
	};
	
	//
	// Internal types
	//

	typedef btAlignedObjectArray<btScalar>	tScalarArray;
	typedef btAlignedObjectArray<btVector3>	tVector3Array;

	/* btSoftBodyWorldInfo	*/ 
	struct	btSoftBodyWorldInfo
	{
		btScalar				air_density;
		btScalar				water_density;
		btScalar				water_offset;
		btVector3				water_normal;
		btBroadphaseInterface*	m_broadphase;
		btCollisionDispatcher*	m_dispatcher;
		btVector3				m_gravity;
		btSparseSdf<3>			m_sparsesdf;
	};	

	/* sCti is Softbody contact info	*/ 
	struct	sCti
	{
		btRigidBody*	m_body;		/* Rigid body			*/ 
		btVector3		m_normal;	/* Outward normal		*/ 
		btScalar		m_offset;	/* Offset from origin	*/ 
	};	

	/* sMedium		*/ 
	struct	sMedium
	{
		btVector3		m_velocity;	/* Velocity				*/ 
		btScalar		m_pressure;	/* Pressure				*/ 
		btScalar		m_density;	/* Density				*/ 
	};

	/* Base type	*/ 
	struct	Element
	{
		void*			m_tag;			// User data
	};
	/* Node			*/ 
	struct	Node : Element
	{
		btVector3				m_x;			// Position
		btVector3				m_q;			// Previous step position
		btVector3				m_v;			// Velocity
		btVector3				m_f;			// Force accumulator
		btVector3				m_n;			// Normal
		btScalar				m_im;			// 1/mass
		btScalar				m_area;			// Area
		btDbvt::Node*			m_leaf;			// Leaf data
		int						m_battach:1;	// Attached
	};
	/* Link			*/ 
	struct	Link : Element
	{
		Node*					m_n[2];			// Node pointers
		btScalar				m_rl;			// Rest length
		btScalar				m_kST;			// Stiffness coefficient
		btScalar				m_c0;			// (ima+imb)*kLST
		btScalar				m_c1;			// rl^2
		btSoftBody::eLType::_	m_type;			// Link type
	};
	/* Face			*/ 
	struct	Face : Element
	{
		Node*					m_n[3];			// Node pointers
		btVector3				m_normal;		// Normal
		btScalar				m_ra;			// Rest area
		btDbvt::Node*			m_leaf;			// Leaf data
	};
	/* RContact		*/ 
	struct	RContact
	{
		btSoftBody::sCti		m_cti;			// Contact infos
		Node*					m_node;			// Owner node
		btMatrix3x3				m_c0;			// Impulse matrix
		btVector3				m_c1;			// Relative anchor
		btScalar				m_c2;			// ima*dt
		btScalar				m_c3;			// Friction
		btScalar				m_c4;			// Hardness
	};
	/* SContact		*/ 
	struct	SContact
	{
		Node*					m_node;			// Node
		Face*					m_face;			// Face
		btVector3				m_weights;		// Weigths
		btVector3				m_normal;		// Normal
		btScalar				m_margin;		// Margin
		btScalar				m_friction;		// Friction
		btScalar				m_cfm[2];		// Constraint force mixing
	};
	/* Anchor		*/ 
	struct	Anchor
	{
		Node*					m_node;			// Node pointer
		btVector3				m_local;		// Anchor position in body space
		btRigidBody*			m_body;			// Body
		btMatrix3x3				m_c0;			// Impulse matrix
		btVector3				m_c1;			// Relative anchor
		btScalar				m_c2;			// ima*dt
	};
	/* Pose			*/ 
	struct	Pose
	{
		bool					m_bvolume;		// Is valid
		bool					m_bframe;		// Is frame
		btScalar				m_volume;		// Rest volume
		tVector3Array			m_pos;			// Reference positions
		tScalarArray			m_wgh;			// Weights
		btVector3				m_com;			// COM
		btMatrix3x3				m_rot;			// Rotation
		btMatrix3x3				m_scl;			// Scale
		btMatrix3x3				m_aqq;			// Base scaling
	};
	/* DFld			*/ 
	struct	DFld
	{
		btAlignedObjectArray<btVector3>	pts;
	};
	/* Config		*/ 
	struct	Config
	{
		btSoftBody::eAeroModel::_	aeromodel;		// Aerodynamic model (default: V_Point)
		btScalar				kLST;			// Linear stiffness coefficient [0,1]
		btScalar				kDP;			// Damping coefficient [0,1]
		btScalar				kDG;			// Drag coefficient [0,+inf]
		btScalar				kLF;			// Lift coefficient [0,+inf]
		btScalar				kPR;			// Pressure coefficient [-inf,+inf]
		btScalar				kVC;			// Volume conversation coefficient [0,+inf]
		btScalar				kDF;			// Dynamic friction coefficient [0,1]
		btScalar				kMT;			// Pose matching coefficient [0,1]		
		btScalar				kSOR;			// SOR(w) [1,2] default 1, never use with solver!=Accurate
		btScalar				kCHR;			// Rigid contacts hardness [0,1]
		btScalar				kSHR;			// Soft contacts hardness [0,1]
		btScalar				kAHR;			// Anchors hardness [0,1]
		btScalar				maxvolume;		// Maximum volume ratio for pose
		btScalar				timescale;		// Time scale
		int						iterations;		// Solver iterations
		int						collisions;		// Collisions flags
	};
	/* SolverState	*/ 
	struct	SolverState
	{
		btScalar				iit;			// 1/iterations
		btScalar				sdt;			// dt*timescale
		btScalar				isdt;			// 1/sdt
		btScalar				velmrg;			// velocity margin
		btScalar				radmrg;			// radial margin
		btScalar				updmrg;			// Update margin
	};

	//
	// Typedef's
	//

	typedef btAlignedObjectArray<Node>			tNodeArray;
	typedef btAlignedObjectArray<btDbvt::Node*>	tLeafArray;
	typedef btAlignedObjectArray<Link>			tLinkArray;
	typedef btAlignedObjectArray<Face>			tFaceArray;
	typedef btAlignedObjectArray<Anchor>		tAnchorArray;
	typedef btAlignedObjectArray<RContact>		tRContactArray;
	typedef btAlignedObjectArray<SContact>		tSContactArray;
	typedef btAlignedObjectArray<btSoftBody*>	tSoftBodyArray;

	//
	// Fields
	//

	Config					m_cfg;			// Configuration
	SolverState				m_sst;			// Solver state
	Pose					m_pose;			// Pose
	DFld					m_dfld;			// Distance field
	void*					m_tag;			// User data
	btSoftBodyWorldInfo*	m_worldInfo;	//
	tAnchorArray			m_anchors;		// Anchors
	tRContactArray			m_rcontacts;	// Rigid contacts
	tSContactArray			m_scontacts;	// Soft contacts
	btScalar				m_timeacc;		// Time accumulator
	btVector3				m_bounds[2];	// Spatial bounds	
	bool					m_bUpdateRtCst;	// Update runtime constants
	btDbvt					m_ndbvt;		// Nodes tree
	btDbvt					m_fdbvt;		// Faces tree
	
	//
	// Api
	//
	
	/* ctor																	*/ 
	btSoftBody(	btSoftBody::btSoftBodyWorldInfo* worldInfo,int node_count,
				const btVector3* x,
				const btScalar* m);
	/* dtor																	*/ 
	virtual ~btSoftBody();
	/* Check for existing link												*/ 
	bool				checkLink(	int node0,
		int node1) const;
	bool				checkLink(	const btSoftBody::Node* node0,
		const btSoftBody::Node* node1) const;
	/* Check for existring face												*/ 
	bool				checkFace(	int node0,
		int node1,
		int node2) const;
	/* Append link															*/ 
	void				appendLink(		int node0,
		int node1,
		btScalar kST,
		btSoftBody::eLType::_ type,
		bool bcheckexist=false);
	void				appendLink(		btSoftBody::Node* node0,
		btSoftBody::Node* node1,
		btScalar kST,
		btSoftBody::eLType::_ type,
		bool bcheckexist=false);
	/* Append face															*/ 
	void				appendFace(		int node0,
		int node1,
		int node2);
	/* Append anchor														*/ 
	void				appendAnchor(	int node,
		btRigidBody* body);
	/* Add force (or gravity) to the entire body							*/ 
	void				addForce(		const btVector3& force);
	/* Add force (or gravity) to a node of the body							*/ 
	void				addForce(		const btVector3& force,
		int node);
	/* Add velocity to the entire body										*/ 
	void				addVelocity(	const btVector3& velocity);
	/* Add velocity to a node of the body									*/ 
	void				addVelocity(	const btVector3& velocity,
		int node);
	/* Set mass																*/ 
	void				setMass(		int node,
		btScalar mass);
	/* Get mass																*/ 
	btScalar			getMass(		int node) const;
	/* Get total mass														*/ 
	btScalar			getTotalMass() const;
	/* Set total mass (weighted by previous masses)							*/ 
	void				setTotalMass(	btScalar mass,
		bool fromfaces=false);
	/* Set total density													*/ 
	void				setTotalDensity(btScalar density);
	/* Transform															*/ 
	void				transform(		const btTransform& trs);
	/* Scale																*/ 
	void				scale(			const btVector3& scl);
	/* Set current state as pose											*/ 
	void				setPose(		bool bvolume,
										bool bframe);
	/* Set current state as distance field									*/ 
	void				setDistanceField(int nominalresolution);
	/* Return the volume													*/ 
	btScalar			getVolume() const;
	/* Generate bending constraints based on distance in the adjency graph	*/ 
	int					generateBendingConstraints(	int distance,
		btScalar stiffness);
	/* Randomize constraints to reduce solver bias							*/ 
	void				randomizeConstraints();
	/* Ray casting															*/ 
	bool				rayCast(const btVector3& org,
								const btVector3& dir,
								sRayCast& results,
								btScalar maxtime=SIMD_INFINITY);
	/* predictMotion														*/ 
	void				predictMotion(btScalar dt);
	/* solveConstraints														*/ 
	void				solveConstraints();
	/* solveCommonConstraints												*/ 
	static void			solveCommonConstraints(btSoftBody** bodies,int count,int iterations);
	/* integrateMotion														*/ 
	void				integrateMotion();
	/* defaultCollisionHandlers												*/ 
	void				defaultCollisionHandler(btCollisionObject* pco);
	void				defaultCollisionHandler(btSoftBody* psb);
	
	//
	// Accessor's and cast.
	//
	
	tNodeArray&			getNodes();
	const tNodeArray&	getNodes() const;
	tLinkArray&			getLinks();
	const tLinkArray&	getLinks() const;
	tFaceArray&			getFaces();
	const tFaceArray&	getFaces() const;
	
	//
	// Cast
	//
		
	static const btSoftBody*	upcast(const btCollisionObject* colObj)
	{
		if (colObj->getInternalType()==CO_SOFT_BODY)
			return (const btSoftBody*)colObj;
		return 0;
	}
	static btSoftBody*			upcast(btCollisionObject* colObj)
	{
		if (colObj->getInternalType()==CO_SOFT_BODY)
			return (btSoftBody*)colObj;
		return 0;
	}
	
};



#endif //_BT_SOFT_BODY_H
