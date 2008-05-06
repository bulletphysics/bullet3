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
#include "BulletCollision/BroadphaseCollision/btDbvt.h"

class btBroadphaseInterface;
class btCollisionDispatcher;

/// btSoftBody is work-in-progress
class	btSoftBody : public btCollisionObject
{
public:
	//
	// Enumerations
	//

	///eAeroModel 
	struct eAeroModel { enum _ {
		V_Point,	///Vertex normals are oriented toward velocity
		V_TwoSided,	///Vertex normals are fliped to match velocity	
		V_OneSided,	///Vertex normals are taken as it is	
		F_TwoSided,	///Face normals are fliped to match velocity
		F_OneSided,	///Face normals are taken as it is
	};};
	
	///eVSolver : velocities solvers
	struct	eVSolver { enum _ {
		Linear,		///Linear solver
		Volume,		///Volume solver
	};};
	
	///ePSolver : positions solvers
	struct	ePSolver { enum _ {
		Linear,		///Linear solver
		Volume,		///Volume solver
		Anchors,	///Anchor solver
		RContacts,	///Rigid contacts solver
		SContacts,	///Soft contacts solver
	};};
	
	///eSolverPresets
	struct	eSolverPresets { enum _ {
		Positions,
		Velocities,
		Default	=	Positions,
	};};
	
	///eFeature
	struct	eFeature { enum _ {
		None,
		Node,
		Link,
		Face,
		Tetra,
	};};
	
	typedef btAlignedObjectArray<eVSolver::_>	tVSolverArray;
	typedef btAlignedObjectArray<ePSolver::_>	tPSolverArray;
	
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
	
	///fMaterial
	struct fMaterial { enum _ {
		DebugDraw	=	0x0001,	/// Enable debug draw
		/* presets	*/ 
		Default		=	DebugDraw,
	};};
		
	//
	// API Types
	//
	
	/* sRayCast		*/ 
	struct sRayCast
	{
		btSoftBody*	body;		/// soft body
		eFeature::_	feature;	/// feature type
		int			index;		/// feature index
		btScalar	time;		/// time of impact (rayorg+raydir*time)
	};
	
	/* ImplicitFn	*/ 
	struct	ImplicitFn
	{
		virtual btScalar	Eval(const btVector3& x)=0;
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
		Element()
		{
			m_tag=0;
		}
	};
	/* Material		*/ 
	struct	Material : Element
	{
		btScalar				m_kLST;			// Linear stiffness coefficient [0,1]
		btScalar				m_kAST;			// Area stiffness coefficient [0,1]
		btScalar				m_kVST;			// Volume stiffness coefficient [0,1]
		int						m_flags;		// Flags
	};
		
	/* Feature		*/ 
	struct	Feature : Element
	{
		Material*				m_material;		// Material
	};
	/* Node			*/ 
	struct	Node : Feature
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
	struct	Link : Feature
	{
		Node*					m_n[2];			// Node pointers
		btScalar				m_rl;			// Rest length
		btScalar				m_c0;			// (ima+imb)*kLST
		btScalar				m_c1;			// rl^2
		btScalar				m_c2;			// |gradient|^2/c0
		btVector3				m_c3;			// gradient
	};
	/* Face			*/ 
	struct	Face : Feature
	{
		Node*					m_n[3];			// Node pointers
		btVector3				m_normal;		// Normal
		btScalar				m_ra;			// Rest area
		btDbvt::Node*			m_leaf;			// Leaf data
	};
	/* Tetra		*/ 
	struct	Tetra : Feature
	{
		Node*					m_n[4];			// Node pointers		
		btScalar				m_rv;			// Rest volume
		btDbvt::Node*			m_leaf;			// Leaf data
		btVector3				m_c0[4];		// gradients
		btScalar				m_c1;			// (4*kVST)/(im0+im1+im2+im3)
		btScalar				m_c2;			// m_c1/sum(|g0..3|^2)
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
	/* Note			*/ 
	struct	Note : Element
	{
		const char*				m_text;			// Text
		btVector3				m_offset;		// Offset
		int						m_rank;			// Rank
		Node*					m_nodes[4];		// Nodes
		btScalar				m_coords[4];	// Coordinates
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
	/* Config		*/ 
	struct	Config
	{
		eAeroModel::_			aeromodel;		// Aerodynamic model (default: V_Point)
		btScalar				kVCF;			// Velocities correction factor (Baumgarte)
		btScalar				kDP;			// Damping coefficient [0,1]
		btScalar				kDG;			// Drag coefficient [0,+inf]
		btScalar				kLF;			// Lift coefficient [0,+inf]
		btScalar				kPR;			// Pressure coefficient [-inf,+inf]
		btScalar				kVC;			// Volume conversation coefficient [0,+inf]
		btScalar				kDF;			// Dynamic friction coefficient [0,1]
		btScalar				kMT;			// Pose matching coefficient [0,1]		
		btScalar				kCHR;			// Rigid contacts hardness [0,1]
		btScalar				kKHR;			// Kinetic contacts hardness [0,1]
		btScalar				kSHR;			// Soft contacts hardness [0,1]
		btScalar				kAHR;			// Anchors hardness [0,1]
		btScalar				maxvolume;		// Maximum volume ratio for pose
		btScalar				timescale;		// Time scale
		int						viterations;	// Velocities solver iterations
		int						piterations;	// Positions solver iterations
		int						diterations;	// Drift solver iterations
		int						collisions;		// Collisions flags
		tVSolverArray			m_vsequence;	// Velocity solvers sequence
		tPSolverArray			m_psequence;	// Position solvers sequence
		tPSolverArray			m_dsequence;	// Drift solvers sequence
	};
	/* SolverState	*/ 
	struct	SolverState
	{
		btScalar				sdt;			// dt*timescale
		btScalar				isdt;			// 1/sdt
		btScalar				velmrg;			// velocity margin
		btScalar				radmrg;			// radial margin
		btScalar				updmrg;			// Update margin
	};

	//
	// Typedef's
	//

	typedef btAlignedObjectArray<Note>			tNoteArray;
	typedef btAlignedObjectArray<Node>			tNodeArray;
	typedef btAlignedObjectArray<btDbvt::Node*>	tLeafArray;
	typedef btAlignedObjectArray<Link>			tLinkArray;
	typedef btAlignedObjectArray<Face>			tFaceArray;
	typedef btAlignedObjectArray<Tetra>			tTetraArray;
	typedef btAlignedObjectArray<Anchor>		tAnchorArray;
	typedef btAlignedObjectArray<RContact>		tRContactArray;
	typedef btAlignedObjectArray<SContact>		tSContactArray;
	typedef btAlignedObjectArray<Material*>		tMaterialArray;
	typedef btAlignedObjectArray<btSoftBody*>	tSoftBodyArray;		

	//
	// Fields
	//

	Config					m_cfg;			// Configuration
	SolverState				m_sst;			// Solver state
	Pose					m_pose;			// Pose
	void*					m_tag;			// User data
	btSoftBodyWorldInfo*	m_worldInfo;	//
	tNoteArray				m_notes;		// Notes
	tNodeArray				m_nodes;		// Nodes
	tLinkArray				m_links;		// Links
	tFaceArray				m_faces;		// Faces
	tTetraArray				m_tetras;		// Tetras
	tAnchorArray			m_anchors;		// Anchors
	tRContactArray			m_rcontacts;	// Rigid contacts
	tSContactArray			m_scontacts;	// Soft contacts
	tMaterialArray			m_materials;	// Materials
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
	/* Append material														*/ 
	Material*			appendMaterial();
	/* Append note															*/ 
	void				appendNote(	const char* text,
									const btVector3& o,
									const btVector4& c=btVector4(1,0,0,0),
									Node* n0=0,
									Node* n1=0,
									Node* n2=0,
									Node* n3=0);
	void				appendNote(	const char* text,
									const btVector3& o,
									Node* feature);
	void				appendNote(	const char* text,
									const btVector3& o,
									Link* feature);
	void				appendNote(	const char* text,
									const btVector3& o,
									Face* feature);
	void				appendNote(	const char* text,
									const btVector3& o,
									Tetra* feature);
	/* Append node															*/ 
	void				appendNode(	const btVector3& x,btScalar m);
	/* Append link															*/ 
	void				appendLink(int model=-1,Material* mat=0);
	void				appendLink(	int node0,
									int node1,
									Material* mat=0,
									bool bcheckexist=false);
	void				appendLink(	btSoftBody::Node* node0,
									btSoftBody::Node* node1,
									Material* mat=0,
									bool bcheckexist=false);
	/* Append face															*/ 
	void				appendFace(int model=-1,Material* mat=0);
	void				appendFace(	int node0,
									int node1,
									int node2,
									Material* mat=0);
	/* Append tetrahedron													*/ 
	void				appendTetra(int model=-1,Material* mat=0);
	void				appendTetra(int node0,
									int node1,
									int node2,
									int node3,
									Material* mat=0);
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
	/* Set volume mass (using tetrahedrons)									*/ 
	void				setVolumeMass(		btScalar mass);
	/* Set volume density (using tetrahedrons)								*/ 
	void				setVolumeDensity(	btScalar density);
	/* Transform															*/ 
	void				transform(		const btTransform& trs);
	/* Translate															*/ 
	void				translate(		const btVector3& trs);
	/* Rotate															*/ 
	void				rotate(	const btQuaternion& rot);
	/* Scale																*/ 
	void				scale(	const btVector3& scl);
	/* Set current state as pose											*/ 
	void				setPose(		bool bvolume,
										bool bframe);
	/* Return the volume													*/ 
	btScalar			getVolume() const;
	/* Generate bending constraints based on distance in the adjency graph	*/ 
	int					generateBendingConstraints(	int distance,
													Material* mat=0);
	/* Generate tetrahedral constraints										*/ 
	int					generateTetrahedralConstraints();
	/* Randomize constraints to reduce solver bias							*/ 
	void				randomizeConstraints();
	/* Refine																*/ 
	void				refine(ImplicitFn* ifn,btScalar accurary,bool cut);
	/* CutLink																*/ 
	bool				cutLink(int node0,int node1,btScalar position);
	bool				cutLink(const Node* node0,const Node* node1,btScalar position);
	/* Ray casting															*/ 
	bool				rayCast(const btVector3& org,
								const btVector3& dir,
								sRayCast& results,
								btScalar maxtime=SIMD_INFINITY);
	/* Solver presets														*/ 
	void				setSolver(eSolverPresets::_ preset);
	/* predictMotion														*/ 
	void				predictMotion(btScalar dt);
	/* solveConstraints														*/ 
	void				solveConstraints();
	/* staticSolve															*/ 
	void				staticSolve(int iterations);
	/* solveCommonConstraints												*/ 
	static void			solveCommonConstraints(btSoftBody** bodies,int count,int iterations);
	/* integrateMotion														*/ 
	void				integrateMotion();
	/* defaultCollisionHandlers												*/ 
	void				defaultCollisionHandler(btCollisionObject* pco);
	void				defaultCollisionHandler(btSoftBody* psb);
		
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

	virtual void getAabb(btVector3& aabbMin,btVector3& aabbMax) const
	{
		aabbMin = m_bounds[0];
		aabbMax = m_bounds[1];
	}

	
};



#endif //_BT_SOFT_BODY_H
