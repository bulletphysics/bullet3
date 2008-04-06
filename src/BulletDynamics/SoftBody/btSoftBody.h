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
#include "BulletDynamics/SoftBody/btSparseSDF.h"

class btBroadphaseInterface;
class btCollisionDispatcher;

/// btSoftBody is work-in-progress
struct	btSoftBody : public btCollisionObject
{

	//
	// Enumerations
	//

	///eLType
	struct eLType { enum _ {
		Structural,	///Master constraints 
		Bending,	///Secondary constraints
	};};

	////eAeroModel 
	struct eAeroModel { enum _ {
		V_Point,	///Vertex normals are oriented toward velocity
		V_TwoSided,	///Vertex normals are fliped to match velocity	
		V_OneSided,	///Vertex normals are taken as it is	
		F_TwoSided,	///Face normals are fliped to match velocity
		F_OneSided,	///Face normals are taken as it is
	};};

	struct	btSoftBodyWorldInfo
	{
		btScalar	air_density;
		btScalar	water_density;
		btScalar	water_offset;
		btVector3	water_normal;	
		btBroadphaseInterface*	m_broadphase;
		btCollisionDispatcher*	m_dispatcher;
		btVector3	m_gravity;

		btSparseSdf<3>						m_sparsesdf;
	};

	//reference or copy?
	btSoftBodyWorldInfo&	m_worldInfo;

	///constructor

	btSoftBody(btSoftBody::btSoftBodyWorldInfo& worldInfo,int node_count,
								   const btVector3* x,
								   const btScalar* m);


	///sCti is Softbody contact info
	struct	sCti
	{
		btRigidBody*	m_body;		/* Rigid body			*/ 
		btVector3		m_normal;	/* Outward normal		*/ 
		btScalar		m_offset;	/* Offset from origin	*/ 
	};
	

	void	StartCollide(const btVector3& aabbMin,const btVector3& aabbMax)
	{
		//??
	}
	void	EndCollide()
	{
		//??
	}

	//
	bool		CheckContact(	const btVector3& x, btSoftBody::sCti& cti);
	////

	///destructor
	virtual ~btSoftBody();



	struct	sMedium
	{
		btVector3		m_velocity;	/* Velocity				*/ 
		btScalar		m_pressure;	/* Pressure				*/ 
		btScalar		m_density;	/* Density				*/ 
	};

	virtual void		EvaluateMedium(	const btVector3& /*position*/, 	sMedium& medium);

	

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
	///Node
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
		btSoftBody::eLType::_		m_type;			// Link type
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
		btSoftBody::sCti	m_cti;			// Contact infos
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
		btSoftBody::eAeroModel::_	aeromodel;		// Aerodynamic model (default: V_Point)
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
	void*				m_tag;			// User data

	//////////////////////

	///btSoftBodyCollisionShape is work-in-progress collision shape for softbodies
	class btSoftBodyCollisionShape : public btConcaveShape
	{
		static btVector3 m_sScaling;
	public:

		tNodeArray			m_nodes;		// Nodes
		tLinkArray			m_links;		// Links
		tFaceArray			m_faces;		// Faces

		btSoftBodyCollisionShape();

		virtual ~btSoftBodyCollisionShape();

		virtual void	processAllTriangles(btTriangleCallback* callback,const btVector3& aabbMin,const btVector3& aabbMax) const;

		///getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
		virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const
		{
			///not yet
			btAssert(0);
		}

		virtual int		getShapeType() const
		{
			return SOFTBODY_SHAPE_PROXYTYPE;
		}
		virtual void	setLocalScaling(const btVector3& scaling)
		{
			///not yet
			btAssert(0);
		}
		virtual const btVector3& getLocalScaling() const
		{
			///not yet
			btAssert(0);
			return m_sScaling;
		}
		virtual void	calculateLocalInertia(btScalar mass,btVector3& inertia) const
		{
			///not yet
			btAssert(0);
		}
		virtual const char*	getName()const
		{
			return "SoftBody";
		}

	};


	btSoftBodyCollisionShape*	m_softBodyCollisionShape;

	btCollisionObjectArray m_overlappingRigidBodies;

	btAlignedObjectArray<btSoftBody*>		m_overlappingSoftBodies;


	//////////////////////

	inline tNodeArray&	getNodes()
	{
		return m_softBodyCollisionShape->m_nodes;
	}
	inline const tNodeArray&	getNodes() const
	{
		return m_softBodyCollisionShape->m_nodes;
	}

	inline tLinkArray&			getLinks()
	{
		return m_softBodyCollisionShape->m_links;
	}
	inline const	tLinkArray&			getLinks() const
	{
		return m_softBodyCollisionShape->m_links;
	}

	inline tFaceArray&			getFaces()
	{
		return m_softBodyCollisionShape->m_faces;
	}

	inline const	tFaceArray&			getFaces() const
	{
		return m_softBodyCollisionShape->m_faces;
	}


	tAnchorArray		m_anchors;		// Anchors
	tContactArray		m_contacts;		// Contacts
	btScalar			m_timeacc;		// Time accumulator
	btVector3			m_bounds[2];	// Spatial bounds	
	bool				m_bUpdateRtCst;	// Update runtime constants

	//
	// Api
	//

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

	void	updateBounds();

	void			updateTransform()
	{
		updateBounds();
	}


};



#endif //_BT_SOFT_BODY_H
