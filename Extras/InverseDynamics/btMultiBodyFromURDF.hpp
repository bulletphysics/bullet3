#ifndef BTMULTIBODYFROMURDF_HPP
#define BTMULTIBODYFROMURDF_HPP

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"
#include "../../examples/CommonInterfaces/CommonGUIHelperInterface.h"
#include "../../examples/Importers/ImportURDFDemo/BulletUrdfImporter.h"
#include "../../examples/Importers/ImportURDFDemo/URDF2Bullet.h"
#include "../../examples/Importers/ImportURDFDemo/MyMultiBodyCreator.h"
#include "../../examples/Importers/ImportURDFDemo/URDF2Bullet.h"
#include "../../examples/Utils/b3BulletDefaultFileIO.h"
/// Create a btMultiBody model from URDF.
/// This is adapted from Bullet URDF loader example
class MyBtMultiBodyFromURDF
{
public:
	/// ctor
	/// @param gravity gravitational acceleration (in world frame)
	/// @param base_fixed if true, the root body is treated as fixed,
	///        if false, it is treated as floating
	MyBtMultiBodyFromURDF(const btVector3 &gravity, const bool base_fixed)
		: m_gravity(gravity), m_base_fixed(base_fixed)
	{
		m_broadphase = 0x0;
		m_dispatcher = 0x0;
		m_solver = 0x0;
		m_collisionConfiguration = 0x0;
		m_dynamicsWorld = 0x0;
		m_multibody = 0x0;
	}
	/// dtor
	~MyBtMultiBodyFromURDF()
	{
		delete m_dynamicsWorld;
		delete m_solver;
		delete m_broadphase;
		delete m_dispatcher;
		delete m_collisionConfiguration;
		delete m_multibody;
	}
	/// @param name path to urdf file
	void setFileName(const std::string name) { m_filename = name; }
	/// load urdf file and build btMultiBody model
	void init()
	{
		this->createEmptyDynamicsWorld();
		m_dynamicsWorld->setGravity(m_gravity);
		b3BulletDefaultFileIO fileIO;
		BulletURDFImporter urdf_importer(&m_nogfx, 0, &fileIO, 1, 0);
		URDFImporterInterface &u2b(urdf_importer);
		bool loadOk = u2b.loadURDF(m_filename.c_str(), m_base_fixed);

		if (loadOk)
		{
			btTransform identityTrans;
			identityTrans.setIdentity();
			MyMultiBodyCreator creation(&m_nogfx);
			const bool use_multibody = true;
			ConvertURDF2Bullet(u2b, creation, identityTrans, m_dynamicsWorld, use_multibody,
							   u2b.getPathPrefix());
			m_multibody = creation.getBulletMultiBody();
			m_dynamicsWorld->stepSimulation(1. / 240., 0);
		}
	}
	/// @return pointer to the btMultiBody model
	btMultiBody *getBtMultiBody() { return m_multibody; }

private:
	// internal utility function
	void createEmptyDynamicsWorld()
	{
		m_collisionConfiguration = new btDefaultCollisionConfiguration();

		/// use the default collision dispatcher. For parallel processing you can use a diffent
		/// dispatcher (see Extras/BulletMultiThreaded)
		m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
		m_broadphase = new btDbvtBroadphase();
		m_solver = new btMultiBodyConstraintSolver;
		m_dynamicsWorld = new btMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, m_solver,
													   m_collisionConfiguration);
		m_dynamicsWorld->setGravity(m_gravity);
	}

	btBroadphaseInterface *m_broadphase;
	btCollisionDispatcher *m_dispatcher;
	btMultiBodyConstraintSolver *m_solver;
	btDefaultCollisionConfiguration *m_collisionConfiguration;
	btMultiBodyDynamicsWorld *m_dynamicsWorld;
	std::string m_filename;
	DummyGUIHelper m_nogfx;
	btMultiBody *m_multibody;
	const btVector3 m_gravity;
	const bool m_base_fixed;
};
#endif  // BTMULTIBODYFROMURDF_HPP
