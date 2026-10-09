#include <cmath>

#include "CoilCreator.hpp"

namespace btInverseDynamics
{
CoilCreator::CoilCreator(int n) : m_num_bodies(n), m_parent((size_t)n)
{
	for (int i = 0; i < m_num_bodies; i++)
	{
		m_parent[(size_t)i] = i - 1;
	}

	// DH parameters (that's what's in the paper ...)
	const idScalar theta_DH = 0;
	const idScalar d_DH = 0.0;
	const idScalar a_DH = idScalar(1.0) / idScalar(m_num_bodies);
	const idScalar alpha_DH = idScalar(5.0) * BT_ID_PI / idScalar(m_num_bodies);
	getVecMatFromDH(theta_DH, d_DH, a_DH, alpha_DH, &m_parent_r_parent_body_ref,
					&m_body_T_parent_ref);
	// always z-axis
	m_body_axis_of_motion(0) = idScalar(0.0);
	m_body_axis_of_motion(1) = idScalar(0.0);
	m_body_axis_of_motion(2) = idScalar(1.0);

	m_mass = idScalar(1.0 / m_num_bodies);
	m_body_r_body_com(0) = idScalar(1.0) / (idScalar(2.0) * idScalar(m_num_bodies));
	m_body_r_body_com(1) = idScalar(0.0);
	m_body_r_body_com(2) = idScalar(0.0);

	m_body_I_body(0, 0) = idScalar(1e-4 / (idScalar(2.0) * idScalar(m_num_bodies)));
	m_body_I_body(0, 1) = idScalar(0.0);
	m_body_I_body(0, 2) = idScalar(0.0);
	m_body_I_body(1, 0) = idScalar(0.0);
	m_body_I_body(1, 1) = (idScalar(3e-4) + idScalar(4.0) / BT_ID_POW(idScalar(m_num_bodies), idScalar(2))) / (idScalar(12.0) * idScalar(m_num_bodies));
	m_body_I_body(1, 2) = idScalar(0.0);
	m_body_I_body(2, 0) = idScalar(0.0);
	m_body_I_body(2, 1) = idScalar(0.0);
	m_body_I_body(2, 2) = m_body_I_body(1, 1);
}

CoilCreator::~CoilCreator() {}

int CoilCreator::getNumBodies(int* num_bodies) const
{
	*num_bodies = m_num_bodies;
	return 0;
}

int CoilCreator::getBody(int body_index, int* parent_index, JointType* joint_type,
						 vec3* parent_r_parent_body_ref, mat33* body_T_parent_ref,
						 vec3* body_axis_of_motion, idScalar* mass, vec3* body_r_body_com,
						 mat33* body_I_body, int* user_int, void** user_ptr) const
{
	if (body_index < 0 || body_index >= m_num_bodies)
	{
		bt_id_error_message("invalid body index %d\n", body_index);
		return -1;
	}
	*parent_index = m_parent[(size_t)body_index];
	*joint_type = REVOLUTE;
	*parent_r_parent_body_ref = m_parent_r_parent_body_ref;
	*body_T_parent_ref = m_body_T_parent_ref;
	*body_axis_of_motion = m_body_axis_of_motion;
	*mass = m_mass;
	*body_r_body_com = m_body_r_body_com;
	*body_I_body = m_body_I_body;

	*user_int = 0;
	*user_ptr = 0;
	return 0;
}
}  // namespace btInverseDynamics
