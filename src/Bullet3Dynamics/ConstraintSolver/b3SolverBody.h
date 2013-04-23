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

#ifndef BT_SOLVER_BODY_H
#define BT_SOLVER_BODY_H

class	btRigidBody;
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Matrix3x3.h"

#include "Bullet3Common/b3AlignedAllocator.h"
#include "Bullet3Common/b3TransformUtil.h"

///Until we get other contributions, only use SIMD on Windows, when using Visual Studio 2008 or later, and not double precision
#ifdef BT_USE_SSE
#define USE_SIMD 1
#endif //


#ifdef USE_SIMD

struct	btSimdScalar
{
	SIMD_FORCE_INLINE	btSimdScalar()
	{

	}

	SIMD_FORCE_INLINE	btSimdScalar(float	fl)
	:m_vec128 (_mm_set1_ps(fl))
	{
	}

	SIMD_FORCE_INLINE	btSimdScalar(__m128 v128)
		:m_vec128(v128)
	{
	}
	union
	{
		__m128		m_vec128;
		float		m_floats[4];
		int			m_ints[4];
		b3Scalar	m_unusedPadding;
	};
	SIMD_FORCE_INLINE	__m128	get128()
	{
		return m_vec128;
	}

	SIMD_FORCE_INLINE	const __m128	get128() const
	{
		return m_vec128;
	}

	SIMD_FORCE_INLINE	void	set128(__m128 v128)
	{
		m_vec128 = v128;
	}

	SIMD_FORCE_INLINE	operator       __m128()       
	{ 
		return m_vec128; 
	}
	SIMD_FORCE_INLINE	operator const __m128() const 
	{ 
		return m_vec128; 
	}
	
	SIMD_FORCE_INLINE	operator float() const 
	{ 
		return m_floats[0]; 
	}

};

///@brief Return the elementwise product of two btSimdScalar
SIMD_FORCE_INLINE btSimdScalar 
operator*(const btSimdScalar& v1, const btSimdScalar& v2) 
{
	return btSimdScalar(_mm_mul_ps(v1.get128(),v2.get128()));
}

///@brief Return the elementwise product of two btSimdScalar
SIMD_FORCE_INLINE btSimdScalar 
operator+(const btSimdScalar& v1, const btSimdScalar& v2) 
{
	return btSimdScalar(_mm_add_ps(v1.get128(),v2.get128()));
}


#else
#define btSimdScalar b3Scalar
#endif

///The b3SolverBody is an internal datastructure for the constraint solver. Only necessary data is packed to increase cache coherence/performance.
ATTRIBUTE_ALIGNED64 (struct)	b3SolverBody
{
	BT_DECLARE_ALIGNED_ALLOCATOR();
	b3Transform		m_worldTransform;
	b3Vector3		m_deltaLinearVelocity;
	b3Vector3		m_deltaAngularVelocity;
	b3Vector3		m_angularFactor;
	b3Vector3		m_linearFactor;
	b3Vector3		m_invMass;
	b3Vector3		m_pushVelocity;
	b3Vector3		m_turnVelocity;
	b3Vector3		m_linearVelocity;
	b3Vector3		m_angularVelocity;

	union 
	{
		void*	m_originalBody;
		int		m_originalBodyIndex;
	};


	void	setWorldTransform(const b3Transform& worldTransform)
	{
		m_worldTransform = worldTransform;
	}

	const b3Transform& getWorldTransform() const
	{
		return m_worldTransform;
	}
	
	SIMD_FORCE_INLINE void	getVelocityInLocalPointObsolete(const b3Vector3& rel_pos, b3Vector3& velocity ) const
	{
		if (m_originalBody)
			velocity = m_linearVelocity+m_deltaLinearVelocity + (m_angularVelocity+m_deltaAngularVelocity).cross(rel_pos);
		else
			velocity.setValue(0,0,0);
	}

	SIMD_FORCE_INLINE void	getAngularVelocity(b3Vector3& angVel) const
	{
		if (m_originalBody)
			angVel =m_angularVelocity+m_deltaAngularVelocity;
		else
			angVel.setValue(0,0,0);
	}


	//Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position
	SIMD_FORCE_INLINE void applyImpulse(const b3Vector3& linearComponent, const b3Vector3& angularComponent,const b3Scalar impulseMagnitude)
	{
		if (m_originalBody)
		{
			m_deltaLinearVelocity += linearComponent*impulseMagnitude*m_linearFactor;
			m_deltaAngularVelocity += angularComponent*(impulseMagnitude*m_angularFactor);
		}
	}

	SIMD_FORCE_INLINE void internalApplyPushImpulse(const b3Vector3& linearComponent, const b3Vector3& angularComponent,b3Scalar impulseMagnitude)
	{
		if (m_originalBody)
		{
			m_pushVelocity += linearComponent*impulseMagnitude*m_linearFactor;
			m_turnVelocity += angularComponent*(impulseMagnitude*m_angularFactor);
		}
	}



	const b3Vector3& getDeltaLinearVelocity() const
	{
		return m_deltaLinearVelocity;
	}

	const b3Vector3& getDeltaAngularVelocity() const
	{
		return m_deltaAngularVelocity;
	}

	const b3Vector3& getPushVelocity() const 
	{
		return m_pushVelocity;
	}

	const b3Vector3& getTurnVelocity() const 
	{
		return m_turnVelocity;
	}


	////////////////////////////////////////////////
	///some internal methods, don't use them
		
	b3Vector3& internalGetDeltaLinearVelocity()
	{
		return m_deltaLinearVelocity;
	}

	b3Vector3& internalGetDeltaAngularVelocity()
	{
		return m_deltaAngularVelocity;
	}

	const b3Vector3& internalGetAngularFactor() const
	{
		return m_angularFactor;
	}

	const b3Vector3& internalGetInvMass() const
	{
		return m_invMass;
	}

	void internalSetInvMass(const b3Vector3& invMass)
	{
		m_invMass = invMass;
	}
	
	b3Vector3& internalGetPushVelocity()
	{
		return m_pushVelocity;
	}

	b3Vector3& internalGetTurnVelocity()
	{
		return m_turnVelocity;
	}

	SIMD_FORCE_INLINE void	internalGetVelocityInLocalPointObsolete(const b3Vector3& rel_pos, b3Vector3& velocity ) const
	{
		velocity = m_linearVelocity+m_deltaLinearVelocity + (m_angularVelocity+m_deltaAngularVelocity).cross(rel_pos);
	}

	SIMD_FORCE_INLINE void	internalGetAngularVelocity(b3Vector3& angVel) const
	{
		angVel = m_angularVelocity+m_deltaAngularVelocity;
	}


	//Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position
	SIMD_FORCE_INLINE void internalApplyImpulse(const b3Vector3& linearComponent, const b3Vector3& angularComponent,const b3Scalar impulseMagnitude)
	{
		if (m_originalBody)
		{
			m_deltaLinearVelocity += linearComponent*impulseMagnitude*m_linearFactor;
			m_deltaAngularVelocity += angularComponent*(impulseMagnitude*m_angularFactor);
		}
	}
		
	
	

	void	writebackVelocity()
	{
		//if (m_originalBody>=0)
		{
			m_linearVelocity +=m_deltaLinearVelocity;
			m_angularVelocity += m_deltaAngularVelocity;
			
			//m_originalBody->setCompanionId(-1);
		}
	}


	void	writebackVelocityAndTransform(b3Scalar timeStep, b3Scalar splitImpulseTurnErp)
	{
        (void) timeStep;
		if (m_originalBody)
		{
			m_linearVelocity += m_deltaLinearVelocity;
			m_angularVelocity += m_deltaAngularVelocity;
			
			//correct the position/orientation based on push/turn recovery
			b3Transform newTransform;
			if (m_pushVelocity[0]!=0.f || m_pushVelocity[1]!=0 || m_pushVelocity[2]!=0 || m_turnVelocity[0]!=0.f || m_turnVelocity[1]!=0 || m_turnVelocity[2]!=0)
			{
			//	btQuaternion orn = m_worldTransform.getRotation();
				b3TransformUtil::integrateTransform(m_worldTransform,m_pushVelocity,m_turnVelocity*splitImpulseTurnErp,timeStep,newTransform);
				m_worldTransform = newTransform;
			}
			//m_worldTransform.setRotation(orn);
			//m_originalBody->setCompanionId(-1);
		}
	}
	


};

#endif //BT_SOLVER_BODY_H


