/*
 * Copyright (c) 2005 Erwin Coumans http://continuousphysics.com/Bullet/
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability 
 * of this software for any purpose.  
 * It is provided "as is" without express or implied warranty.
*/
#ifndef RAYCASTVEHICLE_H
#define RAYCASTVEHICLE_H

#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"

struct btMassProps;
#include "btWheelInfo.h"

struct	btVehicleRaycaster;
class btVehicleTuning;

///Raycast vehicle, very special constraint that turn a rigidbody into a vehicle.
class btRaycastVehicle : public btTypedConstraint
{
public:
	class btVehicleTuning
		{
			public:

			btVehicleTuning()
				:m_suspensionStiffness(5.88f),
				m_suspensionCompression(0.83f),
				m_suspensionDamping(0.88f),
				m_maxSuspensionTravelCm(500.f),
				m_frictionSlip(10.5f)
			{
			}
			float	m_suspensionStiffness;
			float	m_suspensionCompression;
			float	m_suspensionDamping;
			float	m_maxSuspensionTravelCm;
			float	m_frictionSlip;

		};
private:

	btScalar	m_tau;
	btScalar	m_damping;
	btVehicleRaycaster*	m_vehicleRaycaster;
	float		m_pitchControl;
	float	m_steeringValue; 
	float m_currentVehicleSpeedKmHour;

	btRigidBody* m_chassisBody;

	int m_indexRightAxis;
	int m_indexUpAxis;
	int	m_indexForwardAxis;

	void DefaultInit(const btVehicleTuning& tuning);

public:

	//constructor to create a car from an existing rigidbody
	btRaycastVehicle(const btVehicleTuning& tuning,btRigidBody* chassis,	btVehicleRaycaster* raycaster );

	virtual ~btRaycastVehicle() ;

		



	btScalar Raycast(btWheelInfo& wheel);

	virtual void UpdateVehicle(btScalar step);

	void ResetSuspension();

	btScalar	GetSteeringValue(int wheel) const;

	void	SetSteeringValue(btScalar steering,int wheel);


	void	ApplyEngineForce(btScalar force, int wheel);

	const btTransform&	GetWheelTransformWS( int wheelIndex ) const;

	void	UpdateWheelTransform( int wheelIndex );
	
	void	SetRaycastWheelInfo( int wheelIndex , bool isInContact, const btVector3& hitPoint, const btVector3& hitNormal,btScalar depth);

	btWheelInfo&	AddWheel( const btVector3& connectionPointCS0, const btVector3& wheelDirectionCS0,const btVector3& wheelAxleCS,btScalar suspensionRestLength,btScalar wheelRadius,const btVehicleTuning& tuning, bool isFrontWheel);

	inline int		GetNumWheels() const {
		return m_wheelInfo.size();
	}
	
	std::vector<btWheelInfo>	m_wheelInfo;


	const btWheelInfo&	GetWheelInfo(int index) const;

	btWheelInfo&	GetWheelInfo(int index);

	void	UpdateWheelTransformsWS(btWheelInfo& wheel );

	
	void SetBrake(float brake,int wheelIndex);

	void	SetPitchControl(float pitch)
	{
		m_pitchControl = pitch;
	}
	
	void	UpdateSuspension(btScalar deltaTime);

	void	UpdateFriction(btScalar	timeStep);



	inline btRigidBody* GetRigidBody()
	{
		return m_chassisBody;
	}

	const btRigidBody* GetRigidBody() const
	{
		return m_chassisBody;
	}

	inline int	GetRightAxis() const
	{
		return m_indexRightAxis;
	}
	inline int GetUpAxis() const
	{
		return m_indexUpAxis;
	}

	inline int GetForwardAxis() const
	{
		return m_indexForwardAxis;
	}

	virtual void	SetCoordinateSystem(int rightIndex,int upIndex,int forwardIndex)
	{
		m_indexRightAxis = rightIndex;
		m_indexUpAxis = upIndex;
		m_indexForwardAxis = forwardIndex;
	}

	virtual void	BuildJacobian()
	{
		//not yet
	}

	virtual	void	SolveConstraint(btScalar	timeStep)
	{
		//not yet
	}


};

#endif //RAYCASTVEHICLE_H

