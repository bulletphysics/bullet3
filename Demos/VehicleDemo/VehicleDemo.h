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
#ifndef VEHICLE_DEMO_H
#define VEHICLE_DEMO_H

class btVehicleTuning;
struct btVehicleRaycaster;
#include "BulletDynamics/Vehicle/btRaycastVehicle.h"

#include "DemoApplication.h"

///VehicleDemo shows how to setup and use the built-in raycast vehicle
class VehicleDemo : public DemoApplication
{
	public:

	btRigidBody* m_carChassis;
	
	btRaycastVehicle::btVehicleTuning	m_tuning;
	btVehicleRaycaster*	m_vehicleRayCaster;
	btRaycastVehicle*	m_vehicle;

	float		m_cameraHeight;

	float	m_minCameraDistance;
	float	m_maxCameraDistance;


	VehicleDemo();

	virtual void clientMoveAndDisplay();

	virtual void	clientResetScene();

	virtual void displayCallback();
	
	///a very basic camera following the vehicle
	virtual void updateCamera();

	virtual void specialKeyboard(int key, int x, int y);

	void renderme();

	void setupPhysics();
};

#endif //VEHICLE_DEMO_H


