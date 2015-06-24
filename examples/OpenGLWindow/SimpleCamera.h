#ifndef SIMPLE_CAMERA_H
#define SIMPLE_CAMERA_H

#include "../CommonInterfaces/CommonCameraInterface.h"

struct SimpleCamera : public CommonCameraInterface
{
	struct SimpleCameraInternalData* m_data;

	SimpleCamera();
	virtual ~SimpleCamera();

	void update();
	virtual void getCameraProjectionMatrix(float m[16]) const;
	virtual void getCameraViewMatrix(float m[16]) const;
	
	virtual void getCameraTargetPosition(float pos[3]) const;
	virtual void getCameraPosition(float pos[3]) const;

	virtual void getCameraTargetPosition(double pos[3]) const;
	virtual void getCameraPosition(double pos[3]) const;


	virtual void	setCameraTargetPosition(float x,float y,float z);
	virtual void	setCameraDistance(float dist);
	virtual float	getCameraDistance() const;

	virtual void	setCameraUpVector(float x,float y, float z);
	void			getCameraUpVector(float up[3]) const;
	///the setCameraUpAxis will call the 'setCameraUpVector' and 'setCameraForwardVector'
	virtual void	setCameraUpAxis(int axis);
	virtual int		getCameraUpAxis() const;

	virtual void	setCameraYaw(float yaw);
	virtual float	getCameraYaw() const;

	virtual void	setCameraPitch(float pitch);
	virtual float	getCameraPitch() const;

	virtual void	setAspectRatio(float ratio);
	virtual float	getAspectRatio() const;
};

#endif //SIMPLE_CAMERA_H