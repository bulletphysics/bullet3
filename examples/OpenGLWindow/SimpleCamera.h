#ifndef SIMPLE_CAMERA_H
#define SIMPLE_CAMERA_H

struct CommonCameraInterface
{
	virtual void getCameraProjectionMatrix(float m[16])const = 0;
	virtual void getCameraViewMatrix(float m[16]) const = 0;
};

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

	virtual void	setCameraTargetPosition(float x,float y,float z);
	virtual void	setCameraDistance(float dist);
	virtual void	setCameraUpVector(float x,float y, float z);
	///the setCameraUpAxis will call the 'setCameraUpVector' and 'setCameraForwardVector'
	virtual void	setCameraUpAxis(int axis);
	virtual void	setCameraYaw(float yaw);
	
	virtual void	setCameraPitch(float pitch);
	virtual void	setAspectRatio(float ratio);

};

#endif //SIMPLE_CAMERA_H