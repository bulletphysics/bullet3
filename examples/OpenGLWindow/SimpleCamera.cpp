#include "SimpleCamera.h"

#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3Matrix3x3.h"
#include "Bullet3Common/b3Transform.h"


B3_ATTRIBUTE_ALIGNED16(struct) SimpleCameraInternalData
{
	SimpleCameraInternalData()
		:m_cameraTargetPosition(b3MakeVector3(0,0,0)),
		m_cameraDistance(20),
		m_cameraUp(b3MakeVector3(0,1,0)),
		m_cameraForward(b3MakeVector3(1,0,0)),
	 	m_cameraUpAxis(1),	
		m_yaw(20),
		m_pitch(0),
		m_aspect(1),
 		m_frustumZNear(0.01),
		m_frustumZFar(1000),
		m_enableVR(false)
	{
		b3Transform tr;
		tr.setIdentity();
		tr.getOpenGLMatrix(m_offsetTransformVR);
	}
	
    B3_DECLARE_ALIGNED_ALLOCATOR();
    
    B3_ATTRIBUTE_ALIGNED16(float) m_offsetTransformVR[16];
    b3Vector3 m_cameraTargetPosition;
	float m_cameraDistance;
	b3Vector3 m_cameraUp;
	b3Vector3 m_cameraForward;
	int m_cameraUpAxis;
	//the m_cameraPosition is a cached value, recomputed from other values
	b3Vector3 m_cameraPosition;
	float m_yaw;
	
	float m_pitch;
	float m_aspect;
	float m_frustumZNear;
    float m_frustumZFar;

	bool m_enableVR;
	float m_viewMatrixVR[16];
	float m_projectionMatrixVR[16];
	
};




SimpleCamera::SimpleCamera()
{
	m_data = new SimpleCameraInternalData;
}
SimpleCamera::~SimpleCamera()
{
	delete m_data;
}

void	SimpleCamera::setVRCamera(const float viewMat[16], const float projectionMatrix[16])
{
	m_data->m_enableVR = true;
	for (int i=0;i<16;i++)
	{
		m_data->m_viewMatrixVR[i] = viewMat[i];
		m_data->m_projectionMatrixVR[i] = projectionMatrix[i];
		m_data->m_frustumZNear = m_data->m_projectionMatrixVR[14]/(m_data->m_projectionMatrixVR[10]-1);
		m_data->m_frustumZFar = m_data->m_projectionMatrixVR[14]/(m_data->m_projectionMatrixVR[10]+1);
	}
}

bool	SimpleCamera::getVRCamera(float viewMat[16], float projectionMatrix[16])
{
	if (m_data->m_enableVR)
	{
		for (int i=0;i<16;i++)
		{
			viewMat[i] = m_data->m_viewMatrixVR[i];
			projectionMatrix[i] = m_data->m_projectionMatrixVR[i];
		}
	}
	return false;
}



void SimpleCamera::disableVRCamera()
{
	m_data->m_enableVR = false;
}

bool SimpleCamera::isVRCamera() const
{
	return m_data->m_enableVR ;
}


static void    b3CreateFrustum(
                        float left,
                        float right,
                        float bottom,
                        float top,
                        float nearVal,
                        float farVal,
                        float frustum[16])
{

    frustum[0*4+0] = (float(2) * nearVal) / (right - left);
    frustum[0*4+1] = float(0);
    frustum[0*4+2] = float(0);
    frustum[0*4+3] = float(0);

    frustum[1*4+0] = float(0);
    frustum[1*4+1] = (float(2) * nearVal) / (top - bottom);
    frustum[1*4+2] = float(0);
    frustum[1*4+3] = float(0);

    frustum[2*4+0] = (right + left) / (right - left);
    frustum[2*4+1] = (top + bottom) / (top - bottom);
    frustum[2*4+2] = -(farVal + nearVal) / (farVal - nearVal);
    frustum[2*4+3] = float(-1);

    frustum[3*4+0] = float(0);
    frustum[3*4+1] = float(0);
    frustum[3*4+2] = -(float(2) * farVal * nearVal) / (farVal - nearVal);
    frustum[3*4+3] = float(0);

}



#if 0
static void b3CreateDiagonalMatrix(float value, float result[4][4])
{
	for (int i=0;i<4;i++)
	{
		for (int j=0;j<4;j++)
		{
			if (i==j)
			{
				result[i][j] = value;
			} else
			{
				result[i][j] = 0.f;
			}
		}
	}
}
static void b3CreateOrtho(float left, float right, float bottom, float top, float zNear, float zFar, float result[4][4])
{
	b3CreateDiagonalMatrix(1.f,result);

	result[0][0] = 2.f / (right - left);
	result[1][1] = 2.f / (top - bottom);
	result[2][2] = - 2.f / (zFar - zNear);
	result[3][0] = - (right + left) / (right - left);
	result[3][1] = - (top + bottom) / (top - bottom);
	result[3][2] = - (zFar + zNear) / (zFar - zNear);
}
#endif
static void    b3CreateLookAt(const b3Vector3& eye, const b3Vector3& center,const b3Vector3& up, float result[16])
{
    b3Vector3 f = (center - eye).normalized();
    b3Vector3 u = up.normalized();
    b3Vector3 s = (f.cross(u)).normalized();
    u = s.cross(f);

    result[0*4+0] = s.x;
    result[1*4+0] = s.y;
    result[2*4+0] = s.z;

	result[0*4+1] = u.x;
    result[1*4+1] = u.y;
    result[2*4+1] = u.z;

    result[0*4+2] =-f.x;
    result[1*4+2] =-f.y;
    result[2*4+2] =-f.z;

	result[0*4+3] = 0.f;
    result[1*4+3] = 0.f;
    result[2*4+3] = 0.f;

    result[3*4+0] = -s.dot(eye);
    result[3*4+1] = -u.dot(eye);
    result[3*4+2] = f.dot(eye);
    result[3*4+3] = 1.f;
}

void SimpleCamera::setCameraUpAxis(int upAxis)
{
	m_data->m_cameraUpAxis = upAxis;

	update();
}

int		SimpleCamera::getCameraUpAxis() const
{
	return m_data->m_cameraUpAxis;
}

void SimpleCamera::update()
{
	b3Scalar yawRad = m_data->m_yaw * b3Scalar(0.01745329251994329547);// rads per deg
	b3Scalar pitchRad = m_data->m_pitch * b3Scalar(0.01745329251994329547);// rads per deg
	b3Scalar rollRad = 0.0;
	b3Quaternion eyeRot;
	
	int forwardAxis(-1);
	switch (m_data->m_cameraUpAxis)
	{
    case 1:
    	forwardAxis = 2;
    	m_data->m_cameraUp = b3MakeVector3(0,1,0);
    	//gLightPos = b3MakeVector3(-50.f,100,30);
		eyeRot.setEulerZYX(rollRad, yawRad, -pitchRad);
    	break;
    case 2:
		forwardAxis = 1;
		m_data->m_cameraUp = b3MakeVector3(0,0,1);
		//gLightPos = b3MakeVector3(-50.f,30,100);
		eyeRot.setEulerZYX(yawRad, rollRad, pitchRad);
		break;
    default:
		{
			//b3Assert(0);
			return;
		}
	};

	b3Vector3 eyePos = b3MakeVector3(0,0,0);
	eyePos[forwardAxis] = -m_data->m_cameraDistance;

	m_data->m_cameraForward = b3MakeVector3(eyePos[0],eyePos[1],eyePos[2]);
	if (m_data->m_cameraForward.length2() < B3_EPSILON)
	{
		m_data->m_cameraForward.setValue(1.f,0.f,0.f);
	} else
	{
		m_data->m_cameraForward.normalize();
	}
	
	eyePos = b3Matrix3x3(eyeRot)*eyePos;

	m_data->m_cameraPosition = eyePos;
	m_data->m_cameraPosition+= m_data->m_cameraTargetPosition;

}

void SimpleCamera::getCameraProjectionMatrix(float projectionMatrix[16]) const
{
	if (m_data->m_enableVR)
	{
		for (int i=0;i<16;i++)
		{
			projectionMatrix[i] = m_data->m_projectionMatrixVR[i];
		}
	} else
	{
		b3CreateFrustum(-m_data->m_aspect * m_data->m_frustumZNear, m_data->m_aspect * m_data->m_frustumZNear, -m_data->m_frustumZNear,m_data->m_frustumZNear, m_data->m_frustumZNear, m_data->m_frustumZFar,projectionMatrix);
	}
}
void	SimpleCamera::setVRCameraOffsetTransform(const float offset[16])
{
	for (int i=0;i<16;i++)
	{
		m_data->m_offsetTransformVR[i]   = offset[i];
	}
}
void SimpleCamera::getCameraViewMatrix(float viewMatrix[16]) const
{
	if (m_data->m_enableVR)
	{
		for (int i=0;i<16;i++)
		{
			b3Transform tr;
			tr.setFromOpenGLMatrix(m_data->m_viewMatrixVR);
			b3Transform shift=b3Transform::getIdentity();
			shift.setFromOpenGLMatrix(m_data->m_offsetTransformVR);
			tr = tr*shift;
			tr.getOpenGLMatrix(viewMatrix);
			//viewMatrix[i] = m_data->m_viewMatrixVR[i];
		}
	} else
	{
		b3CreateLookAt(m_data->m_cameraPosition,m_data->m_cameraTargetPosition,m_data->m_cameraUp,viewMatrix);
	}
}

void SimpleCamera::getCameraTargetPosition(double pos[3]) const
{
	pos[0] =m_data->m_cameraTargetPosition[0];
	pos[1] =m_data->m_cameraTargetPosition[1];
	pos[2] =m_data->m_cameraTargetPosition[2];
}

void SimpleCamera::getCameraPosition(double pos[3]) const
{
	pos[0] =m_data->m_cameraPosition[0];
	pos[1] =m_data->m_cameraPosition[1];
	pos[2] =m_data->m_cameraPosition[2];
}


void SimpleCamera::getCameraTargetPosition(float pos[3]) const
{
	pos[0] =m_data->m_cameraTargetPosition[0];
	pos[1] =m_data->m_cameraTargetPosition[1];
	pos[2] =m_data->m_cameraTargetPosition[2];
}
void SimpleCamera::getCameraPosition(float pos[3]) const
{
	pos[0] =m_data->m_cameraPosition[0];
	pos[1] =m_data->m_cameraPosition[1];
	pos[2] =m_data->m_cameraPosition[2];
}

void	SimpleCamera::setCameraTargetPosition(float x,float y,float z)
{
	m_data->m_cameraTargetPosition.setValue(x,y,z);
	update();
}
float	SimpleCamera::getCameraDistance() const
{
	return m_data->m_cameraDistance;
}

void	SimpleCamera::setCameraDistance(float dist)
{
	m_data->m_cameraDistance = dist;
	update();
}
void	SimpleCamera::setCameraUpVector(float x,float y ,float z)
{
	m_data->m_cameraUp.setValue(x,y,z);
	update();
}

void	SimpleCamera::getCameraUpVector(float up[3]) const
{
	if (m_data->m_enableVR)
	{
		float viewMatTotal[16];
		getCameraViewMatrix(viewMatTotal);
		up[0] = viewMatTotal[0];
		up[1] = viewMatTotal[4];
		up[2] = viewMatTotal[8];
	} else
	{
		up[0] = float(m_data->m_cameraUp[0]);
		up[1] = float(m_data->m_cameraUp[1]);
		up[2] = float(m_data->m_cameraUp[2]);
	}
}

void	SimpleCamera::getCameraForwardVector(float fwd[3]) const
{
	if (m_data->m_enableVR)
	{
		float viewMatTotal[16];
		getCameraViewMatrix(viewMatTotal);
		fwd[0] = viewMatTotal[2];
		fwd[1] = viewMatTotal[6];
		fwd[2] = viewMatTotal[10];
	} else
	{
		fwd[0] = float(m_data->m_cameraForward[0]);
		fwd[1] = float(m_data->m_cameraForward[1]);
		fwd[2] = float(m_data->m_cameraForward[2]);
	}
}

void	SimpleCamera::setCameraYaw(float yaw)
{
	m_data->m_yaw = yaw;
	update();
}

float	SimpleCamera::getCameraYaw() const
{
	return m_data->m_yaw;
}

void	SimpleCamera::setCameraPitch(float pitch)
{
	m_data->m_pitch = pitch;
	update();
}

void	SimpleCamera::setAspectRatio(float ratio)
{
	m_data->m_aspect = ratio;
	update();
}

float	SimpleCamera::getCameraPitch() const
{
	return m_data->m_pitch;
}
float	SimpleCamera::getAspectRatio() const
{
	return m_data->m_aspect;
}

float SimpleCamera::getCameraFrustumFar() const
{
    return m_data->m_frustumZFar;
}

float SimpleCamera::getCameraFrustumNear() const
{
    return m_data->m_frustumZNear;
}

void SimpleCamera::setCameraFrustumFar(float far)
{
	m_data->m_frustumZFar = far;
}

void SimpleCamera::setCameraFrustumNear(float near)
{
	m_data->m_frustumZNear = near;
}
