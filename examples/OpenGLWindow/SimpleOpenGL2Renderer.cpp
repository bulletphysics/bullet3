
#include "SimpleOpenGL2Renderer.h"
#include "OpenGL2Include.h"
#include "Bullet3Common/b3Vector3.h"


SimpleOpenGL2Renderer::SimpleOpenGL2Renderer(int width, int height)
 :m_width(width),
 m_height(height)
{
    
}

void SimpleOpenGL2Renderer::init()
{
}

const CommonCameraInterface* SimpleOpenGL2Renderer::getActiveCamera() const
{
	return &m_camera;
}
CommonCameraInterface* SimpleOpenGL2Renderer::getActiveCamera()
{
	return &m_camera;
}
void SimpleOpenGL2Renderer::setActiveCamera(CommonCameraInterface* cam)
{
	b3Assert(0);//not supported yet
}

void SimpleOpenGL2Renderer::updateCamera(int upAxis)
{
    float projection[16];
    float view[16];
    m_camera.setAspectRatio((float)m_width/(float)m_height);
	m_camera.setCameraUpAxis(upAxis);
    m_camera.update();
    m_camera.getCameraProjectionMatrix(projection);
    m_camera.getCameraViewMatrix(view);
    GLfloat projMat[16];
    GLfloat viewMat[16];
    for (int i=0;i<16;i++)
    {
        viewMat[i] = view[i];
        projMat[i] = projection[i];
    }
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMultMatrixf(projMat);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glMultMatrixf(viewMat);
}

void SimpleOpenGL2Renderer::removeAllInstances()
{
}


void SimpleOpenGL2Renderer::writeSingleInstanceColorToCPU(float* color, int srcIndex)
{
}
void SimpleOpenGL2Renderer::writeSingleInstanceColorToCPU(double* color, int srcIndex)
{
    
}

void SimpleOpenGL2Renderer::writeSingleInstanceScaleToCPU(float* scale, int srcIndex)
{
}
void SimpleOpenGL2Renderer::writeSingleInstanceScaleToCPU(double* scale, int srcIndex)
{
}


int SimpleOpenGL2Renderer::getTotalNumInstances() const
{
    return 0;
}

void	SimpleOpenGL2Renderer::getCameraViewMatrix(float viewMat[16]) const
{
    b3Assert(0);
}
void	SimpleOpenGL2Renderer::getCameraProjectionMatrix(float projMat[16]) const
{
    b3Assert(0);
    
}


void SimpleOpenGL2Renderer::renderScene()
{
}
    
    


int SimpleOpenGL2Renderer::registerGraphicsInstance(int shapeIndex, const double* position, const double* quaternion, const double* color, const double* scaling)
{
    return 0;
}

int SimpleOpenGL2Renderer::registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling)
{
    return 0;
}

void SimpleOpenGL2Renderer::drawLines(const float* positions, const float color[4], int numPoints, int pointStrideInBytes, const unsigned int* indices, int numIndices, float pointDrawSize)
{
    int pointStrideInFloats = pointStrideInBytes/4;
    glLineWidth(pointDrawSize);
    for (int i=0;i<numIndices;i+=2)
    {
        int index0 = indices[i];
        int index1 = indices[i+1];
        
        b3Vector3 fromColor = b3MakeVector3(color[0],color[1],color[2]);
        b3Vector3 toColor = b3MakeVector3(color[0],color[1],color[2]);
        
        b3Vector3 from= b3MakeVector3(positions[index0*pointStrideInFloats],positions[index0*pointStrideInFloats+1],positions[index0*pointStrideInFloats+2]);
        b3Vector3 to= b3MakeVector3(positions[index1*pointStrideInFloats],positions[index1*pointStrideInFloats+1],positions[index1*pointStrideInFloats+2]);
        
        glBegin(GL_LINES);
        glColor3f(fromColor.getX(), fromColor.getY(), fromColor.getZ());
        glVertex3d(from.getX(), from.getY(), from.getZ());
        glColor3f(toColor.getX(), toColor.getY(), toColor.getZ());
        glVertex3d(to.getX(), to.getY(), to.getZ());
        glEnd();
        
    }
}

void SimpleOpenGL2Renderer::drawLine(const float from[4], const float to[4], const float color[4], float lineWidth)
{
        glLineWidth(lineWidth);
        glBegin(GL_LINES);
        glColor3f(color[0],color[1],color[2]);
        glVertex3d(from[0],from[1],from[2]);
        glVertex3d(to[0],to[1],to[2]);
        glEnd();
}
int SimpleOpenGL2Renderer::registerShape(const float* vertices, int numvertices, const int* indices, int numIndices,int primitiveType, int textureIndex)
{
    return 0;
}

void SimpleOpenGL2Renderer::writeSingleInstanceTransformToCPU(const float* position, const float* orientation, int srcIndex)
{
}
void SimpleOpenGL2Renderer::writeSingleInstanceTransformToCPU(const double* position, const double* orientation, int srcIndex)
{
}
void SimpleOpenGL2Renderer::writeTransforms()
{
}


void SimpleOpenGL2Renderer::drawLine(const double from[4], const double to[4], const double color[4], double lineWidth)
{
    
}
void SimpleOpenGL2Renderer::drawPoint(const float* position, const float color[4], float pointDrawSize)
{
}
void SimpleOpenGL2Renderer::drawPoint(const double* position, const double color[4], double pointDrawSize)
{
}

void SimpleOpenGL2Renderer::updateShape(int shapeIndex, const float* vertices)
{
}

void SimpleOpenGL2Renderer::enableBlend(bool blend)
{
}

void SimpleOpenGL2Renderer::clearZBuffer()
{
	glClear(GL_DEPTH_BUFFER_BIT);
}