#include <cmath>
#include <limits>
#include <cstdlib>
#include "our_gl.h"
#include "Bullet3Common/b3MinMax.h"

IShader::~IShader() {}

Matrix viewport(int x, int y, int w, int h) 
{
    Matrix Viewport;
    Viewport = Matrix::identity();
    Viewport[0][3] = x+w/2.f;
    Viewport[1][3] = y+h/2.f;
    Viewport[2][3] = .5f;
    Viewport[0][0] = w/2.f;
    Viewport[1][1] = h/2.f;
    Viewport[2][2] = .5f;
    return Viewport;
}

Matrix projection(float coeff) {
    Matrix Projection;
    Projection = Matrix::identity();
    Projection[3][2] = coeff;
    return Projection;
}

Matrix lookat(Vec3f eye, Vec3f center, Vec3f up) {
    Vec3f f = (center - eye).normalize();
    Vec3f u = up.normalize();
    Vec3f s = cross(f,u).normalize();
    u = cross(s,f);
    
    Matrix ModelView;
    ModelView[0][0] = s.x;
    ModelView[0][1] = s.y;
    ModelView[0][2] = s.z;
    
    ModelView[1][0] = u.x;
    ModelView[1][1] = u.y;
    ModelView[1][2] = u.z;
    
    ModelView[2][0] =-f.x;
    ModelView[2][1] =-f.y;
    ModelView[2][2] =-f.z;
    
    ModelView[3][0] = 0.f;
    ModelView[3][1] = 0.f;
    ModelView[3][2] = 0.f;
    
    ModelView[0][3] = -(s[0]*eye[0]+s[1]*eye[1]+s[2]*eye[2]);
    ModelView[1][3] = -(u[0]*eye[0]+u[1]*eye[1]+u[2]*eye[2]);
    ModelView[2][3] = f[0]*eye[0]+f[1]*eye[1]+f[2]*eye[2];
    ModelView[3][3] = 1.f;
    
    return ModelView;
}

Vec3f barycentric(Vec2f A, Vec2f B, Vec2f C, Vec2f P) {
    Vec3f s[2];
    for (int i=2; i--; ) {
        s[i][0] = C[i]-A[i];
        s[i][1] = B[i]-A[i];
        s[i][2] = A[i]-P[i];
    }
    Vec3f u = cross(s[0], s[1]);
    if (std::abs(u[2])>1e-2) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
        return Vec3f(1.f-(u.x+u.y)/u.z, u.y/u.z, u.x/u.z);
    return Vec3f(-1,1,1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
}

void triangle(mat<4,3,float> &clipc, IShader &shader, TGAImage &image, float *zbuffer, const Matrix& viewPortMatrix) 
{
    triangle(clipc,shader,image,zbuffer,0,viewPortMatrix,0);
}

void triangle(mat<4,3,float> &clipc, IShader &shader, TGAImage &image, float *zbuffer, int* segmentationMaskBuffer, const Matrix& viewPortMatrix, int objectIndex) {
	mat<3,4,float> pts  = (viewPortMatrix*clipc).transpose(); // transposed to ease access to each of the points
    
	//we don't clip triangles that cross the near plane, just discard them instead of showing artifacts
	if (pts[0][3]<0 || pts[1][3] <0 || pts[2][3] <0)
		return;

	mat<3,2,float> pts2;
    for (int i=0; i<3; i++) pts2[i] = proj<2>(pts[i]/pts[i][3]);

    Vec2f bboxmin( std::numeric_limits<float>::max(),  std::numeric_limits<float>::max());
    Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    Vec2f clamp(image.get_width()-1, image.get_height()-1);
	
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            bboxmin[j] = b3Max(0.f,      b3Min(bboxmin[j], pts2[i][j]));
            bboxmax[j] = b3Min(clamp[j], b3Max(bboxmax[j], pts2[i][j]));
        }
    }

	Vec2i P;
    TGAColor color;
    for (P.x=bboxmin.x; P.x<=bboxmax.x; P.x++) {
        for (P.y=bboxmin.y; P.y<=bboxmax.y; P.y++) {
            Vec3f bc_screen  = barycentric(pts2[0], pts2[1], pts2[2], P);
            Vec3f bc_clip    = Vec3f(bc_screen.x/pts[0][3], bc_screen.y/pts[1][3], bc_screen.z/pts[2][3]);
            bc_clip = bc_clip/(bc_clip.x+bc_clip.y+bc_clip.z);
            float frag_depth = -1*(clipc[2]*bc_clip);
            if (bc_screen.x<0 || bc_screen.y<0 || bc_screen.z<0 ||
				zbuffer[P.x+P.y*image.get_width()]>frag_depth) 
				continue;
            bool discard = shader.fragment(bc_clip, color);
            if (!discard) {
                zbuffer[P.x+P.y*image.get_width()] = frag_depth;
                if (segmentationMaskBuffer)
                {
                    segmentationMaskBuffer[P.x+P.y*image.get_width()] = objectIndex;
                }
                image.set(P.x, P.y, color);
            }
        }
    }
}

