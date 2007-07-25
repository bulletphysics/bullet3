
///Testfile to test differences between vectormath and Bullet LinearMath

#ifdef __PPU__
#include "include/vectormath/ppu/cpp/vectormath_aos.h"
#elif defined __SPU__
#include "include/vectormath/spu/cpp/vectormath_aos.h"
#else
#include "include/vectormath/SSE/cpp/vectormath_aos.h"
//#include "include/vectormath/scalar/cpp/vectormath_aos.h"
#endif

#include "../../src/LinearMath/btTransform.h"
#include <stdio.h>

//Bullet, a btVector can be used for both points and vectors. 
//it is up to the user/developer to use the right multiplication: btTransform for points, and btQuaternion or btMatrix3x3 for vectors.
void	BulletTest()
{

	printf("Bullet Linearmath\n");

	btTransform	tr;
	tr.setIdentity();

	tr.setOrigin(btVector3(10,0,0));
	//initialization
	btVector3	pointA(0,0,0);
	btVector3	pointB,pointC,pointD,pointE;
	//assignment
	pointB = pointA;
	//in-place initialization
	pointB.setValue(1,2,3);
	//transform over tr	
	pointB = tr * pointA;
	printf("pointB = tr * pointA = (%f,%f,%f)\n",pointB.getX(),pointB.getY(),pointB.getZ());
	//transform over tr	
	pointE = tr(pointA);
	//inverse transform
	pointC = tr.inverse() * pointA;
	printf("pointC = tr.inverse() * pointA = (%f,%f,%f)\n",pointC.getX(),pointC.getY(),pointC.getZ());
	//inverse transform
	pointD = tr.invXform( pointA );
	btScalar	x;
	//dot product
	x = pointD.dot(pointE);
	//square length
	x = pointD.length2();
	//length
	x = pointD.length();

	const btVector3& constPointD = pointD;

	//get a normalized vector from constPointD, without changing constPointD
	btVector3 norm = constPointD.normalized();

	//in-place normalize pointD
	pointD.normalize();

	//quaternions & matrices
	btQuaternion	quat(0,0,0,1);
	btQuaternion	quat1(btVector3(0,1,0),90.f * SIMD_RADS_PER_DEG);
	btMatrix3x3		mat0(quat1);
	btMatrix3x3		mat1 = mat0.inverse();
	btMatrix3x3		mat2  = mat0.transpose();
	btTransform tr1(mat2,btVector3(0,10,0));
	btTransform tr2 =tr1.inverse();
	btVector3	pt0(1,1,1);
	btVector3	pt1 = tr2 * pt0;

	printf("btVector3	pt1 = tr2 * pt0 =  (%f,%f,%f)\n",pt1.getX(),pt1.getY(),pt1.getZ());

	
	btVector3	pt2 = tr2.getBasis() * pt0;
	btVector3	pt3 = pt0 * tr2.getBasis();
	btVector3	pt4 =  tr2.getBasis().inverse() * pt0;
	btTransform tr3 =  tr2.inverseTimes(tr2);



}

//vectormath makes a difference between point and vector.
void	VectormathTest()
{

	printf("Vectormath\n");

	Vectormath::Aos::Transform3 tr;
	tr = Vectormath::Aos::Transform3::identity();

	tr.setTranslation(Vectormath::Aos::Vector3(10,0,0));
	//initialization
	Vectormath::Aos::Point3	pointA(0,0,0);
	Vectormath::Aos::Point3	pointB,pointC,pointE;
	Vectormath::Aos::Vector3 pointD;
	//assignment
	pointB = pointA;
	//in-place initialization
	pointB = Vectormath::Aos::Point3(1,2,3); //or
	pointB.setElem(0,1); //or
	pointB.setX(1);

	//transform over tr	
	pointB = tr * pointA;

	printf("pointB = tr * pointA = (%f,%f,%f)\n",(float)pointB.getX(),(float)pointB.getY(),(float)pointB.getZ());
	//transform over tr	
	//pointE = tr(pointA);
	//inverse transform
	pointC = Vectormath::Aos::inverse(tr) * pointA;
	printf("Vectormath::Aos::inverse(tr) * pointA = (%f,%f,%f)\n",(float)pointC.getX(),(float)pointC.getY(),(float)pointC.getZ());
	
	
	
	btScalar	x;
	//dot product
	x = Vectormath::Aos::dot(Vectormath::Aos::Vector3(pointD),Vectormath::Aos::Vector3(pointE));
	//square length
	x = Vectormath::Aos::lengthSqr(Vectormath::Aos::Vector3(pointD));
	//length
	x = Vectormath::Aos::length(Vectormath::Aos::Vector3(pointD));

	const Vectormath::Aos::Vector3& constPointD = (Vectormath::Aos::Vector3&)pointD;

	//get a normalized vector from constPointD, without changing constPointD
	Vectormath::Aos::Vector3 norm = Vectormath::Aos::normalize(constPointD);

	//in-place normalize pointD
	pointD = Vectormath::Aos::normalize(Vectormath::Aos::Vector3(pointD));

	//quaternions & matrices
	Vectormath::Aos::Quat quat(0,0,0,1);
	Vectormath::Aos::Quat quat1;
	quat1 = Vectormath::Aos::Quat::rotationY(90.f * SIMD_RADS_PER_DEG);
	
	Vectormath::Aos::Matrix3	mat0(quat1);
	
	Vectormath::Aos::Matrix3	mat1 = Vectormath::Aos::inverse(mat0);
	Vectormath::Aos::Matrix3	mat2  = Vectormath::Aos::transpose(mat0);
	Vectormath::Aos::Transform3 tr1(mat2,Vectormath::Aos::Vector3(0,10,0));
	Vectormath::Aos::Transform3	tr2 = Vectormath::Aos::inverse(tr1);
	Vectormath::Aos::Point3	pt0(1,1,1);
	Vectormath::Aos::Point3	pt1 = tr2 * pt0;
	printf("Vectormath::Aos::Vector3	pt1 = tr2 * pt0; =  (%f,%f,%f)\n",(float)pt1.getX(),(float)pt1.getY(),(float)pt1.getZ());

	Vectormath::Aos::Vector3	pt2 = tr2.getUpper3x3() * Vectormath::Aos::Vector3(pt0);
	//Vectormath::Aos::Vector3	pt3 = pt0 * tr2.getUpper3x3();
	Vectormath::Aos::Vector3	pt3 = Vectormath::Aos::inverse(tr2.getUpper3x3()) * Vectormath::Aos::Vector3(pt0);
	Vectormath::Aos::Vector3	pt4 =  Vectormath::Aos::inverse(tr2.getUpper3x3()) * Vectormath::Aos::Vector3(pt0);
	Vectormath::Aos::Transform3		tr3 =  Vectormath::Aos::inverse(tr2) * tr2;

}

int main()
{

	BulletTest();

	VectormathTest();
	
	return 0;
}
