
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Stephane Redon / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "BU_Screwing.h"


BU_Screwing::BU_Screwing(const btVector3& relLinVel,const btVector3& relAngVel) {


	const btScalar dx=relLinVel[0];
	const btScalar dy=relLinVel[1];
	const btScalar dz=relLinVel[2];
	const btScalar wx=relAngVel[0];
	const btScalar wy=relAngVel[1];
	const btScalar wz=relAngVel[2];

	// Compute the screwing parameters :
	// w : total amount of rotation
	// s : total amount of translation
	// u : vector along the screwing axis (||u||=1)
	// o : point on the screwing axis
	
	m_w=btSqrt(wx*wx+wy*wy+wz*wz);
	//if (!w) {
	if (fabs(m_w)<SCREWEPSILON ) {

		assert(m_w == 0.f);

		m_w=0.;
		m_s=btSqrt(dx*dx+dy*dy+dz*dz);
		if (fabs(m_s)<SCREWEPSILON ) {
			assert(m_s == 0.);

			m_s=0.;
			m_u=btPoint3(0.,0.,1.);
			m_o=btPoint3(0.,0.,0.);
		}
		else {
			float t=1.f/m_s;
			m_u=btPoint3(dx*t,dy*t,dz*t);
			m_o=btPoint3(0.f,0.f,0.f);
		}
	}
	else { // there is some rotation
		
		// we compute u
		
		float v(1.f/m_w);
		m_u=btPoint3(wx*v,wy*v,wz*v); // normalization
		
		// decomposition of the translation along u and one orthogonal vector
		
		btPoint3 t(dx,dy,dz);
		m_s=t.dot(m_u); // component along u
		if (fabs(m_s)<SCREWEPSILON)
		{
			//printf("m_s component along u < SCREWEPSILION\n");
			m_s=0.f;
		}
		btPoint3 n1(t-(m_s*m_u)); // the remaining part (which is orthogonal to u)
		
		// now we have to compute o
		
		//btScalar len = n1.length2();
		//(len >= BUM_EPSILON2) {
		if (n1[0] || n1[1] || n1[2]) { // n1 is not the zero vector
			n1.normalize();
			btVector3 n1orth=m_u.cross(n1);

			float n2x=btCos(0.5f*m_w);
			float n2y=btSin(0.5f*m_w);
			
			m_o=0.5f*t.dot(n1)*(n1+n2x/n2y*n1orth);
		}
		else 
		{
			m_o=btPoint3(0.f,0.f,0.f);
		}
		
	}
	
}

//Then, I need to compute Pa, the matrix from the reference (global) frame to
//the screwing frame :


void BU_Screwing::LocalMatrix(btTransform &t) const {
//So the whole computations do this : align the Oz axis along the
//	screwing axis (thanks to u), and then find two others orthogonal axes to
//	complete the basis.
		
		if ((m_u[0]>SCREWEPSILON)||(m_u[0]<-SCREWEPSILON)||(m_u[1]>SCREWEPSILON)||(m_u[1]<-SCREWEPSILON)) 
		{ 
			// to avoid numerical problems
			float n=btSqrt(m_u[0]*m_u[0]+m_u[1]*m_u[1]);
			float invn=1.0f/n;
			btMatrix3x3 mat;

	  		mat[0][0]=-m_u[1]*invn;
			mat[0][1]=m_u[0]*invn;
			mat[0][2]=0.f;
	
			mat[1][0]=-m_u[0]*invn*m_u[2];
			mat[1][1]=-m_u[1]*invn*m_u[2];
			mat[1][2]=n;

			mat[2][0]=m_u[0];
			mat[2][1]=m_u[1];
			mat[2][2]=m_u[2];
			
			t.setOrigin(btPoint3(
				  m_o[0]*m_u[1]*invn-m_o[1]*m_u[0]*invn,
				-(m_o[0]*mat[1][0]+m_o[1]*mat[1][1]+m_o[2]*n),
				-(m_o[0]*m_u[0]+m_o[1]*m_u[1]+m_o[2]*m_u[2])));

			t.setBasis(mat);

		}
		else {

			btMatrix3x3 m;

			m[0][0]=1.;
			m[1][0]=0.;
			m[2][0]=0.;

			m[0][1]=0.f;
			m[1][1]=float(btSign(m_u[2]));
			m[2][1]=0.f;

			m[0][2]=0.f;
			m[1][2]=0.f;
			m[2][2]=float(btSign(m_u[2]));

			t.setOrigin(btPoint3(
					-m_o[0],
					-btSign(m_u[2])*m_o[1],
					-btSign(m_u[2])*m_o[2]
				));
			t.setBasis(m);

		}
}

//gives interpolated transform for time in [0..1] in screwing frame
btTransform	BU_Screwing::InBetweenTransform(const btTransform& tr,btScalar t) const
{
	btPoint3 org = tr.getOrigin();

	btPoint3 neworg (
	org.x()*btCos(m_w*t)-org.y()*btSin(m_w*t),
	org.x()*btSin(m_w*t)+org.y()*btCos(m_w*t),
	org.z()+m_s*CalculateF(t));
		
	btTransform newtr;
	newtr.setOrigin(neworg);
	btMatrix3x3 basis = tr.getBasis();
	btMatrix3x3 basisorg = tr.getBasis();

	btQuaternion rot(btVector3(0.,0.,1.),m_w*t);
	btQuaternion tmpOrn;
	tr.getBasis().getRotation(tmpOrn);
	rot = rot *  tmpOrn;

	//to avoid numerical drift, normalize quaternion
	rot.normalize();
	newtr.setBasis(btMatrix3x3(rot));
	return newtr;

}


btScalar BU_Screwing::CalculateF(btScalar t) const
{
	btScalar result;
	if (!m_w)
	{
		result = t;
	} else
	{
		result = ( btTan((m_w*t)/2.f) / btTan(m_w/2.f));
	}
	return result;
}
	
