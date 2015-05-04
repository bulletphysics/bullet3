#include "vec3n.h"
//#include "console.h"


extern int numX;


//
// Cloth - Backward Integrated Spring Network
//
// (c) Stan Melax 2006
// http://www.melax.com/cloth
// freeware demo and source
// Although its free software, I'll gaurantee and support this software as much as is reasonable.
// However, if you choose to use any of this code, then you agree that
// I assume no financial liability should the software not meet your expectations.
// But do feel free to send any feedback.
//
// The core backward integration functionality has all been extracted into the SpringNetwork class.
// This makes it easy for you if you just want to look at or use the math and the algorithms.
// The remainder of the code builds a cloth system with basic render support, I/O, and manipulators,
// so its possible to make use of the technology within a 3D application.
// This code is separated from the SpringNetwork class in order to avoid pushing a particular style
// and prevent any dependancies of the algorithms onto unrelated systems.
// Feel free to adapt any of this into your own 3D engine/environment.
//
// Instead of having unique Hooke force and damping coefficients on each spring, the SpringNetwork
// code uses a spring 'type' that indexes a short list of shared named coefficients.
// This was just more practical for the typical application of this technology.
// Over-designed systems that are too general can be slower for
// the next guy to understand and more painful to use.
// Editing/creation is easier when only 1 number needs to be changed.
// Nonetheless, feel free to adapt to your own needs.
//

#include <stdio.h>
#include <float.h>

#include "vec3n.h"
//#include "console.h"
//#include "manipulatori.h"
//#include "object.h"
//#include "xmlparse.h"




static const float3x3 I(1,0,0,0,1,0,0,0,1);

inline float3x3 dfdx_spring(const float3 &dir,float length,float rest,float k)
{
       // dir is unit length direction, rest is spring's restlength, k is spring constant.
       return  ( (I-outerprod(dir,dir))*Min(1.0f,rest/length) - I) * -k;
}
inline float3x3 dfdx_damp(const float3 &dir,float length,const float3& vel,float rest,float damping)
{
       // inner spring damping   vel is the relative velocity  of the endpoints.
       return (I-outerprod(dir,dir)) * (-damping * -(dot(dir,vel)/Max(length,rest)));
}
inline float3x3 dfdv_damp(const float3 &dir,float damping)
{
       // derivative of force wrt velocity.
       return outerprod(dir,dir) * damping;
}



#include "SpringNetwork.h"


SpringNetwork::SpringNetwork(int _n):X(_n),V(_n),F(_n),dV(_n),A(_n),dFdX(_n),dFdV(_n)
{
       assert(SPRING_STRUCT==0);
       assert(&spring_shear == &spring_struct +SPRING_SHEAR);
       assert(&spring_bend  == &spring_struct +SPRING_BEND);
       assert(&spring_struct== &spring_k[SPRING_STRUCT]);
       assert(&spring_shear == &spring_k[SPRING_SHEAR ]);
       assert(&spring_bend  == &spring_k[SPRING_BEND  ]);
    //   spring_struct=1000000.0f;
     //  spring_shear=1000000.0f;
    
	   spring_struct=1000.0f;
	   spring_shear=100.0f;
    
		spring_bend=25.0f;
       spring_damp=5.0f;
       spring_air=1.0f;
       spring_air=1.0f;
       cloth_step = 0.25f;  // delta time for cloth
       cloth_gravity=float3(0,-10,0);
       cloth_sleepthreshold = 0.001f;
       cloth_sleepcount = 100;
       awake = cloth_sleepcount;
	   
	   //fix/pin two points in worldspace
	   float3Nx3N::Block zero;
	   zero.m = float3x3(0,0,0,0,0,0,0,0,0);
	   zero.c = 0;
	   zero.r = 0;
		S.blocks.Add(zero);
		zero.r = numX-1;
		S.blocks.Add(zero);



}


SpringNetwork::Spring &SpringNetwork::AddBlocks(Spring &s)
{
       // Called during initial creation of springs in our spring network.
       // Sets up the sparse matrices corresponding to connections.
       // Note the indices (s.iab,s.iba) are also stored with spring to avoid looking them up each time a spring is applied
       // All 3 matrices A,dFdX, and dFdV are contstructed identically so the block array layout will be the same for each.
       s.iab = A.blocks.count;   // added 'ab' blocks will have this index.
       A.blocks.Add(float3Nx3N::Block(s.a,s.b));
       dFdX.blocks.Add(float3Nx3N::Block(s.a,s.b));
       dFdV.blocks.Add(float3Nx3N::Block(s.a,s.b));
       s.iba = A.blocks.count;   // added 'ba' blocks will have this index.
       A.blocks.Add(float3Nx3N::Block(s.b,s.a));
       dFdX.blocks.Add(float3Nx3N::Block(s.b,s.a));
       dFdV.blocks.Add(float3Nx3N::Block(s.b,s.a));
       return s;
}

void SpringNetwork::PreSolveSpring(const SpringNetwork::Spring &s)
{
       // Adds this spring's contribution into force vector F and force derivitves dFdX and dFdV
       // One optimization would be premultiply dfdx by dt*dt and F and dFdV by dt right here in this function.
       // However, for educational purposes we wont do that now and intead just follow the paper directly.
       //assert(dFdX.blocks[s.a].c==s.a);  // delete this assert, no bugs here
       //assert(dFdX.blocks[s.a].r==s.a);
       float3 extent = X[s.b] - X[s.a];
       float  length = magnitude(extent);
       float3 dir    = (length==0)?float3(0,0,0): extent * 1.0f/length;
       float3 vel    = V[s.b] - V[s.a];
       float  k      = spring_k[s.type];
       float3 f =  dir * ((k * (length-s.restlen) ) +  spring_damp * dot(vel,dir));  // spring force + damping force
       F[s.a] += f;
       F[s.b] -= f;
       float3x3 dfdx = dfdx_spring(dir,length,s.restlen,k) + dfdx_damp(dir,length,vel,s.restlen,spring_damp);
       dFdX.blocks[s.a].m   -= dfdx;  // diagonal chunk dFdX[a,a]
       dFdX.blocks[s.b].m   -= dfdx;  // diagonal chunk dFdX[b,b]
       dFdX.blocks[s.iab].m += dfdx;  // off-diag chunk dFdX[a,b]
       dFdX.blocks[s.iba].m += dfdx;  // off-diag chunk dFdX[b,a]
       float3x3 dfdv = dfdv_damp(dir,spring_damp);
       dFdV.blocks[s.a].m   -= dfdv;  // diagonal chunk dFdV[a,a]
       dFdV.blocks[s.b].m   -= dfdv;  // diagonal chunk dFdV[b,b]
       dFdV.blocks[s.iab].m += dfdv;  // off-diag chunk dFdV[a,b]
       dFdV.blocks[s.iba].m += dfdv;  // off-diag chunk dFdV[b,a]
}




void SpringNetwork::CalcForces()
{
       // Collect forces and derivatives:  F,dFdX,dFdV
       dFdX.Zero();
       dFdV.InitDiagonal(-spring_air);
       F.Init(cloth_gravity);

		F.element[0]=float3(0,0,0);
		F.element[numX-1]=float3(0,0,0);
	   
       F -= V * spring_air;
       for(int i=0;i<springs.count;i++)
       {
               PreSolveSpring(springs[i]); // will add to F,dFdX,dFdV
       }
}

void SpringNetwork::Simulate(float dt)
{
       // Get ready for conjugate gradient iterative solver step.
       // Initialize operands.
       if(!awake) return;
       CalcForces();
       int n=X.count;  // all our big vectors are of this size
       float3N dFdXmV(n);  // temp to store result of matrix multiply
       float3N B(n);
       dV.Zero();
       A.Identity();  // build up the big matrix we feed to solver
       A -= dFdV * dt +  dFdX * (dt*dt) ;
	   
       dFdXmV = dFdX * V;
       B = F * dt + dFdXmV * (dt*dt);
	   
       ConjGradientFiltered(dV,A,B,S);
       V = V + dV;
//	   V.element[0] = float3(0,0,0);
//	   V.element[numX-1] = float3(0,0,0);

       X = X + V*dt;

       UpdateLimits();
       awake = (dot(V,V)<cloth_sleepthreshold)?awake-1:awake=cloth_sleepcount;
}
