/*
  FLUIDS v.1 - SPH Fluid Simulator for CPU and GPU
  Copyright (C) 2008. Rama Hoetzlein, http://www.rchoetzlein.com

  ZLib license
  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/



#include <conio.h>

#ifdef _MSC_VER
	#include <gl/glut.h>
#else
	#include <GL/glut.h>
#endif

#include "common_defs.h"
#include "mtime.h"
#include "fluid_system.h"

#ifdef BUILD_CUDA
	#include "fluid_system_host.cuh"
#endif

#define EPSILON			0.00001f			//for collision detection

FluidSystem::FluidSystem ()
{
}

void FluidSystem::Initialize ( int mode, int total )
{
	if ( mode != BFLUID ) {
		printf ( "ERROR: FluidSystem not initialized as BFLUID.\n");
	}
	PointSet::Initialize ( mode, total );
	
	FreeBuffers ();
	AddBuffer ( BFLUID, sizeof ( Fluid ), total );
	AddAttribute ( 0, "pos", sizeof ( Vector3DF ), false );	
	AddAttribute ( 0, "color", sizeof ( DWORD ), false );
	AddAttribute ( 0, "vel", sizeof ( Vector3DF ), false );
	AddAttribute ( 0, "ndx", sizeof ( unsigned short ), false );
	AddAttribute ( 0, "age", sizeof ( unsigned short ), false );

	AddAttribute ( 0, "pressure", sizeof ( double ), false );
	AddAttribute ( 0, "density", sizeof ( double ), false );
	AddAttribute ( 0, "sph_force", sizeof ( Vector3DF ), false );
	AddAttribute ( 0, "next", sizeof ( Fluid* ), false );
	AddAttribute ( 0, "tag", sizeof ( bool ), false );		
		
	SPH_Setup ();
	Reset ( total );	
}

void FluidSystem::Reset ( int nmax )
{
	ResetBuffer ( 0, nmax );

	m_DT = 0.003; //  0.001;			// .001 = for point grav

	// Reset parameters
	m_Param [ MAX_FRAC ] = 1.0;
	m_Param [ POINT_GRAV ] = 0.0;
	m_Param [ PLANE_GRAV ] = 1.0;

	m_Param [ BOUND_ZMIN_SLOPE ] = 0.0;
	m_Param [ FORCE_XMAX_SIN ] = 0.0;
	m_Param [ FORCE_XMIN_SIN ] = 0.0;	
	m_Toggle [ WRAP_X ] = false;
	m_Toggle [ WALL_BARRIER ] = false;
	m_Toggle [ LEVY_BARRIER ] = false;
	m_Toggle [ DRAIN_BARRIER ] = false;
	m_Param [ SPH_INTSTIFF ] = 1.00;
	m_Param [ SPH_VISC ] = 0.2;
	m_Param [ SPH_INTSTIFF ] = 0.50;
	m_Param [ SPH_EXTSTIFF ] = 20000;
	m_Param [ SPH_SMOOTHRADIUS ] = 0.01;
	
	m_Vec [ POINT_GRAV_POS ].Set ( 0, 0, 50 );
	m_Vec [ PLANE_GRAV_DIR ].Set ( 0, 0, -9.8 );
	m_Vec [ EMIT_POS ].Set ( 0, 0, 0 );
	m_Vec [ EMIT_RATE ].Set ( 0, 0, 0 );
	m_Vec [ EMIT_ANG ].Set ( 0, 90, 1.0 );
	m_Vec [ EMIT_DANG ].Set ( 0, 0, 0 );
}

int FluidSystem::AddPoint ()
{
	xref ndx;	
	Fluid* f = (Fluid*) AddElem ( 0, ndx );	
	f->sph_force.Set(0,0,0);
	f->vel.Set(0,0,0);
	f->vel_eval.Set(0,0,0);
	f->next = 0x0;
	f->pressure = 0;
	f->density = 0;
	return ndx;
}

int FluidSystem::AddPointReuse ()
{
	xref ndx;	
	Fluid* f;
	if ( NumPoints() <= mBuf[0].max-2 )
		f = (Fluid*) AddElem ( 0, ndx );
	else
		f = (Fluid*) RandomElem ( 0, ndx );

	f->sph_force.Set(0,0,0);
	f->vel.Set(0,0,0);
	f->vel_eval.Set(0,0,0);
	f->next = 0x0;
	f->pressure = 0;
	f->density = 0;
	return ndx;
}

void FluidSystem::Run ()
{
	bool bTiming = false;//true;

	mint::Time start, stop;
	
	float ss = m_Param [ SPH_PDIST ] / m_Param[ SPH_SIMSCALE ];		// simulation scale (not Schutzstaffel)

	if ( m_Vec[EMIT_RATE].x > 0 && (++m_Frame) % (int) m_Vec[EMIT_RATE].x == 0 ) {
		//m_Frame = 0;
		Emit ( ss ); 
	}
	
	#ifdef NOGRID
		// Slow method - O(n^2)
		SPH_ComputePressureSlow ();
		SPH_ComputeForceSlow ();
	#else

		if ( m_Toggle[USE_CUDA] ) {
			
			#ifdef BUILD_CUDA
				// -- GPU --
				start.SetSystemTime ( ACC_NSEC );		
				TransferToCUDA ( mBuf[0].data, (int*) &m_Grid[0], NumPoints() );
				if ( bTiming) { stop.SetSystemTime ( ACC_NSEC ); stop = stop - start; printf ( "TO: %s\n", stop.GetReadableTime().c_str() ); }
			
				start.SetSystemTime ( ACC_NSEC );		
				Grid_InsertParticlesCUDA ();
				if ( bTiming) { stop.SetSystemTime ( ACC_NSEC ); stop = stop - start; printf ( "INSERT (CUDA): %s\n", stop.GetReadableTime().c_str() ); }

				start.SetSystemTime ( ACC_NSEC );
				SPH_ComputePressureCUDA ();
				if ( bTiming) { stop.SetSystemTime ( ACC_NSEC ); stop = stop - start; printf ( "PRESS (CUDA): %s\n", stop.GetReadableTime().c_str() ); }

				start.SetSystemTime ( ACC_NSEC );
				SPH_ComputeForceCUDA (); 
				if ( bTiming) { stop.SetSystemTime ( ACC_NSEC ); stop = stop - start; printf ( "FORCE (CUDA): %s\n", stop.GetReadableTime().c_str() ); }

				//** CUDA integrator is incomplete..
				// Once integrator is done, we can remove TransferTo/From steps
				/*start.SetSystemTime ( ACC_NSEC );
				SPH_AdvanceCUDA( m_DT, m_DT/m_Param[SPH_SIMSCALE] );
				if ( bTiming) { stop.SetSystemTime ( ACC_NSEC ); stop = stop - start; printf ( "ADV (CUDA): %s\n", stop.GetReadableTime().c_str() ); }*/

				start.SetSystemTime ( ACC_NSEC );		
				TransferFromCUDA ( mBuf[0].data, (int*) &m_Grid[0], NumPoints() );
				if ( bTiming) { stop.SetSystemTime ( ACC_NSEC ); stop = stop - start; printf ( "FROM: %s\n", stop.GetReadableTime().c_str() ); }

				// .. Do advance on CPU 
				Advance();

			#endif
			
		} else {
			// -- CPU only --

			start.SetSystemTime ( ACC_NSEC );
			Grid_InsertParticles ();
			if ( bTiming) { stop.SetSystemTime ( ACC_NSEC ); stop = stop - start; printf ( "INSERT: %s\n", stop.GetReadableTime().c_str() ); }
		
			start.SetSystemTime ( ACC_NSEC );
			SPH_ComputePressureGrid ();
			if ( bTiming) { stop.SetSystemTime ( ACC_NSEC ); stop = stop - start; printf ( "PRESS: %s\n", stop.GetReadableTime().c_str() ); }

			start.SetSystemTime ( ACC_NSEC );
			SPH_ComputeForceGridNC ();		
			if ( bTiming) { stop.SetSystemTime ( ACC_NSEC ); stop = stop - start; printf ( "FORCE: %s\n", stop.GetReadableTime().c_str() ); }

			start.SetSystemTime ( ACC_NSEC );
			Advance();
			if ( bTiming) { stop.SetSystemTime ( ACC_NSEC ); stop = stop - start; printf ( "ADV: %s\n", stop.GetReadableTime().c_str() ); }
		}		
		
	#endif
}



void FluidSystem::SPH_DrawDomain ()
{
	Vector3DF min, max;
	min = m_Vec[SPH_VOLMIN];
	max = m_Vec[SPH_VOLMAX];
	min.z += 0.5;

	glColor3f ( 0.0, 0.0, 1.0 );
	glBegin ( GL_LINES );
	glVertex3f ( min.x, min.y, min.z );	glVertex3f ( max.x, min.y, min.z );
	glVertex3f ( min.x, max.y, min.z );	glVertex3f ( max.x, max.y, min.z );
	glVertex3f ( min.x, min.y, min.z );	glVertex3f ( min.x, max.y, min.z );
	glVertex3f ( max.x, min.y, min.z );	glVertex3f ( max.x, max.y, min.z );
	glEnd ();
}

void FluidSystem::Advance ()
{
	char *dat1, *dat1_end;
	Fluid* p;
	Vector3DF norm, z;
	Vector3DF dir, accel;
	Vector3DF vnext;
	Vector3DF min, max;
	double adj;
	float SL, SL2, ss, radius;
	float stiff, damp, speed, diff; 
	SL = m_Param[SPH_LIMIT];
	SL2 = SL*SL;
	
	stiff = m_Param[SPH_EXTSTIFF];
	damp = m_Param[SPH_EXTDAMP];
	radius = m_Param[SPH_PRADIUS];
	min = m_Vec[SPH_VOLMIN];
	max = m_Vec[SPH_VOLMAX];
	ss = m_Param[SPH_SIMSCALE];

	dat1_end = mBuf[0].data + NumPoints()*mBuf[0].stride;
	for ( dat1 = mBuf[0].data; dat1 < dat1_end; dat1 += mBuf[0].stride ) {
		p = (Fluid*) dat1;		

		// Compute Acceleration		
		accel = p->sph_force;
		accel *= m_Param[SPH_PMASS];

		// Velocity limiting 
		speed = accel.x*accel.x + accel.y*accel.y + accel.z*accel.z;
		if ( speed > SL2 ) {
			accel *= SL / sqrt(speed);
		}		
	
		// Boundary Conditions

		// Z-axis walls
		diff = 2 * radius - ( p->pos.z - min.z - (p->pos.x - m_Vec[SPH_VOLMIN].x) * m_Param[BOUND_ZMIN_SLOPE] )*ss;
		if (diff > EPSILON ) {			
			norm.Set ( -m_Param[BOUND_ZMIN_SLOPE], 0, 1.0 - m_Param[BOUND_ZMIN_SLOPE] );
			adj = stiff * diff - damp * norm.Dot ( p->vel_eval );
			accel.x += adj * norm.x; accel.y += adj * norm.y; accel.z += adj * norm.z;
		}		

		diff = 2 * radius - ( max.z - p->pos.z )*ss;
		if (diff > EPSILON) {
			norm.Set ( 0, 0, -1 );
			adj = stiff * diff - damp * norm.Dot ( p->vel_eval );
			accel.x += adj * norm.x; accel.y += adj * norm.y; accel.z += adj * norm.z;
		}
		
		// X-axis walls
		if ( !m_Toggle[WRAP_X] ) {
			diff = 2 * radius - ( p->pos.x - min.x + (sin(m_Time*10.0)-1+(p->pos.y*0.025)*0.25) * m_Param[FORCE_XMIN_SIN] )*ss;	
			//diff = 2 * radius - ( p->pos.x - min.x + (sin(m_Time*10.0)-1) * m_Param[FORCE_XMIN_SIN] )*ss;	
			if (diff > EPSILON ) {
				norm.Set ( 1.0, 0, 0 );
				adj = (m_Param[ FORCE_XMIN_SIN ] + 1) * stiff * diff - damp * norm.Dot ( p->vel_eval ) ;
				accel.x += adj * norm.x; accel.y += adj * norm.y; accel.z += adj * norm.z;					
			}

			diff = 2 * radius - ( max.x - p->pos.x + (sin(m_Time*10.0)-1) * m_Param[FORCE_XMAX_SIN] )*ss;	
			if (diff > EPSILON) {
				norm.Set ( -1, 0, 0 );
				adj = (m_Param[ FORCE_XMAX_SIN ]+1) * stiff * diff - damp * norm.Dot ( p->vel_eval );
				accel.x += adj * norm.x; accel.y += adj * norm.y; accel.z += adj * norm.z;
			}
		}

		// Y-axis walls
		diff = 2 * radius - ( p->pos.y - min.y )*ss;			
		if (diff > EPSILON) {
			norm.Set ( 0, 1, 0 );
			adj = stiff * diff - damp * norm.Dot ( p->vel_eval );
			accel.x += adj * norm.x; accel.y += adj * norm.y; accel.z += adj * norm.z;
		}
		diff = 2 * radius - ( max.y - p->pos.y )*ss;
		if (diff > EPSILON) {
			norm.Set ( 0, -1, 0 );
			adj = stiff * diff - damp * norm.Dot ( p->vel_eval );
			accel.x += adj * norm.x; accel.y += adj * norm.y; accel.z += adj * norm.z;
		}

		// Wall barrier
		if ( m_Toggle[WALL_BARRIER] ) {
			diff = 2 * radius - ( p->pos.x - 0 )*ss;					
			if (diff < 2*radius && diff > EPSILON && fabs(p->pos.y) < 3 && p->pos.z < 10) {
				norm.Set ( 1.0, 0, 0 );
				adj = 2*stiff * diff - damp * norm.Dot ( p->vel_eval ) ;	
				accel.x += adj * norm.x; accel.y += adj * norm.y; accel.z += adj * norm.z;					
			}
		}
		
		// Levy barrier
		if ( m_Toggle[LEVY_BARRIER] ) {
			diff = 2 * radius - ( p->pos.x - 0 )*ss;					
			if (diff < 2*radius && diff > EPSILON && fabs(p->pos.y) > 5 && p->pos.z < 10) {
				norm.Set ( 1.0, 0, 0 );
				adj = 2*stiff * diff - damp * norm.Dot ( p->vel_eval ) ;	
				accel.x += adj * norm.x; accel.y += adj * norm.y; accel.z += adj * norm.z;					
			}
		}
		// Drain barrier
		if ( m_Toggle[DRAIN_BARRIER] ) {
			diff = 2 * radius - ( p->pos.z - min.z-15 )*ss;
			if (diff < 2*radius && diff > EPSILON && (fabs(p->pos.x)>3 || fabs(p->pos.y)>3) ) {
				norm.Set ( 0, 0, 1);
				adj = stiff * diff - damp * norm.Dot ( p->vel_eval );
				accel.x += adj * norm.x; accel.y += adj * norm.y; accel.z += adj * norm.z;
			}
		}

		// Plane gravity
		if ( m_Param[PLANE_GRAV] > 0) 
			accel += m_Vec[PLANE_GRAV_DIR];

		// Point gravity
		if ( m_Param[POINT_GRAV] > 0 ) {
			norm.x = ( p->pos.x - m_Vec[POINT_GRAV_POS].x );
			norm.y = ( p->pos.y - m_Vec[POINT_GRAV_POS].y );
			norm.z = ( p->pos.z - m_Vec[POINT_GRAV_POS].z );
			norm.Normalize ();
			norm *= m_Param[POINT_GRAV];
			accel -= norm;
		}

		// Leapfrog Integration ----------------------------
		vnext = accel;							
		vnext *= m_DT;
		vnext += p->vel;						// v(t+1/2) = v(t-1/2) + a(t) dt
		p->vel_eval = p->vel;
		p->vel_eval += vnext;
		p->vel_eval *= 0.5;					// v(t+1) = [v(t-1/2) + v(t+1/2)] * 0.5		used to compute forces later
		p->vel = vnext;
		vnext *= m_DT/ss;
		p->pos += vnext;						// p(t+1) = p(t) + v(t+1/2) dt

		if ( m_Param[CLR_MODE]==1.0 ) {
			adj = fabs(vnext.x)+fabs(vnext.y)+fabs(vnext.z) / 7000.0;
			adj = (adj > 1.0) ? 1.0 : adj;
			p->clr = COLORA( 0, adj, adj, 1 );
		}
		if ( m_Param[CLR_MODE]==2.0 ) {
			float v = 0.5 + ( p->pressure / 1500.0); 
			if ( v < 0.1 ) v = 0.1;
			if ( v > 1.0 ) v = 1.0;
			p->clr = COLORA ( v, 1-v, 0, 1 );
		}


		// Euler integration -------------------------------
		/* accel += m_Gravity;
		accel *= m_DT;
		p->vel += accel;				// v(t+1) = v(t) + a(t) dt
		p->vel_eval += accel;
		p->vel_eval *= m_DT/d;
		p->pos += p->vel_eval;
		p->vel_eval = p->vel;  */	


		if ( m_Toggle[WRAP_X] ) {
			diff = p->pos.x - (m_Vec[SPH_VOLMIN].x + 2);			// -- Simulates object in center of flow
			if ( diff <= 0 ) {
				p->pos.x = (m_Vec[SPH_VOLMAX].x - 2) + diff*2;				
				p->pos.z = 10;
			}
		}	
	}

	m_Time += m_DT;
}

//------------------------------------------------------ SPH Setup 
//
//  Range = +/- 10.0 * 0.006 (r) =	   0.12			m (= 120 mm = 4.7 inch)
//  Container Volume (Vc) =			   0.001728		m^3
//  Rest Density (D) =				1000.0			kg / m^3
//  Particle Mass (Pm) =			   0.00020543	kg						(mass = vol * density)
//  Number of Particles (N) =		4000.0
//  Water Mass (M) =				   0.821		kg (= 821 grams)
//  Water Volume (V) =				   0.000821     m^3 (= 3.4 cups, .21 gals)
//  Smoothing Radius (R) =             0.02			m (= 20 mm = ~3/4 inch)
//  Particle Radius (Pr) =			   0.00366		m (= 4 mm  = ~1/8 inch)
//  Particle Volume (Pv) =			   2.054e-7		m^3	(= .268 milliliters)
//  Rest Distance (Pd) =			   0.0059		m
//
//  Given: D, Pm, N
//    Pv = Pm / D			0.00020543 kg / 1000 kg/m^3 = 2.054e-7 m^3	
//    Pv = 4/3*pi*Pr^3    cuberoot( 2.054e-7 m^3 * 3/(4pi) ) = 0.00366 m
//     M = Pm * N			0.00020543 kg * 4000.0 = 0.821 kg		
//     V =  M / D              0.821 kg / 1000 kg/m^3 = 0.000821 m^3
//     V = Pv * N			 2.054e-7 m^3 * 4000 = 0.000821 m^3
//    Pd = cuberoot(Pm/D)    cuberoot(0.00020543/1000) = 0.0059 m 
//
// Ideal grid cell size (gs) = 2 * smoothing radius = 0.02*2 = 0.04
// Ideal domain size = k*gs/d = k*0.02*2/0.005 = k*8 = {8, 16, 24, 32, 40, 48, ..}
//    (k = number of cells, gs = cell size, d = simulation scale)

void FluidSystem::SPH_Setup ()
{
	m_Param [ SPH_SIMSCALE ] =		0.004;			// unit size
	m_Param [ SPH_VISC ] =			0.2;			// pascal-second (Pa.s) = 1 kg m^-1 s^-1  (see wikipedia page on viscosity)
	m_Param [ SPH_RESTDENSITY ] =	600.0;			// kg / m^3
	m_Param [ SPH_PMASS ] =			0.00020543;		// kg
	m_Param [ SPH_PRADIUS ] =		0.004;			// m
	m_Param [ SPH_PDIST ] =			0.0059;			// m
	m_Param [ SPH_SMOOTHRADIUS ] =	0.01;			// m 
	m_Param [ SPH_INTSTIFF ] =		1.00;
	m_Param [ SPH_EXTSTIFF ] =		10000.0;
	m_Param [ SPH_EXTDAMP ] =		256.0;
	m_Param [ SPH_LIMIT ] =			200.0;			// m / s

	m_Toggle [ SPH_GRID ] =		false;
	m_Toggle [ SPH_DEBUG ] =	false;

	SPH_ComputeKernels ();
}

void FluidSystem::SPH_ComputeKernels ()
{
	m_Param [ SPH_PDIST ] = pow ( m_Param[SPH_PMASS] / m_Param[SPH_RESTDENSITY], 1/3.0 );
	m_R2 = m_Param [SPH_SMOOTHRADIUS] * m_Param[SPH_SMOOTHRADIUS];
	m_Poly6Kern = 315.0f / (64.0f * 3.141592 * pow( m_Param[SPH_SMOOTHRADIUS], 9) );	// Wpoly6 kernel (denominator part) - 2003 Muller, p.4
	m_SpikyKern = -45.0f / (3.141592 * pow( m_Param[SPH_SMOOTHRADIUS], 6) );			// Laplacian of viscocity (denominator): PI h^6
	m_LapKern = 45.0f / (3.141592 * pow( m_Param[SPH_SMOOTHRADIUS], 6) );
}

void FluidSystem::SPH_CreateExample ( int n, int nmax )
{
	Vector3DF pos;
	Vector3DF min, max;
	
	Reset ( nmax );
	
	switch ( n ) {
	case 0:		// Wave pool
				
		//-- TEST CASE: 2x2x2 grid, 32 particles.  NOTE: Set PRADIUS to 0.0004 to reduce wall influence
		//     grid 0:    3*3*2 = 18 particles
		//     grid 1,2:  3*1*2 =  6 particles
		//     grid 3:    1*1*2 =  2 particles
		//     grid 4,5,6:    0 =  0 particles
		/*m_Vec [ SPH_VOLMIN ].Set ( -2.5, -2.5, 0 );
		m_Vec [ SPH_VOLMAX ].Set ( 2.5, 2.5, 5.0 );	
		m_Vec [ SPH_INITMIN ].Set ( -2.5, -2.5, 0 );	
		m_Vec [ SPH_INITMAX ].Set ( 2.5, 2.5, 1.6 );*/  
		
		m_Vec [ SPH_VOLMIN ].Set ( -30, -30, 0 );
		m_Vec [ SPH_VOLMAX ].Set ( 30, 30, 40 );		

		//m_Vec [ SPH_INITMIN ].Set ( -5, -5, 10 );
		//m_Vec [ SPH_INITMAX ].Set ( 5, 5, 20 );
		
		m_Vec [ SPH_INITMIN ].Set ( -20, -26, 10 );
		m_Vec [ SPH_INITMAX ].Set ( 20, 26, 40 );

		m_Param [ FORCE_XMIN_SIN ] = 12.0;
		m_Param [ BOUND_ZMIN_SLOPE ] = 0.05;
		break;
	case 1:		// Dam break
		m_Vec [ SPH_VOLMIN ].Set ( -30, -14, 0 );
		m_Vec [ SPH_VOLMAX ].Set ( 30, 14, 60 );
		m_Vec [ SPH_INITMIN ].Set ( 0, -13, 0 );
		m_Vec [ SPH_INITMAX ].Set ( 29, 13, 30 );		
		m_Vec [ PLANE_GRAV_DIR ].Set ( 0.0, 0, -9.8 );
		break;
	case 2:		// Dual-Wave pool
		m_Vec [ SPH_VOLMIN ].Set ( -60, -5, 0 );
		m_Vec [ SPH_VOLMAX ].Set ( 60, 5, 50 );
		m_Vec [ SPH_INITMIN ].Set ( -46, -5, 0 );
		m_Vec [ SPH_INITMAX ].Set ( 46, 5, 15 );
		m_Param [ FORCE_XMIN_SIN ] = 8.0;
		m_Param [ FORCE_XMAX_SIN ] = 8.0;
		m_Vec [ PLANE_GRAV_DIR ].Set ( 0.0, 0, -9.8 );
		break;
	case 3:		// Swirl Stream
		m_Vec [ SPH_VOLMIN ].Set ( -30, -30, 0 );
		m_Vec [ SPH_VOLMAX ].Set ( 30, 30, 50 );
		m_Vec [ SPH_INITMIN ].Set ( -30, -30, 0 );
		m_Vec [ SPH_INITMAX ].Set ( 30, 30, 40 );
		m_Vec [ EMIT_POS ].Set ( -20, -20, 22 );
		m_Vec [ EMIT_RATE ].Set ( 1, 4, 0 );
		m_Vec [ EMIT_ANG ].Set ( 0, 120, 1.5 );
		m_Vec [ EMIT_DANG ].Set ( 0, 0, 0 );
		m_Vec [ PLANE_GRAV_DIR ].Set ( 0.0, 0, -9.8 );
		break;
	case 4:		// Shockwave
		m_Vec [ SPH_VOLMIN ].Set ( -60, -15, 0 );
		m_Vec [ SPH_VOLMAX ].Set ( 60, 15, 50 );
		m_Vec [ SPH_INITMIN ].Set ( -59, -14, 0 );
		m_Vec [ SPH_INITMAX ].Set ( 59, 14, 30 );
		m_Vec [ PLANE_GRAV_DIR ].Set ( 0.0, 0, -9.8 );
		m_Toggle [ WALL_BARRIER ] = true;
		m_Toggle [ WRAP_X ] = true;
		break;
	case 5:		// Zero gravity
		m_Vec [ SPH_VOLMIN ].Set ( -40, -40, 0 );
		m_Vec [ SPH_VOLMAX ].Set ( 40, 40, 50 );
		m_Vec [ SPH_INITMIN ].Set ( -20, -20, 20 );
		m_Vec [ SPH_INITMAX ].Set ( 20, 20, 40 );
		m_Vec [ EMIT_POS ].Set ( -20, 0, 40 );
		m_Vec [ EMIT_RATE ].Set ( 2, 1, 0 );		
		m_Vec [ EMIT_ANG ].Set ( 0, 120, 0.25 );
		m_Vec [ PLANE_GRAV_DIR ].Set ( 0, 0, 0 );
		m_Param [ SPH_INTSTIFF ] = 0.20;		
		break;
	case 6:		// Point gravity		
		m_Vec [ SPH_VOLMIN ].Set ( -40, -40, 0 );
		m_Vec [ SPH_VOLMAX ].Set ( 40, 40, 50 );
		m_Vec [ SPH_INITMIN ].Set ( -20, -20, 20 );
		m_Vec [ SPH_INITMAX ].Set ( 20, 20, 40 );
		m_Param [ SPH_INTSTIFF ] = 0.50;		
		m_Vec [ EMIT_POS ].Set ( -20, 20, 25 );
		m_Vec [ EMIT_RATE ].Set ( 1, 4, 0 );		
		m_Vec [ EMIT_ANG ].Set ( -20, 100, 2.0 );
		m_Vec [ POINT_GRAV_POS ].Set ( 0, 0, 25 );
		m_Vec [ PLANE_GRAV_DIR ].Set ( 0, 0, 0 );
		m_Param [ POINT_GRAV ] = 3.5;
		break;
	case 7:		// Levy break
		m_Vec [ SPH_VOLMIN ].Set ( -40, -40, 0 );
		m_Vec [ SPH_VOLMAX ].Set ( 40, 40, 50 );
		m_Vec [ SPH_INITMIN ].Set ( 10, -40, 0 );
		m_Vec [ SPH_INITMAX ].Set ( 40, 40, 50 );
		m_Vec [ EMIT_POS ].Set ( 34, 27, 16.6 );
		m_Vec [ EMIT_RATE ].Set ( 2, 9, 0 );		
		m_Vec [ EMIT_ANG ].Set ( 118, 200, 1.0 );
		m_Toggle [ LEVY_BARRIER ] = true;
		m_Param [ BOUND_ZMIN_SLOPE ] = 0.1;
		m_Vec [ PLANE_GRAV_DIR ].Set ( 0.0, 0, -9.8 );
		break;
	case 8:		// Drain
		m_Vec [ SPH_VOLMIN ].Set ( -20, -20, 0 );
		m_Vec [ SPH_VOLMAX ].Set ( 20, 20, 50 );
		m_Vec [ SPH_INITMIN ].Set ( -15, -20, 20 );
		m_Vec [ SPH_INITMAX ].Set ( 20, 20, 50 );
		m_Vec [ EMIT_POS ].Set ( -16, -16, 30 );
		m_Vec [ EMIT_RATE ].Set ( 1, 4, 0 );		
		m_Vec [ EMIT_ANG ].Set ( -20, 140, 1.8 );
		m_Toggle [ DRAIN_BARRIER ] = true;
		m_Vec [ PLANE_GRAV_DIR ].Set ( 0.0, 0, -9.8 );
		break;
	case 9:			// Tumbler
		m_Vec [ SPH_VOLMIN ].Set ( -30, -30, 0 );
		m_Vec [ SPH_VOLMAX ].Set ( 30, 30, 50 );
		m_Vec [ SPH_INITMIN ].Set ( 24, -29, 20 );
		m_Vec [ SPH_INITMAX ].Set ( 29, 29, 40 );
		m_Param [ SPH_VISC ] = 0.1;		
		m_Param [ SPH_INTSTIFF ] = 0.50;
		m_Param [ SPH_EXTSTIFF ] = 8000;
		//m_Param [ SPH_SMOOTHRADIUS ] = 0.01;
		m_Param [ BOUND_ZMIN_SLOPE ] = 0.4;
		m_Param [ FORCE_XMIN_SIN ] = 12.00;
		m_Vec [ PLANE_GRAV_DIR ].Set ( 0.0, 0, -9.8 );
		break;	
	case 10:		// Large sim
		m_Vec [ SPH_VOLMIN ].Set ( -35, -35, 0 );
		m_Vec [ SPH_VOLMAX ].Set ( 35, 35, 60 );
		m_Vec [ SPH_INITMIN ].Set ( -5, -35, 0 );
		m_Vec [ SPH_INITMAX ].Set ( 30, 0, 60 );
		m_Vec [ PLANE_GRAV_DIR ].Set ( 0.0, 0, -9.8 );
		break;
	}	

	SPH_ComputeKernels ();

	m_Param [ SPH_SIMSIZE ] = m_Param [ SPH_SIMSCALE ] * (m_Vec[SPH_VOLMAX].z - m_Vec[SPH_VOLMIN].z);
	m_Param [ SPH_PDIST ] = pow ( m_Param[SPH_PMASS] / m_Param[SPH_RESTDENSITY], 1/3.0 );	

	float ss = m_Param [ SPH_PDIST ]*0.87 / m_Param[ SPH_SIMSCALE ];	
	printf ( "Spacing: %f\n", ss);
	AddVolume ( m_Vec[SPH_INITMIN], m_Vec[SPH_INITMAX], ss );	// Create the particles

	float cell_size = m_Param[SPH_SMOOTHRADIUS]*2.0;			// Grid cell size (2r)	
	Grid_Setup ( m_Vec[SPH_VOLMIN], m_Vec[SPH_VOLMAX], m_Param[SPH_SIMSCALE], cell_size, 1.0 );												// Setup grid
	Grid_InsertParticles ();									// Insert particles

	Vector3DF vmin, vmax;
	vmin =  m_Vec[SPH_VOLMIN];
	vmin -= Vector3DF(2,2,2);
	vmax =  m_Vec[SPH_VOLMAX];
	vmax += Vector3DF(2,2,-2);

	#ifdef BUILD_CUDA
		FluidClearCUDA ();
		Sleep ( 500 );

		FluidSetupCUDA ( NumPoints(), sizeof(Fluid), *(float3*)& m_GridMin, *(float3*)& m_GridMax, *(float3*)& m_GridRes, *(float3*)& m_GridSize, (int) m_Vec[EMIT_RATE].x );

		Sleep ( 500 );

		FluidParamCUDA ( m_Param[SPH_SIMSCALE], m_Param[SPH_SMOOTHRADIUS], m_Param[SPH_PMASS], m_Param[SPH_RESTDENSITY], m_Param[SPH_INTSTIFF], m_Param[SPH_VISC] );
	#endif

}

// Compute Pressures - Very slow yet simple. O(n^2)
void FluidSystem::SPH_ComputePressureSlow ()
{
	char *dat1, *dat1_end;
	char *dat2, *dat2_end;
	Fluid *p, *q;
	int cnt = 0;
	double dx, dy, dz, sum, dsq, c;
	double d, d2, mR, mR2;
	d = m_Param[SPH_SIMSCALE];
	d2 = d*d;
	mR = m_Param[SPH_SMOOTHRADIUS];
	mR2 = mR*mR;	

	dat1_end = mBuf[0].data + NumPoints()*mBuf[0].stride;
	for ( dat1 = mBuf[0].data; dat1 < dat1_end; dat1 += mBuf[0].stride ) {
		p = (Fluid*) dat1;

		sum = 0.0;
		cnt = 0;
		
		dat2_end = mBuf[0].data + NumPoints()*mBuf[0].stride;
		for ( dat2 = mBuf[0].data; dat2 < dat2_end; dat2 += mBuf[0].stride ) {
			q = (Fluid*) dat2;

			if ( p==q ) continue;
			dx = ( p->pos.x - q->pos.x)*d;		// dist in cm
			dy = ( p->pos.y - q->pos.y)*d;
			dz = ( p->pos.z - q->pos.z)*d;
			dsq = (dx*dx + dy*dy + dz*dz);
			if ( mR2 > dsq ) {
				c =  m_R2 - dsq;
				sum += c * c * c;
				cnt++;
				//if ( p == m_CurrP ) q->tag = true;
			}
		}	
		p->density = sum * m_Param[SPH_PMASS] * m_Poly6Kern ;	
		p->pressure = ( p->density - m_Param[SPH_RESTDENSITY] ) * m_Param[SPH_INTSTIFF];
		p->density = 1.0f / p->density;
	}
}

// Compute Pressures - Using spatial grid, and also create neighbor table
void FluidSystem::SPH_ComputePressureGrid ()
{
	char *dat1, *dat1_end;
	Fluid* p;
	Fluid* pcurr;
	int pndx;
	int i, cnt = 0;
	float dx, dy, dz, sum, dsq, c;
	float d, d2, mR, mR2;
	float radius = m_Param[SPH_SMOOTHRADIUS] / m_Param[SPH_SIMSCALE];
	d = m_Param[SPH_SIMSCALE];
	d2 = d*d;
	mR = m_Param[SPH_SMOOTHRADIUS];
	mR2 = mR*mR;	

	dat1_end = mBuf[0].data + NumPoints()*mBuf[0].stride;
	i = 0;
	for ( dat1 = mBuf[0].data; dat1 < dat1_end; dat1 += mBuf[0].stride, i++ ) {
		p = (Fluid*) dat1;

		sum = 0.0;	
		m_NC[i] = 0;

		Grid_FindCells ( p->pos, radius );
		for (int cell=0; cell < 8; cell++) {
			if ( m_GridCell[cell] != -1 ) {
				pndx = m_Grid [ m_GridCell[cell] ];				
				while ( pndx != -1 ) {					
					pcurr = (Fluid*) (mBuf[0].data + pndx*mBuf[0].stride);					
					if ( pcurr == p ) {pndx = pcurr->next; continue; }
					dx = ( p->pos.x - pcurr->pos.x)*d;		// dist in cm
					dy = ( p->pos.y - pcurr->pos.y)*d;
					dz = ( p->pos.z - pcurr->pos.z)*d;
					dsq = (dx*dx + dy*dy + dz*dz);
					if ( mR2 > dsq ) {
						c =  m_R2 - dsq;
						sum += c * c * c;
						if ( m_NC[i] < MAX_NEIGHBOR ) {
							m_Neighbor[i][ m_NC[i] ] = pndx;
							m_NDist[i][ m_NC[i] ] = sqrt(dsq);
							m_NC[i]++;
						}
					}
					pndx = pcurr->next;
				}
			}
			m_GridCell[cell] = -1;
		}
		p->density = sum * m_Param[SPH_PMASS] * m_Poly6Kern ;	
		p->pressure = ( p->density - m_Param[SPH_RESTDENSITY] ) * m_Param[SPH_INTSTIFF];		
		p->density = 1.0f / p->density;		
	}
}

// Compute Forces - Very slow, but simple. O(n^2)
void FluidSystem::SPH_ComputeForceSlow ()
{
	char *dat1, *dat1_end;
	char *dat2, *dat2_end;
	Fluid *p, *q;
	Vector3DF force, fcurr;
	register double pterm, vterm, dterm;
	double c, r, d, sum, dsq;
	double dx, dy, dz;
	double mR, mR2, visc;

	d = m_Param[SPH_SIMSCALE];
	mR = m_Param[SPH_SMOOTHRADIUS];
	mR2 = (mR*mR);
	visc = m_Param[SPH_VISC];
	vterm = m_LapKern * visc;

	dat1_end = mBuf[0].data + NumPoints()*mBuf[0].stride;
	for ( dat1 = mBuf[0].data; dat1 < dat1_end; dat1 += mBuf[0].stride ) {
		p = (Fluid*) dat1;

		sum = 0.0;
		force.Set ( 0, 0, 0 );
		
		dat2_end = mBuf[0].data + NumPoints()*mBuf[0].stride;
		for ( dat2 = mBuf[0].data; dat2 < dat2_end; dat2 += mBuf[0].stride ) {
			q = (Fluid*) dat2;

			if ( p == q ) continue;
			dx = ( p->pos.x - q->pos.x )*d;			// dist in cm
			dy = ( p->pos.y - q->pos.y )*d;
			dz = ( p->pos.z - q->pos.z )*d;
			dsq = (dx*dx + dy*dy + dz*dz);
			if ( mR2 > dsq ) {
				r = sqrt ( dsq );
				c = (mR - r);
				pterm = -0.5f * c * m_SpikyKern * ( p->pressure + q->pressure) / r;
				dterm = c * p->density * q->density;
				force.x += ( pterm * dx + vterm * (q->vel_eval.x - p->vel_eval.x) ) * dterm;
				force.y += ( pterm * dy + vterm * (q->vel_eval.y - p->vel_eval.y) ) * dterm;
				force.z += ( pterm * dz + vterm * (q->vel_eval.z - p->vel_eval.z) ) * dterm;
			}
		}			
		p->sph_force = force;		
	}
}

// Compute Forces - Using spatial grid. Faster.
void FluidSystem::SPH_ComputeForceGrid ()
{
	char *dat1, *dat1_end;	
	Fluid *p;
	Fluid *pcurr;
	int pndx;
	Vector3DF force, fcurr;
	register double pterm, vterm, dterm;
	double c, d, dsq, r;
	double dx, dy, dz;
	double mR, mR2, visc;
	float radius = m_Param[SPH_SMOOTHRADIUS] / m_Param[SPH_SIMSCALE];

	d = m_Param[SPH_SIMSCALE];
	mR = m_Param[SPH_SMOOTHRADIUS];
	mR2 = (mR*mR);
	visc = m_Param[SPH_VISC];

	dat1_end = mBuf[0].data + NumPoints()*mBuf[0].stride;
	for ( dat1 = mBuf[0].data; dat1 < dat1_end; dat1 += mBuf[0].stride ) {
		p = (Fluid*) dat1;

		force.Set ( 0, 0, 0 );

		Grid_FindCells ( p->pos, radius );
		for (int cell=0; cell < 8; cell++) {
			if ( m_GridCell[cell] != -1 ) {
				pndx = m_Grid [ m_GridCell[cell] ];				
				while ( pndx != -1 ) {					
					pcurr = (Fluid*) (mBuf[0].data + pndx*mBuf[0].stride);					
					if ( pcurr == p ) {pndx = pcurr->next; continue; }
			
					dx = ( p->pos.x - pcurr->pos.x)*d;		// dist in cm
					dy = ( p->pos.y - pcurr->pos.y)*d;
					dz = ( p->pos.z - pcurr->pos.z)*d;
					dsq = (dx*dx + dy*dy + dz*dz);
					if ( mR2 > dsq ) {
						r = sqrt ( dsq );
						c = (mR - r);
						pterm = -0.5f * c * m_SpikyKern * ( p->pressure + pcurr->pressure) / r;
						dterm = c * p->density * pcurr->density;
						vterm =	m_LapKern * visc;
						force.x += ( pterm * dx + vterm * (pcurr->vel_eval.x - p->vel_eval.x) ) * dterm;
						force.y += ( pterm * dy + vterm * (pcurr->vel_eval.y - p->vel_eval.y) ) * dterm;
						force.z += ( pterm * dz + vterm * (pcurr->vel_eval.z - p->vel_eval.z) ) * dterm;
					}
					pndx = pcurr->next;
				}
			}
		}
		p->sph_force = force;
	}
}

// Compute Forces - Using spatial grid with saved neighbor table. Fastest.
void FluidSystem::SPH_ComputeForceGridNC ()
{
	char *dat1, *dat1_end;	
	Fluid *p;
	Fluid *pcurr;
	Vector3DF force, fcurr;
	register float pterm, vterm, dterm;
	int i;
	float c, d;
	float dx, dy, dz;
	float mR, mR2, visc;	

	d = m_Param[SPH_SIMSCALE];
	mR = m_Param[SPH_SMOOTHRADIUS];
	mR2 = (mR*mR);
	visc = m_Param[SPH_VISC];

	dat1_end = mBuf[0].data + NumPoints()*mBuf[0].stride;
	i = 0;
	
	for ( dat1 = mBuf[0].data; dat1 < dat1_end; dat1 += mBuf[0].stride, i++ ) {
		p = (Fluid*) dat1;

		force.Set ( 0, 0, 0 );
		for (int j=0; j < m_NC[i]; j++ ) {
			pcurr = (Fluid*) (mBuf[0].data + m_Neighbor[i][j]*mBuf[0].stride);
			dx = ( p->pos.x - pcurr->pos.x)*d;		// dist in cm
			dy = ( p->pos.y - pcurr->pos.y)*d;
			dz = ( p->pos.z - pcurr->pos.z)*d;				
			c = ( mR - m_NDist[i][j] );
			pterm = -0.5f * c * m_SpikyKern * ( p->pressure + pcurr->pressure) / m_NDist[i][j];
			dterm = c * p->density * pcurr->density;
			vterm = m_LapKern * visc;
			force.x += ( pterm * dx + vterm * (pcurr->vel_eval.x - p->vel_eval.x) ) * dterm;
			force.y += ( pterm * dy + vterm * (pcurr->vel_eval.y - p->vel_eval.y) ) * dterm;
			force.z += ( pterm * dz + vterm * (pcurr->vel_eval.z - p->vel_eval.z) ) * dterm;
		}			
		p->sph_force = force;
	}
}

