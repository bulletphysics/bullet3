/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

#include <ode/config.h>
#include <ode/mass.h>
#include <ode/odemath.h>
#include <ode/matrix.h>


#define _I(i,j) I[(i)*4+(j)]


// return 1 if ok, 0 if bad

static int checkMass (dMass *m)
{
  int i;

  if (m->mass <= 0) {
    dDEBUGMSG ("mass must be > 0");
    return 0;
  }
  if (!dIsPositiveDefinite (m->I,3)) {
    dDEBUGMSG ("inertia must be positive definite");
    return 0;
  }

  // verify that the center of mass position is consistent with the mass
  // and inertia matrix. this is done by checking that the inertia around
  // the center of mass is also positive definite. from the comment in
  // dMassTranslate(), if the body is translated so that its center of mass
  // is at the point of reference, then the new inertia is:
  //   I + mass*crossmat(c)^2
  // note that requiring this to be positive definite is exactly equivalent
  // to requiring that the spatial inertia matrix
  //   [ mass*eye(3,3)   M*crossmat(c)^T ]
  //   [ M*crossmat(c)   I               ]
  // is positive definite, given that I is PD and mass>0. see the theorem
  // about partitioned PD matrices for proof.

  dMatrix3 I2,chat;
  dSetZero (chat,12);
  dCROSSMAT (chat,m->c,4,+,-);
  dMULTIPLY0_333 (I2,chat,chat);
  for (i=0; i<3; i++) I2[i] = m->I[i] + m->mass*I2[i];
  for (i=4; i<7; i++) I2[i] = m->I[i] + m->mass*I2[i];
  for (i=8; i<11; i++) I2[i] = m->I[i] + m->mass*I2[i];
  if (!dIsPositiveDefinite (I2,3)) {
    dDEBUGMSG ("center of mass inconsistent with mass parameters");
    return 0;
  }
  return 1;
}


void dMassSetZero (dMass *m)
{
  dAASSERT (m);
  m->mass = REAL(0.0);
  dSetZero (m->c,sizeof(m->c) / sizeof(dReal));
  dSetZero (m->I,sizeof(m->I) / sizeof(dReal));
}


void dMassSetParameters (dMass *m, dReal themass,
			 dReal cgx, dReal cgy, dReal cgz,
			 dReal I11, dReal I22, dReal I33,
			 dReal I12, dReal I13, dReal I23)
{
  dAASSERT (m);
  dMassSetZero (m);
  m->mass = themass;
  m->c[0] = cgx;
  m->c[1] = cgy;
  m->c[2] = cgz;
  m->_I(0,0) = I11;
  m->_I(1,1) = I22;
  m->_I(2,2) = I33;
  m->_I(0,1) = I12;
  m->_I(0,2) = I13;
  m->_I(1,2) = I23;
  m->_I(1,0) = I12;
  m->_I(2,0) = I13;
  m->_I(2,1) = I23;
  checkMass (m);
}


void dMassSetSphere (dMass *m, dReal density, dReal radius)
{
  dMassSetSphereTotal (m, (REAL(4.0)/REAL(3.0)) * M_PI *
			  radius*radius*radius * density, radius);
}


void dMassSetSphereTotal (dMass *m, dReal total_mass, dReal radius)
{
  dAASSERT (m);
  dMassSetZero (m);
  m->mass = total_mass;
  dReal II = REAL(0.4) * total_mass * radius*radius;
  m->_I(0,0) = II;
  m->_I(1,1) = II;
  m->_I(2,2) = II;

# ifndef dNODEBUG
  checkMass (m);
# endif
}


void dMassSetCappedCylinder (dMass *m, dReal density, int direction,
			     dReal radius, dReal length)
{
  dReal M1,M2,Ia,Ib;
  dAASSERT (m);
  dUASSERT (direction >= 1 && direction <= 3,"bad direction number");
  dMassSetZero (m);
  M1 = M_PI*radius*radius*length*density;			// cylinder mass
  M2 = (REAL(4.0)/REAL(3.0))*M_PI*radius*radius*radius*density;	// total cap mass
  m->mass = M1+M2;
  Ia = M1*(REAL(0.25)*radius*radius + (REAL(1.0)/REAL(12.0))*length*length) +
    M2*(REAL(0.4)*radius*radius + REAL(0.375)*radius*length + REAL(0.25)*length*length);
  Ib = (M1*REAL(0.5) + M2*REAL(0.4))*radius*radius;
  m->_I(0,0) = Ia;
  m->_I(1,1) = Ia;
  m->_I(2,2) = Ia;
  m->_I(direction-1,direction-1) = Ib;

# ifndef dNODEBUG
  checkMass (m);
# endif
}


void dMassSetCappedCylinderTotal (dMass *m, dReal total_mass, int direction,
			     dReal a, dReal b)
{
  dMassSetCappedCylinder (m, 1.0, direction, a, b);
  dMassAdjust (m, total_mass);
}


void dMassSetCylinder (dMass *m, dReal density, int direction,
		       dReal radius, dReal length)
{
  dMassSetCylinderTotal (m, M_PI*radius*radius*length*density,
			    direction, radius, length);
}

void dMassSetCylinderTotal (dMass *m, dReal total_mass, int direction,
			    dReal radius, dReal length)
{
  dReal r2,I;
  dAASSERT (m);
  dMassSetZero (m);
  r2 = radius*radius;
  m->mass = total_mass;
  I = total_mass*(REAL(0.25)*r2 + (REAL(1.0)/REAL(12.0))*length*length);
  m->_I(0,0) = I;
  m->_I(1,1) = I;
  m->_I(2,2) = I;
  m->_I(direction-1,direction-1) = total_mass*REAL(0.5)*r2;

# ifndef dNODEBUG
  checkMass (m);
# endif
}


void dMassSetBox (dMass *m, dReal density,
		  dReal lx, dReal ly, dReal lz)
{
  dMassSetBoxTotal (m, lx*ly*lz*density, lx, ly, lz);
}


void dMassSetBoxTotal (dMass *m, dReal total_mass,
		       dReal lx, dReal ly, dReal lz)
{
  dAASSERT (m);
  dMassSetZero (m);
  m->mass = total_mass;
  m->_I(0,0) = total_mass/REAL(12.0) * (ly*ly + lz*lz);
  m->_I(1,1) = total_mass/REAL(12.0) * (lx*lx + lz*lz);
  m->_I(2,2) = total_mass/REAL(12.0) * (lx*lx + ly*ly);

# ifndef dNODEBUG
  checkMass (m);
# endif
}


void dMassAdjust (dMass *m, dReal newmass)
{
  dAASSERT (m);
  dReal scale = newmass / m->mass;
  m->mass = newmass;
  for (int i=0; i<3; i++) for (int j=0; j<3; j++) m->_I(i,j) *= scale;

# ifndef dNODEBUG
  checkMass (m);
# endif
}


void dMassTranslate (dMass *m, dReal x, dReal y, dReal z)
{
  // if the body is translated by `a' relative to its point of reference,
  // the new inertia about the point of reference is:
  //
  //   I + mass*(crossmat(c)^2 - crossmat(c+a)^2)
  //
  // where c is the existing center of mass and I is the old inertia.

  int i,j;
  dMatrix3 ahat,chat,t1,t2;
  dReal a[3];

  dAASSERT (m);

  // adjust inertia matrix
  dSetZero (chat,12);
  dCROSSMAT (chat,m->c,4,+,-);
  a[0] = x + m->c[0];
  a[1] = y + m->c[1];
  a[2] = z + m->c[2];
  dSetZero (ahat,12);
  dCROSSMAT (ahat,a,4,+,-);
  dMULTIPLY0_333 (t1,ahat,ahat);
  dMULTIPLY0_333 (t2,chat,chat);
  for (i=0; i<3; i++) for (j=0; j<3; j++)
    m->_I(i,j) += m->mass * (t2[i*4+j]-t1[i*4+j]);

  // ensure perfect symmetry
  m->_I(1,0) = m->_I(0,1);
  m->_I(2,0) = m->_I(0,2);
  m->_I(2,1) = m->_I(1,2);

  // adjust center of mass
  m->c[0] += x;
  m->c[1] += y;
  m->c[2] += z;

# ifndef dNODEBUG
  checkMass (m);
# endif
}


void dMassRotate (dMass *m, const dMatrix3 R)
{
  // if the body is rotated by `R' relative to its point of reference,
  // the new inertia about the point of reference is:
  //
  //   R * I * R'
  //
  // where I is the old inertia.

  dMatrix3 t1;
  dReal t2[3];

  dAASSERT (m);

  // rotate inertia matrix
  dMULTIPLY2_333 (t1,m->I,R);
  dMULTIPLY0_333 (m->I,R,t1);

  // ensure perfect symmetry
  m->_I(1,0) = m->_I(0,1);
  m->_I(2,0) = m->_I(0,2);
  m->_I(2,1) = m->_I(1,2);

  // rotate center of mass
  dMULTIPLY0_331 (t2,R,m->c);
  m->c[0] = t2[0];
  m->c[1] = t2[1];
  m->c[2] = t2[2];

# ifndef dNODEBUG
  checkMass (m);
# endif
}


void dMassAdd (dMass *a, const dMass *b)
{
  int i;
  dAASSERT (a && b);
  dReal denom = dRecip (a->mass + b->mass);
  for (i=0; i<3; i++) a->c[i] = (a->c[i]*a->mass + b->c[i]*b->mass)*denom;
  a->mass += b->mass;
  for (i=0; i<12; i++) a->I[i] += b->I[i];
}
