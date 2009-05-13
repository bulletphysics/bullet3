/*
  FLUIDS v.1 - SPH Fluid Simulator for CPU and GPU
  Copyright (C) 2009. Rama Hoetzlein, http://www.rchoetzlein.com

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
// Vector Operations Implemented:
//		=, +, -, *, / (on vectors and scalars)
//		Cross			Cross product vector with op
//		Dot				Dot product vector with op
//		Dist (op)		Distance from vector to op
//		DistSq			Distance^2 from vector to op
//		Length ()		Length of vector
//		Normalize ()	Normalizes vector
//

#include "vector.h"

// Vector2DC Code Definition

#define VTYPE		unsigned char
#define VNAME		2DC

// Constructors/Destructors
inline Vector2DC::Vector2DC() {x=0; y=0;}
inline Vector2DC::~Vector2DC() {}
inline Vector2DC::Vector2DC (VTYPE xa, VTYPE ya) {x=xa; y=ya;}
inline Vector2DC::Vector2DC (Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
inline Vector2DC::Vector2DC (Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
inline Vector2DC::Vector2DC (Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
inline Vector2DC::Vector2DC (Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
inline Vector2DC::Vector2DC (Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
inline Vector2DC::Vector2DC (Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
inline Vector2DC::Vector2DC (Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}

// Member Functions
inline Vector2DC &Vector2DC::operator= (Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator= (Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator= (Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator= (Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator= (Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator= (Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator= (Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}	
	
inline Vector2DC &Vector2DC::operator+= (Vector2DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator+= (Vector2DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator+= (Vector2DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator+= (Vector3DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator+= (Vector3DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator+= (Vector3DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator+= (Vector4DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}

inline Vector2DC &Vector2DC::operator-= (Vector2DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator-= (Vector2DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator-= (Vector2DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator-= (Vector3DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator-= (Vector3DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator-= (Vector3DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator-= (Vector4DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
	
inline Vector2DC &Vector2DC::operator*= (Vector2DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator*= (Vector2DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator*= (Vector2DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator*= (Vector3DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator*= (Vector3DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator*= (Vector3DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator*= (Vector4DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}

inline Vector2DC &Vector2DC::operator/= (Vector2DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator/= (Vector2DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator/= (Vector2DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator/= (Vector3DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator/= (Vector3DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator/= (Vector3DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector2DC &Vector2DC::operator/= (Vector4DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}

// Note: Cross product does not exist for 2D vectors (only 3D)
		
inline double Vector2DC::Dot(Vector2DC &v)			{double dot; dot = (double) x*v.x + (double) y*v.y; return dot;}
inline double Vector2DC::Dot(Vector2DI &v)			{double dot; dot = (double) x*v.x + (double) y*v.y; return dot;}
inline double Vector2DC::Dot(Vector2DF &v)			{double dot; dot = (double) x*v.x + (double) y*v.y; return dot;}

inline double Vector2DC::Dist (Vector2DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector2DC::Dist (Vector2DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector2DC::Dist (Vector2DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector2DC::Dist (Vector3DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector2DC::Dist (Vector3DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector2DC::Dist (Vector3DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector2DC::Dist (Vector4DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector2DC::DistSq (Vector2DC &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
inline double Vector2DC::DistSq (Vector2DI &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
inline double Vector2DC::DistSq (Vector2DF &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
inline double Vector2DC::DistSq (Vector3DC &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
inline double Vector2DC::DistSq (Vector3DI &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
inline double Vector2DC::DistSq (Vector3DF &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
inline double Vector2DC::DistSq (Vector4DF &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}

inline Vector2DC &Vector2DC::Normalize (void) {
	double n = (double) x*x + (double) y*y;
	if (n!=0.0) {
		n = sqrt(n);
		x = (VTYPE) (((double) x*255)/n); 
		y = (VTYPE) (((double) y*255)/n);				
	}
	return *this;
}
inline double Vector2DC::Length (void) { double n; n = (double) x*x + (double) y*y; if (n != 0.0) return sqrt(n); return 0.0; }

inline VTYPE &Vector2DC::X(void)				{return x;}
inline VTYPE &Vector2DC::Y(void)				{return y;}
inline VTYPE Vector2DC::Z(void)				{return 0;}
inline VTYPE Vector2DC::W(void)				{return 0;}
inline const VTYPE &Vector2DC::X(void) const	{return x;}
inline const VTYPE &Vector2DC::Y(void) const	{return y;}
inline const VTYPE Vector2DC::Z(void) const	{return 0;}
inline const VTYPE Vector2DC::W(void) const	{return 0;}
inline VTYPE *Vector2DC::Data (void)			{return &x;}

#undef VTYPE
#undef VNAME

// Vector2DI Code Definition

#define VNAME		2DI
#define VTYPE		int

// Constructors/Destructors
inline Vector2DI::Vector2DI() {x=0; y=0;}
inline Vector2DI::~Vector2DI() {}
inline Vector2DI::Vector2DI (VTYPE xa, VTYPE ya) {x=xa; y=ya;}
inline Vector2DI::Vector2DI (Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
inline Vector2DI::Vector2DI (Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
inline Vector2DI::Vector2DI (Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
inline Vector2DI::Vector2DI (Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
inline Vector2DI::Vector2DI (Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
inline Vector2DI::Vector2DI (Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
inline Vector2DI::Vector2DI (Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}

// Member Functions
inline Vector2DI &Vector2DI::operator= (Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator= (Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator= (Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator= (Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator= (Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator= (Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator= (Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}	
	
inline Vector2DI &Vector2DI::operator+= (Vector2DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator+= (Vector2DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator+= (Vector2DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator+= (Vector3DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator+= (Vector3DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator+= (Vector3DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator+= (Vector4DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}

inline Vector2DI &Vector2DI::operator-= (Vector2DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator-= (Vector2DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator-= (Vector2DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator-= (Vector3DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator-= (Vector3DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator-= (Vector3DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator-= (Vector4DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
	
inline Vector2DI &Vector2DI::operator*= (Vector2DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator*= (Vector2DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator*= (Vector2DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator*= (Vector3DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator*= (Vector3DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator*= (Vector3DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator*= (Vector4DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}

inline Vector2DI &Vector2DI::operator/= (Vector2DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator/= (Vector2DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator/= (Vector2DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator/= (Vector3DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator/= (Vector3DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator/= (Vector3DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector2DI &Vector2DI::operator/= (Vector4DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}

// Note: Cross product does not exist for 2D vectors (only 3D)
		
inline double Vector2DI::Dot(Vector2DC &v)			{double dot; dot = (double) x*v.x + (double) y*v.y; return dot;}
inline double Vector2DI::Dot(Vector2DI &v)			{double dot; dot = (double) x*v.x + (double) y*v.y; return dot;}
inline double Vector2DI::Dot(Vector2DF &v)			{double dot; dot = (double) x*v.x + (double) y*v.y; return dot;}

inline double Vector2DI::Dist (Vector2DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector2DI::Dist (Vector2DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector2DI::Dist (Vector2DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector2DI::Dist (Vector3DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector2DI::Dist (Vector3DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector2DI::Dist (Vector3DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector2DI::Dist (Vector4DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector2DI::DistSq (Vector2DC &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
inline double Vector2DI::DistSq (Vector2DI &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
inline double Vector2DI::DistSq (Vector2DF &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
inline double Vector2DI::DistSq (Vector3DC &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
inline double Vector2DI::DistSq (Vector3DI &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
inline double Vector2DI::DistSq (Vector3DF &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
inline double Vector2DI::DistSq (Vector4DF &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}

inline Vector2DI &Vector2DI::Normalize (void) {
	double n = (double) x*x + (double) y*y;
	if (n!=0.0) {
		n = sqrt(n);
		x = (VTYPE) (((double) x*255)/n);
		y = (VTYPE) (((double) y*255)/n);				
	}
	return *this;
}
inline double Vector2DI::Length (void) { double n; n = (double) x*x + (double) y*y; if (n != 0.0) return sqrt(n); return 0.0; }

inline VTYPE &Vector2DI::X(void)				{return x;}
inline VTYPE &Vector2DI::Y(void)				{return y;}
inline VTYPE Vector2DI::Z(void)				{return 0;}
inline VTYPE Vector2DI::W(void)				{return 0;}
inline const VTYPE &Vector2DI::X(void) const	{return x;}
inline const VTYPE &Vector2DI::Y(void) const	{return y;}
inline const VTYPE Vector2DI::Z(void) const	{return 0;}
inline const VTYPE Vector2DI::W(void) const	{return 0;}
inline VTYPE *Vector2DI::Data (void)			{return &x;}

#undef VTYPE
#undef VNAME

// Vector2DF Code Definition

#define VNAME		2DF
#define VTYPE		double

// Constructors/Destructors
inline Vector2DF::Vector2DF() {x=0; y=0;}
inline Vector2DF::~Vector2DF() {}
inline Vector2DF::Vector2DF (const VTYPE xa, const VTYPE ya) {x=xa; y=ya;}
inline Vector2DF::Vector2DF (const Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
inline Vector2DF::Vector2DF (const Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
inline Vector2DF::Vector2DF (const Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
inline Vector2DF::Vector2DF (const Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
inline Vector2DF::Vector2DF (const Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
inline Vector2DF::Vector2DF (const Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
inline Vector2DF::Vector2DF (const Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}

// Member Functions
inline Vector2DF &Vector2DF::operator= (const Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator= (const Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator= (const Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator= (const Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator= (const Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator= (const Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator= (const Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}	
	
inline Vector2DF &Vector2DF::operator+= (const Vector2DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator+= (const Vector2DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator+= (const Vector2DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator+= (const Vector3DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator+= (const Vector3DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator+= (const Vector3DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator+= (const Vector4DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}

inline Vector2DF &Vector2DF::operator-= (const Vector2DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator-= (const Vector2DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator-= (const Vector2DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator-= (const Vector3DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator-= (const Vector3DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator-= (const Vector3DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator-= (const Vector4DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
	
inline Vector2DF &Vector2DF::operator*= (const Vector2DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator*= (const Vector2DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator*= (const Vector2DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator*= (const Vector3DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator*= (const Vector3DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator*= (const Vector3DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator*= (const Vector4DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}

inline Vector2DF &Vector2DF::operator/= (const Vector2DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator/= (const Vector2DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator/= (const Vector2DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator/= (const Vector3DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator/= (const Vector3DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator/= (const Vector3DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector2DF &Vector2DF::operator/= (const Vector4DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}

// Note: Cross product does not exist for 2D vectors (only 3D)
		
inline double Vector2DF::Dot(const Vector2DC &v)			{double dot; dot = (double) x*v.x + (double) y*v.y; return dot;}
inline double Vector2DF::Dot(const Vector2DI &v)			{double dot; dot = (double) x*v.x + (double) y*v.y; return dot;}
inline double Vector2DF::Dot(const Vector2DF &v)			{double dot; dot = (double) x*v.x + (double) y*v.y; return dot;}

inline double Vector2DF::Dist (const Vector2DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector2DF::Dist (const Vector2DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector2DF::Dist (const Vector2DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector2DF::Dist (const Vector3DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector2DF::Dist (const Vector3DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector2DF::Dist (const Vector3DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector2DF::Dist (const Vector4DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector2DF::DistSq (const Vector2DC &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
inline double Vector2DF::DistSq (const Vector2DI &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
inline double Vector2DF::DistSq (const Vector2DF &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
inline double Vector2DF::DistSq (const Vector3DC &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
inline double Vector2DF::DistSq (const Vector3DI &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
inline double Vector2DF::DistSq (const Vector3DF &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
inline double Vector2DF::DistSq (const Vector4DF &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}

inline Vector2DF &Vector2DF::Normalize (void) {
	double n = (double) x*x + (double) y*y;
	if (n!=0.0) {
		n = sqrt(n);
		x /= n;
		y /= n;
	}
	return *this;
}
inline double Vector2DF::Length (void) { double n; n = (double) x*x + (double) y*y; if (n != 0.0) return sqrt(n); return 0.0; }

inline VTYPE &Vector2DF::X(void)				{return x;}
inline VTYPE &Vector2DF::Y(void)				{return y;}
inline VTYPE Vector2DF::Z(void)					{return 0;}
inline VTYPE Vector2DF::W(void)					{return 0;}
inline const VTYPE &Vector2DF::X(void) const	{return x;}
inline const VTYPE &Vector2DF::Y(void) const	{return y;}
inline const VTYPE Vector2DF::Z(void) const		{return 0;}
inline const VTYPE Vector2DF::W(void) const		{return 0;}
inline VTYPE *Vector2DF::Data (void)			{return &x;}

#undef VTYPE
#undef VNAME

// Vector3DC Code Definition

#define VNAME		3DC
#define VTYPE		unsigned char

// Constructors/Destructors
inline Vector3DC::Vector3DC() {x=0; y=0; z=0;}
inline Vector3DC::~Vector3DC() {}
inline Vector3DC::Vector3DC (VTYPE xa, VTYPE ya, VTYPE za) {x=xa; y=ya; z=za;}
inline Vector3DC::Vector3DC (Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0;}
inline Vector3DC::Vector3DC (Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0;}
inline Vector3DC::Vector3DC (Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0;}
inline Vector3DC::Vector3DC (Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}
inline Vector3DC::Vector3DC (Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}
inline Vector3DC::Vector3DC (Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}
inline Vector3DC::Vector3DC (Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}

// Member Functions
inline Vector3DC &Vector3DC::Set (VTYPE xa, VTYPE ya, VTYPE za) {x=xa; y=ya; z=za; return *this;}

inline Vector3DC &Vector3DC::operator= (Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; return *this;}
inline Vector3DC &Vector3DC::operator= (Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; return *this;}
inline Vector3DC &Vector3DC::operator= (Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; return *this;}
inline Vector3DC &Vector3DC::operator= (Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}
inline Vector3DC &Vector3DC::operator= (Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}
inline Vector3DC &Vector3DC::operator= (Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}
inline Vector3DC &Vector3DC::operator= (Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}	
	
inline Vector3DC &Vector3DC::operator+= (Vector2DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector3DC &Vector3DC::operator+= (Vector2DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector3DC &Vector3DC::operator+= (Vector2DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector3DC &Vector3DC::operator+= (Vector3DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
inline Vector3DC &Vector3DC::operator+= (Vector3DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
inline Vector3DC &Vector3DC::operator+= (Vector3DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
inline Vector3DC &Vector3DC::operator+= (Vector4DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}

inline Vector3DC &Vector3DC::operator-= (Vector2DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector3DC &Vector3DC::operator-= (Vector2DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector3DC &Vector3DC::operator-= (Vector2DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector3DC &Vector3DC::operator-= (Vector3DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
inline Vector3DC &Vector3DC::operator-= (Vector3DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
inline Vector3DC &Vector3DC::operator-= (Vector3DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
inline Vector3DC &Vector3DC::operator-= (Vector4DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
	
inline Vector3DC &Vector3DC::operator*= (Vector2DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector3DC &Vector3DC::operator*= (Vector2DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector3DC &Vector3DC::operator*= (Vector2DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector3DC &Vector3DC::operator*= (Vector3DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
inline Vector3DC &Vector3DC::operator*= (Vector3DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
inline Vector3DC &Vector3DC::operator*= (Vector3DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
inline Vector3DC &Vector3DC::operator*= (Vector4DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}

inline Vector3DC &Vector3DC::operator/= (Vector2DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector3DC &Vector3DC::operator/= (Vector2DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector3DC &Vector3DC::operator/= (Vector2DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector3DC &Vector3DC::operator/= (Vector3DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
inline Vector3DC &Vector3DC::operator/= (Vector3DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
inline Vector3DC &Vector3DC::operator/= (Vector3DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
inline Vector3DC &Vector3DC::operator/= (Vector4DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}

inline Vector3DC &Vector3DC::Cross (Vector3DC &v) {double ax = x, ay = y, az = z; x = (VTYPE) (ay * (double) v.z - az * (double) v.y); y = (VTYPE) (-ax * (double) v.z + az * (double) v.x); z = (VTYPE) (ax * (double) v.y - ay * (double) v.x); return *this;}
inline Vector3DC &Vector3DC::Cross (Vector3DI &v) {double ax = x, ay = y, az = z; x = (VTYPE) (ay * (double) v.z - az * (double) v.y); y = (VTYPE) (-ax * (double) v.z + az * (double) v.x); z = (VTYPE) (ax * (double) v.y - ay * (double) v.x); return *this;}
inline Vector3DC &Vector3DC::Cross (Vector3DF &v) {double ax = x, ay = y, az = z; x = (VTYPE) (ay * (double) v.z - az * (double) v.y); y = (VTYPE) (-ax * (double) v.z + az * (double) v.x); z = (VTYPE) (ax * (double) v.y - ay * (double) v.x); return *this;}

inline double Vector3DC::Dot(Vector3DC &v)			{double dot; dot = (double) x*v.x + (double) y*v.y + (double) z*v.z; return dot;}
inline double Vector3DC::Dot(Vector3DI &v)			{double dot; dot = (double) x*v.x + (double) y*v.y + (double) z*v.z; return dot;}
inline double Vector3DC::Dot(Vector3DF &v)			{double dot; dot = (double) x*v.x + (double) y*v.y + (double) z*v.z; return dot;}

inline double Vector3DC::Dist (Vector2DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector3DC::Dist (Vector2DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector3DC::Dist (Vector2DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector3DC::Dist (Vector3DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector3DC::Dist (Vector3DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector3DC::Dist (Vector3DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector3DC::Dist (Vector4DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector3DC::DistSq (Vector2DC &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z; return (a*a + b*b + c*c);}
inline double Vector3DC::DistSq (Vector2DI &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z; return (a*a + b*b + c*c);}
inline double Vector3DC::DistSq (Vector2DF &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z; return (a*a + b*b + c*c);}
inline double Vector3DC::DistSq (Vector3DC &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}
inline double Vector3DC::DistSq (Vector3DI &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}
inline double Vector3DC::DistSq (Vector3DF &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}
inline double Vector3DC::DistSq (Vector4DF &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}

inline Vector3DC &Vector3DC::Normalize (void) {
	double n = (double) x*x + (double) y*y + (double) z*z;
	if (n!=0.0) {
		n = sqrt(n);
		x = (VTYPE) (((double) x*255)/n);
		y = (VTYPE) (((double) y*255)/n);
		z = (VTYPE) (((double) z*255)/n);
	}
	return *this;
}
inline double Vector3DC::Length (void) { double n; n = (double) x*x + (double) y*y + (double) z*z; if (n != 0.0) return sqrt(n); return 0.0; }

inline VTYPE &Vector3DC::X(void)				{return x;}
inline VTYPE &Vector3DC::Y(void)				{return y;}
inline VTYPE &Vector3DC::Z(void)				{return z;}
inline VTYPE Vector3DC::W(void)					{return 0;}
inline const VTYPE &Vector3DC::X(void) const	{return x;}
inline const VTYPE &Vector3DC::Y(void) const	{return y;}
inline const VTYPE &Vector3DC::Z(void) const	{return z;}
inline const VTYPE Vector3DC::W(void) const		{return 0;}
inline VTYPE *Vector3DC::Data (void)			{return &x;}

#undef VTYPE
#undef VNAME

// Vector3DI Code Definition

#define VNAME		3DI
#define VTYPE		int

// Constructors/Destructors
inline Vector3DI::Vector3DI() {x=0; y=0; z=0;}
inline Vector3DI::~Vector3DI() {}
inline Vector3DI::Vector3DI (VTYPE xa, VTYPE ya, VTYPE za) {x=xa; y=ya; z=za;}
inline Vector3DI::Vector3DI (Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0;}
inline Vector3DI::Vector3DI (Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0;}
inline Vector3DI::Vector3DI (Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0;}
inline Vector3DI::Vector3DI (Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}
inline Vector3DI::Vector3DI (Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}
inline Vector3DI::Vector3DI (Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}
inline Vector3DI::Vector3DI (Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}

// Set Functions
inline Vector3DI &Vector3DI::Set (const int xa, const int ya, const int za)
{
	x = xa; y = ya; z = za;
	return *this;
}

// Member Functions
inline Vector3DI &Vector3DI::operator= (Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector3DI &Vector3DI::operator= (Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector3DI &Vector3DI::operator= (Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector3DI &Vector3DI::operator= (Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}
inline Vector3DI &Vector3DI::operator= (Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}
inline Vector3DI &Vector3DI::operator= (Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}
inline Vector3DI &Vector3DI::operator= (Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}	
	
inline Vector3DI &Vector3DI::operator+= (Vector2DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector3DI &Vector3DI::operator+= (Vector2DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector3DI &Vector3DI::operator+= (Vector2DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector3DI &Vector3DI::operator+= (Vector3DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
inline Vector3DI &Vector3DI::operator+= (Vector3DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
inline Vector3DI &Vector3DI::operator+= (Vector3DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
inline Vector3DI &Vector3DI::operator+= (Vector4DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}

inline Vector3DI &Vector3DI::operator-= (Vector2DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector3DI &Vector3DI::operator-= (Vector2DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector3DI &Vector3DI::operator-= (Vector2DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector3DI &Vector3DI::operator-= (Vector3DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
inline Vector3DI &Vector3DI::operator-= (Vector3DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
inline Vector3DI &Vector3DI::operator-= (Vector3DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
inline Vector3DI &Vector3DI::operator-= (Vector4DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
	
inline Vector3DI &Vector3DI::operator*= (Vector2DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector3DI &Vector3DI::operator*= (Vector2DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector3DI &Vector3DI::operator*= (Vector2DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector3DI &Vector3DI::operator*= (Vector3DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
inline Vector3DI &Vector3DI::operator*= (Vector3DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
inline Vector3DI &Vector3DI::operator*= (Vector3DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
inline Vector3DI &Vector3DI::operator*= (Vector4DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}

inline Vector3DI &Vector3DI::operator/= (Vector2DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector3DI &Vector3DI::operator/= (Vector2DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector3DI &Vector3DI::operator/= (Vector2DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector3DI &Vector3DI::operator/= (Vector3DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
inline Vector3DI &Vector3DI::operator/= (Vector3DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
inline Vector3DI &Vector3DI::operator/= (Vector3DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
inline Vector3DI &Vector3DI::operator/= (Vector4DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}

inline Vector3DI &Vector3DI::Cross (Vector3DC &v) {double ax = x, ay = y, az = z; x = (VTYPE) (ay * (double) v.z - az * (double) v.y); y = (VTYPE) (-ax * (double) v.z + az * (double) v.x); z = (VTYPE) (ax * (double) v.y - ay * (double) v.x); return *this;}
inline Vector3DI &Vector3DI::Cross (Vector3DI &v) {double ax = x, ay = y, az = z; x = (VTYPE) (ay * (double) v.z - az * (double) v.y); y = (VTYPE) (-ax * (double) v.z + az * (double) v.x); z = (VTYPE) (ax * (double) v.y - ay * (double) v.x); return *this;}
inline Vector3DI &Vector3DI::Cross (Vector3DF &v) {double ax = x, ay = y, az = z; x = (VTYPE) (ay * (double) v.z - az * (double) v.y); y = (VTYPE) (-ax * (double) v.z + az * (double) v.x); z = (VTYPE) (ax * (double) v.y - ay * (double) v.x); return *this;}
		
inline double Vector3DI::Dot(Vector3DC &v)			{double dot; dot = (double) x*v.x + (double) y*v.y + (double) z*v.z; return dot;}
inline double Vector3DI::Dot(Vector3DI &v)			{double dot; dot = (double) x*v.x + (double) y*v.y + (double) z*v.z; return dot;}
inline double Vector3DI::Dot(Vector3DF &v)			{double dot; dot = (double) x*v.x + (double) y*v.y + (double) z*v.z; return dot;}

inline double Vector3DI::Dist (Vector2DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector3DI::Dist (Vector2DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector3DI::Dist (Vector2DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector3DI::Dist (Vector3DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector3DI::Dist (Vector3DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector3DI::Dist (Vector3DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector3DI::Dist (Vector4DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector3DI::DistSq (Vector2DC &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z; return (a*a + b*b + c*c);}
inline double Vector3DI::DistSq (Vector2DI &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z; return (a*a + b*b + c*c);}
inline double Vector3DI::DistSq (Vector2DF &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z; return (a*a + b*b + c*c);}
inline double Vector3DI::DistSq (Vector3DC &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}
inline double Vector3DI::DistSq (Vector3DI &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}
inline double Vector3DI::DistSq (Vector3DF &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}
inline double Vector3DI::DistSq (Vector4DF &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}

inline Vector3DI &Vector3DI::Normalize (void) {
	double n = (double) x*x + (double) y*y + (double) z*z;
	if (n!=0.0) {
		n = sqrt(n);
		x = (VTYPE) (((double) x*255)/n);
		y = (VTYPE) (((double) y*255)/n);
		z = (VTYPE) (((double) z*255)/n);
	}
	return *this;
}
inline double Vector3DI::Length (void) { double n; n = (double) x*x + (double) y*y + (double) z*z; if (n != 0.0) return sqrt(n); return 0.0; }

inline VTYPE &Vector3DI::X(void)				{return x;}
inline VTYPE &Vector3DI::Y(void)				{return y;}
inline VTYPE &Vector3DI::Z(void)				{return z;}
inline VTYPE Vector3DI::W(void)					{return 0;}
inline const VTYPE &Vector3DI::X(void) const	{return x;}
inline const VTYPE &Vector3DI::Y(void) const	{return y;}
inline const VTYPE &Vector3DI::Z(void) const	{return z;}
inline const VTYPE Vector3DI::W(void) const		{return 0;}
inline VTYPE *Vector3DI::Data (void)			{return &x;}

#undef VTYPE
#undef VNAME

// Vector3DF Code Definition

#define VNAME		3DF
#define VTYPE		double

// Constructors/Destructors
inline Vector3DF::Vector3DF() {x=0; y=0; z=0;}
inline Vector3DF::~Vector3DF() {}
inline Vector3DF::Vector3DF (const VTYPE xa, const VTYPE ya, const VTYPE za) {x=xa; y=ya; z=za;}
inline Vector3DF::Vector3DF (const Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0;}
inline Vector3DF::Vector3DF (const Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0;}
inline Vector3DF::Vector3DF (const Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0;}
inline Vector3DF::Vector3DF (const Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}
inline Vector3DF::Vector3DF (const Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}
inline Vector3DF::Vector3DF (const Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}
inline Vector3DF::Vector3DF (const Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}

// Set Functions
inline Vector3DF &Vector3DF::Set (const double xa, const double ya, const double za)
{
	x = xa; y = ya; z = za;
	return *this;
}

// Member Functions
inline Vector3DF &Vector3DF::operator= (const int op) {x= (VTYPE) op; y= (VTYPE) op; z= (VTYPE) op; return *this;}
inline Vector3DF &Vector3DF::operator= (const double op) {x= (VTYPE) op; y= (VTYPE) op; z= (VTYPE) op; return *this;}
inline Vector3DF &Vector3DF::operator= (const Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector3DF &Vector3DF::operator= (const Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector3DF &Vector3DF::operator= (const Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
inline Vector3DF &Vector3DF::operator= (const Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}
inline Vector3DF &Vector3DF::operator= (const Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}
inline Vector3DF &Vector3DF::operator= (const Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}
inline Vector3DF &Vector3DF::operator= (const Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}	
	
inline Vector3DF &Vector3DF::operator+= (const int op) {x+= (VTYPE) op; y+= (VTYPE) op; z+= (VTYPE) op; return *this;}
inline Vector3DF &Vector3DF::operator+= (const double op) {x+= (VTYPE) op; y+= (VTYPE) op; z+= (VTYPE) op; return *this;}
inline Vector3DF &Vector3DF::operator+= (const Vector2DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector3DF &Vector3DF::operator+= (const Vector2DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector3DF &Vector3DF::operator+= (const Vector2DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector3DF &Vector3DF::operator+= (const Vector3DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
inline Vector3DF &Vector3DF::operator+= (const Vector3DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
inline Vector3DF &Vector3DF::operator+= (const Vector3DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
inline Vector3DF &Vector3DF::operator+= (const Vector4DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}

inline Vector3DF &Vector3DF::operator-= (const int op) {x-= (VTYPE) op; y-= (VTYPE) op; z-= (VTYPE) op; return *this;}
inline Vector3DF &Vector3DF::operator-= (const double op) {x-= (VTYPE) op; y-= (VTYPE) op; z-= (VTYPE) op; return *this;}
inline Vector3DF &Vector3DF::operator-= (const Vector2DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector3DF &Vector3DF::operator-= (const Vector2DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector3DF &Vector3DF::operator-= (const Vector2DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector3DF &Vector3DF::operator-= (const Vector3DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
inline Vector3DF &Vector3DF::operator-= (const Vector3DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
inline Vector3DF &Vector3DF::operator-= (const Vector3DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
inline Vector3DF &Vector3DF::operator-= (const Vector4DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
	
inline Vector3DF &Vector3DF::operator*= (const int op) {x*= (VTYPE) op; y*= (VTYPE) op; z*= (VTYPE) op; return *this;}
inline Vector3DF &Vector3DF::operator*= (const double op) {x*= (VTYPE) op; y*= (VTYPE) op; z*= (VTYPE) op; return *this;}
inline Vector3DF &Vector3DF::operator*= (const Vector2DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector3DF &Vector3DF::operator*= (const Vector2DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector3DF &Vector3DF::operator*= (const Vector2DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector3DF &Vector3DF::operator*= (const Vector3DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
inline Vector3DF &Vector3DF::operator*= (const Vector3DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
inline Vector3DF &Vector3DF::operator*= (const Vector3DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
inline Vector3DF &Vector3DF::operator*= (const Vector4DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}

inline Vector3DF &Vector3DF::operator/= (const int op) {x/= (VTYPE) op; y/= (VTYPE) op; z/= (VTYPE) op; return *this;}
inline Vector3DF &Vector3DF::operator/= (const double op) {x/= (VTYPE) op; y/= (VTYPE) op; z/= (VTYPE) op; return *this;}
inline Vector3DF &Vector3DF::operator/= (const Vector2DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector3DF &Vector3DF::operator/= (const Vector2DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector3DF &Vector3DF::operator/= (const Vector2DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector3DF &Vector3DF::operator/= (const Vector3DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
inline Vector3DF &Vector3DF::operator/= (const Vector3DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
inline Vector3DF &Vector3DF::operator/= (const Vector3DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
inline Vector3DF &Vector3DF::operator/= (const Vector4DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}

inline Vector3DF &Vector3DF::Cross (const Vector3DC &v) {double ax = x, ay = y, az = z; x = (VTYPE) (ay * (double) v.z - az * (double) v.y); y = (VTYPE) (-ax * (double) v.z + az * (double) v.x); z = (VTYPE) (ax * (double) v.y - ay * (double) v.x); return *this;}
inline Vector3DF &Vector3DF::Cross (const Vector3DI &v) {double ax = x, ay = y, az = z; x = (VTYPE) (ay * (double) v.z - az * (double) v.y); y = (VTYPE) (-ax * (double) v.z + az * (double) v.x); z = (VTYPE) (ax * (double) v.y - ay * (double) v.x); return *this;}
inline Vector3DF &Vector3DF::Cross (const Vector3DF &v) {double ax = x, ay = y, az = z; x = (VTYPE) (ay * (double) v.z - az * (double) v.y); y = (VTYPE) (-ax * (double) v.z + az * (double) v.x); z = (VTYPE) (ax * (double) v.y - ay * (double) v.x); return *this;}
		
inline double Vector3DF::Dot(const Vector3DC &v)			{double dot; dot = (double) x*v.x + (double) y*v.y + (double) z*v.z; return dot;}
inline double Vector3DF::Dot(const Vector3DI &v)			{double dot; dot = (double) x*v.x + (double) y*v.y + (double) z*v.z; return dot;}
inline double Vector3DF::Dot(const Vector3DF &v)			{double dot; dot = (double) x*v.x + (double) y*v.y + (double) z*v.z; return dot;}

inline double Vector3DF::Dist (const Vector2DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector3DF::Dist (const Vector2DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector3DF::Dist (const Vector2DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector3DF::Dist (const Vector3DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector3DF::Dist (const Vector3DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector3DF::Dist (const Vector3DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector3DF::Dist (const Vector4DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
inline double Vector3DF::DistSq (const Vector2DC &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z; return (a*a + b*b + c*c);}
inline double Vector3DF::DistSq (const Vector2DI &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z; return (a*a + b*b + c*c);}
inline double Vector3DF::DistSq (const Vector2DF &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z; return (a*a + b*b + c*c);}
inline double Vector3DF::DistSq (const Vector3DC &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}
inline double Vector3DF::DistSq (const Vector3DI &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}
inline double Vector3DF::DistSq (const Vector3DF &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}
inline double Vector3DF::DistSq (const Vector4DF &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}

inline Vector3DF &Vector3DF::Normalize (void) {
	double n = (double) x*x + (double) y*y + (double) z*z;
	if (n!=0.0) {
		n = sqrt(n);
		x /= n; y /= n; z /= n;
	}
	return *this;
}
inline double Vector3DF::Length (void) { double n; n = (double) x*x + (double) y*y + (double) z*z; if (n != 0.0) return sqrt(n); return 0.0; }

inline VTYPE &Vector3DF::X()				{return x;}
inline VTYPE &Vector3DF::Y()				{return y;}
inline VTYPE &Vector3DF::Z()				{return z;}
inline VTYPE Vector3DF::W()					{return 0;}
inline const VTYPE &Vector3DF::X() const	{return x;}
inline const VTYPE &Vector3DF::Y() const	{return y;}
inline const VTYPE &Vector3DF::Z() const	{return z;}
inline const VTYPE Vector3DF::W() const		{return 0;}
inline VTYPE *Vector3DF::Data ()			{return &x;}

#undef VTYPE
#undef VNAME

// Vector4DF Code Definition

#define VNAME		4DF
#define VTYPE		double

// Constructors/Destructors
inline Vector4DF::Vector4DF() {x=0; y=0; z=0; w=0;}
inline Vector4DF::~Vector4DF() {}
inline Vector4DF::Vector4DF (VTYPE xa, VTYPE ya, VTYPE za, VTYPE wa) {x=xa; y=ya; z=za; w=wa;}
inline Vector4DF::Vector4DF (Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; w=(VTYPE) 0;}
inline Vector4DF::Vector4DF (Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; w=(VTYPE) 0;}
inline Vector4DF::Vector4DF (Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; w=(VTYPE) 0;}
inline Vector4DF::Vector4DF (Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) 0;}
inline Vector4DF::Vector4DF (Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) 0;}
inline Vector4DF::Vector4DF (Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) 0;}
inline Vector4DF::Vector4DF (Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) op.w;}

// Member Functions
inline Vector4DF &Vector4DF::operator= (int op) {x= (VTYPE) op; y= (VTYPE) op; z= (VTYPE) op; w = (VTYPE) op; return *this;}
inline Vector4DF &Vector4DF::operator= (double op) {x= (VTYPE) op; y= (VTYPE) op; z= (VTYPE) op; w = (VTYPE) op; return *this;}
inline Vector4DF &Vector4DF::operator= (Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; w=(VTYPE) 0; return *this;}
inline Vector4DF &Vector4DF::operator= (Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; w=(VTYPE) 0;  return *this;}
inline Vector4DF &Vector4DF::operator= (Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; w=(VTYPE) 0;  return *this;}
inline Vector4DF &Vector4DF::operator= (Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) 0;  return *this;}
inline Vector4DF &Vector4DF::operator= (Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) 0;  return *this;}
inline Vector4DF &Vector4DF::operator= (Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) 0; return *this;}
inline Vector4DF &Vector4DF::operator= (Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) op.w; return *this;}	
	
inline Vector4DF &Vector4DF::operator+= (int op) {x+= (VTYPE) op; y+= (VTYPE) op; z+= (VTYPE) op; w += (VTYPE) op; return *this;}
inline Vector4DF &Vector4DF::operator+= (double op) {x+= (VTYPE) op; y+= (VTYPE) op; z+= (VTYPE) op; w += (VTYPE) op; return *this;}
inline Vector4DF &Vector4DF::operator+= (Vector2DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector4DF &Vector4DF::operator+= (Vector2DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector4DF &Vector4DF::operator+= (Vector2DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
inline Vector4DF &Vector4DF::operator+= (Vector3DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
inline Vector4DF &Vector4DF::operator+= (Vector3DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
inline Vector4DF &Vector4DF::operator+= (Vector3DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
inline Vector4DF &Vector4DF::operator+= (Vector4DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; w+=(VTYPE) op.w; return *this;}	

inline Vector4DF &Vector4DF::operator-= (int op) {x-= (VTYPE) op; y-= (VTYPE) op; z-= (VTYPE) op; w -= (VTYPE) op; return *this;}
inline Vector4DF &Vector4DF::operator-= (double op) {x-= (VTYPE) op; y-= (VTYPE) op; z-= (VTYPE) op; w -= (VTYPE) op; return *this;}
inline Vector4DF &Vector4DF::operator-= (Vector2DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector4DF &Vector4DF::operator-= (Vector2DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector4DF &Vector4DF::operator-= (Vector2DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
inline Vector4DF &Vector4DF::operator-= (Vector3DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
inline Vector4DF &Vector4DF::operator-= (Vector3DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
inline Vector4DF &Vector4DF::operator-= (Vector3DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
inline Vector4DF &Vector4DF::operator-= (Vector4DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; w-=(VTYPE) op.w; return *this;}	

inline Vector4DF &Vector4DF::operator*= (int op) {x*= (VTYPE) op; y*= (VTYPE) op; z*= (VTYPE) op; w *= (VTYPE) op; return *this;}
inline Vector4DF &Vector4DF::operator*= (double op) {x*= (VTYPE) op; y*= (VTYPE) op; z*= (VTYPE) op; w *= (VTYPE) op; return *this;}
inline Vector4DF &Vector4DF::operator*= (Vector2DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector4DF &Vector4DF::operator*= (Vector2DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector4DF &Vector4DF::operator*= (Vector2DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
inline Vector4DF &Vector4DF::operator*= (Vector3DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
inline Vector4DF &Vector4DF::operator*= (Vector3DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
inline Vector4DF &Vector4DF::operator*= (Vector3DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
inline Vector4DF &Vector4DF::operator*= (Vector4DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; w*=(VTYPE) op.w; return *this;}	

inline Vector4DF &Vector4DF::operator/= (int op) {x/= (VTYPE) op; y/= (VTYPE) op; z/= (VTYPE) op; w /= (VTYPE) op; return *this;}
inline Vector4DF &Vector4DF::operator/= (double op) {x/= (VTYPE) op; y/= (VTYPE) op; z/= (VTYPE) op; w /= (VTYPE) op; return *this;}
inline Vector4DF &Vector4DF::operator/= (Vector2DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector4DF &Vector4DF::operator/= (Vector2DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector4DF &Vector4DF::operator/= (Vector2DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
inline Vector4DF &Vector4DF::operator/= (Vector3DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
inline Vector4DF &Vector4DF::operator/= (Vector3DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
inline Vector4DF &Vector4DF::operator/= (Vector3DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
inline Vector4DF &Vector4DF::operator/= (Vector4DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; w/=(VTYPE) op.w; return *this;}	

inline Vector4DF &Vector4DF::Cross (Vector4DF &v) {double ax = x, ay = y, az = z, aw = w; x = (VTYPE) (ay * (double) v.z - az * (double) v.y); y = (VTYPE) (-ax * (double) v.z + az * (double) v.x); z = (VTYPE) (ax * (double) v.y - ay * (double) v.x); w = (VTYPE) 0; return *this;}
		
inline double Vector4DF::Dot(Vector4DF &v)			{double dot; dot = (double) x*v.x + (double) y*v.y + (double) z*v.z + (double) w*v.w; return dot;}

inline double Vector4DF::Dist (Vector4DF &v)		{double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}

inline double Vector4DF::DistSq (Vector4DF &v)		{double a,b,c,d; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; d = (double) w - (double) v.w; return (a*a + b*b + c*c + d*d);}

inline Vector4DF &Vector4DF::Normalize (void) {
	double n = (double) x*x + (double) y*y + (double) z*z + (double) w*w;
	if (n!=0.0) {
		n = sqrt(n);
		x /= n; y /= n; z /= n; w /= n;
	}
	return *this;
}
inline double Vector4DF::Length (void) { double n; n = (double) x*x + (double) y*y + (double) z*z + (double) w*w; if (n != 0.0) return sqrt(n); return 0.0; }

inline VTYPE &Vector4DF::X(void)				{return x;}
inline VTYPE &Vector4DF::Y(void)				{return y;}
inline VTYPE &Vector4DF::Z(void)				{return z;}
inline VTYPE &Vector4DF::W(void)				{return w;}
inline const VTYPE &Vector4DF::X(void) const	{return x;}
inline const VTYPE &Vector4DF::Y(void) const	{return y;}
inline const VTYPE &Vector4DF::Z(void) const	{return z;}
inline const VTYPE &Vector4DF::W(void) const	{return w;}
inline VTYPE *Vector4DF::Data (void)			{return &x;}

#undef VTYPE
#undef VNAME
