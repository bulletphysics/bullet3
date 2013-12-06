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
// ** NOTES **
// Vector code CANNOT be inlined in header file because of dependencies
//    across vector classes (error generated: "Use of undeclared class..")
// 
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <memory.h>
#include <math.h>

#ifndef VECTOR_DEF
	#define VECTOR_DEF

	//#define VECTOR_INITIALIZE				// Initializes vectors	
																
	class Vector2DC;						// Forward Referencing
	class Vector2DI;
	class Vector2DF;
	class Vector3DC;
	class Vector3DI;
	class Vector3DF;
	class Vector4DF;	
	class MatrixF;
	class Matrix4F;

	// Vector2DC Declaration
	
	#define VNAME		2DC
	#define VTYPE		unsigned char

	class Vector2DC {
	public:
		VTYPE x, y;

		// Constructors/Destructors
		inline Vector2DC();
		inline ~Vector2DC();
		inline Vector2DC (VTYPE xa, VTYPE ya);
		inline Vector2DC (Vector2DC &op);	
		inline Vector2DC (Vector2DI &op);	
		inline Vector2DC (Vector2DF &op);	
		inline Vector2DC (Vector3DC &op);	
		inline Vector2DC (Vector3DI &op);	
		inline Vector2DC (Vector3DF &op);	
		inline Vector2DC (Vector4DF &op);

		// Member Functions
		inline Vector2DC &operator= (Vector2DC &op);
		inline Vector2DC &operator= (Vector2DI &op);
		inline Vector2DC &operator= (Vector2DF &op);
		inline Vector2DC &operator= (Vector3DC &op);
		inline Vector2DC &operator= (Vector3DI &op);
		inline Vector2DC &operator= (Vector3DF &op);
		inline Vector2DC &operator= (Vector4DF &op);
		
		inline Vector2DC &operator+= (Vector2DC &op);
		inline Vector2DC &operator+= (Vector2DI &op);
		inline Vector2DC &operator+= (Vector2DF &op);
		inline Vector2DC &operator+= (Vector3DC &op);
		inline Vector2DC &operator+= (Vector3DI &op);
		inline Vector2DC &operator+= (Vector3DF &op);
		inline Vector2DC &operator+= (Vector4DF &op);

		inline Vector2DC &operator-= (Vector2DC &op);
		inline Vector2DC &operator-= (Vector2DI &op);
		inline Vector2DC &operator-= (Vector2DF &op);
		inline Vector2DC &operator-= (Vector3DC &op);
		inline Vector2DC &operator-= (Vector3DI &op);
		inline Vector2DC &operator-= (Vector3DF &op);
		inline Vector2DC &operator-= (Vector4DF &op);
	
		inline Vector2DC &operator*= (Vector2DC &op);
		inline Vector2DC &operator*= (Vector2DI &op);
		inline Vector2DC &operator*= (Vector2DF &op);
		inline Vector2DC &operator*= (Vector3DC &op);
		inline Vector2DC &operator*= (Vector3DI &op);
		inline Vector2DC &operator*= (Vector3DF &op);
		inline Vector2DC &operator*= (Vector4DF &op);

		inline Vector2DC &operator/= (Vector2DC &op);
		inline Vector2DC &operator/= (Vector2DI &op);
		inline Vector2DC &operator/= (Vector2DF &op);
		inline Vector2DC &operator/= (Vector3DC &op);
		inline Vector2DC &operator/= (Vector3DI &op);
		inline Vector2DC &operator/= (Vector3DF &op);
		inline Vector2DC &operator/= (Vector4DF &op);

		// Note: Cross product does not exist for 2D vectors (only 3D)
		
		inline double Dot(Vector2DC &v);
		inline double Dot(Vector2DI &v);
		inline double Dot(Vector2DF &v);

		inline double Dist (Vector2DC &v);
		inline double Dist (Vector2DI &v);
		inline double Dist (Vector2DF &v);
		inline double Dist (Vector3DC &v);
		inline double Dist (Vector3DI &v);
		inline double Dist (Vector3DF &v);
		inline double Dist (Vector4DF &v);

		inline double DistSq (Vector2DC &v);		
		inline double DistSq (Vector2DI &v);		
		inline double DistSq (Vector2DF &v);		
		inline double DistSq (Vector3DC &v);		
		inline double DistSq (Vector3DI &v);		
		inline double DistSq (Vector3DF &v);		
		inline double DistSq (Vector4DF &v);

		inline Vector2DC &Normalize (void);
		inline double Length (void);

		inline VTYPE &X(void);
		inline VTYPE &Y(void);
		inline VTYPE Z(void);
		inline VTYPE W(void);
		inline const VTYPE &X(void) const;			
		inline const VTYPE &Y(void) const;			
		inline const VTYPE Z(void) const;			
		inline const VTYPE W(void) const;
		inline VTYPE *Data (void);
	};
	
	#undef VNAME
	#undef VTYPE

	// Vector2DI Declaration

	#define VNAME		2DI
	#define VTYPE		int

	class Vector2DI {
	public:
		VTYPE x, y;

		// Constructors/Destructors
		inline Vector2DI();							
		inline ~Vector2DI();			
		inline Vector2DI (VTYPE xa, VTYPE ya);
		inline Vector2DI (Vector2DC &op);				
		inline Vector2DI (Vector2DI &op);				
		inline Vector2DI (Vector2DF &op);				
		inline Vector2DI (Vector3DC &op);				
		inline Vector2DI (Vector3DI &op);				
		inline Vector2DI (Vector3DF &op);				
		inline Vector2DI (Vector4DF &op);

		// Member Functions
		inline Vector2DI &operator= (Vector2DC &op);
		inline Vector2DI &operator= (Vector2DI &op);
		inline Vector2DI &operator= (Vector2DF &op);
		inline Vector2DI &operator= (Vector3DC &op);
		inline Vector2DI &operator= (Vector3DI &op);
		inline Vector2DI &operator= (Vector3DF &op);
		inline Vector2DI &operator= (Vector4DF &op);

		inline Vector2DI &operator+= (Vector2DC &op);
		inline Vector2DI &operator+= (Vector2DI &op);
		inline Vector2DI &operator+= (Vector2DF &op);
		inline Vector2DI &operator+= (Vector3DC &op);
		inline Vector2DI &operator+= (Vector3DI &op);
		inline Vector2DI &operator+= (Vector3DF &op);
		inline Vector2DI &operator+= (Vector4DF &op);

		inline Vector2DI &operator-= (Vector2DC &op);
		inline Vector2DI &operator-= (Vector2DI &op);
		inline Vector2DI &operator-= (Vector2DF &op);
		inline Vector2DI &operator-= (Vector3DC &op);
		inline Vector2DI &operator-= (Vector3DI &op);
		inline Vector2DI &operator-= (Vector3DF &op);
		inline Vector2DI &operator-= (Vector4DF &op);
	
		inline Vector2DI &operator*= (Vector2DC &op);
		inline Vector2DI &operator*= (Vector2DI &op);
		inline Vector2DI &operator*= (Vector2DF &op);
		inline Vector2DI &operator*= (Vector3DC &op);
		inline Vector2DI &operator*= (Vector3DI &op);
		inline Vector2DI &operator*= (Vector3DF &op);
		inline Vector2DI &operator*= (Vector4DF &op);

		inline Vector2DI &operator/= (Vector2DC &op);
		inline Vector2DI &operator/= (Vector2DI &op);
		inline Vector2DI &operator/= (Vector2DF &op);
		inline Vector2DI &operator/= (Vector3DC &op);
		inline Vector2DI &operator/= (Vector3DI &op);
		inline Vector2DI &operator/= (Vector3DF &op);
		inline Vector2DI &operator/= (Vector4DF &op);


		// Note: Cross product does not exist for 2D vectors (only 3D)
		
		inline double Dot(Vector2DC &v);
		inline double Dot(Vector2DI &v);
		inline double Dot(Vector2DF &v);

		inline double Dist (Vector2DC &v);
		inline double Dist (Vector2DI &v);
		inline double Dist (Vector2DF &v);
		inline double Dist (Vector3DC &v);
		inline double Dist (Vector3DI &v);
		inline double Dist (Vector3DF &v);
		inline double Dist (Vector4DF &v);

		inline double DistSq (Vector2DC &v);
		inline double DistSq (Vector2DI &v);
		inline double DistSq (Vector2DF &v);
		inline double DistSq (Vector3DC &v);
		inline double DistSq (Vector3DI &v);
		inline double DistSq (Vector3DF &v);
		inline double DistSq (Vector4DF &v);
		
		inline Vector2DI &Normalize (void);
		inline double Length (void);

		inline VTYPE &X(void);
		inline VTYPE &Y(void);
		inline VTYPE Z(void);
		inline VTYPE W(void);
		inline const VTYPE &X(void) const;
		inline const VTYPE &Y(void) const;
		inline const VTYPE Z(void) const;
		inline const VTYPE W(void) const;
		inline VTYPE *Data (void);
	};
	
	#undef VNAME
	#undef VTYPE

	// Vector2DF Declarations

	#define VNAME		2DF
	#define VTYPE		double

	class Vector2DF {
	public:
		VTYPE x, y;

		// Constructors/Destructors
		 Vector2DF ();
		 ~Vector2DF ();
		 Vector2DF (const VTYPE xa, const VTYPE ya);
		 Vector2DF (const Vector2DC &op);
		 Vector2DF (const Vector2DI &op);
		 Vector2DF (const Vector2DF &op);
		 Vector2DF (const Vector3DC &op);
		 Vector2DF (const Vector3DI &op);
		 Vector2DF (const Vector3DF &op);
		 Vector2DF (const Vector4DF &op);

		// Member Functions
		 Vector2DF &operator= (const Vector2DC &op);
		 Vector2DF &operator= (const Vector2DI &op);
		 Vector2DF &operator= (const Vector2DF &op);
		 Vector2DF &operator= (const Vector3DC &op);
		 Vector2DF &operator= (const Vector3DI &op);
		 Vector2DF &operator= (const Vector3DF &op);
		 Vector2DF &operator= (const Vector4DF &op);
		
		 Vector2DF &operator+= (const Vector2DC &op);
		 Vector2DF &operator+= (const Vector2DI &op);
		 Vector2DF &operator+= (const Vector2DF &op);
		 Vector2DF &operator+= (const Vector3DC &op);
		 Vector2DF &operator+= (const Vector3DI &op);
		 Vector2DF &operator+= (const Vector3DF &op);
		 Vector2DF &operator+= (const Vector4DF &op);

		 Vector2DF &operator-= (const Vector2DC &op);
		 Vector2DF &operator-= (const Vector2DI &op);
		 Vector2DF &operator-= (const Vector2DF &op);
		 Vector2DF &operator-= (const Vector3DC &op);
		 Vector2DF &operator-= (const Vector3DI &op);
		 Vector2DF &operator-= (const Vector3DF &op);
		 Vector2DF &operator-= (const Vector4DF &op);

		 Vector2DF &operator*= (const Vector2DC &op);
		 Vector2DF &operator*= (const Vector2DI &op);
		 Vector2DF &operator*= (const Vector2DF &op);
		 Vector2DF &operator*= (const Vector3DC &op);
		 Vector2DF &operator*= (const Vector3DI &op);
		 Vector2DF &operator*= (const Vector3DF &op);
		 Vector2DF &operator*= (const Vector4DF &op);

		 Vector2DF &operator/= (const Vector2DC &op);
		 Vector2DF &operator/= (const Vector2DI &op);
		 Vector2DF &operator/= (const Vector2DF &op);
		 Vector2DF &operator/= (const Vector3DC &op);
		 Vector2DF &operator/= (const Vector3DI &op);
		 Vector2DF &operator/= (const Vector3DF &op);
		 Vector2DF &operator/= (const Vector4DF &op);

		 Vector2DF &operator/= (const double v)		{x /= v; y /= v; return *this;}

		// Note: Cross product does not exist for 2D vectors (only 3D)
		
		 double Dot(const Vector2DC &v);
		 double Dot(const Vector2DI &v);
		 double Dot(const Vector2DF &v);

		 double Dist (const Vector2DC &v);
		 double Dist (const Vector2DI &v);
		 double Dist (const Vector2DF &v);
		 double Dist (const Vector3DC &v);
		 double Dist (const Vector3DI &v);
		 double Dist (const Vector3DF &v);
		 double Dist (const Vector4DF &v);

		 double DistSq (const Vector2DC &v);
		 double DistSq (const Vector2DI &v);
		 double DistSq (const Vector2DF &v);
		 double DistSq (const Vector3DC &v);
		 double DistSq (const Vector3DI &v);
		 double DistSq (const Vector3DF &v);
		 double DistSq (const Vector4DF &v);

		 Vector2DF &Normalize (void);
		 double Length (void);

		 VTYPE &X(void);
		 VTYPE &Y(void);
		 VTYPE Z(void);
		 VTYPE W(void);
		 const VTYPE &X(void) const;
		 const VTYPE &Y(void) const;
		 const VTYPE Z(void) const;
		 const VTYPE W(void) const;
		 VTYPE *Data (void);
	};
	
	#undef VNAME
	#undef VTYPE

	// Vector3DC Declaration
	
	#define VNAME		3DC
	#define VTYPE		unsigned char

	class Vector3DC {
	public:	
		VTYPE x, y, z;
	
		// Constructors/Destructors
		inline Vector3DC();
		inline ~Vector3DC();
		inline Vector3DC (VTYPE xa, VTYPE ya, VTYPE za);
		inline Vector3DC (Vector2DC &op);
		inline Vector3DC (Vector2DI &op);
		inline Vector3DC (Vector2DF &op);
		inline Vector3DC (Vector3DC &op);
		inline Vector3DC (Vector3DI &op);
		inline Vector3DC (Vector3DF &op);
		inline Vector3DC (Vector4DF &op);

		// Member Functions
		inline Vector3DC &Set (VTYPE xa, VTYPE ya, VTYPE za);
		
		inline Vector3DC &operator= (Vector2DC &op);
		inline Vector3DC &operator= (Vector2DI &op);
		inline Vector3DC &operator= (Vector2DF &op);
		inline Vector3DC &operator= (Vector3DC &op);
		inline Vector3DC &operator= (Vector3DI &op);
		inline Vector3DC &operator= (Vector3DF &op);
		inline Vector3DC &operator= (Vector4DF &op);
		
		inline Vector3DC &operator+= (Vector2DC &op);
		inline Vector3DC &operator+= (Vector2DI &op);
		inline Vector3DC &operator+= (Vector2DF &op);
		inline Vector3DC &operator+= (Vector3DC &op);
		inline Vector3DC &operator+= (Vector3DI &op);
		inline Vector3DC &operator+= (Vector3DF &op);
		inline Vector3DC &operator+= (Vector4DF &op);

		inline Vector3DC &operator-= (Vector2DC &op);
		inline Vector3DC &operator-= (Vector2DI &op);
		inline Vector3DC &operator-= (Vector2DF &op);
		inline Vector3DC &operator-= (Vector3DC &op);
		inline Vector3DC &operator-= (Vector3DI &op);
		inline Vector3DC &operator-= (Vector3DF &op);
		inline Vector3DC &operator-= (Vector4DF &op);
	
		inline Vector3DC &operator*= (Vector2DC &op);
		inline Vector3DC &operator*= (Vector2DI &op);
		inline Vector3DC &operator*= (Vector2DF &op);
		inline Vector3DC &operator*= (Vector3DC &op);
		inline Vector3DC &operator*= (Vector3DI &op);
		inline Vector3DC &operator*= (Vector3DF &op);
		inline Vector3DC &operator*= (Vector4DF &op);

		inline Vector3DC &operator/= (Vector2DC &op);
		inline Vector3DC &operator/= (Vector2DI &op);
		inline Vector3DC &operator/= (Vector2DF &op);
		inline Vector3DC &operator/= (Vector3DC &op);
		inline Vector3DC &operator/= (Vector3DI &op);
		inline Vector3DC &operator/= (Vector3DF &op);
		inline Vector3DC &operator/= (Vector4DF &op);

		inline Vector3DC &Cross (Vector3DC &v);
		inline Vector3DC &Cross (Vector3DI &v);
		inline Vector3DC &Cross (Vector3DF &v);	
		
		inline double Dot(Vector3DC &v);
		inline double Dot(Vector3DI &v);
		inline double Dot(Vector3DF &v);

		inline double Dist (Vector2DC &v);
		inline double Dist (Vector2DI &v);
		inline double Dist (Vector2DF &v);
		inline double Dist (Vector3DC &v);
		inline double Dist (Vector3DI &v);
		inline double Dist (Vector3DF &v);
		inline double Dist (Vector4DF &v);

		inline double DistSq (Vector2DC &v);
		inline double DistSq (Vector2DI &v);
		inline double DistSq (Vector2DF &v);
		inline double DistSq (Vector3DC &v);
		inline double DistSq (Vector3DI &v);
		inline double DistSq (Vector3DF &v);
		inline double DistSq (Vector4DF &v);

		inline Vector3DC &Normalize (void);
		inline double Length (void);

		inline VTYPE &X(void);
		inline VTYPE &Y(void);
		inline VTYPE &Z(void);
		inline VTYPE W(void);
		inline const VTYPE &X(void) const;
		inline const VTYPE &Y(void) const;
		inline const VTYPE &Z(void) const;
		inline const VTYPE W(void) const;
		inline VTYPE *Data (void);
	};
	
	#undef VNAME
	#undef VTYPE

	// Vector3DI Declaration

	#define VNAME		3DI
	#define VTYPE		int

	class Vector3DI {
	public:
		VTYPE x, y, z;
	
		// Constructors/Destructors
		inline Vector3DI();
		inline ~Vector3DI();
		inline Vector3DI (VTYPE xa, VTYPE ya, VTYPE za);
		inline Vector3DI (Vector2DC &op);
		inline Vector3DI (Vector2DI &op);
		inline Vector3DI (Vector2DF &op);
		inline Vector3DI (Vector3DC &op);
		inline Vector3DI (Vector3DI &op);
		inline Vector3DI (Vector3DF &op);
		inline Vector3DI (Vector4DF &op);

		// Set Functions
		inline Vector3DI &Set (const int xa, const int ya, const int za);

		// Member Functions
		inline Vector3DI &operator= (Vector2DC &op);
		inline Vector3DI &operator= (Vector2DI &op);
		inline Vector3DI &operator= (Vector2DF &op);
		inline Vector3DI &operator= (Vector3DC &op);
		inline Vector3DI &operator= (Vector3DI &op);
		inline Vector3DI &operator= (Vector3DF &op);
		inline Vector3DI &operator= (Vector4DF &op);
		
		inline Vector3DI &operator+= (Vector2DC &op);
		inline Vector3DI &operator+= (Vector2DI &op);
		inline Vector3DI &operator+= (Vector2DF &op);
		inline Vector3DI &operator+= (Vector3DC &op);
		inline Vector3DI &operator+= (Vector3DI &op);
		inline Vector3DI &operator+= (Vector3DF &op);
		inline Vector3DI &operator+= (Vector4DF &op);

		inline Vector3DI &operator-= (Vector2DC &op);
		inline Vector3DI &operator-= (Vector2DI &op);
		inline Vector3DI &operator-= (Vector2DF &op);
		inline Vector3DI &operator-= (Vector3DC &op);
		inline Vector3DI &operator-= (Vector3DI &op);
		inline Vector3DI &operator-= (Vector3DF &op);
		inline Vector3DI &operator-= (Vector4DF &op);
	
		inline Vector3DI &operator*= (Vector2DC &op);
		inline Vector3DI &operator*= (Vector2DI &op);
		inline Vector3DI &operator*= (Vector2DF &op);
		inline Vector3DI &operator*= (Vector3DC &op);
		inline Vector3DI &operator*= (Vector3DI &op);
		inline Vector3DI &operator*= (Vector3DF &op);
		inline Vector3DI &operator*= (Vector4DF &op);

		inline Vector3DI &operator/= (Vector2DC &op);
		inline Vector3DI &operator/= (Vector2DI &op);
		inline Vector3DI &operator/= (Vector2DF &op);
		inline Vector3DI &operator/= (Vector3DC &op);
		inline Vector3DI &operator/= (Vector3DI &op);
		inline Vector3DI &operator/= (Vector3DF &op);
		inline Vector3DI &operator/= (Vector4DF &op);

		inline Vector3DI &Cross (Vector3DC &v);
		inline Vector3DI &Cross (Vector3DI &v);
		inline Vector3DI &Cross (Vector3DF &v);	
		
		inline double Dot(Vector3DC &v);
		inline double Dot(Vector3DI &v);
		inline double Dot(Vector3DF &v);

		inline double Dist (Vector2DC &v);
		inline double Dist (Vector2DI &v);
		inline double Dist (Vector2DF &v);
		inline double Dist (Vector3DC &v);
		inline double Dist (Vector3DI &v);
		inline double Dist (Vector3DF &v);
		inline double Dist (Vector4DF &v);

		inline double DistSq (Vector2DC &v);
		inline double DistSq (Vector2DI &v);
		inline double DistSq (Vector2DF &v);
		inline double DistSq (Vector3DC &v);
		inline double DistSq (Vector3DI &v);
		inline double DistSq (Vector3DF &v);
		inline double DistSq (Vector4DF &v);

		inline Vector3DI &Normalize (void);
		inline double Length (void);

		inline VTYPE &X(void);
		inline VTYPE &Y(void);
		inline VTYPE &Z(void);
		inline VTYPE W(void);
		inline const VTYPE &X(void) const;
		inline const VTYPE &Y(void) const;
		inline const VTYPE &Z(void) const;
		inline const VTYPE W(void) const;
		inline VTYPE *Data (void);
	};
	
	#undef VNAME
	#undef VTYPE

	// Vector3DF Declarations

	#define VNAME		3DF
	#define VTYPE		float

	class Vector3DF {
	public:
		VTYPE x, y, z;
	
		// Constructors/Destructors
		inline Vector3DF();
		inline ~Vector3DF();
		inline Vector3DF (const VTYPE xa, const VTYPE ya, const VTYPE za);
		inline Vector3DF (const Vector2DC &op);
		inline Vector3DF (const Vector2DI &op);
		inline Vector3DF (const Vector2DF &op);
		inline Vector3DF (const Vector3DC &op);
		inline Vector3DF (const Vector3DI &op);
		inline Vector3DF (const Vector3DF &op);
		inline Vector3DF (const Vector4DF &op);

		// Set Functions
		inline Vector3DF &Set (const double xa, const double ya, const double za);
		
		// Member Functions
		inline Vector3DF &operator= (const int op);
		inline Vector3DF &operator= (const double op);
		inline Vector3DF &operator= (const Vector2DC &op);
		inline Vector3DF &operator= (const Vector2DI &op);
		inline Vector3DF &operator= (const Vector2DF &op);
		inline Vector3DF &operator= (const Vector3DC &op);
		inline Vector3DF &operator= (const Vector3DI &op);
		inline Vector3DF &operator= (const Vector3DF &op);
		inline Vector3DF &operator= (const Vector4DF &op);

		inline Vector3DF &operator+= (const int op);
		inline Vector3DF &operator+= (const double op);
		inline Vector3DF &operator+= (const Vector2DC &op);
		inline Vector3DF &operator+= (const Vector2DI &op);
		inline Vector3DF &operator+= (const Vector2DF &op);
		inline Vector3DF &operator+= (const Vector3DC &op);
		inline Vector3DF &operator+= (const Vector3DI &op);
		inline Vector3DF &operator+= (const Vector3DF &op);
		inline Vector3DF &operator+= (const Vector4DF &op);

		inline Vector3DF &operator-= (const int op);
		inline Vector3DF &operator-= (const double op);
		inline Vector3DF &operator-= (const Vector2DC &op);
		inline Vector3DF &operator-= (const Vector2DI &op);
		inline Vector3DF &operator-= (const Vector2DF &op);
		inline Vector3DF &operator-= (const Vector3DC &op);
		inline Vector3DF &operator-= (const Vector3DI &op);
		inline Vector3DF &operator-= (const Vector3DF &op);
		inline Vector3DF &operator-= (const Vector4DF &op);
	
		inline Vector3DF &operator*= (const int op);
		inline Vector3DF &operator*= (const double op);
		inline Vector3DF &operator*= (const Vector2DC &op);
		inline Vector3DF &operator*= (const Vector2DI &op);
		inline Vector3DF &operator*= (const Vector2DF &op);
		inline Vector3DF &operator*= (const Vector3DC &op);
		inline Vector3DF &operator*= (const Vector3DI &op);
		inline Vector3DF &operator*= (const Vector3DF &op);
		inline Vector3DF &operator*= (const Vector4DF &op);
		Vector3DF &operator*= (const Matrix4F &op);
		Vector3DF &operator*= (const MatrixF &op);				// see vector.cpp

		inline Vector3DF &operator/= (const int op);
		inline Vector3DF &operator/= (const double op);
		inline Vector3DF &operator/= (const Vector2DC &op);
		inline Vector3DF &operator/= (const Vector2DI &op);
		inline Vector3DF &operator/= (const Vector2DF &op);
		inline Vector3DF &operator/= (const Vector3DC &op);
		inline Vector3DF &operator/= (const Vector3DI &op);
		inline Vector3DF &operator/= (const Vector3DF &op);
		inline Vector3DF &operator/= (const Vector4DF &op);

		inline Vector3DF &Cross (const Vector3DC &v);
		inline Vector3DF &Cross (const Vector3DI &v);
		inline Vector3DF &Cross (const Vector3DF &v);	
		
		inline double Dot(const Vector3DC &v);
		inline double Dot(const Vector3DI &v);
		inline double Dot(const Vector3DF &v);

		inline double Dist (const Vector2DC &v);
		inline double Dist (const Vector2DI &v);
		inline double Dist (const Vector2DF &v);
		inline double Dist (const Vector3DC &v);
		inline double Dist (const Vector3DI &v);
		inline double Dist (const Vector3DF &v);
		inline double Dist (const Vector4DF &v);

		inline double DistSq (const Vector2DC &v);
		inline double DistSq (const Vector2DI &v);
		inline double DistSq (const Vector2DF &v);
		inline double DistSq (const Vector3DC &v);
		inline double DistSq (const Vector3DI &v);
		inline double DistSq (const Vector3DF &v);
		inline double DistSq (const Vector4DF &v);
		
		inline Vector3DF &Normalize (void);
		inline double Length (void);

		inline VTYPE &X();
		inline VTYPE &Y();
		inline VTYPE &Z();
		inline VTYPE W();
		inline const VTYPE &X() const;
		inline const VTYPE &Y() const;
		inline const VTYPE &Z() const;
		inline const VTYPE W() const;
		inline VTYPE *Data ();
	};
	
	#undef VNAME
	#undef VTYPE

	// Vector4DF Declarations

	#define VNAME		4DF
	#define VTYPE		double

	class Vector4DF {
	public:
		VTYPE x, y, z, w;
	
		// Constructors/Destructors
		inline Vector4DF();
		inline ~Vector4DF();
		inline Vector4DF (VTYPE xa, VTYPE ya, VTYPE za, VTYPE wa);
		inline Vector4DF (Vector2DC &op);
		inline Vector4DF (Vector2DI &op);
		inline Vector4DF (Vector2DF &op);
		inline Vector4DF (Vector3DC &op);
		inline Vector4DF (Vector3DI &op);
		inline Vector4DF (Vector3DF &op);
		inline Vector4DF (Vector4DF &op);

		// Member Functions
		inline Vector4DF &operator= (int op);
		inline Vector4DF &operator= (double op);
		inline Vector4DF &operator= (Vector2DC &op);
		inline Vector4DF &operator= (Vector2DI &op);
		inline Vector4DF &operator= (Vector2DF &op);
		inline Vector4DF &operator= (Vector3DC &op);
		inline Vector4DF &operator= (Vector3DI &op);
		inline Vector4DF &operator= (Vector3DF &op);
		inline Vector4DF &operator= (Vector4DF &op);

		inline Vector4DF &operator+= (int op);
		inline Vector4DF &operator+= (double op);
		inline Vector4DF &operator+= (Vector2DC &op);
		inline Vector4DF &operator+= (Vector2DI &op);
		inline Vector4DF &operator+= (Vector2DF &op);
		inline Vector4DF &operator+= (Vector3DC &op);
		inline Vector4DF &operator+= (Vector3DI &op);
		inline Vector4DF &operator+= (Vector3DF &op);
		inline Vector4DF &operator+= (Vector4DF &op);

		inline Vector4DF &operator-= (int op);
		inline Vector4DF &operator-= (double op);
		inline Vector4DF &operator-= (Vector2DC &op);
		inline Vector4DF &operator-= (Vector2DI &op);
		inline Vector4DF &operator-= (Vector2DF &op);
		inline Vector4DF &operator-= (Vector3DC &op);
		inline Vector4DF &operator-= (Vector3DI &op);
		inline Vector4DF &operator-= (Vector3DF &op);
		inline Vector4DF &operator-= (Vector4DF &op);
	
		inline Vector4DF &operator*= (int op);
		inline Vector4DF &operator*= (double op);
		inline Vector4DF &operator*= (Vector2DC &op);
		inline Vector4DF &operator*= (Vector2DI &op);
		inline Vector4DF &operator*= (Vector2DF &op);
		inline Vector4DF &operator*= (Vector3DC &op);
		inline Vector4DF &operator*= (Vector3DI &op);
		inline Vector4DF &operator*= (Vector3DF &op);
		inline Vector4DF &operator*= (Vector4DF &op);
		Vector4DF &operator*= (const Matrix4F &op);
		Vector4DF &operator*= (const MatrixF &op);				// see vector.cpp

		inline Vector4DF &operator/= (int op);
		inline Vector4DF &operator/= (double op);
		inline Vector4DF &operator/= (Vector2DC &op);
		inline Vector4DF &operator/= (Vector2DI &op);
		inline Vector4DF &operator/= (Vector2DF &op);
		inline Vector4DF &operator/= (Vector3DC &op);
		inline Vector4DF &operator/= (Vector3DI &op);
		inline Vector4DF &operator/= (Vector3DF &op);
		inline Vector4DF &operator/= (Vector4DF &op);

		inline Vector4DF &Cross (Vector4DF &v);	
		
		inline double Dot(Vector4DF &v);

		inline double Dist (Vector4DF &v);

		inline double DistSq (Vector4DF &v);

		inline Vector4DF &Normalize (void);
		inline double Length (void);

		inline VTYPE &X(void);
		inline VTYPE &Y(void);
		inline VTYPE &Z(void);
		inline VTYPE &W(void);
		inline const VTYPE &X(void) const;
		inline const VTYPE &Y(void) const;
		inline const VTYPE &Z(void) const;
		inline const VTYPE &W(void) const;
		inline VTYPE *Data (void);
	};
	
	#undef VNAME
	#undef VTYPE

    // Vector Code Definitions (Inlined)
	#include "vector.cci"

#endif

