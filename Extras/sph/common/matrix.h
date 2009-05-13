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
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <memory.h>
#include <math.h>

//*********** NOTE
//
// LOOK AT MovieTrackPoint. IN ORDER FOR VECTORS AND MATRICIES TO BE USED IN OBJECTS 
// THAT WILL BE USED IN stl::vectors, THEIR CONSTRUCTORS AND OPERATORS MUST TAKE ONLY
// const PARAMETERS. LOOK AT MatrixF and Vector2DF.. THIS WAS NOT YET DONE WITH
// THE OTHER MATRIX AND VECTOR CLASSES (Vector2DC, Vector2DI, MatrixC, MatrixI, ...)
//


#ifndef MATRIX_DEF
	#define MATRIX_DEF
		
	#include "vector.h"
	#include "mdebug.h"	
	
	//#define MATRIX_INITIALIZE				// Initializes vectors	

	class MatrixC;							// Forward Referencing
	class MatrixI;
	class MatrixF;

	class Matrix {
	public:
		// Member Virtual Functions		
		virtual Matrix &operator= (unsigned char c)=0;
		virtual Matrix &operator= (int c)=0;
		virtual Matrix &operator= (double c)=0;		
		virtual Matrix &operator= (MatrixC &op)=0;
		virtual Matrix &operator= (MatrixI &op)=0;
		virtual Matrix &operator= (MatrixF &op)=0;
		
		virtual Matrix &operator+= (unsigned char c)=0;
		virtual Matrix &operator+= (int c)=0;
		virtual Matrix &operator+= (double c)=0;		
		virtual Matrix &operator+= (MatrixC &op)=0;
		virtual Matrix &operator+= (MatrixI &op)=0;
		virtual Matrix &operator+= (MatrixF &op)=0;

		virtual Matrix &operator-= (unsigned char c)=0;
		virtual Matrix &operator-= (int c)=0;
		virtual Matrix &operator-= (double c)=0;		
		virtual Matrix &operator-= (MatrixC &op)=0;
		virtual Matrix &operator-= (MatrixI &op)=0;
		virtual Matrix &operator-= (MatrixF &op)=0;

		virtual Matrix &operator*= (unsigned char c)=0;
		virtual Matrix &operator*= (int c)=0;
		virtual Matrix &operator*= (double c)=0;		
		virtual Matrix &operator*= (MatrixC &op)=0;
		virtual Matrix &operator*= (MatrixI &op)=0;
		virtual Matrix &operator*= (MatrixF &op)=0;

		virtual Matrix &operator/= (unsigned char c)=0;
		virtual Matrix &operator/= (int c)=0;
		virtual Matrix &operator/= (double c)=0;		
		virtual Matrix &operator/= (MatrixC &op)=0;
		virtual Matrix &operator/= (MatrixI &op)=0;
		virtual Matrix &operator/= (MatrixF &op)=0;

		virtual Matrix &Multiply (MatrixF &op)=0;
		virtual Matrix &Resize (int x, int y)=0;
		virtual Matrix &ResizeSafe (int x, int y)=0;
		virtual Matrix &InsertRow (int r)=0;
		virtual Matrix &InsertCol (int c)=0;
		virtual Matrix &Transpose (void)=0;
		virtual Matrix &Identity (int order)=0;
		/*inline Matrix &RotateX (double ang);
		inline Matrix &RotateY (double ang);
		inline Matrix &RotateZ (double ang); */
		virtual Matrix &Basis (Vector3DF &c1, Vector3DF &c2, Vector3DF &c3)=0;
		virtual Matrix &GaussJordan (MatrixF &b)		{ return *this; }
		virtual Matrix &ConjugateGradient (MatrixF &b)	{ return *this; }

		virtual int GetRows(void)=0;
		virtual int GetCols(void)=0;
		virtual int GetLength(void)=0;		

		virtual unsigned char *GetDataC (void)=0;
		virtual int	*GetDataI (void)=0;
		virtual double *GetDataF (void)=0;

		virtual double GetF (int r, int c);
	};
	
	// MatrixC Declaration	
	#define VNAME		C
	#define VTYPE		unsigned char

	class MatrixC {
	public:
		VTYPE *data;
		int rows, cols, len;		

		// Constructors/Destructors
		inline MatrixC ();
		inline ~MatrixC ();
		inline MatrixC (int r, int c);

		// Member Functions
		inline VTYPE &operator () (int c, int r);
		inline MatrixC &operator= (unsigned char c);
		inline MatrixC &operator= (int c);
		inline MatrixC &operator= (double c);		
		inline MatrixC &operator= (MatrixC &op);
		inline MatrixC &operator= (MatrixI &op);
		inline MatrixC &operator= (MatrixF &op);
		
		inline MatrixC &operator+= (unsigned char c);
		inline MatrixC &operator+= (int c);
		inline MatrixC &operator+= (double c);		
		inline MatrixC &operator+= (MatrixC &op);
		inline MatrixC &operator+= (MatrixI &op);
		inline MatrixC &operator+= (MatrixF &op);

		inline MatrixC &operator-= (unsigned char c);
		inline MatrixC &operator-= (int c);
		inline MatrixC &operator-= (double c);		
		inline MatrixC &operator-= (MatrixC &op);
		inline MatrixC &operator-= (MatrixI &op);
		inline MatrixC &operator-= (MatrixF &op);

		inline MatrixC &operator*= (unsigned char c);
		inline MatrixC &operator*= (int c);
		inline MatrixC &operator*= (double c);		
		inline MatrixC &operator*= (MatrixC &op);
		inline MatrixC &operator*= (MatrixI &op);
		inline MatrixC &operator*= (MatrixF &op);

		inline MatrixC &operator/= (unsigned char c);
		inline MatrixC &operator/= (int c);
		inline MatrixC &operator/= (double c);		
		inline MatrixC &operator/= (MatrixC &op);
		inline MatrixC &operator/= (MatrixI &op);
		inline MatrixC &operator/= (MatrixF &op);

		inline MatrixC &Multiply (MatrixF &op);
		inline MatrixC &Resize (int x, int y);
		inline MatrixC &ResizeSafe (int x, int y);
		inline MatrixC &InsertRow (int r);
		inline MatrixC &InsertCol (int c);
		inline MatrixC &Transpose (void);
		inline MatrixC &Identity (int order);		
		inline MatrixC &Basis (Vector3DF &c1, Vector3DF &c2, Vector3DF &c3);
		inline MatrixC &GaussJordan (MatrixF &b);

		inline int GetX();
		inline int GetY();	
		inline int GetRows(void);
		inline int GetCols(void);
		inline int GetLength(void);
		inline VTYPE *GetData(void);

		inline unsigned char *GetDataC (void)	{return data;}
		inline int *GetDataI (void)				{return NULL;}
		inline double *GetDataF (void)			{return NULL;}		

		inline double GetF (int r, int c);
	};
	#undef VNAME
	#undef VTYPE

	// MatrixI Declaration	
	#define VNAME		I
	#define VTYPE		int

	class MatrixI {
	public:
		VTYPE *data;
		int rows, cols, len;		
	
		// Constructors/Destructors
		inline MatrixI ();
		inline ~MatrixI ();
		inline MatrixI (int r, int c);

		// Member Functions
		inline VTYPE &operator () (int c, int r);
		inline MatrixI &operator= (unsigned char c);
		inline MatrixI &operator= (int c);
		inline MatrixI &operator= (double c);		
		inline MatrixI &operator= (MatrixC &op);
		inline MatrixI &operator= (MatrixI &op);
		inline MatrixI &operator= (MatrixF &op);
		
		inline MatrixI &operator+= (unsigned char c);
		inline MatrixI &operator+= (int c);
		inline MatrixI &operator+= (double c);		
		inline MatrixI &operator+= (MatrixC &op);
		inline MatrixI &operator+= (MatrixI &op);
		inline MatrixI &operator+= (MatrixF &op);

		inline MatrixI &operator-= (unsigned char c);
		inline MatrixI &operator-= (int c);
		inline MatrixI &operator-= (double c);		
		inline MatrixI &operator-= (MatrixC &op);
		inline MatrixI &operator-= (MatrixI &op);
		inline MatrixI &operator-= (MatrixF &op);

		inline MatrixI &operator*= (unsigned char c);
		inline MatrixI &operator*= (int c);
		inline MatrixI &operator*= (double c);		
		inline MatrixI &operator*= (MatrixC &op);
		inline MatrixI &operator*= (MatrixI &op);
		inline MatrixI &operator*= (MatrixF &op);

		inline MatrixI &operator/= (unsigned char c);
		inline MatrixI &operator/= (int c);
		inline MatrixI &operator/= (double c);		
		inline MatrixI &operator/= (MatrixC &op);
		inline MatrixI &operator/= (MatrixI &op);
		inline MatrixI &operator/= (MatrixF &op);

		inline MatrixI &Multiply (MatrixF &op);
		inline MatrixI &Resize (int x, int y);
		inline MatrixI &ResizeSafe (int x, int y);
		inline MatrixI &InsertRow (int r);
		inline MatrixI &InsertCol (int c);
		inline MatrixI &Transpose (void);
		inline MatrixI &Identity (int order);		
		inline MatrixI &Basis (Vector3DF &c1, Vector3DF &c2, Vector3DF &c3);
		inline MatrixI &GaussJordan (MatrixF &b);

		inline int GetX();
		inline int GetY();	
		inline int GetRows(void);
		inline int GetCols(void);
		inline int GetLength(void);
		inline VTYPE *GetData(void);

		inline unsigned char *GetDataC (void)	{return NULL;}
		inline int *GetDataI (void)				{return data;}
		inline double *GetDataF (void)			{return NULL;}
		
		inline double GetF (int r, int c);
	};
	#undef VNAME
	#undef VTYPE

	// MatrixF Declaration	
	#define VNAME		F
	#define VTYPE		double

	class MatrixF {
	public:	
		VTYPE *data;
		int rows, cols, len;		

		// Constructors/Destructors		
		inline MatrixF ();
		inline ~MatrixF ();
		inline MatrixF (const int r, const int c);

		// Member Functions
		inline VTYPE GetVal ( int c, int r );
		inline VTYPE &operator () (const int c, const int r);
		inline MatrixF &operator= (const unsigned char c);
		inline MatrixF &operator= (const int c);
		inline MatrixF &operator= (const double c);		
		inline MatrixF &operator= (const MatrixC &op);
		inline MatrixF &operator= (const MatrixI &op);
		inline MatrixF &operator= (const MatrixF &op);
		
		inline MatrixF &operator+= (const unsigned char c);
		inline MatrixF &operator+= (const int c);
		inline MatrixF &operator+= (const double c);		
		inline MatrixF &operator+= (const MatrixC &op);
		inline MatrixF &operator+= (const MatrixI &op);
		inline MatrixF &operator+= (const MatrixF &op);

		inline MatrixF &operator-= (const unsigned char c);
		inline MatrixF &operator-= (const int c);
		inline MatrixF &operator-= (const double c);		
		inline MatrixF &operator-= (const MatrixC &op);
		inline MatrixF &operator-= (const MatrixI &op);
		inline MatrixF &operator-= (const MatrixF &op);

		inline MatrixF &operator*= (const unsigned char c);
		inline MatrixF &operator*= (const int c);
		inline MatrixF &operator*= (const double c);		
		inline MatrixF &operator*= (const MatrixC &op);
		inline MatrixF &operator*= (const MatrixI &op);
		inline MatrixF &operator*= (const MatrixF &op);		

		inline MatrixF &operator/= (const unsigned char c);
		inline MatrixF &operator/= (const int c);
		inline MatrixF &operator/= (const double c);		
		inline MatrixF &operator/= (const MatrixC &op);
		inline MatrixF &operator/= (const MatrixI &op);
		inline MatrixF &operator/= (const MatrixF &op);

		inline MatrixF &Multiply4x4 (const MatrixF &op);
		inline MatrixF &Multiply (const MatrixF &op);
		inline MatrixF &Resize (const int x, const int y);
		inline MatrixF &ResizeSafe (const int x, const int y);
		inline MatrixF &InsertRow (const int r);
		inline MatrixF &InsertCol (const int c);
		inline MatrixF &Transpose (void);
		inline MatrixF &Identity (const int order);
		inline MatrixF &RotateX (const double ang);
		inline MatrixF &RotateY (const double ang);
		inline MatrixF &RotateZ (const double ang);
		inline MatrixF &Ortho (double sx, double sy, double n, double f);		
		inline MatrixF &Translate (double tx, double ty, double tz);
		inline MatrixF &Basis (const Vector3DF &c1, const Vector3DF &c2, const Vector3DF &c3);
		inline MatrixF &GaussJordan (MatrixF &b);
		inline MatrixF &ConjugateGradient (MatrixF &b);
		inline MatrixF &Submatrix ( MatrixF& b, int mx, int my);
		inline MatrixF &MatrixVector5 (MatrixF& x, int mrows, MatrixF& b );
		inline MatrixF &ConjugateGradient5 (MatrixF &b, int mrows );
		inline double Dot ( MatrixF& b );

		inline void Print ( char* fname );

		inline int GetX();
		inline int GetY();	
		inline int GetRows(void);
		inline int GetCols(void);
		inline int GetLength(void);
		inline VTYPE *GetData(void);
		inline void GetRowVec (int r, Vector3DF &v);

		inline unsigned char *GetDataC (void) const	{return NULL;}
		inline int *GetDataI (void)	const			{return NULL;}
		inline double *GetDataF (void) const		{return data;}

		inline double GetF (const int r, const int c);
	};
	#undef VNAME
	#undef VTYPE

	// MatrixF Declaration	
	#define VNAME		F
	#define VTYPE		float

	class Matrix4F {
	public:	
		VTYPE	data[16];		

		// Constructors/Destructors		
		inline Matrix4F ();		

		// Member Functions
		inline VTYPE &operator () (const int n)					{ return data[n]; }
		inline VTYPE &operator () (const int c, const int r)	{ return data[ (r<<2)+c ]; }
		inline Matrix4F &operator= (const unsigned char c);
		inline Matrix4F &operator= (const int c);
		inline Matrix4F &operator= (const double c);				
		inline Matrix4F &operator+= (const unsigned char c);
		inline Matrix4F &operator+= (const int c);
		inline Matrix4F &operator+= (const double c);				
		inline Matrix4F &operator-= (const unsigned char c);
		inline Matrix4F &operator-= (const int c);
		inline Matrix4F &operator-= (const double c);
		inline Matrix4F &operator*= (const unsigned char c);
		inline Matrix4F &operator*= (const int c);
		inline Matrix4F &operator*= (const double c);
		inline Matrix4F &operator/= (const unsigned char c);
		inline Matrix4F &operator/= (const int c);
		inline Matrix4F &operator/= (const double c);		
		
		inline Matrix4F &Multiply (const Matrix4F &op);
		inline Matrix4F &Transpose (void);
		inline Matrix4F &Identity (const int order);
		inline Matrix4F &RotateX (const double ang);
		inline Matrix4F &RotateY (const double ang);
		inline Matrix4F &RotateZ (const double ang);
		inline Matrix4F &Ortho (double sx, double sy, double n, double f);		
		inline Matrix4F &Translate (double tx, double ty, double tz);
		inline Matrix4F &Basis (const Vector3DF &c1, const Vector3DF &c2, const Vector3DF &c3);		

		// Scale-Rotate-Translate (compound matrix)
		inline Matrix4F &SRT (const Vector3DF &c1, const Vector3DF &c2, const Vector3DF &c3, const Vector3DF& t, const Vector3DF& s);
		inline Matrix4F &SRT (const Vector3DF &c1, const Vector3DF &c2, const Vector3DF &c3, const Vector3DF& t, const float s);

		// invTranslate-invRotate-invScale (compound matrix)
		inline Matrix4F &InvTRS (const Vector3DF &c1, const Vector3DF &c2, const Vector3DF &c3, const Vector3DF& t, const Vector3DF& s);
		inline Matrix4F &InvTRS (const Vector3DF &c1, const Vector3DF &c2, const Vector3DF &c3, const Vector3DF& t, const float s);

		inline int GetX()			{ return 4; }
		inline int GetY()			{ return 4; }
		inline int GetRows(void)	{ return 4; }
		inline int GetCols(void)	{ return 4; }	
		inline int GetLength(void)	{ return 16; }
		inline VTYPE *GetData(void)	{ return data; }
		inline void GetRowVec (int r, Vector3DF &v);

		inline unsigned char *GetDataC (void) const	{return NULL;}
		inline int *GetDataI (void)	const			{return NULL;}
		inline float *GetDataF (void) const		{return (float*) data;}

		inline float GetF (const int r, const int c);
	};
	#undef VNAME
	#undef VTYPE


    // Matrix Code Definitions (Inlined)

	#include "matrix.cci"

#endif

