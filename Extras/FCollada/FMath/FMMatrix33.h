/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FMMatrix33.h
	The file containing the class and global functions for 3x3 matrices.
*/

#ifndef _FM_MATRIX33_H_
#define _FM_MATRIX33_H_

/**
	A 3x3 matrix: use to represent 2D transformations.
	Not used within FCollada.
	
	@ingroup FMath
*/
class FCOLLADA_EXPORT FMMatrix33
{
public:
	float m[3][3];	/**< The matrix elements stored in a 2D array. */

	/**
	 * Creates a FMMatrix33 from the \c float array.
	 *
	 * The float array stores the elements in the following order: m[0][0], 
	 * m[1][0], m[2][0], m[0][1], m[1][1], m[2][1], m[0][2], m[1][2], m[2][2].
	 *
	 * @param _m The \c float array to create the matrix from.
	 */
	FMMatrix33(float* _m);
	
	/**
	 * Creates an empty FMMatrix33.
	 *
	 * The default values are non deterministic.
	 */
	#ifndef _DEBUG
	FMMatrix33() {}
	#else
	FMMatrix33() { memset(m, 55, 3 * 3 * sizeof(float)); }
	#endif 

	/**
	 * Get this FMMatrix33 as an array of \c floats.
	 *
	 * The array contains the elements in the following order: m[0][0], 
	 * m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2].
	 *
	 * @return The \c float array.
	 */
	operator float*() { return &m[0][0]; }

	/**
	 * Get this FMMatrix as an array of \c floats.
	 *
	 * The array contains the elements in the following order: m[0][0], 
	 * m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2].
	 *
	 * @return The \c float array.
	 */
	operator const float*() const { return &m[0][0]; }

	/**
	 * Get a specified row of FMMatrix33 as an array of \c floats.
	 *
	 * @param a The row index, starting at 0, of the row to get.
	 * @return The \c float array of the elements in the specified row.
	 */
	float* operator[](int a) { return m[a]; }

	/**
	 * Assign this FMMatrix33's elements to be the same as that of the given 
	 * FMMatrix33.
	 *
	 * @param copy The FMMatrix to copy elements from.
	 * @return This FMMatrix.
	 */
	FMMatrix33& operator=(const FMMatrix33& copy);

	/**
	 * Gets the transposed of this FMMatrix33.
	 *
	 * @return The transposed of this FMMatrix.
	 */
	FMMatrix33 Transposed() const;

	/**
	 * Gets the inversion of this FMMatrix33.
	 *
	 * @return The inversion of this FMMatrix.
	 */
	FMMatrix33 Inverted() const;

public:
	static FMMatrix33 identity;	/**< The identity FMMatrix33. */

	/**
	 * Gets the FMMatrix33 representation of a 2D counter-clockwise rotation
	 * about the z-axis.
	 *
	 * @param angle The angle of rotation in radians.
	 * @return The rotation FMMatrix.
	 */
	static FMMatrix33 RotationMatrix(float angle);

	/**
	 * Gets the FMMatrix33 representation of a 2D translation.
	 *
	 * @param tx The translation in the x direction.
	 * @param ty The translation in the y direction.
	 * @return The translation FMMatrix.
	 */
	static FMMatrix33 TranslationMatrix(float tx, float ty);
	
	/**
	 * Gets the FMMatrix33 representation of a 2D scale.
	 *
	 * @param sx The scale factor in the x direction.
	 * @param sy The scale factor in the y direction.
	 * @return The scale FMMatrix.
	 */
	static FMMatrix33 ScaleMatrix(float sx, float sy);

	/**
	 * Gets the FMMatrix33 representation of a 2D translation.
	 *
	 * The translation in the x direction is the \a u componenet of the given
	 * FMVector2 and the translation in the y direction is the \a v component.
	 *
	 * @param translation The FMVector2 to get the translation components from.
	 * @return The translation FMMatrix33.
	 */
	static inline FMMatrix33 TranslationMatrix(FMVector2 translation) { return TranslationMatrix(translation.u, translation.v); }

	/**
	 * Gets the FMMatrix33 representation of a 2D scale.
	 *
	 * The scale in the x direction is the \a u componenet of the given
	 * FMVector and the scale in the y direction is the \a v component.
	 *
	 * @param scale The FMVector2 to get the scale components from.
	 * @return The scale FMMatrix33.
	 */
	static inline FMMatrix33 ScaleMatrix(FMVector2 scale) { return ScaleMatrix(scale.u, scale.v); }
};

/**
 * Matrix multiplication with two FMMatrix33.
 *
 * @param m1 The first matrix.
 * @param m2 The second matrix.
 * @return The FMMatrix44 representation of the resulting matrix.
 */
FMMatrix33 FCOLLADA_EXPORT operator*(const FMMatrix33& m1, const FMMatrix33& m2);

#endif // _FM_MATRIX33_H_
