/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FMColor.h
	The file containing the class and global functions for RBGA colors.
*/

#ifndef _FM_COLOR_H_
#define _FM_COLOR_H_

/**
	A RBGA color.
	Not used within FCollada.

	@ingroup FMath
*/
class FCOLLADA_EXPORT FMColor
{
public:
	uint8 r;	/**< The red component. */
	uint8 g;	/**< The green component. */
	uint8 b;	/**< The blue component. */
	uint8 a;	/**< The alpha component. */

	/**
	 * Creates an empty FMColor.
	 *
	 * The default values are non deterministic.
	 */
	FMColor() {}

	/**
	 * Creates the FMColor with the coordinates values.
	 *
	 * The first three coordinates are taken from the FMVector3, where the
	 * first one is the x value, the second is that y, and the third is the z.
	 * The forth value is the \c float specified.
	 * 
	 *
	 * @param _r The red value.
	 * @param _g The green value.
	 * @param _b The blue value.
	 * @param _a The alpha value.
	 */
	FMColor(uint8 _r, uint8 _g, uint8 _b, uint8 _a) { r = _r; g = _g; b = _b; a = _a; }

	/**
	 * Creates the FMColor from a color encoded into a uint32.
	 *
	 * The most significant byte makes up the red value. The second most
	 * significant makes up the green value, the third the blue, and the forth
	 * the alpha.
	 *
	 * @param hex The uint to decode the color values from.
	 */
	FMColor(uint32 hex) { r = uint8((hex & 0xFF000000) >> 24); g = uint8((hex & 0xFF0000) >> 16); b = uint8((hex & 0xFF00) >> 8); a = uint8(hex & 0xFF); }

	/**
	 * Creates the FMColor from a list of \c floats.
	 *
	 * It creates the FMColor with the values specified in the \c floats, which
	 * range from 0.0f to 1.0f. 
	 *
	 * \a componentCount is used to determined how many values to take from the
	 * \c float array. If there are insufficient values, then it will give the
	 * remaining values default values. The default values are 0 for the colors
	 * and 255 for the alpha. It fills in the values in this order: red, green,
	 * blue, alpha.
	 *
	 * @param components The \c float array to get values from.
	 * @param componentCount The amount of \c float to take from the array.
	 */
	FMColor(const float* components, uint32 componentCount);

	/**
	 * Get this FMColor as an array of \c floats.
	 *
	 * It fills the first \a componentCount elements (max 4) of \c components
	 * with the red, green, blue, alpha values of this FMColor in that order.
	 *
	 * @param components The \c float array to fill.
	 * @param componentCount The amount of \c float to fill into the array.
	 */
	void ToFloats(float* components, uint32 componentCount);

	/**
	 * Get the average of the three color values of this FMColor.
	 *
	 * @return The averages of the three colors values of this FMColor.
	 */
	inline uint8 ComponentAverage() { return uint8((uint32(r) + uint32(g) + uint32(b)) / 3); }

	/**
	 * Get this FMColor as an array of uint8s.
	 *
	 * @return The \c uint8 array.
	 */
	operator uint8*() { return &b; }
};

/**
 * Multiplication of a scalar with the FMColor.
 *
 * @param s The scalar to multiply by.
 * @param c The FMColor to multiply with.
 * @return the FMColor representing the resulting color.
 */
inline FMColor operator*(float s, const FMColor& c) { return FMColor(uint8(c.r * s), uint8(c.g * s), uint8(c.b * s), uint8(c.a * s)); }

#endif // _FM_COLOR_H_
