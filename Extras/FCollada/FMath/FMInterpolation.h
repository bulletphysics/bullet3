/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FMInterpolation.h
	The file containing the enum for interpolation.
*/

#ifndef _FM_INTERPOLATION_H_
#define _FM_INTERPOLATION_H_

/**
	A namespace for interpolations.

	@ingroup FMath
*/
namespace FMInterpolation
{
	/** The different types of interpolation. */
	enum Interpolation
	{
		NONE = 0,			/**< No interpolation. Also called step interpolation. */
		LINEAR,				/**< Linear interpolation. */
		BEZIER,				/**< Bezier interpolation. */

		UNKNOWN,			/**< Unknown interpolation. */
		DEFAULT = NONE,		/**< Default interpolation (None). */
	};
};

#endif // _FM_INTERPOLATION_H_
