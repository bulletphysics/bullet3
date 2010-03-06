/*
 *	ICE / OPCODE - Optimized Collision Detection
 * http://www.codercorner.com/Opcode.htm
 * 
 * Copyright (c) 2001-2008 Pierre Terdiman,  pierre@codercorner.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains axes definition.
 *	\file		IceAxes.h
 *	\author		Pierre Terdiman
 *	\date		January, 29, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __ICEAXES_H__
#define __ICEAXES_H__

	enum PointComponent
	{
		_X					= 0,
		_Y					= 1,
		_Z					= 2,
		_W					= 3,

		_FORCE_DWORD		= 0x7fffffff
	};

	enum AxisOrder
	{
		AXES_XYZ			= (_X)|(_Y<<2)|(_Z<<4),
		AXES_XZY			= (_X)|(_Z<<2)|(_Y<<4),
		AXES_YXZ			= (_Y)|(_X<<2)|(_Z<<4),
		AXES_YZX			= (_Y)|(_Z<<2)|(_X<<4),
		AXES_ZXY			= (_Z)|(_X<<2)|(_Y<<4),
		AXES_ZYX			= (_Z)|(_Y<<2)|(_X<<4),

		AXES_FORCE_DWORD	= 0x7fffffff
	};

	class ICEMATHS_API Axes
	{
		public:

		inline_			Axes(AxisOrder order)
						{
							mAxis0 = (order   ) & 3;
							mAxis1 = (order>>2) & 3;
							mAxis2 = (order>>4) & 3;
						}
		inline_			~Axes()		{}

				udword	mAxis0;
				udword	mAxis1;
				udword	mAxis2;
	};

#endif // __ICEAXES_H__
