/*
Impulse based Rigid body simulation using CUDA
Copyright (c) 2007 Takahiro Harada  http://www.iii.u-tokyo.ac.jp/~takahiroharada/projects/impulseCUDA.html

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

//----------   C o n s t r a i n t   s o l v e r    d e m o   ----------------------------

struct btCudaPartProps
{
	float m_mass;
	float m_diameter;
	float m_restCoeff;
};

struct btCudaBoxProps
{
	float minX;
	float maxX;
	float minY;
	float maxY;
	float minZ;
	float maxZ;
};


