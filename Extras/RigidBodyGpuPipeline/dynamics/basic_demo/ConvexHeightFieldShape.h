/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Takahiro Harada

#ifndef CONVEX_HEIGHT_FIELD_SHAPE_H
#define CONVEX_HEIGHT_FIELD_SHAPE_H

#include "Stubs/AdlQuaternion.h"
#include "Stubs/AdlCollisionShape.h"
#include "Stubs/AdlAabb.h"

class ShapeBase;

class ConvexHeightField : public CollisionShape
{
	public:
		enum
		{
			HEIGHT_RES = 4, //was 4 originally
		};
		enum Face
		{
			FACE_XM,
			FACE_XP,
			FACE_YM,
			FACE_YP,
			FACE_ZM,
			FACE_ZP,
			NUM_FACES,
		};

		ConvexHeightField(const float4* vtxBuffer, const int4* idxBuffer, int nTriangles);
		ConvexHeightField(const ShapeBase* shape);
		ConvexHeightField(const float4* eqn, int nEqn);

		ConvexHeightField(): CollisionShape( SHAPE_CONVEX_HEIGHT_FIELD ){}

		virtual ~ConvexHeightField();

		//	CollisionShape interface
		virtual float queryDistance(const float4& p ) const;
		//	distance is not written to normalOut.w
		virtual bool queryDistanceWithNormal( const float4& p, float4& normalOut ) const;

		float querySupportHeight(const float4& p ) const;

		//	what is it?
		float queryW(const float4& p ) const;

		//	others
		u8& sample(Face face, int x, int y);
		u8 sample(Face face, int x, int y) const;

		u8& sampleSupport(Face face, int x, int y);
		u8 sampleSupport(Face face, int x, int y) const;

		float4& sampleNormal(Face face, int x, int y);
		float4 sampleNormal(Face face, int x, int y) const;

		void calcSamplePoints(float4* points) const;
		float4 calcSamplePoint(int sIdx) const;
		const float4* getSamplePoints() const;
		
		int getNumSamplePoints() const;

		//void displaySamples(const float4& translation, const Quaternion& quaternion) const;

	private:
		void create( const float4* vtxBuffer, const int4* idxBuffer, int nTriangles );

	public:
		u8 m_data[HEIGHT_RES*HEIGHT_RES*6];
		float4 m_normal[HEIGHT_RES*HEIGHT_RES*6];
		float m_scale;

		u8 m_supportHeight[HEIGHT_RES*HEIGHT_RES*6];

		float4 m_samplePoints[HEIGHT_RES*HEIGHT_RES*6];
		Aabb m_faceAabbs[6];
};

__inline
u8& ConvexHeightField::sample(Face face, int x, int y)
{
	ADLASSERT( x < HEIGHT_RES );
	ADLASSERT( y < HEIGHT_RES );
	return m_data[ HEIGHT_RES*HEIGHT_RES*face + x + y*HEIGHT_RES ];
}

__inline
u8 ConvexHeightField::sample(Face face, int x, int y) const
{
	ADLASSERT( x < HEIGHT_RES );
	ADLASSERT( y < HEIGHT_RES );
	return m_data[ HEIGHT_RES*HEIGHT_RES*face + x + y*HEIGHT_RES ];
}

__inline
u8& ConvexHeightField::sampleSupport(Face face, int x, int y)
{
	ADLASSERT( x < HEIGHT_RES );
	ADLASSERT( y < HEIGHT_RES );
	return m_supportHeight[ HEIGHT_RES*HEIGHT_RES*face + x + y*HEIGHT_RES ];
}

__inline
u8 ConvexHeightField::sampleSupport(Face face, int x, int y) const
{
	ADLASSERT( x < HEIGHT_RES );
	ADLASSERT( y < HEIGHT_RES );
	return m_supportHeight[ HEIGHT_RES*HEIGHT_RES*face + x + y*HEIGHT_RES ];
}

__inline
float4& ConvexHeightField::sampleNormal(Face face, int x, int y)
{
	ADLASSERT( x < HEIGHT_RES );
	ADLASSERT( y < HEIGHT_RES );
	return m_normal[ HEIGHT_RES*HEIGHT_RES*face + x + y*HEIGHT_RES ];
}

__inline
float4 ConvexHeightField::sampleNormal(Face face, int x, int y) const
{
	ADLASSERT( x < HEIGHT_RES );
	ADLASSERT( y < HEIGHT_RES );
	return m_normal[ HEIGHT_RES*HEIGHT_RES*face + x + y*HEIGHT_RES ];
}


#endif

