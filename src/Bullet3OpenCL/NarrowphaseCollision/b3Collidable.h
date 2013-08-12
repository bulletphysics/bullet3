
#ifndef B3_COLLIDABLE_H
#define B3_COLLIDABLE_H

#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"

enum b3ShapeTypes
{
	SHAPE_HEIGHT_FIELD=1,

	SHAPE_CONVEX_HULL=3,
	SHAPE_PLANE=4,
	SHAPE_CONCAVE_TRIMESH=5,
	SHAPE_COMPOUND_OF_CONVEX_HULLS=6,
	SHAPE_SPHERE=7,
	MAX_NUM_SHAPE_TYPES,
};

struct b3Collidable
{
	union {
		int m_numChildShapes;
		int m_bvhIndex;
	};
	float m_radius;
	int m_shapeType;
	int m_shapeIndex;
};

struct b3CollidableNew
{
	short int m_shapeType;
	short int m_numShapes;
	int m_shapeIndex;
};

struct b3GpuChildShape
{
	b3Vector3	m_childPosition;
	b3Quaternion m_childOrientation;
	int m_shapeIndex;
	int m_unused0;
	int m_unused1;
	int m_unused2;
};

struct b3CompoundOverlappingPair
{
	int m_bodyIndexA;
	int m_bodyIndexB;
//	int	m_pairType;
	int m_childShapeIndexA;
	int m_childShapeIndexB;
};
#endif //B3_COLLIDABLE_H
