
#ifndef B3_COLLIDABLE_H
#define B3_COLLIDABLE_H


#include "Bullet3Common/shared/b3Float4.h"
#include "Bullet3Common/shared/b3Quat.h"

enum b3ShapeTypes
{
	SHAPE_HEIGHT_FIELD=1,

	SHAPE_CONVEX_HULL=3,
	SHAPE_PLANE=4,
	SHAPE_CONCAVE_TRIMESH=5,
	SHAPE_COMPOUND_OF_CONVEX_HULLS=6,
	SHAPE_SPHERE=7,
	SHAPE_CAPSULE=8,
	SHAPE_COMPOUND_OF_SPHERES=9,
	SHAPE_COMPOUND_OF_CAPSULES=10,
	MAX_NUM_SHAPE_TYPES,
};

typedef struct b3Collidable b3Collidable_t;


struct b3Collidable
{
	union {
		int m_numChildShapes;
		int m_bvhIndex;
	};
	union
	{
		float m_radius;
		int	m_compoundBvhIndex;
	};

	int m_shapeType;
	int m_shapeIndex;
};

typedef struct b3GpuChildShape b3GpuChildShape_t;
struct b3GpuChildShape
{
	b3Float4	m_childPosition;
	b3Quat		m_childOrientation;
	int			m_shapeIndex;//used for SHAPE_COMPOUND_OF_CONVEX_HULLS
	float		m_radius;//used for SHAPE_COMPOUND_OF_SPHERES or SHAPE_COMPOUND_OF_CAPSULES
	float		m_height;//used for SHAPE_COMPOUND_OF_CAPSULES
	int			m_unused2;
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
