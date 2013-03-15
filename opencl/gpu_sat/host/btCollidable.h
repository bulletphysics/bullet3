
#ifndef BT_COLLIDABLE_H
#define BT_COLLIDABLE_H

enum btShapeTypes
{
	SHAPE_HEIGHT_FIELD=1,
	SHAPE_CONVEX_HEIGHT_FIELD=2,
	SHAPE_CONVEX_HULL=3,
	SHAPE_PLANE=4,
	SHAPE_CONCAVE_TRIMESH=5,
	SHAPE_COMPOUND_OF_CONVEX_HULLS=6,
	SHAPE_SPHERE=7,
	MAX_NUM_SHAPE_TYPES,
};

struct btCollidable
{
	int m_numChildShapes;
	float m_radius;
	int m_shapeType;
	int m_shapeIndex;
};

struct btCollidableNew
{
	short int m_shapeType;
	short int m_numShapes;
	int m_shapeIndex;
};

struct btGpuChildShape
{
	float	m_childPosition[4];
	float	m_childOrientation[4];
	int m_shapeIndex;
	int m_unused0;
	int m_unused1;
	int m_unused2;
};

struct btCompoundOverlappingPair
{
	int m_bodyIndexA;
	int m_bodyIndexB;
//	int	m_pairType;
	int m_childShapeIndexA;
	int m_childShapeIndexB;
};
#endif //BT_COLLIDABLE_H
