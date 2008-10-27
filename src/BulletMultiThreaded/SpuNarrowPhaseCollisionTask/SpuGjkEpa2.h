#ifndef _68DA1F85_90B7_4bb0_A705_83B4040A75C6_
#define _68DA1F85_90B7_4bb0_A705_83B4040A75C6_
#include "BulletCollision/CollisionShapes/btConvexShape.h"

///btGjkEpaSolver contributed under zlib by Nathanael Presson
struct	SpuGjkEpaSolver2
{
struct	sResults
	{
	enum eStatus
		{
		Separated,		/* Shapes doesnt penetrate												*/ 
		Penetrating,	/* Shapes are penetrating												*/ 
		GJK_Failed,		/* GJK phase fail, no big issue, shapes are probably just 'touching'	*/ 
		EPA_Failed		/* EPA phase fail, bigger problem, need to save parameters, and debug	*/ 
		}		status;
	btVector3	witnesses[2];
	btVector3	normal;
	};

static int		StackSizeRequirement();


static bool		Penetration(void* shapeA,
							SpuConvexPolyhedronVertexData* convexDataA,
							int shapeTypeA,
							float marginA,
							const btTransform& xformA,
							void* shapeB,
							SpuConvexPolyhedronVertexData* convexDataB,
							int shapeTypeB,
							float marginB,
							const btTransform& xformB,
							const btVector3& guess,
							sResults&	results);
};

#endif
