#ifndef _68DA1F85_90B7_4bb0_A705_83B4040A75C6_
#define _68DA1F85_90B7_4bb0_A705_83B4040A75C6_
#include "BulletCollision/CollisionShapes/btConvexShape.h"

///btGjkEpaSolver contributed under zlib by Nathanael Presson
struct	btGjkEpaSolver2
{
struct	sResults
	{
	enum eStatus
		{
		Separated,		/* Shapes doesnt penetrate												*/ 
		Penetrating,	/* Shapes are penetrating												*/ 
		GJK_Failed,		/* GJK phase fail, no big issue, shapes are probably just 'touching'	*/ 
		EPA_Failed,		/* EPA phase fail, bigger problem, need to save parameters, and debug	*/ 
		}		status;
	btVector3	witnesses[2];
	btVector3	normal;
	};

static int		StackSizeRequirement();

static btScalar	Distance(	const btConvexShape* shape0,const btTransform& wtrs0,
							const btConvexShape* shape1,const btTransform& wtrs1,
							sResults&	results);
							
static btScalar	SignedDistance(	const btVector3& position,
								btScalar margin,
								const btConvexShape* shape,
								const btTransform& wtrs,
								sResults& results);

static bool		Penetration(const btConvexShape* shape0,const btTransform& wtrs0,
							const btConvexShape* shape1,const btTransform& wtrs1,
							const btVector3& guess,
							sResults&	results);
};

#endif
