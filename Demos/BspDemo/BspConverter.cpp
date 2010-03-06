/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "BspConverter.h"
#include "BspLoader.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btGeometryUtil.h"
#include <stdio.h>
#include <string.h>


void BspConverter::convertBsp(BspLoader& bspLoader,float scaling)
{
	{
		
		float	playstartf[3] = {0,0,100};

		if (bspLoader.findVectorByName(&playstartf[0],"info_player_start"))
		{
			printf("found playerstart\n");
		} 
		else
		{
			if (bspLoader.findVectorByName(&playstartf[0],"info_player_deathmatch"))
			{
				printf("found deatchmatch start\n");
			}
		}

		btVector3 playerStart (playstartf[0],playstartf[1],playstartf[2]);


		playerStart[2] += 20.f; //start a bit higher

		playerStart *= scaling;

		

		//progressBegin("Loading bsp");

		for (int i=0;i<bspLoader.m_numleafs;i++)
		{
			printf("Reading bspLeaf %i from total %i (%f procent)\n",i, bspLoader.m_numleafs,(100.f*(float)i/float(bspLoader.m_numleafs)) );
			
			bool isValidBrush = false;
			
			BSPLeaf&	leaf = bspLoader.m_dleafs[i];
	
			for (int b=0;b<leaf.numLeafBrushes;b++)
			{
				btAlignedObjectArray<btVector3> planeEquations;
				
				int brushid = bspLoader.m_dleafbrushes[leaf.firstLeafBrush+b];

				BSPBrush& brush = bspLoader.m_dbrushes[brushid];
				if (brush.shaderNum!=-1)
				{
					if (bspLoader.m_dshaders[ brush.shaderNum ].contentFlags & BSPCONTENTS_SOLID)
					{
						brush.shaderNum = -1;

						for (int p=0;p<brush.numSides;p++)
						{
							int sideid = brush.firstSide+p;
							BSPBrushSide& brushside = bspLoader.m_dbrushsides[sideid];
							int planeid = brushside.planeNum;
							BSPPlane& plane = bspLoader.m_dplanes[planeid];
							btVector3 planeEq;
							planeEq.setValue(
								plane.normal[0],
								plane.normal[1],
								plane.normal[2]);
							planeEq[3] = scaling*-plane.dist;

							planeEquations.push_back(planeEq);
							isValidBrush=true;
						}
						if (isValidBrush)
						{

							btAlignedObjectArray<btVector3>	vertices;
							btGeometryUtil::getVerticesFromPlaneEquations(planeEquations,vertices);
						
							bool isEntity = false;
							btVector3 entityTarget(0.f,0.f,0.f);
							addConvexVerticesCollider(vertices,isEntity,entityTarget);
						
						}
					}
				} 
			}
		}

#define USE_ENTITIES
#ifdef USE_ENTITIES

		
		{
			int i;
			for (i=0;i<bspLoader.m_num_entities;i++)
				{
					const BSPEntity& entity = bspLoader.m_entities[i];
					const char* cl = bspLoader.getValueForKey(&entity,"classname");
					if ( !strcmp( cl, "trigger_push" ) ) {
						btVector3 targetLocation(0.f,0.f,0.f);

						cl = bspLoader.getValueForKey(&entity,"target");
						if ( strcmp( cl, "" ) ) {
							//its not empty so ...

							/*
							//lookup the target position for the jumppad:
							const BSPEntity* targetentity = bspLoader.getEntityByValue( "targetname" , cl );
							if (targetentity)
							{
								if (bspLoader.getVectorForKey( targetentity , "origin",&targetLocation[0]))
								{
																	
								}
							}
							*/


							cl = bspLoader.getValueForKey(&entity,"model");
							if ( strcmp( cl, "" ) ) {
								// add the model as a brush
								if (cl[0] == '*')
								{
									int modelnr = atoi(&cl[1]);
									if ((modelnr >=0) && (modelnr < bspLoader.m_nummodels))
									{
										const BSPModel& model = bspLoader.m_dmodels[modelnr];
										for (int n=0;n<model.numBrushes;n++)
										{
											btAlignedObjectArray<btVector3> planeEquations;
											bool	isValidBrush = false;

											//convert brush
											const BSPBrush& brush = bspLoader.m_dbrushes[model.firstBrush+n];
											{
												for (int p=0;p<brush.numSides;p++)
												{
													int sideid = brush.firstSide+p;
													BSPBrushSide& brushside = bspLoader.m_dbrushsides[sideid];
													int planeid = brushside.planeNum;
													BSPPlane& plane = bspLoader.m_dplanes[planeid];
													btVector3 planeEq;
													planeEq.setValue(
														plane.normal[0],
														plane.normal[1],
														plane.normal[2]);
													planeEq[3] = scaling*-plane.dist;
													planeEquations.push_back(planeEq);
													isValidBrush=true;
												}
												if (isValidBrush)
												{
													
													btAlignedObjectArray<btVector3>	vertices;
													btGeometryUtil::getVerticesFromPlaneEquations(planeEquations,vertices);

													bool isEntity=true;
													addConvexVerticesCollider(vertices,isEntity,targetLocation);
													
												}
											}

										}
									}
								} 
								else
								{
									printf("unsupported trigger_push model, md3 ?\n");
								}
							}
						
						}
					}
				}
			}
					
#endif //USE_ENTITIES

		

		//progressEnd();
		}
	
	}






