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
#include "CcdPhysicsEnvironment.h"
#include "LinearMath/btVector3.h"

void BspConverter::convertBsp(BspLoader& bspLoader,float scaling)
{
	{
		btVector3 playerStart (0.f, 0.f, 100.f);

		if (bspLoader.findVectorByName(&playerStart[0],"info_player_start"))
		{
			printf("found playerstart\n");
		} 
		else
		{
			if (bspLoader.findVectorByName(&playerStart[0],"info_player_deathmatch"))
			{
				printf("found deatchmatch start\n");
			}
		}

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
				std::vector<btVector3> planeEquations;
				
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
								plane.normal[2],
								scaling*-plane.dist);

							planeEquations.push_back(planeEq);
							isValidBrush=true;
						}
						if (isValidBrush)
						{

							std::vector<btVector3>	vertices;
							
							getVerticesFromPlaneEquations(planeEquations,vertices);
							printf("getVerticesFromPlaneEquations returned %i\n",(int)vertices.size());

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

							//lookup the target position for the jumppad:
							const BSPEntity* targetentity = bspLoader.getEntityByValue( "targetname" , cl );
							if (targetentity)
							{
								if (bspLoader.getVectorForKey( targetentity , "origin",&targetLocation[0]))
								{
																	
								}
							}

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
											std::vector<btVector3> planeEquations;
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
														plane.normal[2],
														scaling*-plane.dist);
													planeEquations.push_back(planeEq);
													isValidBrush=true;
												}
												if (isValidBrush)
												{
													
													std::vector<btVector3>	vertices;
													getVerticesFromPlaneEquations(planeEquations,vertices);

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






void	BspConverter::getVerticesFromPlaneEquations(const std::vector<btVector3>& planeEquations , std::vector<btVector3>& verticesOut )
{
	const int numbrushes = planeEquations.size();
	// brute force:
	for (int i=0;i<numbrushes;i++)
	{
		const btVector3& N1 = planeEquations[i];
		

		for (int j=i+1;j<numbrushes;j++)
		{
			const btVector3& N2 = planeEquations[j];
				
			for (int k=j+1;k<numbrushes;k++)
			{

				const btVector3& N3 = planeEquations[k];

				btVector3 n2n3; n2n3 = N2.cross(N3);
				btVector3 n3n1; n3n1 = N3.cross(N1);
				btVector3 n1n2; n1n2 = N1.cross(N2);
				
				if ( ( n2n3.length2() > 0.0001f ) &&
					 ( n3n1.length2() > 0.0001f ) &&
					 ( n1n2.length2() > 0.0001f ) )
				{
					//point P out of 3 plane equations:

					//	d1 ( N2 * N3 ) + d2 ( N3 * N1 ) + d3 ( N1 * N2 )  
					//P =  -------------------------------------------------------------------------  
					//   N1 . ( N2 * N3 )  


					float quotient = (N1.dot(n2n3));
					if (btFabs(quotient) > 0.000001f)
					{
						quotient = -1.f / quotient;
						n2n3 *= N1[3];
						n3n1 *= N2[3];
						n1n2 *= N3[3];
						btVector3 potentialVertex = n2n3;
						potentialVertex += n3n1;
						potentialVertex += n1n2;
						potentialVertex *= quotient;

						//check if inside, and replace supportingVertexOut if needed
						if (isInside(planeEquations,potentialVertex,0.1f))
						{
							verticesOut.push_back(potentialVertex);
						}
					}
				}
			}
		}
	}
}




bool	BspConverter::isInside(const std::vector<btVector3>& planeEquations, const btVector3& point, float	margin)
{
	int numbrushes = planeEquations.size();
	for (int i=0;i<numbrushes;i++)
	{
		const btVector3& N1 = planeEquations[i];
		float dist = float(N1.dot(point))+float(N1[3])-margin;
		if (dist>0.f)
		{
			return false;
		}
	}
	return true;
		
}
