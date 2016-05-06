/*
Bullet Collision Detection and Physics Library http://bulletphysics.org
This file is Copyright (c) 2014 Google Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

//original author: Erwin Coumans
*/


#ifndef LOAD_MESH_FROM_COLLADA_H
#define LOAD_MESH_FROM_COLLADA_H

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include "../../OpenGLWindow/GLInstanceGraphicsShape.h"
#include "ColladaGraphicsInstance.h"


void LoadMeshFromCollada(const char* relativeFileName,
						btAlignedObjectArray<GLInstanceGraphicsShape>& visualShapes, 
						btAlignedObjectArray<ColladaGraphicsInstance>& visualShapeInstances,
						btTransform& upAxisTrans,
						float& unitMeterScaling,
						 int clientUpAxis);

//#define COMPARE_WITH_ASSIMP
#ifdef COMPARE_WITH_ASSIMP
void LoadMeshFromColladaAssimp(const char* relativeFileName, 
						btAlignedObjectArray<GLInstanceGraphicsShape>& visualShapes, 
						btAlignedObjectArray<ColladaGraphicsInstance>& visualShapeInstances,
						btTransform& upAxisTrans,
						float& unitMeterScaling
						);
#endif //COMPARE_WITH_ASSIMP

#endif //LOAD_MESH_FROM_COLLADA_H
