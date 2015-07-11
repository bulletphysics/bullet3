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


#include "LoadMeshFromCollada.h"
#include <stdio.h> //fopen
#include "Bullet3Common/b3AlignedObjectArray.h"
#include <string>
#include "tinyxml/tinyxml.h"

#include "Bullet3Common/b3FileUtils.h"
#include "LinearMath/btHashMap.h"
#include <assert.h>
#include "btMatrix4x4.h"




struct VertexSource
{
	std::string m_positionArrayId;
	std::string m_normalArrayId;
};

struct TokenFloatArray
{
	btAlignedObjectArray<float>& m_values;
	TokenFloatArray(btAlignedObjectArray<float>& floatArray)
		:m_values(floatArray) {
	}
	inline void add(const char* token)
	{
		float v = atof(token);
		m_values.push_back(v);
	}
};
struct TokenIntArray
{
	btAlignedObjectArray<int>& m_values;
	TokenIntArray(btAlignedObjectArray<int>& intArray)
		:m_values(intArray)	{
	}
	inline void add(const char* token)
	{
		float v = atoi(token);
		m_values.push_back(v);
	}
};

template <typename AddToken>
void tokenize(const std::string& str, AddToken& tokenAdder, const std::string& delimiters = " ")
{
   std::string::size_type pos, lastPos = 0;
   while(true)
   {
      pos = str.find_first_of(delimiters, lastPos);
      if(pos == std::string::npos)
      {
         pos = str.length();
         if(pos != lastPos)
		 {
			 tokenAdder.add(str.data()+lastPos);
		 }
         break;
      }
      else
      {
         if(pos != lastPos)
		 {
			tokenAdder.add(str.data()+lastPos);
		 }
      }
      lastPos = pos + 1;
   }
}


void	readFloatArray(TiXmlElement* source, btAlignedObjectArray<float>& floatArray, int& componentStride) 
{
	int numVals, stride;
	TiXmlElement* array = source->FirstChildElement("float_array");
	if(array) 
	{
		componentStride = 1;
		if (source->FirstChildElement("technique_common")->FirstChildElement("accessor")->QueryIntAttribute("stride", &stride)!= TIXML_NO_ATTRIBUTE)
		{
			componentStride = stride;
		}
		array->QueryIntAttribute("count", &numVals);
		TokenFloatArray adder(floatArray);
		floatArray.reserve(numVals);
		tokenize(array->GetText(),adder);
		assert(floatArray.size() == numVals);
	}
}

btVector3 getVector3FromXmlText(const char* text)
{
	btVector3 vec(0,0,0);
	btAlignedObjectArray<float> floatArray;
	TokenFloatArray adder(floatArray);
	floatArray.reserve(3);
	tokenize(text,adder);
	assert(floatArray.size() == 3);
	if (floatArray.size()==3)
	{
		vec.setValue(floatArray[0],floatArray[1],floatArray[2]);
	}
	return vec;
}

btVector4 getVector4FromXmlText(const char* text)
{
	btVector4 vec(0,0,0,0);
	btAlignedObjectArray<float> floatArray;
	TokenFloatArray adder(floatArray);
	floatArray.reserve(4);
	tokenize(text,adder);
	assert(floatArray.size() == 4);
	if (floatArray.size()==4)
	{
		vec.setValue(floatArray[0],floatArray[1],floatArray[2],floatArray[3]);
	}
	return vec;
}


void readLibraryGeometries(TiXmlDocument& doc, btAlignedObjectArray<GLInstanceGraphicsShape>& visualShapes, btHashMap<btHashString,int>& name2Shape, float extraScaling) 
{
	btHashMap<btHashString,TiXmlElement* > allSources;
	btHashMap<btHashString,VertexSource> vertexSources;
	for(TiXmlElement* geometry = doc.RootElement()->FirstChildElement("library_geometries")->FirstChildElement("geometry");
			geometry != NULL; geometry = geometry->NextSiblingElement("geometry")) 
	{
		btAlignedObjectArray<btVector3> vertexPositions;
		btAlignedObjectArray<btVector3> vertexNormals;
		btAlignedObjectArray<int> indices;

		const char* geometryName = geometry->Attribute("id");
		for (TiXmlElement* mesh = geometry->FirstChildElement("mesh");(mesh != NULL); mesh = mesh->NextSiblingElement("mesh")) 
		{
			TiXmlElement* vertices2 = mesh->FirstChildElement("vertices");
			
			for (TiXmlElement* source = mesh->FirstChildElement("source");source != NULL;source = source->NextSiblingElement("source")) 
			{
				const char* srcId= source->Attribute("id");
//				printf("source id=%s\n",srcId);
				allSources.insert(srcId,source);
			}
			const char* vertexId = vertices2->Attribute("id");
			//printf("vertices id=%s\n",vertexId);
			VertexSource vs;
			for(TiXmlElement* input = vertices2->FirstChildElement("input");input != NULL;input = input->NextSiblingElement("input")) 
			{
				const char* sem = input->Attribute("semantic");
				std::string semName(sem);
//					printf("sem=%s\n",sem);
		//		const char* src = input->Attribute("source");
//					printf("src=%s\n",src);
				const char* srcIdRef = input->Attribute("source");
				std::string source_name;
				source_name = std::string(srcIdRef);
				source_name = source_name.erase(0, 1);
				if (semName=="POSITION")
				{
					vs.m_positionArrayId = source_name;
				}
				if (semName=="NORMAL")
				{
					vs.m_normalArrayId = source_name;
				}
			}
			vertexSources.insert(vertexId,vs);

			for (TiXmlElement* primitive = mesh->FirstChildElement("triangles"); primitive; primitive = primitive->NextSiblingElement("triangles"))
			{
				std::string positionSourceName;
				std::string normalSourceName;
				int primitiveCount;
				primitive->QueryIntAttribute("count", &primitiveCount);
				int indexStride=1;
				int posOffset = 0;
				int normalOffset = 0;
				int numIndices = 0;
				{

					for (TiXmlElement* input = primitive->FirstChildElement("input");input != NULL;input = input->NextSiblingElement("input")) 
					{
						const char* sem = input->Attribute("semantic");
						std::string semName(sem);
						int offset = atoi(input->Attribute("offset"));
						if ((offset+1)>indexStride)
							indexStride=offset+1;
						//printf("sem=%s\n",sem);
					//	const char* src = input->Attribute("source");

						//printf("src=%s\n",src);
						const char* srcIdRef = input->Attribute("source");
						std::string source_name;
						source_name = std::string(srcIdRef);
						source_name = source_name.erase(0, 1);
							
						if (semName=="VERTEX")
						{
							//now we have POSITION and possibly NORMAL too, using same index array (<p>)
							VertexSource* vs = vertexSources[source_name.c_str()];
							if (vs->m_positionArrayId.length())
							{
								positionSourceName = vs->m_positionArrayId;
								posOffset = offset;
							}
							if (vs->m_normalArrayId.length())
							{
								normalSourceName = vs->m_normalArrayId;
								normalOffset  = offset;
							}
						}
						if (semName=="NORMAL")
						{
							btAssert(normalSourceName.length()==0);
							normalSourceName = source_name;
							normalOffset  = offset;
						}
					}
					numIndices = primitiveCount * 3; 
				}
				btAlignedObjectArray<float> positionFloatArray;
				int posStride=1;
				TiXmlElement** sourcePtr = allSources[positionSourceName.c_str()];
				if (sourcePtr)
				{
					readFloatArray(*sourcePtr,positionFloatArray, posStride);
				}
				btAlignedObjectArray<float> normalFloatArray;
				int normalStride=1;
				sourcePtr = allSources[normalSourceName.c_str()];
				if (sourcePtr)
				{
					readFloatArray(*sourcePtr,normalFloatArray,normalStride);
				}
				btAlignedObjectArray<int> curIndices;
				curIndices.reserve(numIndices*indexStride);
				TokenIntArray adder(curIndices);
				tokenize(primitive->FirstChildElement("p")->GetText(),adder);
				assert(curIndices.size() == numIndices*indexStride);
				int indexOffset = vertexPositions.size();

				for(int index=0; index<numIndices; index++) 
				{
					int posIndex = curIndices[index*indexStride+posOffset];
					int normalIndex = curIndices[index*indexStride+normalOffset];
					vertexPositions.push_back(btVector3(extraScaling*positionFloatArray[posIndex*3+0],
						extraScaling*positionFloatArray[posIndex*3+1],
						extraScaling*positionFloatArray[posIndex*3+2]));
							
					if (normalFloatArray.size() && (normalFloatArray.size()>normalIndex))
					{
						vertexNormals.push_back(btVector3(normalFloatArray[normalIndex*3+0],
															normalFloatArray[normalIndex*3+1],
															normalFloatArray[normalIndex*3+2]));
					} else
					{
						//add a dummy normal of length zero, so it is easy to detect that it is an invalid normal
						vertexNormals.push_back(btVector3(0,0,0));
					}
				}
				int curNumIndices = indices.size();
				indices.resize(curNumIndices+numIndices);
				for(int index=0; index<numIndices; index++) 
				{
					indices[curNumIndices+index] = index+indexOffset;
				}
			}//if(primitive != NULL) 
		}//for each mesh
		
		int shapeIndex = visualShapes.size();
		GLInstanceGraphicsShape& visualShape = visualShapes.expand();
		{
			visualShape.m_vertices = new b3AlignedObjectArray<GLInstanceVertex>;
			visualShape.m_indices = new b3AlignedObjectArray<int>;
			int indexBase = 0;

			btAssert(vertexNormals.size()==vertexPositions.size());
			for (int v=0;v<vertexPositions.size();v++)
			{
				GLInstanceVertex vtx;
				vtx.xyzw[0] = vertexPositions[v].x();
				vtx.xyzw[1] = vertexPositions[v].y();
				vtx.xyzw[2] = vertexPositions[v].z();
				vtx.xyzw[3] = 1.f;
				vtx.normal[0] = vertexNormals[v].x();
				vtx.normal[1] = vertexNormals[v].y();
				vtx.normal[2] = vertexNormals[v].z();
				vtx.uv[0] = 0.5f;
				vtx.uv[1] = 0.5f;
				visualShape.m_vertices->push_back(vtx);
			}

			for (int index=0;index<indices.size();index++)
			{
				visualShape.m_indices->push_back(indices[index]+indexBase);
			}
			
			
			printf(" index_count =%dand vertexPositions.size=%d\n",indices.size(), vertexPositions.size());
			indexBase=visualShape.m_vertices->size();
			visualShape.m_numIndices = visualShape.m_indices->size();
			visualShape.m_numvertices = visualShape.m_vertices->size();
		}
		printf("geometry name=%s\n",geometryName);
		name2Shape.insert(geometryName,shapeIndex);
		

	}//for each geometry
}

void readNodeHierarchy(TiXmlElement* node,btHashMap<btHashString,int>& name2Shape, btAlignedObjectArray<ColladaGraphicsInstance>& visualShapeInstances,  const btMatrix4x4& parentTransMat)
{
	const char* nodeName = node->Attribute("id");
	printf("processing node %s\n", nodeName);

	
	btMatrix4x4 nodeTrans;
	nodeTrans.setIdentity();

	///todo(erwincoumans) we probably have to read the elements 'translate', 'scale', 'rotate' and 'matrix' in-order and accumulate them...
	{
		for (TiXmlElement* transElem = node->FirstChildElement("matrix");transElem;transElem=node->NextSiblingElement("matrix"))
		{
			if (transElem->GetText())
			{
				btAlignedObjectArray<float> floatArray;
				TokenFloatArray adder(floatArray);
				tokenize(transElem->GetText(),adder);
				if (floatArray.size()==16)
				{
					btMatrix4x4 t(floatArray[0],floatArray[1],floatArray[2],floatArray[3],
									floatArray[4],floatArray[5],floatArray[6],floatArray[7],
									floatArray[8],floatArray[9],floatArray[10],floatArray[11],
									floatArray[12],floatArray[13],floatArray[14],floatArray[15]);

					nodeTrans = nodeTrans*t;
				} else
				{
					printf("Error: expected 16 elements in a <matrix> element, skipping\n");
				}
			}
		}
	}

	{
		for (TiXmlElement* transElem = node->FirstChildElement("translate");transElem;transElem=node->NextSiblingElement("translate"))
		{
			if (transElem->GetText())
			{
				btVector3 pos = getVector3FromXmlText(transElem->GetText());
				//nodePos+= unitScaling*parentScaling*pos;
				btMatrix4x4 t;
				t.setPureTranslation(pos);
				nodeTrans = nodeTrans*t;

			}
		}
	}
	{
		for(TiXmlElement* scaleElem = node->FirstChildElement("scale");
				scaleElem!= NULL; scaleElem= node->NextSiblingElement("scale")) 
		{
			if (scaleElem->GetText())
			{
				btVector3 scaling = getVector3FromXmlText(scaleElem->GetText());
				btMatrix4x4 t;
				t.setPureScaling(scaling);
				nodeTrans = nodeTrans*t;
			}
		}
	}
	{
		for(TiXmlElement* rotateElem = node->FirstChildElement("rotate");
				rotateElem!= NULL; rotateElem= node->NextSiblingElement("rotate")) 
		{
			if (rotateElem->GetText())
			{
				//accumulate orientation
				btVector4 rotate = getVector4FromXmlText(rotateElem->GetText());
				btQuaternion orn(btVector3(rotate),btRadians(rotate[3]));//COLLADA DAE rotate is in degrees, convert to radians
				btMatrix4x4 t;
				t.setPureRotation(orn);
				nodeTrans = nodeTrans*t;
			}
		}
	}
	
	nodeTrans = parentTransMat*nodeTrans;
	
	for (TiXmlElement* instanceGeom = node->FirstChildElement("instance_geometry");
				instanceGeom!=0;
				instanceGeom=instanceGeom->NextSiblingElement("instance_geometry"))
	{
		const char* geomUrl = instanceGeom->Attribute("url");
		printf("node referring to geom %s\n", geomUrl);
		geomUrl++;
		int* shapeIndexPtr = name2Shape[geomUrl];
		if (shapeIndexPtr)
		{
		//	int index = *shapeIndexPtr;
			printf("found geom with index %d\n", *shapeIndexPtr);
			ColladaGraphicsInstance& instance = visualShapeInstances.expand();
			instance.m_shapeIndex = *shapeIndexPtr;
			instance.m_worldTransform = nodeTrans;
		} else
		{
			printf("geom not found\n");
		}
	}

	for(TiXmlElement* childNode = node->FirstChildElement("node");
			childNode!= NULL; childNode = childNode->NextSiblingElement("node")) 
	{
		readNodeHierarchy(childNode,name2Shape,visualShapeInstances, nodeTrans);
	}
}
void readVisualSceneInstanceGeometries(TiXmlDocument& doc, btHashMap<btHashString,int>& name2Shape, btAlignedObjectArray<ColladaGraphicsInstance>& visualShapeInstances)
{
	btHashMap<btHashString,TiXmlElement* > allVisualScenes;

	TiXmlElement* libVisualScenes = doc.RootElement()->FirstChildElement("library_visual_scenes");
	if (libVisualScenes==0)
		return;

	{
		for(TiXmlElement* scene = libVisualScenes->FirstChildElement("visual_scene");
				scene != NULL; scene = scene->NextSiblingElement("visual_scene")) 
		{
			const char* sceneName = scene->Attribute("id");
			allVisualScenes.insert(sceneName,scene);
		}
	}

	TiXmlElement* scene = 0;
	{
		TiXmlElement* scenes = doc.RootElement()->FirstChildElement("scene");
		if (scenes)
		{
			TiXmlElement* instanceSceneReference = scenes->FirstChildElement("instance_visual_scene");
			if (instanceSceneReference)
			{
				const char* instanceSceneUrl = instanceSceneReference->Attribute("url");
				TiXmlElement** sceneInstancePtr = allVisualScenes[instanceSceneUrl+1];//skip #
				if (sceneInstancePtr)
				{
					scene = *sceneInstancePtr;
				}
			}
		}
	}

	if (scene)
	{
		for(TiXmlElement* node = scene->FirstChildElement("node");
			node != NULL; node = node->NextSiblingElement("node")) 
		{
			btMatrix4x4 identity;
			identity.setIdentity();
			btVector3 identScaling(1,1,1);
			readNodeHierarchy(node,name2Shape,visualShapeInstances, identity);

		}
		
	}
}

void getUnitMeterScalingAndUpAxisTransform(TiXmlDocument& doc, btTransform& tr, float& unitMeterScaling, int clientUpAxis)
{
	///todo(erwincoumans) those up-axis transformations have been quickly coded without rigorous testing
	
	TiXmlElement* unitMeter = doc.RootElement()->FirstChildElement("asset")->FirstChildElement("unit");
	if (unitMeter)
	{
		const char* meterText = unitMeter->Attribute("meter");
		printf("meterText=%s\n", meterText);
		unitMeterScaling = atof(meterText);
	}

	TiXmlElement* upAxisElem = doc.RootElement()->FirstChildElement("asset")->FirstChildElement("up_axis");
	if (upAxisElem)
	{
		switch (clientUpAxis)
		{
			
			case 1:
			{
				std::string upAxisTxt = upAxisElem->GetText();
				if (upAxisTxt == "X_UP")
				{
					btQuaternion x2y(btVector3(0,0,1),SIMD_HALF_PI);
					tr.setRotation(x2y);
				}
				if (upAxisTxt == "Y_UP")
				{
					//assume Y_UP for now, to be compatible with assimp?
					//client and COLLADA are both Z_UP so no transform needed (identity)
				}
				if (upAxisTxt == "Z_UP")
				{
					btQuaternion z2y(btVector3(1,0,0),-SIMD_HALF_PI);
					tr.setRotation(z2y);
				}
				break;
			}
			case 2:
			{
				std::string upAxisTxt = upAxisElem->GetText();
				if (upAxisTxt == "X_UP")
				{
					btQuaternion x2z(btVector3(0,1,0),-SIMD_HALF_PI);
					tr.setRotation(x2z);
				}
				if (upAxisTxt == "Y_UP")
				{
					btQuaternion y2z(btVector3(1,0,0),SIMD_HALF_PI);
					tr.setRotation(y2z);
				}
				if (upAxisTxt == "Z_UP")
				{
					//client and COLLADA are both Z_UP so no transform needed (identity)
				}
				break;
			}
			case 0:
			default:
			{
				//we don't support X or other up axis
				btAssert(0);
			}
		};
	}
}

void LoadMeshFromCollada(const char* relativeFileName, btAlignedObjectArray<GLInstanceGraphicsShape>& visualShapes, btAlignedObjectArray<ColladaGraphicsInstance>& visualShapeInstances, btTransform& upAxisTransform, float& unitMeterScaling,int clientUpAxis)
{

//	GLInstanceGraphicsShape* instance = 0;
	
	//usually COLLADA files don't have that many visual geometries/shapes
	visualShapes.reserve(32);

	float extraScaling = 1;//0.01;
	btHashMap<btHashString, int> name2ShapeIndex;
	b3FileUtils f;
	char filename[1024];
	if (!f.findFile(relativeFileName,filename,1024))
	{
		printf("File not found: %s\n", filename);
		return;
	}
	 
	TiXmlDocument doc(filename);
	if (!doc.LoadFile())
		return;

	//We need units to be in meter, so apply a scaling using the asset/units meter 
	unitMeterScaling=1;
	upAxisTransform.setIdentity();

	//Also we can optionally compensate all transforms using the asset/up_axis as well as unit meter scaling
	getUnitMeterScalingAndUpAxisTransform(doc, upAxisTransform, unitMeterScaling,clientUpAxis);
	
	btMatrix4x4 ident;
	ident.setIdentity();

	readLibraryGeometries(doc, visualShapes, name2ShapeIndex, extraScaling);
	
	readVisualSceneInstanceGeometries(doc, name2ShapeIndex, visualShapeInstances);

}



#ifdef COMPARE_WITH_ASSIMP

#include <assimp/Importer.hpp>
#include <assimp/mesh.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>


#	include "assimp/ColladaLoader.h"
//#	include "STLLoader.h"
#	include "assimp/SortByPTypeProcess.h"
#	include "assimp/LimitBoneWeightsProcess.h"
#	include "assimp/TriangulateProcess.h"
#	include "assimp/JoinVerticesProcess.h"
#	include "assimp/RemoveVCProcess.h"


namespace Assimp {
	// ------------------------------------------------------------------------------------------------
void GetImporterInstanceList(std::vector< BaseImporter* >& out)
	{
		out.push_back( new ColladaLoader());
	}
	// ------------------------------------------------------------------------------------------------
void GetPostProcessingStepInstanceList(std::vector< BaseProcess* >& out)
	{
		out.push_back( new SortByPTypeProcess());
		out.push_back( new LimitBoneWeightsProcess());
		out.push_back( new TriangulateProcess());
		out.push_back( new JoinVerticesProcess());
		//out.push_back( new RemoveVCProcess());
	}
	
}

static void addMeshParts(const aiScene* scene, const aiNode* node, GLInstanceGraphicsShape* outverts, const aiMatrix4x4& parentTr)
{
        aiMatrix4x4 const& nodeTrans(node->mTransformation);
		
		aiMatrix4x4 trans;
        trans = parentTr * nodeTrans;
		
        for (size_t i = 0; i < node->mNumMeshes; ++i) 
		{
            aiMesh const* mesh = scene->mMeshes[node->mMeshes[i]];
            size_t num_vertices = mesh->mNumVertices;
			if (mesh->mPrimitiveTypes==aiPrimitiveType_TRIANGLE)
			{
				int curVertexBase = outverts->m_vertices->size();
				
				for (int v=0;v<mesh->mNumVertices;v++)
				{
					GLInstanceVertex vtx;
					aiVector3D vWorld = trans*mesh->mVertices[v];
					vtx.xyzw[0] = vWorld.x;
					vtx.xyzw[1] = vWorld.y;
					vtx.xyzw[2] = vWorld.z;
					vtx.xyzw[3] = 1;
					if (mesh->HasNormals())
					{
						vtx.normal[0] = mesh->mNormals[v].x;
						vtx.normal[1] = mesh->mNormals[v].y;
						vtx.normal[2] = mesh->mNormals[v].z;
					} else
					{
						vtx.normal[0] = 0;
						vtx.normal[1] = 0;
						vtx.normal[2] = 1;
					}
					if (mesh->HasTextureCoords(0))
					{
						vtx.uv[0] = mesh->mTextureCoords[0][v].x;
						vtx.uv[1] = mesh->mTextureCoords[0][v].y;
					} else
					{
						vtx.uv[0]=0.5f;
						vtx.uv[1]=0.5f;
					}
					outverts->m_vertices->push_back(vtx);
				}
				for (int f=0;f<mesh->mNumFaces;f++)
				{
					b3Assert(mesh->mFaces[f].mNumIndices == 3);
					int i0 = mesh->mFaces[f].mIndices[0];
					int i1 = mesh->mFaces[f].mIndices[1];
					int i2 = mesh->mFaces[f].mIndices[2];
					outverts->m_indices->push_back(i0+curVertexBase);
					outverts->m_indices->push_back(i1+curVertexBase);
					outverts->m_indices->push_back(i2+curVertexBase);
				}
			}
        }
        for (size_t i=0 ; i<node->mNumChildren ; ++i) {
            addMeshParts(scene,node->mChildren[i], outverts, trans);
        }
}


void LoadMeshFromColladaAssimp(const char* relativeFileName, btAlignedObjectArray<GLInstanceGraphicsShape>& visualShapes, btAlignedObjectArray<ColladaGraphicsInstance>& visualShapeInstances,btTransform& upAxisTrans, float& unitMeterScaling)
{
	upAxisTrans.setIdentity();
	unitMeterScaling=1;

	GLInstanceGraphicsShape* shape = 0;
	
	
	FILE* file = fopen(relativeFileName,"rb");
	if (file)
	{
		int size=0;
		if (fseek(file, 0, SEEK_END) || (size = ftell(file)) == EOF || fseek(file, 0, SEEK_SET))
		{
			printf("Error: Cannot access file to determine size of %s\n", relativeFileName);
		} else
		{
			if (size)
			{
				printf("Open DAE file of %d bytes\n",size);
				
				Assimp::Importer importer;
				//importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS, aiComponent_NORMALS | aiComponent_COLORS);
				importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_LINE | aiPrimitiveType_POINT);
			//	importer.SetPropertyInteger(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, 1);
				aiScene const* scene = importer.ReadFile(relativeFileName,
						aiProcess_JoinIdenticalVertices |
						//aiProcess_RemoveComponent |
						aiProcess_SortByPType |
						aiProcess_Triangulate);
				if (scene)
				{
					shape = &visualShapes.expand();
					shape->m_scaling[0] = 1;
					shape->m_scaling[1] = 1;
					shape->m_scaling[2] = 1;
					shape->m_scaling[3] = 1;
					int index = 0;
					shape->m_indices = new b3AlignedObjectArray<int>();
					shape->m_vertices = new b3AlignedObjectArray<GLInstanceVertex>();

					aiMatrix4x4 ident;
					addMeshParts(scene, scene->mRootNode, shape, ident);
					 shape->m_numIndices = shape->m_indices->size();
					shape->m_numvertices = shape->m_vertices->size();
					ColladaGraphicsInstance& instance = visualShapeInstances.expand();
					instance.m_shapeIndex = visualShapes.size()-1;
				}
			}
		}
		
	}
	
}

#endif //COMPARE_WITH_ASSIMP
