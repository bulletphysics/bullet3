/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2012 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btBulletXmlWorldImporter.h"
#include "tinyxml2.h"
#include "btBulletDynamicsCommon.h"
#include "string_split.h"
using namespace tinyxml2;

struct MyLocalCaster
{
	void* m_ptr;
	int m_int;
	MyLocalCaster()
		: m_ptr(0)
	{
	}
};

btBulletXmlWorldImporter::btBulletXmlWorldImporter(btDynamicsWorld* world)
	: btWorldImporter(world),
	  m_fileVersion(-1),
	  m_fileOk(false)
{
}

btBulletXmlWorldImporter::~btBulletXmlWorldImporter()
{
}

#if 0
static int get_double_attribute_by_name(const XMLElement* pElement, const char* attribName,double* value)
{
	if ( !pElement ) 
		return 0;

	const XMLAttribute* pAttrib=pElement->FirstAttribute();
	while (pAttrib)
	{
		if (pAttrib->Name()==attribName)
			if (pAttrib->QueryDoubleValue(value)==TIXML_SUCCESS)
				return 1;
		pAttrib=pAttrib->Next();
	}
	return 0;
}
#endif

static int get_int_attribute_by_name(const XMLElement* pElement, const char* attribName, int* value)
{
	if (!pElement)
		return 0;

	const XMLAttribute* pAttrib = pElement->FirstAttribute();
	while (pAttrib)
	{
		if (!strcmp(pAttrib->Name(), attribName))
			if (pAttrib->QueryIntValue(value) == XML_SUCCESS)
				return 1;
		//		if (pAttrib->QueryDoubleValue(&dval)==TIXML_SUCCESS) printf( " d=%1.1f", dval);
		pAttrib = pAttrib->Next();
	}
	return 0;
}

void stringToFloatArray(const std::string& string, btAlignedObjectArray<float>& floats)
{
	btAlignedObjectArray<std::string> pieces;

	bullet_utils::split(pieces, string, " ");
	for (int i = 0; i < pieces.size(); ++i)
	{
		btAssert(pieces[i] != "");
		floats.push_back((float)atof(pieces[i].c_str()));
	}
}

static btVector3FloatData TextToVector3Data(const char* txt)
{
	btAssert(txt);
	btAlignedObjectArray<float> floats;
	stringToFloatArray(txt, floats);
	btAssert(floats.size() == 4);

	btVector3FloatData vec4;
	vec4.m_floats[0] = floats[0];
	vec4.m_floats[1] = floats[1];
	vec4.m_floats[2] = floats[2];
	vec4.m_floats[3] = floats[3];
	return vec4;
}

void btBulletXmlWorldImporter::deSerializeVector3FloatData(XMLNode* pParent, btAlignedObjectArray<btVector3FloatData>& vectors)
{
	XMLNode* flNode = pParent->FirstChildElement("m_floats");
	btAssert(flNode);
	while (flNode && flNode->FirstChildElement())
	{
		XMLText* pText = flNode->FirstChildElement()->ToText();
		//		printf("value = %s\n",pText->Value());
		btVector3FloatData vec4 = TextToVector3Data(pText->Value());
		vectors.push_back(vec4);
		flNode = flNode->NextSibling();
	}
}

#define SET_INT_VALUE(xmlnode, targetdata, argname)                                                          \
	btAssert((xmlnode)->FirstChildElement(#argname) && (xmlnode)->FirstChildElement(#argname)->ToElement()); \
	if ((xmlnode)->FirstChildElement(#argname) && (xmlnode)->FirstChildElement(#argname)->ToElement())       \
		(targetdata)->argname = (int)atof(xmlnode->FirstChildElement(#argname)->ToElement()->GetText());

#define SET_FLOAT_VALUE(xmlnode, targetdata, argname)                                                        \
	btAssert((xmlnode)->FirstChildElement(#argname) && (xmlnode)->FirstChildElement(#argname)->ToElement()); \
	if ((xmlnode)->FirstChildElement(#argname) && (xmlnode)->FirstChildElement(#argname)->ToElement())       \
		(targetdata)->argname = (float)atof(xmlnode->FirstChildElement(#argname)->ToElement()->GetText());

#define SET_POINTER_VALUE(xmlnode, targetdata, argname, pointertype) \
	{                                                                \
		XMLNode* node = xmlnode->FirstChildElement(#argname);        \
		btAssert(node);                                              \
		if (node)                                                    \
		{                                                            \
			const char* txt = (node)->ToElement()->GetText();        \
			MyLocalCaster cast;                                      \
			cast.m_int = (int)atof(txt);                             \
			(targetdata).argname = (pointertype)cast.m_ptr;          \
		}                                                            \
	}

#define SET_VECTOR4_VALUE(xmlnode, targetdata, argname)                            \
	{                                                                              \
		XMLNode* flNode = xmlnode->FirstChildElement(#argname);                    \
		btAssert(flNode);                                                          \
		if (flNode && flNode->FirstChildElement())                                 \
		{                                                                          \
			const char* txt = flNode->FirstChildElement()->ToElement()->GetText(); \
			btVector3FloatData vec4 = TextToVector3Data(txt);                      \
			(targetdata)->argname.m_floats[0] = vec4.m_floats[0];                  \
			(targetdata)->argname.m_floats[1] = vec4.m_floats[1];                  \
			(targetdata)->argname.m_floats[2] = vec4.m_floats[2];                  \
			(targetdata)->argname.m_floats[3] = vec4.m_floats[3];                  \
		}                                                                          \
	}

#define SET_MATRIX33_VALUE(n, targetdata, argname)                                      \
	{                                                                                   \
		XMLNode* xmlnode = n->FirstChildElement(#argname);                              \
		btAssert(xmlnode);                                                              \
		if (xmlnode)                                                                    \
		{                                                                               \
			XMLNode* eleNode = xmlnode->FirstChildElement("m_el");                      \
			btAssert(eleNode);                                                          \
			if (eleNode && eleNode->FirstChildElement())                                \
			{                                                                           \
				const char* txt = eleNode->FirstChildElement()->ToElement()->GetText(); \
				btVector3FloatData vec4 = TextToVector3Data(txt);                       \
				(targetdata)->argname.m_el[0].m_floats[0] = vec4.m_floats[0];           \
				(targetdata)->argname.m_el[0].m_floats[1] = vec4.m_floats[1];           \
				(targetdata)->argname.m_el[0].m_floats[2] = vec4.m_floats[2];           \
				(targetdata)->argname.m_el[0].m_floats[3] = vec4.m_floats[3];           \
                                                                                        \
				XMLNode* n1 = eleNode->FirstChildElement()->NextSibling();              \
				btAssert(n1);                                                           \
				if (n1)                                                                 \
				{                                                                       \
					const char* txt = n1->ToElement()->GetText();                       \
					btVector3FloatData vec4 = TextToVector3Data(txt);                   \
					(targetdata)->argname.m_el[1].m_floats[0] = vec4.m_floats[0];       \
					(targetdata)->argname.m_el[1].m_floats[1] = vec4.m_floats[1];       \
					(targetdata)->argname.m_el[1].m_floats[2] = vec4.m_floats[2];       \
					(targetdata)->argname.m_el[1].m_floats[3] = vec4.m_floats[3];       \
                                                                                        \
					XMLNode* n2 = n1->NextSibling();                                    \
					btAssert(n2);                                                       \
					if (n2)                                                             \
					{                                                                   \
						const char* txt = n2->ToElement()->GetText();                   \
						btVector3FloatData vec4 = TextToVector3Data(txt);               \
						(targetdata)->argname.m_el[2].m_floats[0] = vec4.m_floats[0];   \
						(targetdata)->argname.m_el[2].m_floats[1] = vec4.m_floats[1];   \
						(targetdata)->argname.m_el[2].m_floats[2] = vec4.m_floats[2];   \
						(targetdata)->argname.m_el[2].m_floats[3] = vec4.m_floats[3];   \
					}                                                                   \
				}                                                                       \
			}                                                                           \
		}                                                                               \
	}

#define SET_TRANSFORM_VALUE(n, targetdata, argname)                     \
	{                                                                   \
		XMLNode* trNode = n->FirstChildElement(#argname);               \
		btAssert(trNode);                                               \
		if (trNode)                                                     \
		{                                                               \
			SET_VECTOR4_VALUE(trNode, &(targetdata)->argname, m_origin) \
			SET_MATRIX33_VALUE(trNode, &(targetdata)->argname, m_basis) \
		}                                                               \
	}

void btBulletXmlWorldImporter::deSerializeCollisionShapeData(XMLNode* pParent, btCollisionShapeData* colShapeData)
{
	SET_INT_VALUE(pParent, colShapeData, m_shapeType)
	colShapeData->m_name = 0;
}

void btBulletXmlWorldImporter::deSerializeConvexHullShapeData(XMLNode* pParent)
{
	MyLocalCaster cast;
	get_int_attribute_by_name(pParent->ToElement(), "pointer", &cast.m_int);

	btConvexHullShapeData* convexHullData = (btConvexHullShapeData*)btAlignedAlloc(sizeof(btConvexHullShapeData), 16);

	XMLNode* xmlConvexInt = pParent->FirstChildElement("m_convexInternalShapeData");
	btAssert(xmlConvexInt);

	XMLNode* xmlColShape = xmlConvexInt->FirstChildElement("m_collisionShapeData");
	btAssert(xmlColShape);

	deSerializeCollisionShapeData(xmlColShape, &convexHullData->m_convexInternalShapeData.m_collisionShapeData);

	SET_FLOAT_VALUE(xmlConvexInt, &convexHullData->m_convexInternalShapeData, m_collisionMargin)
	SET_VECTOR4_VALUE(xmlConvexInt, &convexHullData->m_convexInternalShapeData, m_localScaling)
	SET_VECTOR4_VALUE(xmlConvexInt, &convexHullData->m_convexInternalShapeData, m_implicitShapeDimensions)

	//convexHullData->m_unscaledPointsFloatPtr
	//#define SET_POINTER_VALUE(xmlnode, targetdata, argname, pointertype)

	{
		XMLNode* node = pParent->FirstChildElement("m_unscaledPointsFloatPtr");
		btAssert(node);
		if (node)
		{
			const char* txt = (node)->ToElement()->GetText();
			MyLocalCaster cast;
			cast.m_int = (int)atof(txt);
			(*convexHullData).m_unscaledPointsFloatPtr = (btVector3FloatData*)cast.m_ptr;
		}
	}

	SET_POINTER_VALUE(pParent, *convexHullData, m_unscaledPointsFloatPtr, btVector3FloatData*);
	SET_POINTER_VALUE(pParent, *convexHullData, m_unscaledPointsDoublePtr, btVector3DoubleData*);
	SET_INT_VALUE(pParent, convexHullData, m_numUnscaledPoints);

	m_collisionShapeData.push_back((btCollisionShapeData*)convexHullData);
	m_pointerLookup.insert(cast.m_ptr, convexHullData);
}

void btBulletXmlWorldImporter::deSerializeCompoundShapeChildData(XMLNode* pParent)
{
	MyLocalCaster cast;
	get_int_attribute_by_name(pParent->ToElement(), "pointer", &cast.m_int);

	int numChildren = 0;
	btAlignedObjectArray<btCompoundShapeChildData>* compoundChildArrayPtr = new btAlignedObjectArray<btCompoundShapeChildData>;
	{
		XMLNode* transNode = pParent->FirstChildElement("m_transform");
		XMLNode* colShapeNode = pParent->FirstChildElement("m_childShape");
		XMLNode* marginNode = pParent->FirstChildElement("m_childMargin");
		XMLNode* childTypeNode = pParent->FirstChildElement("m_childShapeType");

		int i = 0;
		while (transNode && colShapeNode && marginNode && childTypeNode)
		{
			compoundChildArrayPtr->expandNonInitializing();
			SET_VECTOR4_VALUE(transNode, &compoundChildArrayPtr->at(i).m_transform, m_origin)
			SET_MATRIX33_VALUE(transNode, &compoundChildArrayPtr->at(i).m_transform, m_basis)

			const char* txt = (colShapeNode)->ToElement()->GetText();
			MyLocalCaster cast;
			cast.m_int = (int)atof(txt);
			compoundChildArrayPtr->at(i).m_childShape = (btCollisionShapeData*)cast.m_ptr;

			btAssert(childTypeNode->ToElement());
			if (childTypeNode->ToElement())
			{
				compoundChildArrayPtr->at(i).m_childShapeType = (int)atof(childTypeNode->ToElement()->GetText());
			}

			btAssert(marginNode->ToElement());
			if (marginNode->ToElement())
			{
				compoundChildArrayPtr->at(i).m_childMargin = (float)atof(marginNode->ToElement()->GetText());
			}

			transNode = transNode->NextSiblingElement("m_transform");
			colShapeNode = colShapeNode->NextSiblingElement("m_childShape");
			marginNode = marginNode->NextSiblingElement("m_childMargin");
			childTypeNode = childTypeNode->NextSiblingElement("m_childShapeType");
			i++;
		}

		numChildren = i;
	}

	btAssert(numChildren);
	if (numChildren)
	{
		m_compoundShapeChildDataArrays.push_back(compoundChildArrayPtr);
		btCompoundShapeChildData* cd = &compoundChildArrayPtr->at(0);
		m_pointerLookup.insert(cast.m_ptr, cd);
	}
}

void btBulletXmlWorldImporter::deSerializeCompoundShapeData(XMLNode* pParent)
{
	MyLocalCaster cast;
	get_int_attribute_by_name(pParent->ToElement(), "pointer", &cast.m_int);

	btCompoundShapeData* compoundData = (btCompoundShapeData*)btAlignedAlloc(sizeof(btCompoundShapeData), 16);

	XMLNode* xmlColShape = pParent->FirstChildElement("m_collisionShapeData");
	btAssert(xmlColShape);
	deSerializeCollisionShapeData(xmlColShape, &compoundData->m_collisionShapeData);

	SET_INT_VALUE(pParent, compoundData, m_numChildShapes);

	XMLNode* xmlShapeData = pParent->FirstChildElement("m_collisionShapeData");
	btAssert(xmlShapeData);

	{
		XMLNode* node = pParent->FirstChildElement("m_childShapePtr");
		btAssert(node);
		while (node)
		{
			const char* txt = (node)->ToElement()->GetText();
			MyLocalCaster cast;
			cast.m_int = (int)atof(txt);
			compoundData->m_childShapePtr = (btCompoundShapeChildData*)cast.m_ptr;
			node = node->NextSiblingElement("m_childShapePtr");
		}
		//SET_POINTER_VALUE(xmlColShape, *compoundData,m_childShapePtr,btCompoundShapeChildData*);
	}
	SET_FLOAT_VALUE(pParent, compoundData, m_collisionMargin);

	m_collisionShapeData.push_back((btCollisionShapeData*)compoundData);
	m_pointerLookup.insert(cast.m_ptr, compoundData);
}

void btBulletXmlWorldImporter::deSerializeStaticPlaneShapeData(XMLNode* pParent)
{
	MyLocalCaster cast;
	get_int_attribute_by_name(pParent->ToElement(), "pointer", &cast.m_int);

	btStaticPlaneShapeData* planeData = (btStaticPlaneShapeData*)btAlignedAlloc(sizeof(btStaticPlaneShapeData), 16);

	XMLNode* xmlShapeData = pParent->FirstChildElement("m_collisionShapeData");
	btAssert(xmlShapeData);
	deSerializeCollisionShapeData(xmlShapeData, &planeData->m_collisionShapeData);

	SET_VECTOR4_VALUE(pParent, planeData, m_localScaling);
	SET_VECTOR4_VALUE(pParent, planeData, m_planeNormal);
	SET_FLOAT_VALUE(pParent, planeData, m_planeConstant);

	m_collisionShapeData.push_back((btCollisionShapeData*)planeData);
	m_pointerLookup.insert(cast.m_ptr, planeData);
}

void btBulletXmlWorldImporter::deSerializeDynamicsWorldData(XMLNode* pParent)
{
	btContactSolverInfo solverInfo;
	//btVector3 gravity(0,0,0);

	//setDynamicsWorldInfo(gravity,solverInfo);

	//gravity and world info
}

void btBulletXmlWorldImporter::deSerializeConvexInternalShapeData(XMLNode* pParent)
{
	MyLocalCaster cast;
	get_int_attribute_by_name(pParent->ToElement(), "pointer", &cast.m_int);

	btConvexInternalShapeData* convexShape = (btConvexInternalShapeData*)btAlignedAlloc(sizeof(btConvexInternalShapeData), 16);
	memset(convexShape, 0, sizeof(btConvexInternalShapeData));

	XMLNode* xmlShapeData = pParent->FirstChildElement("m_collisionShapeData");
	btAssert(xmlShapeData);

	deSerializeCollisionShapeData(xmlShapeData, &convexShape->m_collisionShapeData);

	SET_FLOAT_VALUE(pParent, convexShape, m_collisionMargin)
	SET_VECTOR4_VALUE(pParent, convexShape, m_localScaling)
	SET_VECTOR4_VALUE(pParent, convexShape, m_implicitShapeDimensions)

	m_collisionShapeData.push_back((btCollisionShapeData*)convexShape);
	m_pointerLookup.insert(cast.m_ptr, convexShape);
}

/*
enum btTypedConstraintType
{
	POINT2POINT_CONSTRAINT_TYPE=3,
	HINGE_CONSTRAINT_TYPE,
	CONETWIST_CONSTRAINT_TYPE,
//	D6_CONSTRAINT_TYPE,
	SLIDER_CONSTRAINT_TYPE,
	CONTACT_CONSTRAINT_TYPE,
	D6_SPRING_CONSTRAINT_TYPE,
	GEAR_CONSTRAINT_TYPE,
	MAX_CONSTRAINT_TYPE
};
*/

void btBulletXmlWorldImporter::deSerializeGeneric6DofConstraintData(XMLNode* pParent)
{
	MyLocalCaster cast;
	get_int_attribute_by_name(pParent->ToElement(), "pointer", &cast.m_int);

	btGeneric6DofConstraintData2* dof6Data = (btGeneric6DofConstraintData2*)btAlignedAlloc(sizeof(btGeneric6DofConstraintData2), 16);

	XMLNode* n = pParent->FirstChildElement("m_typeConstraintData");
	if (n)
	{
		SET_POINTER_VALUE(n, dof6Data->m_typeConstraintData, m_rbA, btRigidBodyData*);
		SET_POINTER_VALUE(n, dof6Data->m_typeConstraintData, m_rbB, btRigidBodyData*);
		dof6Data->m_typeConstraintData.m_name = 0;  //tbd
		SET_INT_VALUE(n, &dof6Data->m_typeConstraintData, m_objectType);
		SET_INT_VALUE(n, &dof6Data->m_typeConstraintData, m_userConstraintType);
		SET_INT_VALUE(n, &dof6Data->m_typeConstraintData, m_userConstraintId);
		SET_INT_VALUE(n, &dof6Data->m_typeConstraintData, m_needsFeedback);
		SET_FLOAT_VALUE(n, &dof6Data->m_typeConstraintData, m_appliedImpulse);
		SET_FLOAT_VALUE(n, &dof6Data->m_typeConstraintData, m_dbgDrawSize);
		SET_INT_VALUE(n, &dof6Data->m_typeConstraintData, m_disableCollisionsBetweenLinkedBodies);
		SET_INT_VALUE(n, &dof6Data->m_typeConstraintData, m_overrideNumSolverIterations);
		SET_FLOAT_VALUE(n, &dof6Data->m_typeConstraintData, m_breakingImpulseThreshold);
		SET_INT_VALUE(n, &dof6Data->m_typeConstraintData, m_isEnabled);
	}

	SET_TRANSFORM_VALUE(pParent, dof6Data, m_rbAFrame);
	SET_TRANSFORM_VALUE(pParent, dof6Data, m_rbBFrame);
	SET_VECTOR4_VALUE(pParent, dof6Data, m_linearUpperLimit);
	SET_VECTOR4_VALUE(pParent, dof6Data, m_linearLowerLimit);
	SET_VECTOR4_VALUE(pParent, dof6Data, m_angularUpperLimit);
	SET_VECTOR4_VALUE(pParent, dof6Data, m_angularLowerLimit);
	SET_INT_VALUE(pParent, dof6Data, m_useLinearReferenceFrameA);
	SET_INT_VALUE(pParent, dof6Data, m_useOffsetForConstraintFrame);

	m_constraintData.push_back((btTypedConstraintData2*)dof6Data);
	m_pointerLookup.insert(cast.m_ptr, dof6Data);
}

void btBulletXmlWorldImporter::deSerializeRigidBodyFloatData(XMLNode* pParent)
{
	MyLocalCaster cast;

	if (!get_int_attribute_by_name(pParent->ToElement(), "pointer", &cast.m_int))
	{
		m_fileOk = false;
		return;
	}

	btRigidBodyData* rbData = (btRigidBodyData*)btAlignedAlloc(sizeof(btRigidBodyData), 16);

	XMLNode* n = pParent->FirstChildElement("m_collisionObjectData");

	if (n)
	{
		SET_POINTER_VALUE(n, rbData->m_collisionObjectData, m_collisionShape, void*);
		SET_TRANSFORM_VALUE(n, &rbData->m_collisionObjectData, m_worldTransform);
		SET_TRANSFORM_VALUE(n, &rbData->m_collisionObjectData, m_interpolationWorldTransform);
		SET_VECTOR4_VALUE(n, &rbData->m_collisionObjectData, m_interpolationLinearVelocity)
		SET_VECTOR4_VALUE(n, &rbData->m_collisionObjectData, m_interpolationAngularVelocity)
		SET_VECTOR4_VALUE(n, &rbData->m_collisionObjectData, m_anisotropicFriction)
		SET_FLOAT_VALUE(n, &rbData->m_collisionObjectData, m_contactProcessingThreshold);
		SET_FLOAT_VALUE(n, &rbData->m_collisionObjectData, m_deactivationTime);
		SET_FLOAT_VALUE(n, &rbData->m_collisionObjectData, m_friction);
		SET_FLOAT_VALUE(n, &rbData->m_collisionObjectData, m_restitution);
		SET_FLOAT_VALUE(n, &rbData->m_collisionObjectData, m_hitFraction);
		SET_FLOAT_VALUE(n, &rbData->m_collisionObjectData, m_ccdSweptSphereRadius);
		SET_FLOAT_VALUE(n, &rbData->m_collisionObjectData, m_ccdMotionThreshold);
		SET_INT_VALUE(n, &rbData->m_collisionObjectData, m_hasAnisotropicFriction);
		SET_INT_VALUE(n, &rbData->m_collisionObjectData, m_collisionFlags);
		SET_INT_VALUE(n, &rbData->m_collisionObjectData, m_islandTag1);
		SET_INT_VALUE(n, &rbData->m_collisionObjectData, m_companionId);
		SET_INT_VALUE(n, &rbData->m_collisionObjectData, m_activationState1);
		SET_INT_VALUE(n, &rbData->m_collisionObjectData, m_internalType);
		SET_INT_VALUE(n, &rbData->m_collisionObjectData, m_checkCollideWith);
	}

	//	SET_VECTOR4_VALUE(pParent,rbData,m_linearVelocity);

	SET_MATRIX33_VALUE(pParent, rbData, m_invInertiaTensorWorld);

	SET_VECTOR4_VALUE(pParent, rbData, m_linearVelocity)
	SET_VECTOR4_VALUE(pParent, rbData, m_angularVelocity)
	SET_VECTOR4_VALUE(pParent, rbData, m_angularFactor)
	SET_VECTOR4_VALUE(pParent, rbData, m_linearFactor)
	SET_VECTOR4_VALUE(pParent, rbData, m_gravity)
	SET_VECTOR4_VALUE(pParent, rbData, m_gravity_acceleration)
	SET_VECTOR4_VALUE(pParent, rbData, m_invInertiaLocal)
	SET_VECTOR4_VALUE(pParent, rbData, m_totalTorque)
	SET_VECTOR4_VALUE(pParent, rbData, m_totalForce)
	SET_FLOAT_VALUE(pParent, rbData, m_inverseMass);
	SET_FLOAT_VALUE(pParent, rbData, m_linearDamping);
	SET_FLOAT_VALUE(pParent, rbData, m_angularDamping);
	SET_FLOAT_VALUE(pParent, rbData, m_additionalDampingFactor);
	SET_FLOAT_VALUE(pParent, rbData, m_additionalLinearDampingThresholdSqr);
	SET_FLOAT_VALUE(pParent, rbData, m_additionalAngularDampingThresholdSqr);
	SET_FLOAT_VALUE(pParent, rbData, m_additionalAngularDampingFactor);
	SET_FLOAT_VALUE(pParent, rbData, m_angularSleepingThreshold);
	SET_FLOAT_VALUE(pParent, rbData, m_linearSleepingThreshold);
	SET_INT_VALUE(pParent, rbData, m_additionalDamping);

	m_rigidBodyData.push_back(rbData);
	m_pointerLookup.insert(cast.m_ptr, rbData);

	//	rbData->m_collisionObjectData.m_collisionShape = (void*) (int)atof(txt);
}

/*
	TETRAHEDRAL_SHAPE_PROXYTYPE,
	CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE,
	,
	CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE,
	CUSTOM_POLYHEDRAL_SHAPE_TYPE,
//implicit convex shapes
IMPLICIT_CONVEX_SHAPES_START_HERE,
	SPHERE_SHAPE_PROXYTYPE,
	MULTI_SPHERE_SHAPE_PROXYTYPE,
	CAPSULE_SHAPE_PROXYTYPE,
	CONE_SHAPE_PROXYTYPE,
	CONVEX_SHAPE_PROXYTYPE,
	CYLINDER_SHAPE_PROXYTYPE,
	UNIFORM_SCALING_SHAPE_PROXYTYPE,
	MINKOWSKI_SUM_SHAPE_PROXYTYPE,
	MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE,
	BOX_2D_SHAPE_PROXYTYPE,
	CONVEX_2D_SHAPE_PROXYTYPE,
	CUSTOM_CONVEX_SHAPE_TYPE,
//concave shapes
CONCAVE_SHAPES_START_HERE,
	//keep all the convex shapetype below here, for the check IsConvexShape in broadphase proxy!
	TRIANGLE_MESH_SHAPE_PROXYTYPE,
	SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE,
	///used for demo integration FAST/Swift collision library and Bullet
	FAST_CONCAVE_MESH_PROXYTYPE,
	//terrain
	TERRAIN_SHAPE_PROXYTYPE,
///Used for GIMPACT Trimesh integration
	GIMPACT_SHAPE_PROXYTYPE,
///Multimaterial mesh
    MULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE,
	
	,
	,
	CUSTOM_CONCAVE_SHAPE_TYPE,
CONCAVE_SHAPES_END_HERE,

	,

	SOFTBODY_SHAPE_PROXYTYPE,
	HFFLUID_SHAPE_PROXYTYPE,
	HFFLUID_BUOYANT_CONVEX_SHAPE_PROXYTYPE,
	INVALID_SHAPE_PROXYTYPE,

	MAX_BROADPHASE_COLLISION_TYPES
*/

void btBulletXmlWorldImporter::fixupConstraintData(btTypedConstraintData2* tcd)
{
	if (tcd->m_rbA)
	{
		btRigidBodyData** ptrptr = (btRigidBodyData**)m_pointerLookup.find(tcd->m_rbA);
		btAssert(ptrptr);
		tcd->m_rbA = ptrptr ? *ptrptr : 0;
	}
	if (tcd->m_rbB)
	{
		btRigidBodyData** ptrptr = (btRigidBodyData**)m_pointerLookup.find(tcd->m_rbB);
		btAssert(ptrptr);
		tcd->m_rbB = ptrptr ? *ptrptr : 0;
	}
}

void btBulletXmlWorldImporter::fixupCollisionDataPointers(btCollisionShapeData* shapeData)
{
	switch (shapeData->m_shapeType)
	{
		case COMPOUND_SHAPE_PROXYTYPE:
		{
			btCompoundShapeData* compound = (btCompoundShapeData*)shapeData;

			void** cdptr = m_pointerLookup.find((void*)compound->m_childShapePtr);
			btCompoundShapeChildData** c = (btCompoundShapeChildData**)cdptr;
			btAssert(c);
			if (c)
			{
				compound->m_childShapePtr = *c;
			}
			else
			{
				compound->m_childShapePtr = 0;
			}
			break;
		}

		case CONVEX_HULL_SHAPE_PROXYTYPE:
		{
			btConvexHullShapeData* convexData = (btConvexHullShapeData*)shapeData;
			btVector3FloatData** ptrptr = (btVector3FloatData**)m_pointerLookup.find((void*)convexData->m_unscaledPointsFloatPtr);
			btAssert(ptrptr);
			if (ptrptr)
			{
				convexData->m_unscaledPointsFloatPtr = *ptrptr;
			}
			else
			{
				convexData->m_unscaledPointsFloatPtr = 0;
			}
			break;
		}

		case BOX_SHAPE_PROXYTYPE:
		case TRIANGLE_SHAPE_PROXYTYPE:
		case STATIC_PLANE_PROXYTYPE:
		case EMPTY_SHAPE_PROXYTYPE:
			break;

		default:
		{
			btAssert(0);
		}
	}
}

void btBulletXmlWorldImporter::auto_serialize_root_level_children(XMLNode* pParent)
{
	int numChildren = 0;
	btAssert(pParent);
	if (pParent)
	{
		XMLNode* pChild;
		for (pChild = pParent->FirstChildElement(); pChild != 0; pChild = pChild->NextSibling(), numChildren++)
		{
			//			printf("child Name=%s\n", pChild->Value());
			if (!strcmp(pChild->Value(), "btVector3FloatData"))
			{
				MyLocalCaster cast;
				get_int_attribute_by_name(pChild->ToElement(), "pointer", &cast.m_int);

				btAlignedObjectArray<btVector3FloatData> v;
				deSerializeVector3FloatData(pChild, v);
				int numVectors = v.size();
				btVector3FloatData* vectors = (btVector3FloatData*)btAlignedAlloc(sizeof(btVector3FloatData) * numVectors, 16);
				for (int i = 0; i < numVectors; i++)
					vectors[i] = v[i];
				m_floatVertexArrays.push_back(vectors);
				m_pointerLookup.insert(cast.m_ptr, vectors);
				continue;
			}

			if (!strcmp(pChild->Value(), "btGeneric6DofConstraintData"))
			{
				deSerializeGeneric6DofConstraintData(pChild);
				continue;
			}

			if (!strcmp(pChild->Value(), "btStaticPlaneShapeData"))
			{
				deSerializeStaticPlaneShapeData(pChild);
				continue;
			}

			if (!strcmp(pChild->Value(), "btCompoundShapeData"))
			{
				deSerializeCompoundShapeData(pChild);
				continue;
			}

			if (!strcmp(pChild->Value(), "btCompoundShapeChildData"))
			{
				deSerializeCompoundShapeChildData(pChild);
				continue;
			}

			if (!strcmp(pChild->Value(), "btConvexHullShapeData"))
			{
				deSerializeConvexHullShapeData(pChild);
				continue;
			}

			if (!strcmp(pChild->Value(), "btDynamicsWorldFloatData"))
			{
				deSerializeDynamicsWorldData(pChild);
				continue;
			}

			if (!strcmp(pChild->Value(), "btConvexInternalShapeData"))
			{
				deSerializeConvexInternalShapeData(pChild);
				continue;
			}
			if (!strcmp(pChild->Value(), "btRigidBodyFloatData"))
			{
				deSerializeRigidBodyFloatData(pChild);
				continue;
			}

			//printf("Error: btBulletXmlWorldImporter doesn't support %s yet\n", pChild->Value());
			//	btAssert(0);
		}
	}

	///=================================================================
	///fixup pointers in various places, in the right order

	//fixup compoundshape child data
	for (int i = 0; i < m_compoundShapeChildDataArrays.size(); i++)
	{
		btAlignedObjectArray<btCompoundShapeChildData>* childDataArray = m_compoundShapeChildDataArrays[i];
		for (int c = 0; c < childDataArray->size(); c++)
		{
			btCompoundShapeChildData* childData = &childDataArray->at(c);
			btCollisionShapeData** ptrptr = (btCollisionShapeData**)m_pointerLookup[childData->m_childShape];
			btAssert(ptrptr);
			if (ptrptr)
			{
				childData->m_childShape = *ptrptr;
			}
		}
	}

	for (int i = 0; i < this->m_collisionShapeData.size(); i++)
	{
		btCollisionShapeData* shapeData = m_collisionShapeData[i];
		fixupCollisionDataPointers(shapeData);
	}

	///now fixup pointers
	for (int i = 0; i < m_rigidBodyData.size(); i++)
	{
		btRigidBodyData* rbData = m_rigidBodyData[i];

		void** ptrptr = m_pointerLookup.find(rbData->m_collisionObjectData.m_collisionShape);
		//btAssert(ptrptr);
		rbData->m_collisionObjectData.m_broadphaseHandle = 0;
		rbData->m_collisionObjectData.m_rootCollisionShape = 0;
		rbData->m_collisionObjectData.m_name = 0;  //tbd
		if (ptrptr)
		{
			rbData->m_collisionObjectData.m_collisionShape = *ptrptr;
		}
	}

	for (int i = 0; i < m_constraintData.size(); i++)
	{
		btTypedConstraintData2* tcd = m_constraintData[i];
		fixupConstraintData(tcd);
	}
	///=================================================================
	///convert data into Bullet data in the right order

	///convert collision shapes
	for (int i = 0; i < this->m_collisionShapeData.size(); i++)
	{
		btCollisionShapeData* shapeData = m_collisionShapeData[i];
		btCollisionShape* shape = convertCollisionShape(shapeData);
		if (shape)
		{
			m_shapeMap.insert(shapeData, shape);
		}
		if (shape && shapeData->m_name)
		{
			char* newname = duplicateName(shapeData->m_name);
			m_objectNameMap.insert(shape, newname);
			m_nameShapeMap.insert(newname, shape);
		}
	}

	for (int i = 0; i < m_rigidBodyData.size(); i++)
	{
#ifdef BT_USE_DOUBLE_PRECISION
		convertRigidBodyDouble(m_rigidBodyData[i]);
#else
		convertRigidBodyFloat(m_rigidBodyData[i]);
#endif
	}

	for (int i = 0; i < m_constraintData.size(); i++)
	{
		btTypedConstraintData2* tcd = m_constraintData[i];
		//		bool isDoublePrecision = false;
		btRigidBody* rbA = 0;
		btRigidBody* rbB = 0;
		{
			btCollisionObject** ptrptr = m_bodyMap.find(tcd->m_rbA);
			if (ptrptr)
			{
				rbA = btRigidBody::upcast(*ptrptr);
			}
		}
		{
			btCollisionObject** ptrptr = m_bodyMap.find(tcd->m_rbB);
			if (ptrptr)
			{
				rbB = btRigidBody::upcast(*ptrptr);
			}
		}
		if (rbA || rbB)
		{
			btAssert(0);  //todo
						  //convertConstraint(tcd,rbA,rbB,isDoublePrecision, m_fileVersion);
		}
	}
}

void btBulletXmlWorldImporter::auto_serialize(XMLNode* pParent)
{
	//	XMLElement* root = pParent->FirstChildElement("bullet_physics");
	if (pParent)
	{
		XMLNode* pChild;
		for (pChild = pParent->FirstChildElement(); pChild != 0; pChild = pChild->NextSibling())
		{
			//if (pChild->Type()==XMLNode::TINYXML_ELEMENT)
			{
				//				printf("root Name=%s\n", pChild->Value());
				auto_serialize_root_level_children(pChild);
			}
		}
	}
	else
	{
		printf("ERROR: no bullet_physics element\n");
	}
}

bool btBulletXmlWorldImporter::loadFile(const char* fileName)
{
	XMLDocument doc;

	XMLError loadOkay = doc.LoadFile(fileName);

	if (loadOkay == XML_SUCCESS)
	{
		if (get_int_attribute_by_name(doc.FirstChildElement()->ToElement(), "version", &m_fileVersion))
		{
			if (m_fileVersion == 281)
			{
				m_fileOk = true;
				int itemcount;
				get_int_attribute_by_name(doc.FirstChildElement()->ToElement(), "itemcount", &itemcount);

				auto_serialize(&doc);
				return m_fileOk;
			}
		}
	}
	return false;
}
