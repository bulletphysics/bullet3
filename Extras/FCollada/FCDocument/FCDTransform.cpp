/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/
/*
	Based on the FS Import classes:
	Copyright (C) 2005-2006 Feeling Software Inc
	Copyright (C) 2005-2006 Autodesk Media Entertainment
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDAnimated.h"
#include "FCDocument/FCDTransform.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
#include "FUtils/FUStringConversion.h"

#ifdef HAS_MAYATYPES
#include <maya/MTransformationMatrix.h>
#endif

FCDTTranslation::FCDTTranslation(FCDocument* document, FCDSceneNode* parent) : FCDTransform(document, parent) {}
FCDTTranslation::~FCDTTranslation() {}

FCDTransform* FCDTTranslation::Clone(FCDSceneNode* newParent)
{
	FCDTTranslation* t = new FCDTTranslation(GetDocument(), newParent);
	t->translation = translation;
	if(IsAnimated())
		GetAnimated()->Clone(GetDocument());
	return t;
}

// Parse a <translate> node from the COLLADA document
FUStatus FCDTTranslation::LoadFromXML(xmlNode* node)
{
	float lengthFactor = GetDocument()->GetLengthUnitConversion();

	const char* content = FUDaeParser::ReadNodeContentDirect(node);
	FloatList factors;
	factors.reserve(3);
	FUStringConversion::ToFloatList(content, factors);
	if (factors.size() != 3) return FUStatus(false);
	
	translation.x = factors[0] * lengthFactor;
	translation.y = factors[1] * lengthFactor;
	translation.z = factors[2] * lengthFactor;
	FCDAnimatedPoint3::Create(GetDocument(), node, &translation);
	return FUStatus(true);
}

// Write the <translate> node to the COLLADA xml document
xmlNode* FCDTTranslation::WriteToXML(xmlNode* parentNode) const
{
	string content = FUStringConversion::ToString(translation);
	xmlNode* transformNode = FUDaeWriter::AddChild(parentNode, DAE_TRANSLATE_ELEMENT, content);
	GetDocument()->WriteAnimatedValueToXML(&translation.x, transformNode, "translation");
	return transformNode;
}

FMMatrix44 FCDTTranslation::ToMatrix() const
{
	return FMMatrix44::TranslationMatrix(translation);
}

FCDAnimated* FCDTTranslation::GetAnimated()
{
	// Returns a FCDAnimatedAngleAxis
	return GetDocument()->FindAnimatedValue(&translation.x);
}

bool FCDTTranslation::IsAnimated() const
{
	// Only need to check against the first value, because we created a FCDAnimatedPoint3
	return GetDocument()->IsValueAnimated(&translation.x);
}

bool FCDTTranslation::IsInverse(const FCDTransform* transform) const
{
	return transform->GetType() == FCDTransform::TRANSLATION
		&& IsEquivalent(-1.0f * translation, ((const FCDTTranslation*)transform)->translation);
}

FCDTRotation::FCDTRotation(FCDocument* document, FCDSceneNode* parent) : FCDTransform(document, parent) {}
FCDTRotation::~FCDTRotation() {}

FCDTransform* FCDTRotation::Clone(FCDSceneNode* newParent)
{
	FCDTRotation* t = new FCDTRotation(GetDocument(), newParent);
	t->angle = angle;
	t->axis = axis;
	if(IsAnimated())
		GetAnimated()->Clone(GetDocument());
	return t;
}
// Parse a <rotate> node from the COLLADA document
FUStatus FCDTRotation::LoadFromXML(xmlNode* node)
{
	const char* content = FUDaeParser::ReadNodeContentDirect(node);
	FloatList factors;
	factors.reserve(4);
	FUStringConversion::ToFloatList(content, factors);
	if (factors.size() != 4) return FUStatus(false);

	axis.x = factors[0]; axis.y = factors[1]; axis.z = factors[2];
	angle = factors[3];
	FCDAnimatedAngleAxis::Create(GetDocument(), node, &axis, &angle);
	return FUStatus(true);
}

// Write the <rotate> node to the COLLADA xml document
xmlNode* FCDTRotation::WriteToXML(xmlNode* parent) const
{
	globalSBuilder.clear();
	FUStringConversion::ToString(globalSBuilder, axis); globalSBuilder += ' '; globalSBuilder += angle;
	xmlNode* transformNode = FUDaeWriter::AddChild(parent, DAE_ROTATE_ELEMENT, globalSBuilder);
	GetDocument()->WriteAnimatedValueToXML(&axis.x, transformNode, "rotation");
	return transformNode;
}

FMMatrix44 FCDTRotation::ToMatrix() const
{
	return FMMatrix44::AxisRotationMatrix(axis, FMath::DegToRad(angle));
}

FCDAnimated* FCDTRotation::GetAnimated()
{
	// Returns a FCDAnimatedAngleAxis
	return GetDocument()->FindAnimatedValue(&axis.x);
}

bool FCDTRotation::IsAnimated() const
{
	// For the axis, we only need to check against the first value, because we created a FCDAnimatedAngleAxis
	return GetDocument()->IsValueAnimated(&axis.x);
}

bool FCDTRotation::IsInverse(const FCDTransform* transform) const
{
	return transform->GetType() == FCDTransform::TRANSLATION
		&& IsEquivalent(axis, ((const FCDTRotation*)transform)->axis)
		&& IsEquivalent(-angle, ((const FCDTRotation*)transform)->angle);
}

FCDTScale::FCDTScale(FCDocument* document, FCDSceneNode* parent) : FCDTransform(document, parent) {}
FCDTScale::~FCDTScale() {}

FCDTransform* FCDTScale::Clone(FCDSceneNode* newParent)
{
	FCDTScale* t = new FCDTScale(GetDocument(), newParent);
	t->scale = scale;
	if(IsAnimated())
		GetAnimated()->Clone(GetDocument());
	return t;
}

// Parse a <scale> node from the COLLADA document
FUStatus FCDTScale::LoadFromXML(xmlNode* node)
{
	const char* content = FUDaeParser::ReadNodeContentDirect(node);
	FloatList factors;
	factors.reserve(3);
	FUStringConversion::ToFloatList(content, factors);
	if (factors.size() != 3) return FUStatus(false);

	scale.x = factors[0]; scale.y = factors[1]; scale.z = factors[2];

	// Register the animated values
	FCDAnimatedPoint3::Create(GetDocument(), node, &scale);

	return FUStatus(true);
}

// Write the <scale> node to the COLLADA xml document
xmlNode* FCDTScale::WriteToXML(xmlNode* parentNode) const
{
	string content = FUStringConversion::ToString(scale);
	xmlNode* transformNode = FUDaeWriter::AddChild(parentNode, DAE_SCALE_ELEMENT, content);
	GetDocument()->WriteAnimatedValueToXML(&scale.x, transformNode, "scale");
	return transformNode;
}

FMMatrix44 FCDTScale::ToMatrix() const
{
	FMMatrix44 mx = FMMatrix44::Identity;
	mx[0][0] = scale.x; mx[1][1] = scale.y; mx[2][2] = scale.z;
	return mx;
}

FCDAnimated* FCDTScale::GetAnimated()
{
	// Returns a FCDAnimatedPoint3
	return GetDocument()->FindAnimatedValue(&scale.x);
}

bool FCDTScale::IsAnimated() const
{
	// We only need to check against the first value, because we created a FCDAnimatedPoint3
	return GetDocument()->IsValueAnimated(&scale.x);
}

FCDTMatrix::FCDTMatrix(FCDocument* document, FCDSceneNode* parent) : FCDTransform(document, parent) {}
FCDTMatrix::~FCDTMatrix() {}

FCDTransform* FCDTMatrix::Clone(FCDSceneNode* newParent)
{
	FCDTMatrix* t = new FCDTMatrix(GetDocument(), newParent);
	t->transform = transform;
	if(IsAnimated())
		GetAnimated()->Clone(GetDocument());
	return t;
}
// Parse a <matrix> node from the COLLADA document
FUStatus FCDTMatrix::LoadFromXML(xmlNode* node)
{
	const char* content = FUDaeParser::ReadNodeContentDirect(node);
	FUStringConversion::ToMatrix(&content, transform, GetDocument()->GetLengthUnitConversion());

	// Register the matrix in the animation system for transform animations
	FCDAnimatedMatrix::Create(GetDocument(), node, &transform);

	return FUStatus(true);
}

// Write the <matrix> node to the COLLADA xml document
xmlNode* FCDTMatrix::WriteToXML(xmlNode* parentNode) const
{
	string content = FUStringConversion::ToString(transform);
	xmlNode* transformNode = FUDaeWriter::AddChild(parentNode, DAE_MATRIX_ELEMENT, content);
	GetDocument()->WriteAnimatedValueToXML(&transform[0][0], transformNode, "transform");
	return transformNode;
}

FCDAnimated* FCDTMatrix::GetAnimated()
{
	// Returns a FCDAnimatedMatrix
	return GetDocument()->FindAnimatedValue(&transform[0][0]);
}

bool FCDTMatrix::IsAnimated() const
{
	// We only need to check against the first value, because we created a FCDAnimatedMatrix
	return GetDocument()->IsValueAnimated(&transform[0][0]);
}

FCDTLookAt::FCDTLookAt(FCDocument* document, FCDSceneNode* parent) : FCDTransform(document, parent) {}
FCDTLookAt::~FCDTLookAt() {}

FCDTransform* FCDTLookAt::Clone(FCDSceneNode* newParent)
{
	FCDTLookAt* t = new FCDTLookAt(GetDocument(), newParent);
	t->position = position;
	t->target = target;
	t->up = up;
	return t;
}
// Parse a <lookat> node from the COLLADA document
FUStatus FCDTLookAt::LoadFromXML(xmlNode* lookAtNode)
{
	float lengthFactor = GetDocument()->GetLengthUnitConversion();
	const char* content = FUDaeParser::ReadNodeContentDirect(lookAtNode);
	FloatList factors;
	factors.reserve(9);
	FUStringConversion::ToFloatList(content, factors);
	if (factors.size() != 9) return FUStatus(false);

	position.x = factors[0] * lengthFactor; position.y = factors[1] * lengthFactor; position.z = factors[2] * lengthFactor;
	target.x = factors[3] * lengthFactor; target.y = factors[4] * lengthFactor; target.z = factors[5] * lengthFactor;
	up.x = factors[6]; up.y = factors[7]; up.z = factors[8];

	return FUStatus(true);
}

// Write the <lookat> node to the COLLADA xml document
xmlNode* FCDTLookAt::WriteToXML(xmlNode* parentNode) const
{
	globalSBuilder.clear();
	FUStringConversion::ToString(globalSBuilder, position); globalSBuilder += ' ';
	FUStringConversion::ToString(globalSBuilder, target); globalSBuilder += ' ';
	FUStringConversion::ToString(globalSBuilder, up);
	xmlNode* transformNode = FUDaeWriter::AddChild(parentNode, DAE_LOOKAT_ELEMENT, globalSBuilder);
	return transformNode;
}

FMMatrix44 FCDTLookAt::ToMatrix() const
{
	FMMatrix44 mx = FMMatrix44::Identity;
	FMVector3 front = ((FMVector3)(position - target)).Normalize();
	FMVector3 side = ((FMVector3)(up ^ front)).Normalize();

	mx[0][0] = side.x;  mx[0][1] = side.y;  mx[0][2] = side.z; 
	mx[1][0] = up.x;    mx[1][1] = up.y;    mx[1][2] = up.z; 
	mx[2][0] = front.x; mx[2][1] = front.y; mx[2][2] = front.z; 
	mx[3][0] = position.x;   mx[3][1] = position.y;   mx[3][2] = position.z;

	return mx;
}

FCDAnimated* FCDTLookAt::GetAnimated()
{
	// Does not support animations
	return NULL;
}

bool FCDTLookAt::IsAnimated() const
{
	// Does not support animations
	return false;
}

FCDTSkew::FCDTSkew(FCDocument* document, FCDSceneNode* parent) : FCDTransform(document, parent) {}
FCDTSkew::~FCDTSkew() {}
FCDTransform* FCDTSkew::Clone(FCDSceneNode* newParent)
{
	FCDTSkew* t = new FCDTSkew(GetDocument(), newParent);
	t->rotateAxis = rotateAxis;
	t->aroundAxis = aroundAxis;
	t->angle = angle;
	return t;
}

// Parse a <skew> element from the COLLADA document
FUStatus FCDTSkew::LoadFromXML(xmlNode* skewNode)
{
	const char* content = FUDaeParser::ReadNodeContentDirect(skewNode);
	FloatList factors;
	factors.reserve(7);
	FUStringConversion::ToFloatList(content, factors);
	if (factors.size() != 7) return FUStatus(false);

	angle = factors[0];
	rotateAxis.x = factors[1]; rotateAxis.y = factors[2]; rotateAxis.z = factors[3];
	aroundAxis.x = factors[4]; aroundAxis.y = factors[5]; aroundAxis.z = factors[6];

	// Check and pre-process the axises
	if (IsEquivalent(rotateAxis, FMVector3::Origin) || IsEquivalent(aroundAxis, FMVector3::Origin)) return FUStatus(false);
	rotateAxis = rotateAxis.Normalize();
	aroundAxis = aroundAxis.Normalize();

	// Register the animated values
	FCDAnimatedAngle::Create(GetDocument(), skewNode, &angle);

	return FUStatus(true);
}

// Write the <lookat> node to the COLLADA xml document
xmlNode* FCDTSkew::WriteToXML(xmlNode* parentNode) const
{
	globalSBuilder.clear();
	globalSBuilder.set(angle); globalSBuilder += ' ';
	FUStringConversion::ToString(globalSBuilder, rotateAxis); globalSBuilder += ' ';
	FUStringConversion::ToString(globalSBuilder, aroundAxis);
	xmlNode* transformNode = FUDaeWriter::AddChild(parentNode, DAE_SKEW_ELEMENT, globalSBuilder);
	GetDocument()->WriteAnimatedValueToXML(&angle, transformNode, "skew");
	return transformNode;
}

FMMatrix44 FCDTSkew::ToMatrix() const
{
	float v[4][4];

	float s = tanf(FMath::DegToRad(angle));

	for (int row = 0; row < 3; ++row)
	{
		for (int col = 0; col < 3; ++col)
		{
			v[col][row] = ((row == col) ? 1.0f : 0.0f) + s * rotateAxis[col] * aroundAxis[row];
		}
	}

	v[0][3] = v[1][3] = v[2][3] = 0.0f;
	v[3][0] = v[3][1] = v[3][2] = 0.0f;
	v[3][3] = 1.0f;

	return FMMatrix44((float*) v);
}

FCDAnimated* FCDTSkew::GetAnimated()
{
	return GetDocument()->FindAnimatedValue(&angle);
}

bool FCDTSkew::IsAnimated() const
{
	// Only angle may be animated for now.
	return GetDocument()->IsValueAnimated(&angle);
}

// Creates a new COLLADA transform, given a transform type.
FCDTransform* FCDTFactory::CreateTransform(FCDocument* document, FCDSceneNode* parent, FCDTransform::Type type)
{
	switch (type)
	{
	case FCDTransform::TRANSLATION: return new FCDTTranslation(document, parent);
	case FCDTransform::ROTATION: return new FCDTRotation(document, parent);
	case FCDTransform::SCALE: return new FCDTScale(document, parent);
	case FCDTransform::SKEW: return new FCDTSkew(document, parent);
	case FCDTransform::MATRIX: return new FCDTMatrix(document, parent);
	case FCDTransform::LOOKAT: return new FCDTLookAt(document, parent);
	default: return NULL;
	}
}

// Imports a COLLADA transform, given an XML tree node.
FCDTransform* FCDTFactory::CreateTransform(FCDocument* document, FCDSceneNode* parent, xmlNode* node)
{
	FCDTransform* transform = NULL;
	if (IsEquivalent(node->name, DAE_ROTATE_ELEMENT)) transform = new FCDTRotation(document, parent);
	else if (IsEquivalent(node->name, DAE_TRANSLATE_ELEMENT)) transform = new FCDTTranslation(document, parent);
	else if (IsEquivalent(node->name, DAE_SCALE_ELEMENT)) transform = new FCDTScale(document, parent);
	else if (IsEquivalent(node->name, DAE_SKEW_ELEMENT)) transform = new FCDTSkew(document, parent);
	else if (IsEquivalent(node->name, DAE_MATRIX_ELEMENT)) transform = new FCDTMatrix(document, parent);
	else if (IsEquivalent(node->name, DAE_LOOKAT_ELEMENT)) transform = new FCDTLookAt(document, parent);
	return transform;
}
