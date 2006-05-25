/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDAnimated.h"
#include "FCDocument/FCDGeometrySource.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
#include "FUtils/FUDaeEnum.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDGeometrySource::FCDGeometrySource(FCDocument* document) : FCDObjectWithId(document, "GeometrySource")
{
	sourceNode = NULL;
	sourceStride = 0;
	sourceType = FUDaeGeometryInput::UNKNOWN;
}

FCDGeometrySource::~FCDGeometrySource()
{
	animatedValues.clear();
}

void FCDGeometrySource::SetSourceData(const FloatList& _sourceData, uint32 _sourceStride, size_t offset, size_t count)
{
	// Remove all the data currently held by the source.
	sourceData.clear();
	sourceStride = _sourceStride;

	// Check the given bounds
	size_t beg = min(offset, _sourceData.size()), end;
	if (count == 0) end = _sourceData.size();
	else end = min(count + offset, _sourceData.size());
	sourceData.insert(sourceData.begin(), _sourceData.begin() + beg, _sourceData.begin() + end);
}

void FCDGeometrySource::SetSourceType(FUDaeGeometryInput::Semantic type)
{
	sourceType = type;
	animatedValues.clear();

	// Most types should remain un-animated
	if (sourceType != FUDaeGeometryInput::POSITION && sourceType != FUDaeGeometryInput::COLOR) return;

	// Look for an animation on this source's objects
	Int32List animatedIndices;
	GetDocument()->FindAnimationChannelsArrayIndices(sourceNode, animatedIndices);
	for (Int32List::iterator itA = animatedIndices.begin(); itA != animatedIndices.end(); ++itA)
	{
		// Check for repeated animated indices
		Int32List::iterator itB = animatedIndices.begin();
		for (; itB != itA && (*itA) != (*itB); ++itB) {}
		if (itB != itA) continue;

		FCDAnimated* animated = NULL;
		if (sourceType == FUDaeGeometryInput::POSITION)
		{
			animated = FCDAnimatedPoint3::Create(GetDocument(), sourceNode, (FMVector3*)&(sourceData[(*itA) * sourceStride]), *itA);
		}
		else
		{
			animated = FCDAnimatedColor::Create(GetDocument(), sourceNode, (FMVector3*)&(sourceData[(*itA) * sourceStride]), *itA);
		}

		// Keep track of these animated values
		if (animated != NULL) animatedValues.push_back(animated);
	}
}

// Clone this data source
FCDGeometrySource* FCDGeometrySource::Clone() const
{
	FCDGeometrySource* clone = new FCDGeometrySource(GetDocument());
	FCDObjectWithId::Clone(clone);
	clone->name = name;
	clone->sourceData = sourceData;
	clone->sourceNode = sourceNode;
	clone->sourceStride = sourceStride;
	clone->SetSourceType(sourceType);
	return clone;
}


// Read in the <source> node of the COLLADA document
FUStatus FCDGeometrySource::LoadFromXML(xmlNode* _sourceNode)
{
	FUStatus status;
	sourceNode = _sourceNode;

	// Read in the name and id of the source
	name = TO_FSTRING(ReadNodeName(sourceNode));
	string id = ReadNodeId(sourceNode);
	if (id.empty())
	{
		status.Warning(FS("Geometry source with no 'id' is unusable."), sourceNode->line);
	}
	SetDaeId(id);
	if (!id.empty() && GetDaeId() != id)
	{
		return status.Fail(FS("Geometry source has duplicate 'id': ") + TO_FSTRING(id), sourceNode->line);
	}

	// Read in the source data
	sourceStride = ReadSource(sourceNode, sourceData);
	if (sourceStride == 0)
	{
		status.Warning(FS("Geometry has source with no data."), sourceNode->line);
	}

	return status;
}

// Write out the <source> node to the COLLADA xml tree
xmlNode* FCDGeometrySource::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* sourceNode = NULL;

	// Export the source directly, using the correct parameters and the length factor
	switch (sourceType)
	{
	case FUDaeGeometryInput::POSITION: sourceNode = AddSourceFloat(parentNode, GetDaeId(), sourceData, sourceStride, FUDaeAccessor::XYZW, GetDocument()->GetLengthUnitConversion()); break;
	case FUDaeGeometryInput::NORMAL: sourceNode = AddSourceFloat(parentNode, GetDaeId(), sourceData, sourceStride, FUDaeAccessor::XYZW); break;
	case FUDaeGeometryInput::GEOTANGENT: sourceNode = AddSourceFloat(parentNode, GetDaeId(), sourceData, sourceStride, FUDaeAccessor::XYZW); break;
	case FUDaeGeometryInput::GEOBINORMAL: sourceNode = AddSourceFloat(parentNode, GetDaeId(), sourceData, sourceStride, FUDaeAccessor::XYZW); break;
	case FUDaeGeometryInput::TEXCOORD: sourceNode = AddSourceFloat(parentNode, GetDaeId(), sourceData, sourceStride, FUDaeAccessor::STPQ); break;
	case FUDaeGeometryInput::TEXTANGENT: sourceNode = AddSourceFloat(parentNode, GetDaeId(), sourceData, sourceStride, FUDaeAccessor::XYZW); break;
	case FUDaeGeometryInput::TEXBINORMAL: sourceNode = AddSourceFloat(parentNode, GetDaeId(), sourceData, sourceStride, FUDaeAccessor::XYZW); break;
	case FUDaeGeometryInput::UV: sourceNode = AddSourceFloat(parentNode, GetDaeId(), sourceData, sourceStride, FUDaeAccessor::XYZW); break;
	case FUDaeGeometryInput::COLOR: sourceNode = AddSourceFloat(parentNode, GetDaeId(), sourceData, sourceStride, FUDaeAccessor::RGBA); break;
	case FUDaeGeometryInput::EXTRA: sourceNode = AddSourceFloat(parentNode, GetDaeId(), sourceData, sourceStride, NULL); break;
	case FUDaeGeometryInput::UNKNOWN: sourceNode = AddSourceFloat(parentNode, GetDaeId(), sourceData, sourceStride, NULL); break;

	case FUDaeGeometryInput::VERTEX: // Refuse to export these sources
	default: break;
	}

	if (!name.empty())
	{
		AddAttribute(sourceNode, DAE_NAME_ATTRIBUTE, name);
	}

	return sourceNode;
}
