/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDGeometry.h"
#include "FCDocument/FCDGeometrySource.h"
#include "FCDocument/FCDGeometrySpline.h"
#include "FUtils/FUStringConversion.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDGeometrySpline::FCDGeometrySpline(FCDocument* document, FCDGeometry* _parent) : FCDObject(document, "FCDGeometrySpline")
{
	parent = _parent;
	isClosed = false;
}

FCDGeometrySpline::~FCDGeometrySpline()
{
	parent = NULL;
	cvs.clear();
	knots.clear();
}

// Read in the <spline> node of the COLLADA document
FUStatus FCDGeometrySpline::LoadFromXML(xmlNode* splineNode)
{
	FUStatus status;

	// Read the curve properties
	isClosed = FUStringConversion::ToBoolean(ReadNodeProperty(splineNode, DAE_CLOSED_ATTRIBUTE));

	// Read in the <control_vertices> element, which define the base type for this curve
	xmlNode* cvsNode = FindChildByType(splineNode, DAE_CONTROL_VERTICES_ELEMENT);
	if (cvsNode == NULL)
	{
		return status.Fail(FS("No <control_vertices> element in spline: ") + TO_FSTRING(parent->GetDaeId()), splineNode->line);
	}
	
	// Read in the per-vertex inputs
	string positionSrc;
	string sknots;
	int hasPositions = 0;
	int hasKnots = 0;

	xmlNodeList vertexInputNodes;
	FindChildrenByType(cvsNode, DAE_INPUT_ELEMENT, vertexInputNodes);
	for (xmlNodeList::iterator it = vertexInputNodes.begin(); it < vertexInputNodes.end(); ++it)
	{
		xmlNode* vertexInputNode = *it;
		string inputSemantic = ReadNodeSemantic(vertexInputNode);
		
		if( strcmp(inputSemantic.c_str(), "POSITION" ) == 0 )
		{
			positionSrc = ReadNodeProperty(vertexInputNode,"source");
			hasPositions = 1;
		}
		else if( strcmp(inputSemantic.c_str(), "KNOTSEQUENCES" ) == 0 )
		{
			sknots = ReadNodeProperty(vertexInputNode,"source");
			hasKnots = 1;
		}
	}

	if (!hasPositions)
	{
		return status.Warning(FS("No vertex position input node in spline: ") + TO_FSTRING(parent->GetDaeId()), splineNode->line);
	}
	if (!hasKnots)
	{
		return status.Warning(FS("No knot sequence input node in spline: ") + TO_FSTRING(parent->GetDaeId()), splineNode->line);
	}

	xmlNode* positionSrcNode = NULL;
	xmlNode* knotSrcNode = NULL;

	// Read in the data sources
	xmlNodeList sourceDataNodes;
	FindChildrenByType(splineNode, DAE_SOURCE_ELEMENT, sourceDataNodes);
	for (xmlNodeList::iterator it = sourceDataNodes.begin(); it != sourceDataNodes.end(); ++it)
	{
		xmlNode* sourceNode = *it;
		
		string srcid = ReadNodeProperty(*it,"id");
		
		if( strcmp( srcid.c_str(), positionSrc.substr(1).c_str() ) == 0 )
			positionSrcNode = sourceNode;
		else if( strcmp( srcid.c_str(), sknots.substr(1).c_str() ) == 0 )
			knotSrcNode = sourceNode;
	}
	
	if (positionSrcNode == NULL)
	{
		return status.Warning(FS("No vertex position source element in spline: ") + TO_FSTRING(parent->GetDaeId()), splineNode->line);
	}
	
	xmlNode* farrayNode = FindChildByType(positionSrcNode, "float_array");
	if (farrayNode == NULL)
	{
		return status.Warning(FS("No vertex position float array element in spline: ") + TO_FSTRING(parent->GetDaeId()), positionSrcNode->line);
	}
	
	// Setup the curve data
	const char* content = ReadNodeContentDirect(farrayNode);
	int32 icount = ReadNodeCount(farrayNode);
	for(int32 i = 0; i < icount / 3; ++i)
	{
		FMVector3 p = FUStringConversion::ToPoint(&content);
		cvs.push_back(p);
	}
	
	farrayNode = FindChildByType(knotSrcNode, "float_array");
	
	if(farrayNode==NULL)
	{
		return status.Warning(FS("No knot sequence float array element in spline: ") + TO_FSTRING(parent->GetDaeId()), farrayNode->line);
	}
	
	content = ReadNodeContentDirect(farrayNode);
	
	icount = atoi(ReadNodeProperty( farrayNode, "count").c_str());
	
	// setup the curve data
	for( int i=0; i< icount; i++ )
	{
		float f = FUStringConversion::ToFloat(&content);
		knots.push_back(double(f));
	}
	
	return status;
}

// Write out the <spline> node to the COLLADA xml tree
xmlNode* FCDGeometrySpline::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* splineNode = AddChild(parentNode, DAE_SPLINE_ELEMENT);
	AddAttribute(splineNode, DAE_CLOSED_ATTRIBUTE, isClosed);

	// Export the control point source
	FUSStringBuilder controlPointSourceId(GetParent()->GetDaeId()); controlPointSourceId += "-cvs";
	AddSourcePosition(splineNode, controlPointSourceId.ToCharPtr(), cvs);

	// Export the knots
	FUSStringBuilder knotSourceId(GetParent()->GetDaeId());
	knotSourceId += "-knots";
	FloatList floatKnots; floatKnots.reserve(knots.size());
	for (FCDKnots::const_iterator itK = knots.begin(); itK != knots.end(); ++itK)
	{
		floatKnots.push_back(float(*itK));
	}
	AddSourceFloat(splineNode, knotSourceId.ToCharPtr(), floatKnots, "KNOT");

	// Write out the control vertices information
	xmlNode* verticesNode = AddChild(splineNode, DAE_CONTROL_VERTICES_ELEMENT);
	AddInput(verticesNode, controlPointSourceId.ToCharPtr(), DAE_POSITION_SPLINE_INPUT);
	AddInput(verticesNode, knotSourceId.ToCharPtr(), DAE_KNOT_SPLINE_INPUT);
	return splineNode;
}
