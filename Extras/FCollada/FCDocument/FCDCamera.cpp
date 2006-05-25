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
#include "FCDocument/FCDCamera.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDAnimated.h"
#include "FCDocument/FCDSceneNode.h"
#include "FUtils/FUStringConversion.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDCamera::FCDCamera(FCDocument* document) : FCDTargetedEntity(document, "Camera")
{
	isPerspective = true;
	isOrthographic = false;
	viewY = viewX = 60.0f;
	hasAperture = hasHorizontalView = hasVerticalView = false;
	nearZ = 1.0f;
	farZ = 1000.0f;
	aspectRatio = 1.0f;
	horizontalAperture = verticalAperture = lensSqueeze = 1.0f;
}

FCDCamera::~FCDCamera()
{
}

void FCDCamera::SetFovX(float _viewX)
{
	viewX = _viewX;
	if (hasVerticalView && !IsEquivalent(viewX, 0.0f)) aspectRatio = viewY / viewX;
	hasHorizontalView = true;
}

void FCDCamera::SetFovY(float _viewY)
{
	viewY = _viewY;
	if (hasHorizontalView && !IsEquivalent(viewX, 0.0f)) aspectRatio = viewY / viewX;
	hasVerticalView = true;
}

void FCDCamera::SetAspectRatio(float _aspectRatio)
{
	aspectRatio = _aspectRatio;
}

// Load this camera from the given COLLADA document's node
FUStatus FCDCamera::LoadFromXML(xmlNode* cameraNode)
{
	FUStatus status = FCDEntity::LoadFromXML(cameraNode);
	if (!status) return status;
	if (!IsEquivalent(cameraNode->name, DAE_CAMERA_ELEMENT))
	{
		return status.Warning(FS("Camera library contains unknown element."), cameraNode->line);
	}

	// COLLADA 1.3: Grab the techniques and their <optics><program> children
	xmlNode* commonTechniqueNode1_3 = FindTechnique(cameraNode, DAE_COMMON_PROFILE);
	xmlNode* commonOpticsNode1_3 = FindChildByType(commonTechniqueNode1_3, DAE_OPTICS_ELEMENT);
	xmlNode* commonProgramNode1_3 = FindChildByType(commonOpticsNode1_3, DAE_PROGRAM_ELEMENT);
	xmlNode* maxTechniqueNode1_3 = FindTechnique(cameraNode, DAEMAX_MAX_PROFILE);
	xmlNode* maxOpticsNode1_3 = FindChildByType(maxTechniqueNode1_3, DAE_OPTICS_ELEMENT);
	xmlNode* maxProgramNode1_3 = FindChildByType(maxOpticsNode1_3, DAE_PROGRAM_ELEMENT);
	xmlNode* mayaTechniqueNode = FindTechnique(cameraNode, DAEMAYA_MAYA_PROFILE);
	xmlNode* mayaOpticsNode = FindChildByType(mayaTechniqueNode, DAE_OPTICS_ELEMENT);
	xmlNode* mayaProgramNode = FindChildByType(mayaOpticsNode, DAE_PROGRAM_ELEMENT);
	bool isCollada1_3 = commonTechniqueNode1_3 != NULL;

	// COLLADA 1.4: Grab the <optics> element's techniques
	xmlNode* opticsNode = FindChildByType(cameraNode, DAE_OPTICS_ELEMENT);
	xmlNode* commonTechniqueNode = FindChildByType(opticsNode, DAE_TECHNIQUE_COMMON_ELEMENT);
	xmlNode* maxTechniqueNode = FindTechnique(opticsNode, DAEMAX_MAX_PROFILE);
	xmlNode* cameraContainerNode = (isCollada1_3) ? ((maxProgramNode1_3 != NULL) ? maxProgramNode1_3 : commonProgramNode1_3) : NULL;
	if (mayaProgramNode == NULL) mayaProgramNode = FindTechnique(opticsNode, DAEMAYA_MAYA_PROFILE);

	// Figure out the camera type
	if (isCollada1_3)
	{
		FUUri programType = ReadNodeUrl(maxProgramNode1_3);
		if (programType.prefix.empty()) programType = ReadNodeUrl(commonProgramNode1_3);
		if (programType.prefix.empty())
		{
			return status.Fail(FS("No standard program type for camera: ") + TO_FSTRING(GetDaeId()), cameraNode->line);
		}
		string cameraType = TO_STRING(programType.prefix);
		isOrthographic = cameraType == DAE_ORTHOGRAPHIC_CAMERA_TYPE;
		isPerspective = cameraType == DAE_PERSPECTIVE_CAMERA_TYPE;
	}
	else
	{
		// Retrieve the <perspective> or <orthographic> element
		cameraContainerNode = FindChildByType(commonTechniqueNode, DAE_CAMERA_ORTHO_ELEMENT);
		isOrthographic = cameraContainerNode != NULL;
		if (!isOrthographic)
		{
			cameraContainerNode = FindChildByType(commonTechniqueNode, DAE_CAMERA_PERSP_ELEMENT);
			isPerspective = cameraContainerNode != NULL;
		}
		else
		{
			isPerspective = false;
		}
	}

	// Check the necessary camera structures
	if (cameraContainerNode == NULL)
	{
		return status.Fail(FS("Cannot find parameter root node for camera: ") + TO_FSTRING(GetDaeId()), cameraNode->line);
	}
	if (!(isPerspective ^ isOrthographic))
	{
		return status.Fail(FS("Unknown program type for camera: ") + TO_FSTRING(GetDaeId()), cameraContainerNode->line);
	}

	// Setup the camera according to the type and its parameters
	// Retrieve all the camera parameters
	StringList parameterNames;
	xmlNodeList parameterNodes;
	FindParameters(cameraContainerNode, parameterNames, parameterNodes);
	FindParameters(maxTechniqueNode, parameterNames, parameterNodes);
	FindParameters(mayaProgramNode, parameterNames, parameterNodes);

	#define CAM_PARAMETER(colladaParam, memberFunction, animatedMember) \
	if (parameterName == colladaParam) { \
		memberFunction(FUStringConversion::ToFloat(parameterValue)); \
		FCDAnimatedFloat::Create(GetDocument(), parameterNode, &animatedMember); } else

	size_t parameterCount = parameterNodes.size();
	for (size_t i = 0; i < parameterCount; ++i)
	{
		xmlNode* parameterNode = parameterNodes[i];
		const string& parameterName = parameterNames[i];
		const char* parameterValue = ReadNodeContentDirect(parameterNode);

		// Process the camera parameters
		CAM_PARAMETER(DAE_ZNEAR_CAMERA_PARAMETER, SetNearZ, nearZ)
		CAM_PARAMETER(DAE_ZFAR_CAMERA_PARAMETER, SetFarZ, farZ)
		CAM_PARAMETER(DAE_XFOV_CAMERA_PARAMETER, SetFovX, viewX)
		CAM_PARAMETER(DAE_YFOV_CAMERA_PARAMETER, SetFovY, viewY)
		CAM_PARAMETER(DAE_XMAG_CAMERA_PARAMETER, SetMagX, viewX)
		CAM_PARAMETER(DAE_YMAG_CAMERA_PARAMETER, SetMagY, viewY)
		CAM_PARAMETER(DAE_ASPECT_CAMERA_PARAMETER, SetAspectRatio, aspectRatio)
		CAM_PARAMETER(DAEMAYA_VAPERTURE_PARAMETER, SetVerticalAperture, verticalAperture)
		CAM_PARAMETER(DAEMAYA_HAPERTURE_PARAMETER, SetHorizontalAperture, horizontalAperture)
		CAM_PARAMETER(DAEMAYA_LENSSQUEEZE_PARAMETER, SetLensSqueeze, lensSqueeze)
		if (parameterName == DAEMAX_TARGET_CAMERA_PARAMETER) 
			SetTargetId(parameterValue); 
		else

		// COLLADA 1.3 backward compatibility
		CAM_PARAMETER(DAE_ZNEAR_CAMERA_PARAMETER1_3, SetNearZ, nearZ)
		CAM_PARAMETER(DAE_ZFAR_CAMERA_PARAMETER1_3, SetFarZ, farZ)
		CAM_PARAMETER(DAE_XFOV_CAMERA_PARAMETER1_3, SetFovX, viewX)
		CAM_PARAMETER(DAE_YFOV_CAMERA_PARAMETER1_3, SetFovY, viewY)
		CAM_PARAMETER(DAE_RIGHT_CAMERA_PARAMETER1_3, SetMagX, viewX)
		CAM_PARAMETER(DAE_TOP_CAMERA_PARAMETER1_3, SetMagY, viewY)
		CAM_PARAMETER(DAEMAYA_VAPERTURE_PARAMETER1_3, SetVerticalAperture, verticalAperture)
		CAM_PARAMETER(DAEMAYA_HAPERTURE_PARAMETER1_3, SetHorizontalAperture, horizontalAperture)
		CAM_PARAMETER(DAEMAYA_LENSSQUEEZE_PARAMETER1_3, SetLensSqueeze, lensSqueeze)
		if (parameterName == DAE_LEFT_CAMERA_PARAMETER1_3) {} // Don't process
		else if (parameterName == DAE_BOTTOM_CAMERA_PARAMETER1_3) {} // Don't process

		// Max-specific parameter: target
		else if (parameterName == DAEMAX_TARGET_CAMERA_PARAMETER
			|| parameterName == DAEMAX_TARGET_CAMERA_PARAMETER1_3)
		{
			SetTargetId(parameterValue);
		}
		else
		{
			status.Warning(FS("Unknown parameter for camera: ") + TO_FSTRING(GetDaeId()), parameterNode->line);
		}
	}

	return status;
}

// Write out this camera to the COLLADA XML document
xmlNode* FCDCamera::WriteToXML(xmlNode* parentNode) const
{
	// Create the base camera node
	xmlNode* cameraNode = WriteToEntityXML(parentNode, DAE_CAMERA_ELEMENT);
	xmlNode* opticsNode = AddChild(cameraNode, DAE_OPTICS_ELEMENT);
	xmlNode* baseNode = AddChild(opticsNode, DAE_TECHNIQUE_COMMON_ELEMENT);
	const char* baseNodeName;
	if (isPerspective) baseNodeName = DAE_CAMERA_PERSP_ELEMENT;
	else if (isOrthographic) baseNodeName = DAE_CAMERA_ORTHO_ELEMENT;
	else baseNodeName = DAEERR_UNKNOWN_ELEMENT;
	baseNode = AddChild(baseNode, baseNodeName);

	// Write out the basic camera parameters
	if (isPerspective)
	{
		if (hasHorizontalView)
		{
			xmlNode* viewNode = AddChild(baseNode, DAE_XFOV_CAMERA_PARAMETER, viewX);
			GetDocument()->WriteAnimatedValueToXML(&viewX, viewNode, "horizontal_fov");
		}
		if (!hasHorizontalView || hasVerticalView)
		{
			xmlNode* viewNode = AddChild(baseNode, DAE_YFOV_CAMERA_PARAMETER, viewY);
			GetDocument()->WriteAnimatedValueToXML(&viewY, viewNode, "vertical_fov");
		}
		if (!hasHorizontalView || !hasVerticalView)
		{
			xmlNode* aspectNode = AddChild(baseNode, DAE_ASPECT_CAMERA_PARAMETER, aspectRatio);
			GetDocument()->WriteAnimatedValueToXML(&aspectRatio, aspectNode, "aspect_ratio");
		}
	}
	if (isOrthographic)
	{
		if (hasHorizontalView)
		{
			xmlNode* viewNode = AddChild(baseNode, DAE_XMAG_CAMERA_PARAMETER, viewX);
			GetDocument()->WriteAnimatedValueToXML(&viewX, viewNode, "horizontal_zoom");
		}
		if (!hasHorizontalView || hasVerticalView)
		{
			xmlNode* viewNode = AddChild(baseNode, DAE_YMAG_CAMERA_PARAMETER, viewY);
			GetDocument()->WriteAnimatedValueToXML(&viewY, viewNode, "vertical_zoom");
		}
		if (!hasHorizontalView || !hasVerticalView)
		{
			xmlNode* aspectNode = AddChild(baseNode, DAE_ASPECT_CAMERA_PARAMETER, aspectRatio);
			GetDocument()->WriteAnimatedValueToXML(&aspectRatio, aspectNode, "aspect_ratio");
		}
	}

	// Near/Far clip plane distance
	xmlNode* clipNode = AddChild(baseNode, DAE_ZNEAR_CAMERA_PARAMETER, nearZ);
	GetDocument()->WriteAnimatedValueToXML(&nearZ, clipNode, "near_clip");
	clipNode = AddChild(baseNode, DAE_ZFAR_CAMERA_PARAMETER, farZ);
	GetDocument()->WriteAnimatedValueToXML(&farZ, clipNode, "near_clip");

	// Add the application-specific technique/parameters
	// Maya has extra aperture information
	if (hasAperture)
	{
		xmlNode* techniqueMayaNode = AddTechniqueChild(opticsNode, DAEMAYA_MAYA_PROFILE);
		xmlNode* apertureNode = AddChild(techniqueMayaNode, DAEMAYA_VAPERTURE_PARAMETER, verticalAperture);
		GetDocument()->WriteAnimatedValueToXML(&verticalAperture, apertureNode, "vertical_aperture");
		apertureNode = AddChild(techniqueMayaNode, DAEMAYA_HAPERTURE_PARAMETER, horizontalAperture);
		GetDocument()->WriteAnimatedValueToXML(&horizontalAperture, apertureNode, "horizontal_aperture");
		apertureNode = AddChild(techniqueMayaNode, DAEMAYA_LENSSQUEEZE_PARAMETER, lensSqueeze);
		GetDocument()->WriteAnimatedValueToXML(&lensSqueeze, apertureNode, "lens_squeeze");
	}

	// Max has target information
	if (GetTargetNode() != NULL)
	{
		xmlNode* techniqueMaxNode = AddTechniqueChild(opticsNode, DAEMAX_MAX_PROFILE);
		AddChild(techniqueMaxNode, DAEMAX_TARGET_CAMERA_PARAMETER, GetTargetNode()->GetDaeId());
	}

	FCDEntity::WriteToExtraXML(cameraNode);
	return cameraNode;
}
