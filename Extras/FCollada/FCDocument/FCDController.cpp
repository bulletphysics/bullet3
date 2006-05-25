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
#include "FCDocument/FCDController.h"
#include "FCDocument/FCDSkinController.h"
#include "FCDocument/FCDMorphController.h"
#include "FUtils/FUDaeParser.h"
using namespace FUDaeParser;

FCDController::FCDController(FCDocument* document) : FCDEntity(document, "Controller")
{
	skinController = NULL;
	morphController = NULL;
}

FCDController::~FCDController()
{
	SAFE_DELETE(skinController);
	SAFE_DELETE(morphController);
}


// Sets the type of this controller to a skin controller.
FCDSkinController* FCDController::CreateSkinController()
{
	SAFE_DELETE(skinController);
	SAFE_DELETE(morphController);
	skinController = new FCDSkinController(GetDocument(), this);
	return skinController;
}

// Sets the type of this controller to a morph controller.
FCDMorphController* FCDController::CreateMorphController()
{
	SAFE_DELETE(skinController);
	SAFE_DELETE(morphController);
	morphController = new FCDMorphController(GetDocument(), this);
	return morphController;
}

FCDEntity* FCDController::GetBaseTarget()
{
	if (skinController != NULL) return skinController->GetTarget();
	else if (morphController != NULL) return morphController->GetBaseTarget();
	else return NULL;
}

const FCDEntity* FCDController::GetBaseTarget() const
{
	if (skinController != NULL) return skinController->GetTarget();
	else if (morphController != NULL) return morphController->GetBaseTarget();
	else return NULL;
}

// Load this controller from a Collada <controller> node
FUStatus FCDController::LoadFromXML(xmlNode* controllerNode)
{
	FUStatus status = FCDEntity::LoadFromXML(controllerNode);
	if (!status) return status;
	if (!IsEquivalent(controllerNode->name, DAE_CONTROLLER_ELEMENT))
	{
		return status.Warning(FS("Unexpected node in controller library: <") + TO_FSTRING((const char*) controllerNode->name) + FC(">."), controllerNode->line);
	}

	// COLLADA 1.3 backward compatiblity: read in the 'target' attribute
	targetId = ReadNodeProperty(controllerNode, DAE_TARGET_ATTRIBUTE);

	// Find the <skin> or <morph> element and process it
	xmlNode* skinNode = FindChildByType(controllerNode, DAE_CONTROLLER_SKIN_ELEMENT);
	xmlNode* morphNode = FindChildByType(controllerNode, DAE_CONTROLLER_MORPH_ELEMENT);
	if (skinNode != NULL && morphNode != NULL)
	{
		status.Warning(FS("A controller cannot be both a skin and a morpher: ") + TO_FSTRING(GetDaeId()), controllerNode->line);
	}
	if (skinNode != NULL)
	{
		// Create and parse in the skin controller
		FCDSkinController* controller = CreateSkinController();
		status.AppendStatus(controller->LoadFromXML(skinNode));
	}
	else if (morphNode != NULL)
	{
		// Create and parse in the morph controller
		FCDMorphController* controller = CreateMorphController();
		status.AppendStatus(controller->LoadFromXML(morphNode));
	}
	else
	{
		status.Warning(FS("No base type element, <skin> or <morph>, found for controller: ") + TO_FSTRING(GetDaeId()), controllerNode->line);
	}
	return status;
}

// Write out this controller to a COLLADA xml node tree
xmlNode* FCDController::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* controllerNode = FCDEntity::WriteToEntityXML(parentNode, DAE_CONTROLLER_ELEMENT);
	if (skinController != NULL) skinController->WriteToXML(controllerNode);
	if (morphController != NULL) morphController->WriteToXML(controllerNode);
	FCDEntity::WriteToExtraXML(controllerNode);
	return controllerNode;
}

// Link this controller to the rest of the document: joints/base geometry...
FUStatus FCDController::Link()
{
	FUStatus status;

	// Link the controller
	if (skinController != NULL) status.AppendStatus(skinController->Link());
	if (morphController != NULL) status.AppendStatus(morphController->Link());
	return status;
}
