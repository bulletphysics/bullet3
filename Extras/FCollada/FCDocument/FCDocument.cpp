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
#include "FCDocument/FCDAnimation.h"
#include "FCDocument/FCDAnimationChannel.h"
#include "FCDocument/FCDAnimationClip.h"
#include "FCDocument/FCDAnimationCurve.h"
#include "FCDocument/FCDAsset.h"
#include "FCDocument/FCDCamera.h"
#include "FCDocument/FCDController.h"
#include "FCDocument/FCDGeometry.h"
#include "FCDocument/FCDImage.h"
#include "FCDocument/FCDLight.h"
#include "FCDocument/FCDLibrary.h"
#include "FCDocument/FCDMaterial.h"
#include "FCDocument/FCDMaterialLibrary.h"
#include "FCDocument/FCDPhysicsMaterial.h"
#include "FCDocument/FCDPhysicsModel.h"
#include "FCDocument/FCDPhysicsSceneNode.h"
#include "FCDocument/FCDSceneNode.h"
#include "FCDocument/FCDTexture.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
#include "FUtils/FUFileManager.h"
#include "FUtils/FUUniqueStringMap.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDocument::FCDocument()
{
	fileManager = new FUFileManager();
	asset = new FCDAsset(this);
	uniqueNameMap = new FUSUniqueStringMap();

	animationLibrary = new FCDLibrary<FCDAnimation>(this);
	animationClipLibrary = new FCDLibrary<FCDAnimationClip>(this);
	cameraLibrary = new FCDLibrary<FCDCamera>(this);
	controllerLibrary = new FCDLibrary<FCDController>(this);
	geometryLibrary = new FCDLibrary<FCDGeometry>(this);
	imageLibrary = new FCDLibrary<FCDImage>(this);
	lightLibrary = new FCDLibrary<FCDLight>(this);
	materialLibrary = new FCDMaterialLibrary(this);
	visualSceneLibrary = new FCDLibrary<FCDSceneNode>(this);

	//physics
	physicsMaterialLibrary = new FCDLibrary<FCDPhysicsMaterial>(this);
	physicsModelLibrary = new FCDLibrary<FCDPhysicsModel>(this);
	physicsSceneLibrary = new FCDLibrary<FCDPhysicsSceneNode>(this);
	physicsSceneRoot = NULL;

	animatedValues.reserve(1024);
	visualSceneRoot = NULL;

	// Document global parameters
	lengthUnitConversion = 1.0f;
	lengthUnitWanted = -1.0f;
	hasStartTime = hasEndTime = false;
	startTime = endTime = 0.0f;
}

FCDocument::~FCDocument()
{
	// Must be released first
	CLEAR_POINTER_VECTOR(layers);
	CLEAR_POINTER_VECTOR(animatedValues);
	animatedValueMap.clear();
	SAFE_DELETE(visualSceneLibrary);

	SAFE_DELETE(animationLibrary);
	SAFE_DELETE(animationClipLibrary);
	SAFE_DELETE(cameraLibrary);
	SAFE_DELETE(controllerLibrary);
	SAFE_DELETE(geometryLibrary);
	SAFE_DELETE(imageLibrary);
	SAFE_DELETE(lightLibrary);
	SAFE_DELETE(materialLibrary);
	SAFE_DELETE(physicsModelLibrary);
	SAFE_DELETE(physicsMaterialLibrary);
	SAFE_DELETE(physicsSceneLibrary);

	physicsSceneRoot = NULL;
	visualSceneRoot = NULL;

	SAFE_DELETE(fileManager);
	SAFE_DELETE(asset);
}

// Retrieve the list of all the COLLADA materials in this document
const FCDMaterialList& FCDocument::GetMaterialList()
{
	return materialLibrary->GetMaterials();
}

// These two parameters now belong to FCDAsset. Keep them here for a few version, for backward compatibility
const FMVector3& FCDocument::GetUpAxis() const { return asset->GetUpAxis(); }
float FCDocument::GetLengthUnit() const { return asset->GetUnitConversionFactor(); }

// Search for a driven curve that needs this animated value as a driver
bool FCDocument::LinkDriver(FCDAnimated* animated)
{
	if (animated->GetTargetPointer().empty()) return false;

	bool driven = false;
	size_t animationCount = animationLibrary->GetEntityCount();
	for (size_t i = 0; i < animationCount; ++i)
	{
		FCDAnimation* animation = animationLibrary->GetEntity(i);
		driven |= animation->LinkDriver(animated);
	}
	return driven;
}

// Search for an animation channel targeting the given pointer
void FCDocument::FindAnimationChannels(const string& pointer, FCDAnimationChannelList& channels)
{
	if (pointer.empty()) return;

	size_t animationCount = (uint32) animationLibrary->GetEntityCount();
	for (size_t i = 0; i < animationCount; ++i)
	{
		FCDAnimation* animation = animationLibrary->GetEntity(i);
		animation->FindAnimationChannels(pointer, channels);
	}
}

// Gather a list of the indices of animated array element belonging to the node
void FCDocument::FindAnimationChannelsArrayIndices(xmlNode* targetArray, Int32List& animatedIndices)
{
	// Calculte the node's pointer
	string pointer;
	CalculateNodeTargetPointer(targetArray, pointer);
	if (pointer.empty()) return;

	// Retrieve the channels for this pointer and extract their matrix indices.
	FCDAnimationChannelList channels;
	FindAnimationChannels(pointer, channels);
	for (vector<FCDAnimationChannel*>::iterator it = channels.begin(); it != channels.end(); ++it)
	{
		string qualifier = (*it)->GetTargetQualifier();
		int32 animatedIndex = ReadTargetMatrixElement(qualifier);
		if (animatedIndex != -1) animatedIndices.push_back(animatedIndex);
	}
}

// Search for a specific COLLADA library items with a given COLLADA id.
FCDAnimation* FCDocument::FindAnimation(const string& daeId) { return animationLibrary->FindDaeId(daeId); }
FCDAnimationClip* FCDocument::FindAnimationClip(const string& daeId) { return animationClipLibrary->FindDaeId(daeId); }
FCDCamera* FCDocument::FindCamera(const string& daeId) { return cameraLibrary->FindDaeId(daeId); }
FCDController* FCDocument::FindController(const string& daeId) { return controllerLibrary->FindDaeId(daeId); }
FCDGeometry* FCDocument::FindGeometry(const string& daeId) { return geometryLibrary->FindDaeId(daeId); }
FCDImage* FCDocument::FindImage(const string& daeId) { return imageLibrary->FindDaeId(daeId); }
FCDLight* FCDocument::FindLight(const string& daeId) { return lightLibrary->FindDaeId(daeId); }
FCDTexture* FCDocument::FindTexture(const string& daeId) { return materialLibrary->FindTexture(daeId); }
FCDMaterial* FCDocument::FindMaterial(const string& daeId) { return  materialLibrary->FindMaterial(daeId); }
FCDEffect* FCDocument::FindEffect(const string& daeId) { return  materialLibrary->FindEffect(daeId); }
FCDSceneNode* FCDocument::FindVisualScene(const string& daeId) { return visualSceneLibrary->FindDaeId(daeId); }
FCDPhysicsSceneNode* FCDocument::FindPhysicsScene(const string& daeId) { return physicsSceneLibrary->FindDaeId(daeId); }
FCDPhysicsMaterial* FCDocument::FindPhysicsMaterial(const string& daeId) { return physicsMaterialLibrary->FindDaeId(daeId); }
FCDPhysicsModel* FCDocument::FindPhysicsModel(const string& daeId) { return physicsModelLibrary->FindDaeId(daeId); }
FCDSceneNode* FCDocument::FindSceneNode(const string& daeId)
{
	// Nasty special case: look through all the visual_scenes for the scene node
	size_t visualSceneCount = visualSceneLibrary->GetEntityCount();
	for (size_t i = 0; i < visualSceneCount; ++i)
	{
		FCDSceneNode* visualScene = visualSceneLibrary->GetEntity(i);
		FCDEntity* found = visualScene->FindDaeId(daeId);
		if (found != NULL) return (FCDSceneNode*) found;
	}
	return NULL;
}

// Add an animated value to the list
void FCDocument::RegisterAnimatedValue(FCDAnimated* animated)
{
	// Look for a duplicate in order to avoid memory loss
	//if (animated->GetValueCount() == 0 || FindAnimatedValue(animated->GetValue(0)) != NULL)
	if (animated->GetValueCount() == 0)
	{
		SAFE_DELETE(animated);
		return;
	}

	// List the new animated value
	animatedValues.push_back(animated);

	// Also add to the map the individual values for easy retrieval
	size_t count = animated->GetValueCount();
	for (size_t i = 0; i < count; ++i)
	{
		const float* value = animated->GetValue(i);
		animatedValueMap[value] = animated;
	}
}

// Unregisters an animated value of the document.
void FCDocument::UnregisterAnimatedValue(FCDAnimated* animated)
{
	if (animated != NULL)
	{
		FCDAnimatedList::iterator it = std::find(animatedValues.begin(), animatedValues.end(), animated);
		if (it != animatedValues.end())
		{
			animatedValues.erase(it);

			// Also remove to the map the individual values contained
			size_t count = animated->GetValueCount();
			for (size_t i = 0; i < count; ++i)
			{
				const float* value = animated->GetValue(i);
				FCDAnimatedValueMap::iterator itV = animatedValueMap.find(value);
				if (itV != animatedValueMap.end() && (*itV).second == animated)
				{
					animatedValueMap.erase(itV);
				}
			}
		}
	}
}

// Retrieve an animated value, given a value pointer
FCDAnimated* FCDocument::FindAnimatedValue(float* ptr)
{
	FCDAnimatedValueMap::iterator it = animatedValueMap.find((const float*) ptr);
	return (it != animatedValueMap.end()) ? (*it).second : NULL;
}

// Retrieve an animated value, given a value pointer
const FCDAnimated* FCDocument::FindAnimatedValue(const float* ptr) const
{
	FCDAnimatedValueMap::const_iterator it = animatedValueMap.find(ptr);
	return (it != animatedValueMap.end()) ? (*it).second : NULL;
}

const FCDAnimated* FCDocument::FindNamedAnimated(const string& shader, const string& attribute) const
{
	string::size_type loc = attribute.find('.', 0);
	string attrname = attribute.substr(loc+1);
	string shdname = attribute.substr(0, loc);
	
	string name0 = shader;
	name0.append(string("-fx/"));
	name0.append(attrname);
	
	string name1 = shader;
	name1.append(string("-fx/"));
	name1.append(shdname);
	name1.append(attrname);
	
	for(FCDAnimatedList::const_iterator itA = animatedValues.begin(); itA != animatedValues.end(); ++itA)
	{	
		if(strcmp((*itA)->GetTargetPointer().c_str(), name0.c_str()) == 0 || strcmp((*itA)->GetTargetPointer().c_str(), name1.c_str()) == 0) return (*itA);
	}
			
	return NULL;
}

// Retrieve an animated float value for a given fully qualified target
const float* FCDocument::FindAnimatedTarget(const string& fullyQualifiedTarget)
{
	if (fullyQualifiedTarget.empty()) return NULL;
	string target = (fullyQualifiedTarget[0] == '#') ? fullyQualifiedTarget.substr(1) : fullyQualifiedTarget;
	string pointer, qualifier;
	SplitTarget(target, pointer, qualifier);

	// Find the pointer
	FCDAnimated* animatedValue = NULL;
	for (FCDAnimatedList::iterator itA = animatedValues.begin(); itA != animatedValues.end(); ++itA)
	{
		FCDAnimated* animated = (*itA);
		if (animated->GetTargetPointer() == pointer) { animatedValue = animated; break; }
	}
	if (animatedValue == NULL) return NULL;

	// Return the qualified value
	size_t index = animatedValue->FindQualifier(qualifier);
	if (index == size_t(-1)) return NULL;
	return animatedValue->GetValue(index);
}

// Returns whether a given value pointer is animated
bool FCDocument::IsValueAnimated(const float* ptr) const
{
	const FCDAnimated* animated = FindAnimatedValue(ptr);
	return (animated != NULL) ? animated->HasCurve() : false;
}

// Insert new library elements
FCDSceneNode* FCDocument::AddVisualScene()
{
	return visualSceneRoot = visualSceneLibrary->AddEntity();
}
FCDPhysicsSceneNode* FCDocument::AddPhysicsScene()
{
	return physicsSceneRoot = physicsSceneLibrary->AddEntity();
}

// Structure and enumeration used to order the libraries
enum nodeOrder { ANIMATION=0, ANIMATION_CLIP, IMAGE, TEXTURE, EFFECT, MATERIAL, GEOMETRY, CONTROLLER, CAMERA, LIGHT, VISUAL_SCENE, PHYSICS_MATERIAL, PHYSICS_MODEL, PHYSICS_SCENE, UNKNOWN };
struct xmlOrderedNode { xmlNode* node; nodeOrder order; };
typedef vector<xmlOrderedNode> xmlOrderedNodeList;

// Loads an entire COLLADA document file
FUStatus FCDocument::LoadFromFile(const fstring& filename)
{
	FUStatus status;

	// Push the filename's path unto the file manager's stack
	fileManager->PushRootFile(filename);

#if FCOLLADA_EXCEPTION
	try {
#endif

		// Parse the document into a XML tree
	string xmlFilename = FUStringConversion::ToString(filename);
	xmlDoc* daeDocument = xmlParseFile(xmlFilename.c_str());
	if (daeDocument != NULL)
	{
		xmlNode* rootNode = xmlDocGetRootElement(daeDocument);

		// Read in the whole document from the root node
		status.AppendStatus(LoadDocumentFromXML(rootNode));

		// Free the XML document
		xmlFreeDoc(daeDocument);
	}
	else
	{
		status.Fail(FS("Corrupted COLLADA document: malformed XML."));
	}

	// Clean-up the XML reader
	xmlCleanupParser();

#if FCOLLADA_EXCEPTION
	} catch(const char* sz) {
		status.Fail(FS("Exception caught while parsing a COLLADA document from file: ") + TO_FSTRING(sz));
#ifdef UNICODE
	} catch(const fchar* sz) {
		status.Fail(FS("Exception caught while parsing a COLLADA document from file: ") + sz);
#endif
	} catch(...)	{
		status.Fail(FS("Exception caught while parsing a COLLADA document from file."));
	}
#endif

	// Restore the orignal OS current folder
	fileManager->PopRootFile();

	if (status.IsSuccessful()) status.AppendString(FC("COLLADA document loaded successfully."));
	return status;
}

// Loads an entire COLLADA document from a given NULL-terminated fstring
FUStatus FCDocument::LoadFromText(const fstring& basePath, const fchar* text)
{
	FUStatus status;

	// Push the given path unto the file manager's stack
	fileManager->PushRootPath(basePath);

#if FCOLLADA_EXCEPTION
	try {
#endif

#ifdef UNICODE
	// Downsize the text document into something 8-bit
	string xmlTextString = FUStringConversion::ToString(text);
	const xmlChar* xmlText = (const xmlChar*) xmlTextString.c_str();
#else
	const xmlChar* xmlText = (const xmlChar*) text;
#endif

	// Parse the document into a XML tree
	xmlDoc* daeDocument = xmlParseDoc(const_cast<xmlChar*>(xmlText));
	if (daeDocument != NULL)
	{
		xmlNode* rootNode = xmlDocGetRootElement(daeDocument);

		// Read in the whole document from the root node
		status.AppendStatus(LoadDocumentFromXML(rootNode));

		// Free the XML document
		xmlFreeDoc(daeDocument);
	}
	else
	{
		status.Fail(FS("Corrupted COLLADA document: malformed XML."));
	}

	// Clean-up the XML reader
	xmlCleanupParser();

#if FCOLLADA_EXCEPTION
	} catch(const char* sz) {
		status.Fail(FS("Exception caught while parsing a COLLADA document from a string: ") + TO_FSTRING(sz));
#ifdef UNICODE
	} catch(const fchar* sz) {
		status.Fail(FS("Exception caught while parsing a COLLADA document from a string: ") + sz);
#endif
	} catch(...)	{
		status.Fail(FC("Exception caught while parsing a COLLADA document from a string."));
	}
#endif

	// Restore the orignal OS current folder
	fileManager->PopRootPath();

	if (status.IsSuccessful()) status.AppendString(FC("COLLADA document loaded successfully."));
	return status;
}

FUStatus FCDocument::LoadDocumentFromXML(xmlNode* colladaNode)
{
	FUStatus status;

	// The only root node supported is "COLLADA"
	if (!IsEquivalent(colladaNode->name, DAE_COLLADA_ELEMENT))
	{
		return status.Fail(FS("Valid document contain only the <COLLADA> root element."), colladaNode->line);
	}

	// Bucket the libraries, so that we can read them in our specific order
	// COLLADA 1.4: the libraries are now strongly-typed, so process all the elements
	xmlNode* sceneNode = NULL;
	xmlOrderedNodeList orderedLibraryNodes;
	for (xmlNode* child = colladaNode->children; child != NULL; child = child->next)
	{
		if (child->type != XML_ELEMENT_NODE) continue;

		xmlOrderedNode n;
		n.node = child;
		n.order = UNKNOWN;
		if (IsEquivalent(child->name, DAE_LIBRARY_ELEMENT))
		{
			// COLLADA 1.3: Read in the type attribute of the library to know its content
			string libraryType = ReadNodeProperty(n.node, DAE_TYPE_ATTRIBUTE);
			if (libraryType == DAE_ANIMATION_TYPE) n.order = ANIMATION;
			else if (libraryType == DAE_EFFECT_TYPE) n.order = EFFECT;
			else if (libraryType == DAE_IMAGE_TYPE) n.order = IMAGE;
			else if (libraryType == DAE_TEXTURE_TYPE) n.order = TEXTURE;
			else if (libraryType == DAE_MATERIAL_TYPE) n.order = MATERIAL;
			else if (libraryType == DAE_GEOMETRY_TYPE) n.order = GEOMETRY;
			else if (libraryType == DAE_CAMERA_TYPE) n.order = CAMERA;
			else if (libraryType == DAE_CONTROLLER_TYPE) n.order = CONTROLLER;
			else if (libraryType == DAE_LIGHT_TYPE) n.order = LIGHT;
		}
		else if (IsEquivalent(child->name, DAE_LIBRARY_ANIMATION_ELEMENT)) n.order = ANIMATION;
		else if (IsEquivalent(child->name, DAE_LIBRARY_ANIMATION_CLIP_ELEMENT)) n.order = ANIMATION_CLIP;
		else if (IsEquivalent(child->name, DAE_LIBRARY_CAMERA_ELEMENT)) n.order = CAMERA;
		else if (IsEquivalent(child->name, DAE_LIBRARY_CONTROLLER_ELEMENT)) n.order = CONTROLLER;
		else if (IsEquivalent(child->name, DAE_LIBRARY_EFFECT_ELEMENT)) n.order = EFFECT;
		else if (IsEquivalent(child->name, DAE_LIBRARY_GEOMETRY_ELEMENT)) n.order = GEOMETRY;
		else if (IsEquivalent(child->name, DAE_LIBRARY_IMAGE_ELEMENT)) n.order = IMAGE;
		else if (IsEquivalent(child->name, DAE_LIBRARY_LIGHT_ELEMENT)) n.order = LIGHT;
		else if (IsEquivalent(child->name, DAE_LIBRARY_MATERIAL_ELEMENT)) n.order = MATERIAL;
		else if (IsEquivalent(child->name, DAE_LIBRARY_VSCENE_ELEMENT)) n.order = VISUAL_SCENE;
		else if (IsEquivalent(child->name, DAE_LIBRARY_FFIELDS_ELEMENT)) continue; // Valid, but not processed
		else if (IsEquivalent(child->name, DAE_LIBRARY_NODE_ELEMENT)) continue; // Valid, but not processed
		else if (IsEquivalent(child->name, DAE_LIBRARY_PMATERIAL_ELEMENT)) n.order = PHYSICS_MATERIAL;
		else if (IsEquivalent(child->name, DAE_LIBRARY_PMODEL_ELEMENT)) n.order = PHYSICS_MODEL;
		else if (IsEquivalent(child->name, DAE_LIBRARY_PSCENE_ELEMENT)) n.order = PHYSICS_SCENE;
		else if (IsEquivalent(child->name, DAE_ASSET_ELEMENT))
		{
			// Read in the asset information
			status.AppendStatus(asset->LoadFromXML(child));

			// Calculate the length conversion unit
			// If the wanted unit length is negative, it implies that no conversion is wanted or that the flag was not set
			if (lengthUnitWanted > 0.0f) lengthUnitConversion = asset->GetUnitConversionFactor() / lengthUnitWanted;
			continue;
		}
		else if (IsEquivalent(child->name, DAE_SCENE_ELEMENT))
		{
			// The <scene> element should be the last element of the document
			sceneNode = child;
			continue;
		}
		else
		{
			status.Warning(FS("Unknown base node type: ") + TO_FSTRING((const char*) child->name), child->line);
			continue;
		}

		xmlOrderedNodeList::iterator it;
		for (it = orderedLibraryNodes.begin(); it != orderedLibraryNodes.end(); ++it)
		{
			if ((uint32) n.order < (uint32) (*it).order) break;
		}
		orderedLibraryNodes.insert(it, n);
	}

	// Process the ordered libraries
	size_t libraryNodeCount = orderedLibraryNodes.size();
	for (size_t i = 0; i < libraryNodeCount; ++i)
	{
		xmlOrderedNode& n = orderedLibraryNodes[i];
		switch (n.order)
		{
		case ANIMATION: status.AppendStatus(animationLibrary->LoadFromXML(n.node)); break;
		case ANIMATION_CLIP: status.AppendStatus(animationClipLibrary->LoadFromXML(n.node)); break;
		case CAMERA: status.AppendStatus(cameraLibrary->LoadFromXML(n.node)); break;
		case CONTROLLER: status.AppendStatus(controllerLibrary->LoadFromXML(n.node)); break;
		case GEOMETRY: status.AppendStatus(geometryLibrary->LoadFromXML(n.node)); break;
		case EFFECT: status.AppendStatus(materialLibrary->LoadFromXML(n.node)); break;
		case IMAGE: status.AppendStatus(imageLibrary->LoadFromXML(n.node)); break;
		case LIGHT: status.AppendStatus(lightLibrary->LoadFromXML(n.node)); break;
		case MATERIAL: status.AppendStatus(materialLibrary->LoadFromXML(n.node)); break;
		case TEXTURE: status.AppendStatus(materialLibrary->LoadFromXML(n.node)); break;
		case PHYSICS_MODEL: status.AppendStatus(physicsModelLibrary->LoadFromXML(n.node)); break;
		case PHYSICS_MATERIAL: status.AppendStatus(physicsMaterialLibrary->LoadFromXML(n.node)); break;
		case PHYSICS_SCENE: status.AppendStatus(physicsSceneLibrary->LoadFromXML(n.node)); break;
		case VISUAL_SCENE: status.AppendStatus(visualSceneLibrary->LoadFromXML(n.node)); break;
		case UNKNOWN: default: break;
		}
	}

	// Read in the <scene> element
	if (sceneNode == NULL)
	{
		return status.Warning(FS("No base <scene> element found."), colladaNode->line);
	}

	// COLLADA 1.4: Look for a <instance_physics_scene> element
	xmlNode* instancePhysicsNode = FindChildByType(sceneNode, DAE_INSTANCE_PHYSICS_SCENE_ELEMENT);
	if (instancePhysicsNode != NULL)
	{
		FUUri instanceUri = ReadNodeUrl(instancePhysicsNode);
		if (instanceUri.prefix.length() > 0)
		{
			status.Fail(FS("Cannot externally reference a <physics_scene> element."), sceneNode->line);
		}
		else if (instanceUri.suffix.length() == 0)
		{
			status.Fail(FS("No valid URI fragment for the instantiation of the physics scene."), sceneNode->line);
		}
		else
		{
			// Look for the correct physics scene to instantiate in the libraries
			physicsSceneRoot = FindPhysicsScene(instanceUri.suffix);
			if (physicsSceneRoot == NULL)
			{
				status.Fail(FS("Cannot find the correct <physics_scene> element to instantiate."), sceneNode->line);
			}
		}
	}

	// COLLADA 1.4: Look for a <instance_visual_scene> element
	xmlNode* instanceSceneNode = FindChildByType(sceneNode, DAE_INSTANCE_VSCENE_ELEMENT);
	if (instanceSceneNode != NULL)
	{
		FUUri instanceUri = ReadNodeUrl(instanceSceneNode);
		if (instanceUri.prefix.length() > 0)
		{
			status.Fail(FS("Cannot externally reference a <visual_scene> element."), sceneNode->line);
		}
		else if (instanceUri.suffix.length() == 0)
		{
			status.Fail(FS("No valid URI fragment for the instantiation of the visual scene."), sceneNode->line);
		}
		else
		{
			// Look for the correct visual scene to instantiate in the libraries
			visualSceneRoot = FindVisualScene(instanceUri.suffix);
			if (visualSceneRoot == NULL)
			{
				status.Fail(FS("Cannot find the correct <visual_scene> element to instantiate."), sceneNode->line);
			}
		}
	}
	else
	{
		// COLLADA 1.3 backward-compatibility, use this <scene> as the <visual_scene> element
		visualSceneRoot = visualSceneLibrary->AddEntity();
		status.AppendStatus(visualSceneRoot->LoadFromXML(sceneNode));
	}


	if (visualSceneRoot != NULL)
	{
		// Link the controllers and the joints
		size_t controllerCount = controllerLibrary->GetEntityCount();
		for (size_t i = 0; i < controllerCount; ++i)
		{
			FCDController* controller = controllerLibrary->GetEntity(i);
			status.AppendStatus(controller->Link());
		}

		// Link the targeted entities, for 3dsMax cameras and lights
		size_t cameraCount = cameraLibrary->GetEntityCount();
		for (size_t i = 0; i < cameraCount; ++i)
		{
			FCDCamera* camera = cameraLibrary->GetEntity(i);
			status.AppendStatus(camera->LinkTarget(visualSceneRoot));
		}
		size_t lightCount = lightLibrary->GetEntityCount();
		for (size_t i = 0; i < lightCount; ++i)
		{
			FCDLight* light = lightLibrary->GetEntity(i);
			status.AppendStatus(light->LinkTarget(visualSceneRoot));
		}
	}

	// Check that all the animation curves that need them, have found drivers
	size_t animationCount = animationLibrary->GetEntityCount();
	for (size_t i = 0; i < animationCount; ++i)
	{
		FCDAnimation* animation = animationLibrary->GetEntity(i);
		status.AppendStatus(animation->Link());
	}
	
	// store the post process commands
	postCmds = visualSceneLibrary->GetPostProcessCmds();

	return status;
}

// Writes out the COLLADA document to a file
FUStatus FCDocument::WriteToFile(const fstring& filename) const
{
	FUStatus status;

	// Push the filename's path unto the file manager's stack
	fileManager->PushRootFile(filename);

#if FCOLLADA_EXCEPTION
	try {
#endif
	// Create a new xml root node from this COLLADA document
	xmlNode* rootNode = CreateNode(DAE_COLLADA_ELEMENT);
	status = WriteDocumentToXML(rootNode);
	if (status.IsSuccessful())
	{
		// Create the XML document and write it out to the given filename
		xmlDoc* daeDocument = xmlNewDoc(NULL); // NULL implies version 1.0
		xmlDocSetRootElement(daeDocument, rootNode);
		intptr_t bytesWritten = xmlSaveFormatFileEnc(FUStringConversion::ToString(filename).c_str(), daeDocument, "utf-8", 1);
		if (bytesWritten < 0)
		{
			status.Fail(FS("Unable to write COLLADA document to file '") + filename + FS("'. Verify that the folder exists and the file is writable."), rootNode->line);
		}
		else if (status.IsSuccessful())
		{
			status.AppendString(FC("COLLADA document written successfully."));
		}
		xmlFreeDoc(daeDocument);
	}
	else
	{
		xmlFreeNode(rootNode);
	}

	// Clean-up
	xmlCleanupParser();

#if FCOLLADA_EXCEPTION
	} catch(const char* sz) {
		status.Fail(FS("Exception caught while parsing a COLLADA document from a string: ") + TO_FSTRING(sz));
#ifdef UNICODE
	} catch(const fchar* sz) {
		status.Fail(FS("Exception caught while parsing a COLLADA document from a string: ") + sz);
#endif
	} catch(...)	{
		status.Fail(FC("Exception caught while parsing a COLLADA document from a string."));
	}
#endif

	fileManager->PopRootFile();
	return status;
}

// Writes out the entire COLLADA document to the given XML root node.
FUStatus FCDocument::WriteDocumentToXML(xmlNode* colladaNode) const
{
	FUStatus status;
	if (colladaNode != NULL)
	{
		// Write the COLLADA document version and namespace: schema-required attributes
		AddAttribute(colladaNode, DAE_NAMESPACE_ATTRIBUTE, DAE_SCHEMA_LOCATION);
		AddAttribute(colladaNode, DAE_VERSION_ATTRIBUTE, DAE_SCHEMA_VERSION);

		// Write out the asset tag
		asset->WriteToXML(colladaNode);

		// Record the animation library. This library is built at the end, but should appear before the <scene> element.
		xmlNode* animationLibraryNode = NULL;
		if (!animationLibrary->IsEmpty())
		{
			animationLibraryNode = AddChild(colladaNode, DAE_LIBRARY_ANIMATION_ELEMENT);
		}

		// Export the libraries
#define EXPORT_LIBRARY(memberName, daeElementName) if (!(memberName)->IsEmpty()) { \
			xmlNode* libraryNode = AddChild(colladaNode, daeElementName); \
			memberName->WriteToXML(libraryNode); }

		EXPORT_LIBRARY(animationClipLibrary, DAE_LIBRARY_ANIMATION_CLIP_ELEMENT);
		EXPORT_LIBRARY(cameraLibrary, DAE_LIBRARY_CAMERA_ELEMENT);
		EXPORT_LIBRARY(lightLibrary, DAE_LIBRARY_LIGHT_ELEMENT);
		EXPORT_LIBRARY(imageLibrary, DAE_LIBRARY_IMAGE_ELEMENT);
		EXPORT_LIBRARY(materialLibrary, DAE_LIBRARY_MATERIAL_ELEMENT);
		EXPORT_LIBRARY(geometryLibrary, DAE_LIBRARY_GEOMETRY_ELEMENT);
		EXPORT_LIBRARY(controllerLibrary, DAE_LIBRARY_CONTROLLER_ELEMENT);
		EXPORT_LIBRARY(visualSceneLibrary, DAE_LIBRARY_VSCENE_ELEMENT);

#undef EXPORT_LIBRARY

		// Create the <scene> element and instantiate the selected visual scene.
		xmlNode* sceneNode = AddChild(colladaNode, DAE_SCENE_ELEMENT);
		if (visualSceneRoot != NULL)
		{
			xmlNode* instanceVisualSceneNode = AddChild(sceneNode, DAE_INSTANCE_VSCENE_ELEMENT);
			AddAttribute(instanceVisualSceneNode, DAE_URL_ATTRIBUTE, string("#") + visualSceneRoot->GetDaeId());
		}
		if (physicsSceneRoot != NULL)
		{
			xmlNode* instancePhysicsSceneNode = AddChild(sceneNode, DAE_INSTANCE_PHYSICS_SCENE_ELEMENT);
			AddAttribute(instancePhysicsSceneNode, DAE_URL_ATTRIBUTE, string("#") + physicsSceneRoot->GetDaeId());
		}

		// Write out the animations
		if (animationLibraryNode != NULL)
		{
			animationLibrary->WriteToXML(animationLibraryNode);
		}
	}
	return status;
}

// Writes out a value's animations, if any, to the animation library of a COLLADA xml document.
void FCDocument::WriteAnimatedValueToXML(const float* value, xmlNode* valueNode, const char* wantedSid, int32 arrayElement) const
{
	// Find the value's animations
	FCDAnimated* animated = const_cast<FCDAnimated*>(FindAnimatedValue(value));
	if (animated != NULL && animated->HasCurve() && valueNode != NULL)
	{
		animated->SetArrayElement(arrayElement);

		// Set a sid unto the xml tree node, in order to support animations
		if (!HasNodeProperty(valueNode, DAE_SID_ATTRIBUTE))
		{
			AddNodeSid(valueNode, wantedSid);
		}

		// Calculate the xml tree node's target for the animation channel and write the animation out
		string target;
		CalculateNodeTargetPointer(valueNode, target);
		if (!target.empty())
		{
			for (uint32 i = 0; i < animated->GetValueCount(); ++i)
			{
				FCDAnimationCurve* curve = animated->GetCurve(i);
				if (curve == NULL) continue;
				curve->SetTargetElement(arrayElement);

				FCDAnimationChannel* channel = curve->GetParent();
				FUAssert(channel != NULL, continue);

				channel->SetTargetPointer(target);
			}
		}
	}
}
