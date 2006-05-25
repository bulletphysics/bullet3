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

/**
	@file FCDController.h
	This file contains the FCDController class.
*/

#ifndef _FCD_CONTROLLER_H_
#define _FCD_CONTROLLER_H_

#include "FCDocument/FCDEntity.h"

class FCDocument;
class FCDSkinController;
class FCDMorphController;

/**
	A generic COLLADA controller.
	A COLLADA controller is used to influence a mesh.
	COLLADA defines two types of controller:
	skins (FCDSkinController) and morphers (FCDMorphController).

	@ingroup FCDGeometry
*/
class FCOLLADA_EXPORT FCDController : public FCDEntity
{
private:
	string targetId; // COLLADA 1.3 backward compatibility

	FCDSkinController* skinController;
	FCDMorphController* morphController;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDLibrary::AddEntity function.
		@param document The COLLADA document that owns the controller. */
	FCDController(FCDocument* document);

	/** Destructor: do not use directly.
		Instead, use the FCDLibrary::ReleaseEntity function. */
	virtual ~FCDController();

	/** Retrieves the entity class type.
		This function is a part of the FCDEntity interface.
		@return The entity class type: CONTROLLER. */
	virtual Type GetType() const { return CONTROLLER; };

	/** Retrieves whether this controller is a skin controller.
		@return Whether this controller is a skin controller. */
	bool HasSkinController() const { return skinController != NULL; }

	/** Retrieves whether this controller is a morph controller.
		@return Whether this controller is a morph controller. */
	bool HasMorphController() const { return morphController != NULL; }

	/** Sets the type of this controller to a skin controller.
		This function will release any previously created morpher or skin.
		@return The new skin controller. */
	FCDSkinController* CreateSkinController();

	/** Sets the type of this controller to a morph controller.
		This function will release any previously created morpher or skin.
		@return The new morph controller. */
	FCDMorphController* CreateMorphController();

	/** Retrieves the skin controller.
		This pointer is only valid for skins. To verify that this is a skin,
		check the HasSkinController function.
		@return The skin controller. This pointer will be NULL, if the controller
			is not a skin. */
	FCDSkinController* GetSkinController() { return skinController; }
	const FCDSkinController* GetSkinController() const { return skinController; } /**< See above. */

	/** Retrieves the morph controller.
		This pointer is only valid for skins. To verify that this is a morpher,
		check the HasMorphController function.
		@return The morph controller. This pointer will be NULL, if the controller
			is not a morpher. */
	FCDMorphController* GetMorphController() { return morphController; }
	const FCDMorphController* GetMorphController() const { return morphController; } /**< See above. */

	/** Retrieves the base target entity for this controller.
		The base target entity may be another controller or a geometry entity.
		To change the base target, use the FCDMorphController::SetBaseTarget
		or the FCDSkinController::SetTarget functions.
		@return The base target entity. This pointer will be NULL, if no base target is defined. */
	FCDEntity* GetBaseTarget();
	const FCDEntity* GetBaseTarget() const; /**< See above. */

	/** [INTERNAL] Retrieves the COLLADA id of the target entity.
		This value is only useful for COLLADA 1.3 backward compatibility.
		For more recent COLLADA documents, this value is unused.
		@return The COLLADA id of the target entity. */
	const string& GetTargetId() const { return targetId; }

	/** [INTERNAL] Reads in the \<controller\> element from a given COLLADA XML tree node.
		@param controllerNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the controller.*/
	virtual FUStatus LoadFromXML(xmlNode* controllerNode);

	/** [INTERNAL] Writes out the \<controller\> element to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the controller information.
		@return The created element XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;

	/** [INTERNAL] Links the controller entities with their many targets/influences.
		This function is executed for all the controllers, after the scene graph has been imported.
		It is mainly used to link the skin and its bones.
		@return The status of the linkage.*/
	FUStatus Link();
};

#endif // _FCD_CONTROLLER_H_

