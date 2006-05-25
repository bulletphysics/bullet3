/*
	Copyright (C) 2006 Feeling Software Inc.
	Available only to licensees.
	Distribution of this file or its content is strictly prohibited.
*/

#ifndef _FC_TEST_SCENE_
#define _FC_TEST_SCENE_

#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDLibrary.h"

class FCDAnimation;
class FCDEffectProfileFX;
class FCDEffectStandard;
class FCDExtra;
class FCDGeometryMesh;
class FCDGeometrySpline;
class FCDMorphController;
class FCDSceneNode;
class FCDSkinController;

namespace FCTestExportImport
{
	// Information pushing functions for the export
	void FillCameraLibrary(FCDCameraLibrary* library);
	void FillLightLibrary(FCDLightLibrary* library);
	void FillVisualScene(FCDSceneNode* scene);
	void FillExtraTree(FCDExtra* extra);
	
	void FillGeometryLibrary(FCDGeometryLibrary* library);
	void FillGeometryMesh(FCDGeometryMesh* mesh);
	void FillGeometrySpline(FCDGeometrySpline* spline);
	void FillControllerLibrary(FCDControllerLibrary* library);
	void FillControllerMorph(FCDMorphController* controller);
	void FillControllerSkin(FCDSkinController* controller);

	void FillImageLibrary(FCDImageLibrary* library);
	void FillMaterialLibrary(FCDMaterialLibrary* library);
	void FillEffectStandard(FCDEffectStandard* profile);
	void FillEffectFX(FCDEffectProfileFX* profile);

	void FillAnimationLibrary(FCDAnimationLibrary* library);
	void FillAnimationLight(FCDocument* document, FCDAnimation* animationTree);

	// Re-import verification functions
	void CheckCameraLibrary(FCDCameraLibrary* library);
	void CheckLightLibrary(FCDLightLibrary* library);
	void CheckVisualScene(FCDSceneNode* imported);
	void CheckExtraTree(FCDExtra* extra);

	void CheckGeometryLibrary(FCDGeometryLibrary* library);
	void CheckGeometryMesh(FCDGeometryMesh* mesh);
	void CheckGeometrySpline(FCDGeometrySpline* spline);
	void CheckControllerLibrary(FCDControllerLibrary* library);
	void CheckControllerMorph(FCDMorphController* controller);
	void CheckControllerSkin(FCDSkinController* controller);

	void CheckImageLibrary(FCDImageLibrary* library);
	void CheckMaterialLibrary(FCDMaterialLibrary* library);
	void CheckEffectStandard(FCDEffectStandard* profile);
	void CheckEffectFX(FCDEffectProfileFX* profile);

	void CheckAnimationLibrary(FCDAnimationLibrary* library);
	void CheckAnimationLight(FCDocument* document, FCDAnimation* animationTree);
};

#endif // _FC_TEST_SCENE_