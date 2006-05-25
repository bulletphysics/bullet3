/*
	Copyright (C) 2006 Feeling Software Inc.
	Available only to licensees.
	Distribution of this file or its content is strictly prohibited.
*/

#include "StdAfx.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDCamera.h"
#include "FCDocument/FCDGeometry.h"
#include "FCDocument/FCDGeometryMesh.h"
#include "FCDocument/FCDGeometrySource.h"
#include "FCDocument/FCDGeometrySpline.h"
#include "FCDocument/FCDGeometryPolygons.h"
#include "FCDocument/FCDLibrary.h"
#include "FCDocument/FCDSceneNode.h"
#include "FCDocument/FCDTransform.h"

#include "FCTestExportImport.h"
using namespace FCTestExportImport;

// Test import of a code-generated scene, with library entities.
// Does the export, re-import and validates that the information is intact.
void TestUniqueScene()
{
	// Write out a simple document with three visual scenes
	FCDocument* doc = new FCDocument();
	FCDSceneNode* sceneNode1 = doc->AddVisualScene();
	sceneNode1->SetName(FC("Scene1"));
	FCDSceneNode* sceneNode2 = doc->AddVisualScene();
	sceneNode2->SetName(FC("Scene2"));
	FCDSceneNode* sceneNode3 = doc->AddVisualScene();
	sceneNode3->SetName(FC("Scene3"));
	FillVisualScene(sceneNode2);

	// Fill in the other libraries
	FillImageLibrary(doc->GetImageLibrary());
	FillCameraLibrary(doc->GetCameraLibrary());
	FillLightLibrary(doc->GetLightLibrary());
	FillGeometryLibrary(doc->GetGeometryLibrary());
	FillControllerLibrary(doc->GetControllerLibrary()); // must occur after FillGeometryLibrary.
	FillMaterialLibrary(doc->GetMaterialLibrary()); // must occur after FillImageLibrary.
	FillAnimationLibrary(doc->GetAnimationLibrary()); // must occur last.
	doc->WriteToFile(FC("TestOut.dae"));
	FailIf(sceneNode1->GetDaeId() == sceneNode2->GetDaeId());
	FailIf(sceneNode1->GetDaeId() == sceneNode3->GetDaeId());
	FailIf(sceneNode2->GetDaeId() == sceneNode3->GetDaeId());

	// Import back this document
	FCDocument* idoc = new FCDocument();
	FUStatus stat = idoc->LoadFromFile(FC("TestOut.dae"));
#ifdef _WIN32
	OutputDebugStringW(stat.GetErrorString());
#endif
	PassIf(stat.IsSuccessful());

	// Verify that all the data we pushed is still available
	// Note that visual scenes may be added by other tests: such as for joints.
	FCDVisualSceneNodeLibrary* vsl = idoc->GetVisualSceneLibrary();
	PassIf(vsl->GetEntityCount() >= 3);

	// Verify that the visual scene ids are unique.
	for (size_t i = 0; i < vsl->GetEntityCount(); ++i)
	{
		FCDSceneNode* inode = vsl->GetEntity(i);
		for (size_t j = 0; j < i; ++j)
		{
			FCDSceneNode* jnode = vsl->GetEntity(j);
			FailIf(inode->GetDaeId() == jnode->GetDaeId());
		}
	}

	// Verify that the three wanted visual scene ids exist and find the one we fill in.
	bool found1 = false, found3 = false;
	FCDSceneNode* found2 = NULL;
	for (size_t i = 0; i < vsl->GetEntityCount(); ++i)
	{
		FCDSceneNode* inode = vsl->GetEntity(i);
		if (inode->GetDaeId() == sceneNode1->GetDaeId())
		{
			FailIf(found1);
			PassIf(inode->GetName() == FC("Scene1"));
			found1 = true;
		}
		else if (inode->GetDaeId() == sceneNode3->GetDaeId())
		{
			FailIf(found3);
			PassIf(inode->GetName() == FC("Scene3"));
			found3 = true;
		}
		else if (inode->GetDaeId() == sceneNode2->GetDaeId())
		{
			FailIf(found2 != NULL);
			PassIf(inode->GetName() == FC("Scene2")); 
			found2 = inode;
		}
	}
	PassIf(found2 != NULL);
	CheckVisualScene(found2);

	// Compare all these re-imported library contents
	CheckImageLibrary(idoc->GetImageLibrary());
	CheckCameraLibrary(idoc->GetCameraLibrary());
	CheckLightLibrary(idoc->GetLightLibrary());
	CheckGeometryLibrary(idoc->GetGeometryLibrary());
	CheckControllerLibrary(idoc->GetControllerLibrary());
	CheckMaterialLibrary(idoc->GetMaterialLibrary());
	CheckAnimationLibrary(idoc->GetAnimationLibrary());
}
