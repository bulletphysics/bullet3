/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	Available only to licensees.
	Distribution of this file or its content is strictly prohibited.
*/

#include "StdAfx.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDSceneNode.h"

// Test import of the Eagle sample, retrieves the "Bone09" joint and does the import sampling to verify its correctness.
void TestImportSampling()
{
	FCDocument document;
	CheckStatus(document.LoadFromFile(FC("Eagle.DAE")));
	FCDSceneNode* node = document.FindSceneNode("Bone09");
	FailIf(node == NULL);

	FloatList keys; FMMatrix44List values;
	node->GenerateSampledMatrixAnimation(keys, values);
	FailIf(keys.size() > 30);
	PassIf(keys.size() == values.size());
}
