/*
	Copyright (C) 2006 Feeling Software Inc.
	Available only to licensees.
	Distribution of this file or its content is strictly prohibited.
*/

#include "StdAfx.h"
#include "FCDocument/FCDExtra.h"
#include "FCTestExportImport.h"

namespace FCTestExportImport
{
	void FillExtraTree(FCDExtra* extra)
	{
		FailIf(extra == NULL);

		// Add a test technique.
		PassIf(extra->GetTechniqueCount() == 0);
		FCDETechnique* technique1 = extra->AddTechnique("FCTEI_TestProfile");
		FCDETechnique* technique2 = extra->AddTechnique("FCTEI_TestProfile");
		FailIf(technique1 == NULL);
		FailIf(technique2 == NULL);
		PassIf(technique1 == technique2);
		PassIf(extra->GetTechniqueCount() == 1);

		// Add a parent parameter to the technique and two subsequent parameters with the same name.
		FCDENode* parameterTree = technique1->AddChildNode();
		parameterTree->SetName("MainParameterTree");
		FCDENode* firstParameter = parameterTree->AddChildNode();
		firstParameter->SetName("SomeParameter");
		firstParameter->SetContent(FS("Test_SomeParameter"));
		firstParameter->AddAttribute("Guts", 0);
		FCDENode* secondParameter = parameterTree->AddChildNode();
		secondParameter->SetName("SomeParameter");
		secondParameter->AddAttribute("Guts", 3);
		secondParameter->SetContent(FS("Test_ThatParameter!"));
		PassIf(parameterTree->GetChildNodeCount() == 2);

		// Add some attributes to the parameter tree
		parameterTree->AddAttribute("Vicious", FC("Squirrel"));
		parameterTree->AddAttribute("Gross", 1002);
	}

	void CheckExtraTree(FCDExtra* extra)
	{
		FailIf(extra == NULL);

		// Find and verify the one technique
		FailIf(extra->GetTechniqueCount() != 1);
		FCDETechnique* technique = extra->GetTechnique(0);
		FailIf(technique == NULL);
		PassIf(IsEquivalent(technique->GetProfile(), "FCTEI_TestProfile"));
		PassIf(extra->FindTechnique("FCTEI_TestProfile") == technique);

		// Find and verify the base parameter tree node
		FailIf(technique->GetChildNodeCount() != 1);
		FCDENode* baseNode = technique->GetChildNode(0);
		PassIf(baseNode != NULL);
		PassIf(extra->FindRootNode("MainParameterTree") == baseNode);

		// Verify the base node attributes
		PassIf(baseNode->GetAttributeCount() == 2);
		FCDEAttribute* a1 = baseNode->FindAttribute("Vicious");
		FCDEAttribute* a2 = baseNode->FindAttribute("Gross");
		FailIf(a1 == NULL);
		FailIf(a2 == NULL);
		FailIf(a1 == a2);
		PassIf(IsEquivalent(a1->value, FC("Squirrel")));
		PassIf(IsEquivalent(FUStringConversion::ToUInt32(a2->value), 1002));

		// Identify the base node leaves
		PassIf(baseNode->GetChildNodeCount() == 2);
		FCDENode* leaf0 = NULL,* leaf3 = NULL;
		for (size_t i = 0; i < 2; ++i)
		{
			FCDENode* leaf = baseNode->GetChildNode(i);
			PassIf(IsEquivalent(leaf->GetName(), "SomeParameter"));
			FCDEAttribute* guts = leaf->FindAttribute("Guts");
			FailIf(guts == NULL || guts->value.empty());
			uint32 gutsIndex = FUStringConversion::ToUInt32(guts->value);
			if (gutsIndex == 0) { FailIf(leaf0 != NULL); leaf0 = leaf; }
			else if (gutsIndex == 3) { FailIf(leaf3 != NULL); leaf3 = leaf; }
			else FailIf(true);
		}
		FailIf(leaf0 == NULL || leaf3 == NULL);

		// Verify the base node leaves
		PassIf(leaf0->GetChildNodeCount() == 0);
		PassIf(leaf3->GetChildNodeCount() == 0);
		PassIf(leaf0->GetAttributeCount() == 1);
		PassIf(leaf3->GetAttributeCount() == 1);
		PassIf(IsEquivalent(leaf0->GetContent(), FC("Test_SomeParameter")));
		PassIf(IsEquivalent(leaf3->GetContent(), FS("Test_ThatParameter!")));
	}
}