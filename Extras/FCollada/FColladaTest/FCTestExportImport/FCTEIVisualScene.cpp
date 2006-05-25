/*
Copyright (C) 2006 Feeling Software Inc.
Available only to licensees.
Distribution of this file or its content is strictly prohibited.
*/

#include "StdAfx.h"
#include "FCDocument/FCDSceneNode.h"
#include "FCDocument/FCDTransform.h"
#include "FCTestExportImport.h"

static const float sampleMatrix[16] = { 0.0f, 2.0f, 0.4f, 2.0f, 7.0f, 0.2f, 991.0f, 2.5f, 11.0f, 25.0f, 1.55f, 0.02f, 0.001f, 12.0f, 1.02e-3f };

namespace FCTestExportImport
{
	void FillVisualScene(FCDSceneNode* scene)
	{
		FCDSceneNode* child = scene->AddChildNode();
		FCDTRotation* rotation = (FCDTRotation*) child->AddTransform(FCDTransform::ROTATION);
		rotation->SetAxis(FMVector3::ZAxis);
		rotation->SetAngle(45.0f);
		FCDTTranslation* translation = (FCDTTranslation*) child->AddTransform(FCDTransform::TRANSLATION);
		translation->SetTranslation(0.0f, 4.0f, 6.0f);
		FCDTScale* scale = (FCDTScale*) child->AddTransform(FCDTransform::SCALE);
		scale->SetScale(FMVector3(3.0f, 0.5f, 2.0f));
		FCDTMatrix* matrix = (FCDTMatrix*) child->AddTransform(FCDTransform::MATRIX);
		matrix->SetTransform(FMMatrix44(sampleMatrix));
		FCDTLookAt* lookAt = (FCDTLookAt*) child->AddTransform(FCDTransform::LOOKAT);
		lookAt->SetPosition(1.0f, 2.0f, 3.0f);
		lookAt->SetTarget(5.0f, 6.0f, 9.0f);
		lookAt->SetUp(12.0f, 0.3f, 0.4f);
		FCDTSkew* skew = (FCDTSkew*) child->AddTransform(FCDTransform::SKEW);
		skew->SetAroundAxis(FMVector3::ZAxis);
		skew->SetRotateAxis(FMVector3::XAxis);
		skew->SetAngle(60.0f);
	}

	void CheckVisualScene(FCDSceneNode* imported)
	{
		// NOTE: the transforms must be in the same order as the exported order.

		PassIf(imported->GetChildrenCount() == 1);
		PassIf(imported->GetParent() == NULL);
		FCDSceneNode* child = imported->GetChild(0);
		FailIf(child == NULL);
		PassIf(child->GetParent() == imported);
		PassIf(child->GetTransformCount() == 6);

		FCDTransform* transform = child->GetTransform(0);
		FailIf(transform == NULL);
		FailIf(transform->GetParent() != child);
		PassIf(transform->GetType() == FCDTransform::ROTATION);
		FCDTRotation* rotation = (FCDTRotation*) transform;
		PassIf(IsEquivalent(rotation->GetAxis(), FMVector3::ZAxis));
		PassIf(IsEquivalent(rotation->GetAngle(), 45.0f));

		transform = child->GetTransform(1);
		FailIf(transform == NULL);
		FailIf(transform->GetParent() != child);
		PassIf(transform->GetType() == FCDTransform::TRANSLATION);
		FCDTTranslation* translation = (FCDTTranslation*) transform;
		PassIf(IsEquivalent(translation->GetTranslation(), FMVector3(0.0f, 4.0f, 6.0f)));

		transform = child->GetTransform(2);
		FailIf(transform == NULL);
		FailIf(transform->GetParent() != child);
		PassIf(transform->GetType() == FCDTransform::SCALE);
		FCDTScale* scale = (FCDTScale*) transform;
		PassIf(IsEquivalent(scale->GetScale(), FMVector3(3.0f, 0.5f, 2.0f)));

		transform = child->GetTransform(3);
		FailIf(transform == NULL);
		FailIf(transform->GetParent() != child);
		PassIf(transform->GetType() == FCDTransform::MATRIX);
		FCDTMatrix* mx = (FCDTMatrix*) transform;
		PassIf(IsEquivalent(mx->GetTransform(), FMMatrix44(sampleMatrix)));

		transform = child->GetTransform(4);
		FailIf(transform == NULL);
		FailIf(transform->GetParent() != child);
		PassIf(transform->GetType() == FCDTransform::LOOKAT);
		FCDTLookAt* lookAt = (FCDTLookAt*) transform;
		PassIf(IsEquivalent(lookAt->GetPosition(), FMVector3(1.0f, 2.0f, 3.0f)));
		PassIf(IsEquivalent(lookAt->GetTarget(), FMVector3(5.0f, 6.0f, 9.0f)));
		PassIf(IsEquivalent(lookAt->GetUp(), FMVector3(12.0f, 0.3f, 0.4f)));

		transform = child->GetTransform(5);
		FailIf(transform == NULL);
		FailIf(transform->GetParent() != child);
		PassIf(transform->GetType() == FCDTransform::SKEW);
		FCDTSkew* skew = (FCDTSkew*) transform;
		PassIf(IsEquivalent(skew->GetAroundAxis(), FMVector3::ZAxis));
		PassIf(IsEquivalent(skew->GetRotateAxis(), FMVector3::XAxis));
		PassIf(IsEquivalent(skew->GetAngle(), 60.0f));
	}
};