/*
	Copyright (C) 2006 Feeling Software Inc.
	Available only to licensees.
	Distribution of this file or its content is strictly prohibited.
*/

#include "StdAfx.h"
#include "FCDocument/FCDEffect.h"
#include "FCDocument/FCDEffectStandard.h"
#include "FCDocument/FCDEffectProfileFX.h"
#include "FCDocument/FCDImage.h"
#include "FCDocument/FCDMaterial.h"
#include "FCDocument/FCDMaterialLibrary.h"
#include "FCDocument/FCDTexture.h"
#include "FCTestExportImport.h"

static const float sampleMatrix[16] = { 0.5f, 0.1f, 0.7f, 2.0f, 1.11f, 0.5e-2f, 111.0f, 0.5f, 0.0f, 0.0f, 0.557f, -10.02f, 0.001f, 12.0f, 1.02e-3f };
static string wantedImageId = "test_image";
static string wantedImage2Id = "test_image";

namespace FCTestExportImport
{
	void FillMaterialLibrary(FCDMaterialLibrary* library)
	{
		// Create an empty material
		FCDMaterial* material = library->AddMaterial();
		material->SetNote(FS("EmptyMaterial244"));

		// Create an effect and attach it to a new material
		FCDEffect* effect = library->AddEffect();
		material = library->AddMaterial();
		material->SetEffect(effect);
		effect->SetNote(FS("EmptyEffect.. for now!"));

		FillEffectStandard((FCDEffectStandard*) effect->AddProfile(FUDaeProfileType::COMMON));
		FillEffectFX((FCDEffectProfileFX*) effect->AddProfile(FUDaeProfileType::CG));
	}

	void FillEffectStandard(FCDEffectStandard* profile)
	{
		FailIf(profile == NULL);
		profile->SetLightingType(FCDEffectStandard::PHONG);
		profile->SetDiffuseColor(FMVector3(1.0f, 0.0f, -2.0f));
		profile->SetSpecularColor(FMVector3(0.0f, 1.0f, 0.4f));
		profile->SetShininess(40.0f);

		// Retrieve two images created earlier
		FCDImage* image1 = profile->GetDocument()->FindImage(wantedImageId);
		FCDImage* image2 = profile->GetDocument()->FindImage(wantedImage2Id);
		PassIf("Dependency: FillImageLibrary!" && image1 != NULL && image2 != NULL);

		// The first bump texture should have placement parameters.
		FCDTexture* texture1 = profile->AddTexture(FUDaeTextureChannel::BUMP);
		texture1->SetImage(image2);
		texture1->SetWrapU(true);
		texture1->SetWrapV(true);
		texture1->SetRepeatU(2.0f);
		texture1->SetRepeatV(2.0f);
		texture1->SetTranslateFrameU(5.0f);
		texture1->SetTranslateFrameV(-0.4f);

		// The second bump texture should have an image and projection parameters.
		FCDTexture* texture2 = profile->AddTexture(FUDaeTextureChannel::BUMP);
		texture2->SetImage(image1);
		texture2->SetProjectionMatrix(FMMatrix44(sampleMatrix));

		// The third texture is a filter texture and will remain empty.
		UNUSED(FCDTexture* texture3 = )profile->AddTexture(FUDaeTextureChannel::FILTER);
	}

	void FillEffectFX(FCDEffectProfileFX* profile)
	{
		FailIf(profile == NULL);
		profile->AddTechnique();
		profile->AddCode();
	}

	void CheckMaterialLibrary(FCDMaterialLibrary* library)
	{
		// There should be two materials within the material library: one is empty, the other is not.
		PassIf(library->GetMaterialCount() == 2);
		PassIf(library->GetEffectCount() == 1);

		FCDMaterial* emptyMaterial = NULL,* material = NULL;
		for (size_t i = 0; i < library->GetMaterialCount(); ++i)
		{
			FCDMaterial* m = library->GetMaterial(i);
			if (m->GetEffect() == NULL) { PassIf(emptyMaterial == NULL); emptyMaterial = m; }
			else { PassIf(material == NULL); material = m; }
		}
		PassIf(emptyMaterial != NULL && material != NULL);

		// Verify the empty material. It should only have a note.
		PassIf(emptyMaterial->GetNote() == FC("EmptyMaterial244"));

		// Verify the other material and its effect.
		FCDEffect* effect = material->GetEffect();
		PassIf(library->GetEffect(0) == effect);
		PassIf(effect->GetNote() == FC("EmptyEffect.. for now!"));
		PassIf(effect->GetProfileCount() == 2);

		CheckEffectStandard((FCDEffectStandard*) effect->FindProfile(FUDaeProfileType::COMMON));
		CheckEffectFX((FCDEffectProfileFX*) effect->FindProfile(FUDaeProfileType::CG));
	}

	void CheckEffectStandard(FCDEffectStandard* profile)
	{
		FailIf(profile == NULL);
		PassIf(profile->GetLightingType() == FCDEffectStandard::PHONG);
		PassIf(IsEquivalent(profile->GetDiffuseColor(), FMVector3(1.0f, 0.0f, -2.0f)));
		PassIf(IsEquivalent(profile->GetSpecularColor(), FMVector3(0.0f, 1.0f, 0.4f)));
		PassIf(IsEquivalent(profile->GetShininess(), 40.0f));

		// There should be two textures in the bump channel.
		PassIf(profile->GetTextureCount(FUDaeTextureChannel::BUMP) == 2);
		FCDTexture* texture1 = NULL,* texture2 = NULL;
		for (size_t i = 0; i < 2; ++i)
		{
			FCDTexture* texture = profile->GetTexture(FUDaeTextureChannel::BUMP, i);
			FailIf(texture == NULL);
			if (texture->HasPlacement2D()) { FailIf(texture1 != NULL); texture1 = texture; }
			else if (texture->HasProjection3D()) { FailIf(texture2 != NULL); texture2 = texture; }
			else FailIf(true);
		}
		PassIf(texture1 != NULL && texture2 != NULL);

		// Verify the texture images
		FCDImage* image1 = profile->GetDocument()->FindImage(wantedImageId);
		FCDImage* image2 = profile->GetDocument()->FindImage(wantedImage2Id);
		PassIf("Dependency: CheckImageLibrary" && image1 != NULL && image2 != NULL);
		PassIf(texture1->GetImage() == image2);
		PassIf(texture2->GetImage() == image1);

		// Verify the placement parameters
		PassIf(IsEquivalent(texture1->GetWrapU(), 1.0f));
		PassIf(IsEquivalent(texture1->GetWrapV(), 1.0f));
		PassIf(IsEquivalent(texture1->GetRepeatU(), 2.0f));
		PassIf(IsEquivalent(texture1->GetRepeatV(), 2.0f));
		PassIf(IsEquivalent(texture1->GetTranslateFrameU(), 5.0f));
		PassIf(IsEquivalent(texture1->GetTranslateFrameV(), -0.4f));

		// There should be an empty texture in the filter channel
		PassIf(profile->GetTextureCount(FUDaeTextureChannel::FILTER) == 1);
		FCDTexture* texture3 = profile->GetTexture(FUDaeTextureChannel::FILTER, 0);
		FailIf(texture3 == NULL || texture3->GetImage() != NULL);
	}

	void CheckEffectFX(FCDEffectProfileFX* profile)
	{
		FailIf(profile == NULL);
		PassIf(profile->GetTechniqueCount() == 1);
		PassIf(profile->GetCodeCount() == 1);
	}

	void FillImageLibrary(FCDImageLibrary* library)
	{
		FailIf(library == NULL || library->GetEntityCount() > 0);
		FCDImage* image1 = library->AddEntity();
		FCDImage* image2 = library->AddEntity();
		FCDImage* image3 = library->AddEntity();

		image1->SetDaeId(wantedImageId);
		image1->SetFilename(FS("Texture1.jpg"));
		image2->SetDaeId(wantedImage2Id);
		image2->SetFilename(FC("Texture3D.jpg"));
		image2->SetWidth(256);
		image2->SetHeight(135);
		image3->SetWidth(33);
		image3->SetDepth(521);

		FailIf(image1->GetDaeId() == image2->GetDaeId());
	}

	void CheckImageLibrary(FCDImageLibrary* library)
	{
		FailIf(library == NULL || library->GetEntityCount() != 3);

		// Retrieve the three images, verify that they match the id/filenames that we created.
		FCDImage* image1 = NULL,* image2 = NULL,* image3 = NULL;
		for (size_t i = 0; i < 3; ++i)
		{
			FCDImage* image = library->GetEntity(i);
			if (IsEquivalent(image->GetDaeId(), wantedImageId)) { FailIf(image1 != NULL); image1 = image; }
			else if (IsEquivalent(image->GetDaeId(), wantedImage2Id)) { FailIf(image2 != NULL); image2 = image; }
			else { FailIf(image3 != NULL); image3 = image; }
		}
		PassIf(image1 != NULL && image2 != NULL && image3 != NULL);

		// Verify the depth/width/height.
		PassIf(image1->GetWidth() == 0 && image1->GetHeight() == 0 && image1->GetDepth() == 0);
		PassIf(image2->GetWidth() == 256 && image2->GetHeight() == 135 && image2->GetDepth() == 0);
		PassIf(image3->GetWidth() == 33 && image3->GetHeight() == 0 && image3->GetDepth() == 521);

		// Verify the filenames. They should be absolute filenames now, so look for the wanted substrings.
		PassIf(strstr(TO_STRING(image1->GetFilename()).c_str(), "Texture1.jpg") != NULL);
		PassIf(strstr(TO_STRING(image2->GetFilename()).c_str(), "Texture3D.jpg") != NULL);
	}
};