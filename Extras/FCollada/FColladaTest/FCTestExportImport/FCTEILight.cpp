/*
	Copyright (C) 2006 Feeling Software Inc.
	Available only to licensees.
	Distribution of this file or its content is strictly prohibited.
*/

#include "StdAfx.h"
#include "FCDocument/FCDLight.h"
#include "FCTestExportImport.h"

namespace FCTestExportImport
{
	void FillLightLibrary(FCDLightLibrary* library)
	{
		// Create four lights of different types.
		FCDLight* pointLight = library->AddEntity();
		pointLight->SetLightType(FCDLight::POINT);
		FCDLight* spotLight = library->AddEntity();
		spotLight->SetLightType(FCDLight::SPOT);
		FCDLight* directionalLight = library->AddEntity();
		directionalLight->SetLightType(FCDLight::DIRECTIONAL);
		FCDLight* ambientLight = library->AddEntity();
		ambientLight->SetLightType(FCDLight::AMBIENT);

		// Set the base colors
		pointLight->SetColor(FMVector3(0.5f, 0.2f, 0.7f));
		spotLight->SetColor(FMVector3(0.25f, 0.25f, 0.75f));
		directionalLight->SetColor(FMVector3(0.1f, 0.0f, -1.0f));
		ambientLight->SetColor(FMVector3(5.0f, 0.0f, 0.4f));

		// Set some intensity
		pointLight->SetIntensity(2.5f);
		spotLight->SetIntensity(1.5f);
		directionalLight->SetIntensity(0.5f);
		ambientLight->SetIntensity(-2.5f);
		
		// Test the extra tree:
		FillExtraTree(ambientLight->GetExtra());
	}

	void CheckLightLibrary(FCDLightLibrary* library)
	{
		// Verify that the library contains four lights and one of each type.
		FCDLight* pointLight = NULL,* spotLight = NULL,* directionalLight = NULL,* ambientLight = NULL;
		for (size_t i = 0; i < library->GetEntityCount(); ++i)
		{
			FCDLight* light = library->GetEntity(i);
			switch (light->GetLightType())
			{
			case FCDLight::AMBIENT: FailIf(ambientLight != NULL); ambientLight = light; break;
			case FCDLight::DIRECTIONAL: FailIf(directionalLight != NULL); directionalLight = light; break;
			case FCDLight::POINT: FailIf(pointLight != NULL); pointLight = light; break;
			case FCDLight::SPOT: FailIf(spotLight != NULL); spotLight = light; break;
			default: FailIf(true); break;
			}
		}
		PassIf(ambientLight != NULL && spotLight != NULL && directionalLight != NULL && pointLight != NULL);

		// Verify the base colors
		PassIf(IsEquivalent(pointLight->GetColor(), FMVector3(0.5f, 0.2f, 0.7f)));
		PassIf(IsEquivalent(spotLight->GetColor(), FMVector3(0.25f, 0.25f, 0.75f)));
		PassIf(IsEquivalent(directionalLight->GetColor(), FMVector3(0.1f, 0.0f, -1.0f)));
		PassIf(IsEquivalent(ambientLight->GetColor(), FMVector3(5.0f, 0.0f, 0.4f)));

		// Verify the intensities
		PassIf(IsEquivalent(pointLight->GetIntensity(), 2.5f));
		PassIf(IsEquivalent(spotLight->GetIntensity(), 1.5f));
		PassIf(IsEquivalent(directionalLight->GetIntensity(), 0.5f));
		PassIf(IsEquivalent(ambientLight->GetIntensity(), -2.5f));
		
		// Test the extra tree
		CheckExtraTree(ambientLight->GetExtra());
	}
};