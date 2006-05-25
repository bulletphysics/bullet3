/*
	Copyright (C) 2006 Feeling Software Inc.
	Available only to licensees.
	Distribution of this file or its content is strictly prohibited.
*/

#include "StdAfx.h"
#include "FCDocument/FCDController.h"
#include "FCDocument/FCDGeometry.h"
#include "FCDocument/FCDGeometryMesh.h"
#include "FCDocument/FCDGeometryPolygons.h"
#include "FCDocument/FCDGeometrySource.h"
#include "FCDocument/FCDGeometrySpline.h"
#include "FCDocument/FCDMorphController.h"
#include "FCDocument/FCDSceneNode.h"
#include "FCDocument/FCDSkinController.h"
#include "FCTestExportImport.h"

static const float positionData[12] = { 0.0f, 0.0f, 3.0f, 5.0f, 0.0f, -2.0f, -3.0f, 4.0f, -2.0f, -3.0f, -4.0f, -2.0f };
static const float colorData[12] = { 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f };
static const float dummyData[10] = { 0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f };
static const uint32 positionIndices[12] = { 0, 1, 2, 0, 2, 3, 0, 3, 1, 3, 2, 1 };
static const uint32 colorIndices[12] = { 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2 };

static const float sampleBindPose1[16] = { 1.0f, 0.4f, 0.4f, 0.0f, 7.77f, 0.0f, 0.3f, 2.5f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
static const float sampleBindPose2[16] = { 0.3f, 0.0f, -0.3f, -21.0f, 0.96f, 0.0f, 2.0f, 2.5f, 0.0f, -5.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
static string splineId, meshId, jointId1, jointId2;

namespace FCTestExportImport
{
	void FillGeometryLibrary(FCDGeometryLibrary* library)
	{
		FCDGeometry* geometry = library->AddEntity();
		PassIf(geometry->GetType() == FCDEntity::GEOMETRY);
		FailIf(geometry->IsMesh());
		FailIf(geometry->IsSpline());
		meshId = geometry->GetDaeId();
		FailIf(meshId.empty());

		// Creates a mesh to export
		FCDGeometryMesh* mesh = geometry->CreateMesh();
		FailIf(geometry->IsSpline());
		PassIf(geometry->IsMesh());
		PassIf(geometry->GetMesh() == mesh);
		PassIf(geometry->GetSpline() == NULL);
		FillGeometryMesh(mesh);

		// Create a spline to export
		geometry = library->AddEntity();
		geometry->CreateMesh();
		FCDGeometrySpline* spline = geometry->CreateSpline();
		PassIf(geometry->IsSpline());
		FailIf(geometry->IsMesh());
		PassIf(geometry->GetMesh() == NULL);
		PassIf(geometry->GetSpline() == spline);
		FillGeometrySpline(spline);
		splineId = geometry->GetDaeId();
		FailIf(splineId.empty());
	}

	void FillGeometryMesh(FCDGeometryMesh* mesh)
	{
		FCDGeometrySource* posSource = mesh->AddVertexSource();
		FailIf(posSource == NULL);
		posSource->SetName(FC("TestPositionSource"));
		posSource->SetSourceType(FUDaeGeometryInput::POSITION);
		posSource->SetSourceData(FloatList(positionData, 12), 3);

		FCDGeometrySource* colorSource = mesh->AddSource();
		FailIf(colorSource == NULL);
		colorSource->SetName(FC("TestColorSource"));
		colorSource->SetSourceType(FUDaeGeometryInput::COLOR);
		colorSource->SetSourceData(FloatList(colorData, 12), 4);

		FCDGeometrySource* dummySource = mesh->AddSource();
		FailIf(dummySource == NULL);
		dummySource->SetName(FC("TestDummySource"));
		dummySource->SetSourceType(FUDaeGeometryInput::EXTRA);
		dummySource->SetSourceData(FloatList(dummyData, 10), 3);

		FCDGeometryPolygons* polys1 = mesh->AddPolygons();
		FailIf(polys1 == NULL || polys1->GetInputCount() != 1);
		FCDGeometryPolygons* polys2 = mesh->AddPolygons();
		FailIf(polys2 == NULL || polys2->GetInputCount() != 1);
		FCDGeometryPolygonsInput* pInput1 = polys1->AddInput(colorSource, 1);
		PassIf(polys1->GetInputCount() == 2);
		PassIf(pInput1 != NULL && pInput1->source == colorSource && pInput1->idx == 1);
		FCDGeometryPolygonsInput* pInput2 = polys2->AddInput(colorSource, 1);
		PassIf(polys2->GetInputCount() == 2);
		PassIf(pInput2 != NULL && pInput2->source == colorSource && pInput2->idx == 1);
		FCDGeometryPolygonsInput* pInput3 = polys1->AddInput(dummySource, 2);
		PassIf(pInput3 != NULL && pInput3->source == dummySource && pInput3->idx == 2);

		// Fill in some indices in order to form a tetrahedron
		polys1->AddFace(3); polys1->AddFace(3); polys1->AddFace(3); polys1->AddFace(3);
		UInt32List* posIndices = polys1->FindIndices(posSource);
		UInt32List* colIndices = polys1->FindIndices(colorSource);
		FailIf(posIndices == NULL || posIndices->size() != 12);
		FailIf(colIndices == NULL || colIndices == posIndices || colIndices->size() != 12);
		*posIndices = UInt32List(positionIndices, 12);
		*colIndices = UInt32List(colorIndices, 12);
	}

	void FillGeometrySpline(FCDGeometrySpline* spline)
	{
		FCDCVs cvs;
		cvs.push_back(FMVector3(1.0f, 2.0f, 4.0f));
		cvs.push_back(FMVector3::XAxis);
		spline->SetCVs(cvs);
		PassIf(spline->GetCVCount() == 2);

		FCDKnots knots;
		knots.push_back(1.4);
		knots.push_back(7.4);
		spline->SetKnots(knots);
		PassIf(spline->GetKnotCount() == 2);
	}

	void CheckGeometryLibrary(FCDGeometryLibrary* library)
	{
		PassIf(library->GetEntityCount() == 2);

		// Find the one mesh and the one spline geometries.
		FCDGeometryMesh* mesh = NULL; FCDGeometrySpline* spline = NULL;
		for (size_t i = 0; i < 2; ++i)
		{
			FCDGeometry* g = library->GetEntity(i);
			if (g->IsMesh()) mesh = g->GetMesh();
			else if (g->IsSpline()) spline = g->GetSpline();
			else FailIf(true);
		}
		FailIf(mesh == NULL || spline == NULL);

		CheckGeometryMesh(mesh);
		CheckGeometrySpline(spline);
	}

	void CheckGeometryMesh(FCDGeometryMesh* mesh)
	{
		// Verify the mesh and its sources
		PassIf(mesh->GetSourceCount() == 3);
		FCDGeometrySource* posSource = NULL,* colorSource = NULL,* dummySource = NULL;
		for (size_t i = 0; i < 3; ++i)
		{
			FCDGeometrySource* source = mesh->GetSource(i);
			FailIf(source == NULL);
			switch (source->GetSourceType())
			{
			case FUDaeGeometryInput::POSITION: posSource = source; PassIf(source->GetName() == FC("TestPositionSource")); break;
			case FUDaeGeometryInput::COLOR: colorSource = source; PassIf(source->GetName() == FC("TestColorSource")); break;
			case FUDaeGeometryInput::EXTRA: dummySource = source; PassIf(source->GetName() == FC("TestDummySource")); break;
			default: FailIf(true); break;
			}
		}
		FailIf(posSource == NULL || colorSource == NULL || dummySource == NULL);
		PassIf(IsEquivalent(posSource->GetSourceData(), positionData, 12));
		PassIf(posSource->GetSourceStride() == 3);
		PassIf(IsEquivalent(colorSource->GetSourceData(), colorData, 12));
		PassIf(colorSource->GetSourceStride() == 4);
		PassIf(IsEquivalent(dummySource->GetSourceData(), dummyData, 10));
		PassIf(dummySource->GetSourceStride() == 3);

		// Find the non-empty polygon set and verify that one of the polygon set is, in fact, empty.
		FCDGeometryPolygons* polys1 = NULL,* polysEmpty = NULL;
		for (size_t i = 0; i < mesh->GetPolygonsCount(); ++i)
		{
			FCDGeometryPolygons* p = mesh->GetPolygons(i);
			if (p->GetFaceCount() == 0) { PassIf(polysEmpty == NULL); polysEmpty = p; }
			else { PassIf(polys1 == NULL); polys1 = p; }
		}
		PassIf(polys1 != NULL && polysEmpty != NULL);

		// Check that we have the wanted tetrahedron in the non-empty polygon set.
		PassIf(polys1->GetFaceCount() == 4);
		PassIf(polys1->GetHoleCount() == 0);
		PassIf(polys1->GetFaceVertexCount(0) == 3 && polys1->GetFaceVertexCount(1) == 3 && polys1->GetFaceVertexCount(2) == 3 && polys1->GetFaceVertexCount(3) == 3);
		UInt32List* posIndices = polys1->FindIndices(posSource);
		UInt32List* colIndices = polys1->FindIndices(colorSource);
		FailIf(posIndices == NULL || posIndices->size() != 12);
		FailIf(colIndices == NULL || colIndices == posIndices || colIndices->size() != 12);
		PassIf(IsEquivalent(*posIndices, positionIndices, 12));
		PassIf(IsEquivalent(*colIndices, colorIndices, 12));
	}

	void CheckGeometrySpline(FCDGeometrySpline* spline)
	{
		// Verify the spline and its data
		FailIf(spline->GetCVCount() != 2);
		FailIf(spline->GetKnotCount() != 2);
		PassIf(IsEquivalent(*spline->GetCV(0), FMVector3(1.0f, 2.0f, 4.0f)));
		PassIf(IsEquivalent(*spline->GetCV(1), FMVector3::XAxis));
		PassIf(IsEquivalent(spline->GetKnot(0), 1.4));
		PassIf(IsEquivalent(spline->GetKnot(1), 7.4));
	}

	void FillControllerLibrary(FCDControllerLibrary* library)
	{
		FailIf(library == NULL);

		// Create a morph controller on the previously created spline.
		FCDController* morpher = library->AddEntity();
		FillControllerMorph(morpher->CreateMorphController());

		// Create a skin controller on the previously created mesh.
		FCDController* skin = library->AddEntity();
		skin->SetNote(FS("A nicey skinny controller. "));
		FillControllerSkin(skin->CreateSkinController());
	}

	void FillControllerMorph(FCDMorphController* controller)
	{
		FailIf(controller == NULL);
		controller->SetMethod(FUDaeMorphMethod::RELATIVE);

		// Retrieve the base spline entity that will be morphed
		// (and for this test only: it'll be the morph targets)
		// Also retrieve the mesh, for similarity checks.
		FCDGeometry* spline = controller->GetDocument()->FindGeometry(splineId);
		FailIf(spline == NULL);
		FCDGeometry* mesh = controller->GetDocument()->FindGeometry(meshId);
		FailIf(mesh == NULL);
		FailIf(controller->IsSimilar(mesh));
		FailIf(controller->IsSimilar(spline));

		controller->SetBaseTarget(spline);
		PassIf(controller->IsSimilar(spline));
		FailIf(controller->IsSimilar(mesh));

		controller->AddTarget(spline, 0.3f);
		controller->AddTarget(spline, 0.6f);
		PassIf(controller->GetTargetCount() == 2);
	}

	void FillControllerSkin(FCDSkinController* controller)
	{
		FailIf(controller == NULL);

		// Create two joints.
		FCDSceneNode* visualScene = controller->GetDocument()->GetVisualSceneLibrary()->AddEntity();
		PassIf(visualScene != NULL);
		FCDSceneNode* joint1 = visualScene->AddChildNode();
		PassIf(joint1 != NULL);
		FCDSceneNode* joint2 = joint1->AddChildNode();
		PassIf(joint2 != NULL);
		jointId1 = joint1->GetDaeId();
		jointId2 = joint2->GetDaeId();
		FailIf(jointId1.empty() || jointId2.empty());
		controller->AddJoint(joint1, FMMatrix44::Identity);
		controller->AddJoint(joint2, FMMatrix44(sampleBindPose1));
		controller->SetBindShapeTransform(FMMatrix44(sampleBindPose2));
		PassIf(controller->GetJointCount() == 2);

		// Retrieve and assign the base target
		FCDGeometry* geometricTarget = controller->GetDocument()->FindGeometry(meshId);
		controller->SetTarget(geometricTarget);

		// Set some influences
		PassIf(controller->GetVertexInfluenceCount()  == 4);
		FCDJointWeightPairList* influence = controller->GetInfluences(0);
		FailIf(influence == NULL);
		influence->push_back(FCDJointWeightPair(0, 0.5f));
		influence->push_back(FCDJointWeightPair(1, 0.5f));

		influence = controller->GetInfluences(3);
		FailIf(influence == NULL);
		influence->push_back(FCDJointWeightPair(1, 1.0f));

		influence = controller->GetInfluences(2);
		FailIf(influence == NULL);
		influence->push_back(FCDJointWeightPair(0, 0.1f));
	}

	void CheckControllerLibrary(FCDControllerLibrary* library)
	{
		FailIf(library == NULL);

		FCDController* morpher = NULL,* skin = NULL;
		for (size_t i = 0; i < library->GetEntityCount(); ++i)
		{
			FCDController* c = library->GetEntity(i);
			if (c->HasMorphController()) { PassIf(morpher == NULL); morpher = c; }
			else if (c->HasSkinController()) { PassIf(skin == NULL); skin = c; }
			else FailIf(true);
		}
		PassIf(morpher != NULL && skin != NULL);

		// Check the morpher
		FailIf(morpher->HasSkinController());
		PassIf(morpher->GetMorphController() != NULL);
		CheckControllerMorph(morpher->GetMorphController());

		// Check the skin
		PassIf(skin->GetSkinController() != NULL);
		FailIf(skin->HasMorphController());
		PassIf(skin->GetNote() == FC("A nicey skinny controller. "));
		CheckControllerSkin(skin->GetSkinController());
	}

	void CheckControllerMorph(FCDMorphController* controller)
	{
		FailIf(controller == NULL);
		PassIf(controller->GetMethod() == FUDaeMorphMethod::RELATIVE);

		// Check that there are two targets, that the weights are correct, as well as the ids.
		PassIf(controller->GetTargetCount() == 2);
		PassIf(controller->GetBaseTarget() != NULL);
		PassIf(controller->GetBaseTarget()->GetDaeId() == splineId);

		FCDMorphTarget* target1 = controller->GetTarget(0);
		FailIf(target1 == NULL);
		FCDMorphTarget* target2 = controller->GetTarget(1);
		FailIf(target2 == NULL);
		PassIf(target1->GetGeometry() == controller->GetBaseTarget());
		PassIf(target2->GetGeometry() == controller->GetBaseTarget());
		PassIf(IsEquivalent(target1->GetWeight(), 0.6f) || IsEquivalent(target1->GetWeight(), 0.3f));
		PassIf(IsEquivalent(target2->GetWeight(), 0.6f) || IsEquivalent(target2->GetWeight(), 0.3f));
		FailIf(IsEquivalent(target1->GetWeight(), target2->GetWeight()));
	}

	void CheckControllerSkin(FCDSkinController* controller)
	{
		FailIf(controller == NULL);

		// Check the base target's identity
		FailIf(controller->GetTarget() == NULL);
		PassIf(controller->GetTarget()->GetType() == FCDEntity::GEOMETRY);
		PassIf(controller->GetTarget()->GetDaeId() == meshId);

		// Retrieve the two joints and verify their ids/bind-pose.
		PassIf(controller->GetJointCount() == 2);
		FCDJointMatrixPair* joint1 = NULL,* joint2 = NULL;
		for (size_t i = 0; i < 2; ++i)
		{
			FCDJointMatrixPair* p = controller->GetJoint(i);
			FailIf(p->joint == NULL);
			if (p->joint->GetDaeId() == jointId1) { FailIf(joint1 != NULL); joint1 = p; }
			else if (p->joint->GetDaeId() == jointId2) { FailIf(joint2 != NULL); joint2 = p; }
			else FailIf(true);
		}
		FailIf(joint1 == NULL || joint2 == NULL);
		PassIf(IsEquivalent(joint1->invertedBindPose, FMMatrix44::Identity));
		PassIf(IsEquivalent(joint2->invertedBindPose, FMMatrix44(sampleBindPose1).Inverted()));

		// Verify the influences
		PassIf(controller->GetVertexInfluenceCount() == 4);
		FCDJointWeightPairList* influence = controller->GetInfluences(0);
		FailIf(influence == NULL);
		PassIf(influence->size() == 2);
		PassIf(IsEquivalent(influence->at(0).jointIndex, 0));
		PassIf(IsEquivalent(influence->at(0).weight, 0.5f));
		PassIf(IsEquivalent(influence->at(1).jointIndex, 1));
		PassIf(IsEquivalent(influence->at(1).weight, 0.5f));

		influence = controller->GetInfluences(1);
		FailIf(influence == NULL);
		PassIf(influence->empty());

		influence = controller->GetInfluences(2);
		FailIf(influence == NULL);
		PassIf(influence->size() == 1);
		PassIf(IsEquivalent(influence->at(0).jointIndex, 0));
		PassIf(IsEquivalent(influence->at(0).weight, 1.0f)); // the weight should have been normalized.

		influence = controller->GetInfluences(3);
		FailIf(influence == NULL);
		PassIf(influence->size() == 1);
		PassIf(IsEquivalent(influence->at(0).jointIndex, 1));
		PassIf(IsEquivalent(influence->at(0).weight, 1.0f));
	}
};