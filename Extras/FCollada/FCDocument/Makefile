#!/usr/bin/make
# FCDocument Makefile for PS3

include ../../MakeDefs

LIBNAME = libfcdocument

prefix=..

LIBPATH=$(prefix)/lib

PPU_INCDIRS=-I.. -I../LibXML/include
PPU_OPTIMIZE_LV= -O3 -funroll-loops

PPU_LIB_TARGET=$(LIBNAME).a
PPU_SRCS = \
	FCDAnimated.cpp \
	FCDAnimation.cpp \
	FCDAnimationChannel.cpp \
	FCDAnimationClip.cpp \
	FCDAnimationCurve.cpp \
	FCDAnimationMultiCurve.cpp \
	FCDAsset.cpp \
	FCDCamera.cpp \
	FCDController.cpp \
	FCDEffect.cpp \
	FCDEffectCode.cpp \
	FCDEffectParameter.cpp \
	FCDEffectParameterFactory.cpp \
	FCDEffectParameterList.cpp \
	FCDEffectPass.cpp \
	FCDEffectPassShader.cpp \
	FCDEffectProfile.cpp \
	FCDEffectStandard.cpp \
	FCDEffectTechnique.cpp \
	FCDEntity.cpp \
	FCDExtra.cpp \
	FCDGeometry.cpp \
	FCDGeometryInstance.cpp \
	FCDGeometryMesh.cpp \
	FCDGeometryPolygons.cpp \
	FCDGeometrySource.cpp \
	FCDGeometrySpline.cpp \
	FCDImage.cpp \
	FCDLight.cpp \
	FCDMaterial.cpp \
	FCDMaterialInstance.cpp \
	FCDMaterialLibrary.cpp \
	FCDMorphController.cpp \
	FCDocument.cpp \
	FCDPhysicsAnalyticalGeometry.cpp \
	FCDPhysicsMaterial.cpp \
	FCDPhysicsModel.cpp \
	FCDPhysicsModelInstance.cpp \
	FCDPhysicsParameterGeneric.cpp \
	FCDPhysicsRigidBody.cpp \
	FCDPhysicsRigidBodyInstance.cpp \
	FCDPhysicsRigidConstraint.cpp \
	FCDPhysicsRigidConstraintInstance.cpp \
	FCDPhysicsSceneNode.cpp \
	FCDPhysicsShape.cpp \
	FCDSceneNode.cpp \
	FCDSkinController.cpp \
	FCDTargetedEntity.cpp \
	FCDTexture.cpp \
	FCDTransform.cpp

include ../../MakeRules
