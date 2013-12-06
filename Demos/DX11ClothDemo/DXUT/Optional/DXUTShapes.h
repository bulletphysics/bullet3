//--------------------------------------------------------------------------------------
// File: DXUTShapes.h
//
// Shape creation functions for DXUT
//
// Copyright (c) Microsoft Corporation. All rights reserved
//--------------------------------------------------------------------------------------
#pragma once
#ifndef DXUT_SHAPES_H
#define DXUT_SHAPES_H
#include <d3d10.h>
#include <d3dx10.h>

HRESULT WINAPI DXUTCreateBox( ID3D10Device* pDevice, float fWidth, float fHeight, float fDepth, ID3DX10Mesh** ppMesh );
HRESULT WINAPI DXUTCreateCylinder( ID3D10Device* pDevice, float fRadius1, float fRadius2, float fLength, UINT uSlices,
                                   UINT uStacks, ID3DX10Mesh** ppMesh );
HRESULT WINAPI DXUTCreatePolygon( ID3D10Device* pDevice, float fLength, UINT uSides, ID3DX10Mesh** ppMesh );
HRESULT WINAPI DXUTCreateSphere( ID3D10Device* pDevice, float fRadius, UINT uSlices, UINT uStacks,
                                 ID3DX10Mesh** ppMesh );
HRESULT WINAPI DXUTCreateTorus( ID3D10Device* pDevice, float fInnerRadius, float fOuterRadius, UINT uSides,
                                UINT uRings, ID3DX10Mesh** ppMesh );
HRESULT WINAPI DXUTCreateTeapot( ID3D10Device* pDevice, ID3DX10Mesh** ppMesh );

#endif
