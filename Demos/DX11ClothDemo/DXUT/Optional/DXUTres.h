//----------------------------------------------------------------------------
// File: dxutres.h
//
// Functions to create DXUT media from arrays in memory 
//
// Copyright (c) Microsoft Corp. All rights reserved.
//-----------------------------------------------------------------------------
#pragma once
#ifndef DXUT_RES_H
#define DXUT_RES_H

HRESULT WINAPI DXUTCreateGUITextureFromInternalArray9( LPDIRECT3DDEVICE9 pd3dDevice, IDirect3DTexture9** ppTexture,
                                                       D3DXIMAGE_INFO* pInfo );
HRESULT WINAPI DXUTCreateGUITextureFromInternalArray11( ID3D11Device* pd3dDevice, ID3D11Texture2D** ppTexture,
                                                        D3DX11_IMAGE_INFO* pInfo );
HRESULT WINAPI DXUTCreateArrowMeshFromInternalArray( LPDIRECT3DDEVICE9 pd3dDevice, ID3DXMesh** ppMesh );

#endif
