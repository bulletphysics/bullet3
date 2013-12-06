/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2010 Advanced Micro Devices

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef CLOTH_H
#define CLOTH_H

#include <fstream>
#include <iostream>
#include <iomanip>

class piece_of_cloth 
{
public:


	void destroy(void)
	{

		if(created)
		{
			SAFE_RELEASE(g_pIndexBuffer);
			SAFE_RELEASE(pVB[0]);
			SAFE_RELEASE(g_pVB_UAV);
			SAFE_RELEASE(pVB_staging);
			SAFE_RELEASE(texture2D_view);
			if(m_vertexBufferDescriptor) delete [] m_vertexBufferDescriptor;
			if(cpu_buffer) delete [] cpu_buffer;
		}
	}

	piece_of_cloth()
	{
		created = false;
		m_vertexBufferDescriptor = NULL;
		cpu_buffer = NULL;
	}
	bool created;

	ID3D11Buffer* g_pIndexBuffer;
	ID3D11Buffer* pVB[1];
	ID3D11Buffer* pVB_staging;

	float* cpu_buffer;

	ID3D11UnorderedAccessView* g_pVB_UAV;
	UINT Strides[1];
	UINT Offsets[1];

	double x_offset, y_offset, z_offset;


	ID3D11ShaderResourceView *texture2D_view;

	int width;
	int height;

	btVertexBufferDescriptor *m_vertexBufferDescriptor;

	void create_texture(std::wstring filename)
	{
		D3DX11_IMAGE_LOAD_INFO loadInfo;
		ZeroMemory(&loadInfo, sizeof(D3DX11_IMAGE_LOAD_INFO) );
		loadInfo.BindFlags = D3D11_BIND_SHADER_RESOURCE;
		loadInfo.Format = DXGI_FORMAT_BC1_UNORM;

		HRESULT hr = D3DX11CreateShaderResourceViewFromFile(g_pd3dDevice, filename.c_str(), &loadInfo, NULL, &texture2D_view, NULL);
		hr = hr;

	}


	void draw(void)
	{

		ID3D11DeviceContext* pd3dImmediateContext = DXUTGetD3D11DeviceContext();

		D3DXMATRIX mWorldViewProjection;
		D3DXVECTOR3 vLightDir;
		D3DXMATRIX mWorld;
		D3DXMATRIX mView;
		D3DXMATRIX mProj;

		// Get the projection & view matrix from the camera class
		mProj = *g_Camera.GetProjMatrix();
		mView = *g_Camera.GetViewMatrix();

		// Get the light direction
		vLightDir = g_LightControl.GetLightDirection();

		// Per frame cb update
		D3D11_MAPPED_SUBRESOURCE MappedResource;

		HRESULT hr;
#ifndef USE_GPU_COPY
		pd3dImmediateContext->Map( pVB_staging, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource );
		memcpy( MappedResource.pData, cpu_buffer, 4*width*height*8 );
		pd3dImmediateContext->Unmap(pVB_staging, 0 );
		pd3dImmediateContext->CopyResource(pVB[0], pVB_staging);
#endif // #ifndef USE_GPU_COPY

		V( pd3dImmediateContext->Map( g_pcbPSPerFrame, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource ) );
		CB_PS_PER_FRAME* pPerFrame = ( CB_PS_PER_FRAME* )MappedResource.pData;
		float fAmbient = 0.1f;
		pPerFrame->m_vLightDirAmbient = D3DXVECTOR4( vLightDir.x, vLightDir.y, vLightDir.z, fAmbient );
		pd3dImmediateContext->Unmap( g_pcbPSPerFrame, 0 );

		pd3dImmediateContext->PSSetConstantBuffers( g_iCBPSPerFrameBind, 1, &g_pcbPSPerFrame );


		///////////////////////////////////////Modify below//////////////////////////////////////////////////////

		//Get the mesh
		//IA setup
		pd3dImmediateContext->IASetInputLayout( g_pVertexLayout11 );

		//This is where we pass the vertex buffer to DX
		pd3dImmediateContext->IASetVertexBuffers( 0, 1, pVB, Strides, Offsets );

		//This is where we pass the index buffer to DX
		pd3dImmediateContext->IASetIndexBuffer( g_pIndexBuffer, DXGI_FORMAT_R32_UINT, 0 );

		/////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Set the shaders
		pd3dImmediateContext->VSSetShader( g_pVertexShader, NULL, 0 );
		pd3dImmediateContext->PSSetShader( g_pPixelShader, NULL, 0 );
		pd3dImmediateContext->GSSetShader( g_pGeometryShader, NULL, 0);

		// Set the per object constant data

		D3DXMATRIXA16 m_scale;
		float sc = 10;
		D3DXMatrixScaling(&m_scale,sc,sc,sc);


		D3DXVECTOR3 vCenter( global_shift_x, global_shift_y, global_shift_z);

		D3DXMatrixTranslation( &g_mCenterMesh, -vCenter.x+x_offset, -vCenter.y+y_offset, -vCenter.z+z_offset );


		mWorld = *g_Camera.GetWorldMatrix();
		mProj = *g_Camera.GetProjMatrix();
		mView = *g_Camera.GetViewMatrix();

		mWorldViewProjection = mView * mProj;


		// VS Per object
		V( pd3dImmediateContext->Map( g_pcbVSPerObject, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource ) );
		CB_VS_PER_OBJECT* pVSPerObject = ( CB_VS_PER_OBJECT* )MappedResource.pData;
		D3DXMatrixTranspose( &pVSPerObject->m_WorldViewProj, &mWorldViewProjection );
		D3DXMatrixTranspose( &pVSPerObject->m_World, &mWorld );
		pd3dImmediateContext->Unmap( g_pcbVSPerObject, 0 );

		pd3dImmediateContext->VSSetConstantBuffers( g_iCBVSPerObjectBind, 1, &g_pcbVSPerObject );

		// PS Per object
		V( pd3dImmediateContext->Map( g_pcbPSPerObject, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource ) );
		CB_PS_PER_OBJECT* pPSPerObject = ( CB_PS_PER_OBJECT* )MappedResource.pData;
		pPSPerObject->m_vObjectColor = D3DXVECTOR4( 1, 1, 1, 1 );
		pd3dImmediateContext->Unmap( g_pcbPSPerObject, 0 );

		pd3dImmediateContext->PSSetConstantBuffers( g_iCBPSPerObjectBind, 1, &g_pcbPSPerObject );

		//Render
		SDKMESH_SUBSET* pSubset = NULL;
		D3D11_PRIMITIVE_TOPOLOGY PrimType;

		if( g_wireFrame )
			pd3dImmediateContext->RSSetState(g_pRasterizerStateWF);
		else	
			pd3dImmediateContext->RSSetState(g_pRasterizerState);

		pd3dImmediateContext->PSSetSamplers( 0, 1, &g_pSamLinear );

		{
			// Get the subset
			pSubset = g_Mesh11.GetSubset( 0, 0 );

			pd3dImmediateContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

			pd3dImmediateContext->PSSetShaderResources(0,1,&texture2D_view);

			//pd3dImmediateContext->DrawIndexed( (width*3*2+2 + height*width*3*2), 0, ( UINT )pSubset->VertexStart );
			pd3dImmediateContext->DrawIndexed( ((height-1)*(width-1)*3*2), 0, ( UINT )pSubset->VertexStart );
		}

		SAFE_RELEASE(pd3dImmediateContext);
	}

	void create_buffers(int width_, int height_)
	{	    
		width = width_;
		height = height_;

		created = true;


		cpu_buffer = new float[width*height*8];
		memset(cpu_buffer,0,width*height*8*4);

		// Set texture coordinates in output buffers
		for(int y = 0; y < height; y++)
		{
			for(int x = 0; x < width; x++)
			{
				cpu_buffer[y*8*width + x*8 + 6] = x/( (float)(width-1));
				cpu_buffer[y*8*width + x*8 + 7] = 1-y/((float)(height-1));
			}
		}



		D3D11_BUFFER_DESC bufferDesc;
		ZeroMemory(&bufferDesc, sizeof(bufferDesc));
		bufferDesc.Usage = D3D11_USAGE_DEFAULT;
		bufferDesc.ByteWidth = sizeof(vertex_struct)*width*height;
		bufferDesc.StructureByteStride = 0;//sizeof(vertex_struct);
		bufferDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER | D3D11_BIND_UNORDERED_ACCESS;
		bufferDesc.MiscFlags = 0;//D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
		bufferDesc.CPUAccessFlags = 0;

		vertex_struct *vertices = new vertex_struct[width*height];
		for(int y = 0; y < height; y++)
		{
			for(int x = 0; x < width; x++)
			{
				double coord = sin(x/5.0)*50;
				//coord = sin(y/);

				vertices[y*width+x].Pos = D3DXVECTOR3( (x/((float)(width-1)))*1000, coord, (y/((float)(height-1)))*1000); 
				vertices[y*width+x].Normal = D3DXVECTOR3(1,0,0);
				vertices[y*width+x].Texcoord = D3DXVECTOR2(x/( (float)(width-1)), 1.f-y/((float)(height-1)));
			}
		}

		D3D11_SUBRESOURCE_DATA InitData;
		InitData.pSysMem = vertices;
		InitData.SysMemPitch = 0;
		InitData.SysMemSlicePitch = 0;

		HRESULT hr = g_pd3dDevice->CreateBuffer(&bufferDesc, &InitData, &pVB[0]);


		D3D11_UNORDERED_ACCESS_VIEW_DESC uavbuffer_desc;
		ZeroMemory(&uavbuffer_desc, sizeof(uavbuffer_desc));
		uavbuffer_desc.Format = DXGI_FORMAT_R32_FLOAT;
		uavbuffer_desc.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;

		uavbuffer_desc.Buffer.NumElements = width*height*sizeof(vertex_struct)/4;
		hr = g_pd3dDevice->CreateUnorderedAccessView(pVB[0], &uavbuffer_desc, &g_pVB_UAV);


		//What is this vertex stride thing all about?
		Strides[0] = ( UINT )g_Mesh11.GetVertexStride( 0, 0 );
		Offsets[0] = 0;


		//unsigned int indices[] = {0,1,2, 1,3,2};
		unsigned int* indices = new unsigned int[(height-1)*(width-1)*3*2];

		for(int y = 0; y < height-1; y++)
		{
			for(int x = 0; x < width-1; x++)
			{
				// *3 indices/triangle, *2 triangles/quad
				int baseIndex = (x + y*(width-1))*3*2;
				indices[baseIndex] = x + y*width;
				indices[baseIndex+1] = x+1 + y*width;
				indices[baseIndex+2] = x+width + y*width;


				indices[baseIndex+3] = x + 1 +  y*width;
				indices[baseIndex+4] = x+(width+1) + y*width;
				indices[baseIndex+5] = x+width + y*width;
			}
		}


		bufferDesc.ByteWidth = sizeof(unsigned int)*((height-1)*(width-1)*3*2);
		bufferDesc.BindFlags = D3D11_BIND_INDEX_BUFFER;

		InitData.pSysMem = indices;

		hr = g_pd3dDevice->CreateBuffer(&bufferDesc, &InitData, &g_pIndexBuffer);

		bufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
		bufferDesc.Usage = D3D11_USAGE_DYNAMIC;
		bufferDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
		bufferDesc.ByteWidth = sizeof(vertex_struct)*width*height;

		hr = g_pd3dDevice->CreateBuffer(&bufferDesc, NULL, &pVB_staging);	

		delete [] indices;
		delete [] vertices;
	}

};
#endif //CLOTH_H


