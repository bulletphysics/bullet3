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

#ifndef CAP_H
#define CAP_H

class cap 
{
	public:

	ID3D11Buffer* g_pIndexBuffer;
	ID3D11Buffer* pVB[1];
	UINT Strides[1];
	UINT Offsets[1];

	double x_offset, y_offset, z_offset;
	
	int width,height;
	bool top;

	ID3D11Texture2D *texture2D;
	ID3D11ShaderResourceView *texture2D_view;
	btCollisionShape *collisionShape;
	btCollisionObject *collisionObject;

	void set_collision_object(btCollisionObject* co)
	{
		collisionObject = co;
	}

	void set_collision_shape(btCollisionShape* cs)
	{
		collisionShape = cs;
	}

	void create_texture(void)
	{
		D3DX11_IMAGE_LOAD_INFO loadInfo;
		ZeroMemory(&loadInfo, sizeof(D3DX11_IMAGE_LOAD_INFO) );
		loadInfo.BindFlags = D3D11_BIND_SHADER_RESOURCE;
		loadInfo.Format = DXGI_FORMAT_BC1_UNORM;

		HRESULT hr = D3DX11CreateShaderResourceViewFromFile(g_pd3dDevice, L"texture.bmp", &loadInfo, NULL, &texture2D_view, NULL);
		hr = hr;
	}

	void destroy()
	{
		SAFE_RELEASE( g_pIndexBuffer );
		SAFE_RELEASE( pVB[0] );
		SAFE_RELEASE( texture2D );
		SAFE_RELEASE( texture2D_view );
	}

	void draw(void)
	{
		

		if (!collisionObject)
			return;


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

	

		
		btTransform trans = collisionObject->getWorldTransform();

		
	    
		btVector3 origin = trans.getOrigin();
		btMatrix3x3 btM = trans.getBasis();
		
		btScalar* scalar_matrix = new btScalar[16];;
		trans.getOpenGLMatrix(scalar_matrix);
		
		D3DXMATRIXA16 m_trans(scalar_matrix[0],scalar_matrix[1],scalar_matrix[2],scalar_matrix[3],
							  scalar_matrix[4],scalar_matrix[5],scalar_matrix[6],scalar_matrix[7],
							  scalar_matrix[8],scalar_matrix[9],scalar_matrix[10],scalar_matrix[11],
							  scalar_matrix[12],scalar_matrix[13],scalar_matrix[14],scalar_matrix[15]);

		D3DXMATRIXA16 m_scale;
		float sc = 10;
		D3DXMatrixScaling(&m_scale,sc,sc,sc);


		D3DXVECTOR3 vCenter( global_shift_x, global_shift_y, global_shift_z);

		D3DXMatrixTranslation( &g_mCenterMesh, -vCenter.x+x_offset, -vCenter.y+y_offset, -vCenter.z+z_offset );


		D3DXMATRIXA16 m_trans_transpose;
		D3DXMatrixTranspose(&m_trans_transpose,&m_trans);
		
		mWorld = *g_Camera.GetWorldMatrix() ;
		mProj = *g_Camera.GetProjMatrix();
		mView = m_trans * *g_Camera.GetViewMatrix();

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

		pd3dImmediateContext->PSSetSamplers( 0, 1, &g_pSamLinear );

		{
			// Get the subset
			pSubset = g_Mesh11.GetSubset( 0, 0 );

			pd3dImmediateContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

			pd3dImmediateContext->PSSetShaderResources(0,1,&texture2D_view);

			pd3dImmediateContext->DrawIndexed( (width*3*2+2 + height*width*3*2), 0, ( UINT )pSubset->VertexStart );
		}

		SAFE_RELEASE(pd3dImmediateContext);
	}


	void create_buffers(int width_, int height_, bool top_)
	{

		top = top_;
		width = width_;
		height = height_;

		D3D11_BUFFER_DESC bufferDesc;
		bufferDesc.Usage = D3D11_USAGE_DEFAULT;
		bufferDesc.ByteWidth = sizeof(vertex_struct)*width*height;
		bufferDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
		bufferDesc.CPUAccessFlags = 0;
		bufferDesc.MiscFlags = 0;


		vertex_struct *vertices = new vertex_struct[width*height];

		btCapsuleShape* cs = static_cast<btCapsuleShape*>(collisionShape);
		if (cs)
		{
			float radius = cs->getRadius();
			float halfHeight = cs->getHalfHeight();
			

			if (top)
			{
				for(int y = 0; y < height; y++)
				{
					for(int x = 0; x < width; x++)
					{	
						float X =  (x/((float)(width-1)))*3.14159;
						float Y =  (y/((float)(height-1)))*3.14159;
						float z_coord = radius*cos(X)*sin(Y);
						float y_coord = radius*sin(X)*sin(Y) + halfHeight;
						float x_coord = radius*cos(Y);
						vertices[y*width+x].Pos = D3DXVECTOR3(x_coord, y_coord, z_coord); 
						vertices[y*width+x].Normal = D3DXVECTOR3(x_coord,y_coord-halfHeight,z_coord);
						vertices[y*width+x].Texcoord = D3DXVECTOR2(x/( (float)(width-1)), y/((float)(height-1)));
					}
				}
			} else	{
				for(int y = 0; y < height; y++)
				{
					for(int x = 0; x < width; x++)
					{	
						float X =  (x/((float)(width-1)))*3.14159;
						float Y =  (y/((float)(height-1)))*3.14159;
						float z_coord = radius*cos(X)*sin(Y);
						float y_coord = -radius*sin(X)*sin(Y) - halfHeight;
						float x_coord = radius*cos(Y);
						vertices[y*width+x].Pos = D3DXVECTOR3(x_coord, y_coord, z_coord); 
						vertices[y*width+x].Normal = D3DXVECTOR3(x_coord,y_coord+halfHeight,z_coord);
						vertices[y*width+x].Texcoord = D3DXVECTOR2(x/( (float)(width-1)), y/((float)(height-1)));
					}
				}
			}

			D3D11_SUBRESOURCE_DATA InitData;
			InitData.pSysMem = vertices;
			InitData.SysMemPitch = 0;
			InitData.SysMemSlicePitch = 0;

			HRESULT hr = g_pd3dDevice->CreateBuffer(&bufferDesc, &InitData, &pVB[0]);
		    
			
			
			//What is this vertex stride thing all about?
			Strides[0] = ( UINT )g_Mesh11.GetVertexStride( 0, 0 );
			Offsets[0] = 0;

			unsigned int* indices = new unsigned int[width*3*2+2 + height*width*3*2];

			for(int y = 0; y < height-1; y++)
			{
				for(int x = 0; x < width-1; x++)
				{
					indices[x*3*2 + y*width*3*2] = x + y*width;
					indices[x*3*2+1 + y*width*3*2] = x+1 + y*width;
					indices[x*3*2+2 + y*width*3*2] = x+width + y*width;

					indices[x*3*2 + 3 + y*width*3*2] = x + 1 +  y*width;
					indices[x*3*2 + 4 + y*width*3*2] = x+(width+1) + y*width;
					indices[x*3*2 + 5 + y*width*3*2] = x+width + y*width;

				}
			}

			bufferDesc.ByteWidth = sizeof(unsigned int)*(width*3*2+2 + height*width*3*2);
			bufferDesc.BindFlags = D3D11_BIND_INDEX_BUFFER;

			InitData.pSysMem = indices;

			hr = g_pd3dDevice->CreateBuffer(&bufferDesc, &InitData, &g_pIndexBuffer);
			hr = hr;
		}
	}
};

#endif CAP_H
