//--------------------------------------------------------------------------------------
// File: DXUTShared.fx
//
// 
// 
// Copyright (c) Microsoft Corporation. All rights reserved.
//--------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------
// Global variables
//--------------------------------------------------------------------------------------
float4 g_MaterialDiffuseColor;      // Material's diffuse color
float3 g_LightDir;                  // Light's direction in world space
float4x4 g_mWorld;                  // World matrix for object
float4x4 g_mWorldViewProjection;    // World * View * Projection matrix



//--------------------------------------------------------------------------------------
// Vertex shader output structure
//--------------------------------------------------------------------------------------
struct VS_OUTPUT
{
    float4 Position   : POSITION;   // vertex position 
    float4 Diffuse    : COLOR0;     // vertex diffuse color 
};


//--------------------------------------------------------------------------------------
// This shader computes standard transform and lighting
//--------------------------------------------------------------------------------------
VS_OUTPUT RenderWith1LightNoTextureVS( float4 vPos : POSITION, 
                                       float3 vNormal : NORMAL )
{
    VS_OUTPUT Output;
  
    // Transform the position from object space to homogeneous projection space
    Output.Position = mul(vPos, g_mWorldViewProjection);
    
    // Transform the normal from object space to world space    
    float3 vNormalWorldSpace;
    vNormalWorldSpace = normalize(mul(vNormal, (float3x3)g_mWorld)); // normal (world space)
    
    // Compute simple directional lighting equation
    Output.Diffuse.rgb = g_MaterialDiffuseColor * max(0,dot(vNormalWorldSpace, g_LightDir));   
    Output.Diffuse.a = 1.0f; 
    
    return Output;    
}


//--------------------------------------------------------------------------------------
float4 RenderWith1LightNoTexturePS( float4 Diffuse : COLOR0 ) : COLOR0
{ 
    return Diffuse;
}


//--------------------------------------------------------------------------------------
technique RenderWith1LightNoTexture
{
    pass P0
    {          
        VertexShader = compile vs_1_1 RenderWith1LightNoTextureVS();
        PixelShader  = compile ps_1_1 RenderWith1LightNoTexturePS(); 
    }
}

