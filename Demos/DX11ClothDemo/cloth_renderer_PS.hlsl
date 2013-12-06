//--------------------------------------------------------------------------------------
// File: BasicHLSL11_PS.hlsl
//
// The pixel shader file for the BasicHLSL11 sample.  
// 
// Copyright (c) Microsoft Corporation. All rights reserved.
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
// Globals
//--------------------------------------------------------------------------------------
cbuffer cbPerObject : register( b0 )
{
	float4		g_vObjectColor			: packoffset( c0 );
};

cbuffer cbPerFrame : register( b1 )
{
	float3		g_vLightDir				: packoffset( c0 );
	float		g_fAmbient				: packoffset( c0.w );
};

//--------------------------------------------------------------------------------------
// Textures and Samplers
//--------------------------------------------------------------------------------------
Texture2D	g_txDiffuse : register( t0 );
SamplerState g_samLinear : register( s0 );

//--------------------------------------------------------------------------------------
// Input / Output structures
//--------------------------------------------------------------------------------------
struct PS_INPUT
{
	float3 vNormal		: NORMAL;
	float2 vTexcoord	: TEXCOORD0;
	float4 vPosition : SV_POSITION;
};

//--------------------------------------------------------------------------------------
// Pixel Shader
//--------------------------------------------------------------------------------------
float4 PSMain( PS_INPUT Input ) : SV_TARGET
{
	float4 vDiffuse = g_txDiffuse.Sample( g_samLinear, Input.vTexcoord );
	
	
	float fLighting = saturate( dot( g_vLightDir, (Input.vNormal)/length(Input.vNormal) ) );
	//float fLighting = saturate( dot( g_vLightDir, float3(Input.vTexcoord.x,0,0) )  );
	fLighting = max( fLighting, g_fAmbient );
	
	//fLighting = dot(g_vLightDir,float3(0,1,0));
	
	return vDiffuse * fLighting;
}


struct VS_OUTPUT
{
	float3 vNormal		: NORMAL;
	float2 vTexcoord	: TEXCOORD0;
	float4 vPosition	: SV_POSITION;
};

[maxvertexcount(3)]
void GSMain(triangle VS_OUTPUT input[3], inout TriangleStream<PS_INPUT> OutputStream)
{
PS_INPUT output = (PS_INPUT)0;


/*
float3 v1 = input[1].vPosition - input[0].vPosition;
float3 v2 = input[2].vPosition - input[0].vPosition;
float3 normal = cross(v1,v2);

normal = normalize(normal);
*/


for(int i = 0; i < 3; i++)
{

output.vNormal = input[i].vNormal;
output.vTexcoord = input[i].vTexcoord;
output.vPosition = input[i].vPosition;
OutputStream.Append(output);
}

OutputStream.RestartStrip();

}