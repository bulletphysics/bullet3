MSTRINGIFY(

cbuffer OutputToVertexArrayCB : register( b0 )
{
	int startNode;
	int numNodes;
	int offsetX;
	int strideX;
	
	int offsetN;	
	int strideN;
	int padding1;
	int padding2;
};


StructuredBuffer<float4> g_nodesx : register( t0 );
StructuredBuffer<float4> g_nodesn : register( t1 );

RWStructuredBuffer<float> g_vertexBuffer : register( u0 );


[numthreads(128, 1, 1)]
void 
OutputToVertexArrayKernel( uint3 Gid : SV_GroupID, uint3 DTid : SV_DispatchThreadID, uint3 GTid : SV_GroupThreadID, uint GI : SV_GroupIndex )
{
	int nodeID = DTid.x;
	if( nodeID < numNodes )
	{			
		float4 nodeX = g_nodesx[nodeID + startNode];
		float4 nodeN = g_nodesn[nodeID + startNode];
		
		// Stride should account for the float->float4 conversion
		int positionDestination = nodeID * strideX + offsetX;		
		g_vertexBuffer[positionDestination] = nodeX.x;
		g_vertexBuffer[positionDestination+1] = nodeX.y;
		g_vertexBuffer[positionDestination+2] = nodeX.z;
		
		int normalDestination = nodeID * strideN + offsetN;
		g_vertexBuffer[normalDestination] = nodeN.x;
		g_vertexBuffer[normalDestination+1] = nodeN.y;
		g_vertexBuffer[normalDestination+2] = nodeN.z;		
	}
}

);