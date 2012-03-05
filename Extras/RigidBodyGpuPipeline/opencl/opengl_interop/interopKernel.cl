MSTRINGIFY(

__kernel void 
interopKernel( const int startOffset, const int numNodes, __global float *g_vertexBuffer)
{
	int nodeID = get_global_id(0);
	if( nodeID < numNodes )
	{
		g_vertexBuffer[nodeID*4 + startOffset+1] += 0.01;
	}
}

);