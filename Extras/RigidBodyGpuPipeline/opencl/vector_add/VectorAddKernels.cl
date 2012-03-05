

__kernel void VectorAdd(__global const float8* a, __global const float8* b, __global float8* c, int numElements)
{
    // get oct-float index into global data array
    int iGID = get_global_id(0);
	if (iGID>=numElements)
		return;

	float8 aGID = a[iGID];
	float8 bGID = b[iGID];

	float8 result = aGID + bGID;
    // write back out to GMEM
    c[iGID] = result;
}
