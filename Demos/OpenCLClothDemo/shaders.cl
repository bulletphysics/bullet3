#pragma OPENCL EXTENSION cl_amd_printf : enable                        

#define float3 float4
#define uint3  uint4

#define PARTICLE_RADIUS 0.05;

#define width 1280
#define height 1024

#define B 0 
#define T height
#define L 0
#define R width

#define shiftNumber 4
#define shiftMask 0xF
#define shiftValue 16.0f
#define stride 4

#define screenWidth1 width
#define screenHeight1 height
#define halfScreenWidth1 screenWidth1/2
#define halfScreenHeight1 screenHeight1/2
#define screenWidth1SubOne (screenWidth1-1)
#define screenHeight1SubOne (screenHeight1-1)
#define stride screenWidth1 
#define screenPixelNumber screenWidth1*screenHeight1
#define depthBufferSize screenPixelNumber*depthComplexity

#define WGS 1

//---------------------------------------------------------------

struct __VSSpriteOut
{
    float4 position; 
    float4 particlePosition; 
};

typedef struct __VSSpriteout VSSpriteOut;

struct __GSSpriteOut
{
    float4 position;
    float2 textureUV;
//	float4 viewSpacePosition;
//	float4 particlePosition;
};

typedef struct __GSSpriteout GSSpriteOut;

//------------------------------------------------------------------------------

__constant float4 g_positions[4] =
{
  (float4)(-1.0f, 1.0f, 0.0f, 0.0f),
  (float4)( 1.0f, 1.0f, 0.0f, 0.0f),
  (float4)( -1.0f, -1.0f, 0.0f, 0.0f),
  (float4)( 1.0f, -1.0f, 0.0f, 0.0f)
};

__constant float2 g_texcoords[4] = 
{ 
	(float2)(0.0f,0.0f), 
    (float2)(1.0f,0.0f),
    (float2)(0.0f,1.0f),
    (float2)(1.0f,1.0f)
};

//------------------------------------------------------------------------------

void copyMatrix(
	float matrix[16],
	__constant float matrix0[16])
{
	uint i;
	
	for (i = 0; i < 16; i++) {
		matrix[i] = matrix0[i];
	}
}

void matrixMulLoopBody(	
	uint i,
	float matrix[16], 
	__constant float matrix0[16], 
	__constant float matrix1[16])
{
	matrix[i] = 0.0f;
	matrix[i] += matrix0[(i%4) + (0*4)] * matrix1[(0) + ((i/4)*4)];
	matrix[i] += matrix0[(i%4) + (1*4)] * matrix1[(1) + ((i/4)*4)];
	matrix[i] += matrix0[(i%4) + (2*4)] * matrix1[(2) + ((i/4)*4)];
	matrix[i] += matrix0[(i%4) + (3*4)] * matrix1[(3) + ((i/4)*4)];
}

void matrixMul(
	float matrix[16], 
	__constant float matrix0[16], 
	__constant float matrix1[16])
{
	matrixMulLoopBody(0, matrix, matrix0, matrix1);
	matrixMulLoopBody(1, matrix, matrix0, matrix1);
	matrixMulLoopBody(2, matrix, matrix0, matrix1);	
	matrixMulLoopBody(3, matrix, matrix0, matrix1);
	matrixMulLoopBody(4, matrix, matrix0, matrix1);
	matrixMulLoopBody(5, matrix, matrix0, matrix1);
	matrixMulLoopBody(6, matrix, matrix0, matrix1);	
	matrixMulLoopBody(7, matrix, matrix0, matrix1);
	matrixMulLoopBody(8, matrix, matrix0, matrix1);
	matrixMulLoopBody(9, matrix, matrix0, matrix1);
	matrixMulLoopBody(10, matrix, matrix0, matrix1);	
	matrixMulLoopBody(11, matrix, matrix0, matrix1);
	matrixMulLoopBody(12, matrix, matrix0, matrix1);
	matrixMulLoopBody(13, matrix, matrix0, matrix1);
	matrixMulLoopBody(14, matrix, matrix0, matrix1);	
	matrixMulLoopBody(15, matrix, matrix0, matrix1);						
}

float4 matrixVectorMul(float matrix[16], float4 vector)
{
	float4 result;

	result.x = matrix[0]*vector.x + matrix[4+0]*vector.y + matrix[8+0]*vector.z + matrix[12+0]*vector.w;
	result.y = matrix[1]*vector.x + matrix[4+1]*vector.y + matrix[8+1]*vector.z + matrix[12+1]*vector.w;
	result.z = matrix[2]*vector.x + matrix[4+2]*vector.y + matrix[8+2]*vector.z + matrix[12+2]*vector.w;
	result.w = matrix[3]*vector.x + matrix[4+3]*vector.y + matrix[8+3]*vector.z + matrix[12+3]*vector.w;

	return result;
}

float3 matrixVector3Mul(__constant float matrix[9], float3 vector)
{
	float3 result;

	result.x = matrix[0]*vector.x + matrix[3+0]*vector.y + matrix[6+0]*vector.z;
	result.y = matrix[1]*vector.x + matrix[3+1]*vector.y + matrix[6+1]*vector.z;
	result.z = matrix[2]*vector.x + matrix[3+2]*vector.y + matrix[6+2]*vector.z;

	return result;
}

//------------------------------------------------------------------------------

//#define DEVICE_CPU 1
#if defined(DEVICE_CPU)
void printMatrix(char * name, __constant float matrix[16])
{
	printf("%s[0] = %f, %f, %f, %f\n", name, matrix[0], matrix[1], matrix[2], matrix[3]);	
	printf("%s[1] = %f, %f, %f, %f\n", name, matrix[4], matrix[5], matrix[6], matrix[7]);	
	printf("%s[2] = %f, %f, %f, %f\n", name, matrix[8], matrix[9], matrix[10], matrix[11]);	
	printf("%s[3] = %f, %f, %f, %f\n", name, matrix[12], matrix[13], matrix[14], matrix[15]);	
}
#endif

#if 1
__kernel void vertexShader(
    __constant float modelview[16],
	__constant float projection[16],
	__global float4 * inputPrimitives, 
	__global float4 * outputPrimitives)
{
	float matrix[16];
	float4 gl_Vertex;
	float4 gl_Position;

	uint id = get_global_id(0);
	
	gl_Vertex = inputPrimitives[id];

	// gl_ProjectionMatrix * gl_ModelViewMatrix * gl_Vertex
	matrixMul(matrix, projection, modelview);
	
	gl_Position = matrixVectorMul(matrix, gl_Vertex);

	outputPrimitives[id] = gl_Position;
}

#else

__kernel void vertexShader(
    __constant float modelview[16],
	__constant float projection[16],
	__global float4 * inputPrimitives, 
	__global float4 * outputPrimitives)
{
	uint id = get_global_id(0);

	outputPrimitives[id] = inputPrimitives[id];
}

#endif

//-----------------------------------------------------------------------------------

__kernel void
clearImage(
	__write_only image2d_t image,
	float4 color)
{

	int2 coords = (int2)(get_global_id(0), get_global_id(1));
	write_imagef(image, coords, color);
}

// OpenGL viewport transformation
// The site http://research.cs.queensu.ca/~jstewart/454/notes/pipeline/
// contains a description of this process
void 
viewportTransform(float4 v, __constant int4 viewport[1], float2 * output)
{
	int4 vp = viewport[0];
	*output 
		= 0.5f * 
		  (float2)(v.x+1,v.y+1) * 
		  (float2)((vp.s2-vp.s0) + vp.s0, 
				   (vp.s3-vp.s1) + vp.s1);
}

#define PARTICLE_WIDTH  32.0f
#define PARTICLE_HEIGHT 32.0f

// Unoptimized triangle rasterizer function
// Details of the algorithm can be found here:
//		http://www.devmaster.net/forums/showthread.php?t=1884
//	
void
rasterizerUnOpt(
    __global struct __GSSpriteOut * outputPrimitives,
//	 __global float4 * outputPrimitives,
	__constant int4  viewport[1],
	__write_only image2d_t screen,
	__read_only image2d_t particle,
	uint v1Offset,
	uint v2Offset,
	uint v3Offset,
	__global float4 * debugOut1)
{
	sampler_t sampler = 
		CLK_NORMALIZED_COORDS_TRUE | CLK_ADDRESS_CLAMP | CLK_FILTER_NEAREST;

	uint id = get_global_id(0);

	struct __GSSpriteOut output;
	float2 v1, v2, v3;
	float2 uv1, uv2, uv3;

	output = outputPrimitives[id*4+v1Offset];
	uv1    = output.textureUV;
	viewportTransform(output.position, viewport, &v1);

	output = outputPrimitives[id*4+v2Offset];
	uv2    = output.textureUV;
	viewportTransform(output.position, viewport, &v2);

	output = outputPrimitives[id*4+v3Offset];
	uv3    = output.textureUV;
	viewportTransform(output.position, viewport, &v3);

	// Bounding rectangle
	int2 min_ = convert_int2(min(v1, min(v2, v3)));
	int2 max_ = convert_int2(max(v1, max(v2, v3)));

	// naive bi-linear interploation for texture coords, note this is 
	// broken with respect to OpenGL and needs to be fixed for the 
	// general case.
	float p1x = v2.x - v1.x;
	float p1y = v2.y - v1.y;
	
	float p2x = v3.x - v1.x;
	float p2y = v3.y - v1.y;

	// Scan through bounding rectangle
	for(int y = min_.y; y < max_.y; y++) {
		for(int x = min_.x; x < max_.x; x++) {
			// When all half-space functions positive, pixel is in triangle
			if((v1.x - v2.x) * (y - v1.y) - (v1.y - v2.y) * (x - v1.x) > 0 &&
			 (v2.x - v3.x) * (y - v2.y) - (v2.y - v3.y) * (x - v2.x) > 0 &&
			 (v3.x - v1.x) * (y - v3.y) - (v3.y - v1.y) * (x - v3.x) > 0) {

				float px = x - v1.x;
				float py = y - v1.y;
	
					write_imagef(
						screen, 
						(int2)(x,y), 
					//	texel);
						(float4)(1.0f,1.0f,1.0f,1.0f));
			}
		}
	}
}

// Optimized rasterizer function
// Details of the algorithm can be found here:
//		http://www.devmaster.net/forums/showthread.php?t=1884
//	
// Currently has a bug, still work in progess
__kernel void
rasterizerXX(
    __global float4 * outputPrimitives,
	__write_only image2d_t screen,
	__global float4 * debugOut1,
	__global int2 * debugOut2)
{
	uint id = get_global_id(0);

//	printf("ras\n");

	float4 v1 = outputPrimitives[id*4+0];
	float4 v2 = outputPrimitives[id*4+1];
	float4 v3 = outputPrimitives[id*4+2];

	float y1 = 0.5f* (v1.y+1) * (T - B) + B;
	float y2 = 0.5f* (v2.y+1) * (T - B) + B;
    float y3 = 0.5f* (v3.y+1) * (T - B) + B;

    float x1 = 0.5f * (v1.x+1) * (R - L) + L;
	float x2 = 0.5f * (v2.x+1) * (R - L) + L;
	float x3 = 0.5f * (v3.x+1) * (R - L) + L;

    const int Y1 = convert_int(shiftValue * y1);
    const int Y2 = convert_int(shiftValue * y2);
    const int Y3 = convert_int(shiftValue * y3);

    const int X1 = convert_int(shiftValue * x1);
    const int X2 = convert_int(shiftValue * x2);
    const int X3 = convert_int(shiftValue * x3);

	debugOut1[id*4+0]   = v1;
	debugOut1[id*4+1]   = v2;
	debugOut1[id*4+2]   = v3;

	debugOut2[id*3+0] = (int2)(X1, Y1);
	debugOut2[id*3+1] = (int2)(X2, Y2);
	debugOut2[id*3+2] = (int2)(X3, Y3);

    // Deltas
    const int DX12 = X1 - X2;
    const int DX23 = X2 - X3;
    const int DX31 = X3 - X1;

    const int DY12 = Y1 - Y2;
    const int DY23 = Y2 - Y3;
    const int DY31 = Y3 - Y1;

    // Fixed-point deltas
    const int FDX12 = DX12 << shiftNumber;
    const int FDX23 = DX23 << shiftNumber;
    const int FDX31 = DX31 << shiftNumber;

    const int FDY12 = DY12 << shiftNumber;
    const int FDY23 = DY23 << shiftNumber;
    const int FDY31 = DY31 << shiftNumber;

    // Bounding rectangle
    int minx = (min(X1, min(X2, X3)) + shiftMask) >> shiftNumber;
	//minx = max(0,minx);
    
	int maxx = (max(X1, min(X2, X3)) + shiftMask) >> shiftNumber;
	//min(maxx , screenWidth1SubOne);

	int miny = (min(Y1, min(Y2, Y3)) + shiftMask) >> shiftNumber;
    //max(0,miny);

	int maxy = (max(Y1, min(Y2, Y3)) + shiftMask) >> shiftNumber;
	//min(maxy , screenHeight1SubOne);

    //(char*&)colorBuffer += miny * stride;
	int offset = miny * stride;

    // Half-edge constants
    int C1 = DY12 * X1 - DX12 * Y1;
    int C2 = DY23 * X2 - DX23 * Y2;
    int C3 = DY31 * X3 - DX31 * Y3;

    // Correct for fill convention
    if(DY12 < 0 || (DY12 == 0 && DX12 > 0)) C1++;
    if(DY23 < 0 || (DY23 == 0 && DX23 > 0)) C2++;
    if(DY31 < 0 || (DY31 == 0 && DX31 > 0)) C3++;

    int CY1 = C1 + DX12 * (miny << shiftNumber) - DY12 * (minx << shiftNumber);
    int CY2 = C2 + DX23 * (miny << shiftNumber) - DY23 * (minx << shiftNumber);
    int CY3 = C3 + DX31 * (miny << shiftNumber) - DY31 * (minx << shiftNumber);

    for(int y = miny; y < maxy; y++) {
        int CX1 = CY1;
        int CX2 = CY2;
        int CX3 = CY3;

		debugOut2[id*3+0] = (int2)(minx, maxx);

        for(int x = minx; x < maxx; x++) {
			debugOut2[id*3+0] = (int2)(CX1, CX2);

            if(CX1 > 0 && CX2 > 0 && CX3 > 0) {
				debugOut2[id*3+0] = (int2)(1, 1);
				write_imagef(
					screen, 
					(int2)(x,y), 
					(float4)(1.0f,1.0f,1.0f,1.0f));
           }

            CX1 -= FDY12;
            CX2 -= FDY23;
            CX3 -= FDY31;
        }

        CY1 += FDX12;
        CY2 += FDX23;
        CY3 += FDX31;

        //(char*&)colorBuffer += stride;
		offset += stride;
    }
}

//------------------------------------------------------------------------------

void geometryShader(
    __constant float modelview[16],
	__constant float projection[16],
	__constant float inverseView[9],
	__constant int4  viewport[1],
	__local struct __VSSpriteOut  * vsOutputPrimitives,
	__global struct __GSSpriteOut * outputPrimitives,
//	 __global float4 * outputPrimitives,
	__write_only image2d_t screen,
	__read_only image2d_t particle,
	__global float4 * debugOut1,
	__global int * debugOut2)
{
	float2 texcoords[4] = 
	{ 
		(float2)(0.0f,0.0f), 
		(float2)(1.0f,0.0f),
		(float2)(0.0f,1.0f),
		(float2)(1.0f,1.0f)
	};

	float matrix[16];

	uint id  = get_global_id(0);
	uint lid = get_local_id(0);
	
	float4 vsPosition = vsOutputPrimitives[lid].position;

	matrixMul(matrix, projection, modelview);
	//
	// Emit two new triangles
	//
	for (uint i = 0; i<4; i++) {
		float3 position = g_positions[i] * PARTICLE_RADIUS;
		position        = matrixVector3Mul(inverseView, position) + vsPosition;
		float3 particlePosition = 
			matrixVector3Mul( 
				inverseView, 
				(float4)(0.0f,0.0f,0.0f,0.0f)) + vsPosition;	// world space
	
		// Compute view space position
		position.w               = 1.0f;
		position                 = matrixVectorMul(matrix, position);

		//perspective division
		position /= position.w;

		struct __GSSpriteOut output;
		output.position  = position;
		//output.textureUV = g_texcoords[i];
		output.textureUV = texcoords[i];
		outputPrimitives[id*4+i] = output; 
	}	

	// Render QUAD - Triangle 1
	rasterizerUnOpt(
		outputPrimitives,
		viewport,
		screen,
		particle,
		0,
		1,
		2,
		debugOut1);

	// Render QUAD - Triangle 2
	rasterizerUnOpt(
		outputPrimitives,
		viewport,
		screen,
		particle,
		2,
		1,
		3,
		debugOut1);
}

__kernel void vertexShaderSprite(
    __constant float modelview[16],
	__constant float projection[16],
	__constant float inverseView[9],
	__constant int4  viewport[1],
	__local struct __VSSpriteOut  * vsOutputPrimitives,
	__global float4               * inputPrimitives, 	
	__global struct __GSSpriteOut * outputPrimitives,
//	__global float4 * outputPrimitives,
	__write_only image2d_t screen,
	__read_only image2d_t particle,
	__global float4 * debugOut1,
	__global int * debugOut2)
{
	float matrix[16];

	uint id  = get_global_id(0);
	uint lid = get_local_id(0);

	// gl_ProjectionMatrix * gl_ModelViewMatrix * gl_Vertex
	matrixMul(matrix, projection, modelview);

	float4 position                          = inputPrimitives[id];
	vsOutputPrimitives[lid].position         = position;
    vsOutputPrimitives[lid].particlePosition = 
		matrixVectorMul(matrix, position); 
	
	geometryShader(
		modelview, 
		projection, 
		inverseView, 
		viewport,
		vsOutputPrimitives, 
		outputPrimitives,
		screen,
		particle,
		debugOut1,
		debugOut2);
}