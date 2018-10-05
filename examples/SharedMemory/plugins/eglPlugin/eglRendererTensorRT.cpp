/*
eglRendererTensorRT
Copyright (c) 2018 Dmitry Chichkov

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use
of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim
that you wrote the original software. If you use this software in a product, an
acknowledgment in the product documentation would be appreciated but is not
required.
2. Altered source versions must be plainly marked as such, and must not be
misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include <cuda_gl_interop.h>
#include <cuda_runtime_api.h>

#include <NvInfer.h>
#include <NvUffParser.h>
using namespace nvinfer1;
using namespace nvuffparser;

//#define DEBUG_TENSORRT_INFERENCE
#ifdef DEBUG_TENSORRT_INFERENCE
#include <algorithm>
#include "stb_image/stb_image_write.h"
#endif  // DEBUG_TENSORRT_INFERENCE

class GLTensorRTLogger : public nvinfer1::ILogger
{
	void log(Severity severity, const char *msg)
	{
		if (severity != Severity::kINFO && severity != Severity::kWARNING)
			b3Error("GLTensorRTLogger: %d %s\n", severity, msg);
#ifdef DEBUG_TENSORRT_INFERENCE
		else
			b3Warning("GLTensorRTLogger: %d %s\n", severity, msg);
#endif  // DEBUG_TENSORRT_INFERENCE
	}
};

struct EGLRendererTensorRT
{
	/** \brief Initializes a TensorRT inference engine from .uff or .plan model files.
	* \param modelFileName a path to .uff or .plan model
	* \param modelInputLayer a name of a input layer. Should be float32, in
	* range of 0..1, shaped (height, width, 3).
	* \param modelOutputLayers a list of output layer names, trailing with zero.
	* Supports only float32 layer outputs.
	* \param width expected width of the input layer (and image).
	* \param height expected height of the input layer.
	* \param kBatchSize process a batch of width x height images. Expected
	* rendered input is width x (height*kBatchSize).
	* This initializes a TensorRT inference engine from .uff or .plan model files.
	* Please refer to TensorRT documentation and examples to create such files.
	*
	* Hint: Use uff.from_tensorflow() and uff_to_plan to convert TensorFlow
	* models.
	*
	* Usage:
	*/
	EGLRendererTensorRT(const char *modelFileName, const char *modelInputLayer,
						const char **modelOutputLayers, int width, int height,
						int kBatchSize = 1, int kWorkspaceSize = 1 << 26);

	~EGLRendererTensorRT();

	int m_width, m_height;
	int m_kBatchSize;

	/// output size in bytes (incl. batch calc.)
	int m_totalOutputSize;

	// feature length in floats
	int m_featureLength;

	/// PBO (pixels to attach to TensorRT)
	unsigned int pbo;
	cudaGraphicsResource_t pboRes;

	/// CUDA / TensorRT memory, used to store output layer before transferring to CPU
	void *outputDataDevice;
	nvinfer1::ICudaEngine *engine;

	/// TensorRT execution context, used to store intermediate activation values
	nvinfer1::IExecutionContext *context;

	//  TensorRT error logger, use -DDEBUG_TENSORRT_INFERENCE to enable more
	//  detailed logging.
	GLTensorRTLogger gLogger;

	// CUDA / TensorRT engine memory bindings, contains CUDA pointers indexed by
	// binding indexes
	btAlignedObjectArray<void *> bindings;

	// Index into bindings array (for the input layer)
	int m_inputBindingIndex;


	int getFeatureLength()
	{
		return m_featureLength;
	}

	void uninitTensorRTEngine();

	/** \brief Transfers pixels from GL to CUDA, executes TensorRT engine and
	* outputs the result to outputBuffer.
	* \param outputBuffer, a pointer to CPU memory
	* \param outputBufferSizeInBytes buffer size in bytes.
	*
	*/
	size_t copyCameraImageFeatures(float *outputBuffer,
								   size_t outputBufferSizeInBytes);


	/** Returns tensor size, in elements.
	*/
	static size_t size(nvinfer1::Dims shape)
	{
		size_t size = shape.nbDims > 0 ? 1 : 0;
		for (int i = 0; i < shape.nbDims; i++)
			size *= shape.d[i];
		return size;
	}
};

EGLRendererTensorRT::EGLRendererTensorRT(const char *modelFileName,
											  const char *modelInputLayer,
											  const char **modelOutputLayers,
											  int width, int height,
											  int kBatchSize, int kWorkspaceSize)
	: m_width(width), m_height(height), m_kBatchSize(kBatchSize), m_totalOutputSize(0), m_featureLength(0),
	  pbo(0), pboRes(0), outputDataDevice(0), engine(0), context(0), m_inputBindingIndex(0)
{
	//if (endswith(modelFileName, ".uff"))
	{
		IBuilder *builder = createInferBuilder(gLogger);
		builder->setMaxBatchSize(kBatchSize);
		builder->setMaxWorkspaceSize(kWorkspaceSize);
		// builder->allowGPUFallback(true);
		// builder->setFp16Mode(true);
		// builder->setDefaultDeviceType(DeviceType::kDLA);

		if (builder == 0)
		{
			b3Error(
				"Failed to create TensorRT Builder object, please check your "
				"TensorRT installation or attempt to load .plan file.\n");
			return;
		}

		INetworkDefinition *network = builder->createNetwork();
		IUffParser *parser = createUffParser();
	    parser->registerInput(modelInputLayer, DimsCHW(3, height, width), UffInputOrder::kNCHW);

		for (const char **modelOutputLayer = modelOutputLayers; *modelOutputLayer; modelOutputLayer++)
			parser->registerOutput(*modelOutputLayer);

		if (network == 0 || parser == 0)
		{
			b3Error(
				"Failed to create a TensorRT object, please check your TensorRT "
				"installation or attempt to load .plan file\n");
			builder->destroy();
			return;
		}

		if (parser->parse(modelFileName, *network, nvinfer1::DataType::kFLOAT))
		{
			engine = builder->buildCudaEngine(*network);
		}
		else
		{
			b3Error(
				"Failed to parse a uff file %s, please check your TensorRT "
				"installation.\n");
			b3Error(
				"Please attempt to convert your model to .plan with "
				"uff.from_tensorflow() and uff_to_plan.\n",
				modelFileName);
		}

		parser->destroy();
		network->destroy();
		builder->destroy();
	}
	/*
	else if (endswith(modelFileName, ".plan"))
	{
		IRuntime *runtime = createInferRuntime(gLogger);
		engine = runtime->deserializeCudaEngine((void *)plan.data(), plan.size(),
												nullptr);
		runtime->destroy();
	}
	else
	{
		b3Error(
			"Unrecognized file %s, please attempt to load .uff or .plan files..\n",
			modelFileName);
		b3Error(
			"To convert your model to .plan, please use uff.from_tensorflow() "
			"and uff_to_plan utility.\n",
			modelFileName);
		return;
	}*/

	// we should have a TensorRT engine by now, complain and abort if we don't
	if (!engine)
	{
		b3Error(
			"Failed to create a TensorRT engine, please check your TensorRT "
			"installation or attempt to load .plan file\n");
		return;
	}

	// create execution context to store intermediate activation values.
	// TODO: support for multiple contexts
	context = engine->createExecutionContext();
	if (!context)
	{
		b3Error(
			"Failed to create a TensorRT engine execution context, please "
			"check available CUDA memory.\n");
		uninitTensorRTEngine();
		return;
	}

	// get the input dimensions of the network
	m_inputBindingIndex = engine->getBindingIndex(modelInputLayer);
	if (m_inputBindingIndex < 0)
	{
		b3Error(
			"Failed to bind input to TensorRT engine. Please check that %s "
			"model contains %s input layer.",
			modelFileName, modelInputLayer);
		uninitTensorRTEngine();
		return;
	}

	// make sure that that rendering is the same size as the network input
	Dims inputDims = engine->getBindingDimensions(m_inputBindingIndex);
	if (m_width != inputDims.d[1] || m_height != inputDims.d[2] ||
		3 != inputDims.d[0])
	{
		b3Error(
			"Error rendered image is %d x %d x %d and inference engine expects "
			"%d x %d x %d.\n",
			m_width, m_height, 3, inputDims.d[1], inputDims.d[2],
			inputDims.d[0]);
		uninitTensorRTEngine();
		return;
	}

	bindings.resize(m_inputBindingIndex + 1);

	// get the output dimensions of the network, calculate m_totalOutputSize, m_featureLength
	for (const char **modelOutputLayer = modelOutputLayers; *modelOutputLayer;
		 modelOutputLayer++)
	{
		int outputBindingIndex = engine->getBindingIndex(*modelOutputLayer);
		if (outputBindingIndex < 0)
		{
			b3Error(
				"Failed to bind output to TensorRT engine. Please check that %s "
				"model contains %s input layer.",
				modelFileName, *modelOutputLayer);
			uninitTensorRTEngine();
			return;
		}

		size_t numOutput = size(engine->getBindingDimensions(outputBindingIndex));
		m_featureLength += numOutput;
		m_totalOutputSize += numOutput * sizeof(float) * m_kBatchSize;
	}

	// Allocate CUDA memory for output buffer
	cudaMalloc(&outputDataDevice, m_totalOutputSize);
	if (outputDataDevice == 0)
	{
		b3Error(
			"Failed to allocate %d bytes of CUDA memory for the TensorRT "
			"engine. Please make sure sufficient CUDA memory is available.\n",
			m_totalOutputSize);
		uninitTensorRTEngine();
		return;
	}

	// Update bindings (containing offset) to a correct memory pointer
	size_t currentOutputPos = 0;
	for (const char **modelOutputLayer = modelOutputLayers; *modelOutputLayer;
		 modelOutputLayer++)
	{
		int outputBindingIndex = engine->getBindingIndex(*modelOutputLayer);
		if(outputBindingIndex >= bindings.size())
			bindings.resize(outputBindingIndex + 1);

		bindings[outputBindingIndex] = (char *)outputDataDevice + currentOutputPos;
		if (bindings[outputBindingIndex] > (char *)outputDataDevice + m_totalOutputSize)
		{
			b3Error(
				"Duplicate or erroneous output binding index is encountered in "
				"the TensorRT output specification.\n",
				m_totalOutputSize);
			uninitTensorRTEngine();
			return;
		}

		size_t numOutput = size(engine->getBindingDimensions(outputBindingIndex));
		currentOutputPos += numOutput * sizeof(float) * m_kBatchSize;
	}

	// we need to initialize PBO, but can't do it yet, because GL is not yet
	// ready.
	pbo = 0;
}

void EGLRendererTensorRT::uninitTensorRTEngine()
{
	if (engine != 0)
	{
		engine->destroy();
		engine = 0;
	}

	if (context != 0)
	{
		context->destroy();
		context = 0;
	}

	if (outputDataDevice)
	{
		cudaFree(outputDataDevice);
		outputDataDevice = 0;
	}
}

EGLRendererTensorRT::~EGLRendererTensorRT() { uninitTensorRTEngine(); }

size_t
EGLRendererTensorRT::copyCameraImageFeatures(float *outputBuffer,
											 size_t outputBufferSizeInBytes)
{
	if (m_totalOutputSize > outputBufferSizeInBytes)
	{
		b3Error(
			"Error during rendering-inferencing, CPU buffer size to store "
			"output features is too small. Expected %d, provided %d\n",
			m_totalOutputSize, outputBufferSizeInBytes);
		return 0;
	}

	// Bind buffer (or Generate PBO, if it is a first run)
	if (!pbo)
	{
		glGenBuffers(1, &pbo);
		glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo);
		glBufferData(GL_PIXEL_PACK_BUFFER,
					 3 * (m_kBatchSize * m_width * m_height * sizeof(GLfloat)), NULL,
					 GL_DYNAMIC_READ);

		if (glGetError() != GL_NO_ERROR)
		{
			b3Error(
				"Error during rendering-inferencing, rendered image size is "
				"invalid (?) (should be %dx%dx3).\n",
				m_height * m_kBatchSize, m_width);
			return 0;
		}

		// Register buffer to CUDA memory
		if (cudaGraphicsGLRegisterBuffer(&pboRes, pbo,
										 cudaGraphicsRegisterFlagsReadOnly) != 0)
		{  // cudaGraphicsMapFlagsWriteDiscard
			b3Error(
				"Error during registering GL buffer in CUDA, rendered image size "
				"is invalid (?) (should be %dx%dx3).\n",
				m_height * m_kBatchSize, m_width);
			return 0;
		}
	}
	else
	{
		glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo);
		glBufferData(GL_PIXEL_PACK_BUFFER,
					 3 * (m_kBatchSize * m_width * m_height * sizeof(GLfloat)), NULL,
					 GL_DYNAMIC_READ);
		if (glGetError() != GL_NO_ERROR)
		{
			b3Error(
				"Error during rendering-inferencing, rendered image size is "
				"invalid (should be %dx%dx3).",
				m_height * m_kBatchSize, m_width);
			return 0;
		}
	}

	// NOTE, we are casting to GL_FLOAT, because TensorRT accepts only floats,
	// TODO: use bytes.
	glReadPixels(0, 0, m_width, m_height * m_kBatchSize, GL_RGB, GL_FLOAT,
				 0);  // 0 is an *offset* into the buffer, not the pointer

	if (glGetError() != GL_NO_ERROR)
	{
		b3Error(
			"Error during rendering-inferencing, rendered image size is "
			"invalid (should be %dx%dx3).",
			m_height * m_kBatchSize, m_width);
		return 0;
	}

	// Unbind the buffer from PBO
	glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

	// Map PBO to CUDA
	cudaGraphicsMapResources(1, &pboRes);

	// Obtain CUDA pointer of PBO. NOTE: FLOATs, range 0..1.  :(  If only TensorRT
	// could accept bytes
	void *inputDataDevice = NULL;
	size_t size = 0;
	cudaGraphicsResourceGetMappedPointer(&inputDataDevice, &size, pboRes);
	if (inputDataDevice == 0)
	{
		b3Error(
			"Error during rendering-inferencing, failure in "
			"cudaGraphicsResourceGetMappedPointer. Try different CUDA/driver "
			"version(?).");
		return 0;
	}

	// Fill TensorRT device bindings, see m_inputBindingIndex. outputBindingIndexes
	// are already set in the init
	bindings[m_inputBindingIndex] = (void *)inputDataDevice;

	// Run Inference
	if (!context->execute(m_kBatchSize, &bindings[0]))
	{
		b3Error(
			"Error during rendering-inferencing, failure executing TensorRT "
			"engine. See TensorRT log, try different TensorRT version(?).");
		return 0;
	}

#ifdef DEBUG_TENSORRT_INFERENCE
	// transfer input image back to host and save it (debug)
	float *inputDataHost =
		(float *)malloc(3 * (m_width * m_height * sizeof(GLfloat)));
	memset(inputDataHost, 0, 3 * (m_width * m_height * sizeof(GLfloat)));
	cudaMemcpy(inputDataHost, inputDataDevice,
			   3 * (m_width * m_height * sizeof(GLfloat)),
			   cudaMemcpyDeviceToHost);

	unsigned char *rgbaBuffer =
		(unsigned char *)malloc(3 * (m_width * m_height * sizeof(unsigned char)));

	for (int i = 0; i < 3 * m_width * m_height; i++)
	{
		rgbaBuffer[i] =
			(unsigned char)(inputDataHost[i] *
							255.0);  // FLOAT, range 0..1, converting to bytes
	}
	stbi_write_png("getScreenPixels.dump.png", m_width, m_height, 3, rgbaBuffer,
				   m_width * 3);

	// inputDataHost: 0.890196 0.835294 0.603922 0.694118 0.341176
	printf("inputDataHost: %f %f %f %f %f\n", inputDataHost[0],
		   inputDataHost[m_width * m_height + 100],
		   inputDataHost[m_width * m_height + 101],
		   inputDataHost[m_width * m_height + 102],
		   inputDataHost[3 * m_width * m_height - 1]);
	free(inputDataHost);
	free(rgbaBuffer);
#endif  // DEBUG_TENSORRT_INFERENCE

	// Unmap resources
	cudaGraphicsUnmapResources(1, &pboRes);

	// Transfer TensorRT output to CPU
	cudaMemcpy(outputBuffer, outputDataDevice, m_totalOutputSize,
			   cudaMemcpyDeviceToHost);

#ifdef DEBUG_TENSORRT_INFERENCE
	// copy output to host and print activations (useful for debugging)
	float *outputDataHost = (float *)malloc(m_totalOutputSize);
	memset(outputDataHost, 0, m_totalOutputSize);
	cudaMemcpy(outputDataHost, outputDataDevice, m_totalOutputSize,
			   cudaMemcpyDeviceToHost);

	// 905:0.520552 633:0.028283 808:0.017819 316:0.015341 906:0.014210
	printf("\n");
	for (int i = 0; i < 5; i++)
	{
		int max_i = std::max_element(outputDataHost, outputDataHost + m_totalOutputSize / sizeof(float)) -
					outputDataHost;
		printf("%d:%f ", max_i, outputDataHost[max_i]);
		outputDataHost[max_i] = 0;
	}
	printf("\n");
	free(outputDataHost);
#endif  // DEBUG_TENSORRT_INFERENCE
}
