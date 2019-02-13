//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.


#ifndef PXCUDACONTEXTMANAGER_PXCUDACONTEXTMANAGER_H
#define PXCUDACONTEXTMANAGER_PXCUDACONTEXTMANAGER_H

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxErrorCallback.h"
#include "foundation/PxFlags.h"
#include "task/PxTaskDefine.h"
#include "cudamanager/PxCudaMemoryManager.h"

/* Forward decl to avoid inclusion of cuda.h */
typedef struct CUctx_st *CUcontext;
typedef struct CUgraphicsResource_st *CUgraphicsResource;
typedef int CUdevice;

namespace physx
{ 
	
class PxGpuDispatcher;


/** \brief Possible graphic/CUDA interoperability modes for context */
struct PxCudaInteropMode
{
    /**
     * \brief Possible graphic/CUDA interoperability modes for context
     */
	enum Enum
	{
		NO_INTEROP = 0,
		D3D10_INTEROP,
		D3D11_INTEROP,
		OGL_INTEROP,

		COUNT
	};
};

struct PxCudaInteropRegisterFlag
{
	enum Enum
	{
		eNONE           = 0x00,
		eREAD_ONLY      = 0x01,
		eWRITE_DISCARD  = 0x02,
		eSURFACE_LDST   = 0x04,
		eTEXTURE_GATHER = 0x08
	};
};

/**
\brief collection of set bits defined in NxCudaInteropRegisterFlag.

@see NxCudaInteropRegisterFlag
*/
typedef PxFlags<PxCudaInteropRegisterFlag::Enum, uint32_t> PxCudaInteropRegisterFlags;
PX_FLAGS_OPERATORS(PxCudaInteropRegisterFlag::Enum, uint32_t)

//! \brief Descriptor used to create a PxCudaContextManager
class PxCudaContextManagerDesc
{
public:
    /**
     * \brief The CUDA context to manage
     *
     * If left NULL, the PxCudaContextManager will create a new context.  If
     * graphicsDevice is also not NULL, this new CUDA context will be bound to
     * that graphics device, enabling the use of CUDA/Graphics interop features.
     *
     * If ctx is not NULL, the specified context must be applied to the thread
     * that is allocating the PxCudaContextManager at creation time (aka, it
     * cannot be popped).  The PxCudaContextManager will take ownership of the
     * context until the manager is released.  All access to the context must be
     * gated by lock acquisition.
     *
     * If the user provides a context for the PxCudaContextManager, the context
     * _must_ have either been created on the GPU ordinal returned by
     * PxGetSuggestedCudaDeviceOrdinal() or on your graphics device.
     *
     * It is perfectly acceptable to allocate device or host pinned memory from
     * the context outside the scope of the PxCudaMemoryManager, so long as you
     * manage its eventual cleanup.
     */
	CUcontext            *ctx;

    /**
     * \brief D3D device pointer or OpenGl context handle
     *
     * Only applicable when ctx is NULL, thus forcing a new context to be
     * created.  In that case, the created context will be bound to this
     * graphics device.
     */
	void	             *graphicsDevice;

#if PX_SUPPORT_GPU_PHYSX
	/**
	  * \brief Application-specific GUID
	  *
	  * If your application employs PhysX modules that use CUDA you need to use a GUID 
	  * so that patches for new architectures can be released for your game.You can obtain a GUID for your 
	  * application from Nvidia.
	  */
	const char*			 appGUID;
#endif
    /**
     * \brief The CUDA/Graphics interop mode of this context
     *
     * If ctx is NULL, this value describes the nature of the graphicsDevice
     * pointer provided by the user.  Else it describes the nature of the
     * context provided by the user.
     */
	PxCudaInteropMode::Enum interopMode;


    /**
     * \brief Size of persistent memory
     *
     * This memory is allocated up front and stays allocated until the
     * PxCudaContextManager is released.  Size is in bytes, has to be power of two
     * and bigger than the page size.  Set to 0 to only use dynamic pages.
     *
     * Note: On Vista O/S and above, there is a per-memory allocation overhead
     * to every CUDA work submission, so we recommend that you carefully tune
     * this initial base memory size to closely approximate the amount of
     * memory your application will consume.

	 Note: This is currently not used by PxSceneFlag::eENABLE_GPU_DYNAMICS. Memory allocation properties are configured
	 for GPU rigid bodies using PxSceneDesc::gpuDynamicsConfig.
     */
	uint32_t	memoryBaseSize[PxCudaBufferMemorySpace::COUNT];

    /**
     * \brief Size of memory pages
     *
     * The memory manager will dynamically grow and shrink in blocks multiple of
     * this page size. Size has to be power of two and bigger than 0.

	Note: This is currently not used by PxSceneFlag::eENABLE_GPU_DYNAMICS. Memory allocation properties are configured
	for GPU rigid bodies using PxSceneDesc::gpuDynamicsConfig.
     */
	uint32_t	memoryPageSize[PxCudaBufferMemorySpace::COUNT];

    /**
     * \brief Maximum size of memory that the memory manager will allocate

	 Note: This is currently not used by PxSceneFlag::eENABLE_GPU_DYNAMICS. Memory allocation properties are configured
	 for GPU rigid bodies using PxSceneDesc::gpuDynamicsConfig.
     */
	uint32_t	maxMemorySize[PxCudaBufferMemorySpace::COUNT];

	PX_INLINE PxCudaContextManagerDesc()
	{
		ctx = NULL;
		interopMode = PxCudaInteropMode::NO_INTEROP;
		graphicsDevice = 0;
#if PX_SUPPORT_GPU_PHYSX
		appGUID  = NULL;
#endif
		for(uint32_t i = 0; i < PxCudaBufferMemorySpace::COUNT; i++)
		{
			memoryBaseSize[i] = 0;
			memoryPageSize[i] = 2 * 1024*1024;
			maxMemorySize[i] = UINT32_MAX;
		}
	}
};


/**
 * \brief Manages memory, thread locks, and task scheduling for a CUDA context
 *
 * A PxCudaContextManager manages access to a single CUDA context, allowing it to
 * be shared between multiple scenes.   Memory allocations are dynamic: starting
 * with an initial heap size and growing on demand by a configurable page size.
 * The context must be acquired from the manager before using any CUDA APIs.
 *
 * The PxCudaContextManager is based on the CUDA driver API and explictly does not
 * support the CUDA runtime API (aka, CUDART).
 *
 * To enable CUDA use by an APEX scene, a PxCudaContextManager must be created
 * (supplying your own CUDA context, or allowing a new context to be allocated
 * for you), the PxGpuDispatcher for that context is retrieved via the
 * getGpuDispatcher() method, and this is assigned to the TaskManager that is
 * given to the scene via its NxApexSceneDesc.
 */
class PxCudaContextManager
{
public:
    /**
     * \brief Acquire the CUDA context for the current thread
     *
     * Acquisitions are allowed to be recursive within a single thread.
     * You can acquire the context multiple times so long as you release
     * it the same count.
     *
     * The context must be acquired before using most CUDA functions.
     *
     * It is not necessary to acquire the CUDA context inside GpuTask
     * launch functions, because the PxGpuDispatcher will have already
     * acquired the context for its worker thread.  However it is not
     * harmfull to (re)acquire the context in code that is shared between
     * GpuTasks and non-task functions.
     */
    virtual void acquireContext() = 0;

    /**
     * \brief Release the CUDA context from the current thread
     *
     * The CUDA context should be released as soon as practically
     * possible, to allow other CPU threads (including the
     * PxGpuDispatcher) to work efficiently.
     */
    virtual void releaseContext() = 0;

	/**
	* \brief Return the CUcontext
	*/
	virtual CUcontext getContext() = 0;

    /**
     * \brief Return the PxCudaMemoryManager instance associated with this
     * CUDA context
	 * Note: This is currently not used by PxSceneFlag::eENABLE_GPU_DYNAMICS. Memory allocation properties are configured
	 * for GPU rigid bodies using PxSceneDesc::gpuDynamicsConfig.
     */
	virtual PxCudaMemoryManager *getMemoryManager() = 0;

    /**
     * \brief Return the PxGpuDispatcher instance associated with this
     * CUDA context
     */
	virtual class physx::PxGpuDispatcher *getGpuDispatcher() = 0;

    /**
     * \brief Context manager has a valid CUDA context
     *
     * This method should be called after creating a PxCudaContextManager,
     * especially if the manager was responsible for allocating its own
     * CUDA context (desc.ctx == NULL).  If it returns false, there is
     * no point in assigning this manager's PxGpuDispatcher to a
     * TaskManager as it will be unable to execute GpuTasks.
     */
    virtual bool contextIsValid() const = 0;

	/* Query CUDA context and device properties, without acquiring context */

    virtual bool supportsArchSM10() const = 0;  //!< G80
    virtual bool supportsArchSM11() const = 0;  //!< G92
    virtual bool supportsArchSM12() const = 0;  //!< GT200
    virtual bool supportsArchSM13() const = 0;  //!< GT260
    virtual bool supportsArchSM20() const = 0;  //!< GF100
    virtual bool supportsArchSM30() const = 0;  //!< GK100
	virtual bool supportsArchSM35() const = 0;  //!< GK110
	virtual bool supportsArchSM50() const = 0;  //!< GM100
	virtual bool supportsArchSM52() const = 0;  //!< GM200
	virtual bool supportsArchSM60() const = 0;  //!< GP100
	virtual bool isIntegrated() const = 0;      //!< true if GPU is an integrated (MCP) part
	virtual bool canMapHostMemory() const = 0;  //!< true if GPU map host memory to GPU (0-copy)
	virtual int  getDriverVersion() const = 0;  //!< returns cached value of cuGetDriverVersion()
	virtual size_t getDeviceTotalMemBytes() const = 0; //!< returns cached value of device memory size
	virtual int	getMultiprocessorCount() const = 0; //!< returns cache value of SM unit count
    virtual unsigned int getClockRate() const = 0; //!< returns cached value of SM clock frequency
    virtual int  getSharedMemPerBlock() const = 0; //!< returns total amount of shared memory available per block in bytes
	virtual int  getSharedMemPerMultiprocessor() const = 0; //!< returns total amount of shared memory available per multiprocessor in bytes
	virtual unsigned int getMaxThreadsPerBlock() const = 0; //!< returns the maximum number of threads per block
    virtual const char *getDeviceName() const = 0; //!< returns device name retrieved from driver
	virtual CUdevice getDevice() const = 0; //!< returns device handle retrieved from driver
	virtual PxCudaInteropMode::Enum getInteropMode() const = 0; //!< interop mode the context was created with

	virtual void setUsingConcurrentStreams(bool) = 0; //!< turn on/off using concurrent streams for GPU work
	virtual bool getUsingConcurrentStreams() const = 0; //!< true if GPU work can run in concurrent streams
    /* End query methods that don't require context to be acquired */

    /**
     * \brief Register a rendering resource with CUDA
     *
     * This function is called to register render resources (allocated
     * from OpenGL) with CUDA so that the memory may be shared
     * between the two systems.  This is only required for render
     * resources that are designed for interop use.  In APEX, each
     * render resource descriptor that could support interop has a
     * 'registerInCUDA' boolean variable.
     *
     * The function must be called again any time your graphics device
     * is reset, to re-register the resource.
     *
     * Returns true if the registration succeeded.  A registered
     * resource must be unregistered before it can be released.
     *
     * \param resource [OUT] the handle to the resource that can be used with CUDA
     * \param buffer [IN] GLuint buffer index to be mapped to cuda
     * \param flags [IN] cuda interop registration flags
     */
    virtual bool registerResourceInCudaGL(CUgraphicsResource &resource, uint32_t buffer, PxCudaInteropRegisterFlags flags = PxCudaInteropRegisterFlags()) = 0;

     /**
     * \brief Register a rendering resource with CUDA
     *
     * This function is called to register render resources (allocated
     * from Direct3D) with CUDA so that the memory may be shared
     * between the two systems.  This is only required for render
     * resources that are designed for interop use.  In APEX, each
     * render resource descriptor that could support interop has a
     * 'registerInCUDA' boolean variable.
     *
     * The function must be called again any time your graphics device
     * is reset, to re-register the resource.
     *
     * Returns true if the registration succeeded.  A registered
     * resource must be unregistered before it can be released.
     *
     * \param resource [OUT] the handle to the resource that can be used with CUDA
     * \param resourcePointer [IN] A pointer to either IDirect3DResource9, or ID3D10Device, or ID3D11Resource to be registered.
     * \param flags [IN] cuda interop registration flags
     */
    virtual bool registerResourceInCudaD3D(CUgraphicsResource &resource, void *resourcePointer, PxCudaInteropRegisterFlags flags = PxCudaInteropRegisterFlags()) = 0;

    /**
     * \brief Unregister a rendering resource with CUDA
     *
     * If a render resource was successfully registered with CUDA using
     * the registerResourceInCuda***() methods, this function must be called
     * to unregister the resource before the it can be released.
     */
    virtual bool unregisterResourceInCuda(CUgraphicsResource resource) = 0;

	/**
	 * \brief Determine if the user has configured a dedicated PhysX GPU in the NV Control Panel
	 * \note If using CUDA Interop, this will always return false
	 * \returns	1 if there is a dedicated GPU
	 *			0 if there is NOT a dedicated GPU
	 *			-1 if the routine is not implemented
	*/
	virtual int	usingDedicatedGPU() const = 0;

    /**
     * \brief Release the PxCudaContextManager
     *
     * When the manager instance is released, it also releases its
     * PxGpuDispatcher instance and PxCudaMemoryManager.  Before the memory
     * manager is released, it frees all allocated memory pages.  If the
     * PxCudaContextManager created the CUDA context it was responsible
     * for, it also frees that context.
     *
     * Do not release the PxCudaContextManager if there are any scenes
     * using its PxGpuDispatcher.  Those scenes must be released first
     * since there is no safe way to remove a PxGpuDispatcher from a
     * TaskManager once the TaskManager has been given to a scene.
     *
     */
	virtual void release() = 0;

protected:

    /**
     * \brief protected destructor, use release() method
     */
    virtual ~PxCudaContextManager() {}
};

/**
 * \brief Convenience class for holding CUDA lock within a scope
 */
class PxScopedCudaLock
{
public:
    /**
     * \brief ScopedCudaLock constructor
     */
	PxScopedCudaLock(PxCudaContextManager& ctx) : mCtx(&ctx)
	{
		mCtx->acquireContext();
	}

    /**
     * \brief ScopedCudaLock destructor
     */
	~PxScopedCudaLock()
	{
		mCtx->releaseContext();
	}

protected:

    /**
     * \brief CUDA context manager pointer (initialized in the constructor)
     */
    PxCudaContextManager* mCtx;
};

} // end physx namespace

#endif // PX_SUPPORT_GPU_PHYSX
#endif // PXCUDACONTEXTMANAGER_PXCUDACONTEXTMANAGER_H
