
#include <MiniCL/cl.h>
#define __PHYSICS_COMMON_H__ 1
#ifdef WIN32
#include "BulletMultiThreaded/Win32ThreadSupport.h"
#else
#include "BulletMultiThreaded/SequentialThreadSupport.h"
#endif
#include "BulletMultiThreaded/MiniCLTaskScheduler.h"
#include "BulletMultiThreaded/MiniCLTask/MiniCLTask.h"
#include "LinearMath/btMinMax.h"

/*
	m_threadSupportCollision = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
								"collision",
								processCollisionTask,
								createCollisionLocalStoreMemory,
								maxNumOutstandingTasks));

	if (!m_spuCollisionTaskProcess)
			m_spuCollisionTaskProcess = new SpuCollisionTaskProcess(m_threadInterface,m_maxNumOutstandingTasks);
	
		m_spuCollisionTaskProcess->initialize2(dispatchInfo.m_useEpa);

		m_spuCollisionTaskProcess->addWorkToTask(pairPtr,i,endIndex);

		//make sure all SPU work is done
		m_spuCollisionTaskProcess->flush2();


*/



CL_API_ENTRY cl_int CL_API_CALL clGetDeviceInfo(
	cl_device_id            device ,
	cl_device_info          param_name ,
	size_t                  param_value_size ,
	void *                  param_value ,
	size_t *                /* param_value_size_ret */) CL_API_SUFFIX__VERSION_1_0
{

	switch (param_name)
	{
	case CL_DEVICE_NAME:
		{
			char deviceName[] = "CPU";
			int nameLen = strlen(deviceName)+1;
			assert(param_value_size>strlen(deviceName));
			if (nameLen < param_value_size)
			{
				sprintf((char*)param_value,"CPU");
			} else
			{
				printf("error: param_value_size should be at least %d, but it is %d\n",nameLen,param_value_size);
			}
			break;
		}
	case CL_DEVICE_TYPE:
		{
			if (param_value_size>=sizeof(cl_device_type))
			{
				cl_device_type* deviceType = (cl_device_type*)param_value;
				*deviceType = CL_DEVICE_TYPE_CPU;
			} else
			{
				printf("error: param_value_size should be at least %d\n",sizeof(cl_device_type));
			}
			break;
		}
	case CL_DEVICE_MAX_COMPUTE_UNITS:
		{
			if (param_value_size>=sizeof(cl_uint))
			{
				cl_uint* numUnits = (cl_uint*)param_value;
				*numUnits= 4;
			} else
			{
				printf("error: param_value_size should be at least %d\n",sizeof(cl_uint));
			}

			break;
		}
	case CL_DEVICE_MAX_WORK_ITEM_SIZES:
		{
			size_t workitem_size[3];

			if (param_value_size>=sizeof(workitem_size))
			{
				size_t* workItemSize = (size_t*)param_value;
				workItemSize[0] = 64;
				workItemSize[1] = 24;
				workItemSize[2] = 16;
			} else
			{
				printf("error: param_value_size should be at least %d\n",sizeof(cl_uint));
			}
			break;
		}
	default:
		{
			printf("error: unsupported param_name:%d\n",param_name);
		}
	}


	return 0;
}

CL_API_ENTRY cl_int CL_API_CALL clReleaseMemObject(cl_mem /* memobj */) CL_API_SUFFIX__VERSION_1_0
{
	return 0;
}



CL_API_ENTRY cl_int CL_API_CALL clReleaseCommandQueue(cl_command_queue /* command_queue */) CL_API_SUFFIX__VERSION_1_0
{
	return 0;
}

CL_API_ENTRY cl_int CL_API_CALL clReleaseProgram(cl_program /* program */) CL_API_SUFFIX__VERSION_1_0
{
	return 0;
}

CL_API_ENTRY cl_int CL_API_CALL clReleaseKernel(cl_kernel   /* kernel */) CL_API_SUFFIX__VERSION_1_0
{
	return 0;
}


// Enqueued Commands APIs
CL_API_ENTRY cl_int CL_API_CALL clEnqueueReadBuffer(cl_command_queue     command_queue ,
                    cl_mem               buffer ,
                    cl_bool             /* blocking_read */,
                    size_t              /* offset */,
                    size_t               cb , 
                    void *               ptr ,
                    cl_uint             /* num_events_in_wait_list */,
                    const cl_event *    /* event_wait_list */,
                    cl_event *          /* event */) CL_API_SUFFIX__VERSION_1_0
{
	MiniCLTaskScheduler* scheduler = (MiniCLTaskScheduler*) command_queue;

	///wait for all work items to be completed
	scheduler->flush();

	memcpy(ptr,buffer,cb);
	return 0;
}


CL_API_ENTRY cl_int CL_API_CALL clEnqueueNDRangeKernel(cl_command_queue /* command_queue */,
                       cl_kernel         clKernel ,
                       cl_uint           work_dim ,
                       const size_t *   /* global_work_offset */,
                       const size_t *    global_work_size ,
                       const size_t *   /* local_work_size */,
                       cl_uint          /* num_events_in_wait_list */,
                       const cl_event * /* event_wait_list */,
                       cl_event *       /* event */) CL_API_SUFFIX__VERSION_1_0
{

	
	MiniCLKernel* kernel = (MiniCLKernel*) clKernel;
	for (int ii=0;ii<work_dim;ii++)
	{
		int maxTask = kernel->m_scheduler->getMaxNumOutstandingTasks();
		int numWorkItems = global_work_size[ii];

		//at minimum 64 work items per task
		int numWorkItemsPerTask = btMax(64,numWorkItems / maxTask);

		for (int t=0;t<numWorkItems;)
		{
			//Performance Hint: tweak this number during benchmarking
			int endIndex = (t+numWorkItemsPerTask) < numWorkItems ? t+numWorkItemsPerTask : numWorkItems;
			kernel->m_scheduler->issueTask(t,endIndex,kernel->m_kernelProgramCommandId,(char*)&kernel->m_argData[0][0],kernel->m_argSizes);
			t = endIndex;
		}
	}
/*

	void* bla = 0;

	scheduler->issueTask(bla,2,3);
	scheduler->flush();

	*/

	return 0;
}

CL_API_ENTRY cl_int CL_API_CALL clSetKernelArg(cl_kernel    clKernel ,
               cl_uint      arg_index ,
               size_t       arg_size ,
               const void *  arg_value ) CL_API_SUFFIX__VERSION_1_0
{
	MiniCLKernel* kernel = (MiniCLKernel* ) clKernel;
	assert(arg_size < MINICL_MAX_ARGLENGTH);
	if (arg_index>MINI_CL_MAX_ARG)
	{
		printf("error: clSetKernelArg arg_index (%d) exceeds %d\n",arg_index,MINI_CL_MAX_ARG);
	} else
	{
		if (arg_size>=MINICL_MAX_ARGLENGTH)
		{
			printf("error: clSetKernelArg argdata too large: %d (maximum is %d)\n",arg_size,MINICL_MAX_ARGLENGTH);
		} else
		{
			memcpy(	kernel->m_argData[arg_index],arg_value,arg_size);
			kernel->m_argSizes[arg_index] = arg_size;
		}
	}
	return 0;
}

// Kernel Object APIs
CL_API_ENTRY cl_kernel CL_API_CALL clCreateKernel(cl_program       program ,
               const char *     kernel_name ,
               cl_int *        /* errcode_ret */) CL_API_SUFFIX__VERSION_1_0
{
	MiniCLTaskScheduler* scheduler = (MiniCLTaskScheduler*) program;
	MiniCLKernel* kernel = new MiniCLKernel();

	kernel->m_kernelProgramCommandId = scheduler->findProgramCommandIdByName(kernel_name);
	kernel->m_scheduler = scheduler;

	return (cl_kernel)kernel;

}


CL_API_ENTRY cl_int CL_API_CALL clBuildProgram(cl_program           /* program */,
               cl_uint              /* num_devices */,
               const cl_device_id * /* device_list */,
               const char *         /* options */, 
               void (*pfn_notify)(cl_program /* program */, void * /* user_data */),
               void *               /* user_data */) CL_API_SUFFIX__VERSION_1_0
{
	return 0;
}

CL_API_ENTRY cl_program CL_API_CALL clCreateProgramWithBinary(cl_context                     context ,
                          cl_uint                        /* num_devices */,
                          const cl_device_id *           /* device_list */,
                          const size_t *                 /* lengths */,
                          const unsigned char **         /* binaries */,
                          cl_int *                       /* binary_status */,
                          cl_int *                       /* errcode_ret */) CL_API_SUFFIX__VERSION_1_0
{
	return (cl_program)context;
}


// Memory Object APIs
CL_API_ENTRY cl_mem CL_API_CALL clCreateBuffer(cl_context   /* context */,
               cl_mem_flags flags ,
               size_t       size,
               void *       host_ptr ,
               cl_int *     errcode_ret ) CL_API_SUFFIX__VERSION_1_0
{
	cl_mem buf = (cl_mem)malloc(size);
	if ((flags&CL_MEM_COPY_HOST_PTR) && host_ptr)
	{
		memcpy(buf,host_ptr,size);
	}
	return buf;
}

// Command Queue APIs
CL_API_ENTRY cl_command_queue CL_API_CALL clCreateCommandQueue(cl_context                      context , 
                     cl_device_id                   /* device */, 
                     cl_command_queue_properties    /* properties */,
                     cl_int *                       /* errcode_ret */) CL_API_SUFFIX__VERSION_1_0
{
	return (cl_command_queue) context;
}

extern CL_API_ENTRY cl_int CL_API_CALL clGetContextInfo(cl_context         /* context */, 
                 cl_context_info    param_name , 
                 size_t             param_value_size , 
                 void *             param_value, 
                 size_t *           param_value_size_ret ) CL_API_SUFFIX__VERSION_1_0
{

	switch (param_name)
	{
	case CL_CONTEXT_DEVICES:
		{
			if (!param_value_size)
			{
				*param_value_size_ret = 13;
			} else
			{
				sprintf((char*)param_value,"MiniCL_Test.");
			}
			break;
		};
	default:
		{
			printf("unsupported\n");
		}
	}
	
	return 0;
}

CL_API_ENTRY cl_context CL_API_CALL clCreateContextFromType(cl_context_properties * /* properties */,
                        cl_device_type          /* device_type */,
                        void (*pfn_notify)(const char *, const void *, size_t, void *) /* pfn_notify */,
                        void *                  /* user_data */,
                        cl_int *                /* errcode_ret */) CL_API_SUFFIX__VERSION_1_0
{
	int maxNumOutstandingTasks = 4;

#ifdef WIN32
	Win32ThreadSupport* threadSupport = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
								"MiniCL",
								processMiniCLTask, //processCollisionTask,
								createMiniCLLocalStoreMemory,//createCollisionLocalStoreMemory,
								maxNumOutstandingTasks));
#else
	SequentialThreadSupport::SequentialThreadConstructionInfo stc("MiniCL",processMiniCLTask,createMiniCLLocalStoreMemory);
	SequentialThreadSupport* threadSupport = new SequentialThreadSupport(stc);
	
#endif
	
	
	MiniCLTaskScheduler* scheduler = new MiniCLTaskScheduler(threadSupport,maxNumOutstandingTasks);

	return (cl_context)scheduler;
}

CL_API_ENTRY cl_int CL_API_CALL clReleaseContext(cl_context  context ) CL_API_SUFFIX__VERSION_1_0
{

	MiniCLTaskScheduler* scheduler = (MiniCLTaskScheduler*) context;
	
	btThreadSupportInterface* threadSupport = scheduler->getThreadSupportInterface();
	delete scheduler;
	delete threadSupport;
	
	return 0;
}
