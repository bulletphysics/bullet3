
#ifndef B3_LAUNCHER_CL_H
#define B3_LAUNCHER_CL_H

#include "b3BufferInfoCL.h"
#include "Bullet3Common/b3MinMax.h"
#include "b3OpenCLArray.h"
#include <stdio.h>

#define B3_DEBUG_SERIALIZE_CL


#ifdef _WIN32
#pragma warning(disable :4996)
#endif
#define B3_CL_MAX_ARG_SIZE 16
B3_ATTRIBUTE_ALIGNED16(struct) b3KernelArgData
{
    int m_isBuffer;
    int m_argIndex;
    int m_argSizeInBytes;
	int m_unusedPadding;
    union
    {
        cl_mem m_clBuffer;
        unsigned char m_argData[B3_CL_MAX_ARG_SIZE];
    };
    
};

class b3LauncherCL
{

	cl_command_queue m_commandQueue;
	cl_kernel m_kernel;
	int m_idx;
	
    b3AlignedObjectArray<b3KernelArgData> m_kernelArguments;
    int m_serializationSizeInBytes;
	bool	m_enableSerialization;

	public:

     b3AlignedObjectArray<b3OpenCLArray<unsigned char>* > m_arrays;
    
		b3LauncherCL(cl_command_queue queue, cl_kernel kernel)
			:m_commandQueue(queue),
			m_kernel(kernel),
			m_idx(0),
			m_enableSerialization(false)
		{
            m_serializationSizeInBytes = sizeof(int);
		}
    
        virtual ~b3LauncherCL()
        {
            for (int i=0;i<m_arrays.size();i++)
            {
                clReleaseMemObject(m_arrays[i]->getBufferCL());
            }
        }

		inline void setBuffer( cl_mem clBuffer)
		{
				if (m_enableSerialization)
				{
					b3KernelArgData kernelArg;
					kernelArg.m_argIndex = m_idx;
					kernelArg.m_isBuffer = 1;
					kernelArg.m_clBuffer = clBuffer;
				
					cl_mem_info param_name = CL_MEM_SIZE;
					size_t param_value;
					size_t sizeInBytes = sizeof(size_t);
					size_t actualSizeInBytes;
					cl_int err;
					err = clGetMemObjectInfo (	kernelArg.m_clBuffer,
											  param_name,
											  sizeInBytes,
											  &param_value,
											  &actualSizeInBytes);
					
					b3Assert( err == CL_SUCCESS );
					kernelArg.m_argSizeInBytes = param_value;
					
					m_kernelArguments.push_back(kernelArg);
					m_serializationSizeInBytes+= sizeof(b3KernelArgData);
					m_serializationSizeInBytes+=param_value;
                }
                cl_int status = clSetKernelArg( m_kernel, m_idx++, sizeof(cl_mem), &clBuffer);
				b3Assert( status == CL_SUCCESS );
            }


		inline void setBuffers( b3BufferInfoCL* buffInfo, int n )
		{
			for(int i=0; i<n; i++)
			{
				if (m_enableSerialization)
				{
					b3KernelArgData kernelArg;
					kernelArg.m_argIndex = m_idx;
					kernelArg.m_isBuffer = 1;
					kernelArg.m_clBuffer = buffInfo[i].m_clBuffer;
				
					cl_mem_info param_name = CL_MEM_SIZE;
					size_t param_value;
					size_t sizeInBytes = sizeof(size_t);
					size_t actualSizeInBytes;
					cl_int err;
					err = clGetMemObjectInfo (	kernelArg.m_clBuffer,
											  param_name,
											  sizeInBytes,
											  &param_value,
											  &actualSizeInBytes);
					
					b3Assert( err == CL_SUCCESS );
					kernelArg.m_argSizeInBytes = param_value;
					
					m_kernelArguments.push_back(kernelArg);
					m_serializationSizeInBytes+= sizeof(b3KernelArgData);
					m_serializationSizeInBytes+=param_value;
                }
                cl_int status = clSetKernelArg( m_kernel, m_idx++, sizeof(cl_mem), &buffInfo[i].m_clBuffer);
				b3Assert( status == CL_SUCCESS );
            }
		}
    
    int getSerializationBufferSize() const 
    {
        return m_serializationSizeInBytes;
    }
    
    inline int deserializeArgs(unsigned char* buf, int bufSize, cl_context ctx)
    {
        int index=0;
        
        int numArguments = *(int*) &buf[index];
        index+=sizeof(int);
        
        for (int i=0;i<numArguments;i++)
        {
            b3KernelArgData* arg = (b3KernelArgData*)&buf[index];

            index+=sizeof(b3KernelArgData);
            if (arg->m_isBuffer)
            {
                b3OpenCLArray<unsigned char>* clData = new b3OpenCLArray<unsigned char>(ctx,m_commandQueue, arg->m_argSizeInBytes);
                clData->resize(arg->m_argSizeInBytes);
                
                clData->copyFromHostPointer(&buf[index], arg->m_argSizeInBytes);
                
                arg->m_clBuffer = clData->getBufferCL();
                
                m_arrays.push_back(clData);
                
                cl_int status = clSetKernelArg( m_kernel, m_idx++, sizeof(cl_mem), &arg->m_clBuffer);
				b3Assert( status == CL_SUCCESS );
                index+=arg->m_argSizeInBytes;
            } else 
            {
                cl_int status = clSetKernelArg( m_kernel, m_idx++, arg->m_argSizeInBytes, &arg->m_argData);
				b3Assert( status == CL_SUCCESS );
            }
			m_kernelArguments.push_back(*arg);
        }
		m_serializationSizeInBytes = index;
        return index;
    }

	inline int validateResults(unsigned char* goldBuffer, int goldBufferCapacity, cl_context ctx)
    {
		 int index=0;
        
        int numArguments = *(int*) &goldBuffer[index];
        index+=sizeof(int);

		if (numArguments != m_kernelArguments.size())
		{
			printf("failed validation: expected %d arguments, found %d\n",numArguments, m_kernelArguments.size());
			return -1;
		}
        
        for (int ii=0;ii<numArguments;ii++)
        {
            b3KernelArgData* argGold = (b3KernelArgData*)&goldBuffer[index];

			if (m_kernelArguments[ii].m_argSizeInBytes != argGold->m_argSizeInBytes)
			{
				printf("failed validation: argument %d sizeInBytes expected: %d, found %d\n",ii, argGold->m_argSizeInBytes, m_kernelArguments[ii].m_argSizeInBytes);
				return -2;
			}

			{
				int expected = argGold->m_isBuffer;
				int found = m_kernelArguments[ii].m_isBuffer;

				if (expected != found)
				{
					printf("failed validation: argument %d isBuffer expected: %d, found %d\n",ii,expected, found);
					return -3;
				}
			}
			index+=sizeof(b3KernelArgData);

			if (argGold->m_isBuffer)
            {

				unsigned char* memBuf= (unsigned char*) malloc(m_kernelArguments[ii].m_argSizeInBytes);
				unsigned char* goldBuf = &goldBuffer[index];
				for (int j=0;j<m_kernelArguments[j].m_argSizeInBytes;j++)
				{
					memBuf[j] = 0xaa;
				}

				cl_int status = 0;
				status = clEnqueueReadBuffer( m_commandQueue, m_kernelArguments[ii].m_clBuffer, CL_TRUE, 0, m_kernelArguments[ii].m_argSizeInBytes,
                                             memBuf, 0,0,0 );
                b3Assert( status==CL_SUCCESS );
                clFinish(m_commandQueue);

				for (int b=0;b<m_kernelArguments[ii].m_argSizeInBytes;b++)
				{
					int expected = goldBuf[b];
					int found = memBuf[b];
					if (expected != found)
					{
						printf("failed validation: argument %d OpenCL data at byte position %d expected: %d, found %d\n",
							ii, b, expected, found);
						return -4;
					}
				}

                
                index+=argGold->m_argSizeInBytes;
            } else 
            {
				
				//compare content
				for (int b=0;b<m_kernelArguments[ii].m_argSizeInBytes;b++)
				{
					int expected = argGold->m_argData[b];
					int found =m_kernelArguments[ii].m_argData[b];
					if (expected != found)
					{
						printf("failed validation: argument %d const data at byte position %d expected: %d, found %d\n",
							ii, b, expected, found);
						return -5;
					}
				}

            }
        }
        return index;
  
	}

    inline int serializeArguments(unsigned char* destBuffer, int destBufferCapacity)
    {
		//initialize to known values
		for (int i=0;i<destBufferCapacity;i++)
			destBuffer[i] = 0xec;

        assert(destBufferCapacity>=m_serializationSizeInBytes);
        
        //todo: use the b3Serializer for this to allow for 32/64bit, endianness etc        
        int numArguments = m_kernelArguments.size();
        int curBufferSize = 0;
        int* dest = (int*)&destBuffer[curBufferSize];
        *dest = numArguments;
        curBufferSize += sizeof(int);
        
        
        
        for (int i=0;i<this->m_kernelArguments.size();i++)
        {
            b3KernelArgData* arg = (b3KernelArgData*) &destBuffer[curBufferSize];
            *arg = m_kernelArguments[i];
            curBufferSize+=sizeof(b3KernelArgData);
            if (arg->m_isBuffer==1)
            {
                //copy the OpenCL buffer content
                cl_int status = 0;
                status = clEnqueueReadBuffer( m_commandQueue, arg->m_clBuffer, 0, 0, arg->m_argSizeInBytes,
                                             &destBuffer[curBufferSize], 0,0,0 );
                b3Assert( status==CL_SUCCESS );
                clFinish(m_commandQueue);
                curBufferSize+=arg->m_argSizeInBytes;
            }
            
        }
        return curBufferSize;
    }
	
	void serializeToFile(const char* fileName, int numWorkItems)
	{
		int num = numWorkItems;
		int buffSize = getSerializationBufferSize();
		unsigned char* buf = new unsigned char[buffSize+sizeof(int)];
		for (int i=0;i<buffSize+1;i++)
		{
			unsigned char* ptr = (unsigned char*)&buf[i];
			*ptr = 0xff;
		}
	//	int actualWrite = serializeArguments(buf,buffSize);
                
	//	unsigned char* cptr = (unsigned char*)&buf[buffSize];
	//            printf("buf[buffSize] = %d\n",*cptr);
                
		assert(buf[buffSize]==0xff);//check for buffer overrun
		int* ptr = (int*)&buf[buffSize];
                
		*ptr = num;
                
		FILE* f = fopen(fileName,"wb");
		fwrite(buf,buffSize+sizeof(int),1,f);
		fclose(f);

		delete[] buf;
	}		


	template<typename T>
		inline void setConst( const T& consts )
		{
			int sz=sizeof(T);
			b3Assert(sz<=B3_CL_MAX_ARG_SIZE);

			if (m_enableSerialization)
			{
				b3KernelArgData kernelArg;
				kernelArg.m_argIndex = m_idx;
				kernelArg.m_isBuffer = 0;
				T* destArg = (T*)kernelArg.m_argData;
				*destArg = consts;
				kernelArg.m_argSizeInBytes = sizeof(T);
				m_kernelArguments.push_back(kernelArg);
				m_serializationSizeInBytes+=sizeof(b3KernelArgData);
			}
            
			cl_int status = clSetKernelArg( m_kernel, m_idx++, sz, &consts );
			b3Assert( status == CL_SUCCESS );
		}

		inline void launch1D( int numThreads, int localSize = 64)
		{
			launch2D( numThreads, 1, localSize, 1 );
		}

		inline void launch2D( int numThreadsX, int numThreadsY, int localSizeX, int localSizeY )
		{
			size_t gRange[3] = {1,1,1};
			size_t lRange[3] = {1,1,1};
			lRange[0] = localSizeX;
			lRange[1] = localSizeY;
			gRange[0] = b3Max((size_t)1, (numThreadsX/lRange[0])+(!(numThreadsX%lRange[0])?0:1));
			gRange[0] *= lRange[0];
			gRange[1] = b3Max((size_t)1, (numThreadsY/lRange[1])+(!(numThreadsY%lRange[1])?0:1));
			gRange[1] *= lRange[1];

			cl_int status = clEnqueueNDRangeKernel( m_commandQueue, 
				m_kernel, 2, NULL, gRange, lRange, 0,0,0 );
            if (status != CL_SUCCESS)
            {
                printf("Error: OpenCL status = %d\n",status);
            }
			b3Assert( status == CL_SUCCESS );

		}
	
		void	enableSerialization(bool serialize)
		{
			m_enableSerialization = serialize;
		}
		
};



#endif //B3_LAUNCHER_CL_H
