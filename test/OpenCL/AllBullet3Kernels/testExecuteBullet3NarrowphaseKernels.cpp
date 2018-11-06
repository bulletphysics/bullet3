
#include <gtest/gtest.h>
#include "Bullet3Common/b3Logging.h"

#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "Bullet3Common/b3CommandLineArgs.h"
#include "Bullet3OpenCL/NarrowphaseCollision/kernels/satKernels.h"
#include "Bullet3OpenCL/NarrowphaseCollision/kernels/mprKernels.h"
#include "Bullet3OpenCL/NarrowphaseCollision/kernels/satConcaveKernels.h"
#include "Bullet3OpenCL/NarrowphaseCollision/kernels/satClipHullContacts.h"
#include "Bullet3OpenCL/NarrowphaseCollision/kernels/bvhTraversal.h"
#include "Bullet3OpenCL/NarrowphaseCollision/kernels/primitiveContacts.h"

#ifdef B3_USE_ZLIB
#include "minizip/unzip.h"
#endif

#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"
extern int gArgc;
extern char** gArgv;

namespace
{
struct ExecuteBullet3NarrowphaseKernels : public ::testing::Test
{
	cl_context m_clContext;
	cl_device_id m_clDevice;
	cl_command_queue m_clQueue;
	char* m_clDeviceName;
	cl_platform_id m_platformId;

	ExecuteBullet3NarrowphaseKernels()
		: m_clDeviceName(0),
		  m_clContext(0),
		  m_clDevice(0),
		  m_clQueue(0),
		  m_platformId(0)
	{
		// You can do set-up work for each test here.

		initCL();
	}

	virtual ~ExecuteBullet3NarrowphaseKernels()
	{
		// You can do clean-up work that doesn't throw exceptions here.
		exitCL();
	}

	// If the constructor and destructor are not enough for setting up
	// and cleaning up each test, you can define the following methods:

#include "initCL.h"

	virtual void SetUp()
	{
		// Code here will be called immediately after the constructor (right
		// before each test).
	}

	virtual void TearDown()
	{
		// Code here will be called immediately after each test (right
		// before the destructor).
	}
};

#if 0
	TEST_F(ExecuteBullet3NarrowphaseKernels,satKernelsCL)
	{
		cl_int errNum=0;

		char flags[1024]={0};

		cl_program satProg = b3OpenCLUtils::compileCLProgramFromString(m_clContext,m_clDevice,satKernelsCL,&errNum,flags,0,true);
		ASSERT_EQ(CL_SUCCESS,errNum);


		{
			cl_kernel m_findSeparatingAxisKernel = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice,satKernelsCL, "findSeparatingAxisKernel",&errNum,satProg );
			ASSERT_EQ(CL_SUCCESS,errNum);
			clReleaseKernel(m_findSeparatingAxisKernel );
		}

		{
			cl_kernel m_findSeparatingAxisVertexFaceKernel = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice,satKernelsCL, "findSeparatingAxisVertexFaceKernel",&errNum,satProg );
			ASSERT_EQ(CL_SUCCESS,errNum);
			clReleaseKernel(m_findSeparatingAxisVertexFaceKernel);
		}

		{
			cl_kernel m_findSeparatingAxisEdgeEdgeKernel = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice,satKernelsCL, "findSeparatingAxisEdgeEdgeKernel",&errNum,satProg );
			ASSERT_EQ(CL_SUCCESS,errNum);
			clReleaseKernel(m_findSeparatingAxisEdgeEdgeKernel);
		}

		{
			cl_kernel m_findConcaveSeparatingAxisKernel = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice,satKernelsCL, "findConcaveSeparatingAxisKernel",&errNum,satProg );
			ASSERT_EQ(CL_SUCCESS,errNum);
			clReleaseKernel(m_findConcaveSeparatingAxisKernel );
		}


		{
			cl_kernel m_findCompoundPairsKernel = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice,satKernelsCL, "findCompoundPairsKernel",&errNum,satProg );
			ASSERT_EQ(CL_SUCCESS,errNum);
			clReleaseKernel(m_findCompoundPairsKernel);
		}

		{
			cl_kernel m_processCompoundPairsKernel = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice,satKernelsCL, "processCompoundPairsKernel",&errNum,satProg );
			ASSERT_EQ(CL_SUCCESS,errNum);
			clReleaseKernel(m_processCompoundPairsKernel);
		}

		clReleaseProgram(satProg);

	}

	TEST_F(ExecuteBullet3NarrowphaseKernels,satConcaveKernelsCL)
	{
		cl_int errNum=0;

		char flags[1024]={0};

		cl_program satConcaveProg = b3OpenCLUtils::compileCLProgramFromString(m_clContext,m_clDevice,satConcaveKernelsCL,&errNum,flags,0,true);
		ASSERT_EQ(CL_SUCCESS,errNum);

		{
			cl_kernel m_findConcaveSeparatingAxisVertexFaceKernel = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice,satConcaveKernelsCL, "findConcaveSeparatingAxisVertexFaceKernel",&errNum,satConcaveProg );
			ASSERT_EQ(CL_SUCCESS,errNum);
			clReleaseKernel(m_findConcaveSeparatingAxisVertexFaceKernel);
		}

		{
			cl_kernel m_findConcaveSeparatingAxisEdgeEdgeKernel = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice,satConcaveKernelsCL, "findConcaveSeparatingAxisEdgeEdgeKernel",&errNum,satConcaveProg );
			ASSERT_EQ(CL_SUCCESS,errNum);
			clReleaseKernel(m_findConcaveSeparatingAxisEdgeEdgeKernel);
		}

		clReleaseProgram(satConcaveProg);
	}


	TEST_F(ExecuteBullet3NarrowphaseKernels,satClipKernelsCL)
	{

		char flags[1024]={0};
		cl_int errNum=0;
//#ifdef CL_PLATFORM_INTEL
//		sprintf(flags,"-g -s \"%s\"","C:/develop/bullet3_experiments2/opencl/gpu_narrowphase/kernels/satClipHullContacts.cl");
//#endif

		cl_program satClipContactsProg = b3OpenCLUtils::compileCLProgramFromString(m_clContext,m_clDevice,satClipKernelsCL,&errNum,flags,0,true);
		ASSERT_EQ(CL_SUCCESS,errNum);

		{
			cl_kernel m_clipHullHullKernel = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice,satClipKernelsCL, "clipHullHullKernel",&errNum,satClipContactsProg);
			ASSERT_EQ(CL_SUCCESS,errNum);
			clReleaseKernel(m_clipHullHullKernel);
		}

		{
			cl_kernel m_clipCompoundsHullHullKernel = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice,satClipKernelsCL, "clipCompoundsHullHullKernel",&errNum,satClipContactsProg);
			ASSERT_EQ(CL_SUCCESS,errNum);
			clReleaseKernel(m_clipCompoundsHullHullKernel);
		}

		{
			cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice,satClipKernelsCL, "findClippingFacesKernel",&errNum,satClipContactsProg);
			ASSERT_EQ(CL_SUCCESS,errNum);
			clReleaseKernel(k);
		}

		{
			cl_kernel k  = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice,satClipKernelsCL, "clipFacesAndFindContactsKernel",&errNum,satClipContactsProg);
			ASSERT_EQ(CL_SUCCESS,errNum);
			clReleaseKernel(k);
		}

		{
			cl_kernel k  = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice,satClipKernelsCL, "clipHullHullConcaveConvexKernel",&errNum,satClipContactsProg);
			ASSERT_EQ(CL_SUCCESS,errNum);
			clReleaseKernel(k);
		}


        {
			cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice,satClipKernelsCL,
                            "newContactReductionKernel",&errNum,satClipContactsProg);
			ASSERT_EQ(CL_SUCCESS,errNum);
			clReleaseKernel(k);
		}

		clReleaseProgram(satClipContactsProg);
	}


	TEST_F(ExecuteBullet3NarrowphaseKernels,bvhTraversalKernels)
	{


		cl_int errNum=0;
		cl_program bvhTraversalProg = b3OpenCLUtils::compileCLProgramFromString(m_clContext,m_clDevice,bvhTraversalKernelCL,&errNum,"",0,true);
		ASSERT_EQ(CL_SUCCESS,errNum);

		{
			cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice,bvhTraversalKernelCL, "bvhTraversalKernel",&errNum,bvhTraversalProg,"");
			ASSERT_EQ(CL_SUCCESS,errNum);
			clReleaseKernel(k);
		}
		clReleaseProgram(bvhTraversalProg);
	}

	TEST_F(ExecuteBullet3NarrowphaseKernels,primitiveContactsKernelsCL)
	{
		cl_int errNum=0;
		cl_program primitiveContactsProg = b3OpenCLUtils::compileCLProgramFromString(m_clContext,m_clDevice,primitiveContactsKernelsCL,&errNum,"",0,true);
		ASSERT_EQ(CL_SUCCESS,errNum);


		{
			cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice,primitiveContactsKernelsCL, "primitiveContactsKernel",&errNum,primitiveContactsProg,"");
			ASSERT_EQ(CL_SUCCESS,errNum);
			clReleaseKernel(k);
		}

		{
			cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice,primitiveContactsKernelsCL, "findConcaveSphereContactsKernel",&errNum,primitiveContactsProg );
			ASSERT_EQ(CL_SUCCESS,errNum);
			clReleaseKernel(k);
		}

		{
			cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice,primitiveContactsKernelsCL, "processCompoundPairsPrimitivesKernel",&errNum,primitiveContactsProg,"");
			ASSERT_EQ(CL_SUCCESS,errNum);
			clReleaseKernel(k);
		}

		clReleaseProgram(primitiveContactsProg);
	}

#endif

unsigned char* openFile(const char* fileName, int* sizeInBytesPtr)
{
	*sizeInBytesPtr = 0;

	unsigned char* buffer = 0;
	const char* prefix[] = {"./", "./data/", "../data/", "../../data/", "../../../data/", "../../../../data/"};
	int numPrefixes = sizeof(prefix) / sizeof(const char*);
	char relativeFileName[1024];

#ifdef B3_USE_ZLIB

	{
		FILE* f = 0;
		int result = 0;

		for (int i = 0; !f && i < numPrefixes; i++)
		{
			sprintf(relativeFileName, "%s%s", prefix[i], "unittest_data.zip");
			f = fopen(relativeFileName, "rb");
		}
		if (f)
		{
			fclose(f);

			unzFile zipfile = unzOpen(relativeFileName);
			if (zipfile == NULL)
			{
				printf("%s: not found\n", relativeFileName);
			}

			// Get info about the zip file
			unz_global_info global_info;
			result = unzGetGlobalInfo(zipfile, &global_info);
			if (result != UNZ_OK)
			{
				b3Printf("could not read file global info\n");
				unzClose(zipfile);
			}
			else
			{
				result = unzLocateFile(zipfile, fileName, 0);
				if (result == UNZ_OK)
				{
					unz_file_info info;
					result = unzGetCurrentFileInfo(zipfile, &info, NULL, 0, NULL, 0, NULL, 0);
					if (result != UNZ_OK)
					{
						b3Printf("unzGetCurrentFileInfo() != UNZ_OK (%d)\n", result);
					}
					else
					{
						result = unzOpenCurrentFile(zipfile);
						if (result == UNZ_OK)
						{
							buffer = (unsigned char*)malloc(info.uncompressed_size);
							result = unzReadCurrentFile(zipfile, buffer, info.uncompressed_size);
							if (result < 0)
							{
								free(buffer);
								buffer = 0;
							}
							else
							{
								*sizeInBytesPtr = info.uncompressed_size;
							}
							unzCloseCurrentFile(zipfile);
						}
						else
						{
							b3Printf("cannot open file %s!\n", fileName);
						}
					}
				}
				else
				{
					b3Printf("cannot find file %s\n", fileName);
				}
				unzClose(zipfile);
			}
		}
	}
#endif  //B3_USE_ZLIB
	if (!buffer)
	{
		FILE* f = 0;
		int result = 0;

		for (int i = 0; !f && i < numPrefixes; i++)
		{
			sprintf(relativeFileName, "%s%s", prefix[i], fileName);
			f = fopen(relativeFileName, "rb");
		}
		//first try from data.zip, otherwise directly load the file from disk

		if (f)
		{
			int sizeInBytes = 0;
			if (fseek(f, 0, SEEK_END) || (sizeInBytes = ftell(f)) == EOF || fseek(f, 0, SEEK_SET))
			{
				b3Printf("error, cannot get file size\n");
			}

			buffer = (unsigned char*)malloc(sizeInBytes);
			int actualRead = fread(buffer, sizeInBytes, 1, f);
			if (actualRead != 1)
			{
				free(buffer);
				buffer = 0;
			}
			else
			{
				*sizeInBytesPtr = sizeInBytes;
			}
			fclose(f);
		}
	}

	return buffer;
}

void testLauncher(const char* fileName2, b3LauncherCL& launcher, cl_context ctx)
{
	int sizeInBytes = 0;

	unsigned char* buf = openFile(fileName2, &sizeInBytes);
	ASSERT_FALSE(buf == NULL);
	if (buf)
	{
		int serializedBytes = launcher.deserializeArgs(buf, sizeInBytes, ctx);
		int num = *(int*)&buf[serializedBytes];

		launcher.launch1D(num);

		free(buf);
		//this clFinish is for testing on errors
	}
}

TEST_F(ExecuteBullet3NarrowphaseKernels, mprKernelsCL)
{
	cl_int errNum = 0;
	const char* srcConcave = satConcaveKernelsCL;
	char flags[1024] = {0};
	cl_program mprProg = b3OpenCLUtils::compileCLProgramFromString(m_clContext, m_clDevice, mprKernelsCL, &errNum, flags, 0, true);
	ASSERT_EQ(CL_SUCCESS, errNum);

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, mprKernelsCL, "mprPenetrationKernel", &errNum, mprProg);
		ASSERT_EQ(CL_SUCCESS, errNum);

		if (1)
		{
			const char* fileNames[] = {"mprPenetrationKernel60.bin", "mprPenetrationKernel61.bin", "mprPenetrationKernel70.bin", "mprPenetrationKernel128.bin"};
			int results[] = {0, 1, 46, 98};

			int numTests = sizeof(fileNames) / sizeof(const char*);
			for (int i = 0; i < numTests; i++)
			{
				b3LauncherCL launcher(m_clQueue, k, fileNames[i]);
				testLauncher(fileNames[i], launcher, m_clContext);
				clFinish(m_clQueue);
				ASSERT_EQ(launcher.getNumArguments(), 11);

				b3KernelArgData data = launcher.getArgument(8);
				ASSERT_TRUE(data.m_isBuffer);
				b3OpenCLArray<int> totalContactsOut(this->m_clContext, this->m_clQueue);
				totalContactsOut.setFromOpenCLBuffer(data.m_clBuffer, 1);
				int numContacts = totalContactsOut.at(0);
				ASSERT_EQ(results[i], numContacts);
			}
			//printf("numContacts = %d\n",numContacts);

			//nContacts = m_totalContactsOut.at(0);
		}

		clReleaseKernel(k);
	}

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, mprKernelsCL, "findSeparatingAxisUnitSphereKernel", &errNum, mprProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(k);
	}

	clReleaseProgram(mprProg);
}

};  // namespace
