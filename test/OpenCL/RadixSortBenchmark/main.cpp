/******************************************************************************
 * Copyright 2010 Duane Merrill
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may ob3ain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License. 
 * 
 * 
 * 
 * 
 * AUTHORS' REQUEST: 
 * 
 * 		If you use|reference|benchmark this code, please cite our Technical 
 * 		Report (http://www.cs.virginia.edu/~dgm4d/papers/RadixSortTR.pdf):
 * 
 *		@TechReport{ Merrill:Sorting:2010,
 *        	author = "Duane Merrill and Andrew Grimshaw",
 *        	title = "Revisiting Sorting for GPGPU Stream Architectures",
 *        	year = "2010",
 *        	institution = "University of Virginia, Department of Computer Science",
 *        	address = "Charlottesville, VA, USA",
 *        	number = "CS2010-03"
 *		}
 * 
 * For more information, see our Google Code project site: 
 * http://code.google.com/p/back40computing/
 * 
 * Thanks!
 ******************************************************************************/

/******************************************************************************
 * Simple test driver program for *large-problem* radix sorting.
 *
 * Useful for demonstrating how to integrate radix sorting into 
 * your application 
 ******************************************************************************/

/******************************************************************************
 * Converted from CUDA to OpenCL/DirectCompute by Erwin Coumans
 ******************************************************************************/
#ifdef _WIN32
#pragma warning(disable : 4996)
#endif
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <algorithm>
#include <string>

//#include <iostream>
#include <sstream>
/**********************
*
*/

#include "Bullet3OpenCL/ParallelPrimitives/b3RadixSort32CL.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "../btgui/Bullet3AppSupport/b3Clock.h"

cl_context g_cxMainContext;
cl_device_id g_device;
cl_command_queue g_cqCommandQueue;

/***********************
*
*/

bool g_verbose;
///Preferred OpenCL device/platform. When < 0 then no preference is used.
///Note that b3OpenCLUtils might still use the preference of using a platform vendor that matches the SDK vendor used to build the application.
///Preferred device/platform take priority over this platform-vendor match
int gPreferredDeviceId = -1;
int gPreferredPlatformId = -1;

/******************************************************************************
 * Routines
 ******************************************************************************/

/**
 * Keys-only sorting.  Uses the GPU to sort the specified vector of elements for the given 
 * number of iterations, displaying runtime information.
 *
 * @param[in] 		num_elements 
 * 		Size in elements of the vector to sort
 * @param[in] 		h_keys 
 * 		Vector of keys to sort 
 * @param[in] 		iterations  
 * 		Number of times to invoke the GPU sorting primitive
  * @param[in] 		cfg 
 * 		Config
 */
template <typename K>
void TimedSort(
	unsigned int num_elements,
	K *h_keys,
	unsigned int iterations)
{
	printf("Keys only, %d iterations, %d elements\n", iterations, num_elements);

	int max_elements = num_elements;
	b3AlignedObjectArray<unsigned int> hostData;
	hostData.resize(num_elements);
	for (int i = 0; i < num_elements; i++)
	{
		hostData[i] = h_keys[i];
	}

	b3RadixSort32CL sorter(g_cxMainContext, g_device, g_cqCommandQueue);

	b3OpenCLArray<unsigned int> gpuData(g_cxMainContext, g_cqCommandQueue);
	gpuData.copyFromHost(hostData);
	//sorter.executeHost(gpuData);
	sorter.execute(gpuData);

	b3AlignedObjectArray<unsigned int> hostDataSorted;
	gpuData.copyToHost(hostDataSorted);

	clFinish(g_cqCommandQueue);

	{
		//printf("Key-values, %d iterations, %d elements", iterations, num_elements);

		// Create sorting enactor

		// Perform the timed number of sorting iterations
		double elapsed = 0;
		float duration = 0;
		b3Clock watch;

		//warm-start
		gpuData.copyFromHost(hostData);
		clFinish(g_cqCommandQueue);
		sorter.execute(gpuData);

		watch.reset();

		for (int i = 0; i < iterations; i++)
		{
			// Move a fresh copy of the problem into device storage
			gpuData.copyFromHost(hostData);
			clFinish(g_cqCommandQueue);

			// Start GPU timing record
			double startMs = watch.getTimeMicroseconds() / 1e3;

			// Call the sorting API routine
			sorter.execute(gpuData);

			clFinish(g_cqCommandQueue);

			double stopMs = watch.getTimeMicroseconds() / 1e3;

			duration = stopMs - startMs;

			// End GPU timing record
			elapsed += (double)duration;
			printf("duration = %f\n", duration);
		}

		// Display timing information
		double avg_runtime = elapsed / iterations;
		//	double throughput = ((double) num_elements) / avg_runtime / 1000.0 / 1000.0;
		//   printf(", %f GPU ms, %f x10^9 elts/sec\n", 	avg_runtime,	throughput);
		double throughput = ((double)num_elements) / avg_runtime / 1000.0;
		printf(", %f GPU ms, %f x10^6 elts/sec\n", avg_runtime, throughput);

		gpuData.copyToHost(hostData);
		for (int i = 0; i < num_elements; i++)
		{
			h_keys[i] = hostData[i];
		}
	}
}

/**
 * Key-value sorting.  Uses the GPU to sort the specified vector of elements for the given 
 * number of iterations, displaying runtime information.
 *
 * @param[in] 		num_elements 
 * 		Size in elements of the vector to sort
 * @param[in] 		h_keys 
 * 		Vector of keys to sort 
 * @param[in,out] 	h_values  
 * 		Vector of values to sort 
 * @param[in] 		iterations  
 * 		Number of times to invoke the GPU sorting primitive
  * @param[in] 		cfg 
 * 		Config
 */
template <typename K, typename V>
void TimedSort(
	unsigned int num_elements,
	K *h_keys,
	V *h_values,
	unsigned int iterations)
{
	printf("Key-values, %d iterations, %d elements\n", iterations, num_elements);

	int max_elements = num_elements;
	b3AlignedObjectArray<b3SortData> hostData;
	hostData.resize(num_elements);
	for (int i = 0; i < num_elements; i++)
	{
		hostData[i].m_key = h_keys[i];
		hostData[i].m_value = h_values[i];
	}

	b3RadixSort32CL sorter(g_cxMainContext, g_device, g_cqCommandQueue);

	b3OpenCLArray<b3SortData> gpuData(g_cxMainContext, g_cqCommandQueue);
	gpuData.copyFromHost(hostData);
	//sorter.executeHost(gpuData);
	sorter.execute(gpuData);

	b3AlignedObjectArray<b3SortData> hostDataSorted;
	gpuData.copyToHost(hostDataSorted);
#if 0
    for (int i=0;i<num_elements;i++)
	{
		printf("hostData[%d].m_key = %d\n",i, hostDataSorted[i].m_key);
        printf("hostData[%d].m_value = %d\n",i,hostDataSorted[i].m_value);
	}
#endif

	clFinish(g_cqCommandQueue);

	{
		//printf("Key-values, %d iterations, %d elements", iterations, num_elements);

		// Create sorting enactor

		// Perform the timed number of sorting iterations
		double elapsed = 0;
		float duration = 0;
		b3Clock watch;

		//warm-start
		gpuData.copyFromHost(hostData);
		sorter.execute(gpuData);
		clFinish(g_cqCommandQueue);

		watch.reset();

		for (int i = 0; i < iterations; i++)
		{
			// Move a fresh copy of the problem into device storage
			gpuData.copyFromHost(hostData);
			clFinish(g_cqCommandQueue);

			// Start GPU timing record
			double startMs = watch.getTimeMicroseconds() / 1e3;

			// Call the sorting API routine
			sorter.execute(gpuData);
			clFinish(g_cqCommandQueue);

			double stopMs = watch.getTimeMicroseconds() / 1e3;

			duration = stopMs - startMs;

			// End GPU timing record
			elapsed += (double)duration;
			printf("duration = %f\n", duration);
		}

		// Display timing information
		double avg_runtime = elapsed / iterations;
		//	double throughput = ((double) num_elements) / avg_runtime / 1000.0 / 1000.0;
		//   printf(", %f GPU ms, %f x10^9 elts/sec\n", 	avg_runtime,	throughput);
		double throughput = ((double)num_elements) / avg_runtime / 1000.0;
		printf(", %f GPU ms, %f x10^6 elts/sec\n", avg_runtime, throughput);

		gpuData.copyToHost(hostData);
		for (int i = 0; i < num_elements; i++)
		{
			h_keys[i] = hostData[i].m_key;
			h_values[i] = hostData[i].m_value;
		}
	}
}

/**
 * Generates random 32-bit keys.
 * 
 * We always take the second-order byte from rand() because the higher-order 
 * bits returned by rand() are commonly considered more uniformly distributed
 * than the lower-order bits.
 * 
 * We can decrease the entropy level of keys by adopting the technique 
 * of Thearling and Smith in which keys are computed from the bitwise AND of 
 * multiple random samples: 
 * 
 * entropy_reduction	| Effectively-unique bits per key
 * -----------------------------------------------------
 * -1					| 0
 * 0					| 32
 * 1					| 25.95
 * 2					| 17.41
 * 3					| 10.78
 * 4					| 6.42
 * ...					| ...
 * 
 */
template <typename K>
void RandomBits(K &key, int entropy_reduction = 0, int lower_key_bits = sizeof(K) * 8)
{
	const unsigned int NUM_UCHARS = (sizeof(K) + sizeof(unsigned char) - 1) / sizeof(unsigned char);
	unsigned char key_bits[NUM_UCHARS];

	do
	{
		for (int j = 0; j < NUM_UCHARS; j++)
		{
			unsigned char quarterword = 0xff;
			for (int i = 0; i <= entropy_reduction; i++)
			{
				quarterword &= (rand() >> 7);
			}
			key_bits[j] = quarterword;
		}

		if (lower_key_bits < sizeof(K) * 8)
		{
			unsigned long long base = 0;
			memcpy(&base, key_bits, sizeof(K));
			base &= (1 << lower_key_bits) - 1;
			memcpy(key_bits, &base, sizeof(K));
		}

		memcpy(&key, key_bits, sizeof(K));

	} while (key != key);  // avoids NaNs when generating random floating point numbers
}

/******************************************************************************
 * Templated routines for printing keys/values to the console 
 ******************************************************************************/

template <typename T>
void PrintValue(T val)
{
	printf("%d", val);
}

template <>
void PrintValue<float>(float val)
{
	printf("%f", val);
}

template <>
void PrintValue<double>(double val)
{
	printf("%f", val);
}

template <>
void PrintValue<unsigned char>(unsigned char val)
{
	printf("%u", val);
}

template <>
void PrintValue<unsigned short>(unsigned short val)
{
	printf("%u", val);
}

template <>
void PrintValue<unsigned int>(unsigned int val)
{
	printf("%u", val);
}

template <>
void PrintValue<long>(long val)
{
	printf("%ld", val);
}

template <>
void PrintValue<unsigned long>(unsigned long val)
{
	printf("%lu", val);
}

template <>
void PrintValue<long long>(long long val)
{
	printf("%lld", val);
}

template <>
void PrintValue<unsigned long long>(unsigned long long val)
{
	printf("%llu", val);
}

/**
 * Compares the equivalence of two arrays
 */
template <typename T, typename SizeT>
int CompareResults(T *computed, T *reference, SizeT len, bool verbose = true)
{
	printf("\n");
	for (SizeT i = 0; i < len; i++)
	{
		if (computed[i] != reference[i])
		{
			printf("INCORRECT: [%lu]: ", (unsigned long)i);
			PrintValue<T>(computed[i]);
			printf(" != ");
			PrintValue<T>(reference[i]);

			if (verbose)
			{
				printf("\nresult[...");
				for (size_t j = (i >= 5) ? i - 5 : 0; (j < i + 5) && (j < len); j++)
				{
					PrintValue<T>(computed[j]);
					printf(", ");
				}
				printf("...]");
				printf("\nreference[...");
				for (size_t j = (i >= 5) ? i - 5 : 0; (j < i + 5) && (j < len); j++)
				{
					PrintValue<T>(reference[j]);
					printf(", ");
				}
				printf("...]");
			}

			return 1;
		}
	}

	printf("CORRECT\n");
	return 0;
}

/**
 * Creates an example sorting problem whose keys is a vector of the specified 
 * number of K elements, values of V elements, and then dispatches the problem 
 * to the GPU for the given number of iterations, displaying runtime information.
 *
 * @param[in] 		iterations  
 * 		Number of times to invoke the GPU sorting primitive
 * @param[in] 		num_elements 
 * 		Size in elements of the vector to sort
 * @param[in] 		cfg 
 * 		Config
 */
template <typename K, typename V>
void TestSort(
	unsigned int iterations,
	int num_elements,
	bool keys_only)
{
	// Allocate the sorting problem on the host and fill the keys with random bytes

	K *h_keys = NULL;
	K *h_reference_keys = NULL;
	V *h_values = NULL;
	h_keys = (K *)malloc(num_elements * sizeof(K));
	h_reference_keys = (K *)malloc(num_elements * sizeof(K));
	if (!keys_only) h_values = (V *)malloc(num_elements * sizeof(V));

	// Use random bits
	for (unsigned int i = 0; i < num_elements; ++i)
	{
		RandomBits<K>(h_keys[i], 0);
		//h_keys[i] = num_elements-i;
		//h_keys[i] = 0xffffffffu-i;
		if (!keys_only)
			h_values[i] = h_keys[i];  //0xffffffffu-i;

		h_reference_keys[i] = h_keys[i];
	}

	// Run the timing test
	if (keys_only)
	{
		TimedSort<K>(num_elements, h_keys, iterations);
	}
	else
	{
		TimedSort<K, V>(num_elements, h_keys, h_values, iterations);
	}

	//	cudaThreadSynchronize();

	// Display sorted key data
	if (g_verbose)
	{
		printf("\n\nKeys:\n");
		for (int i = 0; i < num_elements; i++)
		{
			PrintValue<K>(h_keys[i]);
			printf(", ");
		}
		printf("\n\n");
	}

	// Verify solution
	std::sort(h_reference_keys, h_reference_keys + num_elements);
	CompareResults<K>(h_keys, h_reference_keys, num_elements, true);
	printf("\n");
	fflush(stdout);

	// Free our allocated host memory
	if (h_keys != NULL) free(h_keys);
	if (h_values != NULL) free(h_values);
}

/**
 * Displays the commandline usage for this tool
 */
void Usage()
{
	printf("\ntest_large_problem_sorting [--device=<device index>] [--v] [--i=<num-iterations>] [--n=<num-elements>] [--key-values] [--deviceId=<int>] [--platformId=<int>]\n");
	printf("\n");
	printf("\t--v\tDisplays sorted results to the console.\n");
	printf("\n");
	printf("\t--i\tPerforms the sorting operation <num-iterations> times\n");
	printf("\t\t\ton the device. Re-copies original input each time. Default = 1\n");
	printf("\n");
	printf("\t--n\tThe number of elements to comprise the sample problem\n");
	printf("\t\t\tDefault = 512\n");
	printf("\n");
	printf("\t--key-values\tSpecifies that keys are accommodated by value pairings\n");
	printf("\n");
}

/******************************************************************************
 * Command-line parsing
 ******************************************************************************/
#include <map>
#include <algorithm>
#include <string>

class b3CommandLineArgs
{
protected:
	std::map<std::string, std::string> pairs;

public:
	// Constructor
	b3CommandLineArgs(int argc, char **argv)
	{
		using namespace std;

		for (int i = 1; i < argc; i++)
		{
			string arg = argv[i];

			if ((arg[0] != '-') || (arg[1] != '-'))
			{
				continue;
			}

			string::size_type pos;
			string key, val;
			if ((pos = arg.find('=')) == string::npos)
			{
				key = string(arg, 2, arg.length() - 2);
				val = "";
			}
			else
			{
				key = string(arg, 2, pos - 2);
				val = string(arg, pos + 1, arg.length() - 1);
			}
			pairs[key] = val;
		}
	}

	bool CheckCmdLineFlag(const char *arg_name)
	{
		using namespace std;
		map<string, string>::iterator itr;
		if ((itr = pairs.find(arg_name)) != pairs.end())
		{
			return true;
		}
		return false;
	}

	template <typename T>
	void GetCmdLineArgument(const char *arg_name, T &val);

	int ParsedArgc()
	{
		return pairs.size();
	}
};

template <typename T>
void b3CommandLineArgs::GetCmdLineArgument(const char *arg_name, T &val)
{
	using namespace std;
	map<string, string>::iterator itr;
	if ((itr = pairs.find(arg_name)) != pairs.end())
	{
		istringstream strstream(itr->second);
		strstream >> val;
	}
}

template <>
void b3CommandLineArgs::GetCmdLineArgument<char *>(const char *arg_name, char *&val)
{
	using namespace std;
	map<string, string>::iterator itr;
	if ((itr = pairs.find(arg_name)) != pairs.end())
	{
		string s = itr->second;
		val = (char *)malloc(sizeof(char) * (s.length() + 1));
		strcpy(val, s.c_str());
	}
	else
	{
		val = NULL;
	}
}

/******************************************************************************
 * Main
 ******************************************************************************/

extern bool gDebugSkipLoadingBinary;

void myprintf(const char *msg)
{
	(void *)msg;
}
int main(int argc, char **argv)
{
	//gDebugSkipLoadingBinary = true;

	//	b3SetCustomPrintfFunc(myprintf);

	cl_int ciErrNum;
	b3CommandLineArgs args(argc, argv);

	args.GetCmdLineArgument("deviceId", gPreferredDeviceId);
	args.GetCmdLineArgument("platformId", gPreferredPlatformId);

	b3Printf("Initialize OpenCL using b3OpenCLUtils_createContextFromType\n");
	cl_platform_id platformId;
	//	g_cxMainContext = b3OpenCLUtils_createContextFromType(CL_DEVICE_TYPE_ALL, &ciErrNum, 0, 0,gPreferredDeviceId,gPreferredPlatformId,&platformId);
	g_cxMainContext = b3OpenCLUtils_createContextFromType(CL_DEVICE_TYPE_GPU, &ciErrNum, 0, 0, gPreferredDeviceId, gPreferredPlatformId, &platformId);
	//g_cxMainContext = b3OpenCLUtils_createContextFromType(CL_DEVICE_TYPE_CPU, &ciErrNum, 0, 0,gPreferredDeviceId,gPreferredPlatformId,&platformId);

	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	int numDev = b3OpenCLUtils_getNumDevices(g_cxMainContext);

	if (!numDev)
	{
		b3Error("error: no OpenCL devices\n");
		exit(0);
	}
	int devId = 0;
	g_device = b3OpenCLUtils_getDevice(g_cxMainContext, devId);
	b3OpenCLUtils_printDeviceInfo(g_device);
	// create a command-queue
	g_cqCommandQueue = clCreateCommandQueue(g_cxMainContext, g_device, 0, &ciErrNum);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	//srand(time(NULL));
	srand(0);  // presently deterministic

	unsigned int num_elements = 8 * 1024 * 1024;  //4*1024*1024;//4*1024*1024;//257;//8*524288;//2048;//512;//524288;
	unsigned int iterations = 10;
	bool keys_only = true;

	//
	// Check command line arguments
	//

	if (args.CheckCmdLineFlag("help"))
	{
		Usage();
		return 0;
	}

	args.GetCmdLineArgument("i", iterations);
	args.GetCmdLineArgument("n", num_elements);

	keys_only = !args.CheckCmdLineFlag("key-values");
	g_verbose = args.CheckCmdLineFlag("v");

	TestSort<unsigned int, unsigned int>(
		iterations,
		num_elements,
		keys_only);
}
