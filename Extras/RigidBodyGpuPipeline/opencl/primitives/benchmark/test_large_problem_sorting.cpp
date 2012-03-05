/******************************************************************************
 * Copyright 2010 Duane Merrill
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
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

#include <stdlib.h> 
#include <stdio.h> 
#include <string.h> 
#include <math.h> 
#include <float.h>
#include <algorithm>
#include <string>

#define BUFFERSIZE_WORKAROUND

//#include <iostream>
#include <sstream>
/**********************
*
*/

#include "Adl/Adl.h"
#include "AdlPrimitives/Sort/RadixSort32.h"
#include "AdlPrimitives/Sort/SortData.h"

using namespace adl;


/***********************
*
*/

bool g_verbose;


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
template <typename K, DeviceType type>
void TimedSort(
	unsigned int num_elements, 
	K *h_keys,
	unsigned int iterations, const DeviceUtils::Config& cfg)
{
	std::string sType = "No type selected";
	
	if (type == TYPE_CL)
		sType = "OpenCL";
	else if (type == TYPE_DX11)
		sType = "DX11";

	printf("Keys-only, %s, %d iterations, %d elements\n", sType.c_str(), iterations, num_elements);

	int max_elements = num_elements;

#ifdef BUFFERSIZE_WORKAROUND
	if (max_elements < 1024*256)
		max_elements = 1024*256;
#endif
	
	// Allocate device storage
	Device* deviceData = NULL;

	if ( type == TYPE_CL )
		deviceData = new DeviceCL();
#ifdef ADL_ENABLE_DX11
	else if ( type == TYPE_DX11 )
		deviceData = new DeviceDX11();
#endif //ADL_ENABLE_DX11

	deviceData->initialize(cfg);

	RadixSort32<type>::Data* planData = RadixSort32<type>::allocate( deviceData, max_elements);

	{
		Buffer<unsigned int>	keysInOut(deviceData,max_elements);
		
		// Create sorting enactor
		keysInOut.write(h_keys,num_elements);
		DeviceUtils::waitForCompletion( deviceData);
		
		RadixSort32<type>::execute( planData,keysInOut,num_elements,  32);
		DeviceUtils::waitForCompletion( deviceData);

		// Perform the timed number of sorting iterations
		double elapsed = 0;
		float duration = 0;
		StopwatchHost watch;
		watch.init(deviceData);

		watch.start();
			
		for (int i = 0; i < iterations; i++) 
		{

			// Move a fresh copy of the problem into device storage
			keysInOut.write(h_keys,num_elements);
			DeviceUtils::waitForCompletion( deviceData);

			// Start GPU timing record
			watch.start();
			
			// Call the sorting API routine
			RadixSort32<type>::execute( planData,keysInOut,num_elements,  32);
			DeviceUtils::waitForCompletion( deviceData);
		
			watch.stop();
			duration = watch.getMs();

			// End GPU timing record
			elapsed += (double) duration;
		}

		// Display timing information
		double avg_runtime = elapsed / iterations;
	//	double throughput = ((double) num_elements) / avg_runtime / 1000.0 / 1000.0; 
	//   printf(", %f GPU ms, %f x10^9 elts/sec\n", 	avg_runtime,	throughput);
		double throughput = ((double) num_elements) / avg_runtime / 1000.0 ; 
		printf(", %f GPU ms, %f x10^6 elts/sec\n", 	avg_runtime,	throughput);

		// Copy out data 
		keysInOut.read(h_keys,num_elements);
		
		DeviceUtils::waitForCompletion( deviceData);

	}
	// Free allocated memory
	RadixSort32<type>::deallocate( planData);
    delete deviceData;
    // Clean up events
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
template <typename K, typename V, DeviceType type>
void TimedSort(
	unsigned int num_elements, 
	K *h_keys,
	V *h_values, 
	unsigned int iterations, const DeviceUtils::Config& cfg) 
{
	std::string sType = "No type selected";
	
	if (type == TYPE_CL)
		sType = "OpenCL";
	else if (type == TYPE_DX11)
		sType = "DX11";

	printf("Key-values, %s, %d iterations, %d elements\n", sType.c_str(), iterations, num_elements);

	int max_elements = num_elements;

#ifdef BUFFERSIZE_WORKAROUND
	if (max_elements < 1024*256)
		max_elements = 1024*256;
#endif
	
	// Allocate device storage
	Device* deviceData = NULL;

	if ( type == TYPE_CL )
		deviceData = new DeviceCL();
#ifdef ADL_ENABLE_DX11
	else if ( type == TYPE_DX11 )
		deviceData = new DeviceDX11();
#endif //ADL_ENABLE_DX11

	deviceData->initialize(cfg);
	RadixSort32<type>::Data* planData = RadixSort32<type>::allocate( deviceData, max_elements);
	{
		Buffer<unsigned int>	keysIn(deviceData,max_elements);
		Buffer<unsigned int>	valuesIn(deviceData,max_elements);

		Buffer<unsigned int>	keysOut(deviceData,max_elements);
		Buffer<unsigned int>	valuesOut(deviceData,max_elements);

		//printf("Key-values, %d iterations, %d elements", iterations, num_elements);

		// Create sorting enactor
		keysIn.write(h_keys,num_elements);
		DeviceUtils::waitForCompletion( deviceData);
		valuesIn.write(h_values,num_elements);
		DeviceUtils::waitForCompletion( deviceData);

		
		// Perform a single sorting iteration to allocate memory, prime code caches, etc.
		//RadixSort<type>::execute( planData, buffer,  num_elements );

		//RadixSort32<type>::execute( planData, keysIn,keysOut, valuesIn,valuesOut, num_elements,  32);
		RadixSort32<type>::execute( planData, keysIn,keysOut, valuesIn,valuesOut, num_elements,  32);
		DeviceUtils::waitForCompletion( deviceData);

		// Perform the timed number of sorting iterations
		double elapsed = 0;
		float duration = 0;
		StopwatchHost watch;
		watch.init(deviceData);

		watch.start();
			
		for (int i = 0; i < iterations; i++) 
		{

			// Move a fresh copy of the problem into device storage
			keysIn.write(h_keys,num_elements);
			valuesIn.write(h_values,num_elements);

			DeviceUtils::waitForCompletion( deviceData);

			// Start GPU timing record
			watch.start();
			
			// Call the sorting API routine
			
			RadixSort32<type>::execute( planData, keysIn,keysOut, valuesIn,valuesOut, num_elements,  32);

			DeviceUtils::waitForCompletion( deviceData);
		
			watch.stop();
			duration = watch.getMs();

			// End GPU timing record
			elapsed += (double) duration;
		}

		// Display timing information
		double avg_runtime = elapsed / iterations;
	//	double throughput = ((double) num_elements) / avg_runtime / 1000.0 / 1000.0; 
	//   printf(", %f GPU ms, %f x10^9 elts/sec\n", 	avg_runtime,	throughput);
		double throughput = ((double) num_elements) / avg_runtime / 1000.0 ; 
		printf(", %f GPU ms, %f x10^6 elts/sec\n", 	avg_runtime,	throughput);

		//memset(h_keys,1,num_elements);
		//memset(h_values,1,num_elements);
		// Copy out data 
		keysOut.read(h_keys,num_elements);
		valuesOut.read(h_values,num_elements);

		DeviceUtils::waitForCompletion( deviceData);
	}
    
	// Free allocated memory
	RadixSort32<type>::deallocate( planData);
    delete deviceData;
    // Clean up events
	
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
	
	do {
	
		for (int j = 0; j < NUM_UCHARS; j++) {
			unsigned char quarterword = 0xff;
			for (int i = 0; i <= entropy_reduction; i++) {
				quarterword &= (rand() >> 7);
			}
			key_bits[j] = quarterword;
		}
		
		if (lower_key_bits < sizeof(K) * 8) {
			unsigned long long base = 0;
			memcpy(&base, key_bits, sizeof(K));
			base &= (1 << lower_key_bits) - 1;
			memcpy(key_bits, &base, sizeof(K));
		}
		
		memcpy(&key, key_bits, sizeof(K));
		
	} while (key != key);		// avoids NaNs when generating random floating point numbers 
}


/******************************************************************************
 * Templated routines for printing keys/values to the console 
 ******************************************************************************/

template<typename T> 
void PrintValue(T val) {
	printf("%d", val);
}

template<>
void PrintValue<float>(float val) {
	printf("%f", val);
}

template<>
void PrintValue<double>(double val) {
	printf("%f", val);
}

template<>
void PrintValue<unsigned char>(unsigned char val) {
	printf("%u", val);
}

template<>
void PrintValue<unsigned short>(unsigned short val) {
	printf("%u", val);
}

template<>
void PrintValue<unsigned int>(unsigned int val) {
	printf("%u", val);
}

template<>
void PrintValue<long>(long val) {
	printf("%ld", val);
}

template<>
void PrintValue<unsigned long>(unsigned long val) {
	printf("%lu", val);
}

template<>
void PrintValue<long long>(long long val) {
	printf("%lld", val);
}

template<>
void PrintValue<unsigned long long>(unsigned long long val) {
	printf("%llu", val);
}



/**
 * Compares the equivalence of two arrays
 */
template <typename T, typename SizeT>
int CompareResults(T* computed, T* reference, SizeT len, bool verbose = true)
{
	printf("\n");
	for (SizeT i = 0; i < len; i++) {

		if (computed[i] != reference[i]) {
			printf("INCORRECT: [%lu]: ", (unsigned long) i);
			PrintValue<T>(computed[i]);
			printf(" != ");
			PrintValue<T>(reference[i]);

			if (verbose) {
				printf("\nresult[...");
				for (size_t j = (i >= 5) ? i - 5 : 0; (j < i + 5) && (j < len); j++) {
					PrintValue<T>(computed[j]);
					printf(", ");
				}
				printf("...]");
				printf("\nreference[...");
				for (size_t j = (i >= 5) ? i - 5 : 0; (j < i + 5) && (j < len); j++) {
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
template<typename K, typename V, DeviceType type>
void TestSort(
	unsigned int iterations,
	int num_elements,
	bool keys_only, const DeviceUtils::Config& cfg)
{
    // Allocate the sorting problem on the host and fill the keys with random bytes

	K *h_keys = NULL;
	K *h_reference_keys = NULL;
	V *h_values = NULL;
	h_keys = (K*) malloc(num_elements * sizeof(K));
	h_reference_keys = (K*) malloc(num_elements * sizeof(K));
	if (!keys_only) h_values = (V*) malloc(num_elements * sizeof(V));
	

	// Use random bits
	for (unsigned int i = 0; i < num_elements; ++i) {
		RandomBits<K>(h_keys[i], 0);
		//h_keys[i] = 0xffffffffu-i;
		if (!keys_only)
			h_values[i] = h_keys[i];//0xffffffffu-i;

		h_reference_keys[i] = h_keys[i];
	}

    // Run the timing test 
	if (keys_only) {
		TimedSort<K, type>(num_elements, h_keys, iterations, cfg);
	} else {
		TimedSort<K, V, type>(num_elements, h_keys, h_values, iterations, cfg);
	}

//	cudaThreadSynchronize();
    
	// Display sorted key data
	if (g_verbose) {
		printf("\n\nKeys:\n");
		for (int i = 0; i < num_elements; i++) {	
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
	printf("\ntest_large_problem_sorting [--device=<device index>] [--v] [--i=<num-iterations>] [--n=<num-elements>] [--keys-only]\n"); 
	printf("\n");
	printf("\t--v\tDisplays sorted results to the console.\n");
	printf("\n");
	printf("\t--i\tPerforms the sorting operation <num-iterations> times\n");
	printf("\t\t\ton the device. Re-copies original input each time. Default = 1\n");
	printf("\n");
	printf("\t--n\tThe number of elements to comprise the sample problem\n");
	printf("\t\t\tDefault = 512\n");
	printf("\n");
	printf("\t--keys-only\tSpecifies that keys are not accommodated by value pairings\n");
	printf("\n");
}


/******************************************************************************
 * Command-line parsing
 ******************************************************************************/
#include <map>
#include <algorithm>
#include <string>

class CommandLineArgs
{
protected:

	std::map<std::string, std::string> pairs;

public:

	// Constructor
	CommandLineArgs(int argc, char **argv)
	{
		using namespace std;

	    for (int i = 1; i < argc; i++)
	    {
	        string arg = argv[i];

	        if ((arg[0] != '-') || (arg[1] != '-')) {
	        	continue;
	        }

        	string::size_type pos;
		    string key, val;
	        if ((pos = arg.find( '=')) == string::npos) {
	        	key = string(arg, 2, arg.length() - 2);
	        	val = "";
	        } else {
	        	key = string(arg, 2, pos - 2);
	        	val = string(arg, pos + 1, arg.length() - 1);
	        }
        	pairs[key] = val;
	    }
	}

	bool CheckCmdLineFlag(const char* arg_name)
	{
		using namespace std;
		map<string, string>::iterator itr;
		if ((itr = pairs.find(arg_name)) != pairs.end()) {
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
void CommandLineArgs::GetCmdLineArgument(const char *arg_name, T &val)
{
	using namespace std;
	map<string, string>::iterator itr;
	if ((itr = pairs.find(arg_name)) != pairs.end()) {
		istringstream strstream(itr->second);
		strstream >> val;
    }
}

template <>
void CommandLineArgs::GetCmdLineArgument<char*>(const char* arg_name, char* &val)
{
	using namespace std;
	map<string, string>::iterator itr;
	if ((itr = pairs.find(arg_name)) != pairs.end()) {

		string s = itr->second;
		val = (char*) malloc(sizeof(char) * (s.length() + 1));
		strcpy(val, s.c_str());

	} else {
    	val = NULL;
	}
}





/******************************************************************************
 * Main
 ******************************************************************************/

int main( int argc, char** argv) 
{

	//srand(time(NULL));	
	srand(0);				// presently deterministic

    unsigned int num_elements 					= 1024*1024*12;//16*1024;//8*524288;//2048;//512;//524288;
    unsigned int iterations  					= 10;
    bool keys_only;

    //
	// Check command line arguments
    //

	CommandLineArgs args(argc,argv);

	if (args.CheckCmdLineFlag("help"))
	{
		Usage();
		return 0;
	}
	
	args.GetCmdLineArgument("i", iterations);
	args.GetCmdLineArgument("n", num_elements);
	keys_only = args.CheckCmdLineFlag("keys-only");
	g_verbose = args.CheckCmdLineFlag("v");

	DeviceUtils::Config cfg;

	// Choose AMD or NVidia
#ifdef CL_PLATFORM_AMD
	cfg.m_vendor = DeviceUtils::Config::VD_AMD;
#endif

#ifdef CL_PLATFORM_NVIDIA
	cfg.m_vendor = DeviceUtils::Config::VD_NV;
#endif

	TestSort<unsigned int, unsigned int, TYPE_CL>(
			iterations,
			num_elements, 
			keys_only, cfg);

#ifdef ADL_ENABLE_DX11
	TestSort<unsigned int, unsigned int, TYPE_DX11>(
			iterations,
			num_elements, 
			keys_only, cfg);
#endif //ADL_ENABLE_DX11
}



