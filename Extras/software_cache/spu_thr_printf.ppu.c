/*
 *   SCE CONFIDENTIAL                                      
 *   PLAYSTATION(R)3 Programmer Tool Runtime Library 085.007
 *   Copyright (C) 2005 Sony Computer Entertainment Inc.   
 *   All Rights Reserved.                                  
 *
 * File: spu_thr_printf.c
 * Description:
 *  This sample shows how to output strings by SPU programs.  spu_printf() 
 *  called by 
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/spu_initialize.h>
#include <sys/spu_image.h>
#include <sys/spu_thread.h>
#include <sys/spu_thread_group.h>
#include <sys/spu_utility.h>
#include <sys/paths.h>
#include "spu_printf_server.h" /* SPU printf server */

#define MAX_PHYSICAL_SPU      4 
#define MAX_RAW_SPU           0
#define NUM_SPU_THREADS       4 /* The number of SPU threads in the group */ 
#define PRIORITY            100
#ifdef SN_TARGET_PS3
#define SPU_PROG     (SYS_APP_HOME "/SPU_printf.spu.self")
#else
#define SPU_PROG     (SYS_APP_HOME "/hello.spu.self")
#endif

#define IN_BUF_SIZE 256
#define OUT_BUF_SIZE 256

volatile uint8_t in_buf[IN_BUF_SIZE];
volatile uint8_t out_buf[OUT_BUF_SIZE];
uint32_t in_size = IN_BUF_SIZE;
uint32_t out_size = OUT_BUF_SIZE;

int main(void)
{



	sprintf(in_buf,"hello world");

	sys_spu_thread_group_t group; /* SPU thread group ID */
	const char *group_name = "Group";
	sys_spu_thread_group_attribute_t group_attr;/* SPU thread group attribute*/
	sys_spu_thread_t threads[NUM_SPU_THREADS];  /* SPU thread IDs */
	sys_spu_thread_attribute_t thread_attr;     /* SPU thread attribute */
	const char *thread_names[NUM_SPU_THREADS] = 
		{"SPU Thread 0",
		 "SPU Thread 1",
		 "SPU Thread 2",
		 "SPU Thread 3"}; /* The names of SPU threads */
    sys_spu_image_t spu_img; 
	int ret;
	
	/*
	 * Initialize SPUs
	 */
	printf("Initializing SPUs\n");
	ret = sys_spu_initialize(MAX_PHYSICAL_SPU, MAX_RAW_SPU);
	if (ret != CELL_OK) {
		fprintf(stderr, "sys_spu_initialize failed: %#.8x\n", ret);
		exit(ret);
	}

	/*
	 * Create an SPU thread group
	 */
	printf("Creating an SPU thread group.\n");
	group_attr.name = group_name;
	group_attr.nsize = strlen(group_attr.name) + 1; /* Add 1 for '\0' */
	group_attr.type = SYS_SPU_THREAD_GROUP_TYPE_NORMAL;
	ret = sys_spu_thread_group_create(&group, 
									  NUM_SPU_THREADS,
									  PRIORITY, 
									  &group_attr);
	if (ret != CELL_OK) {
		fprintf(stderr, "sys_spu_thread_group_create failed: %#.8x\n", ret);
		exit(ret);
	}
	
	ret = sys_spu_image_open(&spu_img, SPU_PROG);
	if (ret != CELL_OK) {
		fprintf(stderr, "sys_spu_image_open failed: %#.8x\n", ret);
		exit(ret);
	}

	/*
	 * Initialize the SPU printf server
	 *
	 * What spu_printf_server_initialize() actually does is to create an 
	 * PPU thread and an event queue which handle the events sent by 
	 * spu_printf().  
	 */
	ret = spu_printf_server_initialize();
	if (ret != SUCCEEDED) {
			fprintf(stderr, "spu_printf_server_initialize failed: %#.8x\n", ret);
			exit(ret);
	}
	/*
	 * In this loop, all SPU threads in the SPU thread group are initialized 
	 * with the loaded SPU ELF image.
	 */
	for (int i = 0; i < NUM_SPU_THREADS; i++) {
		sys_spu_thread_argument_t thread_args;
		int spu_num = i;

		printf("Initializing SPU thread %d\n", i);
		
		/*
		 * nsegs, segs and entry_point have already been initialized by 
		 * sys_spu_thread_elf_loader().
		 */
		thread_attr.name = thread_names[i];
		thread_attr.nsize = strlen(thread_names[i]) + 1;
		thread_attr.option = SYS_SPU_THREAD_OPTION_NONE;
		
		/*
		 * Pass the SPU number to the SPU thread as the first parameter.
		 */
		thread_args.arg1 = SYS_SPU_THREAD_ARGUMENT_LET_32(spu_num);
		thread_args.arg2 = SYS_SPU_THREAD_ARGUMENT_LET_64((uint64_t)in_buf);

		

		/*
		 * The third argument specifies the SPU number.  
		 * The SPU number of each SPU thread must be unique within the SPU
		 * thread group.
		 */
		ret = sys_spu_thread_initialize(&threads[i],
										group,
										spu_num,
										&spu_img,
										&thread_attr,
										&thread_args);
		if (ret != CELL_OK) {
			fprintf(stderr, "sys_spu_thread_initialize failed: %#.8x\n", ret);
			exit(ret);
		}
		
		/*
		 * Register the SPU thread to the SPU printf server.
		 *
		 * spu_printf_server_register() establishes the connection between 
		 * the SPU thread and the SPU printf server's event queue.  
		 */
		ret = spu_printf_server_register(threads[i]);
		if (ret != CELL_OK) {
			fprintf(stderr, "spu_printf_server_register failed: %#.8x\n", ret);
			exit(ret);
		}
	}

	printf("All SPU threads have been successfully initialized.\n");

	/*
	 * Start the SPU thread group
	 *
	 * The SPU thread group will be in the READY state, and will become in 
	 * the RUNNING state when the kernel assigns and executes it onto SPUs.
	 */
	printf("Starting the SPU thread group.\n");
	ret = sys_spu_thread_group_start(group);
	if (ret != CELL_OK) {
		fprintf(stderr, "sys_spu_thread_group_start failed: %#.8x\n", ret);
		exit(ret);
	}

	/*
	 * Wait for the termination of the SPU thread group.
	 */
	printf("Waiting for the SPU thread group to be terminated.\n");
	int cause, status;
	ret = sys_spu_thread_group_join(group, &cause, &status);
	if (ret != CELL_OK) {
		fprintf(stderr, "sys_spu_thread_group_join failed: %#.8x\n", ret);
		exit(ret);
	}

	/*
	 * Show the exit cause and status.
	 */
	switch(cause) {
	case SYS_SPU_THREAD_GROUP_JOIN_GROUP_EXIT:
		printf("The SPU thread group exited by sys_spu_thread_group_exit().\n");
		printf("The group's exit status = %d\n", status);
		break;
	case SYS_SPU_THREAD_GROUP_JOIN_ALL_THREADS_EXIT:
		printf("All SPU thread exited by sys_spu_thread_exit().\n");
		for (int i = 0; i < NUM_SPU_THREADS; i++) {
			int thr_exit_status;
			ret = sys_spu_thread_get_exit_status(threads[i], &thr_exit_status);
			if (ret != CELL_OK) {
				fprintf(stderr, "sys_spu_thread_get_exit_status failed: %#.8x\n", ret);
			}
			printf("SPU thread %d's exit status = %d\n", i, thr_exit_status);
		}
		break;
	case SYS_SPU_THREAD_GROUP_JOIN_TERMINATED:
		printf("The SPU thread group is terminated by sys_spu_thread_terminate().\n");
		printf("The group's exit status = %d\n", status);
		break;
	default:
		fprintf(stderr, "Unknown exit cause: %d\n", cause);
		break;
	}

	/*
	 * Destroy the SPU thread group and clean up resources.
	 */
	ret = sys_spu_thread_group_destroy(group);
	if (ret != CELL_OK) {
		fprintf(stderr, "sys_spu_thread_group_destroy failed: %#.8x\n", ret);
	}

	ret = sys_spu_image_close(&spu_img);
	if (ret != CELL_OK) {
		fprintf(stderr, "sys_spu_image_close failed: %.8x\n", ret);
	}
	
	/*
	 * Finalize the SPU printf server.
	 *
	 * This function let the PPU thread exit.  
	 * The event queue will be destroyed.
	 */
	ret = spu_printf_server_finalize();
	if (ret != CELL_OK) {
		fprintf(stderr, "spu_printf_server_finalize failed: %#.8x\n", ret);
	}

	printf("Exiting.\n");
	return 0;
}	


