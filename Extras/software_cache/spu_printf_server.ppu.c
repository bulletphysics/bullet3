/* 
 *   SCE CONFIDENTIAL                                       
 *   PLAYSTATION(R)3 Programmer Tool Runtime Library 085.007 
 *   Copyright (C) 2005 Sony Computer Entertainment Inc.    
 *   All Rights Reserved.                                   
 *
 * The SPU printf server is a PPU thread which collaborates with an SPU to 
 * output strings.
 *
 * On SPU-side, spu_printf() places the output string and arguments on a stack 
 * in the local storage, and passes its local-storage address with an SPU 
 * thread user event from SPU port 1 to PPU.  On PPU-side, 
 * spu_thread_sprintf fetches the stack in the local stroage by DMA, and parse
 * it to string-format.
 *
 * The SPU printf server takes charge of the tasks on PPU-side.  The sequence
 * of its task is as follows.
 *  1. Receive events by sys_event_queue_receive()
 *  2. Parse the received spu_printf stack address by spu_thread_snprintf.
 *  3. Output the parsed string by printf(), and go back to step 1. 
 *
 * Initialization of the SPU printf server and registeration of SPU threads to 
 * the SPU printf server is required.  These can be done by 
 * spu_printf_server_initilize() and spu_printf_server_register().  
 * What they actually do is to create a PPU thread and event queue, and 
 * connect the SPU thread to the event queue.
 */

#include <stdio.h>
#include <sys/ppu_thread.h>
#include <sys/spu_thread.h>
#include <sys/event.h>
#include <spu_printf.h>
#include "spu_printf_server.h"

#define STACK_SIZE      4096
#define PRIO            200

static sys_ppu_thread_t thread;
static sys_event_queue_t equeue;
static sys_event_queue_attribute_t eattr;
static sys_event_port_t terminating_port;

#define TERMINATING_PORT_NAME    0xFEE1DEAD
#define SPU_PORT_PRINTF         0x1

int spu_printf_server_initialize()
{
	int ret;
	
	/* Create event */
	sys_event_queue_attribute_initialize(eattr);
	ret = sys_event_queue_create(&equeue, &eattr, SYS_EVENT_PORT_LOCAL, 127);
	if (ret) {
		printf("sys_event_queue_create faild %d\n", ret);
		return -1;
	}
	
	/* Create PPU thread */
	ret = sys_ppu_thread_create(&thread, spu_printf_server_entry, 0UL, PRIO,
								STACK_SIZE,
								SYS_PPU_THREAD_CREATE_JOINABLE, 
								(char*)"spu_printf_server");
	if (ret) {
		printf ("spu_printf_server_initialize: sys_ppu_thread_create failed %d\n", ret);
		return -1;
	}

	/*
	 * Create the terminating port. This port is used only in 
	 * sys_printf_server_finalize().
	 */
	ret = sys_event_port_create(&terminating_port, 
								SYS_EVENT_PORT_LOCAL,
								TERMINATING_PORT_NAME);
	if (ret) {
		printf ("spu_printf_server_initialize: sys_event_port_create failed %d\n", ret);
		return -1;
	}

	ret = sys_event_port_connect_local(terminating_port, equeue);
	if (ret) {
		printf ("spu_printf_server_initialize: sys_event_port_connect_local failed %d\n", ret);
		return -1;
	}

	return 0;
}


/*
 * Before call this, SPU threads which are registered finishes to send
 * printf event.
 */
int spu_printf_server_finalize()
{
	int ret;
		
	/* 
	 * Send an event from the terminating port to notify the termination to
	 * the SPU printf server
	 */
	ret = sys_event_port_send(terminating_port, 0, 0, 0);
	if (ret) {
		printf("sys_event_queue_cancel failed %d\n", ret);
		return -1;
	}

	/* Wait for the termination of the SPU printf server */
	uint64_t exit_status;
	ret = sys_ppu_thread_join(thread, &exit_status);
	if (ret) {
		printf("sys_ppu_thread_join failed %d\n", ret);
		return -1;
	}

	/* Disconnect and destroy the terminating port */
	ret = sys_event_port_disconnect(terminating_port);
	if (ret) {
		printf("sys_event_disconnect failed %d\n", ret);
	}
	ret = sys_event_port_destroy(terminating_port);
	if (ret) {
		printf("sys_event_port_destroy failed %d\n", ret);
	}	

	/* Destroy the event queue */
	ret = sys_event_queue_destroy(equeue, 0);
	if (ret) {
		printf("sys_event_queue_destroy failed %d\n", ret);
		return -1;
	}

	return 0;
}


int spu_printf_server_register(sys_spu_thread_t spu)
{
	int ret;
	
	ret = sys_spu_thread_connect_event(spu, equeue,
									   SYS_SPU_THREAD_EVENT_USER, SPU_PORT_PRINTF);
	if (ret) {
		printf("sys_spu_thread_connect_event faild %d\n", ret);
		return -1;
	}

	return 0;
}


int spu_printf_server_unregister(sys_spu_thread_t spu)
{
	int ret;
	
	ret = sys_spu_thread_disconnect_event(spu, 
										  SYS_SPU_THREAD_EVENT_USER, SPU_PORT_PRINTF);
	if (ret) {
		printf("sys_spu_thread_disconnect_event faild %d\n", ret);
		return -1;
	}

	return 0;
}


void spu_printf_server_entry(uint64_t arg)
{
	(void)arg; /* This thread does not use the argument */

	int ret;
	sys_event_t event;
	sys_spu_thread_t spu;
	
	for (;;) {
		ret = sys_event_queue_receive(equeue, &event, SYS_NO_TIMEOUT);
		if (ret) {
			printf("sys_event_queue_receive failed %d\n", ret);
			break;
		}
		
		/*
		 * If an event is sent from the terminating port, the SPU printf 
		 * server exits.
		 */
		if (event.source == TERMINATING_PORT_NAME) {
			printf("Finalize the SPU printf server.\n");
			break;
		}

		spu = event.data1;
		
		int sret = spu_thread_printf(spu, event.data3);
		ret = sys_spu_thread_write_spu_mb(spu, sret);
		if (ret) {
			printf("sys_spu_thread_write_spu_mb failed %d\n", ret);
			break;
		}
	}
	
	sys_ppu_thread_exit(0);
}

