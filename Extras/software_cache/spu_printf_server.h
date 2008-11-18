/*   SCE CONFIDENTIAL                                       */
/*   PLAYSTATION(R)3 Programmer Tool Runtime Library 085.007 */
/*   Copyright (C) 2005 Sony Computer Entertainment Inc.    */
/*   All Rights Reserved.                                   */

#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
void spu_printf_server_entry(uint64_t arg);
int spu_printf_server_initialize(void);
int spu_printf_server_finalize(void);
int spu_printf_server_register(sys_spu_thread_t spu);
int spu_printf_server_unregister(sys_spu_thread_t spu);
#ifdef __cplusplus
}
#endif /* __cplusplus */

