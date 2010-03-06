/* @(#)17	1.4  src/include/cbe_mfc.h, sw.includes, sdk_pub 10/11/05 16:00:25 */
/* -------------------------------------------------------------- */
/* (C) Copyright 2001,2005,                                       */
/* International Business Machines Corporation,                   */
/* Sony Computer Entertainment Incorporated,                      */
/* Toshiba Corporation.                                           */
/*                                                                */
/* All Rights Reserved.                                           */
/* -------------------------------------------------------------- */
/* PROLOG END TAG zYx                                              */
#ifndef _CBEA_MFC_H_
#define _CBEA_MFC_H_

/* This header file contains various definitions related to the Memory Flow
 *  Controller (MFC) portion of the Cell Broadband Engine Architecture (CBEA).
 */

/**************************************/
/* MFC DMA Command Opcode Definitions */
/**************************************/

/****************************************************************************/
/* MFC DMA Command flags which identify classes of operations. */
/****************************************************************************/
/* Note: These flags may be used in conjunction with the base command types
 *	 (i.e. MFC_PUT_CMD, MFC_PUTR_CMD, MFC_GET_CMD, and MFC_SNDSIG_CMD)
 *	 to construct the various command permutations.
 */

#define MFC_BARRIER_ENABLE      0x01
#define MFC_FENCE_ENABLE        0x02
#define MFC_LIST_ENABLE         0x04  /* SPU Only */
#define MFC_START_ENABLE        0x08  /* proxy Only */
#define MFC_RESULT_ENABLE       0x10

/****************************************************************************/
/* MFC DMA Put Commands */
/****************************************************************************/

#define MFC_PUT_CMD             0x20
#define MFC_PUTS_CMD            0x28  /* proxy Only */
#define MFC_PUTR_CMD            0x30
#define MFC_PUTF_CMD            0x22
#define MFC_PUTB_CMD            0x21
#define MFC_PUTFS_CMD           0x2A  /* proxy Only */
#define MFC_PUTBS_CMD           0x29  /* proxy Only */
#define MFC_PUTRF_CMD           0x32
#define MFC_PUTRB_CMD           0x31
#define MFC_PUTL_CMD            0x24  /* SPU Only */
#define MFC_PUTRL_CMD           0x34  /* SPU Only */
#define MFC_PUTLF_CMD           0x26  /* SPU Only */
#define MFC_PUTLB_CMD           0x25  /* SPU Only */
#define MFC_PUTRLF_CMD          0x36  /* SPU Only */
#define MFC_PUTRLB_CMD          0x35  /* SPU Only */

/****************************************************************************/
/* MFC DMA Get Commands */
/****************************************************************************/

#define MFC_GET_CMD             0x40
#define MFC_GETS_CMD            0x48  /* proxy Only */
#define MFC_GETF_CMD            0x42
#define MFC_GETB_CMD            0x41
#define MFC_GETFS_CMD           0x4A  /* proxy Only */
#define MFC_GETBS_CMD           0x49  /* proxy Only */
#define MFC_GETL_CMD            0x44  /* SPU Only */
#define MFC_GETLF_CMD           0x46  /* SPU Only */
#define MFC_GETLB_CMD           0x45  /* SPU Only */

/****************************************************************************/
/* MFC DMA Storage Control Commands */
/****************************************************************************/
/* Note: These are only supported on implementations with a SL1 cache
 *	 They are no-ops on the initial (CBE) implementation.
 */

#define MFC_SDCRT_CMD           0x80
#define MFC_SDCRTST_CMD         0x81
#define MFC_SDCRZ_CMD           0x89
#define MFC_SDCRS_CMD           0x8D
#define MFC_SDCRF_CMD           0x8F

/****************************************************************************/
/* MFC Synchronization Commands */
/****************************************************************************/

#define MFC_GETLLAR_CMD         0xD0  /* SPU Only */
#define MFC_PUTLLC_CMD          0xB4  /* SPU Only */
#define MFC_PUTLLUC_CMD         0xB0  /* SPU Only */
#define MFC_PUTQLLUC_CMD	0xB8  /* SPU Only */

#define MFC_SNDSIG_CMD          0xA0
#define MFC_SNDSIGB_CMD         0xA1
#define MFC_SNDSIGF_CMD         0xA2
#define MFC_BARRIER_CMD         0xC0
#define MFC_EIEIO_CMD           0xC8
#define MFC_SYNC_CMD            0xCC


/****************************************************************************/
/* Definitions for constructing a 32-bit command word including the transfer
 *  and replacement class id and the command opcode.
 */
/****************************************************************************/
#define MFC_TCLASS(_tid)       ((_tid) << 24)
#define MFC_RCLASS(_rid)       ((_rid) << 16)

#define MFC_CMD_WORD(_tid, _rid, _cmd) (MFC_TCLASS(_tid) | MFC_RCLASS(_rid) | (_cmd))

/****************************************************************************/
/* Definitions for constructing a 64-bit command word including the size, tag,
 *  transfer and replacement class id and the command opcode.
 */
/****************************************************************************/
#define MFC_SIZE(_size)        ((unsigned long long)(_size) << 48)
#define MFC_TAG(_tag_id)       ((unsigned long long)(_tag_id) << 32)
#define MFC_TR_CMD(_trcmd)     ((unsigned long long)(_trcmd))

#define MFC_CMD_DWORD(_size, _tag_id, _trcmd)  (MFC_SIZE(_size) | MFC_TAG(_tag_id) | MFC_TR_CMD(_trcmd))

/****************************************************************************/
/* Mask definitions for obtaining DMA commands and class ids from packed words.
 */
/****************************************************************************/
#define MFC_CMD_MASK       0x0000FFFF
#define MFC_CLASS_MASK     0x000000FF

/****************************************************************************/
/* DMA max/min size definitions. */
/****************************************************************************/
#define MFC_MIN_DMA_SIZE_SHIFT	4	/* 16 bytes */
#define MFC_MAX_DMA_SIZE_SHIFT	14	/* 16384 bytes */

#define MFC_MIN_DMA_SIZE	(1 << MFC_MIN_DMA_SIZE_SHIFT)
#define MFC_MAX_DMA_SIZE	(1 << MFC_MAX_DMA_SIZE_SHIFT)

#define MFC_MIN_DMA_SIZE_MASK	(MFC_MIN_DMA_SIZE - 1)
#define MFC_MAX_DMA_SIZE_MASK	(MFC_MAX_DMA_SIZE - 1)

#define MFC_MIN_DMA_LIST_SIZE   0x0008  /*   8 bytes */
#define MFC_MAX_DMA_LIST_SIZE   0x4000  /* 16K bytes */

/****************************************************************************/
/* Mask definition for checking proper address alignment. */
/****************************************************************************/
#define MFC_ADDR_MATCH_MASK     0xF
#define MFC_BEST_ADDR_ALIGNMENT 0x80

/****************************************************************************/
/* Definitions related to the Proxy DMA Command Status register (DMA_CMDStatus).
 */
/****************************************************************************/
#define MFC_PROXY_DMA_CMD_ENQUEUE_SUCCESSFUL     0x00
#define MFC_PROXY_DMA_CMD_SEQUENCE_ERROR         0x01
#define MFC_PROXY_DMA_QUEUE_FULL                 0x02

/****************************************************************************/
/* Definitions related to the DMA Queue Status register (DMA_QStatus). */
/****************************************************************************/
#define MFC_PROXY_MAX_QUEUE_SPACE		0x08
#define MFC_PROXY_DMA_Q_EMPTY			0x80000000
#define MFC_PROXY_DMA_Q_FREE_SPACE_MASK		0x0000FFFF

#define MFC_SPU_MAX_QUEUE_SPACE			0x10

/****************************************************************************/
/* Definitions related to the Proxy Tag-Group Query-Type register
 * (Prxy_QueryType). 
 */
/****************************************************************************/
#define MFC_PROXY_DMA_QUERYTYPE_ANY	0x1
#define MFC_PROXY_DMA_QUERYTYPE_ALL	0x2

/****************************************************************************/
/* Definitions related to the Proxy Tag-Group Query-Mask (Prxy_QueryMask)
 * and PU Tag Status (DMA_TagStatus) registers.
 *
 * NOTE: The only use the bottom 5 bits of the tag id value passed to insure
 *       a valid tag id is used.
 */
/****************************************************************************/

#define MFC_TAGID_TO_TAGMASK(tag_id)  (1 << (tag_id & 0x1F))

/****************************************************************************/
/* Definitions related to the Mailbox Status register (SPU_Mbox_Stat) and the 
 * depths of the outbound Mailbox Register (SPU_OutMbox), the outbound 
 * interrupting Mailbox Register (SPU_OutIntrMbox), and the inbound Mailbox
 * Register (SPU_In_Mbox).
 */
/****************************************************************************/
#define MFC_SPU_OUT_MBOX_COUNT_STATUS_MASK	0x000000FF
#define MFC_SPU_OUT_MBOX_COUNT_STATUS_SHIFT	0x0
#define MFC_SPU_IN_MBOX_COUNT_STATUS_MASK	0x0000FF00
#define MFC_SPU_IN_MBOX_COUNT_STATUS_SHIFT	0x8
#define MFC_SPU_OUT_INTR_MBOX_COUNT_STATUS_MASK	0x00FF0000
#define MFC_SPU_OUT_INTR_MBOX_COUNT_STATUS_SHIFT 0x10

/****************************************************************************/
/* Definitions related to the SPC Multi Source Syncronization register
 * (MFC_MSSync). 
 */
/****************************************************************************/
#define MFC_SPC_MSS_STATUS_MASK                      0x1
#define MFC_SPC_MSS_COMPLETE                         0x0
#define MFC_SPC_MSS_NOT_COMPLETE                     0x1


/*******************************************
 * Channel Defines
 *******************************************/

/* Events Defines for channels:
 *    0 (SPU_RdEventStat),
 *    1 (SPU_WrEventMask), and 
 *    2 (SPU_WrEventAck).
 */
#define MFC_TAG_STATUS_UPDATE_EVENT		0x00000001
#define MFC_LIST_STALL_NOTIFY_EVENT		0x00000002
#define MFC_COMMAND_QUEUE_AVAILABLE_EVENT	0x00000008
#define MFC_IN_MBOX_AVAILABLE_EVENT		0x00000010
#define MFC_DECREMENTER_EVENT			0x00000020
#define MFC_OUT_INTR_MBOX_AVAILABLE_EVENT	0x00000040
#define MFC_OUT_MBOX_AVAILABLE_EVENT		0x00000080
#define MFC_SIGNAL_NOTIFY_2_EVENT		0x00000100
#define MFC_SIGNAL_NOTIFY_1_EVENT		0x00000200
#define MFC_LLR_LOST_EVENT			0x00000400
#define MFC_PRIV_ATTN_EVENT		        0x00000800
#define MFC_MULTI_SRC_SYNC_EVENT            	0x00001000



/* Tag Status Update defines for channel 23 (MFC_WrTagUpdate)
 */
#define MFC_TAG_UPDATE_IMMEDIATE		0x0
#define	MFC_TAG_UPDATE_ANY			0x1
#define	MFC_TAG_UPDATE_ALL			0x2

/* Atomic Command Status defines for channel 27 (MFC_RdAtomicStat)
 */
#define MFC_PUTLLC_STATUS			0x00000001
#define	MFC_PUTLLUC_STATUS		        0x00000002
#define	MFC_GETLLAR_STATUS		    	0x00000004

#endif /* _CBEA_MFC_H_ */
