/* Copyright � 2010 - 2013 UNISYS CORPORATION
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 * NON INFRINGEMENT.  See the GNU General Public License for more
 * details.
 */

#ifndef __CONTROLVMCHANNEL_H__
#define __CONTROLVMCHANNEL_H__

#include "commontypes.h"
#include "channel.h"
#include "controlframework.h"
enum { INVALID_GUEST_FIRMWARE, SAMPLE_GUEST_FIRMWARE,
	    TIANO32_GUEST_FIRMWARE, TIANO64_GUEST_FIRMWARE
};

/* {2B3C2D10-7EF5-4ad8-B966-3448B7386B3D} */
#define ULTRA_CONTROLVM_CHANNEL_PROTOCOL_GUID	\
	{0x2b3c2d10, 0x7ef5, 0x4ad8, \
		{0xb9, 0x66, 0x34, 0x48, 0xb7, 0x38, 0x6b, 0x3d} }

static const GUID UltraControlvmChannelProtocolGuid =
	ULTRA_CONTROLVM_CHANNEL_PROTOCOL_GUID;

#define ULTRA_CONTROLVM_CHANNEL_PROTOCOL_SIGNATURE \
	ULTRA_CHANNEL_PROTOCOL_SIGNATURE
#define CONTROLVM_MESSAGE_MAX     64

/* Must increment this whenever you insert or delete fields within
* this channel struct.  Also increment whenever you change the meaning
* of fields within this channel struct so as to break pre-existing
* software.  Note that you can usually add fields to the END of the
* channel struct withOUT needing to increment this. */
#define ULTRA_CONTROLVM_CHANNEL_PROTOCOL_VERSIONID  1

#define ULTRA_CONTROLVM_CHANNEL_OK_CLIENT(pChannel, logCtx)           \
	(ULTRA_check_channel_client(pChannel, \
		UltraControlvmChannelProtocolGuid, \
		"controlvm", \
		sizeof(ULTRA_CONTROLVM_CHANNEL_PROTOCOL), \
		ULTRA_CONTROLVM_CHANNEL_PROTOCOL_VERSIONID, \
		ULTRA_CONTROLVM_CHANNEL_PROTOCOL_SIGNATURE, \
		__FILE__, __LINE__, logCtx))
#define ULTRA_CONTROLVM_CHANNEL_OK_SERVER(actualBytes, logCtx)        \
	(ULTRA_check_channel_server(UltraControlvmChannelProtocolGuid,	\
				    "controlvm",			\
				    sizeof(ULTRA_CONTROLVM_CHANNEL_PROTOCOL), \
				    actualBytes, __FILE__, __LINE__, logCtx))

#define MY_DEVICE_INDEX 0
#define MAX_MACDATA_LEN 8 /* number of bytes for MAC address in config packet */
#define MAX_SERIAL_NUM	32

#define DISK_ZERO_PUN_NUMBER	1  /* Target ID on the SCSI bus for LUN 0 */
#define DISK_ZERO_LUN_NUMBER	3  /* Logical Unit Number */

/* Defines for various channel queues... */
#define CONTROLVM_QUEUE_REQUEST		0
#define CONTROLVM_QUEUE_RESPONSE	1
#define	CONTROLVM_QUEUE_EVENT		2
#define CONTROLVM_QUEUE_ACK		3

/* Max number of messages stored during IOVM creation to be reused
 * after crash */
#define CONTROLVM_CRASHMSG_MAX		2

/** Ids for commands that may appear in either queue of a ControlVm channel.
 *
 *  Commands that are initiated by the command partition (CP), by an IO or
 *  console service partition (SP), or by a guest partition (GP)are:
 *  - issued on the RequestQueue queue (q #0) in the ControlVm channel
 *  - responded to on the ResponseQueue queue (q #1) in the ControlVm channel
 *
 *  Events that are initiated by an IO or console service partition (SP) or
 *  by a guest partition (GP) are:
 *  - issued on the EventQueue queue (q #2) in the ControlVm channel
 *  - responded to on the EventAckQueue queue (q #3) in the ControlVm channel
 */
typedef enum  {
	CONTROLVM_INVALID = 0,
	/* SWITCH commands required Parameter: SwitchNumber  */
	/* BUS commands required Parameter: BusNumber  */
	CONTROLVM_BUS_CREATE = 0x101,	/* CP --> SP, GP */
	CONTROLVM_BUS_DESTROY = 0x102,	/* CP --> SP, GP */
	CONTROLVM_BUS_CONFIGURE = 0x104,	/* CP --> SP */
	CONTROLVM_BUS_CHANGESTATE = 0x105,	/* CP --> SP, GP */
	CONTROLVM_BUS_CHANGESTATE_EVENT = 0x106, /* SP, GP --> CP */
/* DEVICE commands required Parameter: BusNumber, DeviceNumber  */

	CONTROLVM_DEVICE_CREATE = 0x201,	/* CP --> SP, GP */
	CONTROLVM_DEVICE_DESTROY = 0x202,	/* CP --> SP, GP */
	CONTROLVM_DEVICE_CONFIGURE = 0x203,	/* CP --> SP */
	CONTROLVM_DEVICE_CHANGESTATE = 0x204,	/* CP --> SP, GP */
	CONTROLVM_DEVICE_CHANGESTATE_EVENT = 0x205, /* SP, GP --> CP */
	CONTROLVM_DEVICE_RECONFIGURE = 0x206,	/* CP --> Boot */
/* DISK commands required Parameter: BusNumber, DeviceNumber  */
	CONTROLVM_DISK_CREATE = 0x221,	/* CP --> SP */
	CONTROLVM_DISK_DESTROY = 0x222,	/* CP --> SP */
	CONTROLVM_DISK_CONFIGURE = 0x223,	/* CP --> SP */
	CONTROLVM_DISK_CHANGESTATE = 0x224,	/* CP --> SP */
/* CHIPSET commands */
	CONTROLVM_CHIPSET_INIT = 0x301,	/* CP --> SP, GP */
	CONTROLVM_CHIPSET_STOP = 0x302,	/* CP --> SP, GP */
	CONTROLVM_CHIPSET_SHUTDOWN = 0x303,	/* CP --> SP */
	CONTROLVM_CHIPSET_READY = 0x304,	/* CP --> SP */
	CONTROLVM_CHIPSET_SELFTEST = 0x305,	/* CP --> SP */

} CONTROLVM_ID;

struct InterruptInfo {
	 /**< specifies interrupt info. It is used to send interrupts
	  *   for this channel. The peer at the end of this channel
	  *   who has registered an interrupt (using recv fields
	  *   above) will receive the interrupt. Passed as a parameter
	  *   to Issue_VMCALL_IO_QUEUE_TRANSITION, which generates the
	  *   interrupt.  Currently this is used by IOPart-SP to wake
	  *   up GP when Data Channel transitions from empty to
	  *   non-empty.*/
	U64 sendInterruptHandle;

	 /**< specifies interrupt handle. It is used to retrieve the
	  *   corresponding interrupt pin from Monitor; and the
	  *   interrupt pin is used to connect to the corresponding
	  *   intrrupt.  Used by IOPart-GP only. */
	U64 recvInterruptHandle;

	 /**< specifies interrupt vector. It, interrupt pin, and shared are
	  *   used to connect to the corresponding interrupt.  Used by
	  *   IOPart-GP only. */
	U32 recvInterruptVector;

    /**< specifies if the recvInterrupt is shared.  It, interrupt pin
     *   and vector are used to connect to 0 = not shared; 1 = shared.
     *   the corresponding interrupt.  Used by IOPart-GP only. */
	U8 recvInterruptShared;
	U8 reserved[3];	/* Natural alignment purposes */
};

struct PciId {
	U16 Domain;
	U8 Bus;
	U8 Slot;
	U8 Func;
	U8 Reserved[3];	/* Natural alignment purposes */
};

struct PciConfigHdr {
	U16 VendorId;
	U16 SubSysVendor;
	U16 DeviceId;
	U16 SubSysDevice;
	U32 ClassCode;
	U32 Reserved;		/* Natural alignment purposes */
};

struct ScsiId {
	U32 Bus;
	U32 Target;
	U32 Lun;
	U32 Host; /* Command should ignore this for *
		   * DiskArrival/RemovalEvents */
};

struct WWID {
	U32 wwid1;
	U32 wwid2;
};

struct virtDiskInfo  {
	U32 switchNo;		/* defined by SWITCH_CREATE */
	U32 externalPortNo;	/* 0 for SAS RAID provided (external)
				 * virtual disks, 1 for virtual disk
				 * images, 2 for gold disk images */
	U16 VirtualDiskIndex;	/* Index of disk descriptor in the
				 * VirtualDisk segment associated with
				 * externalPortNo */
	U16 Reserved1;
	U32 Reserved2;
};

typedef enum {
	CONTROLVM_ACTION_NONE = 0,
	CONTROLVM_ACTION_SET_RESTORE = 0x05E7,
	CONTROLVM_ACTION_CLEAR_RESTORE = 0x0C18,
	CONTROLVM_ACTION_RESTORING = 0x08E5,
	CONTROLVM_ACTION_RESTORE_BUSY = 0x0999,
	CONTROLVM_ACTION_CLEAR_NVRAM = 0xB01
} CONTROLVM_ACTION;

typedef enum _ULTRA_TOOL_ACTIONS {
	    /* enumeration that defines intended action  */
	    ULTRA_TOOL_ACTION_NONE = 0,	/* normal boot of boot disk */
	ULTRA_TOOL_ACTION_INSTALL = 1,	/* install source disk(s) to boot
					 * disk */
	ULTRA_TOOL_ACTION_CAPTURE = 2,	/* capture boot disk to target disk(s)
					 * as 'gold image' */
	ULTRA_TOOL_ACTION_REPAIR = 3,	/* use source disk(s) to repair
					 * installation on boot disk */
	ULTRA_TOOL_ACTION_CLEAN = 4,	/* 'scrub' virtual disk before
					 * releasing back to storage pool */
	ULTRA_TOOL_ACTION_UPGRADE = 5,	/* upgrade to use content of images
					 * referenced from newer blueprint */
	ULTRA_TOOL_ACTION_DIAG = 6,	/* use tool to invoke diagnostic script
					 * provided by blueprint */
	ULTRA_TOOL_ACTION_FAILED = 7,	/* used when tool fails installation
					   and cannot continue */
	ULTRA_TOOL_ACTION_COUNT = 8
} ULTRA_TOOL_ACTIONS;

typedef struct _ULTRA_EFI_SPAR_INDICATION  {
	U64 BootToFirmwareUI:1;	/* Bit 0: Stop in uefi ui */
	U64 ClearNvram:1;	/* Bit 1: Clear NVRAM */
	U64 ClearCmos:1;	/* Bit 2: Clear CMOS */
	U64 BootToTool:1;	/* Bit 3: Run install tool */
	/* remaining bits are available */
} ULTRA_EFI_SPAR_INDICATION;

typedef enum {
	ULTRA_CHIPSET_FEATURE_REPLY = 0x00000001,
	ULTRA_CHIPSET_FEATURE_PARA_HOTPLUG = 0x00000002,
	ULTRA_CHIPSET_FEATURE_PCIVBUS = 0x00000004
} ULTRA_CHIPSET_FEATURE;

/** This is the common structure that is at the beginning of every
 *  ControlVm message (both commands and responses) in any ControlVm
 *  queue.  Commands are easily distinguished from responses by
 *  looking at the flags.response field.
 */
typedef struct _CONTROLVM_MESSAGE_HEADER  {
	U32 Id;		/* See CONTROLVM_ID. */
	/* For requests, indicates the message type. */
	/* For responses, indicates the type of message we are responding to. */

	U32 MessageSize;	/* Includes size of this struct + size
				 * of message */
	U32 SegmentIndex;	/* Index of segment containing Vm
				 * message/information */
	U32 CompletionStatus;	/* Error status code or result of
				 * message completion */
	struct  {
		U32 failed:1;		   /**< =1 in a response to * signify
					    * failure */
		U32 responseExpected:1;   /**< =1 in all messages that expect a
					   * response (Control ignores this
					   * bit) */
		U32 server:1;		   /**< =1 in all bus & device-related
					    * messages where the message
					    * receiver is to act as the bus or
					    * device server */
		U32 testMessage:1;	   /**< =1 for testing use only
					    * (Control and Command ignore this
					    * bit) */
		U32 partialCompletion:1;  /**< =1 if there are forthcoming
					   * responses/acks associated
					   * with this message */
		U32 preserve:1;	       /**< =1 this is to let us know to
					* preserve channel contents
					* (for running guests)*/
		U32 writerInDiag:1;	/**< =1 the DiagWriter is active in the
					 * Diagnostic Partition*/

		    /* remaining bits in this 32-bit word are available */
	} Flags;
	U32 Reserved;		/* Natural alignment */
	U64 MessageHandle;	/* Identifies the particular message instance,
				 * and is used to match particular */
	/* request instances with the corresponding response instance. */
	U64 PayloadVmOffset;	/* Offset of payload area from start of this
				 * instance of ControlVm segment */
	U32 PayloadMaxBytes;	/* Maximum bytes allocated in payload
				 * area of ControlVm segment */
	U32 PayloadBytes;	/* Actual number of bytes of payload
				 * area to copy between IO/Command; */
	/* if non-zero, there is a payload to copy. */
} CONTROLVM_MESSAGE_HEADER;

typedef struct _CONTROLVM_PACKET_DEVICE_CREATE  {
	U32 busNo;	   /**< bus # (0..n-1) from the msg receiver's
			    * perspective */

	    /* Control uses header SegmentIndex field to access bus number... */
	U32 devNo;	   /**< bus-relative (0..n-1) device number */
	U64 channelAddr;  /**< Guest physical address of the channel, which
			*   can be dereferenced by the receiver
			*   of this ControlVm command */
	U64 channelBytes; /**< specifies size of the channel in bytes */
	GUID dataTypeGuid;/**< specifies format of data in channel */
	GUID devInstGuid; /**< instance guid for the device */
	struct InterruptInfo intr; /**< specifies interrupt information */
} CONTROLVM_PACKET_DEVICE_CREATE;	/* for CONTROLVM_DEVICE_CREATE */

typedef struct _CONTROLVM_PACKET_DEVICE_CONFIGURE  {
	U32 busNo;	      /**< bus # (0..n-1) from the msg
			       * receiver's perspective */

	    /* Control uses header SegmentIndex field to access bus number... */
	U32 devNo;	      /**< bus-relative (0..n-1) device number */
} CONTROLVM_PACKET_DEVICE_CONFIGURE;	/* for CONTROLVM_DEVICE_CONFIGURE */

typedef struct _CONTROLVM_MESSAGE_DEVICE_CREATE  {
	CONTROLVM_MESSAGE_HEADER Header;
	CONTROLVM_PACKET_DEVICE_CREATE Packet;
} CONTROLVM_MESSAGE_DEVICE_CREATE;	/* total 128 bytes */

typedef struct _CONTROLVM_MESSAGE_DEVICE_CONFIGURE  {
	CONTROLVM_MESSAGE_HEADER Header;
	CONTROLVM_PACKET_DEVICE_CONFIGURE Packet;
} CONTROLVM_MESSAGE_DEVICE_CONFIGURE;	/* total 56 bytes */

/* This is the format for a message in any ControlVm queue. */
typedef struct _CONTROLVM_MESSAGE_PACKET  {
	union  {

		/* BEGIN Request messages */
		struct  {
			U32 busNo;	      /*< bus # (0..n-1) from the msg
					       * receiver's perspective */

	    /* Control uses header SegmentIndex field to access bus number... */
			U32 deviceCount;      /*< indicates the max number of
					       * devices on this bus */
			U64 channelAddr;     /*< Guest physical address of the
					      *   channel, which can be
					      *   dereferenced by the receiver
					      *   of this ControlVm command */
			U64 channelBytes;    /*< size of the channel in bytes */
			GUID busDataTypeGuid;/*< indicates format of data in bus
					      * channel */
			GUID busInstGuid;    /*< instance guid for the bus */
		} createBus;	/* for CONTROLVM_BUS_CREATE */
		struct  {
			U32 busNo;	      /*< bus # (0..n-1) from the msg
					       * receiver's perspective */

	    /* Control uses header SegmentIndex field to access bus number... */
			U32 reserved;	/* Natural alignment purposes */
		} destroyBus;	/* for CONTROLVM_BUS_DESTROY */
		struct  {
			U32 busNo;		    /*< bus # (0..n-1) from the
						     * msg receiver's
						     * perspective */

	    /* Control uses header SegmentIndex field to access bus number... */
			U32 reserved1;		    /* for alignment purposes */
			U64 guestHandle;	    /* This is used to convert
					 *  guest physical address to real
					 *  physical address for DMA, for ex. */
			U64 recvBusInterruptHandle;/*< specifies interrupt
					 *   info. It is used by SP to register
					 *   to receive interrupts from the CP.
					 *   This interrupt is used for bus
					 *   level notifications.  The
					 *   corresponding
					 *   sendBusInterruptHandle is kept in
					 *   CP. */
		} configureBus;	/* for CONTROLVM_BUS_CONFIGURE */

		/* for CONTROLVM_DEVICE_CREATE */
		CONTROLVM_PACKET_DEVICE_CREATE createDevice;
		struct  {
			U32 busNo;	      /*< bus # (0..n-1) from the msg
					       * receiver's perspective */

	    /* Control uses header SegmentIndex field to access bus number... */
			U32 devNo;	      /*< bus-relative (0..n-1) device
					       * number */
		} destroyDevice;	/* for CONTROLVM_DEVICE_DESTROY */

		/* for CONTROLVM_DEVICE_CONFIGURE */
		CONTROLVM_PACKET_DEVICE_CONFIGURE configureDevice;
		struct  {
			U32 busNo;	      /*< bus # (0..n-1) from the msg
					       * receiver's perspective */

	    /* Control uses header SegmentIndex field to access bus number... */
			U32 devNo;	      /*< bus-relative (0..n-1) device
					       * number */
		} reconfigureDevice;	/* for CONTROLVM_DEVICE_RECONFIGURE */
		struct  {
			U32 busNo;
			ULTRA_SEGMENT_STATE state;
			U8 reserved[2];	/* Natural alignment purposes */
		} busChangeState;	/* for CONTROLVM_BUS_CHANGESTATE */
		struct  {
			U32 busNo;
			U32 devNo;
			ULTRA_SEGMENT_STATE state;
			struct  {
				U32 physicalDevice:1;	/* =1 if message is for
							 * a physical device */
			/* remaining bits in this 32-bit word are available */
			} flags;
			U8 reserved[2];	/* Natural alignment purposes */
		} deviceChangeState;	/* for CONTROLVM_DEVICE_CHANGESTATE */
		struct  {
			U32 busNo;
			U32 devNo;
			ULTRA_SEGMENT_STATE state;
			U8 reserved[6];	/* Natural alignment purposes */
		} deviceChangeStateEvent; /* for CONTROLVM_DEVICE_CHANGESTATE_EVENT */
		struct  {
			U32 busCount; /*< indicates the max number of busses */
			U32 switchCount; /*< indicates the max number of
					  *   switches (applicable for service
					  *   partition only) */
			ULTRA_CHIPSET_FEATURE features;
			U32 platformNumber;	/* Platform Number */
		} initChipset;	/* for CONTROLVM_CHIPSET_INIT */
		struct  {
			U32 Options; /*< reserved */
			U32 Test;    /*< bit 0 set to run embedded selftest */
		} chipsetSelftest;	/* for CONTROLVM_CHIPSET_SELFTEST */

		    /* END Request messages */

		    /* BEGIN Response messages */

		    /* END Response messages */

		    /* BEGIN Event messages */

		    /* END Event messages */

		    /* BEGIN Ack messages */

		    /* END Ack messages */
		U64 addr;	    /*< a physical address of something, that
				     *   can be dereferenced by the receiver of
				     *   this ControlVm command (depends on
				     *   command id) */
		U64 handle;	    /*< a handle of something (depends on
				     * command id) */
	};
} CONTROLVM_MESSAGE_PACKET;

/* All messages in any ControlVm queue have this layout. */
typedef struct _CONTROLVM_MESSAGE  {
	CONTROLVM_MESSAGE_HEADER hdr;
	CONTROLVM_MESSAGE_PACKET cmd;
} CONTROLVM_MESSAGE;

typedef struct _DEVICE_MAP  {
	GUEST_PHYSICAL_ADDRESS DeviceChannelAddress;
	U64 DeviceChannelSize;
	U32 CA_Index;
	U32 Reserved;		/* natural alignment */
	U64 Reserved2;		/* Align structure on 32-byte boundary */
} DEVICE_MAP;

typedef struct _GUEST_DEVICES  {
	DEVICE_MAP VideoChannel;
	DEVICE_MAP KeyboardChannel;
	DEVICE_MAP NetworkChannel;
	DEVICE_MAP StorageChannel;
	DEVICE_MAP ConsoleChannel;
	U32 PartitionIndex;
	U32 Pad;
} GUEST_DEVICES;

typedef struct _ULTRA_CONTROLVM_CHANNEL_PROTOCOL  {
	 CHANNEL_HEADER Header;
	 GUEST_PHYSICAL_ADDRESS gpControlVm;	/* guest physical address of
						 * this channel */
	 GUEST_PHYSICAL_ADDRESS gpPartitionTables; /* guest physical address of
						    * partition tables */
	 GUEST_PHYSICAL_ADDRESS gpDiagGuest;	/* guest physical address of
						 * diagnostic channel */
	 GUEST_PHYSICAL_ADDRESS gpBootRomDisk;	/* guest phys addr of (read
						 * only) Boot ROM disk */
	 GUEST_PHYSICAL_ADDRESS gpBootRamDisk;	/* guest phys addr of writable
						 * Boot RAM disk */
	 GUEST_PHYSICAL_ADDRESS gpAcpiTable;	/* guest phys addr of acpi
						 * table */
	 GUEST_PHYSICAL_ADDRESS gpControlChannel; /* guest phys addr of control
						   * channel */
	 GUEST_PHYSICAL_ADDRESS gpDiagRomDisk;	/* guest phys addr of diagnostic
						 * ROM disk */
	 GUEST_PHYSICAL_ADDRESS gpNvram;	/* guest phys addr of NVRAM
						 * channel */
	 U64 RequestPayloadOffset;	/* Offset to request payload area */
	 U64 EventPayloadOffset;	/* Offset to event payload area */
	 U32 RequestPayloadBytes;	/* Bytes available in request payload
					 * area */
	 U32 EventPayloadBytes;	/* Bytes available in event payload area */
	 U32 ControlChannelBytes;
	 U32 NvramChannelBytes;	/* Bytes in PartitionNvram segment */
	 U32 MessageBytes;	/* sizeof(CONTROLVM_MESSAGE) */
	 U32 MessageCount;	/* CONTROLVM_MESSAGE_MAX */
	 GUEST_PHYSICAL_ADDRESS gpSmbiosTable;	/* guest phys addr of SMBIOS
						 * tables */
	 GUEST_PHYSICAL_ADDRESS gpPhysicalSmbiosTable;	/* guest phys addr of
							 * SMBIOS table  */
	 /* ULTRA_MAX_GUESTS_PER_SERVICE */
	 GUEST_DEVICES gpObsoleteGuestDevices[16];

	 /* guest physical address of EFI firmware image base  */
	 GUEST_PHYSICAL_ADDRESS VirtualGuestFirmwareImageBase;

	 /* guest physical address of EFI firmware entry point  */
	 GUEST_PHYSICAL_ADDRESS VirtualGuestFirmwareEntryPoint;

	 /* guest EFI firmware image size  */
	 U64 VirtualGuestFirmwareImageSize;

	 /* GPA = 1MB where EFI firmware image is copied to  */
	 GUEST_PHYSICAL_ADDRESS VirtualGuestFirmwareBootBase;
	 GUEST_PHYSICAL_ADDRESS VirtualGuestImageBase;
	 GUEST_PHYSICAL_ADDRESS VirtualGuestImageSize;
	 U64 PrototypeControlChannelOffset;
	 GUEST_PHYSICAL_ADDRESS VirtualGuestPartitionHandle;

	 U16 RestoreAction;	/* Restore Action field to restore the guest
				 * partition */
	U16 DumpAction;		/* For Windows guests it shows if the visordisk
				 * is running in dump mode */
	U16 NvramFailCount;
	U16 SavedCrashMsgCount;	/* = CONTROLVM_CRASHMSG_MAX */
	U32 SavedCrashMsgOffset;	/* Offset to request payload area needed
					 * for crash dump */
	U32 InstallationError;	/* Type of error encountered during
				 * installation */
	U32 InstallationTextId;	/* Id of string to display */
	U16 InstallationRemainingSteps;	/* Number of remaining installation
					 * steps (for progress bars) */
	U8 ToolAction;		/* ULTRA_TOOL_ACTIONS Installation Action
				 * field */
	U8 Reserved;		/* alignment */
	ULTRA_EFI_SPAR_INDICATION EfiSparIndication;
	ULTRA_EFI_SPAR_INDICATION EfiSparIndicationSupported;
	U32 SPReserved;
	U8 Reserved2[28];	/* Force signals to begin on 128-byte cache
				 * line */
	SIGNAL_QUEUE_HEADER RequestQueue;	/* Service or guest partition
						 * uses this queue to send
						 * requests to Control */
	SIGNAL_QUEUE_HEADER ResponseQueue;	/* Control uses this queue to
						 * respond to service or guest
						 * partition requests */
	SIGNAL_QUEUE_HEADER EventQueue;		/* Control uses this queue to
						 * send events to service or
						 * guest partition */
	SIGNAL_QUEUE_HEADER EventAckQueue;	/* Service or guest partition
						 * uses this queue to ack
						 * Control events */

	 /* Request fixed-size message pool - does not include payload */
	 CONTROLVM_MESSAGE RequestMsg[CONTROLVM_MESSAGE_MAX];

	 /* Response fixed-size message pool - does not include payload */
	 CONTROLVM_MESSAGE ResponseMsg[CONTROLVM_MESSAGE_MAX];

	 /* Event fixed-size message pool - does not include payload */
	 CONTROLVM_MESSAGE EventMsg[CONTROLVM_MESSAGE_MAX];

	 /* Ack fixed-size message pool - does not include payload */
	 CONTROLVM_MESSAGE EventAckMsg[CONTROLVM_MESSAGE_MAX];

	 /* Message stored during IOVM creation to be reused after crash */
	 CONTROLVM_MESSAGE SavedCrashMsg[CONTROLVM_CRASHMSG_MAX];
} ULTRA_CONTROLVM_CHANNEL_PROTOCOL;

/* Offsets for VM channel attributes... */
#define VM_CH_REQ_QUEUE_OFFSET \
	offsetof(ULTRA_CONTROLVM_CHANNEL_PROTOCOL, RequestQueue)
#define VM_CH_RESP_QUEUE_OFFSET \
	offsetof(ULTRA_CONTROLVM_CHANNEL_PROTOCOL, ResponseQueue)
#define VM_CH_EVENT_QUEUE_OFFSET \
	offsetof(ULTRA_CONTROLVM_CHANNEL_PROTOCOL, EventQueue)
#define VM_CH_ACK_QUEUE_OFFSET \
	offsetof(ULTRA_CONTROLVM_CHANNEL_PROTOCOL, EventAckQueue)
#define VM_CH_REQ_MSG_OFFSET \
	offsetof(ULTRA_CONTROLVM_CHANNEL_PROTOCOL, RequestMsg)
#define VM_CH_RESP_MSG_OFFSET \
	offsetof(ULTRA_CONTROLVM_CHANNEL_PROTOCOL, ResponseMsg)
#define VM_CH_EVENT_MSG_OFFSET \
	offsetof(ULTRA_CONTROLVM_CHANNEL_PROTOCOL, EventMsg)
#define VM_CH_ACK_MSG_OFFSET \
	offsetof(ULTRA_CONTROLVM_CHANNEL_PROTOCOL, EventAckMsg)
#define VM_CH_CRASH_MSG_OFFSET \
	offsetof(ULTRA_CONTROLVM_CHANNEL_PROTOCOL, SavedCrashMsg)

/* The following header will be located at the beginning of PayloadVmOffset for
 *  various ControlVm commands. The receiver of a ControlVm command with a
 *  PayloadVmOffset will dereference this address and then use ConnectionOffset,
 *  InitiatorOffset, and TargetOffset to get the location of UTF-8 formatted
 *  strings that can be parsed to obtain command-specific information. The value
 *  of TotalLength should equal PayloadBytes.  The format of the strings at
 *  PayloadVmOffset will take different forms depending on the message.  See the
 *  following Wiki page for more information:
 *  https://ustr-linux-1.na.uis.unisys.com/spar/index.php/ControlVm_Parameters_Area
 */
typedef struct _ULTRA_CONTROLVM_PARAMETERS_HEADER  {
	U32 TotalLength;
	U32 HeaderLength;
	U32 ConnectionOffset;
	U32 ConnectionLength;
	U32 InitiatorOffset;
	U32 InitiatorLength;
	U32 TargetOffset;
	U32 TargetLength;
	U32 ClientOffset;
	U32 ClientLength;
	U32 NameOffset;
	U32 NameLength;
	GUID Id;
	U32 Revision;
	U32 Reserved;		/* Natural alignment */
} ULTRA_CONTROLVM_PARAMETERS_HEADER;

#endif				/* __CONTROLVMCHANNEL_H__ */
