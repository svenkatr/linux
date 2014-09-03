/*
 * definition for kernel virtual machines on s390
 *
 * Copyright IBM Corp. 2008, 2009
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (version 2 only)
 * as published by the Free Software Foundation.
 *
 *    Author(s): Carsten Otte <cotte@de.ibm.com>
 */


#ifndef ASM_KVM_HOST_H
#define ASM_KVM_HOST_H
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/kvm_host.h>
#include <linux/kvm.h>
#include <asm/debug.h>
#include <asm/cpu.h>
#include <asm/isc.h>

#define KVM_MAX_VCPUS 64
#define KVM_USER_MEM_SLOTS 32

/*
 * These seem to be used for allocating ->chip in the routing table,
 * which we don't use. 4096 is an out-of-thin-air value. If we need
 * to look at ->chip later on, we'll need to revisit this.
 */
#define KVM_NR_IRQCHIPS 1
#define KVM_IRQCHIP_NUM_PINS 4096

struct sca_entry {
	atomic_t scn;
	__u32	reserved;
	__u64	sda;
	__u64	reserved2[2];
} __attribute__((packed));


struct sca_block {
	__u64	ipte_control;
	__u64	reserved[5];
	__u64	mcn;
	__u64	reserved2;
	struct sca_entry cpu[64];
} __attribute__((packed));

#define CPUSTAT_STOPPED    0x80000000
#define CPUSTAT_WAIT       0x10000000
#define CPUSTAT_ECALL_PEND 0x08000000
#define CPUSTAT_STOP_INT   0x04000000
#define CPUSTAT_IO_INT     0x02000000
#define CPUSTAT_EXT_INT    0x01000000
#define CPUSTAT_RUNNING    0x00800000
#define CPUSTAT_RETAINED   0x00400000
#define CPUSTAT_TIMING_SUB 0x00020000
#define CPUSTAT_SIE_SUB    0x00010000
#define CPUSTAT_RRF        0x00008000
#define CPUSTAT_SLSV       0x00004000
#define CPUSTAT_SLSR       0x00002000
#define CPUSTAT_ZARCH      0x00000800
#define CPUSTAT_MCDS       0x00000100
#define CPUSTAT_SM         0x00000080
#define CPUSTAT_G          0x00000008
#define CPUSTAT_GED        0x00000004
#define CPUSTAT_J          0x00000002
#define CPUSTAT_P          0x00000001

struct kvm_s390_sie_block {
	atomic_t cpuflags;		/* 0x0000 */
	__u32	prefix;			/* 0x0004 */
	__u8	reserved08[4];		/* 0x0008 */
#define PROG_IN_SIE (1<<0)
	__u32	prog0c;			/* 0x000c */
	__u8	reserved10[16];		/* 0x0010 */
#define PROG_BLOCK_SIE 0x00000001
	atomic_t prog20;		/* 0x0020 */
	__u8	reserved24[4];		/* 0x0024 */
	__u64	cputm;			/* 0x0028 */
	__u64	ckc;			/* 0x0030 */
	__u64	epoch;			/* 0x0038 */
	__u8	reserved40[4];		/* 0x0040 */
#define LCTL_CR0	0x8000
#define LCTL_CR6	0x0200
#define LCTL_CR14	0x0002
	__u16   lctl;			/* 0x0044 */
	__s16	icpua;			/* 0x0046 */
#define ICTL_LPSW 0x00400000
	__u32	ictl;			/* 0x0048 */
	__u32	eca;			/* 0x004c */
	__u8	icptcode;		/* 0x0050 */
	__u8	reserved51;		/* 0x0051 */
	__u16	ihcpu;			/* 0x0052 */
	__u8	reserved54[2];		/* 0x0054 */
	__u16	ipa;			/* 0x0056 */
	__u32	ipb;			/* 0x0058 */
	__u32	scaoh;			/* 0x005c */
	__u8	reserved60;		/* 0x0060 */
	__u8	ecb;			/* 0x0061 */
	__u8    ecb2;                   /* 0x0062 */
	__u8    reserved63[1];          /* 0x0063 */
	__u32	scaol;			/* 0x0064 */
	__u8	reserved68[4];		/* 0x0068 */
	__u32	todpr;			/* 0x006c */
	__u8	reserved70[32];		/* 0x0070 */
	psw_t	gpsw;			/* 0x0090 */
	__u64	gg14;			/* 0x00a0 */
	__u64	gg15;			/* 0x00a8 */
	__u8	reservedb0[30];		/* 0x00b0 */
	__u16   iprcc;			/* 0x00ce */
	__u8	reservedd0[48];		/* 0x00d0 */
	__u64	gcr[16];		/* 0x0100 */
	__u64	gbea;			/* 0x0180 */
	__u8	reserved188[24];	/* 0x0188 */
	__u32	fac;			/* 0x01a0 */
	__u8	reserved1a4[20];	/* 0x01a4 */
	__u64	cbrlo;			/* 0x01b8 */
	__u8	reserved1c0[30];	/* 0x01c0 */
	__u64	pp;			/* 0x01de */
	__u8	reserved1e6[2];		/* 0x01e6 */
	__u64	itdba;			/* 0x01e8 */
	__u8	reserved1f0[16];	/* 0x01f0 */
} __attribute__((packed));

struct kvm_s390_itdb {
	__u8	data[256];
} __packed;

struct sie_page {
	struct kvm_s390_sie_block sie_block;
	__u8 reserved200[1024];		/* 0x0200 */
	struct kvm_s390_itdb itdb;	/* 0x0600 */
	__u8 reserved700[2304];		/* 0x0700 */
} __packed;

struct kvm_vcpu_stat {
	u32 exit_userspace;
	u32 exit_null;
	u32 exit_external_request;
	u32 exit_external_interrupt;
	u32 exit_stop_request;
	u32 exit_validity;
	u32 exit_instruction;
	u32 instruction_lctl;
	u32 instruction_lctlg;
	u32 exit_program_interruption;
	u32 exit_instr_and_program;
	u32 deliver_external_call;
	u32 deliver_emergency_signal;
	u32 deliver_service_signal;
	u32 deliver_virtio_interrupt;
	u32 deliver_stop_signal;
	u32 deliver_prefix_signal;
	u32 deliver_restart_signal;
	u32 deliver_program_int;
	u32 deliver_io_int;
	u32 exit_wait_state;
	u32 instruction_pfmf;
	u32 instruction_stidp;
	u32 instruction_spx;
	u32 instruction_stpx;
	u32 instruction_stap;
	u32 instruction_storage_key;
	u32 instruction_stsch;
	u32 instruction_chsc;
	u32 instruction_stsi;
	u32 instruction_stfl;
	u32 instruction_tprot;
	u32 instruction_essa;
	u32 instruction_sigp_sense;
	u32 instruction_sigp_sense_running;
	u32 instruction_sigp_external_call;
	u32 instruction_sigp_emergency;
	u32 instruction_sigp_stop;
	u32 instruction_sigp_arch;
	u32 instruction_sigp_prefix;
	u32 instruction_sigp_restart;
	u32 diagnose_10;
	u32 diagnose_44;
	u32 diagnose_9c;
};

#define PGM_OPERATION            0x01
#define PGM_PRIVILEGED_OP	 0x02
#define PGM_EXECUTE              0x03
#define PGM_PROTECTION           0x04
#define PGM_ADDRESSING           0x05
#define PGM_SPECIFICATION        0x06
#define PGM_DATA                 0x07

struct kvm_s390_interrupt_info {
	struct list_head list;
	u64	type;
	union {
		struct kvm_s390_io_info io;
		struct kvm_s390_ext_info ext;
		struct kvm_s390_pgm_info pgm;
		struct kvm_s390_emerg_info emerg;
		struct kvm_s390_extcall_info extcall;
		struct kvm_s390_prefix_info prefix;
		struct kvm_s390_mchk_info mchk;
	};
};

/* for local_interrupt.action_flags */
#define ACTION_STORE_ON_STOP		(1<<0)
#define ACTION_STOP_ON_STOP		(1<<1)

struct kvm_s390_local_interrupt {
	spinlock_t lock;
	struct list_head list;
	atomic_t active;
	struct kvm_s390_float_interrupt *float_int;
	int timer_due; /* event indicator for waitqueue below */
	wait_queue_head_t *wq;
	atomic_t *cpuflags;
	unsigned int action_bits;
};

struct kvm_s390_float_interrupt {
	spinlock_t lock;
	struct list_head list;
	atomic_t active;
	int next_rr_cpu;
	unsigned long idle_mask[BITS_TO_LONGS(KVM_MAX_VCPUS)];
	unsigned int irq_count;
};


struct kvm_vcpu_arch {
	struct kvm_s390_sie_block *sie_block;
	s390_fp_regs      host_fpregs;
	unsigned int      host_acrs[NUM_ACRS];
	s390_fp_regs      guest_fpregs;
	struct kvm_s390_local_interrupt local_int;
	struct hrtimer    ckc_timer;
	struct tasklet_struct tasklet;
	union  {
		struct cpuid	cpu_id;
		u64		stidp_data;
	};
	struct gmap *gmap;
#define KVM_S390_PFAULT_TOKEN_INVALID	(-1UL)
	unsigned long pfault_token;
	unsigned long pfault_select;
	unsigned long pfault_compare;
};

struct kvm_vm_stat {
	u32 remote_tlb_flush;
};

struct kvm_arch_memory_slot {
};

struct s390_map_info {
	struct list_head list;
	__u64 guest_addr;
	__u64 addr;
	struct page *page;
};

struct s390_io_adapter {
	unsigned int id;
	int isc;
	bool maskable;
	bool masked;
	bool swap;
	struct rw_semaphore maps_lock;
	struct list_head maps;
	atomic_t nr_maps;
};

#define MAX_S390_IO_ADAPTERS ((MAX_ISC + 1) * 8)
#define MAX_S390_ADAPTER_MAPS 256

struct kvm_arch{
	struct sca_block *sca;
	debug_info_t *dbf;
	struct kvm_s390_float_interrupt float_int;
	struct kvm_device *flic;
	struct gmap *gmap;
	int css_support;
	int use_irqchip;
	struct s390_io_adapter *adapters[MAX_S390_IO_ADAPTERS];
};

#define KVM_HVA_ERR_BAD		(-1UL)
#define KVM_HVA_ERR_RO_BAD	(-2UL)

static inline bool kvm_is_error_hva(unsigned long addr)
{
	return IS_ERR_VALUE(addr);
}

#define ASYNC_PF_PER_VCPU	64
struct kvm_vcpu;
struct kvm_async_pf;
struct kvm_arch_async_pf {
	unsigned long pfault_token;
};

bool kvm_arch_can_inject_async_page_present(struct kvm_vcpu *vcpu);

void kvm_arch_async_page_ready(struct kvm_vcpu *vcpu,
			       struct kvm_async_pf *work);

void kvm_arch_async_page_not_present(struct kvm_vcpu *vcpu,
				     struct kvm_async_pf *work);

void kvm_arch_async_page_present(struct kvm_vcpu *vcpu,
				 struct kvm_async_pf *work);

extern int sie64a(struct kvm_s390_sie_block *, u64 *);
extern char sie_exit;
#endif
