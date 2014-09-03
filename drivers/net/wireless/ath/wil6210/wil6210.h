/*
 * Copyright (c) 2012 Qualcomm Atheros, Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef __WIL6210_H__
#define __WIL6210_H__

#include <linux/netdevice.h>
#include <linux/wireless.h>
#include <net/cfg80211.h>

#define WIL_NAME "wil6210"

/**
 * extract bits [@b0:@b1] (inclusive) from the value @x
 * it should be @b0 <= @b1, or result is incorrect
 */
static inline u32 WIL_GET_BITS(u32 x, int b0, int b1)
{
	return (x >> b0) & ((1 << (b1 - b0 + 1)) - 1);
}

#define WIL6210_MEM_SIZE (2*1024*1024UL)

#define WIL6210_RX_RING_SIZE	(128)
#define WIL6210_TX_RING_SIZE	(128)
#define WIL6210_MAX_TX_RINGS	(24) /* HW limit */
#define WIL6210_MAX_CID		(8) /* HW limit */
#define WIL6210_NAPI_BUDGET	(16) /* arbitrary */
#define WIL6210_ITR_TRSH	(10000) /* arbitrary - about 15 IRQs/msec */

/* Hardware definitions begin */

/*
 * Mapping
 * RGF File      | Host addr    |  FW addr
 *               |              |
 * user_rgf      | 0x000000     | 0x880000
 *  dma_rgf      | 0x001000     | 0x881000
 * pcie_rgf      | 0x002000     | 0x882000
 *               |              |
 */

/* Where various structures placed in host address space */
#define WIL6210_FW_HOST_OFF      (0x880000UL)

#define HOSTADDR(fwaddr)        (fwaddr - WIL6210_FW_HOST_OFF)

/*
 * Interrupt control registers block
 *
 * each interrupt controlled by the same bit in all registers
 */
struct RGF_ICR {
	u32 ICC; /* Cause Control, RW: 0 - W1C, 1 - COR */
	u32 ICR; /* Cause, W1C/COR depending on ICC */
	u32 ICM; /* Cause masked (ICR & ~IMV), W1C/COR depending on ICC */
	u32 ICS; /* Cause Set, WO */
	u32 IMV; /* Mask, RW+S/C */
	u32 IMS; /* Mask Set, write 1 to set */
	u32 IMC; /* Mask Clear, write 1 to clear */
} __packed;

/* registers - FW addresses */
#define RGF_USER_HW_MACHINE_STATE	(0x8801dc)
	#define HW_MACHINE_BOOT_DONE	(0x3fffffd)
#define RGF_USER_USER_CPU_0		(0x8801e0)
#define RGF_USER_MAC_CPU_0		(0x8801fc)
#define RGF_USER_USER_SCRATCH_PAD	(0x8802bc)
#define RGF_USER_FW_REV_ID		(0x880a8c) /* chip revision */
#define RGF_USER_CLKS_CTL_0		(0x880abc)
	#define BIT_USER_CLKS_RST_PWGD	BIT(11) /* reset on "power good" */
#define RGF_USER_CLKS_CTL_SW_RST_VEC_0	(0x880b04)
#define RGF_USER_CLKS_CTL_SW_RST_VEC_1	(0x880b08)
#define RGF_USER_CLKS_CTL_SW_RST_VEC_2	(0x880b0c)
#define RGF_USER_CLKS_CTL_SW_RST_VEC_3	(0x880b10)
#define RGF_USER_CLKS_CTL_SW_RST_MASK_0	(0x880b14)
#define RGF_USER_USER_ICR		(0x880b4c) /* struct RGF_ICR */
	#define BIT_USER_USER_ICR_SW_INT_2	BIT(18)

#define RGF_DMA_EP_TX_ICR		(0x881bb4) /* struct RGF_ICR */
	#define BIT_DMA_EP_TX_ICR_TX_DONE	BIT(0)
	#define BIT_DMA_EP_TX_ICR_TX_DONE_N(n)	BIT(n+1) /* n = [0..23] */
#define RGF_DMA_EP_RX_ICR		(0x881bd0) /* struct RGF_ICR */
	#define BIT_DMA_EP_RX_ICR_RX_DONE	BIT(0)
#define RGF_DMA_EP_MISC_ICR		(0x881bec) /* struct RGF_ICR */
	#define BIT_DMA_EP_MISC_ICR_RX_HTRSH	BIT(0)
	#define BIT_DMA_EP_MISC_ICR_TX_NO_ACT	BIT(1)
	#define BIT_DMA_EP_MISC_ICR_FW_INT(n)	BIT(28+n) /* n = [0..3] */

/* Interrupt moderation control */
#define RGF_DMA_ITR_CNT_TRSH		(0x881c5c)
#define RGF_DMA_ITR_CNT_DATA		(0x881c60)
#define RGF_DMA_ITR_CNT_CRL		(0x881c64)
	#define BIT_DMA_ITR_CNT_CRL_EN		BIT(0)
	#define BIT_DMA_ITR_CNT_CRL_EXT_TICK	BIT(1)
	#define BIT_DMA_ITR_CNT_CRL_FOREVER	BIT(2)
	#define BIT_DMA_ITR_CNT_CRL_CLR		BIT(3)
	#define BIT_DMA_ITR_CNT_CRL_REACH_TRSH	BIT(4)

#define RGF_DMA_PSEUDO_CAUSE		(0x881c68)
#define RGF_DMA_PSEUDO_CAUSE_MASK_SW	(0x881c6c)
#define RGF_DMA_PSEUDO_CAUSE_MASK_FW	(0x881c70)
	#define BIT_DMA_PSEUDO_CAUSE_RX		BIT(0)
	#define BIT_DMA_PSEUDO_CAUSE_TX		BIT(1)
	#define BIT_DMA_PSEUDO_CAUSE_MISC	BIT(2)

#define RGF_PCIE_LOS_COUNTER_CTL	(0x882dc4)

/* popular locations */
#define HOST_MBOX   HOSTADDR(RGF_USER_USER_SCRATCH_PAD)
#define HOST_SW_INT (HOSTADDR(RGF_USER_USER_ICR) + \
	offsetof(struct RGF_ICR, ICS))
#define SW_INT_MBOX BIT_USER_USER_ICR_SW_INT_2

/* ISR register bits */
#define ISR_MISC_FW_READY	BIT_DMA_EP_MISC_ICR_FW_INT(0)
#define ISR_MISC_MBOX_EVT	BIT_DMA_EP_MISC_ICR_FW_INT(1)
#define ISR_MISC_FW_ERROR	BIT_DMA_EP_MISC_ICR_FW_INT(3)

/* Hardware definitions end */

/**
 * mk_cidxtid - construct @cidxtid field
 * @cid: CID value
 * @tid: TID value
 *
 * @cidxtid field encoded as bits 0..3 - CID; 4..7 - TID
 */
static inline u8 mk_cidxtid(u8 cid, u8 tid)
{
	return ((tid & 0xf) << 4) | (cid & 0xf);
}

/**
 * parse_cidxtid - parse @cidxtid field
 * @cid: store CID value here
 * @tid: store TID value here
 *
 * @cidxtid field encoded as bits 0..3 - CID; 4..7 - TID
 */
static inline void parse_cidxtid(u8 cidxtid, u8 *cid, u8 *tid)
{
	*cid = cidxtid & 0xf;
	*tid = (cidxtid >> 4) & 0xf;
}

struct wil6210_mbox_ring {
	u32 base;
	u16 entry_size; /* max. size of mbox entry, incl. all headers */
	u16 size;
	u32 tail;
	u32 head;
} __packed;

struct wil6210_mbox_ring_desc {
	__le32 sync;
	__le32 addr;
} __packed;

/* at HOST_OFF_WIL6210_MBOX_CTL */
struct wil6210_mbox_ctl {
	struct wil6210_mbox_ring tx;
	struct wil6210_mbox_ring rx;
} __packed;

struct wil6210_mbox_hdr {
	__le16 seq;
	__le16 len; /* payload, bytes after this header */
	__le16 type;
	u8 flags;
	u8 reserved;
} __packed;

#define WIL_MBOX_HDR_TYPE_WMI (0)

/* max. value for wil6210_mbox_hdr.len */
#define MAX_MBOXITEM_SIZE   (240)

/**
 * struct wil6210_mbox_hdr_wmi - WMI header
 *
 * @mid: MAC ID
 *	00 - default, created by FW
 *	01..0f - WiFi ports, driver to create
 *	10..fe - debug
 *	ff - broadcast
 * @id: command/event ID
 * @timestamp: FW fills for events, free-running msec timer
 */
struct wil6210_mbox_hdr_wmi {
	u8 mid;
	u8 reserved;
	__le16 id;
	__le32 timestamp;
} __packed;

struct pending_wmi_event {
	struct list_head list;
	struct {
		struct wil6210_mbox_hdr hdr;
		struct wil6210_mbox_hdr_wmi wmi;
		u8 data[0];
	} __packed event;
};

enum { /* for wil_ctx.mapped_as */
	wil_mapped_as_none = 0,
	wil_mapped_as_single = 1,
	wil_mapped_as_page = 2,
};

/**
 * struct wil_ctx - software context for Vring descriptor
 */
struct wil_ctx {
	struct sk_buff *skb;
	u8 nr_frags;
	u8 mapped_as;
};

union vring_desc;

struct vring {
	dma_addr_t pa;
	volatile union vring_desc *va; /* vring_desc[size], WriteBack by DMA */
	u16 size; /* number of vring_desc elements */
	u32 swtail;
	u32 swhead;
	u32 hwtail; /* write here to inform hw */
	struct wil_ctx *ctx; /* ctx[size] - software context */
};

/**
 * Additional data for Tx Vring
 */
struct vring_tx_data {
	int enabled;

};

enum { /* for wil6210_priv.status */
	wil_status_fwready = 0,
	wil_status_fwconnecting,
	wil_status_fwconnected,
	wil_status_dontscan,
	wil_status_reset_done,
	wil_status_irqen, /* FIXME: interrupts enabled - for debug */
	wil_status_napi_en, /* NAPI enabled protected by wil->mutex */
};

struct pci_dev;

/**
 * struct tid_ampdu_rx - TID aggregation information (Rx).
 *
 * @reorder_buf: buffer to reorder incoming aggregated MPDUs
 * @reorder_time: jiffies when skb was added
 * @session_timer: check if peer keeps Tx-ing on the TID (by timeout value)
 * @reorder_timer: releases expired frames from the reorder buffer.
 * @last_rx: jiffies of last rx activity
 * @head_seq_num: head sequence number in reordering buffer.
 * @stored_mpdu_num: number of MPDUs in reordering buffer
 * @ssn: Starting Sequence Number expected to be aggregated.
 * @buf_size: buffer size for incoming A-MPDUs
 * @timeout: reset timer value (in TUs).
 * @dialog_token: dialog token for aggregation session
 * @rcu_head: RCU head used for freeing this struct
 * @reorder_lock: serializes access to reorder buffer, see below.
 *
 * This structure's lifetime is managed by RCU, assignments to
 * the array holding it must hold the aggregation mutex.
 *
 * The @reorder_lock is used to protect the members of this
 * struct, except for @timeout, @buf_size and @dialog_token,
 * which are constant across the lifetime of the struct (the
 * dialog token being used only for debugging).
 */
struct wil_tid_ampdu_rx {
	spinlock_t reorder_lock; /* see above */
	struct sk_buff **reorder_buf;
	unsigned long *reorder_time;
	struct timer_list session_timer;
	struct timer_list reorder_timer;
	unsigned long last_rx;
	u16 head_seq_num;
	u16 stored_mpdu_num;
	u16 ssn;
	u16 buf_size;
	u16 timeout;
	u8 dialog_token;
};

struct wil6210_stats {
	u64 tsf;
	u32 snr;
	u16 last_mcs_rx;
	u16 bf_mcs; /* last BF, used for Tx */
	u16 my_rx_sector;
	u16 my_tx_sector;
	u16 peer_rx_sector;
	u16 peer_tx_sector;
};

enum wil_sta_status {
	wil_sta_unused = 0,
	wil_sta_conn_pending = 1,
	wil_sta_connected = 2,
};

#define WIL_STA_TID_NUM (16)

struct wil_net_stats {
	unsigned long	rx_packets;
	unsigned long	tx_packets;
	unsigned long	rx_bytes;
	unsigned long	tx_bytes;
	unsigned long	tx_errors;
	unsigned long	rx_dropped;
	u16 last_mcs_rx;
};

/**
 * struct wil_sta_info - data for peer
 *
 * Peer identified by its CID (connection ID)
 * NIC performs beam forming for each peer;
 * if no beam forming done, frame exchange is not
 * possible.
 */
struct wil_sta_info {
	u8 addr[ETH_ALEN];
	enum wil_sta_status status;
	struct wil_net_stats stats;
	bool data_port_open; /* can send any data, not only EAPOL */
	/* Rx BACK */
	struct wil_tid_ampdu_rx *tid_rx[WIL_STA_TID_NUM];
	unsigned long tid_rx_timer_expired[BITS_TO_LONGS(WIL_STA_TID_NUM)];
	unsigned long tid_rx_stop_requested[BITS_TO_LONGS(WIL_STA_TID_NUM)];
};

struct wil6210_priv {
	struct pci_dev *pdev;
	int n_msi;
	struct wireless_dev *wdev;
	void __iomem *csr;
	ulong status;
	u32 fw_version;
	u32 hw_version;
	u8 n_mids; /* number of additional MIDs as reported by FW */
	/* profile */
	u32 monitor_flags;
	u32 secure_pcp; /* create secure PCP? */
	int sinfo_gen;
	/* cached ISR registers */
	u32 isr_misc;
	/* mailbox related */
	struct mutex wmi_mutex;
	struct wil6210_mbox_ctl mbox_ctl;
	struct completion wmi_ready;
	u16 wmi_seq;
	u16 reply_id; /**< wait for this WMI event */
	void *reply_buf;
	u16 reply_size;
	struct workqueue_struct *wmi_wq; /* for deferred calls */
	struct work_struct wmi_event_worker;
	struct workqueue_struct *wmi_wq_conn; /* for connect worker */
	struct work_struct connect_worker;
	struct work_struct disconnect_worker;
	struct work_struct fw_error_worker;	/* for FW error recovery */
	struct timer_list connect_timer;
	int pending_connect_cid;
	struct list_head pending_wmi_ev;
	/*
	 * protect pending_wmi_ev
	 * - fill in IRQ from wil6210_irq_misc,
	 * - consumed in thread by wmi_event_worker
	 */
	spinlock_t wmi_ev_lock;
	struct napi_struct napi_rx;
	struct napi_struct napi_tx;
	/* DMA related */
	struct vring vring_rx;
	struct vring vring_tx[WIL6210_MAX_TX_RINGS];
	struct vring_tx_data vring_tx_data[WIL6210_MAX_TX_RINGS];
	u8 vring2cid_tid[WIL6210_MAX_TX_RINGS][2]; /* [0] - CID, [1] - TID */
	struct wil_sta_info sta[WIL6210_MAX_CID];
	/* scan */
	struct cfg80211_scan_request *scan_request;

	struct mutex mutex; /* for wil6210_priv access in wil_{up|down} */
	/* statistics */
	struct wil6210_stats stats;
	/* debugfs */
	struct dentry *debug;
	struct debugfs_blob_wrapper fw_code_blob;
	struct debugfs_blob_wrapper fw_data_blob;
	struct debugfs_blob_wrapper fw_peri_blob;
	struct debugfs_blob_wrapper uc_code_blob;
	struct debugfs_blob_wrapper uc_data_blob;
	struct debugfs_blob_wrapper rgf_blob;
};

#define wil_to_wiphy(i) (i->wdev->wiphy)
#define wil_to_dev(i) (wiphy_dev(wil_to_wiphy(i)))
#define wiphy_to_wil(w) (struct wil6210_priv *)(wiphy_priv(w))
#define wil_to_wdev(i) (i->wdev)
#define wdev_to_wil(w) (struct wil6210_priv *)(wdev_priv(w))
#define wil_to_ndev(i) (wil_to_wdev(i)->netdev)
#define ndev_to_wil(n) (wdev_to_wil(n->ieee80211_ptr))

int wil_dbg_trace(struct wil6210_priv *wil, const char *fmt, ...);
int wil_err(struct wil6210_priv *wil, const char *fmt, ...);
int wil_info(struct wil6210_priv *wil, const char *fmt, ...);
#define wil_dbg(wil, fmt, arg...) do { \
	netdev_dbg(wil_to_ndev(wil), fmt, ##arg); \
	wil_dbg_trace(wil, fmt, ##arg); \
} while (0)

#define wil_dbg_irq(wil, fmt, arg...) wil_dbg(wil, "DBG[ IRQ]" fmt, ##arg)
#define wil_dbg_txrx(wil, fmt, arg...) wil_dbg(wil, "DBG[TXRX]" fmt, ##arg)
#define wil_dbg_wmi(wil, fmt, arg...) wil_dbg(wil, "DBG[ WMI]" fmt, ##arg)
#define wil_dbg_misc(wil, fmt, arg...) wil_dbg(wil, "DBG[MISC]" fmt, ##arg)

#define wil_hex_dump_txrx(prefix_str, prefix_type, rowsize,	\
			  groupsize, buf, len, ascii)		\
			  print_hex_dump_debug("DBG[TXRX]" prefix_str,\
					 prefix_type, rowsize,	\
					 groupsize, buf, len, ascii)

#define wil_hex_dump_wmi(prefix_str, prefix_type, rowsize,	\
			 groupsize, buf, len, ascii)		\
			 print_hex_dump_debug("DBG[ WMI]" prefix_str,\
					prefix_type, rowsize,	\
					groupsize, buf, len, ascii)

void wil_memcpy_fromio_32(void *dst, const volatile void __iomem *src,
			  size_t count);
void wil_memcpy_toio_32(volatile void __iomem *dst, const void *src,
			size_t count);

void *wil_if_alloc(struct device *dev, void __iomem *csr);
void wil_if_free(struct wil6210_priv *wil);
int wil_if_add(struct wil6210_priv *wil);
void wil_if_remove(struct wil6210_priv *wil);
int wil_priv_init(struct wil6210_priv *wil);
void wil_priv_deinit(struct wil6210_priv *wil);
int wil_reset(struct wil6210_priv *wil);
void wil_fw_error_recovery(struct wil6210_priv *wil);
void wil_link_on(struct wil6210_priv *wil);
void wil_link_off(struct wil6210_priv *wil);
int wil_up(struct wil6210_priv *wil);
int wil_down(struct wil6210_priv *wil);
void wil_mbox_ring_le2cpus(struct wil6210_mbox_ring *r);
int wil_find_cid(struct wil6210_priv *wil, const u8 *mac);

void __iomem *wmi_buffer(struct wil6210_priv *wil, __le32 ptr);
void __iomem *wmi_addr(struct wil6210_priv *wil, u32 ptr);
int wmi_read_hdr(struct wil6210_priv *wil, __le32 ptr,
		 struct wil6210_mbox_hdr *hdr);
int wmi_send(struct wil6210_priv *wil, u16 cmdid, void *buf, u16 len);
void wmi_recv_cmd(struct wil6210_priv *wil);
int wmi_call(struct wil6210_priv *wil, u16 cmdid, void *buf, u16 len,
	     u16 reply_id, void *reply, u8 reply_size, int to_msec);
void wmi_event_worker(struct work_struct *work);
void wmi_event_flush(struct wil6210_priv *wil);
int wmi_set_ssid(struct wil6210_priv *wil, u8 ssid_len, const void *ssid);
int wmi_get_ssid(struct wil6210_priv *wil, u8 *ssid_len, void *ssid);
int wmi_set_channel(struct wil6210_priv *wil, int channel);
int wmi_get_channel(struct wil6210_priv *wil, int *channel);
int wmi_del_cipher_key(struct wil6210_priv *wil, u8 key_index,
		       const void *mac_addr);
int wmi_add_cipher_key(struct wil6210_priv *wil, u8 key_index,
		       const void *mac_addr, int key_len, const void *key);
int wmi_echo(struct wil6210_priv *wil);
int wmi_set_ie(struct wil6210_priv *wil, u8 type, u16 ie_len, const void *ie);
int wmi_rx_chain_add(struct wil6210_priv *wil, struct vring *vring);
int wmi_p2p_cfg(struct wil6210_priv *wil, int channel);
int wmi_rxon(struct wil6210_priv *wil, bool on);
int wmi_get_temperature(struct wil6210_priv *wil, u32 *t_m, u32 *t_r);
int wmi_disconnect_sta(struct wil6210_priv *wil, const u8 *mac, u16 reason);

void wil6210_clear_irq(struct wil6210_priv *wil);
int wil6210_init_irq(struct wil6210_priv *wil, int irq);
void wil6210_fini_irq(struct wil6210_priv *wil, int irq);
void wil6210_disable_irq(struct wil6210_priv *wil);
void wil6210_enable_irq(struct wil6210_priv *wil);

int wil6210_debugfs_init(struct wil6210_priv *wil);
void wil6210_debugfs_remove(struct wil6210_priv *wil);

struct wireless_dev *wil_cfg80211_init(struct device *dev);
void wil_wdev_free(struct wil6210_priv *wil);

int wmi_set_mac_address(struct wil6210_priv *wil, void *addr);
int wmi_pcp_start(struct wil6210_priv *wil, int bi, u8 wmi_nettype, u8 chan);
int wmi_pcp_stop(struct wil6210_priv *wil);
void wil6210_disconnect(struct wil6210_priv *wil, void *bssid);

int wil_rx_init(struct wil6210_priv *wil);
void wil_rx_fini(struct wil6210_priv *wil);

/* TX API */
int wil_vring_init_tx(struct wil6210_priv *wil, int id, int size,
		      int cid, int tid);
void wil_vring_fini_tx(struct wil6210_priv *wil, int id);

netdev_tx_t wil_start_xmit(struct sk_buff *skb, struct net_device *ndev);
int wil_tx_complete(struct wil6210_priv *wil, int ringid);
void wil6210_unmask_irq_tx(struct wil6210_priv *wil);

/* RX API */
void wil_rx_handle(struct wil6210_priv *wil, int *quota);
void wil6210_unmask_irq_rx(struct wil6210_priv *wil);

int wil_iftype_nl2wmi(enum nl80211_iftype type);

#endif /* __WIL6210_H__ */
