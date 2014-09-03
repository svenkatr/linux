/*
 * Copyright (c) 2008-2011 Atheros Communications Inc.
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

#ifndef ATH9K_H
#define ATH9K_H

#include <linux/etherdevice.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/leds.h>
#include <linux/completion.h>

#include "debug.h"
#include "common.h"
#include "mci.h"
#include "dfs.h"
#include "spectral.h"

struct ath_node;

extern struct ieee80211_ops ath9k_ops;
extern int ath9k_modparam_nohwcrypt;
extern int led_blink;
extern bool is_ath9k_unloaded;

struct ath_config {
	u16 txpowlimit;
};

/*************************/
/* Descriptor Management */
/*************************/

#define ATH_TXSTATUS_RING_SIZE 512

/* Macro to expand scalars to 64-bit objects */
#define	ito64(x) (sizeof(x) == 1) ?			\
	(((unsigned long long int)(x)) & (0xff)) :	\
	(sizeof(x) == 2) ?				\
	(((unsigned long long int)(x)) & 0xffff) :	\
	((sizeof(x) == 4) ?				\
	 (((unsigned long long int)(x)) & 0xffffffff) : \
	 (unsigned long long int)(x))

#define ATH_TXBUF_RESET(_bf) do {				\
		(_bf)->bf_lastbf = NULL;			\
		(_bf)->bf_next = NULL;				\
		memset(&((_bf)->bf_state), 0,			\
		       sizeof(struct ath_buf_state));		\
	} while (0)

#define	DS2PHYS(_dd, _ds)						\
	((_dd)->dd_desc_paddr + ((caddr_t)(_ds) - (caddr_t)(_dd)->dd_desc))
#define ATH_DESC_4KB_BOUND_CHECK(_daddr) ((((_daddr) & 0xFFF) > 0xF7F) ? 1 : 0)
#define ATH_DESC_4KB_BOUND_NUM_SKIPPED(_len) ((_len) / 4096)

struct ath_descdma {
	void *dd_desc;
	dma_addr_t dd_desc_paddr;
	u32 dd_desc_len;
};

int ath_descdma_setup(struct ath_softc *sc, struct ath_descdma *dd,
		      struct list_head *head, const char *name,
		      int nbuf, int ndesc, bool is_tx);

/***********/
/* RX / TX */
/***********/

#define	ATH_TXQ_SETUP(sc, i) ((sc)->tx.txqsetup & (1<<i))

/* increment with wrap-around */
#define INCR(_l, _sz)   do {			\
		(_l)++;				\
		(_l) &= ((_sz) - 1);		\
	} while (0)

#define ATH_RXBUF               512
#define ATH_TXBUF               512
#define ATH_TXBUF_RESERVE       5
#define ATH_MAX_QDEPTH          (ATH_TXBUF / 4 - ATH_TXBUF_RESERVE)
#define ATH_TXMAXTRY            13
#define ATH_MAX_SW_RETRIES      30

#define TID_TO_WME_AC(_tid)				\
	((((_tid) == 0) || ((_tid) == 3)) ? IEEE80211_AC_BE :	\
	 (((_tid) == 1) || ((_tid) == 2)) ? IEEE80211_AC_BK :	\
	 (((_tid) == 4) || ((_tid) == 5)) ? IEEE80211_AC_VI :	\
	 IEEE80211_AC_VO)

#define ATH_AGGR_DELIM_SZ          4
#define ATH_AGGR_MINPLEN           256 /* in bytes, minimum packet length */
/* number of delimiters for encryption padding */
#define ATH_AGGR_ENCRYPTDELIM      10
/* minimum h/w qdepth to be sustained to maximize aggregation */
#define ATH_AGGR_MIN_QDEPTH        2
/* minimum h/w qdepth for non-aggregated traffic */
#define ATH_NON_AGGR_MIN_QDEPTH    8
#define ATH_TX_COMPLETE_POLL_INT   1000
#define ATH_TXFIFO_DEPTH           8
#define ATH_TX_ERROR               0x01

#define IEEE80211_SEQ_SEQ_SHIFT    4
#define IEEE80211_SEQ_MAX          4096
#define IEEE80211_WEP_IVLEN        3
#define IEEE80211_WEP_KIDLEN       1
#define IEEE80211_WEP_CRCLEN       4
#define IEEE80211_MAX_MPDU_LEN     (3840 + FCS_LEN +		\
				    (IEEE80211_WEP_IVLEN +	\
				     IEEE80211_WEP_KIDLEN +	\
				     IEEE80211_WEP_CRCLEN))

/* return whether a bit at index _n in bitmap _bm is set
 * _sz is the size of the bitmap  */
#define ATH_BA_ISSET(_bm, _n)  (((_n) < (WME_BA_BMP_SIZE)) &&		\
				((_bm)[(_n) >> 5] & (1 << ((_n) & 31))))

/* return block-ack bitmap index given sequence and starting sequence */
#define ATH_BA_INDEX(_st, _seq) (((_seq) - (_st)) & (IEEE80211_SEQ_MAX - 1))

/* return the seqno for _start + _offset */
#define ATH_BA_INDEX2SEQ(_seq, _offset) (((_seq) + (_offset)) & (IEEE80211_SEQ_MAX - 1))

/* returns delimiter padding required given the packet length */
#define ATH_AGGR_GET_NDELIM(_len)					\
       (((_len) >= ATH_AGGR_MINPLEN) ? 0 :                             \
        DIV_ROUND_UP(ATH_AGGR_MINPLEN - (_len), ATH_AGGR_DELIM_SZ))

#define BAW_WITHIN(_start, _bawsz, _seqno) \
	((((_seqno) - (_start)) & 4095) < (_bawsz))

#define ATH_AN_2_TID(_an, _tidno)  (&(_an)->tid[(_tidno)])

#define IS_HT_RATE(rate)   (rate & 0x80)
#define IS_CCK_RATE(rate)  ((rate >= 0x18) && (rate <= 0x1e))
#define IS_OFDM_RATE(rate) ((rate >= 0x8) && (rate <= 0xf))

enum {
       WLAN_RC_PHY_OFDM,
       WLAN_RC_PHY_CCK,
};

struct ath_txq {
	int mac80211_qnum; /* mac80211 queue number, -1 means not mac80211 Q */
	u32 axq_qnum; /* ath9k hardware queue number */
	void *axq_link;
	struct list_head axq_q;
	spinlock_t axq_lock;
	u32 axq_depth;
	u32 axq_ampdu_depth;
	bool stopped;
	bool axq_tx_inprogress;
	struct list_head axq_acq;
	struct list_head txq_fifo[ATH_TXFIFO_DEPTH];
	u8 txq_headidx;
	u8 txq_tailidx;
	int pending_frames;
	struct sk_buff_head complete_q;
};

struct ath_atx_ac {
	struct ath_txq *txq;
	struct list_head list;
	struct list_head tid_q;
	bool clear_ps_filter;
	bool sched;
};

struct ath_frame_info {
	struct ath_buf *bf;
	int framelen;
	enum ath9k_key_type keytype;
	u8 keyix;
	u8 rtscts_rate;
	u8 retries : 7;
	u8 baw_tracked : 1;
};

struct ath_rxbuf {
	struct list_head list;
	struct sk_buff *bf_mpdu;
	void *bf_desc;
	dma_addr_t bf_daddr;
	dma_addr_t bf_buf_addr;
};

/**
 * enum buffer_type - Buffer type flags
 *
 * @BUF_AMPDU: This buffer is an ampdu, as part of an aggregate (during TX)
 * @BUF_AGGR: Indicates whether the buffer can be aggregated
 *	(used in aggregation scheduling)
 */
enum buffer_type {
	BUF_AMPDU		= BIT(0),
	BUF_AGGR		= BIT(1),
};

#define bf_isampdu(bf)		(bf->bf_state.bf_type & BUF_AMPDU)
#define bf_isaggr(bf)		(bf->bf_state.bf_type & BUF_AGGR)

struct ath_buf_state {
	u8 bf_type;
	u8 bfs_paprd;
	u8 ndelim;
	bool stale;
	u16 seqno;
	unsigned long bfs_paprd_timestamp;
};

struct ath_buf {
	struct list_head list;
	struct ath_buf *bf_lastbf;	/* last buf of this unit (a frame or
					   an aggregate) */
	struct ath_buf *bf_next;	/* next subframe in the aggregate */
	struct sk_buff *bf_mpdu;	/* enclosing frame structure */
	void *bf_desc;			/* virtual addr of desc */
	dma_addr_t bf_daddr;		/* physical addr of desc */
	dma_addr_t bf_buf_addr;	/* physical addr of data buffer, for DMA */
	struct ieee80211_tx_rate rates[4];
	struct ath_buf_state bf_state;
};

struct ath_atx_tid {
	struct list_head list;
	struct sk_buff_head buf_q;
	struct sk_buff_head retry_q;
	struct ath_node *an;
	struct ath_atx_ac *ac;
	unsigned long tx_buf[BITS_TO_LONGS(ATH_TID_MAX_BUFS)];
	u16 seq_start;
	u16 seq_next;
	u16 baw_size;
	u8 tidno;
	int baw_head;   /* first un-acked tx buffer */
	int baw_tail;   /* next unused tx buffer slot */

	s8 bar_index;
	bool sched;
	bool active;
};

struct ath_node {
	struct ath_softc *sc;
	struct ieee80211_sta *sta; /* station struct we're part of */
	struct ieee80211_vif *vif; /* interface with which we're associated */
	struct ath_atx_tid tid[IEEE80211_NUM_TIDS];
	struct ath_atx_ac ac[IEEE80211_NUM_ACS];

	u16 maxampdu;
	u8 mpdudensity;
	s8 ps_key;

	bool sleeping;
	bool no_ps_filter;

#ifdef CONFIG_ATH9K_STATION_STATISTICS
	struct ath_rx_rate_stats rx_rate_stats;
#endif
};

struct ath_tx_control {
	struct ath_txq *txq;
	struct ath_node *an;
	u8 paprd;
	struct ieee80211_sta *sta;
};


/**
 * @txq_map:  Index is mac80211 queue number.  This is
 *  not necessarily the same as the hardware queue number
 *  (axq_qnum).
 */
struct ath_tx {
	u16 seq_no;
	u32 txqsetup;
	spinlock_t txbuflock;
	struct list_head txbuf;
	struct ath_txq txq[ATH9K_NUM_TX_QUEUES];
	struct ath_descdma txdma;
	struct ath_txq *txq_map[IEEE80211_NUM_ACS];
	struct ath_txq *uapsdq;
	u32 txq_max_pending[IEEE80211_NUM_ACS];
	u16 max_aggr_framelen[IEEE80211_NUM_ACS][4][32];
};

struct ath_rx_edma {
	struct sk_buff_head rx_fifo;
	u32 rx_fifo_hwsize;
};

struct ath_rx {
	u8 defant;
	u8 rxotherant;
	bool discard_next;
	u32 *rxlink;
	u32 num_pkts;
	unsigned int rxfilter;
	struct list_head rxbuf;
	struct ath_descdma rxdma;
	struct ath_rx_edma rx_edma[ATH9K_RX_QUEUE_MAX];

	struct ath_rxbuf *buf_hold;
	struct sk_buff *frag;

	u32 ampdu_ref;
};

int ath_startrecv(struct ath_softc *sc);
bool ath_stoprecv(struct ath_softc *sc);
u32 ath_calcrxfilter(struct ath_softc *sc);
int ath_rx_init(struct ath_softc *sc, int nbufs);
void ath_rx_cleanup(struct ath_softc *sc);
int ath_rx_tasklet(struct ath_softc *sc, int flush, bool hp);
struct ath_txq *ath_txq_setup(struct ath_softc *sc, int qtype, int subtype);
void ath_txq_lock(struct ath_softc *sc, struct ath_txq *txq);
void ath_txq_unlock(struct ath_softc *sc, struct ath_txq *txq);
void ath_txq_unlock_complete(struct ath_softc *sc, struct ath_txq *txq);
void ath_tx_cleanupq(struct ath_softc *sc, struct ath_txq *txq);
bool ath_drain_all_txq(struct ath_softc *sc);
void ath_draintxq(struct ath_softc *sc, struct ath_txq *txq);
void ath_tx_node_init(struct ath_softc *sc, struct ath_node *an);
void ath_tx_node_cleanup(struct ath_softc *sc, struct ath_node *an);
void ath_txq_schedule(struct ath_softc *sc, struct ath_txq *txq);
int ath_tx_init(struct ath_softc *sc, int nbufs);
int ath_txq_update(struct ath_softc *sc, int qnum,
		   struct ath9k_tx_queue_info *q);
void ath_update_max_aggr_framelen(struct ath_softc *sc, int queue, int txop);
int ath_tx_start(struct ieee80211_hw *hw, struct sk_buff *skb,
		 struct ath_tx_control *txctl);
void ath_tx_cabq(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		 struct sk_buff *skb);
void ath_tx_tasklet(struct ath_softc *sc);
void ath_tx_edma_tasklet(struct ath_softc *sc);
int ath_tx_aggr_start(struct ath_softc *sc, struct ieee80211_sta *sta,
		      u16 tid, u16 *ssn);
void ath_tx_aggr_stop(struct ath_softc *sc, struct ieee80211_sta *sta, u16 tid);
void ath_tx_aggr_resume(struct ath_softc *sc, struct ieee80211_sta *sta, u16 tid);

void ath_tx_aggr_wakeup(struct ath_softc *sc, struct ath_node *an);
void ath_tx_aggr_sleep(struct ieee80211_sta *sta, struct ath_softc *sc,
		       struct ath_node *an);
void ath9k_release_buffered_frames(struct ieee80211_hw *hw,
				   struct ieee80211_sta *sta,
				   u16 tids, int nframes,
				   enum ieee80211_frame_release_type reason,
				   bool more_data);

/********/
/* VIFs */
/********/

struct ath_vif {
	struct ath_node mcast_node;
	int av_bslot;
	bool primary_sta_vif;
	__le64 tsf_adjust; /* TSF adjustment for staggered beacons */
	struct ath_buf *av_bcbuf;
};

struct ath9k_vif_iter_data {
	u8 hw_macaddr[ETH_ALEN]; /* address of the first vif */
	u8 mask[ETH_ALEN]; /* bssid mask */
	bool has_hw_macaddr;

	int naps;      /* number of AP vifs */
	int nmeshes;   /* number of mesh vifs */
	int nstations; /* number of station vifs */
	int nwds;      /* number of WDS vifs */
	int nadhocs;   /* number of adhoc vifs */
};

void ath9k_calculate_iter_data(struct ieee80211_hw *hw,
			       struct ieee80211_vif *vif,
			       struct ath9k_vif_iter_data *iter_data);

/*******************/
/* Beacon Handling */
/*******************/

/*
 * Regardless of the number of beacons we stagger, (i.e. regardless of the
 * number of BSSIDs) if a given beacon does not go out even after waiting this
 * number of beacon intervals, the game's up.
 */
#define BSTUCK_THRESH           	9
#define	ATH_BCBUF               	8
#define ATH_DEFAULT_BINTVAL     	100 /* TU */
#define ATH_DEFAULT_BMISS_LIMIT 	10

#define TSF_TO_TU(_h,_l) \
	((((u32)(_h)) << 22) | (((u32)(_l)) >> 10))

struct ath_beacon {
	enum {
		OK,		/* no change needed */
		UPDATE,		/* update pending */
		COMMIT		/* beacon sent, commit change */
	} updateslot;		/* slot time update fsm */

	u32 beaconq;
	u32 bmisscnt;
	struct ieee80211_vif *bslot[ATH_BCBUF];
	int slottime;
	int slotupdate;
	struct ath_descdma bdma;
	struct ath_txq *cabq;
	struct list_head bbuf;

	bool tx_processed;
	bool tx_last;
};

void ath9k_beacon_tasklet(unsigned long data);
void ath9k_beacon_config(struct ath_softc *sc, struct ieee80211_vif *vif,
			 u32 changed);
void ath9k_beacon_assign_slot(struct ath_softc *sc, struct ieee80211_vif *vif);
void ath9k_beacon_remove_slot(struct ath_softc *sc, struct ieee80211_vif *vif);
void ath9k_set_beacon(struct ath_softc *sc);
bool ath9k_csa_is_finished(struct ath_softc *sc, struct ieee80211_vif *vif);
void ath9k_csa_update(struct ath_softc *sc);

/*******************/
/* Link Monitoring */
/*******************/

#define ATH_STA_SHORT_CALINTERVAL 1000    /* 1 second */
#define ATH_AP_SHORT_CALINTERVAL  100     /* 100 ms */
#define ATH_ANI_POLLINTERVAL_OLD  100     /* 100 ms */
#define ATH_ANI_POLLINTERVAL_NEW  1000    /* 1000 ms */
#define ATH_LONG_CALINTERVAL_INT  1000    /* 1000 ms */
#define ATH_LONG_CALINTERVAL      30000   /* 30 seconds */
#define ATH_RESTART_CALINTERVAL   1200000 /* 20 minutes */
#define ATH_ANI_MAX_SKIP_COUNT    10
#define ATH_PAPRD_TIMEOUT         100 /* msecs */
#define ATH_PLL_WORK_INTERVAL     100

void ath_tx_complete_poll_work(struct work_struct *work);
void ath_reset_work(struct work_struct *work);
bool ath_hw_check(struct ath_softc *sc);
void ath_hw_pll_work(struct work_struct *work);
void ath_paprd_calibrate(struct work_struct *work);
void ath_ani_calibrate(unsigned long data);
void ath_start_ani(struct ath_softc *sc);
void ath_stop_ani(struct ath_softc *sc);
void ath_check_ani(struct ath_softc *sc);
int ath_update_survey_stats(struct ath_softc *sc);
void ath_update_survey_nf(struct ath_softc *sc, int channel);
void ath9k_queue_reset(struct ath_softc *sc, enum ath_reset_type type);
void ath_ps_full_sleep(unsigned long data);

/**********/
/* BTCOEX */
/**********/

#define ATH_DUMP_BTCOEX(_s, _val)				\
	do {							\
		len += scnprintf(buf + len, size - len,		\
				 "%20s : %10d\n", _s, (_val));	\
	} while (0)

enum bt_op_flags {
	BT_OP_PRIORITY_DETECTED,
	BT_OP_SCAN,
};

struct ath_btcoex {
	spinlock_t btcoex_lock;
	struct timer_list period_timer; /* Timer for BT period */
	struct timer_list no_stomp_timer;
	u32 bt_priority_cnt;
	unsigned long bt_priority_time;
	unsigned long op_flags;
	int bt_stomp_type; /* Types of BT stomping */
	u32 btcoex_no_stomp; /* in msec */
	u32 btcoex_period; /* in msec */
	u32 btscan_no_stomp; /* in msec */
	u32 duty_cycle;
	u32 bt_wait_time;
	int rssi_count;
	struct ath_mci_profile mci;
	u8 stomp_audio;
};

#ifdef CONFIG_ATH9K_BTCOEX_SUPPORT
int ath9k_init_btcoex(struct ath_softc *sc);
void ath9k_deinit_btcoex(struct ath_softc *sc);
void ath9k_start_btcoex(struct ath_softc *sc);
void ath9k_stop_btcoex(struct ath_softc *sc);
void ath9k_btcoex_timer_resume(struct ath_softc *sc);
void ath9k_btcoex_timer_pause(struct ath_softc *sc);
void ath9k_btcoex_handle_interrupt(struct ath_softc *sc, u32 status);
u16 ath9k_btcoex_aggr_limit(struct ath_softc *sc, u32 max_4ms_framelen);
void ath9k_btcoex_stop_gen_timer(struct ath_softc *sc);
int ath9k_dump_btcoex(struct ath_softc *sc, u8 *buf, u32 size);
#else
static inline int ath9k_init_btcoex(struct ath_softc *sc)
{
	return 0;
}
static inline void ath9k_deinit_btcoex(struct ath_softc *sc)
{
}
static inline void ath9k_start_btcoex(struct ath_softc *sc)
{
}
static inline void ath9k_stop_btcoex(struct ath_softc *sc)
{
}
static inline void ath9k_btcoex_handle_interrupt(struct ath_softc *sc,
						 u32 status)
{
}
static inline u16 ath9k_btcoex_aggr_limit(struct ath_softc *sc,
					  u32 max_4ms_framelen)
{
	return 0;
}
static inline void ath9k_btcoex_stop_gen_timer(struct ath_softc *sc)
{
}
static inline int ath9k_dump_btcoex(struct ath_softc *sc, u8 *buf, u32 size)
{
	return 0;
}
#endif /* CONFIG_ATH9K_BTCOEX_SUPPORT */

/********************/
/*   LED Control    */
/********************/

#define ATH_LED_PIN_DEF 		1
#define ATH_LED_PIN_9287		8
#define ATH_LED_PIN_9300		10
#define ATH_LED_PIN_9485		6
#define ATH_LED_PIN_9462		4

#ifdef CONFIG_MAC80211_LEDS
void ath_init_leds(struct ath_softc *sc);
void ath_deinit_leds(struct ath_softc *sc);
void ath_fill_led_pin(struct ath_softc *sc);
#else
static inline void ath_init_leds(struct ath_softc *sc)
{
}

static inline void ath_deinit_leds(struct ath_softc *sc)
{
}
static inline void ath_fill_led_pin(struct ath_softc *sc)
{
}
#endif

/************************/
/* Wake on Wireless LAN */
/************************/

struct ath9k_wow_pattern {
	u8 pattern_bytes[MAX_PATTERN_SIZE];
	u8 mask_bytes[MAX_PATTERN_SIZE];
	u32 pattern_len;
};

#ifdef CONFIG_ATH9K_WOW
void ath9k_init_wow(struct ieee80211_hw *hw);
int ath9k_suspend(struct ieee80211_hw *hw,
		  struct cfg80211_wowlan *wowlan);
int ath9k_resume(struct ieee80211_hw *hw);
void ath9k_set_wakeup(struct ieee80211_hw *hw, bool enabled);
#else
static inline void ath9k_init_wow(struct ieee80211_hw *hw)
{
}
static inline int ath9k_suspend(struct ieee80211_hw *hw,
				struct cfg80211_wowlan *wowlan)
{
	return 0;
}
static inline int ath9k_resume(struct ieee80211_hw *hw)
{
	return 0;
}
static inline void ath9k_set_wakeup(struct ieee80211_hw *hw, bool enabled)
{
}
#endif /* CONFIG_ATH9K_WOW */

/*******************************/
/* Antenna diversity/combining */
/*******************************/

#define ATH_ANT_RX_CURRENT_SHIFT 4
#define ATH_ANT_RX_MAIN_SHIFT 2
#define ATH_ANT_RX_MASK 0x3

#define ATH_ANT_DIV_COMB_SHORT_SCAN_INTR 50
#define ATH_ANT_DIV_COMB_SHORT_SCAN_PKTCOUNT 0x100
#define ATH_ANT_DIV_COMB_MAX_PKTCOUNT 0x200
#define ATH_ANT_DIV_COMB_INIT_COUNT 95
#define ATH_ANT_DIV_COMB_MAX_COUNT 100
#define ATH_ANT_DIV_COMB_ALT_ANT_RATIO 30
#define ATH_ANT_DIV_COMB_ALT_ANT_RATIO2 20
#define ATH_ANT_DIV_COMB_ALT_ANT_RATIO_LOW_RSSI 50
#define ATH_ANT_DIV_COMB_ALT_ANT_RATIO2_LOW_RSSI 50

#define ATH_ANT_DIV_COMB_LNA1_DELTA_HI -4
#define ATH_ANT_DIV_COMB_LNA1_DELTA_MID -2
#define ATH_ANT_DIV_COMB_LNA1_DELTA_LOW 2

struct ath_ant_comb {
	u16 count;
	u16 total_pkt_count;
	bool scan;
	bool scan_not_start;
	int main_total_rssi;
	int alt_total_rssi;
	int alt_recv_cnt;
	int main_recv_cnt;
	int rssi_lna1;
	int rssi_lna2;
	int rssi_add;
	int rssi_sub;
	int rssi_first;
	int rssi_second;
	int rssi_third;
	int ant_ratio;
	int ant_ratio2;
	bool alt_good;
	int quick_scan_cnt;
	enum ath9k_ant_div_comb_lna_conf main_conf;
	enum ath9k_ant_div_comb_lna_conf first_quick_scan_conf;
	enum ath9k_ant_div_comb_lna_conf second_quick_scan_conf;
	bool first_ratio;
	bool second_ratio;
	unsigned long scan_start_time;

	/*
	 * Card-specific config values.
	 */
	int low_rssi_thresh;
	int fast_div_bias;
};

void ath_ant_comb_scan(struct ath_softc *sc, struct ath_rx_status *rs);

/********************/
/* Main driver core */
/********************/

#define ATH9K_PCI_CUS198          0x0001
#define ATH9K_PCI_CUS230          0x0002
#define ATH9K_PCI_CUS217          0x0004
#define ATH9K_PCI_CUS252          0x0008
#define ATH9K_PCI_WOW             0x0010
#define ATH9K_PCI_BT_ANT_DIV      0x0020
#define ATH9K_PCI_D3_L1_WAR       0x0040
#define ATH9K_PCI_AR9565_1ANT     0x0080
#define ATH9K_PCI_AR9565_2ANT     0x0100
#define ATH9K_PCI_NO_PLL_PWRSAVE  0x0200
#define ATH9K_PCI_KILLER          0x0400

/*
 * Default cache line size, in bytes.
 * Used when PCI device not fully initialized by bootrom/BIOS
*/
#define DEFAULT_CACHELINE       32
#define ATH_CABQ_READY_TIME     80      /* % of beacon interval */
#define ATH_TXPOWER_MAX         100     /* .5 dBm units */
#define MAX_GTT_CNT             5

/* Powersave flags */
#define PS_WAIT_FOR_BEACON        BIT(0)
#define PS_WAIT_FOR_CAB           BIT(1)
#define PS_WAIT_FOR_PSPOLL_DATA   BIT(2)
#define PS_WAIT_FOR_TX_ACK        BIT(3)
#define PS_BEACON_SYNC            BIT(4)
#define PS_WAIT_FOR_ANI           BIT(5)

struct ath_softc {
	struct ieee80211_hw *hw;
	struct device *dev;

	struct survey_info *cur_survey;
	struct survey_info survey[ATH9K_NUM_CHANNELS];

	struct tasklet_struct intr_tq;
	struct tasklet_struct bcon_tasklet;
	struct ath_hw *sc_ah;
	void __iomem *mem;
	int irq;
	spinlock_t sc_serial_rw;
	spinlock_t sc_pm_lock;
	spinlock_t sc_pcu_lock;
	struct mutex mutex;
	struct work_struct paprd_work;
	struct work_struct hw_reset_work;
	struct completion paprd_complete;
	wait_queue_head_t tx_wait;

	unsigned long driver_data;

	u8 gtt_cnt;
	u32 intrstatus;
	u16 ps_flags; /* PS_* */
	u16 curtxpow;
	bool ps_enabled;
	bool ps_idle;
	short nbcnvifs;
	short nvifs;
	unsigned long ps_usecount;

	struct ath_config config;
	struct ath_rx rx;
	struct ath_tx tx;
	struct ath_beacon beacon;

#ifdef CONFIG_MAC80211_LEDS
	bool led_registered;
	char led_name[32];
	struct led_classdev led_cdev;
#endif

	struct ath9k_hw_cal_data caldata;

#ifdef CONFIG_ATH9K_DEBUGFS
	struct ath9k_debug debug;
#endif
	struct ath_beacon_config cur_beacon_conf;
	struct delayed_work tx_complete_work;
	struct delayed_work hw_pll_work;
	struct timer_list sleep_timer;

#ifdef CONFIG_ATH9K_BTCOEX_SUPPORT
	struct ath_btcoex btcoex;
	struct ath_mci_coex mci_coex;
	struct work_struct mci_work;
#endif

	struct ath_descdma txsdma;

	struct ath_ant_comb ant_comb;
	u8 ant_tx, ant_rx;
	struct dfs_pattern_detector *dfs_detector;
	u32 wow_enabled;
	/* relay(fs) channel for spectral scan */
	struct rchan *rfs_chan_spec_scan;
	enum spectral_mode spectral_mode;
	struct ath_spec_scan spec_config;

	struct ieee80211_vif *tx99_vif;
	struct sk_buff *tx99_skb;
	bool tx99_state;
	s16 tx99_power;

#ifdef CONFIG_ATH9K_WOW
	atomic_t wow_got_bmiss_intr;
	atomic_t wow_sleep_proc_intr; /* in the middle of WoW sleep ? */
	u32 wow_intr_before_sleep;
#endif
};

/********/
/* TX99 */
/********/

#ifdef CONFIG_ATH9K_TX99
void ath9k_tx99_init_debug(struct ath_softc *sc);
int ath9k_tx99_send(struct ath_softc *sc, struct sk_buff *skb,
		    struct ath_tx_control *txctl);
#else
static inline void ath9k_tx99_init_debug(struct ath_softc *sc)
{
}
static inline int ath9k_tx99_send(struct ath_softc *sc,
				  struct sk_buff *skb,
				  struct ath_tx_control *txctl)
{
	return 0;
}
#endif /* CONFIG_ATH9K_TX99 */

static inline void ath_read_cachesize(struct ath_common *common, int *csz)
{
	common->bus_ops->read_cachesize(common, csz);
}

void ath9k_tasklet(unsigned long data);
int ath_cabq_update(struct ath_softc *);
u8 ath9k_parse_mpdudensity(u8 mpdudensity);
irqreturn_t ath_isr(int irq, void *dev);
int ath_reset(struct ath_softc *sc);
void ath_cancel_work(struct ath_softc *sc);
void ath_restart_work(struct ath_softc *sc);
int ath9k_init_device(u16 devid, struct ath_softc *sc,
		    const struct ath_bus_ops *bus_ops);
void ath9k_deinit_device(struct ath_softc *sc);
void ath9k_reload_chainmask_settings(struct ath_softc *sc);
u8 ath_txchainmask_reduction(struct ath_softc *sc, u8 chainmask, u32 rate);
void ath_start_rfkill_poll(struct ath_softc *sc);
void ath9k_rfkill_poll_state(struct ieee80211_hw *hw);
void ath9k_ps_wakeup(struct ath_softc *sc);
void ath9k_ps_restore(struct ath_softc *sc);

#ifdef CONFIG_ATH9K_PCI
int ath_pci_init(void);
void ath_pci_exit(void);
#else
static inline int ath_pci_init(void) { return 0; };
static inline void ath_pci_exit(void) {};
#endif

#ifdef CONFIG_ATH9K_AHB
int ath_ahb_init(void);
void ath_ahb_exit(void);
#else
static inline int ath_ahb_init(void) { return 0; };
static inline void ath_ahb_exit(void) {};
#endif

#endif /* ATH9K_H */
