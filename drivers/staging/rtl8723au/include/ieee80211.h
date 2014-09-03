/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 ******************************************************************************/
#ifndef __IEEE80211_H
#define __IEEE80211_H

#include <osdep_service.h>
#include <drv_types.h>
#include "linux/ieee80211.h"
#include "wifi.h"

#include <linux/wireless.h>

#if (WIRELESS_EXT < 22)
#error "Obsolete pre 2007 wireless extensions are not supported"
#endif


#define MGMT_QUEUE_NUM 5

#ifdef CONFIG_8723AU_AP_MODE

/* STA flags */
#define WLAN_STA_AUTH BIT(0)
#define WLAN_STA_ASSOC BIT(1)
#define WLAN_STA_PS BIT(2)
#define WLAN_STA_TIM BIT(3)
#define WLAN_STA_PERM BIT(4)
#define WLAN_STA_AUTHORIZED BIT(5)
#define WLAN_STA_PENDING_POLL BIT(6) /* pending activity poll not ACKed */
#define WLAN_STA_SHORT_PREAMBLE BIT(7)
#define WLAN_STA_PREAUTH BIT(8)
#define WLAN_STA_WME BIT(9)
#define WLAN_STA_MFP BIT(10)
#define WLAN_STA_HT BIT(11)
#define WLAN_STA_WPS BIT(12)
#define WLAN_STA_MAYBE_WPS BIT(13)
#define WLAN_STA_NONERP BIT(31)

#endif

#define IEEE_CMD_SET_WPA_PARAM			1
#define IEEE_CMD_SET_WPA_IE				2
#define IEEE_CMD_SET_ENCRYPTION			3

#define	IEEE_CRYPT_ALG_NAME_LEN			16

#define WPA_CIPHER_NONE		BIT(0)
#define WPA_CIPHER_WEP40	BIT(1)
#define WPA_CIPHER_WEP104 BIT(2)
#define WPA_CIPHER_TKIP		BIT(3)
#define WPA_CIPHER_CCMP		BIT(4)



#define WPA_SELECTOR_LEN 4
extern u8 RTW_WPA_OUI23A_TYPE[] ;
extern u16 RTW_WPA_VERSION23A ;
extern u8 WPA_AUTH_KEY_MGMT_NONE23A[];
extern u8 WPA_AUTH_KEY_MGMT_UNSPEC_802_1X23A[];
extern u8 WPA_AUTH_KEY_MGMT_PSK_OVER_802_1X23A[];
extern u8 WPA_CIPHER_SUITE_NONE23A[];
extern u8 WPA_CIPHER_SUITE_WEP4023A[];
extern u8 WPA_CIPHER_SUITE_TKIP23A[];
extern u8 WPA_CIPHER_SUITE_WRAP23A[];
extern u8 WPA_CIPHER_SUITE_CCMP23A[];
extern u8 WPA_CIPHER_SUITE_WEP10423A[];


#define RSN_HEADER_LEN 4
#define RSN_SELECTOR_LEN 4

extern u16 RSN_VERSION_BSD23A;
extern u8 RSN_AUTH_KEY_MGMT_UNSPEC_802_1X23A[];
extern u8 RSN_AUTH_KEY_MGMT_PSK_OVER_802_1X23A[];
extern u8 RSN_CIPHER_SUITE_NONE23A[];
extern u8 RSN_CIPHER_SUITE_WEP4023A[];
extern u8 RSN_CIPHER_SUITE_TKIP23A[];
extern u8 RSN_CIPHER_SUITE_WRAP23A[];
extern u8 RSN_CIPHER_SUITE_CCMP23A[];
extern u8 RSN_CIPHER_SUITE_WEP10423A[];

enum ratr_table_mode {
	RATR_INX_WIRELESS_NGB = 0,	/*  BGN 40 Mhz 2SS 1SS */
	RATR_INX_WIRELESS_NG = 1,	/*  GN or N */
	RATR_INX_WIRELESS_NB = 2,	/*  BGN 20 Mhz 2SS 1SS  or BN */
	RATR_INX_WIRELESS_N = 3,
	RATR_INX_WIRELESS_GB = 4,
	RATR_INX_WIRELESS_G = 5,
	RATR_INX_WIRELESS_B = 6,
	RATR_INX_WIRELESS_MC = 7,
	RATR_INX_WIRELESS_AC_N = 8,
};

enum NETWORK_TYPE
{
    WIRELESS_INVALID = 0,
    /* Sub-Element */
    WIRELESS_11B = BIT(0), /*  tx: cck only , rx: cck only, hw: cck */
    WIRELESS_11G = BIT(1), /*  tx: ofdm only, rx: ofdm & cck, hw: cck & ofdm */
    WIRELESS_11A = BIT(2), /*  tx: ofdm only, rx: ofdm only, hw: ofdm only */
    WIRELESS_11_24N = BIT(3), /*  tx: MCS only, rx: MCS & cck, hw: MCS & cck */
    WIRELESS_11_5N = BIT(4), /*  tx: MCS only, rx: MCS & ofdm, hw: ofdm only */
	/* WIRELESS_AUTO		= BIT(5), */
	WIRELESS_AC		= BIT(6),

    /* Combination */
    WIRELESS_11BG = (WIRELESS_11B|WIRELESS_11G), /*  tx: cck & ofdm, rx: cck & ofdm & MCS, hw: cck & ofdm */
    WIRELESS_11G_24N = (WIRELESS_11G|WIRELESS_11_24N), /*  tx: ofdm & MCS, rx: ofdm & cck & MCS, hw: cck & ofdm */
    WIRELESS_11A_5N = (WIRELESS_11A|WIRELESS_11_5N), /*  tx: ofdm & MCS, rx: ofdm & MCS, hw: ofdm only */
    WIRELESS_11BG_24N = (WIRELESS_11B|WIRELESS_11G|WIRELESS_11_24N), /*  tx: ofdm & cck & MCS, rx: ofdm & cck & MCS, hw: ofdm & cck */
    WIRELESS_11AGN = (WIRELESS_11A|WIRELESS_11G|WIRELESS_11_24N|WIRELESS_11_5N), /*  tx: ofdm & MCS, rx: ofdm & MCS, hw: ofdm only */
    WIRELESS_11ABGN = (WIRELESS_11A|WIRELESS_11B|WIRELESS_11G|WIRELESS_11_24N|WIRELESS_11_5N),
};

#define SUPPORTED_24G_NETTYPE_MSK (WIRELESS_11B | WIRELESS_11G | WIRELESS_11_24N)
#define SUPPORTED_5G_NETTYPE_MSK (WIRELESS_11A | WIRELESS_11_5N)

#define IsSupported24G(NetType) ((NetType) & SUPPORTED_24G_NETTYPE_MSK ? true : false)
#define IsSupported5G(NetType) ((NetType) & SUPPORTED_5G_NETTYPE_MSK ? true : false)

#define IsEnableHWCCK(NetType) IsSupported24G(NetType)
#define IsEnableHWOFDM(NetType) ((NetType) & (WIRELESS_11G|WIRELESS_11_24N|SUPPORTED_5G_NETTYPE_MSK) ? true : false)

#define IsSupportedRxCCK(NetType) IsEnableHWCCK(NetType)
#define IsSupportedRxOFDM(NetType) IsEnableHWOFDM(NetType)
#define IsSupportedRxMCS(NetType) IsEnableHWOFDM(NetType)

#define IsSupportedTxCCK(NetType) ((NetType) & (WIRELESS_11B) ? true : false)
#define IsSupportedTxOFDM(NetType) ((NetType) & (WIRELESS_11G|WIRELESS_11A) ? true : false)
#define IsSupportedTxMCS(NetType) ((NetType) & (WIRELESS_11_24N|WIRELESS_11_5N) ? true : false)


struct ieee_param {
	u32 cmd;
	u8 sta_addr[ETH_ALEN];
	union {
		struct {
			u8 name;
			u32 value;
		} wpa_param;
		struct {
			u32 len;
			u8 reserved[32];
			u8 data[0];
		} wpa_ie;
	        struct{
			int command;
			int reason_code;
		} mlme;
		struct {
			u8 alg[IEEE_CRYPT_ALG_NAME_LEN];
			u8 set_tx;
			u32 err;
			u8 idx;
			u8 seq[8]; /* sequence counter (set: RX, get: TX) */
			u16 key_len;
			u8 key[0];
		} crypt;
#ifdef CONFIG_8723AU_AP_MODE
		struct {
			u16 aid;
			u16 capability;
			int flags;
			u8 tx_supp_rates[16];
			struct ieee80211_ht_cap ht_cap;
		} add_sta;
		struct {
			u8	reserved[2];/* for set max_num_sta */
			u8	buf[0];
		} bcn_ie;
#endif

	} u;
};


#define MIN_FRAG_THRESHOLD     256U
#define	MAX_FRAG_THRESHOLD     2346U

/* QoS,QOS */
#define NORMAL_ACK			0
#define NO_ACK				1
#define NON_EXPLICIT_ACK	2
#define BLOCK_ACK			3

/* IEEE 802.11 defines */

#define P80211_OUI_LEN 3

struct ieee80211_snap_hdr {

        u8    dsap;   /* always 0xAA */
        u8    ssap;   /* always 0xAA */
        u8    ctrl;   /* always 0x03 */
        u8    oui[P80211_OUI_LEN];    /* organizational universal id */

} __attribute__ ((packed));


#define SNAP_SIZE sizeof(struct ieee80211_snap_hdr)

#define WLAN_FC_GET_TYPE(fc) ((fc) & IEEE80211_FCTL_FTYPE)
#define WLAN_FC_GET_STYPE(fc) ((fc) & IEEE80211_FCTL_STYPE)

#define WLAN_QC_GET_TID(qc) ((qc) & 0x0f)

#define WLAN_GET_SEQ_FRAG(seq) ((seq) & RTW_IEEE80211_SCTL_FRAG)
#define WLAN_GET_SEQ_SEQ(seq)  ((seq) & RTW_IEEE80211_SCTL_SEQ)


#define WLAN_REASON_JOIN_WRONG_CHANNEL       65534
#define WLAN_REASON_EXPIRATION_CHK 65535



#define IEEE80211_STATMASK_SIGNAL (1<<0)
#define IEEE80211_STATMASK_RSSI (1<<1)
#define IEEE80211_STATMASK_NOISE (1<<2)
#define IEEE80211_STATMASK_RATE (1<<3)
#define IEEE80211_STATMASK_WEMASK 0x7


#define IEEE80211_CCK_MODULATION    (1<<0)
#define IEEE80211_OFDM_MODULATION   (1<<1)

#define IEEE80211_24GHZ_BAND     (1<<0)
#define IEEE80211_52GHZ_BAND     (1<<1)

#define IEEE80211_CCK_RATE_LEN			4
#define IEEE80211_NUM_OFDM_RATESLEN	8


#define IEEE80211_CCK_RATE_1MB		        0x02
#define IEEE80211_CCK_RATE_2MB		        0x04
#define IEEE80211_CCK_RATE_5MB		        0x0B
#define IEEE80211_CCK_RATE_11MB		        0x16
#define IEEE80211_OFDM_RATE_LEN			8
#define IEEE80211_OFDM_RATE_6MB		        0x0C
#define IEEE80211_OFDM_RATE_9MB		        0x12
#define IEEE80211_OFDM_RATE_12MB		0x18
#define IEEE80211_OFDM_RATE_18MB		0x24
#define IEEE80211_OFDM_RATE_24MB		0x30
#define IEEE80211_OFDM_RATE_36MB		0x48
#define IEEE80211_OFDM_RATE_48MB		0x60
#define IEEE80211_OFDM_RATE_54MB		0x6C
#define IEEE80211_BASIC_RATE_MASK		0x80

#define IEEE80211_CCK_RATE_1MB_MASK		(1<<0)
#define IEEE80211_CCK_RATE_2MB_MASK		(1<<1)
#define IEEE80211_CCK_RATE_5MB_MASK		(1<<2)
#define IEEE80211_CCK_RATE_11MB_MASK		(1<<3)
#define IEEE80211_OFDM_RATE_6MB_MASK		(1<<4)
#define IEEE80211_OFDM_RATE_9MB_MASK		(1<<5)
#define IEEE80211_OFDM_RATE_12MB_MASK		(1<<6)
#define IEEE80211_OFDM_RATE_18MB_MASK		(1<<7)
#define IEEE80211_OFDM_RATE_24MB_MASK		(1<<8)
#define IEEE80211_OFDM_RATE_36MB_MASK		(1<<9)
#define IEEE80211_OFDM_RATE_48MB_MASK		(1<<10)
#define IEEE80211_OFDM_RATE_54MB_MASK		(1<<11)

#define IEEE80211_CCK_RATES_MASK	        0x0000000F
#define IEEE80211_CCK_BASIC_RATES_MASK	(IEEE80211_CCK_RATE_1MB_MASK | \
	IEEE80211_CCK_RATE_2MB_MASK)
#define IEEE80211_CCK_DEFAULT_RATES_MASK	(IEEE80211_CCK_BASIC_RATES_MASK | \
        IEEE80211_CCK_RATE_5MB_MASK | \
        IEEE80211_CCK_RATE_11MB_MASK)

#define IEEE80211_OFDM_RATES_MASK		0x00000FF0
#define IEEE80211_OFDM_BASIC_RATES_MASK	(IEEE80211_OFDM_RATE_6MB_MASK | \
	IEEE80211_OFDM_RATE_12MB_MASK | \
	IEEE80211_OFDM_RATE_24MB_MASK)
#define IEEE80211_OFDM_DEFAULT_RATES_MASK	(IEEE80211_OFDM_BASIC_RATES_MASK | \
	IEEE80211_OFDM_RATE_9MB_MASK  | \
	IEEE80211_OFDM_RATE_18MB_MASK | \
	IEEE80211_OFDM_RATE_36MB_MASK | \
	IEEE80211_OFDM_RATE_48MB_MASK | \
	IEEE80211_OFDM_RATE_54MB_MASK)
#define IEEE80211_DEFAULT_RATES_MASK (IEEE80211_OFDM_DEFAULT_RATES_MASK | \
                                IEEE80211_CCK_DEFAULT_RATES_MASK)

#define IEEE80211_NUM_OFDM_RATES	    8
#define IEEE80211_NUM_CCK_RATES	            4
#define IEEE80211_OFDM_SHIFT_MASK_A         4

#define WEP_KEYS 4
#define WEP_KEY_LEN 13



/*

 802.11 data frame from AP

      ,-------------------------------------------------------------------.
Bytes |  2   |  2   |    6    |    6    |    6    |  2   | 0..2312 |   4  |
      |------|------|---------|---------|---------|------|---------|------|
Desc. | ctrl | dura |  DA/RA  |   TA    |    SA   | Sequ |  frame  |  fcs |
      |      | tion | (BSSID) |         |         | ence |  data   |      |
      `-------------------------------------------------------------------'

Total: 28-2340 bytes

*/

struct ieee80211_header_data {
	u16 frame_ctl;
	u16 duration_id;
	u8 addr1[6];
	u8 addr2[6];
	u8 addr3[6];
	u16 seq_ctrl;
};

struct ieee80211_info_element_hdr {
	u8 id;
	u8 len;
} __attribute__ ((packed));

struct ieee80211_info_element {
	u8 id;
	u8 len;
	u8 data[0];
} __attribute__ ((packed));


struct ieee80211_txb {
	u8 nr_frags;
	u8 encrypted;
	u16 reserved;
	u16 frag_size;
	u16 payload_size;
	struct sk_buff *fragments[0];
};


/* MAX_RATES_LENGTH needs to be 12.  The spec says 8, and many APs
 * only use 8, and then use extended rates for the remaining supported
 * rates.  Other APs, however, stick all of their supported rates on the
 * main rates information element... */
#define MAX_RATES_LENGTH                  ((u8)12)
#define MAX_RATES_EX_LENGTH               ((u8)16)
#define MAX_CHANNEL_NUMBER                 161

#define MAX_WPA_IE_LEN (256)
#define MAX_WPS_IE_LEN (512)
#define MAX_P2P_IE_LEN (256)
#define MAX_WFD_IE_LEN (128)

#define IW_ESSID_MAX_SIZE 32

/*
join_res:
-1: authentication fail
-2: association fail
> 0: TID
*/

#define DEFAULT_MAX_SCAN_AGE (15 * HZ)
#define DEFAULT_FTS 2346
#define MAC_FMT "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC_ARG(x) ((u8*)(x))[0],((u8*)(x))[1],((u8*)(x))[2],((u8*)(x))[3],((u8*)(x))[4],((u8*)(x))[5]

#define CFG_IEEE80211_RESERVE_FCS (1<<0)
#define CFG_IEEE80211_COMPUTE_FCS (1<<1)

#define MAXTID	16

#define IEEE_A            (1<<0)
#define IEEE_B            (1<<1)
#define IEEE_G            (1<<2)
#define IEEE_MODE_MASK    (IEEE_A|IEEE_B|IEEE_G)

/* Baron move to ieee80211.c */
int ieee80211_is_empty_essid23a(const char *essid, int essid_len);

enum _PUBLIC_ACTION{
	ACT_PUBLIC_BSSCOEXIST = 0, /*  20/40 BSS Coexistence */
	ACT_PUBLIC_DSE_ENABLE = 1,
	ACT_PUBLIC_DSE_DEENABLE = 2,
	ACT_PUBLIC_DSE_REG_LOCATION = 3,
	ACT_PUBLIC_EXT_CHL_SWITCH = 4,
	ACT_PUBLIC_DSE_MSR_REQ = 5,
	ACT_PUBLIC_DSE_MSR_RPRT = 6,
	ACT_PUBLIC_MP = 7, /*  Measurement Pilot */
	ACT_PUBLIC_DSE_PWR_CONSTRAINT = 8,
	ACT_PUBLIC_VENDOR = 9, /*  for WIFI_DIRECT */
	ACT_PUBLIC_GAS_INITIAL_REQ = 10,
	ACT_PUBLIC_GAS_INITIAL_RSP = 11,
	ACT_PUBLIC_GAS_COMEBACK_REQ = 12,
	ACT_PUBLIC_GAS_COMEBACK_RSP = 13,
	ACT_PUBLIC_TDLS_DISCOVERY_RSP = 14,
	ACT_PUBLIC_LOCATION_TRACK = 15,
	ACT_PUBLIC_MAX
};

#define WME_OUI_TYPE 2
#define WME_OUI_SUBTYPE_INFORMATION_ELEMENT 0
#define WME_OUI_SUBTYPE_PARAMETER_ELEMENT 1
#define WME_OUI_SUBTYPE_TSPEC_ELEMENT 2
#define WME_VERSION 1


#define OUI_BROADCOM 0x00904c /* Broadcom (Epigram) */

#define VENDOR_HT_CAPAB_OUI_TYPE 0x33 /* 00-90-4c:0x33 */

/* Represent channel details, subset of ieee80211_channel */
struct rtw_ieee80211_channel {
	/* enum ieee80211_band band; */
	/* u16 center_freq; */
	u16 hw_value;
	u32 flags;
	/* int max_antenna_gain; */
	/* int max_power; */
	/* int max_reg_power; */
	/* bool beacon_found; */
	/* u32 orig_flags; */
	/* int orig_mag; */
	/* int orig_mpwr; */
};

#define CHAN_FMT \
	/*"band:%d, "*/ \
	/*"center_freq:%u, "*/ \
	"hw_value:%u, " \
	"flags:0x%08x" \
	/*"max_antenna_gain:%d\n"*/ \
	/*"max_power:%d\n"*/ \
	/*"max_reg_power:%d\n"*/ \
	/*"beacon_found:%u\n"*/ \
	/*"orig_flags:0x%08x\n"*/ \
	/*"orig_mag:%d\n"*/ \
	/*"orig_mpwr:%d\n"*/

#define CHAN_ARG(channel) \
	/*(channel)->band*/ \
	/*, (channel)->center_freq*/ \
	(channel)->hw_value \
	, (channel)->flags \
	/*, (channel)->max_antenna_gain*/ \
	/*, (channel)->max_power*/ \
	/*, (channel)->max_reg_power*/ \
	/*, (channel)->beacon_found*/ \
	/*, (channel)->orig_flags*/ \
	/*, (channel)->orig_mag*/ \
	/*, (channel)->orig_mpwr*/ \

/* Parsed Information Elements */
struct rtw_ieee802_11_elems {
	u8 *ssid;
	u8 ssid_len;
	u8 *supp_rates;
	u8 supp_rates_len;
	u8 *fh_params;
	u8 fh_params_len;
	u8 *ds_params;
	u8 ds_params_len;
	u8 *cf_params;
	u8 cf_params_len;
	u8 *tim;
	u8 tim_len;
	u8 *ibss_params;
	u8 ibss_params_len;
	u8 *challenge;
	u8 challenge_len;
	u8 *erp_info;
	u8 erp_info_len;
	u8 *ext_supp_rates;
	u8 ext_supp_rates_len;
	u8 *wpa_ie;
	u8 wpa_ie_len;
	u8 *rsn_ie;
	u8 rsn_ie_len;
	u8 *wme;
	u8 wme_len;
	u8 *wme_tspec;
	u8 wme_tspec_len;
	u8 *wps_ie;
	u8 wps_ie_len;
	u8 *power_cap;
	u8 power_cap_len;
	u8 *supp_channels;
	u8 supp_channels_len;
	u8 *mdie;
	u8 mdie_len;
	u8 *ftie;
	u8 ftie_len;
	u8 *timeout_int;
	u8 timeout_int_len;
	u8 *ht_capabilities;
	u8 ht_capabilities_len;
	u8 *ht_operation;
	u8 ht_operation_len;
	u8 *vendor_ht_cap;
	u8 vendor_ht_cap_len;
};

enum parse_res {
	ParseOK = 0,
	ParseUnknown = 1,
	ParseFailed = -1
};

enum parse_res rtw_ieee802_11_parse_elems23a(u8 *start, uint len,
				struct rtw_ieee802_11_elems *elems,
				int show_errors);

u8 *rtw_set_fixed_ie23a(unsigned char *pbuf, unsigned int len, unsigned char *source, unsigned int *frlen);
u8 *rtw_set_ie23a(u8 *pbuf, int index, uint len, u8 *source, uint *frlen);

enum secondary_ch_offset {
	SCN = 0, /* no secondary channel */
	SCA = 1, /* secondary channel above */
	SCB = 3,  /* secondary channel below */
};
u8 secondary_ch_offset_to_hal_ch_offset23a(u8 ch_offset);
u8 hal_ch_offset_to_secondary_ch_offset23a(u8 ch_offset);
u8 *rtw_set_ie23a_ch_switch(u8 *buf, u32 *buf_len, u8 ch_switch_mode, u8 new_ch, u8 ch_switch_cnt);
u8 *rtw_set_ie23a_secondary_ch_offset(u8 *buf, u32 *buf_len, u8 secondary_ch_offset);
u8 *rtw_set_ie23a_mesh_ch_switch_parm(u8 *buf, u32 *buf_len, u8 ttl, u8 flags, u16 reason, u16 precedence);

u8 *rtw_get_ie23a(u8*pbuf, int index, int *len, int limit);
u8 *rtw_get_ie23a_ex(u8 *in_ie, uint in_len, u8 eid, u8 *oui, u8 oui_len, u8 *ie, uint *ielen);
int rtw_ies_remove_ie23a(u8 *ies, uint *ies_len, uint offset, u8 eid, u8 *oui, u8 oui_len);

void rtw_set_supported_rate23a(u8* SupportedRates, uint mode) ;

unsigned char *rtw_get_wpa_ie23a(unsigned char *pie, int *wpa_ie_len, int limit);
unsigned char *rtw_get_wpa2_ie23a(unsigned char *pie, int *rsn_ie_len, int limit);
int rtw_get_wpa_cipher_suite23a(u8 *s);
int rtw_get_wpa2_cipher_suite23a(u8 *s);
int rtw_parse_wpa_ie23a(u8* wpa_ie, int wpa_ie_len, int *group_cipher, int *pairwise_cipher, int *is_8021x);
int rtw_parse_wpa2_ie23a(u8* wpa_ie, int wpa_ie_len, int *group_cipher, int *pairwise_cipher, int *is_8021x);

int rtw_get_sec_ie23a(u8 *in_ie,uint in_len,u8 *rsn_ie,u16 *rsn_len,u8 *wpa_ie,u16 *wpa_len);

u8 rtw_is_wps_ie23a(u8 *ie_ptr, uint *wps_ielen);
u8 *rtw_get_wps_ie23a(u8 *in_ie, uint in_len, u8 *wps_ie, uint *wps_ielen);
u8 *rtw_get_wps_attr23a(u8 *wps_ie, uint wps_ielen, u16 target_attr_id ,u8 *buf_attr, u32 *len_attr);
u8 *rtw_get_wps_attr_content23a(u8 *wps_ie, uint wps_ielen, u16 target_attr_id ,u8 *buf_content, uint *len_content);

/**
 * for_each_ie - iterate over continuous IEs
 * @ie:
 * @buf:
 * @buf_len:
 */
#define for_each_ie(ie, buf, buf_len) \
	for (ie = (void*)buf; (((u8*)ie) - ((u8*)buf) + 1) < buf_len; ie = (void*)(((u8*)ie) + *(((u8*)ie)+1) + 2))

void dump_ies23a(u8 *buf, u32 buf_len);
void dump_wps_ie23a(u8 *ie, u32 ie_len);

#ifdef CONFIG_8723AU_P2P
void dump_p2p_ie23a(u8 *ie, u32 ie_len);
u8 *rtw_get_p2p_ie23a(u8 *in_ie, int in_len, u8 *p2p_ie, uint *p2p_ielen);
u8 *rtw_get_p2p_attr23a(u8 *p2p_ie, uint p2p_ielen, u8 target_attr_id ,u8 *buf_attr, u32 *len_attr);
u8 *rtw_get_p2p_attr23a_content(u8 *p2p_ie, uint p2p_ielen, u8 target_attr_id ,u8 *buf_content, uint *len_content);
u32 rtw_set_p2p_attr_content23a(u8 *pbuf, u8 attr_id, u16 attr_len, u8 *pdata_attr);
void rtw_wlan_bssid_ex_remove_p2p_attr23a(struct wlan_bssid_ex *bss_ex, u8 attr_id);
#endif

#ifdef CONFIG_8723AU_P2P
int rtw_get_wfd_ie(u8 *in_ie, int in_len, u8 *wfd_ie, uint *wfd_ielen);
int rtw_get_wfd_attr_content(u8 *wfd_ie, uint wfd_ielen, u8 target_attr_id ,u8 *attr_content, uint *attr_contentlen);
#endif /*  CONFIG_8723AU_P2P */

uint	rtw_get_rateset_len23a(u8	*rateset);

struct registry_priv;
int rtw_generate_ie23a(struct registry_priv *pregistrypriv);


int rtw_get_bit_value_from_ieee_value23a(u8 val);

uint rtw_is_cckrates_included23a(u8 *rate);

uint rtw_is_cckratesonly_included23a(u8 *rate);

int rtw_check_network_type23a(unsigned char *rate, int ratelen, int channel);

void rtw_get_bcn_info23a(struct wlan_network *pnetwork);

void rtw_macaddr_cfg23a(u8 *mac_addr);

u16 rtw_mcs_rate23a(u8 rf_type, u8 bw_40MHz, u8 short_GI_20, u8 short_GI_40, unsigned char * MCS_rate);

int rtw_action_frame_parse23a(const u8 *frame, u32 frame_len, u8* category, u8 *action);
const char *action_public_str23a(u8 action);

#endif /* IEEE80211_H */
