/******************************************************************************
 *
 * Copyright(c) 2007 - 2012 Realtek Corporation. All rights reserved.
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
/*-----------------------------------------------------------------------------

	For type defines and data structure defines

------------------------------------------------------------------------------*/


#ifndef __DRV_TYPES_H__
#define __DRV_TYPES_H__

#include <osdep_service.h>
#include <wlan_bssdef.h>


enum _NIC_VERSION {
	RTL8711_NIC,
	RTL8712_NIC,
	RTL8713_NIC,
	RTL8716_NIC

};


#include <rtw_ht.h>

#include <rtw_cmd.h>
#include <wlan_bssdef.h>
#include <rtw_xmit.h>
#include <rtw_recv.h>
#include <hal_intf.h>
#include <hal_com.h>
#include <rtw_qos.h>
#include <rtw_security.h>
#include <rtw_pwrctrl.h>
#include <rtw_io.h>
#include <rtw_eeprom.h>
#include <sta_info.h>
#include <rtw_mlme.h>
#include <rtw_debug.h>
#include <rtw_rf.h>
#include <rtw_event.h>
#include <rtw_led.h>
#include <rtw_mlme_ext.h>
#include <rtw_p2p.h>
#include <rtw_ap.h>

#include "ioctl_cfg80211.h"

#define SPEC_DEV_ID_NONE BIT(0)
#define SPEC_DEV_ID_DISABLE_HT BIT(1)
#define SPEC_DEV_ID_ENABLE_PS BIT(2)
#define SPEC_DEV_ID_RF_CONFIG_1T1R BIT(3)
#define SPEC_DEV_ID_RF_CONFIG_2T2R BIT(4)
#define SPEC_DEV_ID_ASSIGN_IFNAME BIT(5)

struct specific_device_id {
	u32		flags;

	u16		idVendor;
	u16		idProduct;

};

struct registry_priv {
	u8	chip_version;
	u8	rfintfs;
	struct	cfg80211_ssid ssid;
	u8	channel;/* ad-hoc support requirement */
	u8	wireless_mode;/* A, B, G, auto */
	u8	scan_mode;/* active, passive */
	u8	preamble;/* long, short, auto */
	u8	vrtl_carrier_sense;/* Enable, Disable, Auto */
	u8	vcs_type;/* RTS/CTS, CTS-to-self */
	u16	rts_thresh;
	u16  frag_thresh;
	u8	adhoc_tx_pwr;
	u8	soft_ap;
	u8	power_mgnt;
	u8	ips_mode;
	u8	smart_ps;
	u8	long_retry_lmt;
	u8	short_retry_lmt;
	u16	busy_thresh;
	u8	ack_policy;
	u8	software_encrypt;
	u8	software_decrypt;
	u8	acm_method;
	  /* UAPSD */
	u8	wmm_enable;
	u8	uapsd_enable;

	struct wlan_bssid_ex    dev_network;

	u8	ht_enable;
	u8	cbw40_enable;
	u8	ampdu_enable;/* for tx */
	u8	rx_stbc;
	u8	ampdu_amsdu;/* A-MPDU Supports A-MSDU is permitted */
	u8	lowrate_two_xmit;

	u8	rf_config;
	u8	low_power;

	u8	wifi_spec;/*  !turbo_mode */

	u8	channel_plan;
#ifdef CONFIG_8723AU_BT_COEXIST
	u8	btcoex;
	u8	bt_iso;
	u8	bt_sco;
	u8	bt_ampdu;
#endif
	bool	bAcceptAddbaReq;

	u8	antdiv_cfg;
	u8	antdiv_type;

	u8	usbss_enable;/* 0:disable,1:enable */
	u8	hwpdn_mode;/* 0:disable,1:enable,2:decide by EFUSE config */
	u8	hwpwrp_detect;/* 0:disable,1:enable */

	u8	hw_wps_pbc;/* 0:disable,1:enable */

	u8	max_roaming_times; /* max number driver will try to roaming */

	u8 enable80211d;

	u8 ifname[16];
	u8 if2name[16];

	u8 notch_filter;

	u8 regulatory_tid;
};


#define MAX_CONTINUAL_URB_ERR 4

#define GET_PRIMARY_ADAPTER(padapter)					\
	(((struct rtw_adapter *)padapter)->dvobj->if1)

enum _IFACE_ID {
	IFACE_ID0, /* maping to PRIMARY_ADAPTER */
	IFACE_ID1, /* maping to SECONDARY_ADAPTER */
	IFACE_ID2,
	IFACE_ID3,
	IFACE_ID_MAX,
};

struct dvobj_priv {
	struct rtw_adapter *if1; /* PRIMARY_ADAPTER */
	struct rtw_adapter *if2; /* SECONDARY_ADAPTER */

	/* for local/global synchronization */
	struct mutex hw_init_mutex;
	struct mutex h2c_fwcmd_mutex;
	struct mutex setch_mutex;
	struct mutex setbw_mutex;

	unsigned char	oper_channel; /* saved chan info when set chan bw */
	unsigned char	oper_bwmode;
	unsigned char	oper_ch_offset;/* PRIME_CHNL_OFFSET */

	struct rtw_adapter *padapters[IFACE_ID_MAX];
	u8 iface_nums; /*  total number of ifaces used runtime */

	/* For 92D, DMDP have 2 interface. */
	u8	InterfaceNumber;
	u8	NumInterfaces;

	/* In /Out Pipe information */
	int	RtInPipe[2];
	int	RtOutPipe[3];
	u8	Queue2Pipe[HW_QUEUE_ENTRY];/* for out pipe mapping */

	u8	irq_alloc;

/*-------- below is for USB INTERFACE --------*/

	u8	nr_endpoint;
	u8	ishighspeed;
	u8	RtNumInPipes;
	u8	RtNumOutPipes;
	int	ep_num[5]; /* endpoint number */

	int	RegUsbSS;

	struct semaphore usb_suspend_sema;

	struct mutex  usb_vendor_req_mutex;

	u8 *usb_alloc_vendor_req_buf;
	u8 *usb_vendor_req_buf;

	struct usb_interface *pusbintf;
	struct usb_device *pusbdev;
	atomic_t continual_urb_error;

/*-------- below is for PCIE INTERFACE --------*/

};

static inline struct device *dvobj_to_dev(struct dvobj_priv *dvobj)
{
	/* todo: get interface type from dvobj and the return the dev accordingly */
	return &dvobj->pusbintf->dev;
}

enum _IFACE_TYPE {
	IFACE_PORT0, /* mapping to port0 for C/D series chips */
	IFACE_PORT1, /* mapping to port1 for C/D series chip */
	MAX_IFACE_PORT,
};

enum _ADAPTER_TYPE {
	PRIMARY_ADAPTER,
	SECONDARY_ADAPTER,
	MAX_ADAPTER,
};

struct rtw_adapter {
	int	pid[3];/* process id from UI, 0:wps, 1:hostapd, 2:dhcpcd */
	int	bDongle;/* build-in module or external dongle */
	u16	chip_type;
	u16	HardwareType;

	struct dvobj_priv *dvobj;
	struct	mlme_priv mlmepriv;
	struct	mlme_ext_priv mlmeextpriv;
	struct	cmd_priv	cmdpriv;
	struct	evt_priv	evtpriv;
	/* struct	io_queue	*pio_queue; */
	struct	io_priv	iopriv;
	struct	xmit_priv	xmitpriv;
	struct	recv_priv	recvpriv;
	struct	sta_priv	stapriv;
	struct	security_priv	securitypriv;
	struct	registry_priv	registrypriv;
	struct	pwrctrl_priv	pwrctrlpriv;
	struct	eeprom_priv eeprompriv;
	struct	led_priv	ledpriv;

#ifdef CONFIG_8723AU_AP_MODE
	struct	hostapd_priv	*phostapdpriv;
#endif

	struct cfg80211_wifidirect_info	cfg80211_wdinfo;
	u32	setband;
	struct wifidirect_info	wdinfo;

#ifdef CONFIG_8723AU_P2P
	struct wifi_display_info wfd_info;
#endif /* CONFIG_8723AU_P2P */

	void *HalData;
	u32 hal_data_sz;
	struct hal_ops	HalFunc;

	s32	bDriverStopped;
	s32	bSurpriseRemoved;
	s32  bCardDisableWOHSM;

	u32	IsrContent;
	u32	ImrContent;

	u8	EepromAddressSize;
	u8	hw_init_completed;
	u8	bDriverIsGoingToUnload;
	u8	init_adpt_in_progress;
	u8	bHaltInProgress;

	void *cmdThread;
	void *evtThread;
	void *xmitThread;
	void *recvThread;

	void (*intf_start)(struct rtw_adapter *adapter);
	void (*intf_stop)(struct rtw_adapter *adapter);

	struct net_device *pnetdev;

	/*  used by rtw_rereg_nd_name related function */
	struct rereg_nd_name_data {
		struct net_device *old_pnetdev;
		char old_ifname[IFNAMSIZ];
		u8 old_ips_mode;
		u8 old_bRegUseLed;
	} rereg_nd_name_priv;

	int bup;
	struct net_device_stats stats;
	struct iw_statistics iwstats;
	struct proc_dir_entry *dir_dev;/*  for proc directory */

	struct wireless_dev *rtw_wdev;
	int net_closed;

	u8 bFWReady;
	u8 bBTFWReady;
	u8 bReadPortCancel;
	u8 bWritePortCancel;
	u8 bRxRSSIDisplay;
	/* The driver will show the desired chan nor when this flag is 1. */
	u8 bNotifyChannelChange;
#ifdef CONFIG_8723AU_P2P
	/* driver will show current P2P status when the  application reads it*/
	u8 bShowGetP2PState;
#endif
	struct rtw_adapter *pbuddy_adapter;

	/* extend to support multi interface */
	/* IFACE_ID0 is equals to PRIMARY_ADAPTER */
	/* IFACE_ID1 is equals to SECONDARY_ADAPTER */
	u8 iface_id;

#ifdef CONFIG_BR_EXT
	_lock					br_ext_lock;
	/* unsigned int			macclone_completed; */
	struct nat25_network_db_entry	*nethash[NAT25_HASH_SIZE];
	int				pppoe_connection_in_progress;
	unsigned char			pppoe_addr[MACADDRLEN];
	unsigned char			scdb_mac[MACADDRLEN];
	unsigned char			scdb_ip[4];
	struct nat25_network_db_entry	*scdb_entry;
	unsigned char			br_mac[MACADDRLEN];
	unsigned char			br_ip[4];

	struct br_ext_info		ethBrExtInfo;
#endif	/*  CONFIG_BR_EXT */

	u8    fix_rate;

	unsigned char     in_cta_test;

};

#define adapter_to_dvobj(adapter) (adapter->dvobj)

int rtw_handle_dualmac23a(struct rtw_adapter *adapter, bool init);

static inline u8 *myid(struct eeprom_priv *peepriv)
{
	return peepriv->mac_addr;
}

#endif /* __DRV_TYPES_H__ */
