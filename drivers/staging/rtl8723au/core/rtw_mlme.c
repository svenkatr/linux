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
#define _RTW_MLME_C_

#include <osdep_service.h>
#include <drv_types.h>
#include <recv_osdep.h>
#include <xmit_osdep.h>
#include <hal_intf.h>
#include <mlme_osdep.h>
#include <sta_info.h>
#include <linux/ieee80211.h>
#include <wifi.h>
#include <wlan_bssdef.h>
#include <rtw_ioctl_set.h>

extern u8 rtw_do_join23a(struct rtw_adapter * padapter);

static void rtw_init_mlme_timer(struct rtw_adapter *padapter)
{
	struct	mlme_priv *pmlmepriv = &padapter->mlmepriv;

	setup_timer(&pmlmepriv->assoc_timer, rtw23a_join_to_handler,
		    (unsigned long)padapter);

	setup_timer(&pmlmepriv->scan_to_timer, rtw_scan_timeout_handler23a,
		    (unsigned long)padapter);

	setup_timer(&pmlmepriv->dynamic_chk_timer,
		    rtw_dynamic_check_timer_handler, (unsigned long)padapter);

	setup_timer(&pmlmepriv->set_scan_deny_timer,
		    rtw_set_scan_deny_timer_hdl, (unsigned long)padapter);
}

int _rtw_init_mlme_priv23a(struct rtw_adapter *padapter)
{
	struct mlme_priv *pmlmepriv = &padapter->mlmepriv;
	int res = _SUCCESS;

	pmlmepriv->nic_hdl = padapter;

	pmlmepriv->fw_state = 0;
	pmlmepriv->cur_network.network.InfrastructureMode = Ndis802_11AutoUnknown;
	pmlmepriv->scan_mode=SCAN_ACTIVE;/*  1: active, 0: pasive. Maybe someday we should rename this varable to "active_mode" (Jeff) */

	spin_lock_init(&pmlmepriv->lock);
	_rtw_init_queue23a(&pmlmepriv->scanned_queue);

	memset(&pmlmepriv->assoc_ssid, 0, sizeof(struct cfg80211_ssid));

	/* allocate DMA-able/Non-Page memory for cmd_buf and rsp_buf */

	rtw_clear_scan_deny(padapter);

	rtw_init_mlme_timer(padapter);
	return res;
}

#ifdef CONFIG_8723AU_AP_MODE
static void rtw_free_mlme_ie_data(u8 **ppie, u32 *plen)
{
	if(*ppie)
	{
		kfree(*ppie);
		*plen = 0;
		*ppie=NULL;
	}
}
#endif

void rtw23a_free_mlme_priv_ie_data(struct mlme_priv *pmlmepriv)
{
#ifdef CONFIG_8723AU_AP_MODE
	kfree(pmlmepriv->assoc_req);
	kfree(pmlmepriv->assoc_rsp);
	rtw_free_mlme_ie_data(&pmlmepriv->wps_beacon_ie, &pmlmepriv->wps_beacon_ie_len);
	rtw_free_mlme_ie_data(&pmlmepriv->wps_probe_req_ie, &pmlmepriv->wps_probe_req_ie_len);
	rtw_free_mlme_ie_data(&pmlmepriv->wps_probe_resp_ie, &pmlmepriv->wps_probe_resp_ie_len);
	rtw_free_mlme_ie_data(&pmlmepriv->wps_assoc_resp_ie, &pmlmepriv->wps_assoc_resp_ie_len);

	rtw_free_mlme_ie_data(&pmlmepriv->p2p_beacon_ie, &pmlmepriv->p2p_beacon_ie_len);
	rtw_free_mlme_ie_data(&pmlmepriv->p2p_probe_req_ie, &pmlmepriv->p2p_probe_req_ie_len);
	rtw_free_mlme_ie_data(&pmlmepriv->p2p_probe_resp_ie, &pmlmepriv->p2p_probe_resp_ie_len);
	rtw_free_mlme_ie_data(&pmlmepriv->p2p_go_probe_resp_ie, &pmlmepriv->p2p_go_probe_resp_ie_len);
	rtw_free_mlme_ie_data(&pmlmepriv->p2p_assoc_req_ie, &pmlmepriv->p2p_assoc_req_ie_len);

	rtw_free_mlme_ie_data(&pmlmepriv->wfd_beacon_ie, &pmlmepriv->wfd_beacon_ie_len);
	rtw_free_mlme_ie_data(&pmlmepriv->wfd_probe_req_ie, &pmlmepriv->wfd_probe_req_ie_len);
	rtw_free_mlme_ie_data(&pmlmepriv->wfd_probe_resp_ie, &pmlmepriv->wfd_probe_resp_ie_len);
	rtw_free_mlme_ie_data(&pmlmepriv->wfd_go_probe_resp_ie, &pmlmepriv->wfd_go_probe_resp_ie_len);
	rtw_free_mlme_ie_data(&pmlmepriv->wfd_assoc_req_ie, &pmlmepriv->wfd_assoc_req_ie_len);
#endif
}

void _rtw_free_mlme_priv23a(struct mlme_priv *pmlmepriv)
{

	rtw23a_free_mlme_priv_ie_data(pmlmepriv);

}

struct wlan_network *rtw_alloc_network(struct mlme_priv *pmlmepriv)
{
	struct wlan_network *pnetwork;

	pnetwork = kzalloc(sizeof(struct wlan_network), GFP_ATOMIC);
	if (pnetwork) {
		INIT_LIST_HEAD(&pnetwork->list);
		pnetwork->network_type = 0;
		pnetwork->fixed = false;
		pnetwork->last_scanned = jiffies;
		pnetwork->aid = 0;
		pnetwork->join_res = 0;
	}

	return pnetwork;
}

void _rtw_free_network23a(struct mlme_priv *pmlmepriv,
		       struct wlan_network *pnetwork, u8 isfreeall)
{
	u32 lifetime = SCANQUEUE_LIFETIME;

	if (!pnetwork)
		return;

	if (pnetwork->fixed == true)
		return;

	if ((check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE) == true) ||
	    (check_fwstate(pmlmepriv, WIFI_ADHOC_STATE) == true))
		lifetime = 1;

	list_del_init(&pnetwork->list);

	kfree(pnetwork);
}

void _rtw_free_network23a_nolock23a(struct mlme_priv *pmlmepriv,
			      struct wlan_network *pnetwork)
{

	if (pnetwork == NULL)
		return;

	if (pnetwork->fixed == true)
		return;

	list_del_init(&pnetwork->list);

	kfree(pnetwork);
}

/*
	return the wlan_network with the matching addr

	Shall be calle under atomic context... to avoid possible racing condition...
*/
struct wlan_network *
_rtw_find_network23a(struct rtw_queue *scanned_queue, u8 *addr)
{
	struct list_head *phead, *plist;
	struct wlan_network *pnetwork = NULL;

	if (is_zero_ether_addr(addr)) {
		pnetwork = NULL;
		goto exit;
	}

	/* spin_lock_bh(&scanned_queue->lock); */

	phead = get_list_head(scanned_queue);
	plist = phead->next;

	while (plist != phead) {
		pnetwork = container_of(plist, struct wlan_network, list);

		if (ether_addr_equal(addr, pnetwork->network.MacAddress))
			break;

		plist = plist->next;
        }

	if(plist == phead)
		pnetwork = NULL;

	/* spin_unlock_bh(&scanned_queue->lock); */

exit:

	return pnetwork;
}

void _rtw_free_network23a_queue23a(struct rtw_adapter *padapter, u8 isfreeall)
{
	struct list_head *phead, *plist, *ptmp;
	struct wlan_network *pnetwork;
	struct mlme_priv* pmlmepriv = &padapter->mlmepriv;
	struct rtw_queue *scanned_queue = &pmlmepriv->scanned_queue;

	spin_lock_bh(&scanned_queue->lock);

	phead = get_list_head(scanned_queue);

	list_for_each_safe(plist, ptmp, phead) {
		pnetwork = container_of(plist, struct wlan_network, list);

		_rtw_free_network23a(pmlmepriv,pnetwork, isfreeall);
	}

	spin_unlock_bh(&scanned_queue->lock);

}

int rtw_if_up23a(struct rtw_adapter *padapter)	{

	int res;

	if(padapter->bDriverStopped || padapter->bSurpriseRemoved ||
		(check_fwstate(&padapter->mlmepriv, _FW_LINKED)== false)) {
		RT_TRACE(_module_rtl871x_mlme_c_, _drv_info_, ("rtw_if_up23a:bDriverStopped(%d) OR bSurpriseRemoved(%d)", padapter->bDriverStopped, padapter->bSurpriseRemoved));
		res=false;
	}
	else
		res=  true;

	return res;
}

void rtw_generate_random_ibss23a(u8* pibss)
{
	unsigned long curtime = jiffies;

	pibss[0] = 0x02;  /* in ad-hoc mode bit1 must set to 1 */
	pibss[1] = 0x11;
	pibss[2] = 0x87;
	pibss[3] = (u8)(curtime & 0xff) ;/* p[0]; */
	pibss[4] = (u8)((curtime>>8) & 0xff) ;/* p[1]; */
	pibss[5] = (u8)((curtime>>16) & 0xff) ;/* p[2]; */

	return;
}

u8 *rtw_get_capability23a_from_ie(u8 *ie)
{
	return ie + 8 + 2;
}

u16 rtw_get_capability23a(struct wlan_bssid_ex *bss)
{
	u16	val;

	memcpy((u8 *)&val, rtw_get_capability23a_from_ie(bss->IEs), 2);

	return le16_to_cpu(val);
}

u8 *rtw_get_timestampe_from_ie23a(u8 *ie)
{
	return ie + 0;
}

u8 *rtw_get_beacon_interval23a_from_ie(u8 *ie)
{
	return ie + 8;
}

int	rtw_init_mlme_priv23a (struct rtw_adapter *padapter)/* struct	mlme_priv *pmlmepriv) */
{
	int	res;

	res = _rtw_init_mlme_priv23a(padapter);/*  (pmlmepriv); */

	return res;
}

void rtw_free_mlme_priv23a (struct mlme_priv *pmlmepriv)
{

	RT_TRACE(_module_rtl871x_mlme_c_, _drv_err_, ("rtw_free_mlme_priv23a\n"));
	_rtw_free_mlme_priv23a(pmlmepriv);

}

void rtw_free_network(struct mlme_priv *pmlmepriv, struct	wlan_network *pnetwork, u8 is_freeall);
void rtw_free_network(struct mlme_priv *pmlmepriv, struct	wlan_network *pnetwork, u8 is_freeall)/* struct	wlan_network *pnetwork, _queue	*free_queue) */
{

	RT_TRACE(_module_rtl871x_mlme_c_, _drv_err_,
		 ("rtw_free_network ==> ssid = %s\n\n" ,
		  pnetwork->network.Ssid.ssid));
	_rtw_free_network23a(pmlmepriv, pnetwork, is_freeall);

}

void rtw_free_network_nolock(struct mlme_priv *pmlmepriv, struct wlan_network *pnetwork);
void rtw_free_network_nolock(struct mlme_priv *pmlmepriv, struct wlan_network *pnetwork)
{

	/* RT_TRACE(_module_rtl871x_mlme_c_, _drv_err_, ("rtw_free_network ==> ssid = %s\n\n" , pnetwork->network.Ssid.ssid)); */
	_rtw_free_network23a_nolock23a(pmlmepriv, pnetwork);

}

void rtw_free_network_queue23a(struct rtw_adapter* dev, u8 isfreeall)
{

	_rtw_free_network23a_queue23a(dev, isfreeall);

}

/*
	return the wlan_network with the matching addr

	Shall be calle under atomic context... to avoid possible racing condition...
*/
struct	wlan_network *
rtw_find_network23a(struct rtw_queue *scanned_queue, u8 *addr)
{
	struct wlan_network *pnetwork;

	pnetwork = _rtw_find_network23a(scanned_queue, addr);

	return pnetwork;
}

int rtw_is_same_ibss23a(struct rtw_adapter *adapter, struct wlan_network *pnetwork)
{
	int ret = true;
	struct security_priv *psecuritypriv = &adapter->securitypriv;

	if ((psecuritypriv->dot11PrivacyAlgrthm != _NO_PRIVACY_) &&
		    (pnetwork->network.Privacy == 0))
	{
		ret = false;
	}
	else if ((psecuritypriv->dot11PrivacyAlgrthm == _NO_PRIVACY_) &&
		 (pnetwork->network.Privacy == 1))
	{
		ret = false;
	}
	else
	{
		ret = true;
	}

	return ret;
}

inline int is_same_ess(struct wlan_bssid_ex *a, struct wlan_bssid_ex *b);
inline int is_same_ess(struct wlan_bssid_ex *a, struct wlan_bssid_ex *b)
{
	/* RT_TRACE(_module_rtl871x_mlme_c_, _drv_err_, ("(%s,%d)(%s,%d)\n", */
	/*		a->Ssid.Ssid, a->Ssid.SsidLength, b->Ssid.Ssid, b->Ssid.SsidLength)); */
	return (a->Ssid.ssid_len == b->Ssid.ssid_len) &&
		!memcmp(a->Ssid.ssid, b->Ssid.ssid, a->Ssid.ssid_len);
}

int is_same_network23a(struct wlan_bssid_ex *src, struct wlan_bssid_ex *dst)
{
	 u16 s_cap, d_cap;

	memcpy((u8 *)&s_cap, rtw_get_capability23a_from_ie(src->IEs), 2);
	memcpy((u8 *)&d_cap, rtw_get_capability23a_from_ie(dst->IEs), 2);

	s_cap = le16_to_cpu(s_cap);
	d_cap = le16_to_cpu(d_cap);

	return ((src->Ssid.ssid_len == dst->Ssid.ssid_len) &&
		/*	(src->Configuration.DSConfig == dst->Configuration.DSConfig) && */
		ether_addr_equal(src->MacAddress, dst->MacAddress) &&
		((!memcmp(src->Ssid.ssid, dst->Ssid.ssid, src->Ssid.ssid_len))) &&
		((s_cap & WLAN_CAPABILITY_IBSS) ==
		 (d_cap & WLAN_CAPABILITY_IBSS)) &&
		((s_cap & WLAN_CAPABILITY_ESS) ==
		 (d_cap & WLAN_CAPABILITY_ESS)));
}

struct wlan_network *rtw_get_oldest_wlan_network23a(struct rtw_queue *scanned_queue)
{
	struct list_head *plist, *phead;

	struct wlan_network *pwlan;
	struct wlan_network *oldest = NULL;

	phead = get_list_head(scanned_queue);

	list_for_each(plist, phead) {
		pwlan = container_of(plist, struct wlan_network, list);

		if (pwlan->fixed != true) {
			if (!oldest || time_after(oldest->last_scanned,
						  pwlan->last_scanned))
				oldest = pwlan;
		}
	}

	return oldest;
}

void update_network23a(struct wlan_bssid_ex *dst, struct wlan_bssid_ex *src,
	struct rtw_adapter * padapter, bool update_ie)
{
	u8 ss_ori = dst->PhyInfo.SignalStrength;
	u8 sq_ori = dst->PhyInfo.SignalQuality;
	long rssi_ori = dst->Rssi;

	u8 ss_smp = src->PhyInfo.SignalStrength;
	u8 sq_smp = src->PhyInfo.SignalQuality;
	long rssi_smp = src->Rssi;

	u8 ss_final;
	u8 sq_final;
	long rssi_final;

	DBG_8723A("%s %s(%pM, ch%u) ss_ori:%3u, sq_ori:%3u, rssi_ori:%3ld, ss_smp:%3u, sq_smp:%3u, rssi_smp:%3ld\n",
		  __func__, src->Ssid.ssid, src->MacAddress,
		  src->Configuration.DSConfig, ss_ori, sq_ori, rssi_ori,
		  ss_smp, sq_smp, rssi_smp
	);

	/* The rule below is 1/5 for sample value, 4/5 for history value */
	if (check_fwstate(&padapter->mlmepriv, _FW_LINKED) && is_same_network23a(&padapter->mlmepriv.cur_network.network, src)) {
		/* Take the recvpriv's value for the connected AP*/
		ss_final = padapter->recvpriv.signal_strength;
		sq_final = padapter->recvpriv.signal_qual;
		/* the rssi value here is undecorated, and will be used for antenna diversity */
		if (sq_smp != 101) /* from the right channel */
			rssi_final = (src->Rssi+dst->Rssi*4)/5;
		else
			rssi_final = rssi_ori;
	}
	else {
		if (sq_smp != 101) { /* from the right channel */
			ss_final = ((u32)(src->PhyInfo.SignalStrength)+(u32)(dst->PhyInfo.SignalStrength)*4)/5;
			sq_final = ((u32)(src->PhyInfo.SignalQuality)+(u32)(dst->PhyInfo.SignalQuality)*4)/5;
			rssi_final = (src->Rssi+dst->Rssi*4)/5;
		} else {
			/* bss info not receving from the right channel, use the original RX signal infos */
			ss_final = dst->PhyInfo.SignalStrength;
			sq_final = dst->PhyInfo.SignalQuality;
			rssi_final = dst->Rssi;
		}

	}

	if (update_ie)
		memcpy((u8 *)dst, (u8 *)src, get_wlan_bssid_ex_sz(src));

	dst->PhyInfo.SignalStrength = ss_final;
	dst->PhyInfo.SignalQuality = sq_final;
	dst->Rssi = rssi_final;

	DBG_8723A("%s %s(%pM), SignalStrength:%u, SignalQuality:%u, RawRSSI:%ld\n",
		  __func__, dst->Ssid.ssid, dst->MacAddress,
		  dst->PhyInfo.SignalStrength,
		  dst->PhyInfo.SignalQuality, dst->Rssi);

}

static void update_current_network(struct rtw_adapter *adapter, struct wlan_bssid_ex *pnetwork)
{
	struct	mlme_priv *pmlmepriv = &adapter->mlmepriv;

	if ((check_fwstate(pmlmepriv, _FW_LINKED)== true) && (is_same_network23a(&pmlmepriv->cur_network.network, pnetwork)))
	{
		/* RT_TRACE(_module_rtl871x_mlme_c_,_drv_err_,"Same Network\n"); */

		/* if(pmlmepriv->cur_network.network.IELength<= pnetwork->IELength) */
		{
			update_network23a(&pmlmepriv->cur_network.network, pnetwork,adapter, true);
			rtw_update_protection23a(adapter, (pmlmepriv->cur_network.network.IEs) + sizeof (struct ndis_802_11_fixed_ies),
									pmlmepriv->cur_network.network.IELength);
		}
	}

}

/*

Caller must hold pmlmepriv->lock first.

*/
void rtw_update_scanned_network23a(struct rtw_adapter *adapter, struct wlan_bssid_ex *target)
{
	struct list_head *plist, *phead;
	struct mlme_priv *pmlmepriv = &adapter->mlmepriv;
	struct wlan_network *pnetwork = NULL;
	struct wlan_network *oldest = NULL;
	struct rtw_queue *queue = &pmlmepriv->scanned_queue;
	u32 bssid_ex_sz;
	int found = 0;

	spin_lock_bh(&queue->lock);
	phead = get_list_head(queue);

	list_for_each(plist, phead) {
		pnetwork = container_of(plist, struct wlan_network, list);

		if (is_same_network23a(&pnetwork->network, target)) {
			found = 1;
			break;
		}
		if (!oldest || time_after(oldest->last_scanned,
					  pnetwork->last_scanned))
			oldest = pnetwork;
	}

	/* If we didn't find a match, then get a new network slot to initialize
	 * with this beacon's information */
	if (!found) {
		pnetwork = rtw_alloc_network(pmlmepriv);
		if (!pnetwork) {
			if (!oldest) {
				RT_TRACE(_module_rtl871x_mlme_c_, _drv_err_,
					 ("\n\n\nsomething wrong here\n\n\n"));
				goto exit;
			}
			pnetwork = oldest;
		} else
			list_add_tail(&pnetwork->list, &queue->queue);

		bssid_ex_sz = get_wlan_bssid_ex_sz(target);
		target->Length = bssid_ex_sz;
		memcpy(&pnetwork->network, target, bssid_ex_sz);

		/*  variable initialize */
		pnetwork->fixed = false;
		pnetwork->last_scanned = jiffies;

		pnetwork->network_type = 0;
		pnetwork->aid = 0;
		pnetwork->join_res = 0;

		/* bss info not receving from the right channel */
		if (pnetwork->network.PhyInfo.SignalQuality == 101)
			pnetwork->network.PhyInfo.SignalQuality = 0;
	} else {
		/*
		 * we have an entry and we are going to update it. But
		 * this entry may be already expired. In this case we
		 * do the same as we found a new net and call the
		 * new_net handler
		 */
		bool update_ie = true;

		pnetwork->last_scanned = jiffies;

		/* target.reserved == 1, means that scanned network is
		 * a bcn frame. */
		if ((pnetwork->network.IELength>target->IELength) &&
		    (target->reserved == 1))
			update_ie = false;

		update_network23a(&pnetwork->network, target,adapter, update_ie);
	}

exit:
	spin_unlock_bh(&queue->lock);

}

void rtw_add_network(struct rtw_adapter *adapter, struct wlan_bssid_ex *pnetwork)
{
	update_current_network(adapter, pnetwork);
	rtw_update_scanned_network23a(adapter, pnetwork);
}

/* select the desired network based on the capability of the (i)bss. */
/*  check items: (1) security */
/*			   (2) network_type */
/*			   (3) WMM */
/*			   (4) HT */
/*                      (5) others */
int rtw_is_desired_network(struct rtw_adapter *adapter, struct wlan_network *pnetwork)
{
	struct security_priv *psecuritypriv = &adapter->securitypriv;
	struct mlme_priv *pmlmepriv = &adapter->mlmepriv;
	u32 desired_encmode;
	u32 privacy;

	/* u8 wps_ie[512]; */
	uint wps_ielen;

	int bselected = true;

	desired_encmode = psecuritypriv->ndisencryptstatus;
	privacy = pnetwork->network.Privacy;

	if (check_fwstate(pmlmepriv, WIFI_UNDER_WPS))
	{
		if (rtw_get_wps_ie23a(pnetwork->network.IEs+_FIXED_IE_LENGTH_, pnetwork->network.IELength-_FIXED_IE_LENGTH_, NULL, &wps_ielen)!= NULL)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	if (adapter->registrypriv.wifi_spec == 1) /* for  correct flow of 8021X  to do.... */
	{
		if ((desired_encmode == Ndis802_11EncryptionDisabled) && (privacy != 0))
	            bselected = false;
	}

	if ((desired_encmode != Ndis802_11EncryptionDisabled) && (privacy == 0)) {
		DBG_8723A("desired_encmode: %d, privacy: %d\n", desired_encmode, privacy);
		bselected = false;
	}

	if (check_fwstate(pmlmepriv, WIFI_ADHOC_STATE) == true)
	{
		if (pnetwork->network.InfrastructureMode != pmlmepriv->cur_network.network.InfrastructureMode)
			bselected = false;
	}

	return bselected;
}

/* TODO: Perry : For Power Management */
void rtw_atimdone_event_callback23a(struct rtw_adapter	*adapter , u8 *pbuf)
{

	RT_TRACE(_module_rtl871x_mlme_c_,_drv_err_,("receive atimdone_evet\n"));

	return;
}

void rtw_survey_event_cb23a(struct rtw_adapter	*adapter, u8 *pbuf)
{
	u32 len;
	struct wlan_bssid_ex *pnetwork;
	struct	mlme_priv *pmlmepriv = &adapter->mlmepriv;

	pnetwork = (struct wlan_bssid_ex *)pbuf;

	RT_TRACE(_module_rtl871x_mlme_c_,_drv_info_,("rtw_survey_event_cb23a, ssid=%s\n",  pnetwork->Ssid.ssid));

	len = get_wlan_bssid_ex_sz(pnetwork);
	if(len > (sizeof(struct wlan_bssid_ex)))
	{
		RT_TRACE(_module_rtl871x_mlme_c_,_drv_err_,("\n ****rtw_survey_event_cb23a: return a wrong bss ***\n"));
		return;
	}

	spin_lock_bh(&pmlmepriv->lock);

	/*  update IBSS_network 's timestamp */
	if ((check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE)) == true)
	{
		/* RT_TRACE(_module_rtl871x_mlme_c_, _drv_err_,"rtw_survey_event_cb23a : WIFI_ADHOC_MASTER_STATE\n\n"); */
		if (ether_addr_equal(pmlmepriv->cur_network.network.MacAddress,
				     pnetwork->MacAddress)) {
			struct wlan_network* ibss_wlan = NULL;

			memcpy(pmlmepriv->cur_network.network.IEs, pnetwork->IEs, 8);
			spin_lock_bh(&pmlmepriv->scanned_queue.lock);
			ibss_wlan = rtw_find_network23a(&pmlmepriv->scanned_queue,  pnetwork->MacAddress);
			if (ibss_wlan)
			{
				memcpy(ibss_wlan->network.IEs , pnetwork->IEs, 8);
				spin_unlock_bh(&pmlmepriv->scanned_queue.lock);
				goto exit;
			}
			spin_unlock_bh(&pmlmepriv->scanned_queue.lock);
		}
	}

	/*  lock pmlmepriv->lock when you accessing network_q */
	if ((check_fwstate(pmlmepriv, _FW_UNDER_LINKING)) == false)
	{
	        if (pnetwork->Ssid.ssid[0] == 0)
			pnetwork->Ssid.ssid_len = 0;

		rtw_add_network(adapter, pnetwork);
	}

exit:

	spin_unlock_bh(&pmlmepriv->lock);

	return;
}

void rtw_surveydone_event_callback23a(struct rtw_adapter	*adapter, u8 *pbuf)
{
	struct	mlme_priv *pmlmepriv = &adapter->mlmepriv;
	struct mlme_ext_priv *pmlmeext = &adapter->mlmeextpriv;

	spin_lock_bh(&pmlmepriv->lock);

	if (pmlmepriv->wps_probe_req_ie) {
		pmlmepriv->wps_probe_req_ie_len = 0;
		kfree(pmlmepriv->wps_probe_req_ie);
		pmlmepriv->wps_probe_req_ie = NULL;
	}

	RT_TRACE(_module_rtl871x_mlme_c_, _drv_info_, ("rtw_surveydone_event_callback23a: fw_state:%x\n\n", get_fwstate(pmlmepriv)));

	if (check_fwstate(pmlmepriv, _FW_UNDER_SURVEY)) {
		del_timer_sync(&pmlmepriv->scan_to_timer);

		_clr_fwstate_(pmlmepriv, _FW_UNDER_SURVEY);
	} else {

		RT_TRACE(_module_rtl871x_mlme_c_, _drv_err_, ("nic status =%x, survey done event comes too late!\n", get_fwstate(pmlmepriv)));
	}

	rtw_set_signal_stat_timer(&adapter->recvpriv);

	if (pmlmepriv->to_join == true) {
		if ((check_fwstate(pmlmepriv, WIFI_ADHOC_STATE) == true)) {
			if (check_fwstate(pmlmepriv, _FW_LINKED) == false) {
				set_fwstate(pmlmepriv, _FW_UNDER_LINKING);

				if (rtw_select_and_join_from_scanned_queue23a(pmlmepriv) == _SUCCESS) {
					mod_timer(&pmlmepriv->assoc_timer,
						  jiffies + msecs_to_jiffies(MAX_JOIN_TIMEOUT));
				} else {
					struct wlan_bssid_ex *pdev_network = &adapter->registrypriv.dev_network;
					u8 *pibss = adapter->registrypriv.dev_network.MacAddress;

					_clr_fwstate_(pmlmepriv, _FW_UNDER_SURVEY);

					RT_TRACE(_module_rtl871x_mlme_c_, _drv_err_, ("switching to adhoc master\n"));

					memset(&pdev_network->Ssid, 0, sizeof(struct cfg80211_ssid));
					memcpy(&pdev_network->Ssid, &pmlmepriv->assoc_ssid, sizeof(struct cfg80211_ssid));

					rtw_update_registrypriv_dev_network23a(adapter);
					rtw_generate_random_ibss23a(pibss);

					pmlmepriv->fw_state = WIFI_ADHOC_MASTER_STATE;

					if (rtw_createbss_cmd23a(adapter)!= _SUCCESS)
					{
					RT_TRACE(_module_rtl871x_mlme_c_, _drv_err_, ("Error =>rtw_createbss_cmd23a status FAIL\n"));
					}

					pmlmepriv->to_join = false;
				}
			}
		} else {
			int ret;
			set_fwstate(pmlmepriv, _FW_UNDER_LINKING);
			pmlmepriv->to_join = false;
			ret = rtw_select_and_join_from_scanned_queue23a(pmlmepriv);
			if (ret == _SUCCESS) {
				unsigned long e;
				e = msecs_to_jiffies(MAX_JOIN_TIMEOUT);
				mod_timer(&pmlmepriv->assoc_timer, jiffies + e);
			} else if (ret == 2)/* there is no need to wait for join */
			{
				_clr_fwstate_(pmlmepriv, _FW_UNDER_LINKING);
				rtw_indicate_connect23a(adapter);
			} else {
				DBG_8723A("try_to_join, but select scanning queue fail, to_roaming:%d\n", rtw_to_roaming(adapter));
				if (rtw_to_roaming(adapter) != 0) {
					if (--pmlmepriv->to_roaming == 0
						|| _SUCCESS != rtw_sitesurvey_cmd23a(adapter, &pmlmepriv->assoc_ssid, 1, NULL, 0)
					) {
						rtw_set_roaming(adapter, 0);
						rtw_free_assoc_resources23a(adapter, 1);
						rtw_indicate_disconnect23a(adapter);
					} else {
						pmlmepriv->to_join = true;
					}
				}
				_clr_fwstate_(pmlmepriv, _FW_UNDER_LINKING);
			}
		}
	}

	spin_unlock_bh(&pmlmepriv->lock);

#ifdef CONFIG_8723AU_P2P
	if (check_fwstate(pmlmepriv, _FW_LINKED) == true)
		p2p_ps_wk_cmd23a(adapter, P2P_PS_SCAN_DONE, 0);
#endif /*  CONFIG_8723AU_P2P */

	rtw_os_xmit_schedule23a(adapter);

	if(pmlmeext->sitesurvey_res.bss_cnt == 0)
		rtw_hal_sreset_reset23a(adapter);

	rtw_cfg80211_surveydone_event_callback(adapter);

}

void rtw_dummy_event_callback23a(struct rtw_adapter *adapter , u8 *pbuf)
{
}

void rtw23a_fwdbg_event_callback(struct rtw_adapter *adapter , u8 *pbuf)
{
}

static void free_scanqueue(struct	mlme_priv *pmlmepriv)
{
	struct wlan_network *pnetwork;
	struct rtw_queue *scan_queue = &pmlmepriv->scanned_queue;
	struct list_head *plist, *phead, *ptemp;

	RT_TRACE(_module_rtl871x_mlme_c_, _drv_notice_, ("+free_scanqueue\n"));
	spin_lock_bh(&scan_queue->lock);

	phead = get_list_head(scan_queue);

	list_for_each_safe(plist, ptemp, phead) {
		list_del_init(plist);
		pnetwork = container_of(plist, struct wlan_network, list);
		kfree(pnetwork);
        }

	spin_unlock_bh(&scan_queue->lock);

}

/*
*rtw_free_assoc_resources23a: the caller has to lock pmlmepriv->lock
*/
void rtw_free_assoc_resources23a(struct rtw_adapter *adapter, int lock_scanned_queue)
{
	struct wlan_network* pwlan = NULL;
	struct	mlme_priv *pmlmepriv = &adapter->mlmepriv;
	struct	sta_priv *pstapriv = &adapter->stapriv;
	struct wlan_network *tgt_network = &pmlmepriv->cur_network;

	RT_TRACE(_module_rtl871x_mlme_c_, _drv_notice_, ("+rtw_free_assoc_resources23a\n"));
	RT_TRACE(_module_rtl871x_mlme_c_, _drv_info_, ("tgt_network->network.MacAddress="MAC_FMT" ssid=%s\n",
		MAC_ARG(tgt_network->network.MacAddress), tgt_network->network.Ssid.ssid));

	if(check_fwstate(pmlmepriv, WIFI_STATION_STATE|WIFI_AP_STATE))
	{
		struct sta_info* psta;

		psta = rtw_get_stainfo23a(&adapter->stapriv, tgt_network->network.MacAddress);

		{
			spin_lock_bh(&pstapriv->sta_hash_lock);
			rtw_free_stainfo23a(adapter,  psta);
		}

		spin_unlock_bh(&pstapriv->sta_hash_lock);

	}

	if (check_fwstate(pmlmepriv, WIFI_ADHOC_STATE|WIFI_ADHOC_MASTER_STATE|WIFI_AP_STATE))
	{
		struct sta_info* psta;

		rtw_free_all_stainfo23a(adapter);

		psta = rtw_get_bcmc_stainfo23a(adapter);
		spin_lock_bh(&pstapriv->sta_hash_lock);
		rtw_free_stainfo23a(adapter, psta);
		spin_unlock_bh(&pstapriv->sta_hash_lock);

		rtw_init_bcmc_stainfo23a(adapter);
	}

	if(lock_scanned_queue)
		spin_lock_bh(&pmlmepriv->scanned_queue.lock);

	pwlan = rtw_find_network23a(&pmlmepriv->scanned_queue, tgt_network->network.MacAddress);
	if(pwlan)
		pwlan->fixed = false;
	else
		RT_TRACE(_module_rtl871x_mlme_c_,_drv_err_,("rtw_free_assoc_resources23a : pwlan== NULL\n\n"));

	if (check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE) && (adapter->stapriv.asoc_sta_count == 1))
		rtw_free_network_nolock(pmlmepriv, pwlan);

	if(lock_scanned_queue)
		spin_unlock_bh(&pmlmepriv->scanned_queue.lock);

	pmlmepriv->key_mask = 0;

}

/*
*rtw_indicate_connect23a: the caller has to lock pmlmepriv->lock
*/
void rtw_indicate_connect23a(struct rtw_adapter *padapter)
{
	struct mlme_priv	*pmlmepriv = &padapter->mlmepriv;

	RT_TRACE(_module_rtl871x_mlme_c_, _drv_err_, ("+rtw_indicate_connect23a\n"));

	pmlmepriv->to_join = false;

	if(!check_fwstate(&padapter->mlmepriv, _FW_LINKED)) {
		set_fwstate(pmlmepriv, _FW_LINKED);

		rtw_led_control(padapter, LED_CTL_LINK);

		rtw_os_indicate_connect23a(padapter);
	}

	rtw_set_roaming(padapter, 0);

	rtw_set_scan_deny(padapter, 3000);

	RT_TRACE(_module_rtl871x_mlme_c_, _drv_err_, ("-rtw_indicate_connect23a: fw_state=0x%08x\n", get_fwstate(pmlmepriv)));

}

/*
*rtw_indicate_disconnect23a: the caller has to lock pmlmepriv->lock
*/
void rtw_indicate_disconnect23a(struct rtw_adapter *padapter)
{
	struct	mlme_priv *pmlmepriv = &padapter->mlmepriv;

	RT_TRACE(_module_rtl871x_mlme_c_, _drv_err_, ("+rtw_indicate_disconnect23a\n"));

	_clr_fwstate_(pmlmepriv, _FW_UNDER_LINKING|WIFI_UNDER_WPS);

        /* DBG_8723A("clear wps when %s\n", __func__); */

	if (rtw_to_roaming(padapter) > 0)
		_clr_fwstate_(pmlmepriv, _FW_LINKED);

	if (check_fwstate(&padapter->mlmepriv, _FW_LINKED) ||
	    (rtw_to_roaming(padapter) <= 0)) {
		rtw_os_indicate_disconnect23a(padapter);

		/* set ips_deny_time to avoid enter IPS before LPS leave */
		padapter->pwrctrlpriv.ips_deny_time =
			jiffies + msecs_to_jiffies(3000);

		_clr_fwstate_(pmlmepriv, _FW_LINKED);

		rtw_led_control(padapter, LED_CTL_NO_LINK);

		rtw_clear_scan_deny(padapter);

	}

#ifdef CONFIG_8723AU_P2P
	p2p_ps_wk_cmd23a(padapter, P2P_PS_DISABLE, 1);
#endif /*  CONFIG_8723AU_P2P */

	rtw_lps_ctrl_wk_cmd23a(padapter, LPS_CTRL_DISCONNECT, 1);

}

inline void rtw_indicate_scan_done23a(struct rtw_adapter *padapter, bool aborted)
{
	rtw_os_indicate_scan_done23a(padapter, aborted);
}

void rtw_scan_abort23a(struct rtw_adapter *adapter)
{
	unsigned long start;
	struct mlme_priv *pmlmepriv = &adapter->mlmepriv;
	struct mlme_ext_priv *pmlmeext = &adapter->mlmeextpriv;

	start = jiffies;
	pmlmeext->scan_abort = true;
	while (check_fwstate(pmlmepriv, _FW_UNDER_SURVEY) &&
	       jiffies_to_msecs(jiffies - start) <= 200) {

		if (adapter->bDriverStopped || adapter->bSurpriseRemoved)
			break;

		DBG_8723A(FUNC_NDEV_FMT"fw_state = _FW_UNDER_SURVEY!\n", FUNC_NDEV_ARG(adapter->pnetdev));
		msleep(20);
	}

	if (check_fwstate(pmlmepriv, _FW_UNDER_SURVEY)) {
		if (!adapter->bDriverStopped && !adapter->bSurpriseRemoved)
			DBG_8723A(FUNC_NDEV_FMT"waiting for scan_abort time out!\n", FUNC_NDEV_ARG(adapter->pnetdev));
		rtw_indicate_scan_done23a(adapter, true);
	}
	pmlmeext->scan_abort = false;
}

static struct sta_info *rtw_joinbss_update_stainfo(struct rtw_adapter *padapter, struct wlan_network *pnetwork)
{
	int i;
	struct sta_info *bmc_sta, *psta = NULL;
	struct recv_reorder_ctrl *preorder_ctrl;
	struct sta_priv *pstapriv = &padapter->stapriv;

	psta = rtw_get_stainfo23a(pstapriv, pnetwork->network.MacAddress);
	if (psta == NULL) {
		psta = rtw_alloc_stainfo23a(pstapriv, pnetwork->network.MacAddress);
	}

	if (psta) /* update ptarget_sta */
	{
		DBG_8723A("%s\n", __func__);

		psta->aid  = pnetwork->join_res;
			psta->mac_id = 0;

		/* sta mode */
		rtw_hal_set_odm_var23a(padapter, HAL_ODM_STA_INFO, psta, true);

		/* security related */
		if (padapter->securitypriv.dot11AuthAlgrthm == dot11AuthAlgrthm_8021X)
		{
			padapter->securitypriv.binstallGrpkey = false;
			padapter->securitypriv.busetkipkey = false;
			padapter->securitypriv.bgrpkey_handshake = false;

			psta->ieee8021x_blocked = true;
			psta->dot118021XPrivacy = padapter->securitypriv.dot11PrivacyAlgrthm;

			memset((u8 *)&psta->dot118021x_UncstKey, 0, sizeof (union Keytype));

			memset((u8 *)&psta->dot11tkiprxmickey, 0, sizeof (union Keytype));
			memset((u8 *)&psta->dot11tkiptxmickey, 0, sizeof (union Keytype));

			memset((u8 *)&psta->dot11txpn, 0, sizeof (union pn48));
			memset((u8 *)&psta->dot11rxpn, 0, sizeof (union pn48));
		}

		/*	Commented by Albert 2012/07/21 */
		/*	When doing the WPS, the wps_ie_len won't equal to 0 */
		/*	And the Wi-Fi driver shouldn't allow the data packet to be tramsmitted. */
		if (padapter->securitypriv.wps_ie_len != 0)
		{
			psta->ieee8021x_blocked = true;
			padapter->securitypriv.wps_ie_len = 0;
		}

		/* for A-MPDU Rx reordering buffer control for bmc_sta & sta_info */
		/* if A-MPDU Rx is enabled, reseting  rx_ordering_ctrl wstart_b(indicate_seq) to default value = 0xffff */
		/* todo: check if AP can send A-MPDU packets */
		for (i = 0; i < 16 ; i++)
		{
			/* preorder_ctrl = &precvpriv->recvreorder_ctrl[i]; */
			preorder_ctrl = &psta->recvreorder_ctrl[i];
			preorder_ctrl->enable = false;
			preorder_ctrl->indicate_seq = 0xffff;
			preorder_ctrl->wend_b = 0xffff;
			preorder_ctrl->wsize_b = 64;/* max_ampdu_sz; ex. 32(kbytes) -> wsize_b = 32 */
		}

		bmc_sta = rtw_get_bcmc_stainfo23a(padapter);
		if (bmc_sta)
		{
			for (i = 0; i < 16 ; i++)
			{
				/* preorder_ctrl = &precvpriv->recvreorder_ctrl[i]; */
				preorder_ctrl = &bmc_sta->recvreorder_ctrl[i];
				preorder_ctrl->enable = false;
				preorder_ctrl->indicate_seq = 0xffff;
				preorder_ctrl->wend_b = 0xffff;
				preorder_ctrl->wsize_b = 64;/* max_ampdu_sz; ex. 32(kbytes) -> wsize_b = 32 */
			}
		}

		/* misc. */
		update_sta_info23a(padapter, psta);

	}

	return psta;
}

/* pnetwork : returns from rtw23a_joinbss_event_cb */
/* ptarget_wlan: found from scanned_queue */
static void rtw_joinbss_update_network23a(struct rtw_adapter *padapter, struct wlan_network *ptarget_wlan, struct wlan_network  *pnetwork)
{
	struct mlme_priv *pmlmepriv = &padapter->mlmepriv;
	struct wlan_network *cur_network = &pmlmepriv->cur_network;

	DBG_8723A("%s\n", __func__);

	RT_TRACE(_module_rtl871x_mlme_c_, _drv_info_, ("\nfw_state:%x, BSSID:"MAC_FMT"\n"
		, get_fwstate(pmlmepriv), MAC_ARG(pnetwork->network.MacAddress)));

	/*  why not use ptarget_wlan?? */
	memcpy(&cur_network->network, &pnetwork->network, pnetwork->network.Length);
	/*  some IEs in pnetwork is wrong, so we should use ptarget_wlan IEs */
	cur_network->network.IELength = ptarget_wlan->network.IELength;
	memcpy(&cur_network->network.IEs[0], &ptarget_wlan->network.IEs[0], MAX_IE_SZ);

	cur_network->aid = pnetwork->join_res;

	rtw_set_signal_stat_timer(&padapter->recvpriv);
	padapter->recvpriv.signal_strength = ptarget_wlan->network.PhyInfo.SignalStrength;
	padapter->recvpriv.signal_qual = ptarget_wlan->network.PhyInfo.SignalQuality;
	/* the ptarget_wlan->network.Rssi is raw data, we use ptarget_wlan->network.PhyInfo.SignalStrength instead (has scaled) */
	padapter->recvpriv.rssi = translate_percentage_to_dbm(ptarget_wlan->network.PhyInfo.SignalStrength);
	DBG_8723A("%s signal_strength:%3u, rssi:%3d, signal_qual:%3u\n",
		  __func__, padapter->recvpriv.signal_strength,
		  padapter->recvpriv.rssi, padapter->recvpriv.signal_qual);
	rtw_set_signal_stat_timer(&padapter->recvpriv);

	/* update fw_state will clr _FW_UNDER_LINKING here indirectly */
	switch (pnetwork->network.InfrastructureMode) {
	case Ndis802_11Infrastructure:
		if (pmlmepriv->fw_state&WIFI_UNDER_WPS)
			pmlmepriv->fw_state = WIFI_STATION_STATE|WIFI_UNDER_WPS;
		else
			pmlmepriv->fw_state = WIFI_STATION_STATE;
		break;
	case Ndis802_11IBSS:
		pmlmepriv->fw_state = WIFI_ADHOC_STATE;
		break;
	default:
		pmlmepriv->fw_state = WIFI_NULL_STATE;
		RT_TRACE(_module_rtl871x_mlme_c_, _drv_err_, ("Invalid network_mode\n"));
		break;
	}

	rtw_update_protection23a(padapter, (cur_network->network.IEs) + sizeof (struct ndis_802_11_fixed_ies),
									(cur_network->network.IELength));

	rtw_update_ht_cap23a(padapter, cur_network->network.IEs, cur_network->network.IELength);
}

/* Notes: the fucntion could be > passive_level (the same context as Rx tasklet) */
/* pnetwork : returns from rtw23a_joinbss_event_cb */
/* ptarget_wlan: found from scanned_queue */
/* if join_res > 0, for (fw_state==WIFI_STATION_STATE), we check if  "ptarget_sta" & "ptarget_wlan" exist. */
/* if join_res > 0, for (fw_state==WIFI_ADHOC_STATE), we only check if "ptarget_wlan" exist. */
/* if join_res > 0, update "cur_network->network" from "pnetwork->network" if (ptarget_wlan !=NULL). */

void rtw_joinbss_event_prehandle23a(struct rtw_adapter *adapter, u8 *pbuf)
{
	static u8 retry=0;
	struct sta_info *ptarget_sta= NULL, *pcur_sta = NULL;
	struct	sta_priv *pstapriv = &adapter->stapriv;
	struct	mlme_priv *pmlmepriv = &adapter->mlmepriv;
	struct wlan_network	*pnetwork	= (struct wlan_network *)pbuf;
	struct wlan_network *cur_network = &pmlmepriv->cur_network;
	struct wlan_network	*pcur_wlan = NULL, *ptarget_wlan = NULL;
	unsigned int		the_same_macaddr = false;

	RT_TRACE(_module_rtl871x_mlme_c_,_drv_info_,("joinbss event call back received with res=%d\n", pnetwork->join_res));

	rtw_get_encrypt_decrypt_from_registrypriv23a(adapter);

	if (pmlmepriv->assoc_ssid.ssid_len == 0) {
		RT_TRACE(_module_rtl871x_mlme_c_,_drv_err_,("@@@@@   joinbss event call back  for Any SSid\n"));
	} else {
		RT_TRACE(_module_rtl871x_mlme_c_,_drv_err_,
			 ("@@@@@   rtw23a_joinbss_event_cb for SSid:%s\n",
			  pmlmepriv->assoc_ssid.ssid));
	}

	if (ether_addr_equal(pnetwork->network.MacAddress,
			     cur_network->network.MacAddress))
		the_same_macaddr = true;
	else
		the_same_macaddr = false;

	pnetwork->network.Length = get_wlan_bssid_ex_sz(&pnetwork->network);
	if(pnetwork->network.Length > sizeof(struct wlan_bssid_ex))
	{
		RT_TRACE(_module_rtl871x_mlme_c_,_drv_err_,("\n\n ***joinbss_evt_callback return a wrong bss ***\n\n"));
		return;
	}

	spin_lock_bh(&pmlmepriv->lock);

	RT_TRACE(_module_rtl871x_mlme_c_,_drv_info_,("\n rtw23a_joinbss_event_cb !! _enter_critical\n"));

	if(pnetwork->join_res > 0)
	{
		spin_lock_bh(&pmlmepriv->scanned_queue.lock);
		retry = 0;
		if (check_fwstate(pmlmepriv,_FW_UNDER_LINKING))
		{
			/* s1. find ptarget_wlan */
			if(check_fwstate(pmlmepriv, _FW_LINKED))
			{
				if(the_same_macaddr == true)
				{
					ptarget_wlan = rtw_find_network23a(&pmlmepriv->scanned_queue, cur_network->network.MacAddress);
				}
				else
				{
					pcur_wlan = rtw_find_network23a(&pmlmepriv->scanned_queue, cur_network->network.MacAddress);
					if(pcur_wlan)	pcur_wlan->fixed = false;

					pcur_sta = rtw_get_stainfo23a(pstapriv, cur_network->network.MacAddress);
					if(pcur_sta) {
						spin_lock_bh(&pstapriv->sta_hash_lock);
						rtw_free_stainfo23a(adapter,  pcur_sta);
						spin_unlock_bh(&pstapriv->sta_hash_lock);
					}

					ptarget_wlan = rtw_find_network23a(&pmlmepriv->scanned_queue, pnetwork->network.MacAddress);
					if(check_fwstate(pmlmepriv, WIFI_STATION_STATE) == true) {
						if(ptarget_wlan)	ptarget_wlan->fixed = true;
					}
				}

			}
			else
			{
				ptarget_wlan = rtw_find_network23a(&pmlmepriv->scanned_queue, pnetwork->network.MacAddress);
				if(check_fwstate(pmlmepriv, WIFI_STATION_STATE) == true) {
					if(ptarget_wlan)	ptarget_wlan->fixed = true;
				}
			}

			/* s2. update cur_network */
			if(ptarget_wlan)
			{
				rtw_joinbss_update_network23a(adapter, ptarget_wlan, pnetwork);
			}
			else
			{
				RT_TRACE(_module_rtl871x_mlme_c_,_drv_err_,("Can't find ptarget_wlan when joinbss_event callback\n"));
				spin_unlock_bh(&pmlmepriv->scanned_queue.lock);
				goto ignore_joinbss_callback;
			}

			/* s3. find ptarget_sta & update ptarget_sta after update cur_network only for station mode */
			if(check_fwstate(pmlmepriv, WIFI_STATION_STATE) == true)
			{
				ptarget_sta = rtw_joinbss_update_stainfo(adapter, pnetwork);
				if(ptarget_sta==NULL)
				{
					RT_TRACE(_module_rtl871x_mlme_c_,_drv_err_,("Can't update stainfo when joinbss_event callback\n"));
					spin_unlock_bh(&pmlmepriv->scanned_queue.lock);
					goto ignore_joinbss_callback;
				}
			}

			/* s4. indicate connect */
			if (check_fwstate(pmlmepriv, WIFI_STATION_STATE) == true)
			{
				rtw_indicate_connect23a(adapter);
			} else {
					/* adhoc mode will rtw_indicate_connect23a when rtw_stassoc_event_callback23a */
				RT_TRACE(_module_rtl871x_mlme_c_,_drv_info_,("adhoc mode, fw_state:%x", get_fwstate(pmlmepriv)));
			}

			/* s5. Cancle assoc_timer */
			del_timer_sync(&pmlmepriv->assoc_timer);

			RT_TRACE(_module_rtl871x_mlme_c_,_drv_info_,("Cancle assoc_timer\n"));
		} else {
			RT_TRACE(_module_rtl871x_mlme_c_, _drv_err_,
				 ("rtw23a_joinbss_event_cb err: fw_state:%x",
				 get_fwstate(pmlmepriv)));
			spin_unlock_bh(&pmlmepriv->scanned_queue.lock);
			goto ignore_joinbss_callback;
		}
		spin_unlock_bh(&pmlmepriv->scanned_queue.lock);
	} else if(pnetwork->join_res == -4) {
		rtw_reset_securitypriv23a(adapter);
		mod_timer(&pmlmepriv->assoc_timer,
			  jiffies + msecs_to_jiffies(1));

		/* rtw_free_assoc_resources23a(adapter, 1); */

		if((check_fwstate(pmlmepriv, _FW_UNDER_LINKING))) {
			RT_TRACE(_module_rtl871x_mlme_c_, _drv_err_,
				 ("fail! clear _FW_UNDER_LINKING ^^^fw_state=%x\n",
				 get_fwstate(pmlmepriv)));
			_clr_fwstate_(pmlmepriv, _FW_UNDER_LINKING);
		}

	} else {
		/* if join_res < 0 (join fails), then try again */
		mod_timer(&pmlmepriv->assoc_timer,
			  jiffies + msecs_to_jiffies(1));
		_clr_fwstate_(pmlmepriv, _FW_UNDER_LINKING);
	}

ignore_joinbss_callback:

	spin_unlock_bh(&pmlmepriv->lock);
}

void rtw23a_joinbss_event_cb(struct rtw_adapter *adapter, u8 *pbuf)
{
	struct wlan_network	*pnetwork	= (struct wlan_network *)pbuf;

	mlmeext_joinbss_event_callback23a(adapter, pnetwork->join_res);

	rtw_os_xmit_schedule23a(adapter);

}

/* FOR AP , AD-HOC mode */
void rtw_stassoc_hw_rpt23a(struct rtw_adapter *adapter, struct sta_info *psta)
{
	u16 media_status;

	if (psta == NULL)	return;

	media_status = (psta->mac_id<<8)|1; /*   MACID|OPMODE:1 connect */
	rtw_hal_set_hwreg23a(adapter, HW_VAR_H2C_MEDIA_STATUS_RPT, (u8 *)&media_status);
}

void rtw_stassoc_event_callback23a(struct rtw_adapter *adapter, u8 *pbuf)
{
	struct sta_info *psta;
	struct mlme_priv *pmlmepriv = &adapter->mlmepriv;
	struct stassoc_event	*pstassoc	= (struct stassoc_event*)pbuf;
	struct wlan_network *cur_network = &pmlmepriv->cur_network;
	struct wlan_network	*ptarget_wlan = NULL;

	if(rtw_access_ctrl23a(adapter, pstassoc->macaddr) == false)
		return;

#ifdef CONFIG_8723AU_AP_MODE
	if(check_fwstate(pmlmepriv, WIFI_AP_STATE))
	{
		psta = rtw_get_stainfo23a(&adapter->stapriv, pstassoc->macaddr);
		if (psta) {
			/* bss_cap_update_on_sta_join23a(adapter, psta); */
			/* sta_info_update23a(adapter, psta); */
			ap_sta_info_defer_update23a(adapter, psta);

			rtw_stassoc_hw_rpt23a(adapter,psta);
		}
		return;
	}
#endif
	/* for AD-HOC mode */
	psta = rtw_get_stainfo23a(&adapter->stapriv, pstassoc->macaddr);
	if (psta != NULL) {
		/* the sta have been in sta_info_queue => do nothing */
		RT_TRACE(_module_rtl871x_mlme_c_,_drv_err_,("Error: rtw_stassoc_event_callback23a: sta has been in sta_hash_queue\n"));
		return; /* between drv has received this event before and  fw have not yet to set key to CAM_ENTRY) */
	}

	psta = rtw_alloc_stainfo23a(&adapter->stapriv, pstassoc->macaddr);
	if (psta == NULL) {
		RT_TRACE(_module_rtl871x_mlme_c_,_drv_err_,("Can't alloc sta_info when rtw_stassoc_event_callback23a\n"));
		return;
	}

	/* to do : init sta_info variable */
	psta->qos_option = 0;
	psta->mac_id = (uint)pstassoc->cam_id;
	/* psta->aid = (uint)pstassoc->cam_id; */
	DBG_8723A("%s\n",__func__);
	/* for ad-hoc mode */
	rtw_hal_set_odm_var23a(adapter,HAL_ODM_STA_INFO,psta,true);

	rtw_stassoc_hw_rpt23a(adapter,psta);

	if(adapter->securitypriv.dot11AuthAlgrthm==dot11AuthAlgrthm_8021X)
		psta->dot118021XPrivacy = adapter->securitypriv.dot11PrivacyAlgrthm;

	psta->ieee8021x_blocked = false;

	spin_lock_bh(&pmlmepriv->lock);

	if ( (check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE)==true ) ||
		(check_fwstate(pmlmepriv, WIFI_ADHOC_STATE)==true ) )
	{
		if(adapter->stapriv.asoc_sta_count== 2)
		{
			spin_lock_bh(&pmlmepriv->scanned_queue.lock);
			ptarget_wlan = rtw_find_network23a(&pmlmepriv->scanned_queue, cur_network->network.MacAddress);
			if(ptarget_wlan)	ptarget_wlan->fixed = true;
			spin_unlock_bh(&pmlmepriv->scanned_queue.lock);
			/*  a sta + bc/mc_stainfo (not Ibss_stainfo) */
			rtw_indicate_connect23a(adapter);
		}
	}

	spin_unlock_bh(&pmlmepriv->lock);

	mlmeext_sta_add_event_callback23a(adapter, psta);
}

void rtw_stadel_event_callback23a(struct rtw_adapter *adapter, u8 *pbuf)
{
	int mac_id=-1;
	struct sta_info *psta;
	struct wlan_network* pwlan = NULL;
	struct wlan_bssid_ex    *pdev_network=NULL;
	u8* pibss = NULL;
	struct	mlme_priv *pmlmepriv = &adapter->mlmepriv;
	struct	stadel_event *pstadel	= (struct stadel_event*)pbuf;
	struct	sta_priv *pstapriv = &adapter->stapriv;
	struct wlan_network *tgt_network = &pmlmepriv->cur_network;

	psta = rtw_get_stainfo23a(&adapter->stapriv, pstadel->macaddr);
	if(psta)
		mac_id = psta->mac_id;
	else
		mac_id = pstadel->mac_id;

	DBG_8723A("%s(mac_id=%d)=" MAC_FMT "\n", __func__, mac_id, MAC_ARG(pstadel->macaddr));

	if(mac_id>=0) {
		u16 media_status;
		media_status = (mac_id<<8)|0; /*   MACID|OPMODE:0 means disconnect */
		/* for STA,AP,ADHOC mode, report disconnect stauts to FW */
		rtw_hal_set_hwreg23a(adapter, HW_VAR_H2C_MEDIA_STATUS_RPT, (u8 *)&media_status);
	}

        if (check_fwstate(pmlmepriv, WIFI_AP_STATE))
        {
		return;
        }

	mlmeext_sta_del_event_callback23a(adapter);

	spin_lock_bh(&pmlmepriv->lock);

	if (check_fwstate(pmlmepriv, WIFI_STATION_STATE))
	{
		if (rtw_to_roaming(adapter) > 0)
			pmlmepriv->to_roaming--; /* this stadel_event is caused by roaming, decrease to_roaming */
		else if (rtw_to_roaming(adapter) == 0)
			rtw_set_roaming(adapter, adapter->registrypriv.max_roaming_times);
		if (*((unsigned short *)(pstadel->rsvd)) != WLAN_REASON_EXPIRATION_CHK)
			rtw_set_roaming(adapter, 0); /* don't roam */

		rtw_free_uc_swdec_pending_queue23a(adapter);

		rtw_free_assoc_resources23a(adapter, 1);
		rtw_indicate_disconnect23a(adapter);
		spin_lock_bh(&pmlmepriv->scanned_queue.lock);
		/*  remove the network entry in scanned_queue */
		pwlan = rtw_find_network23a(&pmlmepriv->scanned_queue, tgt_network->network.MacAddress);
		if (pwlan) {
			pwlan->fixed = false;
			rtw_free_network_nolock(pmlmepriv, pwlan);
		}
		spin_unlock_bh(&pmlmepriv->scanned_queue.lock);

		_rtw23a_roaming(adapter, tgt_network);
	}

	if (check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE) ||
	      check_fwstate(pmlmepriv, WIFI_ADHOC_STATE))
	{

		spin_lock_bh(&pstapriv->sta_hash_lock);
		rtw_free_stainfo23a(adapter,  psta);
		spin_unlock_bh(&pstapriv->sta_hash_lock);

		if (adapter->stapriv.asoc_sta_count == 1) /* a sta + bc/mc_stainfo (not Ibss_stainfo) */
		{
			spin_lock_bh(&pmlmepriv->scanned_queue.lock);
			/* free old ibss network */
			/* pwlan = rtw_find_network23a(&pmlmepriv->scanned_queue, pstadel->macaddr); */
			pwlan = rtw_find_network23a(&pmlmepriv->scanned_queue, tgt_network->network.MacAddress);
			if (pwlan)
			{
				pwlan->fixed = false;
				rtw_free_network_nolock(pmlmepriv, pwlan);
			}
			spin_unlock_bh(&pmlmepriv->scanned_queue.lock);
			/* re-create ibss */
			pdev_network = &adapter->registrypriv.dev_network;
			pibss = adapter->registrypriv.dev_network.MacAddress;

			memcpy(pdev_network, &tgt_network->network, get_wlan_bssid_ex_sz(&tgt_network->network));

			memset(&pdev_network->Ssid, 0,
			       sizeof(struct cfg80211_ssid));
			memcpy(&pdev_network->Ssid, &pmlmepriv->assoc_ssid,
			       sizeof(struct cfg80211_ssid));

			rtw_update_registrypriv_dev_network23a(adapter);

			rtw_generate_random_ibss23a(pibss);

			if (check_fwstate(pmlmepriv, WIFI_ADHOC_STATE))
			{
				set_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE);
				_clr_fwstate_(pmlmepriv, WIFI_ADHOC_STATE);
			}

			if (rtw_createbss_cmd23a(adapter)!= _SUCCESS)
			{

				RT_TRACE(_module_rtl871x_ioctl_set_c_, _drv_err_, ("***Error =>stadel_event_callback: rtw_createbss_cmd23a status FAIL***\n "));

			}

		}

	}

	spin_unlock_bh(&pmlmepriv->lock);

}

void rtw_cpwm_event_callback23a(struct rtw_adapter *padapter, u8 *pbuf)
{

	RT_TRACE(_module_rtl871x_mlme_c_,_drv_err_,("+rtw_cpwm_event_callback23a !!!\n"));

}

/*
* rtw23a_join_to_handler - Timeout/faliure handler for CMD JoinBss
* @adapter: pointer to _adapter structure
*/
void rtw23a_join_to_handler (unsigned long data)
{
	struct rtw_adapter *adapter = (struct rtw_adapter *)data;
	struct	mlme_priv *pmlmepriv = &adapter->mlmepriv;
	int do_join_r;

	DBG_8723A("%s, fw_state=%x\n", __func__, get_fwstate(pmlmepriv));

	if(adapter->bDriverStopped ||adapter->bSurpriseRemoved)
		return;

	spin_lock_bh(&pmlmepriv->lock);

	if (rtw_to_roaming(adapter) > 0) { /* join timeout caused by roaming */
		while(1) {
			pmlmepriv->to_roaming--;
			if (rtw_to_roaming(adapter) != 0) { /* try another */
				DBG_8723A("%s try another roaming\n", __func__);
				if (_SUCCESS!= (do_join_r = rtw_do_join23a(adapter))) {
					DBG_8723A("%s roaming do_join return %d\n", __func__ , do_join_r);
					continue;
				}
				break;
			} else {
				DBG_8723A("%s We've try roaming but fail\n", __func__);
				rtw_indicate_disconnect23a(adapter);
				break;
			}
		}
	} else {
		rtw_indicate_disconnect23a(adapter);
		free_scanqueue(pmlmepriv);/*  */

		/* indicate disconnect for the case that join_timeout and check_fwstate != FW_LINKED */
		rtw_cfg80211_indicate_disconnect(adapter);
	}

	spin_unlock_bh(&pmlmepriv->lock);

}

/*
* rtw_scan_timeout_handler23a - Timeout/Faliure handler for CMD SiteSurvey
* @data: pointer to _adapter structure
*/
void rtw_scan_timeout_handler23a(unsigned long data)
{
	struct rtw_adapter *adapter = (struct rtw_adapter *)data;
	struct	mlme_priv *pmlmepriv = &adapter->mlmepriv;

	DBG_8723A(FUNC_ADPT_FMT" fw_state =%x\n", FUNC_ADPT_ARG(adapter), get_fwstate(pmlmepriv));

	spin_lock_bh(&pmlmepriv->lock);

	_clr_fwstate_(pmlmepriv, _FW_UNDER_SURVEY);

	spin_unlock_bh(&pmlmepriv->lock);

	rtw_indicate_scan_done23a(adapter, true);
}

static void rtw_auto_scan_handler(struct rtw_adapter *padapter)
{
	struct mlme_priv *pmlmepriv = &padapter->mlmepriv;

	/* auto site survey per 60sec */
	if (pmlmepriv->scan_interval > 0) {
		pmlmepriv->scan_interval--;
		if (pmlmepriv->scan_interval == 0) {
			DBG_8723A("%s\n", __func__);
			rtw_set_802_11_bssid23a_list_scan(padapter, NULL, 0);
			pmlmepriv->scan_interval = SCAN_INTERVAL;/*  30*2 sec = 60sec */
		}
	}
}

void rtw_dynamic_check_timer_handler(unsigned long data)
{
	struct rtw_adapter *adapter = (struct rtw_adapter *)data;
	struct registry_priv *pregistrypriv = &adapter->registrypriv;

	if (adapter->hw_init_completed == false)
		goto out;

	if ((adapter->bDriverStopped == true)||(adapter->bSurpriseRemoved == true))
		goto out;

	if (adapter->net_closed == true)
		goto out;

	rtw_dynamic_chk_wk_cmd23a(adapter);

	if (pregistrypriv->wifi_spec == 1)
	{
#ifdef CONFIG_8723AU_P2P
		struct wifidirect_info *pwdinfo = &adapter->wdinfo;
		if (rtw_p2p_chk_state(pwdinfo, P2P_STATE_NONE))
#endif
		{
			/* auto site survey */
			rtw_auto_scan_handler(adapter);
		}
	}
out:
	mod_timer(&adapter->mlmepriv.dynamic_chk_timer,
		  jiffies + msecs_to_jiffies(2000));
}

inline bool rtw_is_scan_deny(struct rtw_adapter *adapter)
{
	struct mlme_priv *mlmepriv = &adapter->mlmepriv;
	return (atomic_read(&mlmepriv->set_scan_deny) != 0) ? true : false;
}

void rtw_clear_scan_deny(struct rtw_adapter *adapter)
{
	struct mlme_priv *mlmepriv = &adapter->mlmepriv;
	atomic_set(&mlmepriv->set_scan_deny, 0);
	if (0)
	DBG_8723A(FUNC_ADPT_FMT"\n", FUNC_ADPT_ARG(adapter));
}

void rtw_set_scan_deny_timer_hdl(unsigned long data)
{
	struct rtw_adapter *adapter = (struct rtw_adapter *)data;
	rtw_clear_scan_deny(adapter);
}

void rtw_set_scan_deny(struct rtw_adapter *adapter, u32 ms)
{
	struct mlme_priv *mlmepriv = &adapter->mlmepriv;

	if (0)
	DBG_8723A(FUNC_ADPT_FMT"\n", FUNC_ADPT_ARG(adapter));
	atomic_set(&mlmepriv->set_scan_deny, 1);
	mod_timer(&mlmepriv->set_scan_deny_timer,
		  jiffies + msecs_to_jiffies(ms));

}

#if defined(IEEE80211_SCAN_RESULT_EXPIRE)
#define RTW_SCAN_RESULT_EXPIRE IEEE80211_SCAN_RESULT_EXPIRE/HZ*1000 -1000 /* 3000 -1000 */
#else
#define RTW_SCAN_RESULT_EXPIRE 2000
#endif

/*
* Select a new join candidate from the original @param candidate and @param competitor
* @return true: candidate is updated
* @return false: candidate is not updated
*/
static int rtw_check_join_candidate(struct mlme_priv *pmlmepriv
	, struct wlan_network **candidate, struct wlan_network *competitor)
{
	int updated = false;
	struct rtw_adapter *adapter = container_of(pmlmepriv, struct rtw_adapter, mlmepriv);

	/* check bssid, if needed */
	if (pmlmepriv->assoc_by_bssid == true) {
		if (!ether_addr_equal(competitor->network.MacAddress,
				      pmlmepriv->assoc_bssid))
			goto exit;
	}

	/* check ssid, if needed */
	if (pmlmepriv->assoc_ssid.ssid_len) {
		if (competitor->network.Ssid.ssid_len !=
		    pmlmepriv->assoc_ssid.ssid_len ||
		    memcmp(competitor->network.Ssid.ssid,
			   pmlmepriv->assoc_ssid.ssid,
			   pmlmepriv->assoc_ssid.ssid_len))
			goto exit;
	}

	if (rtw_is_desired_network(adapter, competitor)  == false)
		goto exit;

	if (rtw_to_roaming(adapter) > 0) {
		unsigned int passed;

		passed = jiffies_to_msecs(jiffies - competitor->last_scanned);
		if (passed >= RTW_SCAN_RESULT_EXPIRE ||
		    is_same_ess(&competitor->network,
				&pmlmepriv->cur_network.network) == false)
			goto exit;
	}

	if (*candidate == NULL ||(*candidate)->network.Rssi<competitor->network.Rssi) {
		*candidate = competitor;
		updated = true;
	}

	if (updated) {
		DBG_8723A("[by_bssid:%u][assoc_ssid:%s][to_roaming:%u] new candidate: %s("MAC_FMT") rssi:%d\n",
			pmlmepriv->assoc_by_bssid,
			pmlmepriv->assoc_ssid.ssid,
			rtw_to_roaming(adapter),
			(*candidate)->network.Ssid.ssid,
			MAC_ARG((*candidate)->network.MacAddress),
			(int)(*candidate)->network.Rssi);
	}

exit:
	return updated;
}

/*
Calling context:
The caller of the sub-routine will be in critical section...

The caller must hold the following spinlock

pmlmepriv->lock

*/

int rtw_select_and_join_from_scanned_queue23a(struct mlme_priv *pmlmepriv)
{
	int ret;
	struct list_head *phead, *plist, *ptmp;
	struct rtw_adapter *adapter;
	struct rtw_queue *queue = &pmlmepriv->scanned_queue;
	struct	wlan_network	*pnetwork = NULL;
	struct	wlan_network	*candidate = NULL;

	spin_lock_bh(&pmlmepriv->scanned_queue.lock);
	phead = get_list_head(queue);
	adapter = pmlmepriv->nic_hdl;

	list_for_each_safe(plist, ptmp, phead) {
		pnetwork = container_of(plist, struct wlan_network, list);
		if (!pnetwork) {
			RT_TRACE(_module_rtl871x_mlme_c_, _drv_err_,
				 ("%s return _FAIL:(pnetwork == NULL)\n",
				  __func__));
			ret = _FAIL;
			goto exit;
		}

		rtw_check_join_candidate(pmlmepriv, &candidate, pnetwork);
	}

	if (!candidate) {
		DBG_8723A("%s: return _FAIL(candidate == NULL)\n", __func__);
		ret = _FAIL;
		goto exit;
	} else {
		DBG_8723A("%s: candidate: %s("MAC_FMT", ch:%u)\n", __func__,
			  candidate->network.Ssid.ssid,
			  MAC_ARG(candidate->network.MacAddress),
			  candidate->network.Configuration.DSConfig);
	}

	/*  check for situation of  _FW_LINKED */
	if (check_fwstate(pmlmepriv, _FW_LINKED) == true) {
		DBG_8723A("%s: _FW_LINKED while ask_for_joinbss!!!\n",
			  __func__);

		rtw_disassoc_cmd23a(adapter, 0, true);
		rtw_indicate_disconnect23a(adapter);
		rtw_free_assoc_resources23a(adapter, 0);
	}
	set_fwstate(pmlmepriv, _FW_UNDER_LINKING);
	ret = rtw_joinbss_cmd23a(adapter, candidate);

exit:
	spin_unlock_bh(&pmlmepriv->scanned_queue.lock);

	return ret;
}

int rtw_set_auth23a(struct rtw_adapter * adapter,
		 struct security_priv *psecuritypriv)
{
	struct cmd_obj* pcmd;
	struct setauth_parm *psetauthparm;
	struct cmd_priv *pcmdpriv = &adapter->cmdpriv;
	int res = _SUCCESS;

	pcmd = (struct cmd_obj *)kzalloc(sizeof(struct cmd_obj), GFP_KERNEL);
	if (!pcmd) {
		res = _FAIL;  /* try again */
		goto exit;
	}

	psetauthparm = (struct setauth_parm*)
		kzalloc(sizeof(struct setauth_parm), GFP_KERNEL);
	if (!psetauthparm) {
		kfree(pcmd);
		res = _FAIL;
		goto exit;
	}

	psetauthparm->mode = (unsigned char)psecuritypriv->dot11AuthAlgrthm;

	pcmd->cmdcode = _SetAuth_CMD_;
	pcmd->parmbuf = (unsigned char *)psetauthparm;
	pcmd->cmdsz =  (sizeof(struct setauth_parm));
	pcmd->rsp = NULL;
	pcmd->rspsz = 0;

	INIT_LIST_HEAD(&pcmd->list);

	RT_TRACE(_module_rtl871x_mlme_c_,_drv_err_,
		 ("after enqueue set_auth_cmd, auth_mode=%x\n",
		  psecuritypriv->dot11AuthAlgrthm));

	res = rtw_enqueue_cmd23a(pcmdpriv, pcmd);

exit:

	return res;
}

int rtw_set_key23a(struct rtw_adapter *adapter,
		struct security_priv *psecuritypriv, int keyid, u8 set_tx)
{
	u8 keylen;
	struct cmd_obj *pcmd;
	struct setkey_parm *psetkeyparm;
	struct cmd_priv *pcmdpriv = &adapter->cmdpriv;
	struct mlme_priv *pmlmepriv = &adapter->mlmepriv;
	int res = _SUCCESS;

	pcmd = (struct cmd_obj *)kzalloc(sizeof(struct cmd_obj), GFP_KERNEL);
	if (!pcmd) {
		res = _FAIL;  /* try again */
		goto exit;
	}
	psetkeyparm = kzalloc(sizeof(struct setkey_parm), GFP_KERNEL);
	if (!psetkeyparm) {
		kfree(pcmd);
		res = _FAIL;
		goto exit;
	}

	if (psecuritypriv->dot11AuthAlgrthm == dot11AuthAlgrthm_8021X) {
		psetkeyparm->algorithm = (unsigned char)
			psecuritypriv->dot118021XGrpPrivacy;
		RT_TRACE(_module_rtl871x_mlme_c_, _drv_err_,
			 ("\n rtw_set_key23a: psetkeyparm->algorithm = (unsigned "
			  "char)psecuritypriv->dot118021XGrpPrivacy =%d\n",
			  psetkeyparm->algorithm));
	} else {
		psetkeyparm->algorithm = (u8)psecuritypriv->dot11PrivacyAlgrthm;
		RT_TRACE(_module_rtl871x_mlme_c_, _drv_err_,
			 ("\n rtw_set_key23a: psetkeyparm->algorithm = (u8)"
			  "psecuritypriv->dot11PrivacyAlgrthm =%d\n",
			  psetkeyparm->algorithm));
	}
	psetkeyparm->keyid = (u8)keyid;/* 0~3 */
	psetkeyparm->set_tx = set_tx;
	if (is_wep_enc(psetkeyparm->algorithm))
		pmlmepriv->key_mask |= CHKBIT(psetkeyparm->keyid);

	DBG_8723A("==> rtw_set_key23a algorithm(%x), keyid(%x), key_mask(%x)\n",
		  psetkeyparm->algorithm, psetkeyparm->keyid,
		  pmlmepriv->key_mask);
	RT_TRACE(_module_rtl871x_mlme_c_, _drv_err_,
		 ("\n rtw_set_key23a: psetkeyparm->algorithm =%d psetkeyparm->"
		  "keyid = (u8)keyid =%d\n", psetkeyparm->algorithm, keyid));

	switch (psetkeyparm->algorithm) {
	case _WEP40_:
		keylen = 5;
		memcpy(&psetkeyparm->key[0],
		       &psecuritypriv->dot11DefKey[keyid].skey[0], keylen);
		break;
	case _WEP104_:
		keylen = 13;
		memcpy(&psetkeyparm->key[0],
		       &psecuritypriv->dot11DefKey[keyid].skey[0], keylen);
		break;
	case _TKIP_:
		keylen = 16;
		memcpy(&psetkeyparm->key,
		       &psecuritypriv->dot118021XGrpKey[keyid], keylen);
		psetkeyparm->grpkey = 1;
		break;
	case _AES_:
		keylen = 16;
		memcpy(&psetkeyparm->key,
		       &psecuritypriv->dot118021XGrpKey[keyid], keylen);
		psetkeyparm->grpkey = 1;
		break;
	default:
		RT_TRACE(_module_rtl871x_mlme_c_, _drv_err_,
			 ("\n rtw_set_key23a:psecuritypriv->dot11PrivacyAlgrthm = "
			  "%x (must be 1 or 2 or 4 or 5)\n",
			  psecuritypriv->dot11PrivacyAlgrthm));
		res = _FAIL;
		kfree(pcmd);
		kfree(psetkeyparm);
		goto exit;
	}

	pcmd->cmdcode = _SetKey_CMD_;
	pcmd->parmbuf = (u8 *)psetkeyparm;
	pcmd->cmdsz =  (sizeof(struct setkey_parm));
	pcmd->rsp = NULL;
	pcmd->rspsz = 0;

	INIT_LIST_HEAD(&pcmd->list);

	/* sema_init(&pcmd->cmd_sem, 0); */

	res = rtw_enqueue_cmd23a(pcmdpriv, pcmd);

exit:

	return res;
}

/* adjust IEs for rtw_joinbss_cmd23a in WMM */
int rtw_restruct_wmm_ie23a(struct rtw_adapter *adapter, u8 *in_ie,
			u8 *out_ie, uint in_len, uint initial_out_len)
{
	unsigned int ielength = 0;
	unsigned int i, j;

	i = 12; /* after the fixed IE */
	while(i < in_len) {
		ielength = initial_out_len;

		/* WMM element ID and OUI */
		if (in_ie[i] == 0xDD && in_ie[i + 2] == 0x00 &&
		    in_ie[i + 3] == 0x50 && in_ie[i + 4] == 0xF2 &&
		    in_ie[i + 5] == 0x02 && i+5 < in_len) {

			/* Append WMM IE to the last index of out_ie */
                        for (j = i; j < i + 9; j++) {
				out_ie[ielength] = in_ie[j];
				ielength++;
                        }
                        out_ie[initial_out_len + 1] = 0x07;
                        out_ie[initial_out_len + 6] = 0x00;
                        out_ie[initial_out_len + 8] = 0x00;

			break;
		}

		i += (in_ie[i + 1] + 2); /*  to the next IE element */
	}

	return ielength;
}

/*  */
/*  Ported from 8185: IsInPreAuthKeyList().
    (Renamed from SecIsInPreAuthKeyList(), 2006-10-13.) */
/*  Added by Annie, 2006-05-07. */
/*  */
/*  Search by BSSID, */
/*  Return Value: */
/*		-1	:if there is no pre-auth key in the  table */
/*		>= 0	:if there is pre-auth key, and   return the entry id */
/*  */
/*  */

static int SecIsInPMKIDList(struct rtw_adapter *Adapter, u8 *bssid)
{
	struct security_priv *psecuritypriv = &Adapter->securitypriv;
	int i = 0;

	do {
		if (psecuritypriv->PMKIDList[i].bUsed &&
                    ether_addr_equal(psecuritypriv->PMKIDList[i].Bssid, bssid)) {
			break;
		} else {
			i++;
			/* continue; */
		}
	} while(i < NUM_PMKID_CACHE);

	if (i == NUM_PMKID_CACHE) {
		i = -1;/*  Could not find. */
	} else {
		/*  There is one Pre-Authentication Key for
		    the specific BSSID. */
	}

	return i;
}

/*  */
/*  Check the RSN IE length */
/*  If the RSN IE length <= 20, the RSN IE didn't include
    the PMKID information */
/*  0-11th element in the array are the fixed IE */
/*  12th element in the array is the IE */
/*  13th element in the array is the IE length */
/*  */

static int rtw_append_pmkid(struct rtw_adapter *Adapter, int iEntry,
			    u8 *ie, uint ie_len)
{
	struct security_priv *psecuritypriv = &Adapter->securitypriv;

	if (ie[13] <= 20) {
		/*  The RSN IE didn't include the PMK ID,
		    append the PMK information */
			ie[ie_len] = 1;
			ie_len++;
			ie[ie_len] = 0;	/* PMKID count = 0x0100 */
			ie_len++;
			memcpy(&ie[ie_len],
			       &psecuritypriv->PMKIDList[iEntry].PMKID, 16);

			ie_len += 16;
			ie[13] += 18;/* PMKID length = 2+16 */
	}
	return ie_len;
}
int rtw_restruct_sec_ie23a(struct rtw_adapter *adapter, u8 *in_ie, u8 *out_ie,
			uint in_len)
{
	u8 authmode;
	uint ielength;
	int iEntry;
	struct mlme_priv *pmlmepriv = &adapter->mlmepriv;
	struct security_priv *psecuritypriv = &adapter->securitypriv;
	uint ndisauthmode = psecuritypriv->ndisauthtype;
	uint ndissecuritytype = psecuritypriv->ndisencryptstatus;

	RT_TRACE(_module_rtl871x_mlme_c_, _drv_notice_,
		 ("+rtw_restruct_sec_ie23a: ndisauthmode=%d ndissecuritytype=%d\n",
		  ndisauthmode, ndissecuritytype));

	/* copy fixed ie only */
	memcpy(out_ie, in_ie, 12);
	ielength = 12;
	if ((ndisauthmode==Ndis802_11AuthModeWPA) ||
	    (ndisauthmode==Ndis802_11AuthModeWPAPSK))
		authmode=_WPA_IE_ID_;
	if ((ndisauthmode==Ndis802_11AuthModeWPA2) ||
	    (ndisauthmode==Ndis802_11AuthModeWPA2PSK))
		authmode=_WPA2_IE_ID_;

	if (check_fwstate(pmlmepriv, WIFI_UNDER_WPS)) {
		memcpy(out_ie + ielength, psecuritypriv->wps_ie,
		       psecuritypriv->wps_ie_len);

		ielength += psecuritypriv->wps_ie_len;
	} else if ((authmode==_WPA_IE_ID_) || (authmode==_WPA2_IE_ID_)) {
		/* copy RSN or SSN */
		memcpy(&out_ie[ielength], &psecuritypriv->supplicant_ie[0],
		       psecuritypriv->supplicant_ie[1] + 2);
		ielength += psecuritypriv->supplicant_ie[1] + 2;
		rtw_report_sec_ie23a(adapter, authmode,
				  psecuritypriv->supplicant_ie);
	}

	iEntry = SecIsInPMKIDList(adapter, pmlmepriv->assoc_bssid);
	if (iEntry < 0)	{
		return ielength;
	} else {
		if (authmode == _WPA2_IE_ID_) {
			ielength=rtw_append_pmkid(adapter, iEntry,
						  out_ie, ielength);
		}
	}

	return ielength;
}

void rtw_init_registrypriv_dev_network23a(struct rtw_adapter* adapter)
{
	struct registry_priv* pregistrypriv = &adapter->registrypriv;
	struct eeprom_priv* peepriv = &adapter->eeprompriv;
	struct wlan_bssid_ex    *pdev_network = &pregistrypriv->dev_network;
	u8 *myhwaddr = myid(peepriv);

	ether_addr_copy(pdev_network->MacAddress, myhwaddr);

	memcpy(&pdev_network->Ssid, &pregistrypriv->ssid,
	       sizeof(struct cfg80211_ssid));

	pdev_network->Configuration.Length=sizeof(struct ndis_802_11_config);
	pdev_network->Configuration.BeaconPeriod = 100;
	pdev_network->Configuration.FHConfig.Length = 0;
	pdev_network->Configuration.FHConfig.HopPattern = 0;
	pdev_network->Configuration.FHConfig.HopSet = 0;
	pdev_network->Configuration.FHConfig.DwellTime = 0;

}

void rtw_update_registrypriv_dev_network23a(struct rtw_adapter* adapter)
{
	int sz = 0;
	struct registry_priv* pregistrypriv = &adapter->registrypriv;
	struct wlan_bssid_ex *pdev_network = &pregistrypriv->dev_network;
	struct security_priv *psecuritypriv = &adapter->securitypriv;
	struct wlan_network *cur_network = &adapter->mlmepriv.cur_network;
	/* struct	xmit_priv	*pxmitpriv = &adapter->xmitpriv; */

	pdev_network->Privacy =
		(psecuritypriv->dot11PrivacyAlgrthm > 0 ? 1 : 0);

	pdev_network->Rssi = 0;

	switch (pregistrypriv->wireless_mode)
	{
	case WIRELESS_11B:
		pdev_network->NetworkTypeInUse = Ndis802_11DS;
		break;
	case WIRELESS_11G:
	case WIRELESS_11BG:
	case WIRELESS_11_24N:
	case WIRELESS_11G_24N:
	case WIRELESS_11BG_24N:
		pdev_network->NetworkTypeInUse = Ndis802_11OFDM24;
		break;
	case WIRELESS_11A:
	case WIRELESS_11A_5N:
		pdev_network->NetworkTypeInUse = Ndis802_11OFDM5;
		break;
	case WIRELESS_11ABGN:
		if (pregistrypriv->channel > 14)
			pdev_network->NetworkTypeInUse = Ndis802_11OFDM5;
		else
			pdev_network->NetworkTypeInUse = Ndis802_11OFDM24;
		break;
	default :
		/*  TODO */
		break;
	}

	pdev_network->Configuration.DSConfig = pregistrypriv->channel;
	RT_TRACE(_module_rtl871x_mlme_c_, _drv_info_,
		 ("pregistrypriv->channel =%d, pdev_network->Configuration."
		  "DSConfig = 0x%x\n", pregistrypriv->channel,
		  pdev_network->Configuration.DSConfig));

	if (cur_network->network.InfrastructureMode == Ndis802_11IBSS)
		pdev_network->Configuration.ATIMWindow = 0;

	pdev_network->InfrastructureMode =
		cur_network->network.InfrastructureMode;

	/*  1. Supported rates */
	/*  2. IE */

	sz = rtw_generate_ie23a(pregistrypriv);

	pdev_network->IELength = sz;

	pdev_network->Length =
		get_wlan_bssid_ex_sz((struct wlan_bssid_ex *)pdev_network);

	/* notes: translate IELength & Length after assign the
	   Length to cmdsz in createbss_cmd(); */
	/* pdev_network->IELength = cpu_to_le32(sz); */

}

void rtw_get_encrypt_decrypt_from_registrypriv23a(struct rtw_adapter* adapter)
{

}

/* the fucntion is at passive_level */
void rtw_joinbss_reset23a(struct rtw_adapter *padapter)
{
	u8 threshold;
	struct mlme_priv *pmlmepriv = &padapter->mlmepriv;
	struct ht_priv *phtpriv = &pmlmepriv->htpriv;

	/* todo: if you want to do something io/reg/hw setting
	   before join_bss, please add code here */

	pmlmepriv->num_FortyMHzIntolerant = 0;

	pmlmepriv->num_sta_no_ht = 0;

	phtpriv->ampdu_enable = false;/* reset to disabled */

	/*  TH = 1 => means that invalidate usb rx aggregation */
	/*  TH = 0 => means that validate usb rx aggregation, use init value. */
	if (phtpriv->ht_option) {
		if (padapter->registrypriv.wifi_spec == 1)
			threshold = 1;
		else
			threshold = 0;
		rtw_hal_set_hwreg23a(padapter, HW_VAR_RXDMA_AGG_PG_TH,
				  (u8 *)(&threshold));
	} else {
		threshold = 1;
		rtw_hal_set_hwreg23a(padapter, HW_VAR_RXDMA_AGG_PG_TH,
				  (u8 *)(&threshold));
	}
}

/* the fucntion is >= passive_level */
unsigned int rtw_restructure_ht_ie23a(struct rtw_adapter *padapter, u8 *in_ie,
				   u8 *out_ie, uint in_len, uint *pout_len)
{
	u32 ielen, out_len;
	int max_rx_ampdu_factor;
	unsigned char *p, *pframe;
	struct ieee80211_ht_cap ht_capie;
	unsigned char WMM_IE[] = {0x00, 0x50, 0xf2, 0x02, 0x00, 0x01, 0x00};
	struct mlme_priv *pmlmepriv = &padapter->mlmepriv;
	struct qos_priv *pqospriv = &pmlmepriv->qospriv;
	struct ht_priv *phtpriv = &pmlmepriv->htpriv;

	phtpriv->ht_option = false;

	p = rtw_get_ie23a(in_ie + 12, _HT_CAPABILITY_IE_, &ielen, in_len - 12);

	if (p && ielen > 0) {
		u32 rx_packet_offset, max_recvbuf_sz;
		if (pqospriv->qos_option == 0) {
			out_len = *pout_len;
			pframe = rtw_set_ie23a(out_ie + out_len,
					    _VENDOR_SPECIFIC_IE_,
					    _WMM_IE_Length_, WMM_IE, pout_len);

			pqospriv->qos_option = 1;
		}

		out_len = *pout_len;

		memset(&ht_capie, 0, sizeof(struct ieee80211_ht_cap));

		ht_capie.cap_info = IEEE80211_HT_CAP_SUP_WIDTH_20_40 |
			IEEE80211_HT_CAP_SGI_20 | IEEE80211_HT_CAP_SGI_40 |
			IEEE80211_HT_CAP_TX_STBC | IEEE80211_HT_CAP_DSSSCCK40;

		rtw_hal_get_def_var23a(padapter, HAL_DEF_RX_PACKET_OFFSET,
				    &rx_packet_offset);
		rtw_hal_get_def_var23a(padapter, HAL_DEF_MAX_RECVBUF_SZ,
				    &max_recvbuf_sz);

		rtw_hal_get_def_var23a(padapter, HW_VAR_MAX_RX_AMPDU_FACTOR,
				    &max_rx_ampdu_factor);
		ht_capie.ampdu_params_info = max_rx_ampdu_factor & 0x03;

		if (padapter->securitypriv.dot11PrivacyAlgrthm == _AES_)
			ht_capie.ampdu_params_info |=
				(IEEE80211_HT_AMPDU_PARM_DENSITY& (0x07 << 2));
		else
			ht_capie.ampdu_params_info |=
				(IEEE80211_HT_AMPDU_PARM_DENSITY & 0x00);

		pframe = rtw_set_ie23a(out_ie + out_len, _HT_CAPABILITY_IE_,
				    sizeof(struct ieee80211_ht_cap),
				    (unsigned char*)&ht_capie, pout_len);

		phtpriv->ht_option = true;

		p = rtw_get_ie23a(in_ie + 12, _HT_ADD_INFO_IE_, &ielen, in_len-12);
		if (p && (ielen == sizeof(struct ieee80211_ht_addt_info))) {
			out_len = *pout_len;
			pframe = rtw_set_ie23a(out_ie + out_len, _HT_ADD_INFO_IE_,
					    ielen, p + 2 , pout_len);
		}
	}

	return phtpriv->ht_option;
}

/* the fucntion is > passive_level (in critical_section) */
void rtw_update_ht_cap23a(struct rtw_adapter *padapter, u8 *pie, uint ie_len)
{
	u8 *p, max_ampdu_sz;
	int len;
	/* struct sta_info *bmc_sta, *psta; */
	struct ieee80211_ht_cap *pht_capie;
	struct ieee80211_ht_addt_info *pht_addtinfo;
	/* struct recv_reorder_ctrl *preorder_ctrl; */
	struct mlme_priv	*pmlmepriv = &padapter->mlmepriv;
	struct ht_priv		*phtpriv = &pmlmepriv->htpriv;
	/* struct recv_priv *precvpriv = &padapter->recvpriv; */
	struct registry_priv *pregistrypriv = &padapter->registrypriv;
	/* struct wlan_network *pcur_network = &pmlmepriv->cur_network;; */
	struct mlme_ext_priv	*pmlmeext = &padapter->mlmeextpriv;
	struct mlme_ext_info *pmlmeinfo = &pmlmeext->mlmext_info;

	if (!phtpriv->ht_option)
		return;

	if ((!pmlmeinfo->HT_info_enable) || (!pmlmeinfo->HT_caps_enable))
		return;

	DBG_8723A("+rtw_update_ht_cap23a()\n");

	/* maybe needs check if ap supports rx ampdu. */
	if ((phtpriv->ampdu_enable == false) && (pregistrypriv->ampdu_enable == 1)) {
		if (pregistrypriv->wifi_spec == 1)
			phtpriv->ampdu_enable = false;
		else
			phtpriv->ampdu_enable = true;
	} else if (pregistrypriv->ampdu_enable == 2) {
		phtpriv->ampdu_enable = true;
	}

	/* check Max Rx A-MPDU Size */
	len = 0;
	p = rtw_get_ie23a(pie+sizeof (struct ndis_802_11_fixed_ies), _HT_CAPABILITY_IE_, &len, ie_len-sizeof (struct ndis_802_11_fixed_ies));
	if (p && len > 0) {
		pht_capie = (struct ieee80211_ht_cap *)(p+2);
		max_ampdu_sz = (pht_capie->ampdu_params_info & IEEE80211_HT_AMPDU_PARM_FACTOR);
		max_ampdu_sz = 1 << (max_ampdu_sz+3); /*  max_ampdu_sz (kbytes); */

		/* DBG_8723A("rtw_update_ht_cap23a(): max_ampdu_sz =%d\n", max_ampdu_sz); */
		phtpriv->rx_ampdu_maxlen = max_ampdu_sz;

	}

	len = 0;
	p = rtw_get_ie23a(pie+sizeof (struct ndis_802_11_fixed_ies), _HT_ADD_INFO_IE_, &len, ie_len-sizeof (struct ndis_802_11_fixed_ies));
	if (p && len>0)
	{
		pht_addtinfo = (struct ieee80211_ht_addt_info *)(p+2);
		/* todo: */
	}

	/* update cur_bwmode & cur_ch_offset */
	if ((pregistrypriv->cbw40_enable) &&
		(pmlmeinfo->HT_caps.u.HT_cap_element.HT_caps_info & BIT(1)) &&
		(pmlmeinfo->HT_info.infos[0] & BIT(2)))
	{
		int i;
		u8	rf_type;

		padapter->HalFunc.GetHwRegHandler(padapter, HW_VAR_RF_TYPE, (u8 *)(&rf_type));

		/* update the MCS rates */
		for (i = 0; i < 16; i++)
		{
			if ((rf_type == RF_1T1R) || (rf_type == RF_1T2R))
				pmlmeinfo->HT_caps.u.HT_cap_element.MCS_rate[i] &= MCS_rate_1R23A[i];
			else
				pmlmeinfo->HT_caps.u.HT_cap_element.MCS_rate[i] &= MCS_rate_2R23A[i];
		}
		/* switch to the 40M Hz mode accoring to the AP */
		pmlmeext->cur_bwmode = HT_CHANNEL_WIDTH_40;
		switch ((pmlmeinfo->HT_info.infos[0] & 0x3))
		{
			case HT_EXTCHNL_OFFSET_UPPER:
				pmlmeext->cur_ch_offset = HAL_PRIME_CHNL_OFFSET_LOWER;
				break;

			case HT_EXTCHNL_OFFSET_LOWER:
				pmlmeext->cur_ch_offset = HAL_PRIME_CHNL_OFFSET_UPPER;
				break;

			default:
				pmlmeext->cur_ch_offset = HAL_PRIME_CHNL_OFFSET_DONT_CARE;
				break;
		}
	}

	/*  */
	/*  Config SM Power Save setting */
	/*  */
	pmlmeinfo->SM_PS = (pmlmeinfo->HT_caps.u.HT_cap_element.HT_caps_info & 0x0C) >> 2;
	if (pmlmeinfo->SM_PS == WLAN_HT_CAP_SM_PS_STATIC)
		DBG_8723A("%s(): WLAN_HT_CAP_SM_PS_STATIC\n", __func__);

	/*  */
	/*  Config current HT Protection mode. */
	/*  */
	pmlmeinfo->HT_protection = pmlmeinfo->HT_info.infos[1] & 0x3;
}

void rtw_issue_addbareq_cmd23a(struct rtw_adapter *padapter, struct xmit_frame *pxmitframe)
{
	u8 issued;
	int priority;
	struct sta_info *psta = NULL;
	struct ht_priv	*phtpriv;
	struct pkt_attrib *pattrib = &pxmitframe->attrib;
	s32 bmcst = is_multicast_ether_addr(pattrib->ra);

	if (bmcst || (padapter->mlmepriv.LinkDetectInfo.NumTxOkInPeriod<100))
		return;

	priority = pattrib->priority;

	if (pattrib->psta)
		psta = pattrib->psta;
	else
	{
		DBG_8723A("%s, call rtw_get_stainfo23a()\n", __func__);
		psta = rtw_get_stainfo23a(&padapter->stapriv, pattrib->ra);
	}

	if (psta == NULL)
	{
		DBG_8723A("%s, psta == NUL\n", __func__);
		return;
	}

	if (!(psta->state &_FW_LINKED))
	{
		DBG_8723A("%s, psta->state(0x%x) != _FW_LINKED\n", __func__, psta->state);
		return;
	}

	phtpriv = &psta->htpriv;

	if ((phtpriv->ht_option == true) && (phtpriv->ampdu_enable == true))
	{
		issued = (phtpriv->agg_enable_bitmap>>priority)&0x1;
		issued |= (phtpriv->candidate_tid_bitmap>>priority)&0x1;

		if (0 == issued)
		{
			DBG_8723A("rtw_issue_addbareq_cmd23a, p =%d\n", priority);
			psta->htpriv.candidate_tid_bitmap |= CHKBIT((u8)priority);
			rtw_addbareq_cmd23a(padapter, (u8) priority, pattrib->ra);
		}
	}
}

inline void rtw_set_roaming(struct rtw_adapter *adapter, u8 to_roaming)
{
	if (to_roaming == 0)
		adapter->mlmepriv.to_join = false;
	adapter->mlmepriv.to_roaming = to_roaming;
}

inline u8 rtw_to_roaming(struct rtw_adapter *adapter)
{
	return adapter->mlmepriv.to_roaming;
}

void rtw23a_roaming(struct rtw_adapter *padapter, struct wlan_network *tgt_network)
{
	struct mlme_priv	*pmlmepriv = &padapter->mlmepriv;

	spin_lock_bh(&pmlmepriv->lock);
	_rtw23a_roaming(padapter, tgt_network);
	spin_unlock_bh(&pmlmepriv->lock);
}
void _rtw23a_roaming(struct rtw_adapter *padapter, struct wlan_network *tgt_network)
{
	struct mlme_priv	*pmlmepriv = &padapter->mlmepriv;
	struct wlan_network *pnetwork;
	int do_join_r;

	if (tgt_network != NULL)
		pnetwork = tgt_network;
	else
		pnetwork = &pmlmepriv->cur_network;

	if (0 < rtw_to_roaming(padapter)) {
		DBG_8723A("roaming from %s("MAC_FMT"), length:%d\n",
			  pnetwork->network.Ssid.ssid,
			  MAC_ARG(pnetwork->network.MacAddress),
			  pnetwork->network.Ssid.ssid_len);
		memcpy(&pmlmepriv->assoc_ssid, &pnetwork->network.Ssid,
		       sizeof(struct cfg80211_ssid));

		pmlmepriv->assoc_by_bssid = false;

		while(1) {
			if (_SUCCESS == (do_join_r = rtw_do_join23a(padapter))) {
				break;
			} else {
				DBG_8723A("roaming do_join return %d\n", do_join_r);
				pmlmepriv->to_roaming--;

				if (0 < rtw_to_roaming(padapter)) {
					continue;
				} else {
					DBG_8723A("%s(%d) -to roaming fail, indicate_disconnect\n", __func__, __LINE__);
					rtw_indicate_disconnect23a(padapter);
					break;
				}
			}
		}
	}
}

int rtw_linked_check(struct rtw_adapter *padapter)
{
	if ((check_fwstate(&padapter->mlmepriv, WIFI_AP_STATE)) ||
	    (check_fwstate(&padapter->mlmepriv, WIFI_ADHOC_STATE|WIFI_ADHOC_MASTER_STATE))) {
		if (padapter->stapriv.asoc_sta_count > 2)
			return true;
	} else {	/* Station mode */
		if (check_fwstate(&padapter->mlmepriv, _FW_LINKED) == true)
			return true;
	}
	return false;
}
