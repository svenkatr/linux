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

#include "wil6210.h"
#include "wmi.h"

#define CHAN60G(_channel, _flags) {				\
	.band			= IEEE80211_BAND_60GHZ,		\
	.center_freq		= 56160 + (2160 * (_channel)),	\
	.hw_value		= (_channel),			\
	.flags			= (_flags),			\
	.max_antenna_gain	= 0,				\
	.max_power		= 40,				\
}

static struct ieee80211_channel wil_60ghz_channels[] = {
	CHAN60G(1, 0),
	CHAN60G(2, 0),
	CHAN60G(3, 0),
/* channel 4 not supported yet */
};

static struct ieee80211_supported_band wil_band_60ghz = {
	.channels = wil_60ghz_channels,
	.n_channels = ARRAY_SIZE(wil_60ghz_channels),
	.ht_cap = {
		.ht_supported = true,
		.cap = 0, /* TODO */
		.ampdu_factor = IEEE80211_HT_MAX_AMPDU_64K, /* TODO */
		.ampdu_density = IEEE80211_HT_MPDU_DENSITY_8, /* TODO */
		.mcs = {
				/* MCS 1..12 - SC PHY */
			.rx_mask = {0xfe, 0x1f}, /* 1..12 */
			.tx_params = IEEE80211_HT_MCS_TX_DEFINED, /* TODO */
		},
	},
};

static const struct ieee80211_txrx_stypes
wil_mgmt_stypes[NUM_NL80211_IFTYPES] = {
	[NL80211_IFTYPE_STATION] = {
		.tx = BIT(IEEE80211_STYPE_ACTION >> 4) |
		BIT(IEEE80211_STYPE_PROBE_RESP >> 4),
		.rx = BIT(IEEE80211_STYPE_ACTION >> 4) |
		BIT(IEEE80211_STYPE_PROBE_REQ >> 4)
	},
	[NL80211_IFTYPE_AP] = {
		.tx = BIT(IEEE80211_STYPE_ACTION >> 4) |
		BIT(IEEE80211_STYPE_PROBE_RESP >> 4),
		.rx = BIT(IEEE80211_STYPE_ACTION >> 4) |
		BIT(IEEE80211_STYPE_PROBE_REQ >> 4)
	},
	[NL80211_IFTYPE_P2P_CLIENT] = {
		.tx = BIT(IEEE80211_STYPE_ACTION >> 4) |
		BIT(IEEE80211_STYPE_PROBE_RESP >> 4),
		.rx = BIT(IEEE80211_STYPE_ACTION >> 4) |
		BIT(IEEE80211_STYPE_PROBE_REQ >> 4)
	},
	[NL80211_IFTYPE_P2P_GO] = {
		.tx = BIT(IEEE80211_STYPE_ACTION >> 4) |
		BIT(IEEE80211_STYPE_PROBE_RESP >> 4),
		.rx = BIT(IEEE80211_STYPE_ACTION >> 4) |
		BIT(IEEE80211_STYPE_PROBE_REQ >> 4)
	},
};

static const u32 wil_cipher_suites[] = {
	WLAN_CIPHER_SUITE_GCMP,
};

int wil_iftype_nl2wmi(enum nl80211_iftype type)
{
	static const struct {
		enum nl80211_iftype nl;
		enum wmi_network_type wmi;
	} __nl2wmi[] = {
		{NL80211_IFTYPE_ADHOC,		WMI_NETTYPE_ADHOC},
		{NL80211_IFTYPE_STATION,	WMI_NETTYPE_INFRA},
		{NL80211_IFTYPE_AP,		WMI_NETTYPE_AP},
		{NL80211_IFTYPE_P2P_CLIENT,	WMI_NETTYPE_P2P},
		{NL80211_IFTYPE_P2P_GO,		WMI_NETTYPE_P2P},
		{NL80211_IFTYPE_MONITOR,	WMI_NETTYPE_ADHOC}, /* FIXME */
	};
	uint i;

	for (i = 0; i < ARRAY_SIZE(__nl2wmi); i++) {
		if (__nl2wmi[i].nl == type)
			return __nl2wmi[i].wmi;
	}

	return -EOPNOTSUPP;
}

static int wil_cid_fill_sinfo(struct wil6210_priv *wil, int cid,
			      struct station_info *sinfo)
{
	struct wmi_notify_req_cmd cmd = {
		.cid = cid,
		.interval_usec = 0,
	};
	struct {
		struct wil6210_mbox_hdr_wmi wmi;
		struct wmi_notify_req_done_event evt;
	} __packed reply;
	struct wil_net_stats *stats = &wil->sta[cid].stats;
	int rc;

	rc = wmi_call(wil, WMI_NOTIFY_REQ_CMDID, &cmd, sizeof(cmd),
		      WMI_NOTIFY_REQ_DONE_EVENTID, &reply, sizeof(reply), 20);
	if (rc)
		return rc;

	wil_dbg_wmi(wil, "Link status for CID %d: {\n"
		    "  MCS %d TSF 0x%016llx\n"
		    "  BF status 0x%08x SNR 0x%08x SQI %d%%\n"
		    "  Tx Tpt %d goodput %d Rx goodput %d\n"
		    "  Sectors(rx:tx) my %d:%d peer %d:%d\n""}\n",
		    cid, le16_to_cpu(reply.evt.bf_mcs),
		    le64_to_cpu(reply.evt.tsf), reply.evt.status,
		    le32_to_cpu(reply.evt.snr_val),
		    reply.evt.sqi,
		    le32_to_cpu(reply.evt.tx_tpt),
		    le32_to_cpu(reply.evt.tx_goodput),
		    le32_to_cpu(reply.evt.rx_goodput),
		    le16_to_cpu(reply.evt.my_rx_sector),
		    le16_to_cpu(reply.evt.my_tx_sector),
		    le16_to_cpu(reply.evt.other_rx_sector),
		    le16_to_cpu(reply.evt.other_tx_sector));

	sinfo->generation = wil->sinfo_gen;

	sinfo->filled = STATION_INFO_RX_BYTES |
			STATION_INFO_TX_BYTES |
			STATION_INFO_RX_PACKETS |
			STATION_INFO_TX_PACKETS |
			STATION_INFO_RX_BITRATE |
			STATION_INFO_TX_BITRATE |
			STATION_INFO_RX_DROP_MISC |
			STATION_INFO_TX_FAILED;

	sinfo->txrate.flags = RATE_INFO_FLAGS_MCS | RATE_INFO_FLAGS_60G;
	sinfo->txrate.mcs = le16_to_cpu(reply.evt.bf_mcs);
	sinfo->rxrate.flags = RATE_INFO_FLAGS_MCS | RATE_INFO_FLAGS_60G;
	sinfo->rxrate.mcs = stats->last_mcs_rx;
	sinfo->rx_bytes = stats->rx_bytes;
	sinfo->rx_packets = stats->rx_packets;
	sinfo->rx_dropped_misc = stats->rx_dropped;
	sinfo->tx_bytes = stats->tx_bytes;
	sinfo->tx_packets = stats->tx_packets;
	sinfo->tx_failed = stats->tx_errors;

	if (test_bit(wil_status_fwconnected, &wil->status)) {
		sinfo->filled |= STATION_INFO_SIGNAL;
		sinfo->signal = reply.evt.sqi;
	}

	return rc;
}

static int wil_cfg80211_get_station(struct wiphy *wiphy,
				    struct net_device *ndev,
				    u8 *mac, struct station_info *sinfo)
{
	struct wil6210_priv *wil = wiphy_to_wil(wiphy);
	int rc;

	int cid = wil_find_cid(wil, mac);

	wil_dbg_misc(wil, "%s(%pM) CID %d\n", __func__, mac, cid);
	if (cid < 0)
		return cid;

	rc = wil_cid_fill_sinfo(wil, cid, sinfo);

	return rc;
}

/*
 * Find @idx-th active STA for station dump.
 */
static int wil_find_cid_by_idx(struct wil6210_priv *wil, int idx)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(wil->sta); i++) {
		if (wil->sta[i].status == wil_sta_unused)
			continue;
		if (idx == 0)
			return i;
		idx--;
	}

	return -ENOENT;
}

static int wil_cfg80211_dump_station(struct wiphy *wiphy,
				     struct net_device *dev, int idx,
				     u8 *mac, struct station_info *sinfo)
{
	struct wil6210_priv *wil = wiphy_to_wil(wiphy);
	int rc;
	int cid = wil_find_cid_by_idx(wil, idx);

	if (cid < 0)
		return -ENOENT;

	memcpy(mac, wil->sta[cid].addr, ETH_ALEN);
	wil_dbg_misc(wil, "%s(%pM) CID %d\n", __func__, mac, cid);

	rc = wil_cid_fill_sinfo(wil, cid, sinfo);

	return rc;
}

static int wil_cfg80211_change_iface(struct wiphy *wiphy,
				     struct net_device *ndev,
				     enum nl80211_iftype type, u32 *flags,
				     struct vif_params *params)
{
	struct wil6210_priv *wil = wiphy_to_wil(wiphy);
	struct wireless_dev *wdev = wil->wdev;

	switch (type) {
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_AP:
	case NL80211_IFTYPE_P2P_CLIENT:
	case NL80211_IFTYPE_P2P_GO:
		break;
	case NL80211_IFTYPE_MONITOR:
		if (flags)
			wil->monitor_flags = *flags;
		else
			wil->monitor_flags = 0;

		break;
	default:
		return -EOPNOTSUPP;
	}

	wdev->iftype = type;

	return 0;
}

static int wil_cfg80211_scan(struct wiphy *wiphy,
			     struct cfg80211_scan_request *request)
{
	struct wil6210_priv *wil = wiphy_to_wil(wiphy);
	struct wireless_dev *wdev = wil->wdev;
	struct {
		struct wmi_start_scan_cmd cmd;
		u16 chnl[4];
	} __packed cmd;
	uint i, n;
	int rc;

	if (wil->scan_request) {
		wil_err(wil, "Already scanning\n");
		return -EAGAIN;
	}

	/* check we are client side */
	switch (wdev->iftype) {
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_P2P_CLIENT:
		break;
	default:
		return -EOPNOTSUPP;
	}

	/* FW don't support scan after connection attempt */
	if (test_bit(wil_status_dontscan, &wil->status)) {
		wil_err(wil, "Can't scan now\n");
		return -EBUSY;
	}

	wil->scan_request = request;

	memset(&cmd, 0, sizeof(cmd));
	cmd.cmd.num_channels = 0;
	n = min(request->n_channels, 4U);
	for (i = 0; i < n; i++) {
		int ch = request->channels[i]->hw_value;
		if (ch == 0) {
			wil_err(wil,
				"Scan requested for unknown frequency %dMhz\n",
				request->channels[i]->center_freq);
			continue;
		}
		/* 0-based channel indexes */
		cmd.cmd.channel_list[cmd.cmd.num_channels++].channel = ch - 1;
		wil_dbg_misc(wil, "Scan for ch %d  : %d MHz\n", ch,
			     request->channels[i]->center_freq);
	}

	rc = wmi_send(wil, WMI_START_SCAN_CMDID, &cmd, sizeof(cmd.cmd) +
			cmd.cmd.num_channels * sizeof(cmd.cmd.channel_list[0]));

	if (rc)
		wil->scan_request = NULL;

	return rc;
}

static int wil_cfg80211_connect(struct wiphy *wiphy,
				struct net_device *ndev,
				struct cfg80211_connect_params *sme)
{
	struct wil6210_priv *wil = wiphy_to_wil(wiphy);
	struct cfg80211_bss *bss;
	struct wmi_connect_cmd conn;
	const u8 *ssid_eid;
	const u8 *rsn_eid;
	int ch;
	int rc = 0;

	if (test_bit(wil_status_fwconnecting, &wil->status) ||
	    test_bit(wil_status_fwconnected, &wil->status))
		return -EALREADY;

	bss = cfg80211_get_bss(wiphy, sme->channel, sme->bssid,
			       sme->ssid, sme->ssid_len,
			       WLAN_CAPABILITY_ESS, WLAN_CAPABILITY_ESS);
	if (!bss) {
		wil_err(wil, "Unable to find BSS\n");
		return -ENOENT;
	}

	ssid_eid = ieee80211_bss_get_ie(bss, WLAN_EID_SSID);
	if (!ssid_eid) {
		wil_err(wil, "No SSID\n");
		rc = -ENOENT;
		goto out;
	}

	rsn_eid = sme->ie ?
			cfg80211_find_ie(WLAN_EID_RSN, sme->ie, sme->ie_len) :
			NULL;
	if (rsn_eid) {
		if (sme->ie_len > WMI_MAX_IE_LEN) {
			rc = -ERANGE;
			wil_err(wil, "IE too large (%td bytes)\n",
				sme->ie_len);
			goto out;
		}
		/*
		 * For secure assoc, send:
		 * (1) WMI_DELETE_CIPHER_KEY_CMD
		 * (2) WMI_SET_APPIE_CMD
		 */
		rc = wmi_del_cipher_key(wil, 0, bss->bssid);
		if (rc) {
			wil_err(wil, "WMI_DELETE_CIPHER_KEY_CMD failed\n");
			goto out;
		}
		/* WMI_SET_APPIE_CMD */
		rc = wmi_set_ie(wil, WMI_FRAME_ASSOC_REQ, sme->ie_len, sme->ie);
		if (rc) {
			wil_err(wil, "WMI_SET_APPIE_CMD failed\n");
			goto out;
		}
	}

	/* WMI_CONNECT_CMD */
	memset(&conn, 0, sizeof(conn));
	switch (bss->capability & WLAN_CAPABILITY_DMG_TYPE_MASK) {
	case WLAN_CAPABILITY_DMG_TYPE_AP:
		conn.network_type = WMI_NETTYPE_INFRA;
		break;
	case WLAN_CAPABILITY_DMG_TYPE_PBSS:
		conn.network_type = WMI_NETTYPE_P2P;
		break;
	default:
		wil_err(wil, "Unsupported BSS type, capability= 0x%04x\n",
			bss->capability);
		goto out;
	}
	if (rsn_eid) {
		conn.dot11_auth_mode = WMI_AUTH11_SHARED;
		conn.auth_mode = WMI_AUTH_WPA2_PSK;
		conn.pairwise_crypto_type = WMI_CRYPT_AES_GCMP;
		conn.pairwise_crypto_len = 16;
	} else {
		conn.dot11_auth_mode = WMI_AUTH11_OPEN;
		conn.auth_mode = WMI_AUTH_NONE;
	}

	conn.ssid_len = min_t(u8, ssid_eid[1], 32);
	memcpy(conn.ssid, ssid_eid+2, conn.ssid_len);

	ch = bss->channel->hw_value;
	if (ch == 0) {
		wil_err(wil, "BSS at unknown frequency %dMhz\n",
			bss->channel->center_freq);
		rc = -EOPNOTSUPP;
		goto out;
	}
	conn.channel = ch - 1;

	memcpy(conn.bssid, bss->bssid, ETH_ALEN);
	memcpy(conn.dst_mac, bss->bssid, ETH_ALEN);

	set_bit(wil_status_fwconnecting, &wil->status);

	rc = wmi_send(wil, WMI_CONNECT_CMDID, &conn, sizeof(conn));
	if (rc == 0) {
		/* Connect can take lots of time */
		mod_timer(&wil->connect_timer,
			  jiffies + msecs_to_jiffies(2000));
	} else {
		clear_bit(wil_status_fwconnecting, &wil->status);
	}

 out:
	cfg80211_put_bss(wiphy, bss);

	return rc;
}

static int wil_cfg80211_disconnect(struct wiphy *wiphy,
				   struct net_device *ndev,
				   u16 reason_code)
{
	int rc;
	struct wil6210_priv *wil = wiphy_to_wil(wiphy);

	rc = wmi_send(wil, WMI_DISCONNECT_CMDID, NULL, 0);

	return rc;
}

static int wil_cfg80211_mgmt_tx(struct wiphy *wiphy,
				struct wireless_dev *wdev,
				struct cfg80211_mgmt_tx_params *params,
				u64 *cookie)
{
	const u8 *buf = params->buf;
	size_t len = params->len;
	struct wil6210_priv *wil = wiphy_to_wil(wiphy);
	int rc;
	struct ieee80211_mgmt *mgmt_frame = (void *)buf;
	struct wmi_sw_tx_req_cmd *cmd;
	struct {
		struct wil6210_mbox_hdr_wmi wmi;
		struct wmi_sw_tx_complete_event evt;
	} __packed evt;

	cmd = kmalloc(sizeof(*cmd) + len, GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	memcpy(cmd->dst_mac, mgmt_frame->da, WMI_MAC_LEN);
	cmd->len = cpu_to_le16(len);
	memcpy(cmd->payload, buf, len);

	rc = wmi_call(wil, WMI_SW_TX_REQ_CMDID, cmd, sizeof(*cmd) + len,
		      WMI_SW_TX_COMPLETE_EVENTID, &evt, sizeof(evt), 2000);
	if (rc == 0)
		rc = evt.evt.status;

	kfree(cmd);

	return rc;
}

static int wil_cfg80211_set_channel(struct wiphy *wiphy,
				    struct cfg80211_chan_def *chandef)
{
	struct wil6210_priv *wil = wiphy_to_wil(wiphy);
	struct wireless_dev *wdev = wil->wdev;

	wdev->preset_chandef = *chandef;

	return 0;
}

static int wil_cfg80211_add_key(struct wiphy *wiphy,
				struct net_device *ndev,
				u8 key_index, bool pairwise,
				const u8 *mac_addr,
				struct key_params *params)
{
	struct wil6210_priv *wil = wiphy_to_wil(wiphy);

	/* group key is not used */
	if (!pairwise)
		return 0;

	return wmi_add_cipher_key(wil, key_index, mac_addr,
				  params->key_len, params->key);
}

static int wil_cfg80211_del_key(struct wiphy *wiphy,
				struct net_device *ndev,
				u8 key_index, bool pairwise,
				const u8 *mac_addr)
{
	struct wil6210_priv *wil = wiphy_to_wil(wiphy);

	/* group key is not used */
	if (!pairwise)
		return 0;

	return wmi_del_cipher_key(wil, key_index, mac_addr);
}

/* Need to be present or wiphy_new() will WARN */
static int wil_cfg80211_set_default_key(struct wiphy *wiphy,
					struct net_device *ndev,
					u8 key_index, bool unicast,
					bool multicast)
{
	return 0;
}

static int wil_remain_on_channel(struct wiphy *wiphy,
				 struct wireless_dev *wdev,
				 struct ieee80211_channel *chan,
				 unsigned int duration,
				 u64 *cookie)
{
	struct wil6210_priv *wil = wiphy_to_wil(wiphy);
	int rc;

	/* TODO: handle duration */
	wil_info(wil, "%s(%d, %d ms)\n", __func__, chan->center_freq, duration);

	rc = wmi_set_channel(wil, chan->hw_value);
	if (rc)
		return rc;

	rc = wmi_rxon(wil, true);

	return rc;
}

static int wil_cancel_remain_on_channel(struct wiphy *wiphy,
					struct wireless_dev *wdev,
					u64 cookie)
{
	struct wil6210_priv *wil = wiphy_to_wil(wiphy);
	int rc;

	wil_info(wil, "%s()\n", __func__);

	rc = wmi_rxon(wil, false);

	return rc;
}

static int wil_fix_bcon(struct wil6210_priv *wil,
			struct cfg80211_beacon_data *bcon)
{
	struct ieee80211_mgmt *f = (struct ieee80211_mgmt *)bcon->probe_resp;
	size_t hlen = offsetof(struct ieee80211_mgmt, u.probe_resp.variable);
	int rc = 0;

	if (bcon->probe_resp_len <= hlen)
		return 0;

	if (!bcon->proberesp_ies) {
		bcon->proberesp_ies = f->u.probe_resp.variable;
		bcon->proberesp_ies_len = bcon->probe_resp_len - hlen;
		rc = 1;
	}
	if (!bcon->assocresp_ies) {
		bcon->assocresp_ies = f->u.probe_resp.variable;
		bcon->assocresp_ies_len = bcon->probe_resp_len - hlen;
		rc = 1;
	}

	return rc;
}

static int wil_cfg80211_start_ap(struct wiphy *wiphy,
				 struct net_device *ndev,
				 struct cfg80211_ap_settings *info)
{
	int rc = 0;
	struct wil6210_priv *wil = wiphy_to_wil(wiphy);
	struct wireless_dev *wdev = ndev->ieee80211_ptr;
	struct ieee80211_channel *channel = info->chandef.chan;
	struct cfg80211_beacon_data *bcon = &info->beacon;
	u8 wmi_nettype = wil_iftype_nl2wmi(wdev->iftype);

	if (!channel) {
		wil_err(wil, "AP: No channel???\n");
		return -EINVAL;
	}

	wil_dbg_misc(wil, "AP on Channel %d %d MHz, %s\n", channel->hw_value,
		     channel->center_freq, info->privacy ? "secure" : "open");
	print_hex_dump_bytes("SSID ", DUMP_PREFIX_OFFSET,
			     info->ssid, info->ssid_len);

	if (wil_fix_bcon(wil, bcon))
		wil_dbg_misc(wil, "Fixed bcon\n");

	mutex_lock(&wil->mutex);

	rc = wil_reset(wil);
	if (rc)
		goto out;

	/* Rx VRING. */
	rc = wil_rx_init(wil);
	if (rc)
		goto out;

	rc = wmi_set_ssid(wil, info->ssid_len, info->ssid);
	if (rc)
		goto out;

	/* MAC address - pre-requisite for other commands */
	wmi_set_mac_address(wil, ndev->dev_addr);

	/* IE's */
	/* bcon 'head IE's are not relevant for 60g band */
	/*
	 * FW do not form regular beacon, so bcon IE's are not set
	 * For the DMG bcon, when it will be supported, bcon IE's will
	 * be reused; add something like:
	 * wmi_set_ie(wil, WMI_FRAME_BEACON, bcon->beacon_ies_len,
	 * bcon->beacon_ies);
	 */
	wmi_set_ie(wil, WMI_FRAME_PROBE_RESP, bcon->proberesp_ies_len,
		   bcon->proberesp_ies);
	wmi_set_ie(wil, WMI_FRAME_ASSOC_RESP, bcon->assocresp_ies_len,
		   bcon->assocresp_ies);

	wil->secure_pcp = info->privacy;

	rc = wmi_pcp_start(wil, info->beacon_interval, wmi_nettype,
			   channel->hw_value);
	if (rc)
		goto out;


	netif_carrier_on(ndev);

out:
	mutex_unlock(&wil->mutex);
	return rc;
}

static int wil_cfg80211_stop_ap(struct wiphy *wiphy,
				struct net_device *ndev)
{
	int rc = 0;
	struct wil6210_priv *wil = wiphy_to_wil(wiphy);

	mutex_lock(&wil->mutex);

	rc = wmi_pcp_stop(wil);

	mutex_unlock(&wil->mutex);
	return rc;
}

static int wil_cfg80211_del_station(struct wiphy *wiphy,
				    struct net_device *dev, u8 *mac)
{
	struct wil6210_priv *wil = wiphy_to_wil(wiphy);

	mutex_lock(&wil->mutex);
	wil6210_disconnect(wil, mac);
	mutex_unlock(&wil->mutex);

	return 0;
}

static struct cfg80211_ops wil_cfg80211_ops = {
	.scan = wil_cfg80211_scan,
	.connect = wil_cfg80211_connect,
	.disconnect = wil_cfg80211_disconnect,
	.change_virtual_intf = wil_cfg80211_change_iface,
	.get_station = wil_cfg80211_get_station,
	.dump_station = wil_cfg80211_dump_station,
	.remain_on_channel = wil_remain_on_channel,
	.cancel_remain_on_channel = wil_cancel_remain_on_channel,
	.mgmt_tx = wil_cfg80211_mgmt_tx,
	.set_monitor_channel = wil_cfg80211_set_channel,
	.add_key = wil_cfg80211_add_key,
	.del_key = wil_cfg80211_del_key,
	.set_default_key = wil_cfg80211_set_default_key,
	/* AP mode */
	.start_ap = wil_cfg80211_start_ap,
	.stop_ap = wil_cfg80211_stop_ap,
	.del_station = wil_cfg80211_del_station,
};

static void wil_wiphy_init(struct wiphy *wiphy)
{
	/* TODO: set real value */
	wiphy->max_scan_ssids = 10;
	wiphy->max_num_pmkids = 0 /* TODO: */;
	wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION) |
				 BIT(NL80211_IFTYPE_AP) |
				 BIT(NL80211_IFTYPE_MONITOR);
	/* TODO: enable P2P when integrated with supplicant:
	 * BIT(NL80211_IFTYPE_P2P_CLIENT) | BIT(NL80211_IFTYPE_P2P_GO)
	 */
	wiphy->flags |= WIPHY_FLAG_HAVE_AP_SME |
			WIPHY_FLAG_AP_PROBE_RESP_OFFLOAD;
	dev_warn(wiphy_dev(wiphy), "%s : flags = 0x%08x\n",
		 __func__, wiphy->flags);
	wiphy->probe_resp_offload =
		NL80211_PROBE_RESP_OFFLOAD_SUPPORT_WPS |
		NL80211_PROBE_RESP_OFFLOAD_SUPPORT_WPS2 |
		NL80211_PROBE_RESP_OFFLOAD_SUPPORT_P2P;

	wiphy->bands[IEEE80211_BAND_60GHZ] = &wil_band_60ghz;

	/* TODO: figure this out */
	wiphy->signal_type = CFG80211_SIGNAL_TYPE_UNSPEC;

	wiphy->cipher_suites = wil_cipher_suites;
	wiphy->n_cipher_suites = ARRAY_SIZE(wil_cipher_suites);
	wiphy->mgmt_stypes = wil_mgmt_stypes;
}

struct wireless_dev *wil_cfg80211_init(struct device *dev)
{
	int rc = 0;
	struct wireless_dev *wdev;

	wdev = kzalloc(sizeof(struct wireless_dev), GFP_KERNEL);
	if (!wdev)
		return ERR_PTR(-ENOMEM);

	wdev->wiphy = wiphy_new(&wil_cfg80211_ops,
				sizeof(struct wil6210_priv));
	if (!wdev->wiphy) {
		rc = -ENOMEM;
		goto out;
	}

	set_wiphy_dev(wdev->wiphy, dev);
	wil_wiphy_init(wdev->wiphy);

	rc = wiphy_register(wdev->wiphy);
	if (rc < 0)
		goto out_failed_reg;

	return wdev;

out_failed_reg:
	wiphy_free(wdev->wiphy);
out:
	kfree(wdev);

	return ERR_PTR(rc);
}

void wil_wdev_free(struct wil6210_priv *wil)
{
	struct wireless_dev *wdev = wil_to_wdev(wil);

	if (!wdev)
		return;

	wiphy_unregister(wdev->wiphy);
	wiphy_free(wdev->wiphy);
	kfree(wdev);
}
