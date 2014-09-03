/*
 *	IPV4 GSO/GRO offload support
 *	Linux INET implementation
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 *	GRE GSO support
 */

#include <linux/skbuff.h>
#include <linux/init.h>
#include <net/protocol.h>
#include <net/gre.h>

static int gre_gso_send_check(struct sk_buff *skb)
{
	if (!skb->encapsulation)
		return -EINVAL;
	return 0;
}

static struct sk_buff *gre_gso_segment(struct sk_buff *skb,
				       netdev_features_t features)
{
	struct sk_buff *segs = ERR_PTR(-EINVAL);
	netdev_features_t enc_features;
	int ghl;
	struct gre_base_hdr *greh;
	u16 mac_offset = skb->mac_header;
	int mac_len = skb->mac_len;
	__be16 protocol = skb->protocol;
	int tnl_hlen;
	bool csum;

	if (unlikely(skb_shinfo(skb)->gso_type &
				~(SKB_GSO_TCPV4 |
				  SKB_GSO_TCPV6 |
				  SKB_GSO_UDP |
				  SKB_GSO_DODGY |
				  SKB_GSO_TCP_ECN |
				  SKB_GSO_GRE |
				  SKB_GSO_IPIP)))
		goto out;

	if (unlikely(!pskb_may_pull(skb, sizeof(*greh))))
		goto out;

	greh = (struct gre_base_hdr *)skb_transport_header(skb);

	ghl = skb_inner_network_header(skb) - skb_transport_header(skb);
	if (unlikely(ghl < sizeof(*greh)))
		goto out;

	csum = !!(greh->flags & GRE_CSUM);

	if (unlikely(!pskb_may_pull(skb, ghl)))
		goto out;

	/* setup inner skb. */
	skb->protocol = greh->protocol;
	skb->encapsulation = 0;

	__skb_pull(skb, ghl);
	skb_reset_mac_header(skb);
	skb_set_network_header(skb, skb_inner_network_offset(skb));
	skb->mac_len = skb_inner_network_offset(skb);

	/* segment inner packet. */
	enc_features = skb->dev->hw_enc_features & netif_skb_features(skb);
	segs = skb_mac_gso_segment(skb, enc_features);
	if (!segs || IS_ERR(segs)) {
		skb_gso_error_unwind(skb, protocol, ghl, mac_offset, mac_len);
		goto out;
	}

	skb = segs;
	tnl_hlen = skb_tnl_header_len(skb);
	do {
		__skb_push(skb, ghl);
		if (csum) {
			__be32 *pcsum;

			if (skb_has_shared_frag(skb)) {
				int err;

				err = __skb_linearize(skb);
				if (err) {
					kfree_skb_list(segs);
					segs = ERR_PTR(err);
					goto out;
				}
			}

			greh = (struct gre_base_hdr *)(skb->data);
			pcsum = (__be32 *)(greh + 1);
			*pcsum = 0;
			*(__sum16 *)pcsum = csum_fold(skb_checksum(skb, 0, skb->len, 0));
		}
		__skb_push(skb, tnl_hlen - ghl);

		skb_reset_inner_headers(skb);
		skb->encapsulation = 1;

		skb_reset_mac_header(skb);
		skb_set_network_header(skb, mac_len);
		skb->mac_len = mac_len;
		skb->protocol = protocol;
	} while ((skb = skb->next));
out:
	return segs;
}

/* Compute the whole skb csum in s/w and store it, then verify GRO csum
 * starting from gro_offset.
 */
static __sum16 gro_skb_checksum(struct sk_buff *skb)
{
	__sum16 sum;

	skb->csum = skb_checksum(skb, 0, skb->len, 0);
	NAPI_GRO_CB(skb)->csum = csum_sub(skb->csum,
		csum_partial(skb->data, skb_gro_offset(skb), 0));
	sum = csum_fold(NAPI_GRO_CB(skb)->csum);
	if (unlikely(skb->ip_summed == CHECKSUM_COMPLETE)) {
		if (unlikely(!sum))
			netdev_rx_csum_fault(skb->dev);
	} else
		skb->ip_summed = CHECKSUM_COMPLETE;

	return sum;
}

static struct sk_buff **gre_gro_receive(struct sk_buff **head,
					struct sk_buff *skb)
{
	struct sk_buff **pp = NULL;
	struct sk_buff *p;
	const struct gre_base_hdr *greh;
	unsigned int hlen, grehlen;
	unsigned int off;
	int flush = 1;
	struct packet_offload *ptype;
	__be16 type;

	off = skb_gro_offset(skb);
	hlen = off + sizeof(*greh);
	greh = skb_gro_header_fast(skb, off);
	if (skb_gro_header_hard(skb, hlen)) {
		greh = skb_gro_header_slow(skb, hlen, off);
		if (unlikely(!greh))
			goto out;
	}

	/* Only support version 0 and K (key), C (csum) flags. Note that
	 * although the support for the S (seq#) flag can be added easily
	 * for GRO, this is problematic for GSO hence can not be enabled
	 * here because a GRO pkt may end up in the forwarding path, thus
	 * requiring GSO support to break it up correctly.
	 */
	if ((greh->flags & ~(GRE_KEY|GRE_CSUM)) != 0)
		goto out;

	type = greh->protocol;

	rcu_read_lock();
	ptype = gro_find_receive_by_type(type);
	if (ptype == NULL)
		goto out_unlock;

	grehlen = GRE_HEADER_SECTION;

	if (greh->flags & GRE_KEY)
		grehlen += GRE_HEADER_SECTION;

	if (greh->flags & GRE_CSUM)
		grehlen += GRE_HEADER_SECTION;

	hlen = off + grehlen;
	if (skb_gro_header_hard(skb, hlen)) {
		greh = skb_gro_header_slow(skb, hlen, off);
		if (unlikely(!greh))
			goto out_unlock;
	}
	if (greh->flags & GRE_CSUM) { /* Need to verify GRE csum first */
		__sum16 csum = 0;

		if (skb->ip_summed == CHECKSUM_COMPLETE)
			csum = csum_fold(NAPI_GRO_CB(skb)->csum);
		/* Don't trust csum error calculated/reported by h/w */
		if (skb->ip_summed == CHECKSUM_NONE || csum != 0)
			csum = gro_skb_checksum(skb);

		/* GRE CSUM is the 1's complement of the 1's complement sum
		 * of the GRE hdr plus payload so it should add up to 0xffff
		 * (and 0 after csum_fold()) just like the IPv4 hdr csum.
		 */
		if (csum)
			goto out_unlock;
	}
	flush = 0;

	for (p = *head; p; p = p->next) {
		const struct gre_base_hdr *greh2;

		if (!NAPI_GRO_CB(p)->same_flow)
			continue;

		/* The following checks are needed to ensure only pkts
		 * from the same tunnel are considered for aggregation.
		 * The criteria for "the same tunnel" includes:
		 * 1) same version (we only support version 0 here)
		 * 2) same protocol (we only support ETH_P_IP for now)
		 * 3) same set of flags
		 * 4) same key if the key field is present.
		 */
		greh2 = (struct gre_base_hdr *)(p->data + off);

		if (greh2->flags != greh->flags ||
		    greh2->protocol != greh->protocol) {
			NAPI_GRO_CB(p)->same_flow = 0;
			continue;
		}
		if (greh->flags & GRE_KEY) {
			/* compare keys */
			if (*(__be32 *)(greh2+1) != *(__be32 *)(greh+1)) {
				NAPI_GRO_CB(p)->same_flow = 0;
				continue;
			}
		}
	}

	skb_gro_pull(skb, grehlen);

	/* Adjusted NAPI_GRO_CB(skb)->csum after skb_gro_pull()*/
	skb_gro_postpull_rcsum(skb, greh, grehlen);

	pp = ptype->callbacks.gro_receive(head, skb);

out_unlock:
	rcu_read_unlock();
out:
	NAPI_GRO_CB(skb)->flush |= flush;

	return pp;
}

static int gre_gro_complete(struct sk_buff *skb, int nhoff)
{
	struct gre_base_hdr *greh = (struct gre_base_hdr *)(skb->data + nhoff);
	struct packet_offload *ptype;
	unsigned int grehlen = sizeof(*greh);
	int err = -ENOENT;
	__be16 type;

	type = greh->protocol;
	if (greh->flags & GRE_KEY)
		grehlen += GRE_HEADER_SECTION;

	if (greh->flags & GRE_CSUM)
		grehlen += GRE_HEADER_SECTION;

	rcu_read_lock();
	ptype = gro_find_complete_by_type(type);
	if (ptype != NULL)
		err = ptype->callbacks.gro_complete(skb, nhoff + grehlen);

	rcu_read_unlock();
	return err;
}

static const struct net_offload gre_offload = {
	.callbacks = {
		.gso_send_check = gre_gso_send_check,
		.gso_segment = gre_gso_segment,
		.gro_receive = gre_gro_receive,
		.gro_complete = gre_gro_complete,
	},
};

static int __init gre_offload_init(void)
{
	return inet_add_offload(&gre_offload, IPPROTO_GRE);
}
device_initcall(gre_offload_init);
