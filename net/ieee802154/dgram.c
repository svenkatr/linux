/*
 * IEEE 802.15.4 dgram socket interface
 *
 * Copyright 2007, 2008 Siemens AG
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Written by:
 * Sergey Lapin <slapin@ossfans.org>
 * Dmitry Eremin-Solenikov <dbaryshkov@gmail.com>
 */

#include <linux/net.h>
#include <linux/module.h>
#include <linux/if_arp.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <net/sock.h>
#include <net/af_ieee802154.h>
#include <net/ieee802154.h>
#include <net/ieee802154_netdev.h>

#include <asm/ioctls.h>

#include "af802154.h"

static HLIST_HEAD(dgram_head);
static DEFINE_RWLOCK(dgram_lock);

struct dgram_sock {
	struct sock sk;

	struct ieee802154_addr src_addr;
	struct ieee802154_addr dst_addr;

	unsigned int bound:1;
	unsigned int want_ack:1;
};

static inline struct dgram_sock *dgram_sk(const struct sock *sk)
{
	return container_of(sk, struct dgram_sock, sk);
}

static void dgram_hash(struct sock *sk)
{
	write_lock_bh(&dgram_lock);
	sk_add_node(sk, &dgram_head);
	sock_prot_inuse_add(sock_net(sk), sk->sk_prot, 1);
	write_unlock_bh(&dgram_lock);
}

static void dgram_unhash(struct sock *sk)
{
	write_lock_bh(&dgram_lock);
	if (sk_del_node_init(sk))
		sock_prot_inuse_add(sock_net(sk), sk->sk_prot, -1);
	write_unlock_bh(&dgram_lock);
}

static int dgram_init(struct sock *sk)
{
	struct dgram_sock *ro = dgram_sk(sk);

	ro->dst_addr.mode = IEEE802154_ADDR_LONG;
	ro->dst_addr.pan_id = cpu_to_le16(IEEE802154_PANID_BROADCAST);
	ro->want_ack = 1;
	memset(&ro->dst_addr.extended_addr, 0xff, IEEE802154_ADDR_LEN);
	return 0;
}

static void dgram_close(struct sock *sk, long timeout)
{
	sk_common_release(sk);
}

static int dgram_bind(struct sock *sk, struct sockaddr *uaddr, int len)
{
	struct sockaddr_ieee802154 *addr = (struct sockaddr_ieee802154 *)uaddr;
	struct ieee802154_addr haddr;
	struct dgram_sock *ro = dgram_sk(sk);
	int err = -EINVAL;
	struct net_device *dev;

	lock_sock(sk);

	ro->bound = 0;

	if (len < sizeof(*addr))
		goto out;

	if (addr->family != AF_IEEE802154)
		goto out;

	ieee802154_addr_from_sa(&haddr, &addr->addr);
	dev = ieee802154_get_dev(sock_net(sk), &haddr);
	if (!dev) {
		err = -ENODEV;
		goto out;
	}

	if (dev->type != ARPHRD_IEEE802154) {
		err = -ENODEV;
		goto out_put;
	}

	ro->src_addr = haddr;

	ro->bound = 1;
	err = 0;
out_put:
	dev_put(dev);
out:
	release_sock(sk);

	return err;
}

static int dgram_ioctl(struct sock *sk, int cmd, unsigned long arg)
{
	switch (cmd) {
	case SIOCOUTQ:
	{
		int amount = sk_wmem_alloc_get(sk);

		return put_user(amount, (int __user *)arg);
	}

	case SIOCINQ:
	{
		struct sk_buff *skb;
		unsigned long amount;

		amount = 0;
		spin_lock_bh(&sk->sk_receive_queue.lock);
		skb = skb_peek(&sk->sk_receive_queue);
		if (skb != NULL) {
			/*
			 * We will only return the amount
			 * of this packet since that is all
			 * that will be read.
			 */
			amount = skb->len - ieee802154_hdr_length(skb);
		}
		spin_unlock_bh(&sk->sk_receive_queue.lock);
		return put_user(amount, (int __user *)arg);
	}

	}
	return -ENOIOCTLCMD;
}

/* FIXME: autobind */
static int dgram_connect(struct sock *sk, struct sockaddr *uaddr,
			int len)
{
	struct sockaddr_ieee802154 *addr = (struct sockaddr_ieee802154 *)uaddr;
	struct dgram_sock *ro = dgram_sk(sk);
	int err = 0;

	if (len < sizeof(*addr))
		return -EINVAL;

	if (addr->family != AF_IEEE802154)
		return -EINVAL;

	lock_sock(sk);

	if (!ro->bound) {
		err = -ENETUNREACH;
		goto out;
	}

	ieee802154_addr_from_sa(&ro->dst_addr, &addr->addr);

out:
	release_sock(sk);
	return err;
}

static int dgram_disconnect(struct sock *sk, int flags)
{
	struct dgram_sock *ro = dgram_sk(sk);

	lock_sock(sk);

	ro->dst_addr.mode = IEEE802154_ADDR_LONG;
	memset(&ro->dst_addr.extended_addr, 0xff, IEEE802154_ADDR_LEN);

	release_sock(sk);

	return 0;
}

static int dgram_sendmsg(struct kiocb *iocb, struct sock *sk,
		struct msghdr *msg, size_t size)
{
	struct net_device *dev;
	unsigned int mtu;
	struct sk_buff *skb;
	struct dgram_sock *ro = dgram_sk(sk);
	int hlen, tlen;
	int err;

	if (msg->msg_flags & MSG_OOB) {
		pr_debug("msg->msg_flags = 0x%x\n", msg->msg_flags);
		return -EOPNOTSUPP;
	}

	if (!ro->bound)
		dev = dev_getfirstbyhwtype(sock_net(sk), ARPHRD_IEEE802154);
	else
		dev = ieee802154_get_dev(sock_net(sk), &ro->src_addr);

	if (!dev) {
		pr_debug("no dev\n");
		err = -ENXIO;
		goto out;
	}
	mtu = dev->mtu;
	pr_debug("name = %s, mtu = %u\n", dev->name, mtu);

	if (size > mtu) {
		pr_debug("size = %Zu, mtu = %u\n", size, mtu);
		err = -EMSGSIZE;
		goto out_dev;
	}

	hlen = LL_RESERVED_SPACE(dev);
	tlen = dev->needed_tailroom;
	skb = sock_alloc_send_skb(sk, hlen + tlen + size,
			msg->msg_flags & MSG_DONTWAIT,
			&err);
	if (!skb)
		goto out_dev;

	skb_reserve(skb, hlen);

	skb_reset_network_header(skb);

	mac_cb(skb)->flags = IEEE802154_FC_TYPE_DATA;
	if (ro->want_ack)
		mac_cb(skb)->flags |= MAC_CB_FLAG_ACKREQ;

	mac_cb(skb)->seq = ieee802154_mlme_ops(dev)->get_dsn(dev);
	err = dev_hard_header(skb, dev, ETH_P_IEEE802154, &ro->dst_addr,
			ro->bound ? &ro->src_addr : NULL, size);
	if (err < 0)
		goto out_skb;

	skb_reset_mac_header(skb);

	err = memcpy_fromiovec(skb_put(skb, size), msg->msg_iov, size);
	if (err < 0)
		goto out_skb;

	skb->dev = dev;
	skb->sk  = sk;
	skb->protocol = htons(ETH_P_IEEE802154);

	dev_put(dev);

	err = dev_queue_xmit(skb);
	if (err > 0)
		err = net_xmit_errno(err);

	return err ?: size;

out_skb:
	kfree_skb(skb);
out_dev:
	dev_put(dev);
out:
	return err;
}

static int dgram_recvmsg(struct kiocb *iocb, struct sock *sk,
		struct msghdr *msg, size_t len, int noblock, int flags,
		int *addr_len)
{
	size_t copied = 0;
	int err = -EOPNOTSUPP;
	struct sk_buff *skb;
	DECLARE_SOCKADDR(struct sockaddr_ieee802154 *, saddr, msg->msg_name);

	skb = skb_recv_datagram(sk, flags, noblock, &err);
	if (!skb)
		goto out;

	copied = skb->len;
	if (len < copied) {
		msg->msg_flags |= MSG_TRUNC;
		copied = len;
	}

	/* FIXME: skip headers if necessary ?! */
	err = skb_copy_datagram_iovec(skb, 0, msg->msg_iov, copied);
	if (err)
		goto done;

	sock_recv_ts_and_drops(msg, sk, skb);

	if (saddr) {
		saddr->family = AF_IEEE802154;
		ieee802154_addr_to_sa(&saddr->addr, &mac_cb(skb)->source);
		*addr_len = sizeof(*saddr);
	}

	if (flags & MSG_TRUNC)
		copied = skb->len;
done:
	skb_free_datagram(sk, skb);
out:
	if (err)
		return err;
	return copied;
}

static int dgram_rcv_skb(struct sock *sk, struct sk_buff *skb)
{
	skb = skb_share_check(skb, GFP_ATOMIC);
	if (!skb)
		return NET_RX_DROP;

	if (sock_queue_rcv_skb(sk, skb) < 0) {
		kfree_skb(skb);
		return NET_RX_DROP;
	}

	return NET_RX_SUCCESS;
}

static inline bool
ieee802154_match_sock(__le64 hw_addr, __le16 pan_id, __le16 short_addr,
		      struct dgram_sock *ro)
{
	if (!ro->bound)
		return true;

	if (ro->src_addr.mode == IEEE802154_ADDR_LONG &&
	    hw_addr == ro->src_addr.extended_addr)
		return true;

	if (ro->src_addr.mode == IEEE802154_ADDR_SHORT &&
	    pan_id == ro->src_addr.pan_id &&
	    short_addr == ro->src_addr.short_addr)
		return true;

	return false;
}

int ieee802154_dgram_deliver(struct net_device *dev, struct sk_buff *skb)
{
	struct sock *sk, *prev = NULL;
	int ret = NET_RX_SUCCESS;
	__le16 pan_id, short_addr;
	__le64 hw_addr;

	/* Data frame processing */
	BUG_ON(dev->type != ARPHRD_IEEE802154);

	pan_id = ieee802154_mlme_ops(dev)->get_pan_id(dev);
	short_addr = ieee802154_mlme_ops(dev)->get_short_addr(dev);
	hw_addr = ieee802154_devaddr_from_raw(dev->dev_addr);

	read_lock(&dgram_lock);
	sk_for_each(sk, &dgram_head) {
		if (ieee802154_match_sock(hw_addr, pan_id, short_addr,
					  dgram_sk(sk))) {
			if (prev) {
				struct sk_buff *clone;
				clone = skb_clone(skb, GFP_ATOMIC);
				if (clone)
					dgram_rcv_skb(prev, clone);
			}

			prev = sk;
		}
	}

	if (prev)
		dgram_rcv_skb(prev, skb);
	else {
		kfree_skb(skb);
		ret = NET_RX_DROP;
	}
	read_unlock(&dgram_lock);

	return ret;
}

static int dgram_getsockopt(struct sock *sk, int level, int optname,
		    char __user *optval, int __user *optlen)
{
	struct dgram_sock *ro = dgram_sk(sk);

	int val, len;

	if (level != SOL_IEEE802154)
		return -EOPNOTSUPP;

	if (get_user(len, optlen))
		return -EFAULT;

	len = min_t(unsigned int, len, sizeof(int));

	switch (optname) {
	case WPAN_WANTACK:
		val = ro->want_ack;
		break;
	default:
		return -ENOPROTOOPT;
	}

	if (put_user(len, optlen))
		return -EFAULT;
	if (copy_to_user(optval, &val, len))
		return -EFAULT;
	return 0;
}

static int dgram_setsockopt(struct sock *sk, int level, int optname,
		    char __user *optval, unsigned int optlen)
{
	struct dgram_sock *ro = dgram_sk(sk);
	int val;
	int err = 0;

	if (optlen < sizeof(int))
		return -EINVAL;

	if (get_user(val, (int __user *)optval))
		return -EFAULT;

	lock_sock(sk);

	switch (optname) {
	case WPAN_WANTACK:
		ro->want_ack = !!val;
		break;
	default:
		err = -ENOPROTOOPT;
		break;
	}

	release_sock(sk);
	return err;
}

struct proto ieee802154_dgram_prot = {
	.name		= "IEEE-802.15.4-MAC",
	.owner		= THIS_MODULE,
	.obj_size	= sizeof(struct dgram_sock),
	.init		= dgram_init,
	.close		= dgram_close,
	.bind		= dgram_bind,
	.sendmsg	= dgram_sendmsg,
	.recvmsg	= dgram_recvmsg,
	.hash		= dgram_hash,
	.unhash		= dgram_unhash,
	.connect	= dgram_connect,
	.disconnect	= dgram_disconnect,
	.ioctl		= dgram_ioctl,
	.getsockopt	= dgram_getsockopt,
	.setsockopt	= dgram_setsockopt,
};

