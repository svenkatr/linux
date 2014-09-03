/*
 * net/tipc/socket.c: TIPC socket API
 *
 * Copyright (c) 2001-2007, 2012-2014, Ericsson AB
 * Copyright (c) 2004-2008, 2010-2013, Wind River Systems
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the names of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "core.h"
#include "port.h"

#include <linux/export.h>

#define SS_LISTENING	-1	/* socket is listening */
#define SS_READY	-2	/* socket is connectionless */

#define CONN_TIMEOUT_DEFAULT	8000	/* default connect timeout = 8s */

static int backlog_rcv(struct sock *sk, struct sk_buff *skb);
static void tipc_data_ready(struct sock *sk);
static void tipc_write_space(struct sock *sk);
static int tipc_release(struct socket *sock);
static int tipc_accept(struct socket *sock, struct socket *new_sock, int flags);

static const struct proto_ops packet_ops;
static const struct proto_ops stream_ops;
static const struct proto_ops msg_ops;

static struct proto tipc_proto;
static struct proto tipc_proto_kern;

/*
 * Revised TIPC socket locking policy:
 *
 * Most socket operations take the standard socket lock when they start
 * and hold it until they finish (or until they need to sleep).  Acquiring
 * this lock grants the owner exclusive access to the fields of the socket
 * data structures, with the exception of the backlog queue.  A few socket
 * operations can be done without taking the socket lock because they only
 * read socket information that never changes during the life of the socket.
 *
 * Socket operations may acquire the lock for the associated TIPC port if they
 * need to perform an operation on the port.  If any routine needs to acquire
 * both the socket lock and the port lock it must take the socket lock first
 * to avoid the risk of deadlock.
 *
 * The dispatcher handling incoming messages cannot grab the socket lock in
 * the standard fashion, since invoked it runs at the BH level and cannot block.
 * Instead, it checks to see if the socket lock is currently owned by someone,
 * and either handles the message itself or adds it to the socket's backlog
 * queue; in the latter case the queued message is processed once the process
 * owning the socket lock releases it.
 *
 * NOTE: Releasing the socket lock while an operation is sleeping overcomes
 * the problem of a blocked socket operation preventing any other operations
 * from occurring.  However, applications must be careful if they have
 * multiple threads trying to send (or receive) on the same socket, as these
 * operations might interfere with each other.  For example, doing a connect
 * and a receive at the same time might allow the receive to consume the
 * ACK message meant for the connect.  While additional work could be done
 * to try and overcome this, it doesn't seem to be worthwhile at the present.
 *
 * NOTE: Releasing the socket lock while an operation is sleeping also ensures
 * that another operation that must be performed in a non-blocking manner is
 * not delayed for very long because the lock has already been taken.
 *
 * NOTE: This code assumes that certain fields of a port/socket pair are
 * constant over its lifetime; such fields can be examined without taking
 * the socket lock and/or port lock, and do not need to be re-read even
 * after resuming processing after waiting.  These fields include:
 *   - socket type
 *   - pointer to socket sk structure (aka tipc_sock structure)
 *   - pointer to port structure
 *   - port reference
 */

#include "socket.h"

/**
 * advance_rx_queue - discard first buffer in socket receive queue
 *
 * Caller must hold socket lock
 */
static void advance_rx_queue(struct sock *sk)
{
	kfree_skb(__skb_dequeue(&sk->sk_receive_queue));
}

/**
 * reject_rx_queue - reject all buffers in socket receive queue
 *
 * Caller must hold socket lock
 */
static void reject_rx_queue(struct sock *sk)
{
	struct sk_buff *buf;

	while ((buf = __skb_dequeue(&sk->sk_receive_queue)))
		tipc_reject_msg(buf, TIPC_ERR_NO_PORT);
}

/**
 * tipc_sk_create - create a TIPC socket
 * @net: network namespace (must be default network)
 * @sock: pre-allocated socket structure
 * @protocol: protocol indicator (must be 0)
 * @kern: caused by kernel or by userspace?
 *
 * This routine creates additional data structures used by the TIPC socket,
 * initializes them, and links them together.
 *
 * Returns 0 on success, errno otherwise
 */
static int tipc_sk_create(struct net *net, struct socket *sock,
			  int protocol, int kern)
{
	const struct proto_ops *ops;
	socket_state state;
	struct sock *sk;
	struct tipc_sock *tsk;
	struct tipc_port *port;
	u32 ref;

	/* Validate arguments */
	if (unlikely(protocol != 0))
		return -EPROTONOSUPPORT;

	switch (sock->type) {
	case SOCK_STREAM:
		ops = &stream_ops;
		state = SS_UNCONNECTED;
		break;
	case SOCK_SEQPACKET:
		ops = &packet_ops;
		state = SS_UNCONNECTED;
		break;
	case SOCK_DGRAM:
	case SOCK_RDM:
		ops = &msg_ops;
		state = SS_READY;
		break;
	default:
		return -EPROTOTYPE;
	}

	/* Allocate socket's protocol area */
	if (!kern)
		sk = sk_alloc(net, AF_TIPC, GFP_KERNEL, &tipc_proto);
	else
		sk = sk_alloc(net, AF_TIPC, GFP_KERNEL, &tipc_proto_kern);

	if (sk == NULL)
		return -ENOMEM;

	tsk = tipc_sk(sk);
	port = &tsk->port;

	ref = tipc_port_init(port, TIPC_LOW_IMPORTANCE);
	if (!ref) {
		pr_warn("Socket registration failed, ref. table exhausted\n");
		sk_free(sk);
		return -ENOMEM;
	}

	/* Finish initializing socket data structures */
	sock->ops = ops;
	sock->state = state;

	sock_init_data(sock, sk);
	sk->sk_backlog_rcv = backlog_rcv;
	sk->sk_rcvbuf = sysctl_tipc_rmem[1];
	sk->sk_data_ready = tipc_data_ready;
	sk->sk_write_space = tipc_write_space;
	tipc_sk(sk)->conn_timeout = CONN_TIMEOUT_DEFAULT;
	tipc_port_unlock(port);

	if (sock->state == SS_READY) {
		tipc_port_set_unreturnable(port, true);
		if (sock->type == SOCK_DGRAM)
			tipc_port_set_unreliable(port, true);
	}
	return 0;
}

/**
 * tipc_sock_create_local - create TIPC socket from inside TIPC module
 * @type: socket type - SOCK_RDM or SOCK_SEQPACKET
 *
 * We cannot use sock_creat_kern here because it bumps module user count.
 * Since socket owner and creator is the same module we must make sure
 * that module count remains zero for module local sockets, otherwise
 * we cannot do rmmod.
 *
 * Returns 0 on success, errno otherwise
 */
int tipc_sock_create_local(int type, struct socket **res)
{
	int rc;

	rc = sock_create_lite(AF_TIPC, type, 0, res);
	if (rc < 0) {
		pr_err("Failed to create kernel socket\n");
		return rc;
	}
	tipc_sk_create(&init_net, *res, 0, 1);

	return 0;
}

/**
 * tipc_sock_release_local - release socket created by tipc_sock_create_local
 * @sock: the socket to be released.
 *
 * Module reference count is not incremented when such sockets are created,
 * so we must keep it from being decremented when they are released.
 */
void tipc_sock_release_local(struct socket *sock)
{
	tipc_release(sock);
	sock->ops = NULL;
	sock_release(sock);
}

/**
 * tipc_sock_accept_local - accept a connection on a socket created
 * with tipc_sock_create_local. Use this function to avoid that
 * module reference count is inadvertently incremented.
 *
 * @sock:    the accepting socket
 * @newsock: reference to the new socket to be created
 * @flags:   socket flags
 */

int tipc_sock_accept_local(struct socket *sock, struct socket **newsock,
			   int flags)
{
	struct sock *sk = sock->sk;
	int ret;

	ret = sock_create_lite(sk->sk_family, sk->sk_type,
			       sk->sk_protocol, newsock);
	if (ret < 0)
		return ret;

	ret = tipc_accept(sock, *newsock, flags);
	if (ret < 0) {
		sock_release(*newsock);
		return ret;
	}
	(*newsock)->ops = sock->ops;
	return ret;
}

/**
 * tipc_release - destroy a TIPC socket
 * @sock: socket to destroy
 *
 * This routine cleans up any messages that are still queued on the socket.
 * For DGRAM and RDM socket types, all queued messages are rejected.
 * For SEQPACKET and STREAM socket types, the first message is rejected
 * and any others are discarded.  (If the first message on a STREAM socket
 * is partially-read, it is discarded and the next one is rejected instead.)
 *
 * NOTE: Rejected messages are not necessarily returned to the sender!  They
 * are returned or discarded according to the "destination droppable" setting
 * specified for the message by the sender.
 *
 * Returns 0 on success, errno otherwise
 */
static int tipc_release(struct socket *sock)
{
	struct sock *sk = sock->sk;
	struct tipc_sock *tsk;
	struct tipc_port *port;
	struct sk_buff *buf;

	/*
	 * Exit if socket isn't fully initialized (occurs when a failed accept()
	 * releases a pre-allocated child socket that was never used)
	 */
	if (sk == NULL)
		return 0;

	tsk = tipc_sk(sk);
	port = &tsk->port;
	lock_sock(sk);

	/*
	 * Reject all unreceived messages, except on an active connection
	 * (which disconnects locally & sends a 'FIN+' to peer)
	 */
	while (sock->state != SS_DISCONNECTING) {
		buf = __skb_dequeue(&sk->sk_receive_queue);
		if (buf == NULL)
			break;
		if (TIPC_SKB_CB(buf)->handle != NULL)
			kfree_skb(buf);
		else {
			if ((sock->state == SS_CONNECTING) ||
			    (sock->state == SS_CONNECTED)) {
				sock->state = SS_DISCONNECTING;
				tipc_port_disconnect(port->ref);
			}
			tipc_reject_msg(buf, TIPC_ERR_NO_PORT);
		}
	}

	/* Destroy TIPC port; also disconnects an active connection and
	 * sends a 'FIN-' to peer.
	 */
	tipc_port_destroy(port);

	/* Discard any remaining (connection-based) messages in receive queue */
	__skb_queue_purge(&sk->sk_receive_queue);

	/* Reject any messages that accumulated in backlog queue */
	sock->state = SS_DISCONNECTING;
	release_sock(sk);

	sock_put(sk);
	sock->sk = NULL;

	return 0;
}

/**
 * tipc_bind - associate or disassocate TIPC name(s) with a socket
 * @sock: socket structure
 * @uaddr: socket address describing name(s) and desired operation
 * @uaddr_len: size of socket address data structure
 *
 * Name and name sequence binding is indicated using a positive scope value;
 * a negative scope value unbinds the specified name.  Specifying no name
 * (i.e. a socket address length of 0) unbinds all names from the socket.
 *
 * Returns 0 on success, errno otherwise
 *
 * NOTE: This routine doesn't need to take the socket lock since it doesn't
 *       access any non-constant socket information.
 */
static int tipc_bind(struct socket *sock, struct sockaddr *uaddr,
		     int uaddr_len)
{
	struct sock *sk = sock->sk;
	struct sockaddr_tipc *addr = (struct sockaddr_tipc *)uaddr;
	struct tipc_sock *tsk = tipc_sk(sk);
	int res = -EINVAL;

	lock_sock(sk);
	if (unlikely(!uaddr_len)) {
		res = tipc_withdraw(&tsk->port, 0, NULL);
		goto exit;
	}

	if (uaddr_len < sizeof(struct sockaddr_tipc)) {
		res = -EINVAL;
		goto exit;
	}
	if (addr->family != AF_TIPC) {
		res = -EAFNOSUPPORT;
		goto exit;
	}

	if (addr->addrtype == TIPC_ADDR_NAME)
		addr->addr.nameseq.upper = addr->addr.nameseq.lower;
	else if (addr->addrtype != TIPC_ADDR_NAMESEQ) {
		res = -EAFNOSUPPORT;
		goto exit;
	}

	if ((addr->addr.nameseq.type < TIPC_RESERVED_TYPES) &&
	    (addr->addr.nameseq.type != TIPC_TOP_SRV) &&
	    (addr->addr.nameseq.type != TIPC_CFG_SRV)) {
		res = -EACCES;
		goto exit;
	}

	res = (addr->scope > 0) ?
		tipc_publish(&tsk->port, addr->scope, &addr->addr.nameseq) :
		tipc_withdraw(&tsk->port, -addr->scope, &addr->addr.nameseq);
exit:
	release_sock(sk);
	return res;
}

/**
 * tipc_getname - get port ID of socket or peer socket
 * @sock: socket structure
 * @uaddr: area for returned socket address
 * @uaddr_len: area for returned length of socket address
 * @peer: 0 = own ID, 1 = current peer ID, 2 = current/former peer ID
 *
 * Returns 0 on success, errno otherwise
 *
 * NOTE: This routine doesn't need to take the socket lock since it only
 *       accesses socket information that is unchanging (or which changes in
 *       a completely predictable manner).
 */
static int tipc_getname(struct socket *sock, struct sockaddr *uaddr,
			int *uaddr_len, int peer)
{
	struct sockaddr_tipc *addr = (struct sockaddr_tipc *)uaddr;
	struct tipc_sock *tsk = tipc_sk(sock->sk);

	memset(addr, 0, sizeof(*addr));
	if (peer) {
		if ((sock->state != SS_CONNECTED) &&
			((peer != 2) || (sock->state != SS_DISCONNECTING)))
			return -ENOTCONN;
		addr->addr.id.ref = tipc_port_peerport(&tsk->port);
		addr->addr.id.node = tipc_port_peernode(&tsk->port);
	} else {
		addr->addr.id.ref = tsk->port.ref;
		addr->addr.id.node = tipc_own_addr;
	}

	*uaddr_len = sizeof(*addr);
	addr->addrtype = TIPC_ADDR_ID;
	addr->family = AF_TIPC;
	addr->scope = 0;
	addr->addr.name.domain = 0;

	return 0;
}

/**
 * tipc_poll - read and possibly block on pollmask
 * @file: file structure associated with the socket
 * @sock: socket for which to calculate the poll bits
 * @wait: ???
 *
 * Returns pollmask value
 *
 * COMMENTARY:
 * It appears that the usual socket locking mechanisms are not useful here
 * since the pollmask info is potentially out-of-date the moment this routine
 * exits.  TCP and other protocols seem to rely on higher level poll routines
 * to handle any preventable race conditions, so TIPC will do the same ...
 *
 * TIPC sets the returned events as follows:
 *
 * socket state		flags set
 * ------------		---------
 * unconnected		no read flags
 *			POLLOUT if port is not congested
 *
 * connecting		POLLIN/POLLRDNORM if ACK/NACK in rx queue
 *			no write flags
 *
 * connected		POLLIN/POLLRDNORM if data in rx queue
 *			POLLOUT if port is not congested
 *
 * disconnecting	POLLIN/POLLRDNORM/POLLHUP
 *			no write flags
 *
 * listening		POLLIN if SYN in rx queue
 *			no write flags
 *
 * ready		POLLIN/POLLRDNORM if data in rx queue
 * [connectionless]	POLLOUT (since port cannot be congested)
 *
 * IMPORTANT: The fact that a read or write operation is indicated does NOT
 * imply that the operation will succeed, merely that it should be performed
 * and will not block.
 */
static unsigned int tipc_poll(struct file *file, struct socket *sock,
			      poll_table *wait)
{
	struct sock *sk = sock->sk;
	struct tipc_sock *tsk = tipc_sk(sk);
	u32 mask = 0;

	sock_poll_wait(file, sk_sleep(sk), wait);

	switch ((int)sock->state) {
	case SS_UNCONNECTED:
		if (!tsk->port.congested)
			mask |= POLLOUT;
		break;
	case SS_READY:
	case SS_CONNECTED:
		if (!tsk->port.congested)
			mask |= POLLOUT;
		/* fall thru' */
	case SS_CONNECTING:
	case SS_LISTENING:
		if (!skb_queue_empty(&sk->sk_receive_queue))
			mask |= (POLLIN | POLLRDNORM);
		break;
	case SS_DISCONNECTING:
		mask = (POLLIN | POLLRDNORM | POLLHUP);
		break;
	}

	return mask;
}

/**
 * dest_name_check - verify user is permitted to send to specified port name
 * @dest: destination address
 * @m: descriptor for message to be sent
 *
 * Prevents restricted configuration commands from being issued by
 * unauthorized users.
 *
 * Returns 0 if permission is granted, otherwise errno
 */
static int dest_name_check(struct sockaddr_tipc *dest, struct msghdr *m)
{
	struct tipc_cfg_msg_hdr hdr;

	if (likely(dest->addr.name.name.type >= TIPC_RESERVED_TYPES))
		return 0;
	if (likely(dest->addr.name.name.type == TIPC_TOP_SRV))
		return 0;
	if (likely(dest->addr.name.name.type != TIPC_CFG_SRV))
		return -EACCES;

	if (!m->msg_iovlen || (m->msg_iov[0].iov_len < sizeof(hdr)))
		return -EMSGSIZE;
	if (copy_from_user(&hdr, m->msg_iov[0].iov_base, sizeof(hdr)))
		return -EFAULT;
	if ((ntohs(hdr.tcm_type) & 0xC000) && (!capable(CAP_NET_ADMIN)))
		return -EACCES;

	return 0;
}

static int tipc_wait_for_sndmsg(struct socket *sock, long *timeo_p)
{
	struct sock *sk = sock->sk;
	struct tipc_sock *tsk = tipc_sk(sk);
	DEFINE_WAIT(wait);
	int done;

	do {
		int err = sock_error(sk);
		if (err)
			return err;
		if (sock->state == SS_DISCONNECTING)
			return -EPIPE;
		if (!*timeo_p)
			return -EAGAIN;
		if (signal_pending(current))
			return sock_intr_errno(*timeo_p);

		prepare_to_wait(sk_sleep(sk), &wait, TASK_INTERRUPTIBLE);
		done = sk_wait_event(sk, timeo_p, !tsk->port.congested);
		finish_wait(sk_sleep(sk), &wait);
	} while (!done);
	return 0;
}


/**
 * tipc_sendmsg - send message in connectionless manner
 * @iocb: if NULL, indicates that socket lock is already held
 * @sock: socket structure
 * @m: message to send
 * @total_len: length of message
 *
 * Message must have an destination specified explicitly.
 * Used for SOCK_RDM and SOCK_DGRAM messages,
 * and for 'SYN' messages on SOCK_SEQPACKET and SOCK_STREAM connections.
 * (Note: 'SYN+' is prohibited on SOCK_STREAM.)
 *
 * Returns the number of bytes sent on success, or errno otherwise
 */
static int tipc_sendmsg(struct kiocb *iocb, struct socket *sock,
			struct msghdr *m, size_t total_len)
{
	struct sock *sk = sock->sk;
	struct tipc_sock *tsk = tipc_sk(sk);
	struct tipc_port *port = &tsk->port;
	DECLARE_SOCKADDR(struct sockaddr_tipc *, dest, m->msg_name);
	int needs_conn;
	long timeo;
	int res = -EINVAL;

	if (unlikely(!dest))
		return -EDESTADDRREQ;
	if (unlikely((m->msg_namelen < sizeof(*dest)) ||
		     (dest->family != AF_TIPC)))
		return -EINVAL;
	if (total_len > TIPC_MAX_USER_MSG_SIZE)
		return -EMSGSIZE;

	if (iocb)
		lock_sock(sk);

	needs_conn = (sock->state != SS_READY);
	if (unlikely(needs_conn)) {
		if (sock->state == SS_LISTENING) {
			res = -EPIPE;
			goto exit;
		}
		if (sock->state != SS_UNCONNECTED) {
			res = -EISCONN;
			goto exit;
		}
		if (tsk->port.published) {
			res = -EOPNOTSUPP;
			goto exit;
		}
		if (dest->addrtype == TIPC_ADDR_NAME) {
			tsk->port.conn_type = dest->addr.name.name.type;
			tsk->port.conn_instance = dest->addr.name.name.instance;
		}

		/* Abort any pending connection attempts (very unlikely) */
		reject_rx_queue(sk);
	}

	timeo = sock_sndtimeo(sk, m->msg_flags & MSG_DONTWAIT);
	do {
		if (dest->addrtype == TIPC_ADDR_NAME) {
			res = dest_name_check(dest, m);
			if (res)
				break;
			res = tipc_send2name(port,
					     &dest->addr.name.name,
					     dest->addr.name.domain,
					     m->msg_iov,
					     total_len);
		} else if (dest->addrtype == TIPC_ADDR_ID) {
			res = tipc_send2port(port,
					     &dest->addr.id,
					     m->msg_iov,
					     total_len);
		} else if (dest->addrtype == TIPC_ADDR_MCAST) {
			if (needs_conn) {
				res = -EOPNOTSUPP;
				break;
			}
			res = dest_name_check(dest, m);
			if (res)
				break;
			res = tipc_port_mcast_xmit(port,
						   &dest->addr.nameseq,
						   m->msg_iov,
						   total_len);
		}
		if (likely(res != -ELINKCONG)) {
			if (needs_conn && (res >= 0))
				sock->state = SS_CONNECTING;
			break;
		}
		res = tipc_wait_for_sndmsg(sock, &timeo);
		if (res)
			break;
	} while (1);

exit:
	if (iocb)
		release_sock(sk);
	return res;
}

static int tipc_wait_for_sndpkt(struct socket *sock, long *timeo_p)
{
	struct sock *sk = sock->sk;
	struct tipc_sock *tsk = tipc_sk(sk);
	struct tipc_port *port = &tsk->port;
	DEFINE_WAIT(wait);
	int done;

	do {
		int err = sock_error(sk);
		if (err)
			return err;
		if (sock->state == SS_DISCONNECTING)
			return -EPIPE;
		else if (sock->state != SS_CONNECTED)
			return -ENOTCONN;
		if (!*timeo_p)
			return -EAGAIN;
		if (signal_pending(current))
			return sock_intr_errno(*timeo_p);

		prepare_to_wait(sk_sleep(sk), &wait, TASK_INTERRUPTIBLE);
		done = sk_wait_event(sk, timeo_p,
				     (!port->congested || !port->connected));
		finish_wait(sk_sleep(sk), &wait);
	} while (!done);
	return 0;
}

/**
 * tipc_send_packet - send a connection-oriented message
 * @iocb: if NULL, indicates that socket lock is already held
 * @sock: socket structure
 * @m: message to send
 * @total_len: length of message
 *
 * Used for SOCK_SEQPACKET messages and SOCK_STREAM data.
 *
 * Returns the number of bytes sent on success, or errno otherwise
 */
static int tipc_send_packet(struct kiocb *iocb, struct socket *sock,
			    struct msghdr *m, size_t total_len)
{
	struct sock *sk = sock->sk;
	struct tipc_sock *tsk = tipc_sk(sk);
	DECLARE_SOCKADDR(struct sockaddr_tipc *, dest, m->msg_name);
	int res = -EINVAL;
	long timeo;

	/* Handle implied connection establishment */
	if (unlikely(dest))
		return tipc_sendmsg(iocb, sock, m, total_len);

	if (total_len > TIPC_MAX_USER_MSG_SIZE)
		return -EMSGSIZE;

	if (iocb)
		lock_sock(sk);

	if (unlikely(sock->state != SS_CONNECTED)) {
		if (sock->state == SS_DISCONNECTING)
			res = -EPIPE;
		else
			res = -ENOTCONN;
		goto exit;
	}

	timeo = sock_sndtimeo(sk, m->msg_flags & MSG_DONTWAIT);
	do {
		res = tipc_send(&tsk->port, m->msg_iov, total_len);
		if (likely(res != -ELINKCONG))
			break;
		res = tipc_wait_for_sndpkt(sock, &timeo);
		if (res)
			break;
	} while (1);
exit:
	if (iocb)
		release_sock(sk);
	return res;
}

/**
 * tipc_send_stream - send stream-oriented data
 * @iocb: (unused)
 * @sock: socket structure
 * @m: data to send
 * @total_len: total length of data to be sent
 *
 * Used for SOCK_STREAM data.
 *
 * Returns the number of bytes sent on success (or partial success),
 * or errno if no data sent
 */
static int tipc_send_stream(struct kiocb *iocb, struct socket *sock,
			    struct msghdr *m, size_t total_len)
{
	struct sock *sk = sock->sk;
	struct tipc_sock *tsk = tipc_sk(sk);
	struct msghdr my_msg;
	struct iovec my_iov;
	struct iovec *curr_iov;
	int curr_iovlen;
	char __user *curr_start;
	u32 hdr_size;
	int curr_left;
	int bytes_to_send;
	int bytes_sent;
	int res;

	lock_sock(sk);

	/* Handle special cases where there is no connection */
	if (unlikely(sock->state != SS_CONNECTED)) {
		if (sock->state == SS_UNCONNECTED)
			res = tipc_send_packet(NULL, sock, m, total_len);
		else
			res = sock->state == SS_DISCONNECTING ? -EPIPE : -ENOTCONN;
		goto exit;
	}

	if (unlikely(m->msg_name)) {
		res = -EISCONN;
		goto exit;
	}

	if (total_len > (unsigned int)INT_MAX) {
		res = -EMSGSIZE;
		goto exit;
	}

	/*
	 * Send each iovec entry using one or more messages
	 *
	 * Note: This algorithm is good for the most likely case
	 * (i.e. one large iovec entry), but could be improved to pass sets
	 * of small iovec entries into send_packet().
	 */
	curr_iov = m->msg_iov;
	curr_iovlen = m->msg_iovlen;
	my_msg.msg_iov = &my_iov;
	my_msg.msg_iovlen = 1;
	my_msg.msg_flags = m->msg_flags;
	my_msg.msg_name = NULL;
	bytes_sent = 0;

	hdr_size = msg_hdr_sz(&tsk->port.phdr);

	while (curr_iovlen--) {
		curr_start = curr_iov->iov_base;
		curr_left = curr_iov->iov_len;

		while (curr_left) {
			bytes_to_send = tsk->port.max_pkt - hdr_size;
			if (bytes_to_send > TIPC_MAX_USER_MSG_SIZE)
				bytes_to_send = TIPC_MAX_USER_MSG_SIZE;
			if (curr_left < bytes_to_send)
				bytes_to_send = curr_left;
			my_iov.iov_base = curr_start;
			my_iov.iov_len = bytes_to_send;
			res = tipc_send_packet(NULL, sock, &my_msg,
					       bytes_to_send);
			if (res < 0) {
				if (bytes_sent)
					res = bytes_sent;
				goto exit;
			}
			curr_left -= bytes_to_send;
			curr_start += bytes_to_send;
			bytes_sent += bytes_to_send;
		}

		curr_iov++;
	}
	res = bytes_sent;
exit:
	release_sock(sk);
	return res;
}

/**
 * auto_connect - complete connection setup to a remote port
 * @tsk: tipc socket structure
 * @msg: peer's response message
 *
 * Returns 0 on success, errno otherwise
 */
static int auto_connect(struct tipc_sock *tsk, struct tipc_msg *msg)
{
	struct tipc_port *port = &tsk->port;
	struct socket *sock = tsk->sk.sk_socket;
	struct tipc_portid peer;

	peer.ref = msg_origport(msg);
	peer.node = msg_orignode(msg);

	__tipc_port_connect(port->ref, port, &peer);

	if (msg_importance(msg) > TIPC_CRITICAL_IMPORTANCE)
		return -EINVAL;
	msg_set_importance(&port->phdr, (u32)msg_importance(msg));
	sock->state = SS_CONNECTED;
	return 0;
}

/**
 * set_orig_addr - capture sender's address for received message
 * @m: descriptor for message info
 * @msg: received message header
 *
 * Note: Address is not captured if not requested by receiver.
 */
static void set_orig_addr(struct msghdr *m, struct tipc_msg *msg)
{
	DECLARE_SOCKADDR(struct sockaddr_tipc *, addr, m->msg_name);

	if (addr) {
		addr->family = AF_TIPC;
		addr->addrtype = TIPC_ADDR_ID;
		memset(&addr->addr, 0, sizeof(addr->addr));
		addr->addr.id.ref = msg_origport(msg);
		addr->addr.id.node = msg_orignode(msg);
		addr->addr.name.domain = 0;	/* could leave uninitialized */
		addr->scope = 0;		/* could leave uninitialized */
		m->msg_namelen = sizeof(struct sockaddr_tipc);
	}
}

/**
 * anc_data_recv - optionally capture ancillary data for received message
 * @m: descriptor for message info
 * @msg: received message header
 * @tport: TIPC port associated with message
 *
 * Note: Ancillary data is not captured if not requested by receiver.
 *
 * Returns 0 if successful, otherwise errno
 */
static int anc_data_recv(struct msghdr *m, struct tipc_msg *msg,
			 struct tipc_port *tport)
{
	u32 anc_data[3];
	u32 err;
	u32 dest_type;
	int has_name;
	int res;

	if (likely(m->msg_controllen == 0))
		return 0;

	/* Optionally capture errored message object(s) */
	err = msg ? msg_errcode(msg) : 0;
	if (unlikely(err)) {
		anc_data[0] = err;
		anc_data[1] = msg_data_sz(msg);
		res = put_cmsg(m, SOL_TIPC, TIPC_ERRINFO, 8, anc_data);
		if (res)
			return res;
		if (anc_data[1]) {
			res = put_cmsg(m, SOL_TIPC, TIPC_RETDATA, anc_data[1],
				       msg_data(msg));
			if (res)
				return res;
		}
	}

	/* Optionally capture message destination object */
	dest_type = msg ? msg_type(msg) : TIPC_DIRECT_MSG;
	switch (dest_type) {
	case TIPC_NAMED_MSG:
		has_name = 1;
		anc_data[0] = msg_nametype(msg);
		anc_data[1] = msg_namelower(msg);
		anc_data[2] = msg_namelower(msg);
		break;
	case TIPC_MCAST_MSG:
		has_name = 1;
		anc_data[0] = msg_nametype(msg);
		anc_data[1] = msg_namelower(msg);
		anc_data[2] = msg_nameupper(msg);
		break;
	case TIPC_CONN_MSG:
		has_name = (tport->conn_type != 0);
		anc_data[0] = tport->conn_type;
		anc_data[1] = tport->conn_instance;
		anc_data[2] = tport->conn_instance;
		break;
	default:
		has_name = 0;
	}
	if (has_name) {
		res = put_cmsg(m, SOL_TIPC, TIPC_DESTNAME, 12, anc_data);
		if (res)
			return res;
	}

	return 0;
}

static int tipc_wait_for_rcvmsg(struct socket *sock, long timeo)
{
	struct sock *sk = sock->sk;
	DEFINE_WAIT(wait);
	int err;

	for (;;) {
		prepare_to_wait(sk_sleep(sk), &wait, TASK_INTERRUPTIBLE);
		if (timeo && skb_queue_empty(&sk->sk_receive_queue)) {
			if (sock->state == SS_DISCONNECTING) {
				err = -ENOTCONN;
				break;
			}
			release_sock(sk);
			timeo = schedule_timeout(timeo);
			lock_sock(sk);
		}
		err = 0;
		if (!skb_queue_empty(&sk->sk_receive_queue))
			break;
		err = sock_intr_errno(timeo);
		if (signal_pending(current))
			break;
		err = -EAGAIN;
		if (!timeo)
			break;
	}
	finish_wait(sk_sleep(sk), &wait);
	return err;
}

/**
 * tipc_recvmsg - receive packet-oriented message
 * @iocb: (unused)
 * @m: descriptor for message info
 * @buf_len: total size of user buffer area
 * @flags: receive flags
 *
 * Used for SOCK_DGRAM, SOCK_RDM, and SOCK_SEQPACKET messages.
 * If the complete message doesn't fit in user area, truncate it.
 *
 * Returns size of returned message data, errno otherwise
 */
static int tipc_recvmsg(struct kiocb *iocb, struct socket *sock,
			struct msghdr *m, size_t buf_len, int flags)
{
	struct sock *sk = sock->sk;
	struct tipc_sock *tsk = tipc_sk(sk);
	struct tipc_port *port = &tsk->port;
	struct sk_buff *buf;
	struct tipc_msg *msg;
	long timeo;
	unsigned int sz;
	u32 err;
	int res;

	/* Catch invalid receive requests */
	if (unlikely(!buf_len))
		return -EINVAL;

	lock_sock(sk);

	if (unlikely(sock->state == SS_UNCONNECTED)) {
		res = -ENOTCONN;
		goto exit;
	}

	timeo = sock_rcvtimeo(sk, flags & MSG_DONTWAIT);
restart:

	/* Look for a message in receive queue; wait if necessary */
	res = tipc_wait_for_rcvmsg(sock, timeo);
	if (res)
		goto exit;

	/* Look at first message in receive queue */
	buf = skb_peek(&sk->sk_receive_queue);
	msg = buf_msg(buf);
	sz = msg_data_sz(msg);
	err = msg_errcode(msg);

	/* Discard an empty non-errored message & try again */
	if ((!sz) && (!err)) {
		advance_rx_queue(sk);
		goto restart;
	}

	/* Capture sender's address (optional) */
	set_orig_addr(m, msg);

	/* Capture ancillary data (optional) */
	res = anc_data_recv(m, msg, port);
	if (res)
		goto exit;

	/* Capture message data (if valid) & compute return value (always) */
	if (!err) {
		if (unlikely(buf_len < sz)) {
			sz = buf_len;
			m->msg_flags |= MSG_TRUNC;
		}
		res = skb_copy_datagram_iovec(buf, msg_hdr_sz(msg),
					      m->msg_iov, sz);
		if (res)
			goto exit;
		res = sz;
	} else {
		if ((sock->state == SS_READY) ||
		    ((err == TIPC_CONN_SHUTDOWN) || m->msg_control))
			res = 0;
		else
			res = -ECONNRESET;
	}

	/* Consume received message (optional) */
	if (likely(!(flags & MSG_PEEK))) {
		if ((sock->state != SS_READY) &&
		    (++port->conn_unacked >= TIPC_FLOW_CONTROL_WIN))
			tipc_acknowledge(port->ref, port->conn_unacked);
		advance_rx_queue(sk);
	}
exit:
	release_sock(sk);
	return res;
}

/**
 * tipc_recv_stream - receive stream-oriented data
 * @iocb: (unused)
 * @m: descriptor for message info
 * @buf_len: total size of user buffer area
 * @flags: receive flags
 *
 * Used for SOCK_STREAM messages only.  If not enough data is available
 * will optionally wait for more; never truncates data.
 *
 * Returns size of returned message data, errno otherwise
 */
static int tipc_recv_stream(struct kiocb *iocb, struct socket *sock,
			    struct msghdr *m, size_t buf_len, int flags)
{
	struct sock *sk = sock->sk;
	struct tipc_sock *tsk = tipc_sk(sk);
	struct tipc_port *port = &tsk->port;
	struct sk_buff *buf;
	struct tipc_msg *msg;
	long timeo;
	unsigned int sz;
	int sz_to_copy, target, needed;
	int sz_copied = 0;
	u32 err;
	int res = 0;

	/* Catch invalid receive attempts */
	if (unlikely(!buf_len))
		return -EINVAL;

	lock_sock(sk);

	if (unlikely(sock->state == SS_UNCONNECTED)) {
		res = -ENOTCONN;
		goto exit;
	}

	target = sock_rcvlowat(sk, flags & MSG_WAITALL, buf_len);
	timeo = sock_rcvtimeo(sk, flags & MSG_DONTWAIT);

restart:
	/* Look for a message in receive queue; wait if necessary */
	res = tipc_wait_for_rcvmsg(sock, timeo);
	if (res)
		goto exit;

	/* Look at first message in receive queue */
	buf = skb_peek(&sk->sk_receive_queue);
	msg = buf_msg(buf);
	sz = msg_data_sz(msg);
	err = msg_errcode(msg);

	/* Discard an empty non-errored message & try again */
	if ((!sz) && (!err)) {
		advance_rx_queue(sk);
		goto restart;
	}

	/* Optionally capture sender's address & ancillary data of first msg */
	if (sz_copied == 0) {
		set_orig_addr(m, msg);
		res = anc_data_recv(m, msg, port);
		if (res)
			goto exit;
	}

	/* Capture message data (if valid) & compute return value (always) */
	if (!err) {
		u32 offset = (u32)(unsigned long)(TIPC_SKB_CB(buf)->handle);

		sz -= offset;
		needed = (buf_len - sz_copied);
		sz_to_copy = (sz <= needed) ? sz : needed;

		res = skb_copy_datagram_iovec(buf, msg_hdr_sz(msg) + offset,
					      m->msg_iov, sz_to_copy);
		if (res)
			goto exit;

		sz_copied += sz_to_copy;

		if (sz_to_copy < sz) {
			if (!(flags & MSG_PEEK))
				TIPC_SKB_CB(buf)->handle =
				(void *)(unsigned long)(offset + sz_to_copy);
			goto exit;
		}
	} else {
		if (sz_copied != 0)
			goto exit; /* can't add error msg to valid data */

		if ((err == TIPC_CONN_SHUTDOWN) || m->msg_control)
			res = 0;
		else
			res = -ECONNRESET;
	}

	/* Consume received message (optional) */
	if (likely(!(flags & MSG_PEEK))) {
		if (unlikely(++port->conn_unacked >= TIPC_FLOW_CONTROL_WIN))
			tipc_acknowledge(port->ref, port->conn_unacked);
		advance_rx_queue(sk);
	}

	/* Loop around if more data is required */
	if ((sz_copied < buf_len) &&	/* didn't get all requested data */
	    (!skb_queue_empty(&sk->sk_receive_queue) ||
	    (sz_copied < target)) &&	/* and more is ready or required */
	    (!(flags & MSG_PEEK)) &&	/* and aren't just peeking at data */
	    (!err))			/* and haven't reached a FIN */
		goto restart;

exit:
	release_sock(sk);
	return sz_copied ? sz_copied : res;
}

/**
 * tipc_write_space - wake up thread if port congestion is released
 * @sk: socket
 */
static void tipc_write_space(struct sock *sk)
{
	struct socket_wq *wq;

	rcu_read_lock();
	wq = rcu_dereference(sk->sk_wq);
	if (wq_has_sleeper(wq))
		wake_up_interruptible_sync_poll(&wq->wait, POLLOUT |
						POLLWRNORM | POLLWRBAND);
	rcu_read_unlock();
}

/**
 * tipc_data_ready - wake up threads to indicate messages have been received
 * @sk: socket
 * @len: the length of messages
 */
static void tipc_data_ready(struct sock *sk)
{
	struct socket_wq *wq;

	rcu_read_lock();
	wq = rcu_dereference(sk->sk_wq);
	if (wq_has_sleeper(wq))
		wake_up_interruptible_sync_poll(&wq->wait, POLLIN |
						POLLRDNORM | POLLRDBAND);
	rcu_read_unlock();
}

/**
 * filter_connect - Handle all incoming messages for a connection-based socket
 * @tsk: TIPC socket
 * @msg: message
 *
 * Returns TIPC error status code and socket error status code
 * once it encounters some errors
 */
static u32 filter_connect(struct tipc_sock *tsk, struct sk_buff **buf)
{
	struct sock *sk = &tsk->sk;
	struct tipc_port *port = &tsk->port;
	struct socket *sock = sk->sk_socket;
	struct tipc_msg *msg = buf_msg(*buf);

	u32 retval = TIPC_ERR_NO_PORT;
	int res;

	if (msg_mcast(msg))
		return retval;

	switch ((int)sock->state) {
	case SS_CONNECTED:
		/* Accept only connection-based messages sent by peer */
		if (msg_connected(msg) && tipc_port_peer_msg(port, msg)) {
			if (unlikely(msg_errcode(msg))) {
				sock->state = SS_DISCONNECTING;
				__tipc_port_disconnect(port);
			}
			retval = TIPC_OK;
		}
		break;
	case SS_CONNECTING:
		/* Accept only ACK or NACK message */
		if (unlikely(msg_errcode(msg))) {
			sock->state = SS_DISCONNECTING;
			sk->sk_err = ECONNREFUSED;
			retval = TIPC_OK;
			break;
		}

		if (unlikely(!msg_connected(msg)))
			break;

		res = auto_connect(tsk, msg);
		if (res) {
			sock->state = SS_DISCONNECTING;
			sk->sk_err = -res;
			retval = TIPC_OK;
			break;
		}

		/* If an incoming message is an 'ACK-', it should be
		 * discarded here because it doesn't contain useful
		 * data. In addition, we should try to wake up
		 * connect() routine if sleeping.
		 */
		if (msg_data_sz(msg) == 0) {
			kfree_skb(*buf);
			*buf = NULL;
			if (waitqueue_active(sk_sleep(sk)))
				wake_up_interruptible(sk_sleep(sk));
		}
		retval = TIPC_OK;
		break;
	case SS_LISTENING:
	case SS_UNCONNECTED:
		/* Accept only SYN message */
		if (!msg_connected(msg) && !(msg_errcode(msg)))
			retval = TIPC_OK;
		break;
	case SS_DISCONNECTING:
		break;
	default:
		pr_err("Unknown socket state %u\n", sock->state);
	}
	return retval;
}

/**
 * rcvbuf_limit - get proper overload limit of socket receive queue
 * @sk: socket
 * @buf: message
 *
 * For all connection oriented messages, irrespective of importance,
 * the default overload value (i.e. 67MB) is set as limit.
 *
 * For all connectionless messages, by default new queue limits are
 * as belows:
 *
 * TIPC_LOW_IMPORTANCE       (4 MB)
 * TIPC_MEDIUM_IMPORTANCE    (8 MB)
 * TIPC_HIGH_IMPORTANCE      (16 MB)
 * TIPC_CRITICAL_IMPORTANCE  (32 MB)
 *
 * Returns overload limit according to corresponding message importance
 */
static unsigned int rcvbuf_limit(struct sock *sk, struct sk_buff *buf)
{
	struct tipc_msg *msg = buf_msg(buf);

	if (msg_connected(msg))
		return sysctl_tipc_rmem[2];

	return sk->sk_rcvbuf >> TIPC_CRITICAL_IMPORTANCE <<
		msg_importance(msg);
}

/**
 * filter_rcv - validate incoming message
 * @sk: socket
 * @buf: message
 *
 * Enqueues message on receive queue if acceptable; optionally handles
 * disconnect indication for a connected socket.
 *
 * Called with socket lock already taken; port lock may also be taken.
 *
 * Returns TIPC error status code (TIPC_OK if message is not to be rejected)
 */
static u32 filter_rcv(struct sock *sk, struct sk_buff *buf)
{
	struct socket *sock = sk->sk_socket;
	struct tipc_sock *tsk = tipc_sk(sk);
	struct tipc_msg *msg = buf_msg(buf);
	unsigned int limit = rcvbuf_limit(sk, buf);
	u32 res = TIPC_OK;

	/* Reject message if it is wrong sort of message for socket */
	if (msg_type(msg) > TIPC_DIRECT_MSG)
		return TIPC_ERR_NO_PORT;

	if (sock->state == SS_READY) {
		if (msg_connected(msg))
			return TIPC_ERR_NO_PORT;
	} else {
		res = filter_connect(tsk, &buf);
		if (res != TIPC_OK || buf == NULL)
			return res;
	}

	/* Reject message if there isn't room to queue it */
	if (sk_rmem_alloc_get(sk) + buf->truesize >= limit)
		return TIPC_ERR_OVERLOAD;

	/* Enqueue message */
	TIPC_SKB_CB(buf)->handle = NULL;
	__skb_queue_tail(&sk->sk_receive_queue, buf);
	skb_set_owner_r(buf, sk);

	sk->sk_data_ready(sk);
	return TIPC_OK;
}

/**
 * backlog_rcv - handle incoming message from backlog queue
 * @sk: socket
 * @buf: message
 *
 * Caller must hold socket lock, but not port lock.
 *
 * Returns 0
 */
static int backlog_rcv(struct sock *sk, struct sk_buff *buf)
{
	u32 res;

	res = filter_rcv(sk, buf);
	if (res)
		tipc_reject_msg(buf, res);
	return 0;
}

/**
 * tipc_sk_rcv - handle incoming message
 * @sk:  socket receiving message
 * @buf: message
 *
 * Called with port lock already taken.
 *
 * Returns TIPC error status code (TIPC_OK if message is not to be rejected)
 */
u32 tipc_sk_rcv(struct sock *sk, struct sk_buff *buf)
{
	u32 res;

	/*
	 * Process message if socket is unlocked; otherwise add to backlog queue
	 *
	 * This code is based on sk_receive_skb(), but must be distinct from it
	 * since a TIPC-specific filter/reject mechanism is utilized
	 */
	bh_lock_sock(sk);
	if (!sock_owned_by_user(sk)) {
		res = filter_rcv(sk, buf);
	} else {
		if (sk_add_backlog(sk, buf, rcvbuf_limit(sk, buf)))
			res = TIPC_ERR_OVERLOAD;
		else
			res = TIPC_OK;
	}
	bh_unlock_sock(sk);

	return res;
}

static int tipc_wait_for_connect(struct socket *sock, long *timeo_p)
{
	struct sock *sk = sock->sk;
	DEFINE_WAIT(wait);
	int done;

	do {
		int err = sock_error(sk);
		if (err)
			return err;
		if (!*timeo_p)
			return -ETIMEDOUT;
		if (signal_pending(current))
			return sock_intr_errno(*timeo_p);

		prepare_to_wait(sk_sleep(sk), &wait, TASK_INTERRUPTIBLE);
		done = sk_wait_event(sk, timeo_p, sock->state != SS_CONNECTING);
		finish_wait(sk_sleep(sk), &wait);
	} while (!done);
	return 0;
}

/**
 * tipc_connect - establish a connection to another TIPC port
 * @sock: socket structure
 * @dest: socket address for destination port
 * @destlen: size of socket address data structure
 * @flags: file-related flags associated with socket
 *
 * Returns 0 on success, errno otherwise
 */
static int tipc_connect(struct socket *sock, struct sockaddr *dest,
			int destlen, int flags)
{
	struct sock *sk = sock->sk;
	struct sockaddr_tipc *dst = (struct sockaddr_tipc *)dest;
	struct msghdr m = {NULL,};
	long timeout = (flags & O_NONBLOCK) ? 0 : tipc_sk(sk)->conn_timeout;
	socket_state previous;
	int res;

	lock_sock(sk);

	/* For now, TIPC does not allow use of connect() with DGRAM/RDM types */
	if (sock->state == SS_READY) {
		res = -EOPNOTSUPP;
		goto exit;
	}

	/*
	 * Reject connection attempt using multicast address
	 *
	 * Note: send_msg() validates the rest of the address fields,
	 *       so there's no need to do it here
	 */
	if (dst->addrtype == TIPC_ADDR_MCAST) {
		res = -EINVAL;
		goto exit;
	}

	previous = sock->state;
	switch (sock->state) {
	case SS_UNCONNECTED:
		/* Send a 'SYN-' to destination */
		m.msg_name = dest;
		m.msg_namelen = destlen;

		/* If connect is in non-blocking case, set MSG_DONTWAIT to
		 * indicate send_msg() is never blocked.
		 */
		if (!timeout)
			m.msg_flags = MSG_DONTWAIT;

		res = tipc_sendmsg(NULL, sock, &m, 0);
		if ((res < 0) && (res != -EWOULDBLOCK))
			goto exit;

		/* Just entered SS_CONNECTING state; the only
		 * difference is that return value in non-blocking
		 * case is EINPROGRESS, rather than EALREADY.
		 */
		res = -EINPROGRESS;
	case SS_CONNECTING:
		if (previous == SS_CONNECTING)
			res = -EALREADY;
		if (!timeout)
			goto exit;
		timeout = msecs_to_jiffies(timeout);
		/* Wait until an 'ACK' or 'RST' arrives, or a timeout occurs */
		res = tipc_wait_for_connect(sock, &timeout);
		break;
	case SS_CONNECTED:
		res = -EISCONN;
		break;
	default:
		res = -EINVAL;
		break;
	}
exit:
	release_sock(sk);
	return res;
}

/**
 * tipc_listen - allow socket to listen for incoming connections
 * @sock: socket structure
 * @len: (unused)
 *
 * Returns 0 on success, errno otherwise
 */
static int tipc_listen(struct socket *sock, int len)
{
	struct sock *sk = sock->sk;
	int res;

	lock_sock(sk);

	if (sock->state != SS_UNCONNECTED)
		res = -EINVAL;
	else {
		sock->state = SS_LISTENING;
		res = 0;
	}

	release_sock(sk);
	return res;
}

static int tipc_wait_for_accept(struct socket *sock, long timeo)
{
	struct sock *sk = sock->sk;
	DEFINE_WAIT(wait);
	int err;

	/* True wake-one mechanism for incoming connections: only
	 * one process gets woken up, not the 'whole herd'.
	 * Since we do not 'race & poll' for established sockets
	 * anymore, the common case will execute the loop only once.
	*/
	for (;;) {
		prepare_to_wait_exclusive(sk_sleep(sk), &wait,
					  TASK_INTERRUPTIBLE);
		if (timeo && skb_queue_empty(&sk->sk_receive_queue)) {
			release_sock(sk);
			timeo = schedule_timeout(timeo);
			lock_sock(sk);
		}
		err = 0;
		if (!skb_queue_empty(&sk->sk_receive_queue))
			break;
		err = -EINVAL;
		if (sock->state != SS_LISTENING)
			break;
		err = sock_intr_errno(timeo);
		if (signal_pending(current))
			break;
		err = -EAGAIN;
		if (!timeo)
			break;
	}
	finish_wait(sk_sleep(sk), &wait);
	return err;
}

/**
 * tipc_accept - wait for connection request
 * @sock: listening socket
 * @newsock: new socket that is to be connected
 * @flags: file-related flags associated with socket
 *
 * Returns 0 on success, errno otherwise
 */
static int tipc_accept(struct socket *sock, struct socket *new_sock, int flags)
{
	struct sock *new_sk, *sk = sock->sk;
	struct sk_buff *buf;
	struct tipc_port *new_port;
	struct tipc_msg *msg;
	struct tipc_portid peer;
	u32 new_ref;
	long timeo;
	int res;

	lock_sock(sk);

	if (sock->state != SS_LISTENING) {
		res = -EINVAL;
		goto exit;
	}
	timeo = sock_rcvtimeo(sk, flags & O_NONBLOCK);
	res = tipc_wait_for_accept(sock, timeo);
	if (res)
		goto exit;

	buf = skb_peek(&sk->sk_receive_queue);

	res = tipc_sk_create(sock_net(sock->sk), new_sock, 0, 1);
	if (res)
		goto exit;

	new_sk = new_sock->sk;
	new_port = &tipc_sk(new_sk)->port;
	new_ref = new_port->ref;
	msg = buf_msg(buf);

	/* we lock on new_sk; but lockdep sees the lock on sk */
	lock_sock_nested(new_sk, SINGLE_DEPTH_NESTING);

	/*
	 * Reject any stray messages received by new socket
	 * before the socket lock was taken (very, very unlikely)
	 */
	reject_rx_queue(new_sk);

	/* Connect new socket to it's peer */
	peer.ref = msg_origport(msg);
	peer.node = msg_orignode(msg);
	tipc_port_connect(new_ref, &peer);
	new_sock->state = SS_CONNECTED;

	tipc_port_set_importance(new_port, msg_importance(msg));
	if (msg_named(msg)) {
		new_port->conn_type = msg_nametype(msg);
		new_port->conn_instance = msg_nameinst(msg);
	}

	/*
	 * Respond to 'SYN-' by discarding it & returning 'ACK'-.
	 * Respond to 'SYN+' by queuing it on new socket.
	 */
	if (!msg_data_sz(msg)) {
		struct msghdr m = {NULL,};

		advance_rx_queue(sk);
		tipc_send_packet(NULL, new_sock, &m, 0);
	} else {
		__skb_dequeue(&sk->sk_receive_queue);
		__skb_queue_head(&new_sk->sk_receive_queue, buf);
		skb_set_owner_r(buf, new_sk);
	}
	release_sock(new_sk);
exit:
	release_sock(sk);
	return res;
}

/**
 * tipc_shutdown - shutdown socket connection
 * @sock: socket structure
 * @how: direction to close (must be SHUT_RDWR)
 *
 * Terminates connection (if necessary), then purges socket's receive queue.
 *
 * Returns 0 on success, errno otherwise
 */
static int tipc_shutdown(struct socket *sock, int how)
{
	struct sock *sk = sock->sk;
	struct tipc_sock *tsk = tipc_sk(sk);
	struct tipc_port *port = &tsk->port;
	struct sk_buff *buf;
	int res;

	if (how != SHUT_RDWR)
		return -EINVAL;

	lock_sock(sk);

	switch (sock->state) {
	case SS_CONNECTING:
	case SS_CONNECTED:

restart:
		/* Disconnect and send a 'FIN+' or 'FIN-' message to peer */
		buf = __skb_dequeue(&sk->sk_receive_queue);
		if (buf) {
			if (TIPC_SKB_CB(buf)->handle != NULL) {
				kfree_skb(buf);
				goto restart;
			}
			tipc_port_disconnect(port->ref);
			tipc_reject_msg(buf, TIPC_CONN_SHUTDOWN);
		} else {
			tipc_port_shutdown(port->ref);
		}

		sock->state = SS_DISCONNECTING;

		/* fall through */

	case SS_DISCONNECTING:

		/* Discard any unreceived messages */
		__skb_queue_purge(&sk->sk_receive_queue);

		/* Wake up anyone sleeping in poll */
		sk->sk_state_change(sk);
		res = 0;
		break;

	default:
		res = -ENOTCONN;
	}

	release_sock(sk);
	return res;
}

/**
 * tipc_setsockopt - set socket option
 * @sock: socket structure
 * @lvl: option level
 * @opt: option identifier
 * @ov: pointer to new option value
 * @ol: length of option value
 *
 * For stream sockets only, accepts and ignores all IPPROTO_TCP options
 * (to ease compatibility).
 *
 * Returns 0 on success, errno otherwise
 */
static int tipc_setsockopt(struct socket *sock, int lvl, int opt,
			   char __user *ov, unsigned int ol)
{
	struct sock *sk = sock->sk;
	struct tipc_sock *tsk = tipc_sk(sk);
	struct tipc_port *port = &tsk->port;
	u32 value;
	int res;

	if ((lvl == IPPROTO_TCP) && (sock->type == SOCK_STREAM))
		return 0;
	if (lvl != SOL_TIPC)
		return -ENOPROTOOPT;
	if (ol < sizeof(value))
		return -EINVAL;
	res = get_user(value, (u32 __user *)ov);
	if (res)
		return res;

	lock_sock(sk);

	switch (opt) {
	case TIPC_IMPORTANCE:
		tipc_port_set_importance(port, value);
		break;
	case TIPC_SRC_DROPPABLE:
		if (sock->type != SOCK_STREAM)
			tipc_port_set_unreliable(port, value);
		else
			res = -ENOPROTOOPT;
		break;
	case TIPC_DEST_DROPPABLE:
		tipc_port_set_unreturnable(port, value);
		break;
	case TIPC_CONN_TIMEOUT:
		tipc_sk(sk)->conn_timeout = value;
		/* no need to set "res", since already 0 at this point */
		break;
	default:
		res = -EINVAL;
	}

	release_sock(sk);

	return res;
}

/**
 * tipc_getsockopt - get socket option
 * @sock: socket structure
 * @lvl: option level
 * @opt: option identifier
 * @ov: receptacle for option value
 * @ol: receptacle for length of option value
 *
 * For stream sockets only, returns 0 length result for all IPPROTO_TCP options
 * (to ease compatibility).
 *
 * Returns 0 on success, errno otherwise
 */
static int tipc_getsockopt(struct socket *sock, int lvl, int opt,
			   char __user *ov, int __user *ol)
{
	struct sock *sk = sock->sk;
	struct tipc_sock *tsk = tipc_sk(sk);
	struct tipc_port *port = &tsk->port;
	int len;
	u32 value;
	int res;

	if ((lvl == IPPROTO_TCP) && (sock->type == SOCK_STREAM))
		return put_user(0, ol);
	if (lvl != SOL_TIPC)
		return -ENOPROTOOPT;
	res = get_user(len, ol);
	if (res)
		return res;

	lock_sock(sk);

	switch (opt) {
	case TIPC_IMPORTANCE:
		value = tipc_port_importance(port);
		break;
	case TIPC_SRC_DROPPABLE:
		value = tipc_port_unreliable(port);
		break;
	case TIPC_DEST_DROPPABLE:
		value = tipc_port_unreturnable(port);
		break;
	case TIPC_CONN_TIMEOUT:
		value = tipc_sk(sk)->conn_timeout;
		/* no need to set "res", since already 0 at this point */
		break;
	case TIPC_NODE_RECVQ_DEPTH:
		value = 0; /* was tipc_queue_size, now obsolete */
		break;
	case TIPC_SOCK_RECVQ_DEPTH:
		value = skb_queue_len(&sk->sk_receive_queue);
		break;
	default:
		res = -EINVAL;
	}

	release_sock(sk);

	if (res)
		return res;	/* "get" failed */

	if (len < sizeof(value))
		return -EINVAL;

	if (copy_to_user(ov, &value, sizeof(value)))
		return -EFAULT;

	return put_user(sizeof(value), ol);
}

/* Protocol switches for the various types of TIPC sockets */

static const struct proto_ops msg_ops = {
	.owner		= THIS_MODULE,
	.family		= AF_TIPC,
	.release	= tipc_release,
	.bind		= tipc_bind,
	.connect	= tipc_connect,
	.socketpair	= sock_no_socketpair,
	.accept		= sock_no_accept,
	.getname	= tipc_getname,
	.poll		= tipc_poll,
	.ioctl		= sock_no_ioctl,
	.listen		= sock_no_listen,
	.shutdown	= tipc_shutdown,
	.setsockopt	= tipc_setsockopt,
	.getsockopt	= tipc_getsockopt,
	.sendmsg	= tipc_sendmsg,
	.recvmsg	= tipc_recvmsg,
	.mmap		= sock_no_mmap,
	.sendpage	= sock_no_sendpage
};

static const struct proto_ops packet_ops = {
	.owner		= THIS_MODULE,
	.family		= AF_TIPC,
	.release	= tipc_release,
	.bind		= tipc_bind,
	.connect	= tipc_connect,
	.socketpair	= sock_no_socketpair,
	.accept		= tipc_accept,
	.getname	= tipc_getname,
	.poll		= tipc_poll,
	.ioctl		= sock_no_ioctl,
	.listen		= tipc_listen,
	.shutdown	= tipc_shutdown,
	.setsockopt	= tipc_setsockopt,
	.getsockopt	= tipc_getsockopt,
	.sendmsg	= tipc_send_packet,
	.recvmsg	= tipc_recvmsg,
	.mmap		= sock_no_mmap,
	.sendpage	= sock_no_sendpage
};

static const struct proto_ops stream_ops = {
	.owner		= THIS_MODULE,
	.family		= AF_TIPC,
	.release	= tipc_release,
	.bind		= tipc_bind,
	.connect	= tipc_connect,
	.socketpair	= sock_no_socketpair,
	.accept		= tipc_accept,
	.getname	= tipc_getname,
	.poll		= tipc_poll,
	.ioctl		= sock_no_ioctl,
	.listen		= tipc_listen,
	.shutdown	= tipc_shutdown,
	.setsockopt	= tipc_setsockopt,
	.getsockopt	= tipc_getsockopt,
	.sendmsg	= tipc_send_stream,
	.recvmsg	= tipc_recv_stream,
	.mmap		= sock_no_mmap,
	.sendpage	= sock_no_sendpage
};

static const struct net_proto_family tipc_family_ops = {
	.owner		= THIS_MODULE,
	.family		= AF_TIPC,
	.create		= tipc_sk_create
};

static struct proto tipc_proto = {
	.name		= "TIPC",
	.owner		= THIS_MODULE,
	.obj_size	= sizeof(struct tipc_sock),
	.sysctl_rmem	= sysctl_tipc_rmem
};

static struct proto tipc_proto_kern = {
	.name		= "TIPC",
	.obj_size	= sizeof(struct tipc_sock),
	.sysctl_rmem	= sysctl_tipc_rmem
};

/**
 * tipc_socket_init - initialize TIPC socket interface
 *
 * Returns 0 on success, errno otherwise
 */
int tipc_socket_init(void)
{
	int res;

	res = proto_register(&tipc_proto, 1);
	if (res) {
		pr_err("Failed to register TIPC protocol type\n");
		goto out;
	}

	res = sock_register(&tipc_family_ops);
	if (res) {
		pr_err("Failed to register TIPC socket type\n");
		proto_unregister(&tipc_proto);
		goto out;
	}
 out:
	return res;
}

/**
 * tipc_socket_stop - stop TIPC socket interface
 */
void tipc_socket_stop(void)
{
	sock_unregister(tipc_family_ops.family);
	proto_unregister(&tipc_proto);
}
