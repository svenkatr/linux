/* visorchannel.h
 *
 * Copyright � 2010 - 2013 UNISYS CORPORATION
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 * NON INFRINGEMENT.  See the GNU General Public License for more
 * details.
 */

#ifndef __VISORCHANNEL_H__
#define __VISORCHANNEL_H__

#include "commontypes.h"
#include "memregion.h"
#include "channel.h"
#ifndef HOSTADDRESS
#define HOSTADDRESS U64
#endif
#ifndef BOOL
#define BOOL int
#endif

/* VISORCHANNEL is an opaque structure to users.
 * Fields are declared only in the implementation .c files.
 */
typedef struct VISORCHANNEL_Tag VISORCHANNEL;

/* Note that for visorchannel_create() and visorchannel_create_overlapped(),
 * <channelBytes> and <guid> arguments may be 0 if we are a channel CLIENT.
 * In this case, the values can simply be read from the channel header.
 */
VISORCHANNEL *visorchannel_create(HOSTADDRESS physaddr,
				  ulong channelBytes, GUID guid);
VISORCHANNEL *visorchannel_create_overlapped(ulong channelBytes,
					     VISORCHANNEL *parent, ulong off,
					     GUID guid);
VISORCHANNEL *visorchannel_create_with_lock(HOSTADDRESS physaddr,
					    ulong channelBytes, GUID guid);
VISORCHANNEL *visorchannel_create_overlapped_with_lock(ulong channelBytes,
						       VISORCHANNEL *parent,
						       ulong off, GUID guid);
void visorchannel_destroy(VISORCHANNEL *channel);
int visorchannel_read(VISORCHANNEL *channel, ulong offset,
		      void *local, ulong nbytes);
int visorchannel_write(VISORCHANNEL *channel, ulong offset,
		       void *local, ulong nbytes);
int visorchannel_clear(VISORCHANNEL *channel, ulong offset,
		       U8 ch, ulong nbytes);
BOOL visorchannel_signalremove(VISORCHANNEL *channel, U32 queue, void *msg);
BOOL visorchannel_signalinsert(VISORCHANNEL *channel, U32 queue, void *msg);
int visorchannel_signalqueue_slots_avail(VISORCHANNEL *channel, U32 queue);
int visorchannel_signalqueue_max_slots(VISORCHANNEL *channel, U32 queue);

HOSTADDRESS visorchannel_get_physaddr(VISORCHANNEL *channel);
ulong visorchannel_get_nbytes(VISORCHANNEL *channel);
char *visorchannel_id(VISORCHANNEL *channel, char *s);
char *visorchannel_zoneid(VISORCHANNEL *channel, char *s);
U64 visorchannel_get_clientpartition(VISORCHANNEL *channel);
GUID visorchannel_get_GUID(VISORCHANNEL *channel);
MEMREGION *visorchannel_get_memregion(VISORCHANNEL *channel);
char *visorchannel_GUID_id(GUID *guid, char *s);
void visorchannel_debug(VISORCHANNEL *channel, int nQueues,
			struct seq_file *seq, U32 off);
void visorchannel_dump_section(VISORCHANNEL *chan, char *s,
			       int off, int len, struct seq_file *seq);
void *visorchannel_get_header(VISORCHANNEL *channel);

#define	VISORCHANNEL_CHANGE_SERVER_STATE(chan, chanId, newstate)	\
	do {								\
		U8 *p = (U8 *)visorchannel_get_header(chan);		\
		if (p) {						\
			ULTRA_CHANNEL_SERVER_TRANSITION(p, chanId, SrvState, \
							newstate, logCtx); \
			visorchannel_write				\
				(chan,					\
				 offsetof(ULTRA_CHANNEL_PROTOCOL, SrvState), \
				 p +					\
				 offsetof(ULTRA_CHANNEL_PROTOCOL, SrvState), \
				 sizeof(U32));				\
		}							\
	} while (0)

#define	VISORCHANNEL_CHANGE_CLIENT_STATE(chan, chanId, newstate)	\
	do {								\
		U8 *p = (U8 *)visorchannel_get_header(chan);		\
		if (p) {						\
			ULTRA_CHANNEL_CLIENT_TRANSITION(p, chanId,	\
							newstate, logCtx); \
			visorchannel_write				\
				(chan,					\
				 offsetof(ULTRA_CHANNEL_PROTOCOL, CliStateOS), \
				 p +					\
				 offsetof(ULTRA_CHANNEL_PROTOCOL, CliStateOS), \
				 sizeof(U32));				\
		}							\
	} while (0)

#endif
