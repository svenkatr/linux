/*
   drbd_actlog.c

   This file is part of DRBD by Philipp Reisner and Lars Ellenberg.

   Copyright (C) 2003-2008, LINBIT Information Technologies GmbH.
   Copyright (C) 2003-2008, Philipp Reisner <philipp.reisner@linbit.com>.
   Copyright (C) 2003-2008, Lars Ellenberg <lars.ellenberg@linbit.com>.

   drbd is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   drbd is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with drbd; see the file COPYING.  If not, write to
   the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.

 */

#include <linux/slab.h>
#include <linux/crc32c.h>
#include <linux/drbd.h>
#include <linux/drbd_limits.h>
#include <linux/dynamic_debug.h>
#include "drbd_int.h"
#include "drbd_wrappers.h"


enum al_transaction_types {
	AL_TR_UPDATE = 0,
	AL_TR_INITIALIZED = 0xffff
};
/* all fields on disc in big endian */
struct __packed al_transaction_on_disk {
	/* don't we all like magic */
	__be32	magic;

	/* to identify the most recent transaction block
	 * in the on disk ring buffer */
	__be32	tr_number;

	/* checksum on the full 4k block, with this field set to 0. */
	__be32	crc32c;

	/* type of transaction, special transaction types like:
	 * purge-all, set-all-idle, set-all-active, ... to-be-defined
	 * see also enum al_transaction_types */
	__be16	transaction_type;

	/* we currently allow only a few thousand extents,
	 * so 16bit will be enough for the slot number. */

	/* how many updates in this transaction */
	__be16	n_updates;

	/* maximum slot number, "al-extents" in drbd.conf speak.
	 * Having this in each transaction should make reconfiguration
	 * of that parameter easier. */
	__be16	context_size;

	/* slot number the context starts with */
	__be16	context_start_slot_nr;

	/* Some reserved bytes.  Expected usage is a 64bit counter of
	 * sectors-written since device creation, and other data generation tag
	 * supporting usage */
	__be32	__reserved[4];

	/* --- 36 byte used --- */

	/* Reserve space for up to AL_UPDATES_PER_TRANSACTION changes
	 * in one transaction, then use the remaining byte in the 4k block for
	 * context information.  "Flexible" number of updates per transaction
	 * does not help, as we have to account for the case when all update
	 * slots are used anyways, so it would only complicate code without
	 * additional benefit.
	 */
	__be16	update_slot_nr[AL_UPDATES_PER_TRANSACTION];

	/* but the extent number is 32bit, which at an extent size of 4 MiB
	 * allows to cover device sizes of up to 2**54 Byte (16 PiB) */
	__be32	update_extent_nr[AL_UPDATES_PER_TRANSACTION];

	/* --- 420 bytes used (36 + 64*6) --- */

	/* 4096 - 420 = 3676 = 919 * 4 */
	__be32	context[AL_CONTEXT_PER_TRANSACTION];
};

struct update_odbm_work {
	struct drbd_work w;
	struct drbd_device *device;
	unsigned int enr;
};

struct update_al_work {
	struct drbd_work w;
	struct drbd_device *device;
	struct completion event;
	int err;
};


void *drbd_md_get_buffer(struct drbd_device *device)
{
	int r;

	wait_event(device->misc_wait,
		   (r = atomic_cmpxchg(&device->md_io_in_use, 0, 1)) == 0 ||
		   device->state.disk <= D_FAILED);

	return r ? NULL : page_address(device->md_io_page);
}

void drbd_md_put_buffer(struct drbd_device *device)
{
	if (atomic_dec_and_test(&device->md_io_in_use))
		wake_up(&device->misc_wait);
}

void wait_until_done_or_force_detached(struct drbd_device *device, struct drbd_backing_dev *bdev,
				     unsigned int *done)
{
	long dt;

	rcu_read_lock();
	dt = rcu_dereference(bdev->disk_conf)->disk_timeout;
	rcu_read_unlock();
	dt = dt * HZ / 10;
	if (dt == 0)
		dt = MAX_SCHEDULE_TIMEOUT;

	dt = wait_event_timeout(device->misc_wait,
			*done || test_bit(FORCE_DETACH, &device->flags), dt);
	if (dt == 0) {
		drbd_err(device, "meta-data IO operation timed out\n");
		drbd_chk_io_error(device, 1, DRBD_FORCE_DETACH);
	}
}

static int _drbd_md_sync_page_io(struct drbd_device *device,
				 struct drbd_backing_dev *bdev,
				 struct page *page, sector_t sector,
				 int rw, int size)
{
	struct bio *bio;
	int err;

	device->md_io.done = 0;
	device->md_io.error = -ENODEV;

	if ((rw & WRITE) && !test_bit(MD_NO_FUA, &device->flags))
		rw |= REQ_FUA | REQ_FLUSH;
	rw |= REQ_SYNC;

	bio = bio_alloc_drbd(GFP_NOIO);
	bio->bi_bdev = bdev->md_bdev;
	bio->bi_iter.bi_sector = sector;
	err = -EIO;
	if (bio_add_page(bio, page, size, 0) != size)
		goto out;
	bio->bi_private = &device->md_io;
	bio->bi_end_io = drbd_md_io_complete;
	bio->bi_rw = rw;

	if (!(rw & WRITE) && device->state.disk == D_DISKLESS && device->ldev == NULL)
		/* special case, drbd_md_read() during drbd_adm_attach(): no get_ldev */
		;
	else if (!get_ldev_if_state(device, D_ATTACHING)) {
		/* Corresponding put_ldev in drbd_md_io_complete() */
		drbd_err(device, "ASSERT FAILED: get_ldev_if_state() == 1 in _drbd_md_sync_page_io()\n");
		err = -ENODEV;
		goto out;
	}

	bio_get(bio); /* one bio_put() is in the completion handler */
	atomic_inc(&device->md_io_in_use); /* drbd_md_put_buffer() is in the completion handler */
	if (drbd_insert_fault(device, (rw & WRITE) ? DRBD_FAULT_MD_WR : DRBD_FAULT_MD_RD))
		bio_endio(bio, -EIO);
	else
		submit_bio(rw, bio);
	wait_until_done_or_force_detached(device, bdev, &device->md_io.done);
	if (bio_flagged(bio, BIO_UPTODATE))
		err = device->md_io.error;

 out:
	bio_put(bio);
	return err;
}

int drbd_md_sync_page_io(struct drbd_device *device, struct drbd_backing_dev *bdev,
			 sector_t sector, int rw)
{
	int err;
	struct page *iop = device->md_io_page;

	D_ASSERT(device, atomic_read(&device->md_io_in_use) == 1);

	BUG_ON(!bdev->md_bdev);

	drbd_dbg(device, "meta_data io: %s [%d]:%s(,%llus,%s) %pS\n",
	     current->comm, current->pid, __func__,
	     (unsigned long long)sector, (rw & WRITE) ? "WRITE" : "READ",
	     (void*)_RET_IP_ );

	if (sector < drbd_md_first_sector(bdev) ||
	    sector + 7 > drbd_md_last_sector(bdev))
		drbd_alert(device, "%s [%d]:%s(,%llus,%s) out of range md access!\n",
		     current->comm, current->pid, __func__,
		     (unsigned long long)sector, (rw & WRITE) ? "WRITE" : "READ");

	/* we do all our meta data IO in aligned 4k blocks. */
	err = _drbd_md_sync_page_io(device, bdev, iop, sector, rw, 4096);
	if (err) {
		drbd_err(device, "drbd_md_sync_page_io(,%llus,%s) failed with error %d\n",
		    (unsigned long long)sector, (rw & WRITE) ? "WRITE" : "READ", err);
	}
	return err;
}

static struct bm_extent *find_active_resync_extent(struct drbd_device *device, unsigned int enr)
{
	struct lc_element *tmp;
	tmp = lc_find(device->resync, enr/AL_EXT_PER_BM_SECT);
	if (unlikely(tmp != NULL)) {
		struct bm_extent  *bm_ext = lc_entry(tmp, struct bm_extent, lce);
		if (test_bit(BME_NO_WRITES, &bm_ext->flags))
			return bm_ext;
	}
	return NULL;
}

static struct lc_element *_al_get(struct drbd_device *device, unsigned int enr, bool nonblock)
{
	struct lc_element *al_ext;
	struct bm_extent *bm_ext;
	int wake;

	spin_lock_irq(&device->al_lock);
	bm_ext = find_active_resync_extent(device, enr);
	if (bm_ext) {
		wake = !test_and_set_bit(BME_PRIORITY, &bm_ext->flags);
		spin_unlock_irq(&device->al_lock);
		if (wake)
			wake_up(&device->al_wait);
		return NULL;
	}
	if (nonblock)
		al_ext = lc_try_get(device->act_log, enr);
	else
		al_ext = lc_get(device->act_log, enr);
	spin_unlock_irq(&device->al_lock);
	return al_ext;
}

bool drbd_al_begin_io_fastpath(struct drbd_device *device, struct drbd_interval *i)
{
	/* for bios crossing activity log extent boundaries,
	 * we may need to activate two extents in one go */
	unsigned first = i->sector >> (AL_EXTENT_SHIFT-9);
	unsigned last = i->size == 0 ? first : (i->sector + (i->size >> 9) - 1) >> (AL_EXTENT_SHIFT-9);

	D_ASSERT(device, (unsigned)(last - first) <= 1);
	D_ASSERT(device, atomic_read(&device->local_cnt) > 0);

	/* FIXME figure out a fast path for bios crossing AL extent boundaries */
	if (first != last)
		return false;

	return _al_get(device, first, true);
}

static
bool drbd_al_begin_io_prepare(struct drbd_device *device, struct drbd_interval *i)
{
	/* for bios crossing activity log extent boundaries,
	 * we may need to activate two extents in one go */
	unsigned first = i->sector >> (AL_EXTENT_SHIFT-9);
	unsigned last = i->size == 0 ? first : (i->sector + (i->size >> 9) - 1) >> (AL_EXTENT_SHIFT-9);
	unsigned enr;
	bool need_transaction = false;

	D_ASSERT(device, first <= last);
	D_ASSERT(device, atomic_read(&device->local_cnt) > 0);

	for (enr = first; enr <= last; enr++) {
		struct lc_element *al_ext;
		wait_event(device->al_wait,
				(al_ext = _al_get(device, enr, false)) != NULL);
		if (al_ext->lc_number != enr)
			need_transaction = true;
	}
	return need_transaction;
}

static int al_write_transaction(struct drbd_device *device, bool delegate);

/* When called through generic_make_request(), we must delegate
 * activity log I/O to the worker thread: a further request
 * submitted via generic_make_request() within the same task
 * would be queued on current->bio_list, and would only start
 * after this function returns (see generic_make_request()).
 *
 * However, if we *are* the worker, we must not delegate to ourselves.
 */

/*
 * @delegate:   delegate activity log I/O to the worker thread
 */
void drbd_al_begin_io_commit(struct drbd_device *device, bool delegate)
{
	bool locked = false;

	BUG_ON(delegate && current == first_peer_device(device)->connection->worker.task);

	/* Serialize multiple transactions.
	 * This uses test_and_set_bit, memory barrier is implicit.
	 */
	wait_event(device->al_wait,
			device->act_log->pending_changes == 0 ||
			(locked = lc_try_lock_for_transaction(device->act_log)));

	if (locked) {
		/* Double check: it may have been committed by someone else,
		 * while we have been waiting for the lock. */
		if (device->act_log->pending_changes) {
			bool write_al_updates;

			rcu_read_lock();
			write_al_updates = rcu_dereference(device->ldev->disk_conf)->al_updates;
			rcu_read_unlock();

			if (write_al_updates)
				al_write_transaction(device, delegate);
			spin_lock_irq(&device->al_lock);
			/* FIXME
			if (err)
				we need an "lc_cancel" here;
			*/
			lc_committed(device->act_log);
			spin_unlock_irq(&device->al_lock);
		}
		lc_unlock(device->act_log);
		wake_up(&device->al_wait);
	}
}

/*
 * @delegate:   delegate activity log I/O to the worker thread
 */
void drbd_al_begin_io(struct drbd_device *device, struct drbd_interval *i, bool delegate)
{
	BUG_ON(delegate && current == first_peer_device(device)->connection->worker.task);

	if (drbd_al_begin_io_prepare(device, i))
		drbd_al_begin_io_commit(device, delegate);
}

int drbd_al_begin_io_nonblock(struct drbd_device *device, struct drbd_interval *i)
{
	struct lru_cache *al = device->act_log;
	/* for bios crossing activity log extent boundaries,
	 * we may need to activate two extents in one go */
	unsigned first = i->sector >> (AL_EXTENT_SHIFT-9);
	unsigned last = i->size == 0 ? first : (i->sector + (i->size >> 9) - 1) >> (AL_EXTENT_SHIFT-9);
	unsigned nr_al_extents;
	unsigned available_update_slots;
	unsigned enr;

	D_ASSERT(device, first <= last);

	nr_al_extents = 1 + last - first; /* worst case: all touched extends are cold. */
	available_update_slots = min(al->nr_elements - al->used,
				al->max_pending_changes - al->pending_changes);

	/* We want all necessary updates for a given request within the same transaction
	 * We could first check how many updates are *actually* needed,
	 * and use that instead of the worst-case nr_al_extents */
	if (available_update_slots < nr_al_extents)
		return -EWOULDBLOCK;

	/* Is resync active in this area? */
	for (enr = first; enr <= last; enr++) {
		struct lc_element *tmp;
		tmp = lc_find(device->resync, enr/AL_EXT_PER_BM_SECT);
		if (unlikely(tmp != NULL)) {
			struct bm_extent  *bm_ext = lc_entry(tmp, struct bm_extent, lce);
			if (test_bit(BME_NO_WRITES, &bm_ext->flags)) {
				if (!test_and_set_bit(BME_PRIORITY, &bm_ext->flags))
					return -EBUSY;
				return -EWOULDBLOCK;
			}
		}
	}

	/* Checkout the refcounts.
	 * Given that we checked for available elements and update slots above,
	 * this has to be successful. */
	for (enr = first; enr <= last; enr++) {
		struct lc_element *al_ext;
		al_ext = lc_get_cumulative(device->act_log, enr);
		if (!al_ext)
			drbd_info(device, "LOGIC BUG for enr=%u\n", enr);
	}
	return 0;
}

void drbd_al_complete_io(struct drbd_device *device, struct drbd_interval *i)
{
	/* for bios crossing activity log extent boundaries,
	 * we may need to activate two extents in one go */
	unsigned first = i->sector >> (AL_EXTENT_SHIFT-9);
	unsigned last = i->size == 0 ? first : (i->sector + (i->size >> 9) - 1) >> (AL_EXTENT_SHIFT-9);
	unsigned enr;
	struct lc_element *extent;
	unsigned long flags;

	D_ASSERT(device, first <= last);
	spin_lock_irqsave(&device->al_lock, flags);

	for (enr = first; enr <= last; enr++) {
		extent = lc_find(device->act_log, enr);
		if (!extent) {
			drbd_err(device, "al_complete_io() called on inactive extent %u\n", enr);
			continue;
		}
		lc_put(device->act_log, extent);
	}
	spin_unlock_irqrestore(&device->al_lock, flags);
	wake_up(&device->al_wait);
}

#if (PAGE_SHIFT + 3) < (AL_EXTENT_SHIFT - BM_BLOCK_SHIFT)
/* Currently BM_BLOCK_SHIFT, BM_EXT_SHIFT and AL_EXTENT_SHIFT
 * are still coupled, or assume too much about their relation.
 * Code below will not work if this is violated.
 * Will be cleaned up with some followup patch.
 */
# error FIXME
#endif

static unsigned int al_extent_to_bm_page(unsigned int al_enr)
{
	return al_enr >>
		/* bit to page */
		((PAGE_SHIFT + 3) -
		/* al extent number to bit */
		 (AL_EXTENT_SHIFT - BM_BLOCK_SHIFT));
}

static unsigned int rs_extent_to_bm_page(unsigned int rs_enr)
{
	return rs_enr >>
		/* bit to page */
		((PAGE_SHIFT + 3) -
		/* resync extent number to bit */
		 (BM_EXT_SHIFT - BM_BLOCK_SHIFT));
}

static sector_t al_tr_number_to_on_disk_sector(struct drbd_device *device)
{
	const unsigned int stripes = device->ldev->md.al_stripes;
	const unsigned int stripe_size_4kB = device->ldev->md.al_stripe_size_4k;

	/* transaction number, modulo on-disk ring buffer wrap around */
	unsigned int t = device->al_tr_number % (device->ldev->md.al_size_4k);

	/* ... to aligned 4k on disk block */
	t = ((t % stripes) * stripe_size_4kB) + t/stripes;

	/* ... to 512 byte sector in activity log */
	t *= 8;

	/* ... plus offset to the on disk position */
	return device->ldev->md.md_offset + device->ldev->md.al_offset + t;
}

static int
_al_write_transaction(struct drbd_device *device)
{
	struct al_transaction_on_disk *buffer;
	struct lc_element *e;
	sector_t sector;
	int i, mx;
	unsigned extent_nr;
	unsigned crc = 0;
	int err = 0;

	if (!get_ldev(device)) {
		drbd_err(device, "disk is %s, cannot start al transaction\n",
			drbd_disk_str(device->state.disk));
		return -EIO;
	}

	/* The bitmap write may have failed, causing a state change. */
	if (device->state.disk < D_INCONSISTENT) {
		drbd_err(device,
			"disk is %s, cannot write al transaction\n",
			drbd_disk_str(device->state.disk));
		put_ldev(device);
		return -EIO;
	}

	buffer = drbd_md_get_buffer(device); /* protects md_io_buffer, al_tr_cycle, ... */
	if (!buffer) {
		drbd_err(device, "disk failed while waiting for md_io buffer\n");
		put_ldev(device);
		return -ENODEV;
	}

	memset(buffer, 0, sizeof(*buffer));
	buffer->magic = cpu_to_be32(DRBD_AL_MAGIC);
	buffer->tr_number = cpu_to_be32(device->al_tr_number);

	i = 0;

	/* Even though no one can start to change this list
	 * once we set the LC_LOCKED -- from drbd_al_begin_io(),
	 * lc_try_lock_for_transaction() --, someone may still
	 * be in the process of changing it. */
	spin_lock_irq(&device->al_lock);
	list_for_each_entry(e, &device->act_log->to_be_changed, list) {
		if (i == AL_UPDATES_PER_TRANSACTION) {
			i++;
			break;
		}
		buffer->update_slot_nr[i] = cpu_to_be16(e->lc_index);
		buffer->update_extent_nr[i] = cpu_to_be32(e->lc_new_number);
		if (e->lc_number != LC_FREE)
			drbd_bm_mark_for_writeout(device,
					al_extent_to_bm_page(e->lc_number));
		i++;
	}
	spin_unlock_irq(&device->al_lock);
	BUG_ON(i > AL_UPDATES_PER_TRANSACTION);

	buffer->n_updates = cpu_to_be16(i);
	for ( ; i < AL_UPDATES_PER_TRANSACTION; i++) {
		buffer->update_slot_nr[i] = cpu_to_be16(-1);
		buffer->update_extent_nr[i] = cpu_to_be32(LC_FREE);
	}

	buffer->context_size = cpu_to_be16(device->act_log->nr_elements);
	buffer->context_start_slot_nr = cpu_to_be16(device->al_tr_cycle);

	mx = min_t(int, AL_CONTEXT_PER_TRANSACTION,
		   device->act_log->nr_elements - device->al_tr_cycle);
	for (i = 0; i < mx; i++) {
		unsigned idx = device->al_tr_cycle + i;
		extent_nr = lc_element_by_index(device->act_log, idx)->lc_number;
		buffer->context[i] = cpu_to_be32(extent_nr);
	}
	for (; i < AL_CONTEXT_PER_TRANSACTION; i++)
		buffer->context[i] = cpu_to_be32(LC_FREE);

	device->al_tr_cycle += AL_CONTEXT_PER_TRANSACTION;
	if (device->al_tr_cycle >= device->act_log->nr_elements)
		device->al_tr_cycle = 0;

	sector = al_tr_number_to_on_disk_sector(device);

	crc = crc32c(0, buffer, 4096);
	buffer->crc32c = cpu_to_be32(crc);

	if (drbd_bm_write_hinted(device))
		err = -EIO;
	else {
		bool write_al_updates;
		rcu_read_lock();
		write_al_updates = rcu_dereference(device->ldev->disk_conf)->al_updates;
		rcu_read_unlock();
		if (write_al_updates) {
			if (drbd_md_sync_page_io(device, device->ldev, sector, WRITE)) {
				err = -EIO;
				drbd_chk_io_error(device, 1, DRBD_META_IO_ERROR);
			} else {
				device->al_tr_number++;
				device->al_writ_cnt++;
			}
		}
	}

	drbd_md_put_buffer(device);
	put_ldev(device);

	return err;
}


static int w_al_write_transaction(struct drbd_work *w, int unused)
{
	struct update_al_work *aw = container_of(w, struct update_al_work, w);
	struct drbd_device *device = aw->device;
	int err;

	err = _al_write_transaction(device);
	aw->err = err;
	complete(&aw->event);

	return err != -EIO ? err : 0;
}

/* Calls from worker context (see w_restart_disk_io()) need to write the
   transaction directly. Others came through generic_make_request(),
   those need to delegate it to the worker. */
static int al_write_transaction(struct drbd_device *device, bool delegate)
{
	if (delegate) {
		struct update_al_work al_work;
		init_completion(&al_work.event);
		al_work.w.cb = w_al_write_transaction;
		al_work.device = device;
		drbd_queue_work_front(&first_peer_device(device)->connection->sender_work,
				      &al_work.w);
		wait_for_completion(&al_work.event);
		return al_work.err;
	} else
		return _al_write_transaction(device);
}

static int _try_lc_del(struct drbd_device *device, struct lc_element *al_ext)
{
	int rv;

	spin_lock_irq(&device->al_lock);
	rv = (al_ext->refcnt == 0);
	if (likely(rv))
		lc_del(device->act_log, al_ext);
	spin_unlock_irq(&device->al_lock);

	return rv;
}

/**
 * drbd_al_shrink() - Removes all active extents form the activity log
 * @device:	DRBD device.
 *
 * Removes all active extents form the activity log, waiting until
 * the reference count of each entry dropped to 0 first, of course.
 *
 * You need to lock device->act_log with lc_try_lock() / lc_unlock()
 */
void drbd_al_shrink(struct drbd_device *device)
{
	struct lc_element *al_ext;
	int i;

	D_ASSERT(device, test_bit(__LC_LOCKED, &device->act_log->flags));

	for (i = 0; i < device->act_log->nr_elements; i++) {
		al_ext = lc_element_by_index(device->act_log, i);
		if (al_ext->lc_number == LC_FREE)
			continue;
		wait_event(device->al_wait, _try_lc_del(device, al_ext));
	}

	wake_up(&device->al_wait);
}

int drbd_initialize_al(struct drbd_device *device, void *buffer)
{
	struct al_transaction_on_disk *al = buffer;
	struct drbd_md *md = &device->ldev->md;
	sector_t al_base = md->md_offset + md->al_offset;
	int al_size_4k = md->al_stripes * md->al_stripe_size_4k;
	int i;

	memset(al, 0, 4096);
	al->magic = cpu_to_be32(DRBD_AL_MAGIC);
	al->transaction_type = cpu_to_be16(AL_TR_INITIALIZED);
	al->crc32c = cpu_to_be32(crc32c(0, al, 4096));

	for (i = 0; i < al_size_4k; i++) {
		int err = drbd_md_sync_page_io(device, device->ldev, al_base + i * 8, WRITE);
		if (err)
			return err;
	}
	return 0;
}

static int w_update_odbm(struct drbd_work *w, int unused)
{
	struct update_odbm_work *udw = container_of(w, struct update_odbm_work, w);
	struct drbd_device *device = udw->device;
	struct sib_info sib = { .sib_reason = SIB_SYNC_PROGRESS, };

	if (!get_ldev(device)) {
		if (__ratelimit(&drbd_ratelimit_state))
			drbd_warn(device, "Can not update on disk bitmap, local IO disabled.\n");
		kfree(udw);
		return 0;
	}

	drbd_bm_write_page(device, rs_extent_to_bm_page(udw->enr));
	put_ldev(device);

	kfree(udw);

	if (drbd_bm_total_weight(device) <= device->rs_failed) {
		switch (device->state.conn) {
		case C_SYNC_SOURCE:  case C_SYNC_TARGET:
		case C_PAUSED_SYNC_S: case C_PAUSED_SYNC_T:
			drbd_resync_finished(device);
		default:
			/* nothing to do */
			break;
		}
	}
	drbd_bcast_event(device, &sib);

	return 0;
}


/* ATTENTION. The AL's extents are 4MB each, while the extents in the
 * resync LRU-cache are 16MB each.
 * The caller of this function has to hold an get_ldev() reference.
 *
 * TODO will be obsoleted once we have a caching lru of the on disk bitmap
 */
static void drbd_try_clear_on_disk_bm(struct drbd_device *device, sector_t sector,
				      int count, int success)
{
	struct lc_element *e;
	struct update_odbm_work *udw;

	unsigned int enr;

	D_ASSERT(device, atomic_read(&device->local_cnt));

	/* I simply assume that a sector/size pair never crosses
	 * a 16 MB extent border. (Currently this is true...) */
	enr = BM_SECT_TO_EXT(sector);

	e = lc_get(device->resync, enr);
	if (e) {
		struct bm_extent *ext = lc_entry(e, struct bm_extent, lce);
		if (ext->lce.lc_number == enr) {
			if (success)
				ext->rs_left -= count;
			else
				ext->rs_failed += count;
			if (ext->rs_left < ext->rs_failed) {
				drbd_warn(device, "BAD! sector=%llus enr=%u rs_left=%d "
				    "rs_failed=%d count=%d cstate=%s\n",
				     (unsigned long long)sector,
				     ext->lce.lc_number, ext->rs_left,
				     ext->rs_failed, count,
				     drbd_conn_str(device->state.conn));

				/* We don't expect to be able to clear more bits
				 * than have been set when we originally counted
				 * the set bits to cache that value in ext->rs_left.
				 * Whatever the reason (disconnect during resync,
				 * delayed local completion of an application write),
				 * try to fix it up by recounting here. */
				ext->rs_left = drbd_bm_e_weight(device, enr);
			}
		} else {
			/* Normally this element should be in the cache,
			 * since drbd_rs_begin_io() pulled it already in.
			 *
			 * But maybe an application write finished, and we set
			 * something outside the resync lru_cache in sync.
			 */
			int rs_left = drbd_bm_e_weight(device, enr);
			if (ext->flags != 0) {
				drbd_warn(device, "changing resync lce: %d[%u;%02lx]"
				     " -> %d[%u;00]\n",
				     ext->lce.lc_number, ext->rs_left,
				     ext->flags, enr, rs_left);
				ext->flags = 0;
			}
			if (ext->rs_failed) {
				drbd_warn(device, "Kicking resync_lru element enr=%u "
				     "out with rs_failed=%d\n",
				     ext->lce.lc_number, ext->rs_failed);
			}
			ext->rs_left = rs_left;
			ext->rs_failed = success ? 0 : count;
			/* we don't keep a persistent log of the resync lru,
			 * we can commit any change right away. */
			lc_committed(device->resync);
		}
		lc_put(device->resync, &ext->lce);
		/* no race, we are within the al_lock! */

		if (ext->rs_left == ext->rs_failed) {
			ext->rs_failed = 0;

			udw = kmalloc(sizeof(*udw), GFP_ATOMIC);
			if (udw) {
				udw->enr = ext->lce.lc_number;
				udw->w.cb = w_update_odbm;
				udw->device = device;
				drbd_queue_work_front(&first_peer_device(device)->connection->sender_work,
						      &udw->w);
			} else {
				drbd_warn(device, "Could not kmalloc an udw\n");
			}
		}
	} else {
		drbd_err(device, "lc_get() failed! locked=%d/%d flags=%lu\n",
		    device->resync_locked,
		    device->resync->nr_elements,
		    device->resync->flags);
	}
}

void drbd_advance_rs_marks(struct drbd_device *device, unsigned long still_to_go)
{
	unsigned long now = jiffies;
	unsigned long last = device->rs_mark_time[device->rs_last_mark];
	int next = (device->rs_last_mark + 1) % DRBD_SYNC_MARKS;
	if (time_after_eq(now, last + DRBD_SYNC_MARK_STEP)) {
		if (device->rs_mark_left[device->rs_last_mark] != still_to_go &&
		    device->state.conn != C_PAUSED_SYNC_T &&
		    device->state.conn != C_PAUSED_SYNC_S) {
			device->rs_mark_time[next] = now;
			device->rs_mark_left[next] = still_to_go;
			device->rs_last_mark = next;
		}
	}
}

/* clear the bit corresponding to the piece of storage in question:
 * size byte of data starting from sector.  Only clear a bits of the affected
 * one ore more _aligned_ BM_BLOCK_SIZE blocks.
 *
 * called by worker on C_SYNC_TARGET and receiver on SyncSource.
 *
 */
void __drbd_set_in_sync(struct drbd_device *device, sector_t sector, int size,
		       const char *file, const unsigned int line)
{
	/* Is called from worker and receiver context _only_ */
	unsigned long sbnr, ebnr, lbnr;
	unsigned long count = 0;
	sector_t esector, nr_sectors;
	int wake_up = 0;
	unsigned long flags;

	if (size <= 0 || !IS_ALIGNED(size, 512) || size > DRBD_MAX_BIO_SIZE) {
		drbd_err(device, "drbd_set_in_sync: sector=%llus size=%d nonsense!\n",
				(unsigned long long)sector, size);
		return;
	}

	if (!get_ldev(device))
		return; /* no disk, no metadata, no bitmap to clear bits in */

	nr_sectors = drbd_get_capacity(device->this_bdev);
	esector = sector + (size >> 9) - 1;

	if (!expect(sector < nr_sectors))
		goto out;
	if (!expect(esector < nr_sectors))
		esector = nr_sectors - 1;

	lbnr = BM_SECT_TO_BIT(nr_sectors-1);

	/* we clear it (in sync).
	 * round up start sector, round down end sector.  we make sure we only
	 * clear full, aligned, BM_BLOCK_SIZE (4K) blocks */
	if (unlikely(esector < BM_SECT_PER_BIT-1))
		goto out;
	if (unlikely(esector == (nr_sectors-1)))
		ebnr = lbnr;
	else
		ebnr = BM_SECT_TO_BIT(esector - (BM_SECT_PER_BIT-1));
	sbnr = BM_SECT_TO_BIT(sector + BM_SECT_PER_BIT-1);

	if (sbnr > ebnr)
		goto out;

	/*
	 * ok, (capacity & 7) != 0 sometimes, but who cares...
	 * we count rs_{total,left} in bits, not sectors.
	 */
	count = drbd_bm_clear_bits(device, sbnr, ebnr);
	if (count) {
		drbd_advance_rs_marks(device, drbd_bm_total_weight(device));
		spin_lock_irqsave(&device->al_lock, flags);
		drbd_try_clear_on_disk_bm(device, sector, count, true);
		spin_unlock_irqrestore(&device->al_lock, flags);

		/* just wake_up unconditional now, various lc_chaged(),
		 * lc_put() in drbd_try_clear_on_disk_bm(). */
		wake_up = 1;
	}
out:
	put_ldev(device);
	if (wake_up)
		wake_up(&device->al_wait);
}

/*
 * this is intended to set one request worth of data out of sync.
 * affects at least 1 bit,
 * and at most 1+DRBD_MAX_BIO_SIZE/BM_BLOCK_SIZE bits.
 *
 * called by tl_clear and drbd_send_dblock (==drbd_make_request).
 * so this can be _any_ process.
 */
int __drbd_set_out_of_sync(struct drbd_device *device, sector_t sector, int size,
			    const char *file, const unsigned int line)
{
	unsigned long sbnr, ebnr, flags;
	sector_t esector, nr_sectors;
	unsigned int enr, count = 0;
	struct lc_element *e;

	/* this should be an empty REQ_FLUSH */
	if (size == 0)
		return 0;

	if (size < 0 || !IS_ALIGNED(size, 512) || size > DRBD_MAX_BIO_SIZE) {
		drbd_err(device, "sector: %llus, size: %d\n",
			(unsigned long long)sector, size);
		return 0;
	}

	if (!get_ldev(device))
		return 0; /* no disk, no metadata, no bitmap to set bits in */

	nr_sectors = drbd_get_capacity(device->this_bdev);
	esector = sector + (size >> 9) - 1;

	if (!expect(sector < nr_sectors))
		goto out;
	if (!expect(esector < nr_sectors))
		esector = nr_sectors - 1;

	/* we set it out of sync,
	 * we do not need to round anything here */
	sbnr = BM_SECT_TO_BIT(sector);
	ebnr = BM_SECT_TO_BIT(esector);

	/* ok, (capacity & 7) != 0 sometimes, but who cares...
	 * we count rs_{total,left} in bits, not sectors.  */
	spin_lock_irqsave(&device->al_lock, flags);
	count = drbd_bm_set_bits(device, sbnr, ebnr);

	enr = BM_SECT_TO_EXT(sector);
	e = lc_find(device->resync, enr);
	if (e)
		lc_entry(e, struct bm_extent, lce)->rs_left += count;
	spin_unlock_irqrestore(&device->al_lock, flags);

out:
	put_ldev(device);

	return count;
}

static
struct bm_extent *_bme_get(struct drbd_device *device, unsigned int enr)
{
	struct lc_element *e;
	struct bm_extent *bm_ext;
	int wakeup = 0;
	unsigned long rs_flags;

	spin_lock_irq(&device->al_lock);
	if (device->resync_locked > device->resync->nr_elements/2) {
		spin_unlock_irq(&device->al_lock);
		return NULL;
	}
	e = lc_get(device->resync, enr);
	bm_ext = e ? lc_entry(e, struct bm_extent, lce) : NULL;
	if (bm_ext) {
		if (bm_ext->lce.lc_number != enr) {
			bm_ext->rs_left = drbd_bm_e_weight(device, enr);
			bm_ext->rs_failed = 0;
			lc_committed(device->resync);
			wakeup = 1;
		}
		if (bm_ext->lce.refcnt == 1)
			device->resync_locked++;
		set_bit(BME_NO_WRITES, &bm_ext->flags);
	}
	rs_flags = device->resync->flags;
	spin_unlock_irq(&device->al_lock);
	if (wakeup)
		wake_up(&device->al_wait);

	if (!bm_ext) {
		if (rs_flags & LC_STARVING)
			drbd_warn(device, "Have to wait for element"
			     " (resync LRU too small?)\n");
		BUG_ON(rs_flags & LC_LOCKED);
	}

	return bm_ext;
}

static int _is_in_al(struct drbd_device *device, unsigned int enr)
{
	int rv;

	spin_lock_irq(&device->al_lock);
	rv = lc_is_used(device->act_log, enr);
	spin_unlock_irq(&device->al_lock);

	return rv;
}

/**
 * drbd_rs_begin_io() - Gets an extent in the resync LRU cache and sets it to BME_LOCKED
 * @device:	DRBD device.
 * @sector:	The sector number.
 *
 * This functions sleeps on al_wait. Returns 0 on success, -EINTR if interrupted.
 */
int drbd_rs_begin_io(struct drbd_device *device, sector_t sector)
{
	unsigned int enr = BM_SECT_TO_EXT(sector);
	struct bm_extent *bm_ext;
	int i, sig;
	int sa = 200; /* Step aside 200 times, then grab the extent and let app-IO wait.
			 200 times -> 20 seconds. */

retry:
	sig = wait_event_interruptible(device->al_wait,
			(bm_ext = _bme_get(device, enr)));
	if (sig)
		return -EINTR;

	if (test_bit(BME_LOCKED, &bm_ext->flags))
		return 0;

	for (i = 0; i < AL_EXT_PER_BM_SECT; i++) {
		sig = wait_event_interruptible(device->al_wait,
					       !_is_in_al(device, enr * AL_EXT_PER_BM_SECT + i) ||
					       test_bit(BME_PRIORITY, &bm_ext->flags));

		if (sig || (test_bit(BME_PRIORITY, &bm_ext->flags) && sa)) {
			spin_lock_irq(&device->al_lock);
			if (lc_put(device->resync, &bm_ext->lce) == 0) {
				bm_ext->flags = 0; /* clears BME_NO_WRITES and eventually BME_PRIORITY */
				device->resync_locked--;
				wake_up(&device->al_wait);
			}
			spin_unlock_irq(&device->al_lock);
			if (sig)
				return -EINTR;
			if (schedule_timeout_interruptible(HZ/10))
				return -EINTR;
			if (sa && --sa == 0)
				drbd_warn(device, "drbd_rs_begin_io() stepped aside for 20sec."
					 "Resync stalled?\n");
			goto retry;
		}
	}
	set_bit(BME_LOCKED, &bm_ext->flags);
	return 0;
}

/**
 * drbd_try_rs_begin_io() - Gets an extent in the resync LRU cache, does not sleep
 * @device:	DRBD device.
 * @sector:	The sector number.
 *
 * Gets an extent in the resync LRU cache, sets it to BME_NO_WRITES, then
 * tries to set it to BME_LOCKED. Returns 0 upon success, and -EAGAIN
 * if there is still application IO going on in this area.
 */
int drbd_try_rs_begin_io(struct drbd_device *device, sector_t sector)
{
	unsigned int enr = BM_SECT_TO_EXT(sector);
	const unsigned int al_enr = enr*AL_EXT_PER_BM_SECT;
	struct lc_element *e;
	struct bm_extent *bm_ext;
	int i;

	spin_lock_irq(&device->al_lock);
	if (device->resync_wenr != LC_FREE && device->resync_wenr != enr) {
		/* in case you have very heavy scattered io, it may
		 * stall the syncer undefined if we give up the ref count
		 * when we try again and requeue.
		 *
		 * if we don't give up the refcount, but the next time
		 * we are scheduled this extent has been "synced" by new
		 * application writes, we'd miss the lc_put on the
		 * extent we keep the refcount on.
		 * so we remembered which extent we had to try again, and
		 * if the next requested one is something else, we do
		 * the lc_put here...
		 * we also have to wake_up
		 */
		e = lc_find(device->resync, device->resync_wenr);
		bm_ext = e ? lc_entry(e, struct bm_extent, lce) : NULL;
		if (bm_ext) {
			D_ASSERT(device, !test_bit(BME_LOCKED, &bm_ext->flags));
			D_ASSERT(device, test_bit(BME_NO_WRITES, &bm_ext->flags));
			clear_bit(BME_NO_WRITES, &bm_ext->flags);
			device->resync_wenr = LC_FREE;
			if (lc_put(device->resync, &bm_ext->lce) == 0)
				device->resync_locked--;
			wake_up(&device->al_wait);
		} else {
			drbd_alert(device, "LOGIC BUG\n");
		}
	}
	/* TRY. */
	e = lc_try_get(device->resync, enr);
	bm_ext = e ? lc_entry(e, struct bm_extent, lce) : NULL;
	if (bm_ext) {
		if (test_bit(BME_LOCKED, &bm_ext->flags))
			goto proceed;
		if (!test_and_set_bit(BME_NO_WRITES, &bm_ext->flags)) {
			device->resync_locked++;
		} else {
			/* we did set the BME_NO_WRITES,
			 * but then could not set BME_LOCKED,
			 * so we tried again.
			 * drop the extra reference. */
			bm_ext->lce.refcnt--;
			D_ASSERT(device, bm_ext->lce.refcnt > 0);
		}
		goto check_al;
	} else {
		/* do we rather want to try later? */
		if (device->resync_locked > device->resync->nr_elements-3)
			goto try_again;
		/* Do or do not. There is no try. -- Yoda */
		e = lc_get(device->resync, enr);
		bm_ext = e ? lc_entry(e, struct bm_extent, lce) : NULL;
		if (!bm_ext) {
			const unsigned long rs_flags = device->resync->flags;
			if (rs_flags & LC_STARVING)
				drbd_warn(device, "Have to wait for element"
				     " (resync LRU too small?)\n");
			BUG_ON(rs_flags & LC_LOCKED);
			goto try_again;
		}
		if (bm_ext->lce.lc_number != enr) {
			bm_ext->rs_left = drbd_bm_e_weight(device, enr);
			bm_ext->rs_failed = 0;
			lc_committed(device->resync);
			wake_up(&device->al_wait);
			D_ASSERT(device, test_bit(BME_LOCKED, &bm_ext->flags) == 0);
		}
		set_bit(BME_NO_WRITES, &bm_ext->flags);
		D_ASSERT(device, bm_ext->lce.refcnt == 1);
		device->resync_locked++;
		goto check_al;
	}
check_al:
	for (i = 0; i < AL_EXT_PER_BM_SECT; i++) {
		if (lc_is_used(device->act_log, al_enr+i))
			goto try_again;
	}
	set_bit(BME_LOCKED, &bm_ext->flags);
proceed:
	device->resync_wenr = LC_FREE;
	spin_unlock_irq(&device->al_lock);
	return 0;

try_again:
	if (bm_ext)
		device->resync_wenr = enr;
	spin_unlock_irq(&device->al_lock);
	return -EAGAIN;
}

void drbd_rs_complete_io(struct drbd_device *device, sector_t sector)
{
	unsigned int enr = BM_SECT_TO_EXT(sector);
	struct lc_element *e;
	struct bm_extent *bm_ext;
	unsigned long flags;

	spin_lock_irqsave(&device->al_lock, flags);
	e = lc_find(device->resync, enr);
	bm_ext = e ? lc_entry(e, struct bm_extent, lce) : NULL;
	if (!bm_ext) {
		spin_unlock_irqrestore(&device->al_lock, flags);
		if (__ratelimit(&drbd_ratelimit_state))
			drbd_err(device, "drbd_rs_complete_io() called, but extent not found\n");
		return;
	}

	if (bm_ext->lce.refcnt == 0) {
		spin_unlock_irqrestore(&device->al_lock, flags);
		drbd_err(device, "drbd_rs_complete_io(,%llu [=%u]) called, "
		    "but refcnt is 0!?\n",
		    (unsigned long long)sector, enr);
		return;
	}

	if (lc_put(device->resync, &bm_ext->lce) == 0) {
		bm_ext->flags = 0; /* clear BME_LOCKED, BME_NO_WRITES and BME_PRIORITY */
		device->resync_locked--;
		wake_up(&device->al_wait);
	}

	spin_unlock_irqrestore(&device->al_lock, flags);
}

/**
 * drbd_rs_cancel_all() - Removes all extents from the resync LRU (even BME_LOCKED)
 * @device:	DRBD device.
 */
void drbd_rs_cancel_all(struct drbd_device *device)
{
	spin_lock_irq(&device->al_lock);

	if (get_ldev_if_state(device, D_FAILED)) { /* Makes sure ->resync is there. */
		lc_reset(device->resync);
		put_ldev(device);
	}
	device->resync_locked = 0;
	device->resync_wenr = LC_FREE;
	spin_unlock_irq(&device->al_lock);
	wake_up(&device->al_wait);
}

/**
 * drbd_rs_del_all() - Gracefully remove all extents from the resync LRU
 * @device:	DRBD device.
 *
 * Returns 0 upon success, -EAGAIN if at least one reference count was
 * not zero.
 */
int drbd_rs_del_all(struct drbd_device *device)
{
	struct lc_element *e;
	struct bm_extent *bm_ext;
	int i;

	spin_lock_irq(&device->al_lock);

	if (get_ldev_if_state(device, D_FAILED)) {
		/* ok, ->resync is there. */
		for (i = 0; i < device->resync->nr_elements; i++) {
			e = lc_element_by_index(device->resync, i);
			bm_ext = lc_entry(e, struct bm_extent, lce);
			if (bm_ext->lce.lc_number == LC_FREE)
				continue;
			if (bm_ext->lce.lc_number == device->resync_wenr) {
				drbd_info(device, "dropping %u in drbd_rs_del_all, apparently"
				     " got 'synced' by application io\n",
				     device->resync_wenr);
				D_ASSERT(device, !test_bit(BME_LOCKED, &bm_ext->flags));
				D_ASSERT(device, test_bit(BME_NO_WRITES, &bm_ext->flags));
				clear_bit(BME_NO_WRITES, &bm_ext->flags);
				device->resync_wenr = LC_FREE;
				lc_put(device->resync, &bm_ext->lce);
			}
			if (bm_ext->lce.refcnt != 0) {
				drbd_info(device, "Retrying drbd_rs_del_all() later. "
				     "refcnt=%d\n", bm_ext->lce.refcnt);
				put_ldev(device);
				spin_unlock_irq(&device->al_lock);
				return -EAGAIN;
			}
			D_ASSERT(device, !test_bit(BME_LOCKED, &bm_ext->flags));
			D_ASSERT(device, !test_bit(BME_NO_WRITES, &bm_ext->flags));
			lc_del(device->resync, &bm_ext->lce);
		}
		D_ASSERT(device, device->resync->used == 0);
		put_ldev(device);
	}
	spin_unlock_irq(&device->al_lock);
	wake_up(&device->al_wait);

	return 0;
}

/**
 * drbd_rs_failed_io() - Record information on a failure to resync the specified blocks
 * @device:	DRBD device.
 * @sector:	The sector number.
 * @size:	Size of failed IO operation, in byte.
 */
void drbd_rs_failed_io(struct drbd_device *device, sector_t sector, int size)
{
	/* Is called from worker and receiver context _only_ */
	unsigned long sbnr, ebnr, lbnr;
	unsigned long count;
	sector_t esector, nr_sectors;
	int wake_up = 0;

	if (size <= 0 || !IS_ALIGNED(size, 512) || size > DRBD_MAX_BIO_SIZE) {
		drbd_err(device, "drbd_rs_failed_io: sector=%llus size=%d nonsense!\n",
				(unsigned long long)sector, size);
		return;
	}
	nr_sectors = drbd_get_capacity(device->this_bdev);
	esector = sector + (size >> 9) - 1;

	if (!expect(sector < nr_sectors))
		return;
	if (!expect(esector < nr_sectors))
		esector = nr_sectors - 1;

	lbnr = BM_SECT_TO_BIT(nr_sectors-1);

	/*
	 * round up start sector, round down end sector.  we make sure we only
	 * handle full, aligned, BM_BLOCK_SIZE (4K) blocks */
	if (unlikely(esector < BM_SECT_PER_BIT-1))
		return;
	if (unlikely(esector == (nr_sectors-1)))
		ebnr = lbnr;
	else
		ebnr = BM_SECT_TO_BIT(esector - (BM_SECT_PER_BIT-1));
	sbnr = BM_SECT_TO_BIT(sector + BM_SECT_PER_BIT-1);

	if (sbnr > ebnr)
		return;

	/*
	 * ok, (capacity & 7) != 0 sometimes, but who cares...
	 * we count rs_{total,left} in bits, not sectors.
	 */
	spin_lock_irq(&device->al_lock);
	count = drbd_bm_count_bits(device, sbnr, ebnr);
	if (count) {
		device->rs_failed += count;

		if (get_ldev(device)) {
			drbd_try_clear_on_disk_bm(device, sector, count, false);
			put_ldev(device);
		}

		/* just wake_up unconditional now, various lc_chaged(),
		 * lc_put() in drbd_try_clear_on_disk_bm(). */
		wake_up = 1;
	}
	spin_unlock_irq(&device->al_lock);
	if (wake_up)
		wake_up(&device->al_wait);
}
