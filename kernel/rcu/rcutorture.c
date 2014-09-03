/*
 * Read-Copy Update module-based torture test facility
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 * Copyright (C) IBM Corporation, 2005, 2006
 *
 * Authors: Paul E. McKenney <paulmck@us.ibm.com>
 *	  Josh Triplett <josh@freedesktop.org>
 *
 * See also:  Documentation/RCU/torture.txt
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/smp.h>
#include <linux/rcupdate.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/atomic.h>
#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/moduleparam.h>
#include <linux/percpu.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/freezer.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/stat.h>
#include <linux/srcu.h>
#include <linux/slab.h>
#include <linux/trace_clock.h>
#include <asm/byteorder.h>
#include <linux/torture.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Paul E. McKenney <paulmck@us.ibm.com> and Josh Triplett <josh@freedesktop.org>");


torture_param(int, fqs_duration, 0,
	      "Duration of fqs bursts (us), 0 to disable");
torture_param(int, fqs_holdoff, 0, "Holdoff time within fqs bursts (us)");
torture_param(int, fqs_stutter, 3, "Wait time between fqs bursts (s)");
torture_param(bool, gp_exp, false, "Use expedited GP wait primitives");
torture_param(bool, gp_normal, false,
	     "Use normal (non-expedited) GP wait primitives");
torture_param(int, irqreader, 1, "Allow RCU readers from irq handlers");
torture_param(int, n_barrier_cbs, 0,
	     "# of callbacks/kthreads for barrier testing");
torture_param(int, nfakewriters, 4, "Number of RCU fake writer threads");
torture_param(int, nreaders, -1, "Number of RCU reader threads");
torture_param(int, object_debug, 0,
	     "Enable debug-object double call_rcu() testing");
torture_param(int, onoff_holdoff, 0, "Time after boot before CPU hotplugs (s)");
torture_param(int, onoff_interval, 0,
	     "Time between CPU hotplugs (s), 0=disable");
torture_param(int, shuffle_interval, 3, "Number of seconds between shuffles");
torture_param(int, shutdown_secs, 0, "Shutdown time (s), <= zero to disable.");
torture_param(int, stall_cpu, 0, "Stall duration (s), zero to disable.");
torture_param(int, stall_cpu_holdoff, 10,
	     "Time to wait before starting stall (s).");
torture_param(int, stat_interval, 60,
	     "Number of seconds between stats printk()s");
torture_param(int, stutter, 5, "Number of seconds to run/halt test");
torture_param(int, test_boost, 1, "Test RCU prio boost: 0=no, 1=maybe, 2=yes.");
torture_param(int, test_boost_duration, 4,
	     "Duration of each boost test, seconds.");
torture_param(int, test_boost_interval, 7,
	     "Interval between boost tests, seconds.");
torture_param(bool, test_no_idle_hz, true,
	     "Test support for tickless idle CPUs");
torture_param(bool, verbose, true,
	     "Enable verbose debugging printk()s");

static char *torture_type = "rcu";
module_param(torture_type, charp, 0444);
MODULE_PARM_DESC(torture_type, "Type of RCU to torture (rcu, rcu_bh, ...)");

static int nrealreaders;
static struct task_struct *writer_task;
static struct task_struct **fakewriter_tasks;
static struct task_struct **reader_tasks;
static struct task_struct *stats_task;
static struct task_struct *fqs_task;
static struct task_struct *boost_tasks[NR_CPUS];
static struct task_struct *stall_task;
static struct task_struct **barrier_cbs_tasks;
static struct task_struct *barrier_task;

#define RCU_TORTURE_PIPE_LEN 10

struct rcu_torture {
	struct rcu_head rtort_rcu;
	int rtort_pipe_count;
	struct list_head rtort_free;
	int rtort_mbtest;
};

static LIST_HEAD(rcu_torture_freelist);
static struct rcu_torture __rcu *rcu_torture_current;
static unsigned long rcu_torture_current_version;
static struct rcu_torture rcu_tortures[10 * RCU_TORTURE_PIPE_LEN];
static DEFINE_SPINLOCK(rcu_torture_lock);
static DEFINE_PER_CPU(long [RCU_TORTURE_PIPE_LEN + 1],
		      rcu_torture_count) = { 0 };
static DEFINE_PER_CPU(long [RCU_TORTURE_PIPE_LEN + 1],
		      rcu_torture_batch) = { 0 };
static atomic_t rcu_torture_wcount[RCU_TORTURE_PIPE_LEN + 1];
static atomic_t n_rcu_torture_alloc;
static atomic_t n_rcu_torture_alloc_fail;
static atomic_t n_rcu_torture_free;
static atomic_t n_rcu_torture_mberror;
static atomic_t n_rcu_torture_error;
static long n_rcu_torture_barrier_error;
static long n_rcu_torture_boost_ktrerror;
static long n_rcu_torture_boost_rterror;
static long n_rcu_torture_boost_failure;
static long n_rcu_torture_boosts;
static long n_rcu_torture_timers;
static long n_barrier_attempts;
static long n_barrier_successes;
static struct list_head rcu_torture_removed;

#if defined(MODULE) || defined(CONFIG_RCU_TORTURE_TEST_RUNNABLE)
#define RCUTORTURE_RUNNABLE_INIT 1
#else
#define RCUTORTURE_RUNNABLE_INIT 0
#endif
int rcutorture_runnable = RCUTORTURE_RUNNABLE_INIT;
module_param(rcutorture_runnable, int, 0444);
MODULE_PARM_DESC(rcutorture_runnable, "Start rcutorture at boot");

#if defined(CONFIG_RCU_BOOST) && !defined(CONFIG_HOTPLUG_CPU)
#define rcu_can_boost() 1
#else /* #if defined(CONFIG_RCU_BOOST) && !defined(CONFIG_HOTPLUG_CPU) */
#define rcu_can_boost() 0
#endif /* #else #if defined(CONFIG_RCU_BOOST) && !defined(CONFIG_HOTPLUG_CPU) */

#ifdef CONFIG_RCU_TRACE
static u64 notrace rcu_trace_clock_local(void)
{
	u64 ts = trace_clock_local();
	unsigned long __maybe_unused ts_rem = do_div(ts, NSEC_PER_USEC);
	return ts;
}
#else /* #ifdef CONFIG_RCU_TRACE */
static u64 notrace rcu_trace_clock_local(void)
{
	return 0ULL;
}
#endif /* #else #ifdef CONFIG_RCU_TRACE */

static unsigned long boost_starttime;	/* jiffies of next boost test start. */
DEFINE_MUTEX(boost_mutex);		/* protect setting boost_starttime */
					/*  and boost task create/destroy. */
static atomic_t barrier_cbs_count;	/* Barrier callbacks registered. */
static bool barrier_phase;		/* Test phase. */
static atomic_t barrier_cbs_invoked;	/* Barrier callbacks invoked. */
static wait_queue_head_t *barrier_cbs_wq; /* Coordinate barrier testing. */
static DECLARE_WAIT_QUEUE_HEAD(barrier_wq);

/*
 * Allocate an element from the rcu_tortures pool.
 */
static struct rcu_torture *
rcu_torture_alloc(void)
{
	struct list_head *p;

	spin_lock_bh(&rcu_torture_lock);
	if (list_empty(&rcu_torture_freelist)) {
		atomic_inc(&n_rcu_torture_alloc_fail);
		spin_unlock_bh(&rcu_torture_lock);
		return NULL;
	}
	atomic_inc(&n_rcu_torture_alloc);
	p = rcu_torture_freelist.next;
	list_del_init(p);
	spin_unlock_bh(&rcu_torture_lock);
	return container_of(p, struct rcu_torture, rtort_free);
}

/*
 * Free an element to the rcu_tortures pool.
 */
static void
rcu_torture_free(struct rcu_torture *p)
{
	atomic_inc(&n_rcu_torture_free);
	spin_lock_bh(&rcu_torture_lock);
	list_add_tail(&p->rtort_free, &rcu_torture_freelist);
	spin_unlock_bh(&rcu_torture_lock);
}

/*
 * Operations vector for selecting different types of tests.
 */

struct rcu_torture_ops {
	void (*init)(void);
	int (*readlock)(void);
	void (*read_delay)(struct torture_random_state *rrsp);
	void (*readunlock)(int idx);
	int (*completed)(void);
	void (*deferred_free)(struct rcu_torture *p);
	void (*sync)(void);
	void (*exp_sync)(void);
	void (*call)(struct rcu_head *head, void (*func)(struct rcu_head *rcu));
	void (*cb_barrier)(void);
	void (*fqs)(void);
	void (*stats)(char *page);
	int irq_capable;
	int can_boost;
	const char *name;
};

static struct rcu_torture_ops *cur_ops;

/*
 * Definitions for rcu torture testing.
 */

static int rcu_torture_read_lock(void) __acquires(RCU)
{
	rcu_read_lock();
	return 0;
}

static void rcu_read_delay(struct torture_random_state *rrsp)
{
	const unsigned long shortdelay_us = 200;
	const unsigned long longdelay_ms = 50;

	/* We want a short delay sometimes to make a reader delay the grace
	 * period, and we want a long delay occasionally to trigger
	 * force_quiescent_state. */

	if (!(torture_random(rrsp) % (nrealreaders * 2000 * longdelay_ms)))
		mdelay(longdelay_ms);
	if (!(torture_random(rrsp) % (nrealreaders * 2 * shortdelay_us)))
		udelay(shortdelay_us);
#ifdef CONFIG_PREEMPT
	if (!preempt_count() &&
	    !(torture_random(rrsp) % (nrealreaders * 20000)))
		preempt_schedule();  /* No QS if preempt_disable() in effect */
#endif
}

static void rcu_torture_read_unlock(int idx) __releases(RCU)
{
	rcu_read_unlock();
}

static int rcu_torture_completed(void)
{
	return rcu_batches_completed();
}

static void
rcu_torture_cb(struct rcu_head *p)
{
	int i;
	struct rcu_torture *rp = container_of(p, struct rcu_torture, rtort_rcu);

	if (torture_must_stop_irq()) {
		/* Test is ending, just drop callbacks on the floor. */
		/* The next initialization will pick up the pieces. */
		return;
	}
	i = rp->rtort_pipe_count;
	if (i > RCU_TORTURE_PIPE_LEN)
		i = RCU_TORTURE_PIPE_LEN;
	atomic_inc(&rcu_torture_wcount[i]);
	if (++rp->rtort_pipe_count >= RCU_TORTURE_PIPE_LEN) {
		rp->rtort_mbtest = 0;
		rcu_torture_free(rp);
	} else {
		cur_ops->deferred_free(rp);
	}
}

static int rcu_no_completed(void)
{
	return 0;
}

static void rcu_torture_deferred_free(struct rcu_torture *p)
{
	call_rcu(&p->rtort_rcu, rcu_torture_cb);
}

static void rcu_sync_torture_init(void)
{
	INIT_LIST_HEAD(&rcu_torture_removed);
}

static struct rcu_torture_ops rcu_ops = {
	.init		= rcu_sync_torture_init,
	.readlock	= rcu_torture_read_lock,
	.read_delay	= rcu_read_delay,
	.readunlock	= rcu_torture_read_unlock,
	.completed	= rcu_torture_completed,
	.deferred_free	= rcu_torture_deferred_free,
	.sync		= synchronize_rcu,
	.exp_sync	= synchronize_rcu_expedited,
	.call		= call_rcu,
	.cb_barrier	= rcu_barrier,
	.fqs		= rcu_force_quiescent_state,
	.stats		= NULL,
	.irq_capable	= 1,
	.can_boost	= rcu_can_boost(),
	.name		= "rcu"
};

/*
 * Definitions for rcu_bh torture testing.
 */

static int rcu_bh_torture_read_lock(void) __acquires(RCU_BH)
{
	rcu_read_lock_bh();
	return 0;
}

static void rcu_bh_torture_read_unlock(int idx) __releases(RCU_BH)
{
	rcu_read_unlock_bh();
}

static int rcu_bh_torture_completed(void)
{
	return rcu_batches_completed_bh();
}

static void rcu_bh_torture_deferred_free(struct rcu_torture *p)
{
	call_rcu_bh(&p->rtort_rcu, rcu_torture_cb);
}

static struct rcu_torture_ops rcu_bh_ops = {
	.init		= rcu_sync_torture_init,
	.readlock	= rcu_bh_torture_read_lock,
	.read_delay	= rcu_read_delay,  /* just reuse rcu's version. */
	.readunlock	= rcu_bh_torture_read_unlock,
	.completed	= rcu_bh_torture_completed,
	.deferred_free	= rcu_bh_torture_deferred_free,
	.sync		= synchronize_rcu_bh,
	.exp_sync	= synchronize_rcu_bh_expedited,
	.call		= call_rcu_bh,
	.cb_barrier	= rcu_barrier_bh,
	.fqs		= rcu_bh_force_quiescent_state,
	.stats		= NULL,
	.irq_capable	= 1,
	.name		= "rcu_bh"
};

/*
 * Don't even think about trying any of these in real life!!!
 * The names includes "busted", and they really means it!
 * The only purpose of these functions is to provide a buggy RCU
 * implementation to make sure that rcutorture correctly emits
 * buggy-RCU error messages.
 */
static void rcu_busted_torture_deferred_free(struct rcu_torture *p)
{
	/* This is a deliberate bug for testing purposes only! */
	rcu_torture_cb(&p->rtort_rcu);
}

static void synchronize_rcu_busted(void)
{
	/* This is a deliberate bug for testing purposes only! */
}

static void
call_rcu_busted(struct rcu_head *head, void (*func)(struct rcu_head *rcu))
{
	/* This is a deliberate bug for testing purposes only! */
	func(head);
}

static struct rcu_torture_ops rcu_busted_ops = {
	.init		= rcu_sync_torture_init,
	.readlock	= rcu_torture_read_lock,
	.read_delay	= rcu_read_delay,  /* just reuse rcu's version. */
	.readunlock	= rcu_torture_read_unlock,
	.completed	= rcu_no_completed,
	.deferred_free	= rcu_busted_torture_deferred_free,
	.sync		= synchronize_rcu_busted,
	.exp_sync	= synchronize_rcu_busted,
	.call		= call_rcu_busted,
	.cb_barrier	= NULL,
	.fqs		= NULL,
	.stats		= NULL,
	.irq_capable	= 1,
	.name		= "rcu_busted"
};

/*
 * Definitions for srcu torture testing.
 */

DEFINE_STATIC_SRCU(srcu_ctl);

static int srcu_torture_read_lock(void) __acquires(&srcu_ctl)
{
	return srcu_read_lock(&srcu_ctl);
}

static void srcu_read_delay(struct torture_random_state *rrsp)
{
	long delay;
	const long uspertick = 1000000 / HZ;
	const long longdelay = 10;

	/* We want there to be long-running readers, but not all the time. */

	delay = torture_random(rrsp) %
		(nrealreaders * 2 * longdelay * uspertick);
	if (!delay)
		schedule_timeout_interruptible(longdelay);
	else
		rcu_read_delay(rrsp);
}

static void srcu_torture_read_unlock(int idx) __releases(&srcu_ctl)
{
	srcu_read_unlock(&srcu_ctl, idx);
}

static int srcu_torture_completed(void)
{
	return srcu_batches_completed(&srcu_ctl);
}

static void srcu_torture_deferred_free(struct rcu_torture *rp)
{
	call_srcu(&srcu_ctl, &rp->rtort_rcu, rcu_torture_cb);
}

static void srcu_torture_synchronize(void)
{
	synchronize_srcu(&srcu_ctl);
}

static void srcu_torture_call(struct rcu_head *head,
			      void (*func)(struct rcu_head *head))
{
	call_srcu(&srcu_ctl, head, func);
}

static void srcu_torture_barrier(void)
{
	srcu_barrier(&srcu_ctl);
}

static void srcu_torture_stats(char *page)
{
	int cpu;
	int idx = srcu_ctl.completed & 0x1;

	page += sprintf(page, "%s%s per-CPU(idx=%d):",
		       torture_type, TORTURE_FLAG, idx);
	for_each_possible_cpu(cpu) {
		page += sprintf(page, " %d(%lu,%lu)", cpu,
			       per_cpu_ptr(srcu_ctl.per_cpu_ref, cpu)->c[!idx],
			       per_cpu_ptr(srcu_ctl.per_cpu_ref, cpu)->c[idx]);
	}
	sprintf(page, "\n");
}

static void srcu_torture_synchronize_expedited(void)
{
	synchronize_srcu_expedited(&srcu_ctl);
}

static struct rcu_torture_ops srcu_ops = {
	.init		= rcu_sync_torture_init,
	.readlock	= srcu_torture_read_lock,
	.read_delay	= srcu_read_delay,
	.readunlock	= srcu_torture_read_unlock,
	.completed	= srcu_torture_completed,
	.deferred_free	= srcu_torture_deferred_free,
	.sync		= srcu_torture_synchronize,
	.exp_sync	= srcu_torture_synchronize_expedited,
	.call		= srcu_torture_call,
	.cb_barrier	= srcu_torture_barrier,
	.stats		= srcu_torture_stats,
	.name		= "srcu"
};

/*
 * Definitions for sched torture testing.
 */

static int sched_torture_read_lock(void)
{
	preempt_disable();
	return 0;
}

static void sched_torture_read_unlock(int idx)
{
	preempt_enable();
}

static void rcu_sched_torture_deferred_free(struct rcu_torture *p)
{
	call_rcu_sched(&p->rtort_rcu, rcu_torture_cb);
}

static struct rcu_torture_ops sched_ops = {
	.init		= rcu_sync_torture_init,
	.readlock	= sched_torture_read_lock,
	.read_delay	= rcu_read_delay,  /* just reuse rcu's version. */
	.readunlock	= sched_torture_read_unlock,
	.completed	= rcu_no_completed,
	.deferred_free	= rcu_sched_torture_deferred_free,
	.sync		= synchronize_sched,
	.exp_sync	= synchronize_sched_expedited,
	.call		= call_rcu_sched,
	.cb_barrier	= rcu_barrier_sched,
	.fqs		= rcu_sched_force_quiescent_state,
	.stats		= NULL,
	.irq_capable	= 1,
	.name		= "sched"
};

/*
 * RCU torture priority-boost testing.  Runs one real-time thread per
 * CPU for moderate bursts, repeatedly registering RCU callbacks and
 * spinning waiting for them to be invoked.  If a given callback takes
 * too long to be invoked, we assume that priority inversion has occurred.
 */

struct rcu_boost_inflight {
	struct rcu_head rcu;
	int inflight;
};

static void rcu_torture_boost_cb(struct rcu_head *head)
{
	struct rcu_boost_inflight *rbip =
		container_of(head, struct rcu_boost_inflight, rcu);

	smp_mb(); /* Ensure RCU-core accesses precede clearing ->inflight */
	rbip->inflight = 0;
}

static int rcu_torture_boost(void *arg)
{
	unsigned long call_rcu_time;
	unsigned long endtime;
	unsigned long oldstarttime;
	struct rcu_boost_inflight rbi = { .inflight = 0 };
	struct sched_param sp;

	VERBOSE_TOROUT_STRING("rcu_torture_boost started");

	/* Set real-time priority. */
	sp.sched_priority = 1;
	if (sched_setscheduler(current, SCHED_FIFO, &sp) < 0) {
		VERBOSE_TOROUT_STRING("rcu_torture_boost RT prio failed!");
		n_rcu_torture_boost_rterror++;
	}

	init_rcu_head_on_stack(&rbi.rcu);
	/* Each pass through the following loop does one boost-test cycle. */
	do {
		/* Wait for the next test interval. */
		oldstarttime = boost_starttime;
		while (ULONG_CMP_LT(jiffies, oldstarttime)) {
			schedule_timeout_interruptible(oldstarttime - jiffies);
			stutter_wait("rcu_torture_boost");
			if (torture_must_stop())
				goto checkwait;
		}

		/* Do one boost-test interval. */
		endtime = oldstarttime + test_boost_duration * HZ;
		call_rcu_time = jiffies;
		while (ULONG_CMP_LT(jiffies, endtime)) {
			/* If we don't have a callback in flight, post one. */
			if (!rbi.inflight) {
				smp_mb(); /* RCU core before ->inflight = 1. */
				rbi.inflight = 1;
				call_rcu(&rbi.rcu, rcu_torture_boost_cb);
				if (jiffies - call_rcu_time >
					 test_boost_duration * HZ - HZ / 2) {
					VERBOSE_TOROUT_STRING("rcu_torture_boost boosting failed");
					n_rcu_torture_boost_failure++;
				}
				call_rcu_time = jiffies;
			}
			cond_resched();
			stutter_wait("rcu_torture_boost");
			if (torture_must_stop())
				goto checkwait;
		}

		/*
		 * Set the start time of the next test interval.
		 * Yes, this is vulnerable to long delays, but such
		 * delays simply cause a false negative for the next
		 * interval.  Besides, we are running at RT priority,
		 * so delays should be relatively rare.
		 */
		while (oldstarttime == boost_starttime &&
		       !kthread_should_stop()) {
			if (mutex_trylock(&boost_mutex)) {
				boost_starttime = jiffies +
						  test_boost_interval * HZ;
				n_rcu_torture_boosts++;
				mutex_unlock(&boost_mutex);
				break;
			}
			schedule_timeout_uninterruptible(1);
		}

		/* Go do the stutter. */
checkwait:	stutter_wait("rcu_torture_boost");
	} while (!torture_must_stop());

	/* Clean up and exit. */
	while (!kthread_should_stop() || rbi.inflight) {
		torture_shutdown_absorb("rcu_torture_boost");
		schedule_timeout_uninterruptible(1);
	}
	smp_mb(); /* order accesses to ->inflight before stack-frame death. */
	destroy_rcu_head_on_stack(&rbi.rcu);
	torture_kthread_stopping("rcu_torture_boost");
	return 0;
}

/*
 * RCU torture force-quiescent-state kthread.  Repeatedly induces
 * bursts of calls to force_quiescent_state(), increasing the probability
 * of occurrence of some important types of race conditions.
 */
static int
rcu_torture_fqs(void *arg)
{
	unsigned long fqs_resume_time;
	int fqs_burst_remaining;

	VERBOSE_TOROUT_STRING("rcu_torture_fqs task started");
	do {
		fqs_resume_time = jiffies + fqs_stutter * HZ;
		while (ULONG_CMP_LT(jiffies, fqs_resume_time) &&
		       !kthread_should_stop()) {
			schedule_timeout_interruptible(1);
		}
		fqs_burst_remaining = fqs_duration;
		while (fqs_burst_remaining > 0 &&
		       !kthread_should_stop()) {
			cur_ops->fqs();
			udelay(fqs_holdoff);
			fqs_burst_remaining -= fqs_holdoff;
		}
		stutter_wait("rcu_torture_fqs");
	} while (!torture_must_stop());
	torture_kthread_stopping("rcu_torture_fqs");
	return 0;
}

/*
 * RCU torture writer kthread.  Repeatedly substitutes a new structure
 * for that pointed to by rcu_torture_current, freeing the old structure
 * after a series of grace periods (the "pipeline").
 */
static int
rcu_torture_writer(void *arg)
{
	bool exp;
	int i;
	struct rcu_torture *rp;
	struct rcu_torture *rp1;
	struct rcu_torture *old_rp;
	static DEFINE_TORTURE_RANDOM(rand);

	VERBOSE_TOROUT_STRING("rcu_torture_writer task started");
	set_user_nice(current, MAX_NICE);

	do {
		schedule_timeout_uninterruptible(1);
		rp = rcu_torture_alloc();
		if (rp == NULL)
			continue;
		rp->rtort_pipe_count = 0;
		udelay(torture_random(&rand) & 0x3ff);
		old_rp = rcu_dereference_check(rcu_torture_current,
					       current == writer_task);
		rp->rtort_mbtest = 1;
		rcu_assign_pointer(rcu_torture_current, rp);
		smp_wmb(); /* Mods to old_rp must follow rcu_assign_pointer() */
		if (old_rp) {
			i = old_rp->rtort_pipe_count;
			if (i > RCU_TORTURE_PIPE_LEN)
				i = RCU_TORTURE_PIPE_LEN;
			atomic_inc(&rcu_torture_wcount[i]);
			old_rp->rtort_pipe_count++;
			if (gp_normal == gp_exp)
				exp = !!(torture_random(&rand) & 0x80);
			else
				exp = gp_exp;
			if (!exp) {
				cur_ops->deferred_free(old_rp);
			} else {
				cur_ops->exp_sync();
				list_add(&old_rp->rtort_free,
					 &rcu_torture_removed);
				list_for_each_entry_safe(rp, rp1,
							 &rcu_torture_removed,
							 rtort_free) {
					i = rp->rtort_pipe_count;
					if (i > RCU_TORTURE_PIPE_LEN)
						i = RCU_TORTURE_PIPE_LEN;
					atomic_inc(&rcu_torture_wcount[i]);
					if (++rp->rtort_pipe_count >=
					    RCU_TORTURE_PIPE_LEN) {
						rp->rtort_mbtest = 0;
						list_del(&rp->rtort_free);
						rcu_torture_free(rp);
					}
				 }
			}
		}
		rcutorture_record_progress(++rcu_torture_current_version);
		stutter_wait("rcu_torture_writer");
	} while (!torture_must_stop());
	torture_kthread_stopping("rcu_torture_writer");
	return 0;
}

/*
 * RCU torture fake writer kthread.  Repeatedly calls sync, with a random
 * delay between calls.
 */
static int
rcu_torture_fakewriter(void *arg)
{
	DEFINE_TORTURE_RANDOM(rand);

	VERBOSE_TOROUT_STRING("rcu_torture_fakewriter task started");
	set_user_nice(current, MAX_NICE);

	do {
		schedule_timeout_uninterruptible(1 + torture_random(&rand)%10);
		udelay(torture_random(&rand) & 0x3ff);
		if (cur_ops->cb_barrier != NULL &&
		    torture_random(&rand) % (nfakewriters * 8) == 0) {
			cur_ops->cb_barrier();
		} else if (gp_normal == gp_exp) {
			if (torture_random(&rand) & 0x80)
				cur_ops->sync();
			else
				cur_ops->exp_sync();
		} else if (gp_normal) {
			cur_ops->sync();
		} else {
			cur_ops->exp_sync();
		}
		stutter_wait("rcu_torture_fakewriter");
	} while (!torture_must_stop());

	torture_kthread_stopping("rcu_torture_fakewriter");
	return 0;
}

void rcutorture_trace_dump(void)
{
	static atomic_t beenhere = ATOMIC_INIT(0);

	if (atomic_read(&beenhere))
		return;
	if (atomic_xchg(&beenhere, 1) != 0)
		return;
	ftrace_dump(DUMP_ALL);
}

/*
 * RCU torture reader from timer handler.  Dereferences rcu_torture_current,
 * incrementing the corresponding element of the pipeline array.  The
 * counter in the element should never be greater than 1, otherwise, the
 * RCU implementation is broken.
 */
static void rcu_torture_timer(unsigned long unused)
{
	int idx;
	int completed;
	int completed_end;
	static DEFINE_TORTURE_RANDOM(rand);
	static DEFINE_SPINLOCK(rand_lock);
	struct rcu_torture *p;
	int pipe_count;
	unsigned long long ts;

	idx = cur_ops->readlock();
	completed = cur_ops->completed();
	ts = rcu_trace_clock_local();
	p = rcu_dereference_check(rcu_torture_current,
				  rcu_read_lock_bh_held() ||
				  rcu_read_lock_sched_held() ||
				  srcu_read_lock_held(&srcu_ctl));
	if (p == NULL) {
		/* Leave because rcu_torture_writer is not yet underway */
		cur_ops->readunlock(idx);
		return;
	}
	if (p->rtort_mbtest == 0)
		atomic_inc(&n_rcu_torture_mberror);
	spin_lock(&rand_lock);
	cur_ops->read_delay(&rand);
	n_rcu_torture_timers++;
	spin_unlock(&rand_lock);
	preempt_disable();
	pipe_count = p->rtort_pipe_count;
	if (pipe_count > RCU_TORTURE_PIPE_LEN) {
		/* Should not happen, but... */
		pipe_count = RCU_TORTURE_PIPE_LEN;
	}
	completed_end = cur_ops->completed();
	if (pipe_count > 1) {
		do_trace_rcu_torture_read(cur_ops->name, &p->rtort_rcu, ts,
					  completed, completed_end);
		rcutorture_trace_dump();
	}
	__this_cpu_inc(rcu_torture_count[pipe_count]);
	completed = completed_end - completed;
	if (completed > RCU_TORTURE_PIPE_LEN) {
		/* Should not happen, but... */
		completed = RCU_TORTURE_PIPE_LEN;
	}
	__this_cpu_inc(rcu_torture_batch[completed]);
	preempt_enable();
	cur_ops->readunlock(idx);
}

/*
 * RCU torture reader kthread.  Repeatedly dereferences rcu_torture_current,
 * incrementing the corresponding element of the pipeline array.  The
 * counter in the element should never be greater than 1, otherwise, the
 * RCU implementation is broken.
 */
static int
rcu_torture_reader(void *arg)
{
	int completed;
	int completed_end;
	int idx;
	DEFINE_TORTURE_RANDOM(rand);
	struct rcu_torture *p;
	int pipe_count;
	struct timer_list t;
	unsigned long long ts;

	VERBOSE_TOROUT_STRING("rcu_torture_reader task started");
	set_user_nice(current, MAX_NICE);
	if (irqreader && cur_ops->irq_capable)
		setup_timer_on_stack(&t, rcu_torture_timer, 0);

	do {
		if (irqreader && cur_ops->irq_capable) {
			if (!timer_pending(&t))
				mod_timer(&t, jiffies + 1);
		}
		idx = cur_ops->readlock();
		completed = cur_ops->completed();
		ts = rcu_trace_clock_local();
		p = rcu_dereference_check(rcu_torture_current,
					  rcu_read_lock_bh_held() ||
					  rcu_read_lock_sched_held() ||
					  srcu_read_lock_held(&srcu_ctl));
		if (p == NULL) {
			/* Wait for rcu_torture_writer to get underway */
			cur_ops->readunlock(idx);
			schedule_timeout_interruptible(HZ);
			continue;
		}
		if (p->rtort_mbtest == 0)
			atomic_inc(&n_rcu_torture_mberror);
		cur_ops->read_delay(&rand);
		preempt_disable();
		pipe_count = p->rtort_pipe_count;
		if (pipe_count > RCU_TORTURE_PIPE_LEN) {
			/* Should not happen, but... */
			pipe_count = RCU_TORTURE_PIPE_LEN;
		}
		completed_end = cur_ops->completed();
		if (pipe_count > 1) {
			do_trace_rcu_torture_read(cur_ops->name, &p->rtort_rcu,
						  ts, completed, completed_end);
			rcutorture_trace_dump();
		}
		__this_cpu_inc(rcu_torture_count[pipe_count]);
		completed = completed_end - completed;
		if (completed > RCU_TORTURE_PIPE_LEN) {
			/* Should not happen, but... */
			completed = RCU_TORTURE_PIPE_LEN;
		}
		__this_cpu_inc(rcu_torture_batch[completed]);
		preempt_enable();
		cur_ops->readunlock(idx);
		schedule();
		stutter_wait("rcu_torture_reader");
	} while (!torture_must_stop());
	if (irqreader && cur_ops->irq_capable)
		del_timer_sync(&t);
	torture_kthread_stopping("rcu_torture_reader");
	return 0;
}

/*
 * Create an RCU-torture statistics message in the specified buffer.
 */
static void
rcu_torture_printk(char *page)
{
	int cpu;
	int i;
	long pipesummary[RCU_TORTURE_PIPE_LEN + 1] = { 0 };
	long batchsummary[RCU_TORTURE_PIPE_LEN + 1] = { 0 };

	for_each_possible_cpu(cpu) {
		for (i = 0; i < RCU_TORTURE_PIPE_LEN + 1; i++) {
			pipesummary[i] += per_cpu(rcu_torture_count, cpu)[i];
			batchsummary[i] += per_cpu(rcu_torture_batch, cpu)[i];
		}
	}
	for (i = RCU_TORTURE_PIPE_LEN - 1; i >= 0; i--) {
		if (pipesummary[i] != 0)
			break;
	}
	page += sprintf(page, "%s%s ", torture_type, TORTURE_FLAG);
	page += sprintf(page,
		       "rtc: %p ver: %lu tfle: %d rta: %d rtaf: %d rtf: %d ",
		       rcu_torture_current,
		       rcu_torture_current_version,
		       list_empty(&rcu_torture_freelist),
		       atomic_read(&n_rcu_torture_alloc),
		       atomic_read(&n_rcu_torture_alloc_fail),
		       atomic_read(&n_rcu_torture_free));
	page += sprintf(page, "rtmbe: %d rtbke: %ld rtbre: %ld ",
		       atomic_read(&n_rcu_torture_mberror),
		       n_rcu_torture_boost_ktrerror,
		       n_rcu_torture_boost_rterror);
	page += sprintf(page, "rtbf: %ld rtb: %ld nt: %ld ",
		       n_rcu_torture_boost_failure,
		       n_rcu_torture_boosts,
		       n_rcu_torture_timers);
	page = torture_onoff_stats(page);
	page += sprintf(page, "barrier: %ld/%ld:%ld",
		       n_barrier_successes,
		       n_barrier_attempts,
		       n_rcu_torture_barrier_error);
	page += sprintf(page, "\n%s%s ", torture_type, TORTURE_FLAG);
	if (atomic_read(&n_rcu_torture_mberror) != 0 ||
	    n_rcu_torture_barrier_error != 0 ||
	    n_rcu_torture_boost_ktrerror != 0 ||
	    n_rcu_torture_boost_rterror != 0 ||
	    n_rcu_torture_boost_failure != 0 ||
	    i > 1) {
		page += sprintf(page, "!!! ");
		atomic_inc(&n_rcu_torture_error);
		WARN_ON_ONCE(1);
	}
	page += sprintf(page, "Reader Pipe: ");
	for (i = 0; i < RCU_TORTURE_PIPE_LEN + 1; i++)
		page += sprintf(page, " %ld", pipesummary[i]);
	page += sprintf(page, "\n%s%s ", torture_type, TORTURE_FLAG);
	page += sprintf(page, "Reader Batch: ");
	for (i = 0; i < RCU_TORTURE_PIPE_LEN + 1; i++)
		page += sprintf(page, " %ld", batchsummary[i]);
	page += sprintf(page, "\n%s%s ", torture_type, TORTURE_FLAG);
	page += sprintf(page, "Free-Block Circulation: ");
	for (i = 0; i < RCU_TORTURE_PIPE_LEN + 1; i++) {
		page += sprintf(page, " %d",
			       atomic_read(&rcu_torture_wcount[i]));
	}
	page += sprintf(page, "\n");
	if (cur_ops->stats)
		cur_ops->stats(page);
}

/*
 * Print torture statistics.  Caller must ensure that there is only
 * one call to this function at a given time!!!  This is normally
 * accomplished by relying on the module system to only have one copy
 * of the module loaded, and then by giving the rcu_torture_stats
 * kthread full control (or the init/cleanup functions when rcu_torture_stats
 * thread is not running).
 */
static void
rcu_torture_stats_print(void)
{
	int size = nr_cpu_ids * 200 + 8192;
	char *buf;

	buf = kmalloc(size, GFP_KERNEL);
	if (!buf) {
		pr_err("rcu-torture: Out of memory, need: %d", size);
		return;
	}
	rcu_torture_printk(buf);
	pr_alert("%s", buf);
	kfree(buf);
}

/*
 * Periodically prints torture statistics, if periodic statistics printing
 * was specified via the stat_interval module parameter.
 */
static int
rcu_torture_stats(void *arg)
{
	VERBOSE_TOROUT_STRING("rcu_torture_stats task started");
	do {
		schedule_timeout_interruptible(stat_interval * HZ);
		rcu_torture_stats_print();
		torture_shutdown_absorb("rcu_torture_stats");
	} while (!torture_must_stop());
	torture_kthread_stopping("rcu_torture_stats");
	return 0;
}

static inline void
rcu_torture_print_module_parms(struct rcu_torture_ops *cur_ops, const char *tag)
{
	pr_alert("%s" TORTURE_FLAG
		 "--- %s: nreaders=%d nfakewriters=%d "
		 "stat_interval=%d verbose=%d test_no_idle_hz=%d "
		 "shuffle_interval=%d stutter=%d irqreader=%d "
		 "fqs_duration=%d fqs_holdoff=%d fqs_stutter=%d "
		 "test_boost=%d/%d test_boost_interval=%d "
		 "test_boost_duration=%d shutdown_secs=%d "
		 "stall_cpu=%d stall_cpu_holdoff=%d "
		 "n_barrier_cbs=%d "
		 "onoff_interval=%d onoff_holdoff=%d\n",
		 torture_type, tag, nrealreaders, nfakewriters,
		 stat_interval, verbose, test_no_idle_hz, shuffle_interval,
		 stutter, irqreader, fqs_duration, fqs_holdoff, fqs_stutter,
		 test_boost, cur_ops->can_boost,
		 test_boost_interval, test_boost_duration, shutdown_secs,
		 stall_cpu, stall_cpu_holdoff,
		 n_barrier_cbs,
		 onoff_interval, onoff_holdoff);
}

static void rcutorture_booster_cleanup(int cpu)
{
	struct task_struct *t;

	if (boost_tasks[cpu] == NULL)
		return;
	mutex_lock(&boost_mutex);
	t = boost_tasks[cpu];
	boost_tasks[cpu] = NULL;
	mutex_unlock(&boost_mutex);

	/* This must be outside of the mutex, otherwise deadlock! */
	torture_stop_kthread(rcu_torture_boost, t);
}

static int rcutorture_booster_init(int cpu)
{
	int retval;

	if (boost_tasks[cpu] != NULL)
		return 0;  /* Already created, nothing more to do. */

	/* Don't allow time recalculation while creating a new task. */
	mutex_lock(&boost_mutex);
	VERBOSE_TOROUT_STRING("Creating rcu_torture_boost task");
	boost_tasks[cpu] = kthread_create_on_node(rcu_torture_boost, NULL,
						  cpu_to_node(cpu),
						  "rcu_torture_boost");
	if (IS_ERR(boost_tasks[cpu])) {
		retval = PTR_ERR(boost_tasks[cpu]);
		VERBOSE_TOROUT_STRING("rcu_torture_boost task create failed");
		n_rcu_torture_boost_ktrerror++;
		boost_tasks[cpu] = NULL;
		mutex_unlock(&boost_mutex);
		return retval;
	}
	kthread_bind(boost_tasks[cpu], cpu);
	wake_up_process(boost_tasks[cpu]);
	mutex_unlock(&boost_mutex);
	return 0;
}

/*
 * CPU-stall kthread.  It waits as specified by stall_cpu_holdoff, then
 * induces a CPU stall for the time specified by stall_cpu.
 */
static int rcu_torture_stall(void *args)
{
	unsigned long stop_at;

	VERBOSE_TOROUT_STRING("rcu_torture_stall task started");
	if (stall_cpu_holdoff > 0) {
		VERBOSE_TOROUT_STRING("rcu_torture_stall begin holdoff");
		schedule_timeout_interruptible(stall_cpu_holdoff * HZ);
		VERBOSE_TOROUT_STRING("rcu_torture_stall end holdoff");
	}
	if (!kthread_should_stop()) {
		stop_at = get_seconds() + stall_cpu;
		/* RCU CPU stall is expected behavior in following code. */
		pr_alert("rcu_torture_stall start.\n");
		rcu_read_lock();
		preempt_disable();
		while (ULONG_CMP_LT(get_seconds(), stop_at))
			continue;  /* Induce RCU CPU stall warning. */
		preempt_enable();
		rcu_read_unlock();
		pr_alert("rcu_torture_stall end.\n");
	}
	torture_shutdown_absorb("rcu_torture_stall");
	while (!kthread_should_stop())
		schedule_timeout_interruptible(10 * HZ);
	return 0;
}

/* Spawn CPU-stall kthread, if stall_cpu specified. */
static int __init rcu_torture_stall_init(void)
{
	if (stall_cpu <= 0)
		return 0;
	return torture_create_kthread(rcu_torture_stall, NULL, stall_task);
}

/* Callback function for RCU barrier testing. */
void rcu_torture_barrier_cbf(struct rcu_head *rcu)
{
	atomic_inc(&barrier_cbs_invoked);
}

/* kthread function to register callbacks used to test RCU barriers. */
static int rcu_torture_barrier_cbs(void *arg)
{
	long myid = (long)arg;
	bool lastphase = 0;
	bool newphase;
	struct rcu_head rcu;

	init_rcu_head_on_stack(&rcu);
	VERBOSE_TOROUT_STRING("rcu_torture_barrier_cbs task started");
	set_user_nice(current, MAX_NICE);
	do {
		wait_event(barrier_cbs_wq[myid],
			   (newphase =
			    ACCESS_ONCE(barrier_phase)) != lastphase ||
			   torture_must_stop());
		lastphase = newphase;
		smp_mb(); /* ensure barrier_phase load before ->call(). */
		if (torture_must_stop())
			break;
		cur_ops->call(&rcu, rcu_torture_barrier_cbf);
		if (atomic_dec_and_test(&barrier_cbs_count))
			wake_up(&barrier_wq);
	} while (!torture_must_stop());
	cur_ops->cb_barrier();
	destroy_rcu_head_on_stack(&rcu);
	torture_kthread_stopping("rcu_torture_barrier_cbs");
	return 0;
}

/* kthread function to drive and coordinate RCU barrier testing. */
static int rcu_torture_barrier(void *arg)
{
	int i;

	VERBOSE_TOROUT_STRING("rcu_torture_barrier task starting");
	do {
		atomic_set(&barrier_cbs_invoked, 0);
		atomic_set(&barrier_cbs_count, n_barrier_cbs);
		smp_mb(); /* Ensure barrier_phase after prior assignments. */
		barrier_phase = !barrier_phase;
		for (i = 0; i < n_barrier_cbs; i++)
			wake_up(&barrier_cbs_wq[i]);
		wait_event(barrier_wq,
			   atomic_read(&barrier_cbs_count) == 0 ||
			   torture_must_stop());
		if (torture_must_stop())
			break;
		n_barrier_attempts++;
		cur_ops->cb_barrier(); /* Implies smp_mb() for wait_event(). */
		if (atomic_read(&barrier_cbs_invoked) != n_barrier_cbs) {
			n_rcu_torture_barrier_error++;
			WARN_ON_ONCE(1);
		}
		n_barrier_successes++;
		schedule_timeout_interruptible(HZ / 10);
	} while (!torture_must_stop());
	torture_kthread_stopping("rcu_torture_barrier");
	return 0;
}

/* Initialize RCU barrier testing. */
static int rcu_torture_barrier_init(void)
{
	int i;
	int ret;

	if (n_barrier_cbs == 0)
		return 0;
	if (cur_ops->call == NULL || cur_ops->cb_barrier == NULL) {
		pr_alert("%s" TORTURE_FLAG
			 " Call or barrier ops missing for %s,\n",
			 torture_type, cur_ops->name);
		pr_alert("%s" TORTURE_FLAG
			 " RCU barrier testing omitted from run.\n",
			 torture_type);
		return 0;
	}
	atomic_set(&barrier_cbs_count, 0);
	atomic_set(&barrier_cbs_invoked, 0);
	barrier_cbs_tasks =
		kzalloc(n_barrier_cbs * sizeof(barrier_cbs_tasks[0]),
			GFP_KERNEL);
	barrier_cbs_wq =
		kzalloc(n_barrier_cbs * sizeof(barrier_cbs_wq[0]),
			GFP_KERNEL);
	if (barrier_cbs_tasks == NULL || !barrier_cbs_wq)
		return -ENOMEM;
	for (i = 0; i < n_barrier_cbs; i++) {
		init_waitqueue_head(&barrier_cbs_wq[i]);
		ret = torture_create_kthread(rcu_torture_barrier_cbs,
					     (void *)(long)i,
					     barrier_cbs_tasks[i]);
		if (ret)
			return ret;
	}
	return torture_create_kthread(rcu_torture_barrier, NULL, barrier_task);
}

/* Clean up after RCU barrier testing. */
static void rcu_torture_barrier_cleanup(void)
{
	int i;

	torture_stop_kthread(rcu_torture_barrier, barrier_task);
	if (barrier_cbs_tasks != NULL) {
		for (i = 0; i < n_barrier_cbs; i++)
			torture_stop_kthread(rcu_torture_barrier_cbs,
					     barrier_cbs_tasks[i]);
		kfree(barrier_cbs_tasks);
		barrier_cbs_tasks = NULL;
	}
	if (barrier_cbs_wq != NULL) {
		kfree(barrier_cbs_wq);
		barrier_cbs_wq = NULL;
	}
}

static int rcutorture_cpu_notify(struct notifier_block *self,
				 unsigned long action, void *hcpu)
{
	long cpu = (long)hcpu;

	switch (action) {
	case CPU_ONLINE:
	case CPU_DOWN_FAILED:
		(void)rcutorture_booster_init(cpu);
		break;
	case CPU_DOWN_PREPARE:
		rcutorture_booster_cleanup(cpu);
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block rcutorture_cpu_nb = {
	.notifier_call = rcutorture_cpu_notify,
};

static void
rcu_torture_cleanup(void)
{
	int i;

	rcutorture_record_test_transition();
	if (torture_cleanup()) {
		if (cur_ops->cb_barrier != NULL)
			cur_ops->cb_barrier();
		return;
	}

	rcu_torture_barrier_cleanup();
	torture_stop_kthread(rcu_torture_stall, stall_task);
	torture_stop_kthread(rcu_torture_writer, writer_task);

	if (reader_tasks) {
		for (i = 0; i < nrealreaders; i++)
			torture_stop_kthread(rcu_torture_reader,
					     reader_tasks[i]);
		kfree(reader_tasks);
	}
	rcu_torture_current = NULL;

	if (fakewriter_tasks) {
		for (i = 0; i < nfakewriters; i++) {
			torture_stop_kthread(rcu_torture_fakewriter,
					     fakewriter_tasks[i]);
		}
		kfree(fakewriter_tasks);
		fakewriter_tasks = NULL;
	}

	torture_stop_kthread(rcu_torture_stats, stats_task);
	torture_stop_kthread(rcu_torture_fqs, fqs_task);
	if ((test_boost == 1 && cur_ops->can_boost) ||
	    test_boost == 2) {
		unregister_cpu_notifier(&rcutorture_cpu_nb);
		for_each_possible_cpu(i)
			rcutorture_booster_cleanup(i);
	}

	/* Wait for all RCU callbacks to fire.  */

	if (cur_ops->cb_barrier != NULL)
		cur_ops->cb_barrier();

	rcu_torture_stats_print();  /* -After- the stats thread is stopped! */

	if (atomic_read(&n_rcu_torture_error) || n_rcu_torture_barrier_error)
		rcu_torture_print_module_parms(cur_ops, "End of test: FAILURE");
	else if (torture_onoff_failures())
		rcu_torture_print_module_parms(cur_ops,
					       "End of test: RCU_HOTPLUG");
	else
		rcu_torture_print_module_parms(cur_ops, "End of test: SUCCESS");
}

#ifdef CONFIG_DEBUG_OBJECTS_RCU_HEAD
static void rcu_torture_leak_cb(struct rcu_head *rhp)
{
}

static void rcu_torture_err_cb(struct rcu_head *rhp)
{
	/*
	 * This -might- happen due to race conditions, but is unlikely.
	 * The scenario that leads to this happening is that the
	 * first of the pair of duplicate callbacks is queued,
	 * someone else starts a grace period that includes that
	 * callback, then the second of the pair must wait for the
	 * next grace period.  Unlikely, but can happen.  If it
	 * does happen, the debug-objects subsystem won't have splatted.
	 */
	pr_alert("rcutorture: duplicated callback was invoked.\n");
}
#endif /* #ifdef CONFIG_DEBUG_OBJECTS_RCU_HEAD */

/*
 * Verify that double-free causes debug-objects to complain, but only
 * if CONFIG_DEBUG_OBJECTS_RCU_HEAD=y.  Otherwise, say that the test
 * cannot be carried out.
 */
static void rcu_test_debug_objects(void)
{
#ifdef CONFIG_DEBUG_OBJECTS_RCU_HEAD
	struct rcu_head rh1;
	struct rcu_head rh2;

	init_rcu_head_on_stack(&rh1);
	init_rcu_head_on_stack(&rh2);
	pr_alert("rcutorture: WARN: Duplicate call_rcu() test starting.\n");

	/* Try to queue the rh2 pair of callbacks for the same grace period. */
	preempt_disable(); /* Prevent preemption from interrupting test. */
	rcu_read_lock(); /* Make it impossible to finish a grace period. */
	call_rcu(&rh1, rcu_torture_leak_cb); /* Start grace period. */
	local_irq_disable(); /* Make it harder to start a new grace period. */
	call_rcu(&rh2, rcu_torture_leak_cb);
	call_rcu(&rh2, rcu_torture_err_cb); /* Duplicate callback. */
	local_irq_enable();
	rcu_read_unlock();
	preempt_enable();

	/* Wait for them all to get done so we can safely return. */
	rcu_barrier();
	pr_alert("rcutorture: WARN: Duplicate call_rcu() test complete.\n");
	destroy_rcu_head_on_stack(&rh1);
	destroy_rcu_head_on_stack(&rh2);
#else /* #ifdef CONFIG_DEBUG_OBJECTS_RCU_HEAD */
	pr_alert("rcutorture: !CONFIG_DEBUG_OBJECTS_RCU_HEAD, not testing duplicate call_rcu()\n");
#endif /* #else #ifdef CONFIG_DEBUG_OBJECTS_RCU_HEAD */
}

static int __init
rcu_torture_init(void)
{
	int i;
	int cpu;
	int firsterr = 0;
	static struct rcu_torture_ops *torture_ops[] = {
		&rcu_ops, &rcu_bh_ops, &rcu_busted_ops, &srcu_ops, &sched_ops,
	};

	torture_init_begin(torture_type, verbose, &rcutorture_runnable);

	/* Process args and tell the world that the torturer is on the job. */
	for (i = 0; i < ARRAY_SIZE(torture_ops); i++) {
		cur_ops = torture_ops[i];
		if (strcmp(torture_type, cur_ops->name) == 0)
			break;
	}
	if (i == ARRAY_SIZE(torture_ops)) {
		pr_alert("rcu-torture: invalid torture type: \"%s\"\n",
			 torture_type);
		pr_alert("rcu-torture types:");
		for (i = 0; i < ARRAY_SIZE(torture_ops); i++)
			pr_alert(" %s", torture_ops[i]->name);
		pr_alert("\n");
		torture_init_end();
		return -EINVAL;
	}
	if (cur_ops->fqs == NULL && fqs_duration != 0) {
		pr_alert("rcu-torture: ->fqs NULL and non-zero fqs_duration, fqs disabled.\n");
		fqs_duration = 0;
	}
	if (cur_ops->init)
		cur_ops->init(); /* no "goto unwind" prior to this point!!! */

	if (nreaders >= 0)
		nrealreaders = nreaders;
	else
		nrealreaders = 2 * num_online_cpus();
	rcu_torture_print_module_parms(cur_ops, "Start of test");

	/* Set up the freelist. */

	INIT_LIST_HEAD(&rcu_torture_freelist);
	for (i = 0; i < ARRAY_SIZE(rcu_tortures); i++) {
		rcu_tortures[i].rtort_mbtest = 0;
		list_add_tail(&rcu_tortures[i].rtort_free,
			      &rcu_torture_freelist);
	}

	/* Initialize the statistics so that each run gets its own numbers. */

	rcu_torture_current = NULL;
	rcu_torture_current_version = 0;
	atomic_set(&n_rcu_torture_alloc, 0);
	atomic_set(&n_rcu_torture_alloc_fail, 0);
	atomic_set(&n_rcu_torture_free, 0);
	atomic_set(&n_rcu_torture_mberror, 0);
	atomic_set(&n_rcu_torture_error, 0);
	n_rcu_torture_barrier_error = 0;
	n_rcu_torture_boost_ktrerror = 0;
	n_rcu_torture_boost_rterror = 0;
	n_rcu_torture_boost_failure = 0;
	n_rcu_torture_boosts = 0;
	for (i = 0; i < RCU_TORTURE_PIPE_LEN + 1; i++)
		atomic_set(&rcu_torture_wcount[i], 0);
	for_each_possible_cpu(cpu) {
		for (i = 0; i < RCU_TORTURE_PIPE_LEN + 1; i++) {
			per_cpu(rcu_torture_count, cpu)[i] = 0;
			per_cpu(rcu_torture_batch, cpu)[i] = 0;
		}
	}

	/* Start up the kthreads. */

	firsterr = torture_create_kthread(rcu_torture_writer, NULL,
					  writer_task);
	if (firsterr)
		goto unwind;
	fakewriter_tasks = kzalloc(nfakewriters * sizeof(fakewriter_tasks[0]),
				   GFP_KERNEL);
	if (fakewriter_tasks == NULL) {
		VERBOSE_TOROUT_ERRSTRING("out of memory");
		firsterr = -ENOMEM;
		goto unwind;
	}
	for (i = 0; i < nfakewriters; i++) {
		firsterr = torture_create_kthread(rcu_torture_fakewriter,
						  NULL, fakewriter_tasks[i]);
		if (firsterr)
			goto unwind;
	}
	reader_tasks = kzalloc(nrealreaders * sizeof(reader_tasks[0]),
			       GFP_KERNEL);
	if (reader_tasks == NULL) {
		VERBOSE_TOROUT_ERRSTRING("out of memory");
		firsterr = -ENOMEM;
		goto unwind;
	}
	for (i = 0; i < nrealreaders; i++) {
		firsterr = torture_create_kthread(rcu_torture_reader, NULL,
						  reader_tasks[i]);
		if (firsterr)
			goto unwind;
	}
	if (stat_interval > 0) {
		firsterr = torture_create_kthread(rcu_torture_stats, NULL,
						  stats_task);
		if (firsterr)
			goto unwind;
	}
	if (test_no_idle_hz) {
		firsterr = torture_shuffle_init(shuffle_interval * HZ);
		if (firsterr)
			goto unwind;
	}
	if (stutter < 0)
		stutter = 0;
	if (stutter) {
		firsterr = torture_stutter_init(stutter * HZ);
		if (firsterr)
			goto unwind;
	}
	if (fqs_duration < 0)
		fqs_duration = 0;
	if (fqs_duration) {
		/* Create the fqs thread */
		torture_create_kthread(rcu_torture_fqs, NULL, fqs_task);
		if (firsterr)
			goto unwind;
	}
	if (test_boost_interval < 1)
		test_boost_interval = 1;
	if (test_boost_duration < 2)
		test_boost_duration = 2;
	if ((test_boost == 1 && cur_ops->can_boost) ||
	    test_boost == 2) {

		boost_starttime = jiffies + test_boost_interval * HZ;
		register_cpu_notifier(&rcutorture_cpu_nb);
		for_each_possible_cpu(i) {
			if (cpu_is_offline(i))
				continue;  /* Heuristic: CPU can go offline. */
			firsterr = rcutorture_booster_init(i);
			if (firsterr)
				goto unwind;
		}
	}
	firsterr = torture_shutdown_init(shutdown_secs, rcu_torture_cleanup);
	if (firsterr)
		goto unwind;
	firsterr = torture_onoff_init(onoff_holdoff * HZ, onoff_interval * HZ);
	if (firsterr)
		goto unwind;
	firsterr = rcu_torture_stall_init();
	if (firsterr)
		goto unwind;
	firsterr = rcu_torture_barrier_init();
	if (firsterr)
		goto unwind;
	if (object_debug)
		rcu_test_debug_objects();
	rcutorture_record_test_transition();
	torture_init_end();
	return 0;

unwind:
	torture_init_end();
	rcu_torture_cleanup();
	return firsterr;
}

module_init(rcu_torture_init);
module_exit(rcu_torture_cleanup);
