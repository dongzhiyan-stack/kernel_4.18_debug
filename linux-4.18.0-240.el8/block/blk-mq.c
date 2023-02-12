/*
 * Block multiqueue core code
 *
 * Copyright (C) 2013-2014 Jens Axboe
 * Copyright (C) 2013-2014 Christoph Hellwig
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/backing-dev.h>
#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/kmemleak.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/smp.h>
#include <linux/llist.h>
#include <linux/list_sort.h>
#include <linux/cpu.h>
#include <linux/cache.h>
#include <linux/sched/sysctl.h>
#include <linux/sched/topology.h>
#include <linux/sched/signal.h>
#include <linux/delay.h>
#include <linux/crash_dump.h>
#include <linux/prefetch.h>

#include <trace/events/block.h>

#include <linux/blk-mq.h>
#include <linux/t10-pi.h>
#include "blk.h"
#include "blk-mq.h"
#include "blk-mq-debugfs.h"
#include "blk-mq-tag.h"
#include "blk-pm.h"
#include "blk-stat.h"
#include "blk-mq-sched.h"
#include "blk-rq-qos.h"

static enum cpuhp_state blk_mq_online;

static void blk_mq_poll_stats_start(struct request_queue *q);
static void blk_mq_poll_stats_fn(struct blk_stat_callback *cb);

static int blk_mq_poll_stats_bkt(const struct request *rq)
{
	int ddir, sectors, bucket;

	ddir = rq_data_dir(rq);
	sectors = blk_rq_stats_sectors(rq);

	bucket = ddir + 2 * ilog2(sectors);

	if (bucket < 0)
		return -1;
	else if (bucket >= BLK_MQ_POLL_STATS_BKTS)
		return ddir + BLK_MQ_POLL_STATS_BKTS - 2;

	return bucket;
}

/*
 * Check if any of the ctx, dispatch list or elevator
 * have pending work in this hardware queue.
 */
static bool blk_mq_hctx_has_pending(struct blk_mq_hw_ctx *hctx)
{
	return !list_empty_careful(&hctx->dispatch) ||
		sbitmap_any_bit_set(&hctx->ctx_map) ||
			blk_mq_sched_has_work(hctx);
}

/*
 * Mark this ctx as having pending work in this hardware queue
 */
static void blk_mq_hctx_mark_pending(struct blk_mq_hw_ctx *hctx,
				     struct blk_mq_ctx *ctx)
{
	const int bit = ctx->index_hw[hctx->type];

	if (!sbitmap_test_bit(&hctx->ctx_map, bit))
		sbitmap_set_bit(&hctx->ctx_map, bit);
}

static void blk_mq_hctx_clear_pending(struct blk_mq_hw_ctx *hctx,
				      struct blk_mq_ctx *ctx)
{
	const int bit = ctx->index_hw[hctx->type];

	sbitmap_clear_bit(&hctx->ctx_map, bit);
}

struct mq_inflight {
	struct hd_struct *part;
	unsigned int inflight[2];
};

static bool blk_mq_check_inflight(struct blk_mq_hw_ctx *hctx,
				  struct request *rq, void *priv,
				  bool reserved)
{
	struct mq_inflight *mi = priv;

	if (rq->part == mi->part)
		mi->inflight[rq_data_dir(rq)]++;

	return true;
}

unsigned int blk_mq_in_flight(struct request_queue *q, struct hd_struct *part)
{
	struct mq_inflight mi = { .part = part };

	blk_mq_queue_tag_busy_iter(q, blk_mq_check_inflight, &mi);

	return mi.inflight[0] + mi.inflight[1];
}

void blk_mq_in_flight_rw(struct request_queue *q, struct hd_struct *part,
			 unsigned int inflight[2])
{
	struct mq_inflight mi = { .part = part };

	blk_mq_queue_tag_busy_iter(q, blk_mq_check_inflight, &mi);
	inflight[0] = mi.inflight[0];
	inflight[1] = mi.inflight[1];
}

void blk_freeze_queue_start(struct request_queue *q)
{
	mutex_lock(&q->mq_freeze_lock);
	if (++q->mq_freeze_depth == 1) {
		percpu_ref_kill(&q->q_usage_counter);
		mutex_unlock(&q->mq_freeze_lock);
		if (queue_is_mq(q))
			blk_mq_run_hw_queues(q, false);
	} else {
		mutex_unlock(&q->mq_freeze_lock);
	}
}
EXPORT_SYMBOL_GPL(blk_freeze_queue_start);

void blk_mq_freeze_queue_wait(struct request_queue *q)
{
	wait_event(q->mq_freeze_wq, percpu_ref_is_zero(&q->q_usage_counter));
}
EXPORT_SYMBOL_GPL(blk_mq_freeze_queue_wait);

int blk_mq_freeze_queue_wait_timeout(struct request_queue *q,
				     unsigned long timeout)
{
	return wait_event_timeout(q->mq_freeze_wq,
					percpu_ref_is_zero(&q->q_usage_counter),
					timeout);
}
EXPORT_SYMBOL_GPL(blk_mq_freeze_queue_wait_timeout);

/*
 * Guarantee no request is in use, so we can change any data structure of
 * the queue afterward.
 */
void blk_freeze_queue(struct request_queue *q)
{
	/*
	 * In the !blk_mq case we are only calling this to kill the
	 * q_usage_counter, otherwise this increases the freeze depth
	 * and waits for it to return to zero.  For this reason there is
	 * no blk_unfreeze_queue(), and blk_freeze_queue() is not
	 * exported to drivers as the only user for unfreeze is blk_mq.
	 */
	blk_freeze_queue_start(q);
	blk_mq_freeze_queue_wait(q);
}

void blk_mq_freeze_queue(struct request_queue *q)
{
	/*
	 * ...just an alias to keep freeze and unfreeze actions balanced
	 * in the blk_mq_* namespace
	 */
	blk_freeze_queue(q);
}
EXPORT_SYMBOL_GPL(blk_mq_freeze_queue);

void blk_mq_unfreeze_queue(struct request_queue *q)
{
	mutex_lock(&q->mq_freeze_lock);
	q->mq_freeze_depth--;
	WARN_ON_ONCE(q->mq_freeze_depth < 0);
	if (!q->mq_freeze_depth) {
		percpu_ref_resurrect(&q->q_usage_counter);
		wake_up_all(&q->mq_freeze_wq);
	}
	mutex_unlock(&q->mq_freeze_lock);
}
EXPORT_SYMBOL_GPL(blk_mq_unfreeze_queue);

/*
 * FIXME: replace the scsi_internal_device_*block_nowait() calls in the
 * mpt3sas driver such that this function can be removed.
 */
void blk_mq_quiesce_queue_nowait(struct request_queue *q)
{
	blk_queue_flag_set(QUEUE_FLAG_QUIESCED, q);
}
EXPORT_SYMBOL_GPL(blk_mq_quiesce_queue_nowait);

/**
 * blk_mq_quiesce_queue() - wait until all ongoing dispatches have finished
 * @q: request queue.
 *
 * Note: this function does not prevent that the struct request end_io()
 * callback function is invoked. Once this function is returned, we make
 * sure no dispatch can happen until the queue is unquiesced via
 * blk_mq_unquiesce_queue().
 */
void blk_mq_quiesce_queue(struct request_queue *q)
{
	struct blk_mq_hw_ctx *hctx;
	unsigned int i;
	bool rcu = false;

	blk_mq_quiesce_queue_nowait(q);

	queue_for_each_hw_ctx(q, hctx, i) {
		if (hctx->flags & BLK_MQ_F_BLOCKING)
			synchronize_srcu(hctx->srcu);
		else
			rcu = true;
	}
	if (rcu)
		synchronize_rcu();
}
EXPORT_SYMBOL_GPL(blk_mq_quiesce_queue);

/*
 * blk_mq_unquiesce_queue() - counterpart of blk_mq_quiesce_queue()
 * @q: request queue.
 *
 * This function recovers queue into the state before quiescing
 * which is done by blk_mq_quiesce_queue.
 */
void blk_mq_unquiesce_queue(struct request_queue *q)
{
	blk_queue_flag_clear(QUEUE_FLAG_QUIESCED, q);

	/* dispatch requests which are inserted during quiescing */
	blk_mq_run_hw_queues(q, true);
}
EXPORT_SYMBOL_GPL(blk_mq_unquiesce_queue);

void blk_mq_wake_waiters(struct request_queue *q)
{
	struct blk_mq_hw_ctx *hctx;
	unsigned int i;

	queue_for_each_hw_ctx(q, hctx, i)
		if (blk_mq_hw_queue_mapped(hctx))
			blk_mq_tag_wakeup_all(hctx->tags, true);
}

/*
 * Only need start/end time stamping if we have iostat or
 * blk stats enabled, or using an IO scheduler.
 */
static inline bool blk_mq_need_time_stamp(struct request *rq)
{
	return (rq->rq_flags & (RQF_IO_STAT | RQF_STATS)) || rq->q->elevator;
}

static struct request *blk_mq_rq_ctx_init(struct blk_mq_alloc_data *data,
		unsigned int tag, u64 alloc_time_ns)
{
	struct blk_mq_tags *tags = blk_mq_tags_from_data(data);
	struct request *rq = tags->static_rqs[tag];
	req_flags_t rq_flags = 0;

	if (data->flags & BLK_MQ_REQ_INTERNAL) {
		rq->tag = BLK_MQ_NO_TAG;
		rq->internal_tag = tag;
	} else {
		if (data->hctx->flags & BLK_MQ_F_TAG_SHARED) {
			rq_flags = RQF_MQ_INFLIGHT;
			atomic_inc(&data->hctx->nr_active);
		}
		rq->tag = tag;
		rq->internal_tag = BLK_MQ_NO_TAG;
		data->hctx->tags->rqs[rq->tag] = rq;
	}

	/* csd/requeue_work/fifo_time is initialized before use */
	rq->q = data->q;
	rq->mq_ctx = data->ctx;
	rq->mq_hctx = data->hctx;
	rq->rq_flags = rq_flags;
	rq->cmd_flags = data->cmd_flags;
	if (data->flags & BLK_MQ_REQ_PREEMPT)
		rq->rq_flags |= RQF_PREEMPT;
	if (blk_queue_io_stat(data->q))
		rq->rq_flags |= RQF_IO_STAT;
	INIT_LIST_HEAD(&rq->queuelist);
	INIT_HLIST_NODE(&rq->hash);
	RB_CLEAR_NODE(&rq->rb_node);
	rq->rq_disk = NULL;
	rq->part = NULL;
#ifdef CONFIG_BLK_RQ_ALLOC_TIME
	rq_aux(rq)->alloc_time_ns = alloc_time_ns;
#endif
	if (blk_mq_need_time_stamp(rq))
		rq->start_time_ns = ktime_get_ns();
	else
		rq->start_time_ns = 0;
	rq->io_start_time_ns = 0;
	rq_aux(rq)->stats_sectors = 0;
	rq->nr_phys_segments = 0;
#if defined(CONFIG_BLK_DEV_INTEGRITY)
	rq->nr_integrity_segments = 0;
#endif
	/* tag was already set */
	rq->extra_len = 0;
	WRITE_ONCE(rq->deadline, 0);

	rq->timeout = 0;

	rq->end_io = NULL;
	rq->end_io_data = NULL;

	data->ctx->rq_dispatched[op_is_sync(data->cmd_flags)]++;
	refcount_set(&rq->ref, 1);

	if (!op_is_flush(data->cmd_flags)) {
		struct elevator_queue *e = data->q->elevator;

		rq->elv.icq = NULL;
		if (e && e->type->ops.prepare_request) {
			if (e->type->icq_cache)
				blk_mq_sched_assign_ioc(rq);

			e->type->ops.prepare_request(rq, data->bio);
			rq->rq_flags |= RQF_ELVPRIV;
		}
	}

	data->hctx->queued++;
	return rq;
}

static struct request *__blk_mq_alloc_request(struct blk_mq_alloc_data *data)
{
	struct request_queue *q = data->q;
	struct elevator_queue *e = q->elevator;
	u64 alloc_time_ns = 0;
	unsigned int tag;

	/* alloc_time includes depth and tag waits */
	if (blk_queue_rq_alloc_time(q))
		alloc_time_ns = ktime_get_ns();

	if (data->cmd_flags & REQ_NOWAIT)
		data->flags |= BLK_MQ_REQ_NOWAIT;

	if (e) {
		data->flags |= BLK_MQ_REQ_INTERNAL;

		/*
		 * Flush requests are special and go directly to the
		 * dispatch list. Don't include reserved tags in the
		 * limiting, as it isn't useful.
		 */
		if (!op_is_flush(data->cmd_flags) &&
		    e->type->ops.limit_depth &&
		    !(data->flags & BLK_MQ_REQ_RESERVED))
			e->type->ops.limit_depth(data->cmd_flags, data);
	}

retry:
	data->ctx = blk_mq_get_ctx(q);
	data->hctx = blk_mq_map_queue(q, data->cmd_flags, data->ctx);
	if (!(data->flags & BLK_MQ_REQ_INTERNAL))
		blk_mq_tag_busy(data->hctx);

	/*
	 * Waiting allocations only fail because of an inactive hctx.  In that
	 * case just retry the hctx assignment and tag allocation as CPU hotplug
	 * should have migrated us to an online CPU by now.
	 */
	tag = blk_mq_get_tag(data);
	if (tag == BLK_MQ_NO_TAG) {
		if (data->flags & BLK_MQ_REQ_NOWAIT)
			return NULL;

		/*
		 * Give up the CPU and sleep for a random short time to ensure
		 * that thread using a realtime scheduling class are migrated
		 * off the the CPU, and thus off the hctx that is going away.
		 */
		msleep(3);
		goto retry;
	}
	return blk_mq_rq_ctx_init(data, tag, alloc_time_ns);
}

struct request *blk_mq_alloc_request(struct request_queue *q, unsigned int op,
		blk_mq_req_flags_t flags)
{
	struct blk_mq_alloc_data data = {
		.q		= q,
		.flags		= flags,
		.cmd_flags	= op,
	};
	struct request *rq;
	int ret;

	ret = blk_queue_enter(q, flags);
	if (ret)
		return ERR_PTR(ret);

	rq = __blk_mq_alloc_request(&data);
	if (!rq)
		goto out_queue_exit;
	rq->__data_len = 0;
	rq->__sector = (sector_t) -1;
	rq->bio = rq->biotail = NULL;
	return rq;
out_queue_exit:
	blk_queue_exit(q);
	return ERR_PTR(-EWOULDBLOCK);
}
EXPORT_SYMBOL(blk_mq_alloc_request);

struct request *blk_mq_alloc_request_hctx(struct request_queue *q,
	unsigned int op, blk_mq_req_flags_t flags, unsigned int hctx_idx)
{
	struct blk_mq_alloc_data data = {
		.q		= q,
		.flags		= flags,
		.cmd_flags	= op,
	};
	u64 alloc_time_ns = 0;
	unsigned int cpu;
	unsigned int tag;
	int ret;

	/* alloc_time includes depth and tag waits */
	if (blk_queue_rq_alloc_time(q))
		alloc_time_ns = ktime_get_ns();

	/*
	 * If the tag allocator sleeps we could get an allocation for a
	 * different hardware context.  No need to complicate the low level
	 * allocator for this for the rare use case of a command tied to
	 * a specific queue.
	 */
	if (WARN_ON_ONCE(!(flags & (BLK_MQ_REQ_NOWAIT | BLK_MQ_REQ_RESERVED))))
		return ERR_PTR(-EINVAL);

	if (hctx_idx >= q->nr_hw_queues)
		return ERR_PTR(-EIO);

	ret = blk_queue_enter(q, flags);
	if (ret)
		return ERR_PTR(ret);

	/*
	 * Check if the hardware context is actually mapped to anything.
	 * If not tell the caller that it should skip this queue.
	 */
	ret = -EXDEV;
	data.hctx = q->queue_hw_ctx[hctx_idx];
	if (!blk_mq_hw_queue_mapped(data.hctx))
		goto out_queue_exit;
	cpu = cpumask_first_and(data.hctx->cpumask, cpu_online_mask);
	data.ctx = __blk_mq_get_ctx(q, cpu);

	if (q->elevator)
		data.flags |= BLK_MQ_REQ_INTERNAL;
	else
		blk_mq_tag_busy(data.hctx);

	ret = -EWOULDBLOCK;
	tag = blk_mq_get_tag(&data);
	if (tag == BLK_MQ_NO_TAG)
		goto out_queue_exit;
	return blk_mq_rq_ctx_init(&data, tag, alloc_time_ns);

out_queue_exit:
	blk_queue_exit(q);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(blk_mq_alloc_request_hctx);

static void __blk_mq_free_request(struct request *rq)
{
	struct request_queue *q = rq->q;
	struct blk_mq_ctx *ctx = rq->mq_ctx;
	struct blk_mq_hw_ctx *hctx = rq->mq_hctx;
	const int sched_tag = rq->internal_tag;

	blk_pm_mark_last_busy(rq);
	rq->mq_hctx = NULL;
	if (rq->tag != BLK_MQ_NO_TAG)
		blk_mq_put_tag(hctx->tags, ctx, rq->tag);
	if (sched_tag != BLK_MQ_NO_TAG)
		blk_mq_put_tag(hctx->sched_tags, ctx, sched_tag);
	blk_mq_sched_restart(hctx);
	blk_queue_exit(q);
}

void blk_mq_free_request(struct request *rq)
{
	struct request_queue *q = rq->q;
	struct elevator_queue *e = q->elevator;
	struct blk_mq_ctx *ctx = rq->mq_ctx;
	struct blk_mq_hw_ctx *hctx = rq->mq_hctx;

	if (rq->rq_flags & RQF_ELVPRIV) {
		if (e && e->type->ops.finish_request)
			e->type->ops.finish_request(rq);
		if (rq->elv.icq) {
			put_io_context(rq->elv.icq->ioc);
			rq->elv.icq = NULL;
		}
	}

	ctx->rq_completed[rq_is_sync(rq)]++;
	if (rq->rq_flags & RQF_MQ_INFLIGHT)
		atomic_dec(&hctx->nr_active);

	if (unlikely(laptop_mode && !blk_rq_is_passthrough(rq)))
		laptop_io_completion(q->backing_dev_info);

	rq_qos_done(q, rq);

	WRITE_ONCE(rq->state, MQ_RQ_IDLE);
	if (refcount_dec_and_test(&rq->ref))
		__blk_mq_free_request(rq);
}
EXPORT_SYMBOL_GPL(blk_mq_free_request);

inline void __blk_mq_end_request(struct request *rq, blk_status_t error)
{
	u64 now = 0;

	if (blk_mq_need_time_stamp(rq))
		now = ktime_get_ns();

	if (rq->rq_flags & RQF_STATS) {
		blk_mq_poll_stats_start(rq->q);
		blk_stat_add(rq, now);
	}

	if (rq->internal_tag != BLK_MQ_NO_TAG)
		blk_mq_sched_completed_request(rq, now);

	blk_account_io_done(rq, now);

	if (rq->end_io) {
		rq_qos_done(rq->q, rq);
		rq->end_io(rq, error);
	} else {
		blk_mq_free_request(rq);
	}
}
EXPORT_SYMBOL(__blk_mq_end_request);

void blk_mq_end_request(struct request *rq, blk_status_t error)
{
	if (blk_update_request(rq, error, blk_rq_bytes(rq)))
		BUG();
	__blk_mq_end_request(rq, error);
}
EXPORT_SYMBOL(blk_mq_end_request);

static void __blk_mq_complete_request_remote(void *data)
{
	struct request *rq = data;
	struct request_queue *q = rq->q;

	q->mq_ops->complete(rq);
}

/**
 * blk_mq_force_complete_rq() - Force complete the request, bypassing any error
 * 				injection that could drop the completion.
 * @rq: Request to be force completed
 *
 * Drivers should use blk_mq_complete_request() to complete requests in their
 * normal IO path. For timeout error recovery, drivers may call this forced
 * completion routine after they've reclaimed timed out requests to bypass
 * potentially subsequent fake timeouts.
 */
void blk_mq_force_complete_rq(struct request *rq)
{
	struct blk_mq_ctx *ctx = rq->mq_ctx;
	struct request_queue *q = rq->q;
	bool shared = false;
	int cpu;

	WRITE_ONCE(rq->state, MQ_RQ_COMPLETE);
	/*
	 * Most of single queue controllers, there is only one irq vector
	 * for handling IO completion, and the only irq's affinity is set
	 * as all possible CPUs. On most of ARCHs, this affinity means the
	 * irq is handled on one specific CPU.
	 *
	 * So complete IO reqeust in softirq context in case of single queue
	 * for not degrading IO performance by irqsoff latency.
	 */
	if (q->nr_hw_queues == 1) {
		__blk_complete_request(rq);
		return;
	}

	/*
	 * For a polled request, always complete locallly, it's pointless
	 * to redirect the completion.
	 */
	if ((rq->cmd_flags & REQ_HIPRI) ||
	    !test_bit(QUEUE_FLAG_SAME_COMP, &q->queue_flags)) {
		q->mq_ops->complete(rq);
		return;
	}

	cpu = get_cpu();
	if (!test_bit(QUEUE_FLAG_SAME_FORCE, &q->queue_flags))
		shared = cpus_share_cache(cpu, ctx->cpu);

	if (cpu != ctx->cpu && !shared && cpu_online(ctx->cpu)) {
		rq->csd.func = __blk_mq_complete_request_remote;
		rq->csd.info = rq;
		rq->csd.flags = 0;
		smp_call_function_single_async(ctx->cpu, &rq->csd);
	} else {
		q->mq_ops->complete(rq);
	}
	put_cpu();
}
EXPORT_SYMBOL_GPL(blk_mq_force_complete_rq);

static void hctx_unlock(struct blk_mq_hw_ctx *hctx, int srcu_idx)
	__releases(hctx->srcu)
{
	if (!(hctx->flags & BLK_MQ_F_BLOCKING))
		rcu_read_unlock();
	else
		srcu_read_unlock(hctx->srcu, srcu_idx);
}

static void hctx_lock(struct blk_mq_hw_ctx *hctx, int *srcu_idx)
	__acquires(hctx->srcu)
{
	if (!(hctx->flags & BLK_MQ_F_BLOCKING)) {
		/* shut up gcc false positive */
		*srcu_idx = 0;
		rcu_read_lock();
	} else
		*srcu_idx = srcu_read_lock(hctx->srcu);
}

/**
 * blk_mq_complete_request - end I/O on a request
 * @rq:		the request being processed
 *
 * Description:
 *	Ends all I/O on a request. It does not handle partial completions.
 *	The actual completion happens out-of-order, through a IPI handler.
 **/
bool blk_mq_complete_request(struct request *rq)
{
	if (unlikely(blk_should_fake_timeout(rq->q)))
		return false;
	blk_mq_force_complete_rq(rq);
	return true;
}
EXPORT_SYMBOL(blk_mq_complete_request);

void blk_mq_start_request(struct request *rq)
{
	struct request_queue *q = rq->q;

	blk_mq_sched_started_request(rq);

	trace_block_rq_issue(q, rq);

	if (test_bit(QUEUE_FLAG_STATS, &q->queue_flags)) {
		rq->io_start_time_ns = ktime_get_ns();
		rq_aux(rq)->stats_sectors = blk_rq_sectors(rq);
		rq->rq_flags |= RQF_STATS;
		rq_qos_issue(q, rq);
	}

	WARN_ON_ONCE(blk_mq_rq_state(rq) != MQ_RQ_IDLE);

	blk_add_timer(rq);
	WRITE_ONCE(rq->state, MQ_RQ_IN_FLIGHT);

	if (q->dma_drain_size && blk_rq_bytes(rq)) {
		/*
		 * Make sure space for the drain appears.  We know we can do
		 * this because max_hw_segments has been adjusted to be one
		 * fewer than the device can handle.
		 */
		rq->nr_phys_segments++;
	}

#ifdef CONFIG_BLK_DEV_INTEGRITY
	if (blk_integrity_rq(rq) && req_op(rq) == REQ_OP_WRITE)
		q->integrity.profile->ext_ops->prepare_fn(rq);
#endif
	//一个req可能多次执行blk_mq_start_request()派发IO请求，因此派发时可能遇到驱动繁忙，这样就会再次派发该IO
	/******process_rq_stat***************/
        if(rq->rq_disk && rq->rq_disk->process_io.enable && rq->p_process_rq_stat){
            struct process_rq_stat *p_process_rq_stat_tmp = rq->p_process_rq_stat;
	    struct process_io_info *p_process_io_info_tmp = rq->p_process_rq_stat->p_process_io_info;
            //在IO请求插入IO队列，分配process_rq_stat后并清0。而第1次执行到这里派发这个IO请求，p_process_rq_stat_tmp->rq_issue_time 肯定是0
	    if(p_process_rq_stat_tmp->rq_issue_time == 0){
	        atomic_dec(&(rq->rq_disk->process_io.rq_in_queue));
	        atomic_inc(&(rq->rq_disk->process_io.rq_in_driver));
	        p_process_rq_stat_tmp->rq_issue_time = ktime_to_us(ktime_get());
	        p_process_rq_stat_tmp->id_time = p_process_rq_stat_tmp->rq_issue_time - p_process_rq_stat_tmp->rq_inset_time;
	    }else{
	        printk(KERN_DEBUG"%s rq:0x%llx repeat dispatch\n",__func__,(u64)rq);
	    }
            //printk(KERN_DEBUG"%s %s %d rq:0x%llx process_rq_stat:0x%llx rq_issue_time:%lld p_process_io_info_tmp:0x%llx pid:%d  rq_real_issue_time:%lld\n",__func__,current->comm,current->pid,(u64)rq,(u64)(rq->p_process_rq_stat),p_process_rq_stat_tmp->rq_issue_time,(u64)p_process_io_info_tmp,p_process_io_info_tmp->pid,p_process_rq_stat_tmp->rq_real_issue_time);

	    /*p_process_rq_stat_tmp->rq_issue_time = ktime_to_us(ktime_get());
	    p_process_rq_stat_tmp->id_time = p_process_rq_stat_tmp->rq_issue_time - p_process_rq_stat_tmp->rq_inset_time;*/

	    //赋值该req传输的字节数，不能再IO插入队列时更新，必须在IO派发时更新。因为IO插入队列后，可能会合并到其他req，合并到的req的IO字节数会增加。在派发派发时，都是最新的req字节数，不会是被合并的req
            p_process_rq_stat_tmp->req_size = blk_rq_bytes(rq);            
	    //spin_lock_irq(&(rq->rq_disk->process_io.process_lock));
	    /*spin_lock_irq(&(rq->rq_disk->process_io.io_data_lock));
	    //累加进程的IO请求在队列的时间,需加锁保护，因为同时可能执行print_process_io_info函数清0
	    //p_process_io_info_tmp->all_id_time += p_process_rq_stat_tmp->id_time;

	    //在IO队列IO请求数减1，需加锁保护，因为同时进程可能在向IO队列插入其他IO请求而 rq->rq_disk->process_io.rq_in_queue++
	    rq->rq_disk->process_io.rq_in_queue --;
	    //在磁盘驱动IO请求数加1，加锁同理
	    rq->rq_disk->process_io.rq_in_driver ++;
	    //spin_unlock_irq(&(rq->rq_disk->process_io.process_lock));
	    spin_unlock_irq(&(rq->rq_disk->process_io.io_data_lock));
            */

	    if(atomic_read(&(rq->rq_disk->process_io.rq_in_queue)) < 0){
	        printk(KERN_DEBUG"%s rq_in_queue:%d!!!!!!\n",__func__,atomic_read(&(rq->rq_disk->process_io.rq_in_queue)));
	    }
	    if(p_process_rq_stat_tmp->id_time > p_process_io_info_tmp->max_id_time){//记录最大的id time
	        //p_process_io_info_tmp->max_id_time = p_process_rq_stat_tmp->id_time;
		//??????没必要再记录req指针了，因为req释放后会立即被新的进程使用，这样req指针就不能代表某个进程了
		//p_process_io_info_tmp->max_id_time_rq = rq;

		//记录这个IO请求ID更大时的 在IO队列的IO请求数 和 在磁盘驱动层的IO请求数 到IO请求自己的 process_rq_stat成员rq_inflght_issue_tmp里。在IO传输传输完成时再赋值给process_io_info的rq_inflght_issue。最初设计是保存z到p_process_io_info_tmp->rq_inflght_issue_tmp里。但是会有问题，因为，后续的IO请求也会因id时间很大而更新rq_inflght_issue_tmp，这样保存rq_inflght_issue_tmp的IO请求数就不是前一个IO请求的了
		//rq_inflght_issue_tmp临时保存,blk_account_io_done中，在spin lock加锁代码里，再把rq_inflght_issue_tmp的值保存到rq_inflght_issue.为什么要这样？因为不这样做，print_process_io_info()对rq_inflght_issue会清0，就会丢失正在传输的IO时的rq_in_queue 和 rq_in_driver数据
		p_process_rq_stat_tmp->rq_inflght_issue_tmp[0] =  atomic_read(&(rq->rq_disk->process_io.rq_in_queue));
		p_process_rq_stat_tmp->rq_inflght_issue_tmp[1] =  atomic_read(&(rq->rq_disk->process_io.rq_in_driver));
	    }
	}
	if(rq->q->high_io_prio_enable){
	    atomic_inc(&(rq->q->rq_in_diver_count));
        }
}
EXPORT_SYMBOL(blk_mq_start_request);

static void __blk_mq_requeue_request(struct request *rq)
{
	struct request_queue *q = rq->q;

	blk_mq_put_driver_tag(rq);

	trace_block_rq_requeue(q, rq);
	rq_qos_requeue(q, rq);

	if (blk_mq_request_started(rq)) {
		WRITE_ONCE(rq->state, MQ_RQ_IDLE);
		rq->rq_flags &= ~RQF_TIMED_OUT;
		if (q->dma_drain_size && blk_rq_bytes(rq))
			rq->nr_phys_segments--;
	}
}
/******process_rq_stat***************/
void free_all_process_io_info(struct process_io_control *p_process_io_tmp)
{
        struct process_io_info *p_process_io_info_tmp = NULL;
	struct process_io_info *p_process_io_info_del = NULL;
        //spin_lock_irq(&(p_process_io_tmp->process_lock));
	list_for_each_entry_safe(p_process_io_info_tmp,p_process_io_info_del,&(p_process_io_tmp->process_io_control_head), process_io_info_list){
	    list_del(&p_process_io_info_tmp->process_io_info_list);
            kmem_cache_free(p_process_io_tmp->process_io_info_cachep, p_process_io_info_tmp);
	}
	//spin_unlock_irq(&(p_process_io_tmp->process_lock));
	
        while(1)//加锁while循环是一旦 p_process_io_tmp->read_lock_count 不是0，这里做成死循环，正常情况 p_process_io_tmp->read_lock_count肯定会是0
	{
	    //当 read_lock_count 是0，说明process_io_control_head_del上的process_io_info在从process_io_control_head链表剔除后，遍历 process_io_control_head链表的进程都退出了宽限期，可以放心释放了
	    if(atomic_read(&(p_process_io_tmp->read_lock_count)) == 0){
		list_for_each_entry_safe(p_process_io_info_tmp,p_process_io_info_del,&(p_process_io_tmp->process_io_control_head_del), process_io_info_del){
		    list_del(&p_process_io_info_tmp->process_io_info_del);
		    kmem_cache_free(p_process_io_tmp->process_io_info_cachep,p_process_io_info_tmp);
		}
		break;
	    }
	    msleep(10);
        }
        
	kmem_cache_destroy(p_process_io_tmp->process_io_info_cachep);
        kmem_cache_destroy(p_process_io_tmp->process_rq_stat_cachep);
	printk("free_all_process_io_info ok\n");
}
EXPORT_SYMBOL(free_all_process_io_info);
void print_process_io_info(struct process_io_control *p_process_io_tmp)
{
	struct process_io_info *p_process_io_info_tmp = NULL;
	struct process_io_info *p_process_io_info_tmp_copy = NULL;
	//struct process_io_info *p_process_io_info_del = NULL;
        int i = 0;
	struct process_rq_stat *p_process_rq_stat_tmp,*p_process_rq_stat_n;

	list_for_each_entry_safe(p_process_rq_stat_tmp,p_process_rq_stat_n, &(p_process_io_tmp->process_io_insert_head),process_io_insert){
	    smp_rmb();
	    //printk("print_process_io_info p_process_io_tmp:0x%llx p_process_rq_stat_tmp:0x%llx\n",(u64)p_process_io_tmp,(u64)p_process_rq_stat_tmp);
	    if(p_process_rq_stat_tmp->has_delete){
		//不能通过req得到process_io_insert_lock锁，因为req此时可能已经释放了，是个无效指针
                //spin_lock_irq(&(p_process_rq_stat_tmp->rq->rq_disk->process_io.process_io_insert_lock));
                spin_lock_irq(&(p_process_io_tmp->process_io_insert_lock));
		list_del(&p_process_rq_stat_tmp->process_io_insert);
                //list_del(&req->queuelist_insert);
                //spin_unlock_irq(&(p_process_rq_stat_tmp->rq->rq_disk->process_io.process_io_insert_lock));
                spin_unlock_irq(&(p_process_io_tmp->process_io_insert_lock));
	        kmem_cache_free(p_process_io_tmp->process_rq_stat_cachep, p_process_rq_stat_tmp);
	    }else if(p_process_rq_stat_tmp->rq_inset_time !=0){
	       //一个IO 30s还没有派发，那可能就出问题了
	       if(ktime_to_us(ktime_get()) - p_process_rq_stat_tmp->rq_inset_time > 30000000){
	           printk("rq:0x%llx long time do not dispatch\n",(u64)(p_process_rq_stat_tmp->rq));
	       }
	    }
	}

        //spin_lock_irq(&(p_process_io_tmp->process_lock));
        atomic_inc(&(p_process_io_tmp->read_lock_count));//类似 rcu_read_lock()开始宽限期Grace Period
	//list_for_each_entry(p_process_io_info_tmp, &(p_process_io_tmp->process_io_control_head), process_io_info_list){
	list_for_each_entry_rcu(p_process_io_info_tmp, &(p_process_io_tmp->process_io_control_head), process_io_info_list){
		if(p_process_io_info_tmp->complete_rq_count != 0)//这1s时间内，p_process_io_info_tmp代表的进程必须传输完成一个IO才会统计打印
		{
		    int complete_rq_count;
		    u32 max_id_time,max_dc_time,max_idc_time,rq_inflght_issue_queue,rq_inflght_issue_driver,rq_inflght_done_queue,rq_inflght_done_driver,avg_id_time,avg_dc_time,avg_idc_time,io_size,max_real_dc_time,max_hctx_list_rq_count;

		    //获取 p_process_io_info_tmp 下边这些参数，最后清0，需要spin lock操作.与blk_account_io_done()对这些参数的赋值形成互斥，保证这里的清0不影响blk_account_io_done()对这些参数的赋值
	            //spin_lock_irq(&(p_process_io_tmp->io_data_lock));
	            spin_lock_irq(&(p_process_io_info_tmp->io_data_lock));
		    complete_rq_count = p_process_io_info_tmp->complete_rq_count;
		    max_id_time = p_process_io_info_tmp->max_id_time;
		    max_dc_time = p_process_io_info_tmp->max_dc_time;
		    max_idc_time = p_process_io_info_tmp->max_idc_time;
		    rq_inflght_issue_queue = p_process_io_info_tmp->rq_inflght_issue[0];
		    rq_inflght_issue_driver = p_process_io_info_tmp->rq_inflght_issue[1];
		    rq_inflght_done_queue = p_process_io_info_tmp->rq_inflght_done[0];
		    rq_inflght_done_driver = p_process_io_info_tmp->rq_inflght_done[1];
		    avg_id_time = p_process_io_info_tmp->all_id_time/p_process_io_info_tmp->complete_rq_count;
		    avg_dc_time = p_process_io_info_tmp->all_dc_time/p_process_io_info_tmp->complete_rq_count;
		    avg_idc_time = p_process_io_info_tmp->all_idc_time/p_process_io_info_tmp->complete_rq_count;
		    io_size = (p_process_io_info_tmp->io_size)>>20;//除以1024*1024转成单位M

		    max_hctx_list_rq_count = p_process_io_info_tmp->max_hctx_list_rq_count;
		    max_real_dc_time = p_process_io_info_tmp->max_real_dc_time;

		    //对p_process_io_info_tmp对应的进程传输完成的IO请求数清0
		    p_process_io_info_tmp->complete_rq_count = 0;
		    //对max_id_time等清0。但是有一点需要考虑，如果进程正在传输一个IO请求，比如id time很大并赋值给max_id_time，然后派发给驱动，这里对max_id_time等清0。如果这个IO请求的id time很大，那就丢失这个数据了。
		    //好的做法是，判断此时正好有一个IO请求在传输，id time很大，那就不对max_id_time清0
		    p_process_io_info_tmp->max_id_time = 0;
		    p_process_io_info_tmp->max_dc_time = 0;
		    p_process_io_info_tmp->max_idc_time = 0;

		    //对all_id_time等清0。但是有一点需要考虑，这里清0，不就丢失了正在IO传输的那个IO请求的耗时数据???????????
		    p_process_io_info_tmp->all_id_time = 0;
		    p_process_io_info_tmp->all_dc_time = 0;
		    p_process_io_info_tmp->all_idc_time = 0;

		    p_process_io_info_tmp->io_size = 0;
		    p_process_io_info_tmp->max_hctx_list_rq_count = 0;
		    p_process_io_info_tmp->max_real_dc_time = 0;

		    //spin_unlock_irq(&(p_process_io_tmp->io_data_lock)); 
		    spin_unlock_irq(&(p_process_io_info_tmp->io_data_lock));
	            
		    printk("%s %d rq_count:%d io_size:%dM max_id_time:%dus max_dc_time:%dus max_idc_time:%dus max_real_dc_time:%dus max_hctx_list_rq:%d rq_inflght_issue:%d_%d rq_inflght_done:%d_%d  avg_id_time:%dus avg_dc_time:%dus avg_idc_time:%dus\n",p_process_io_info_tmp->comm,p_process_io_info_tmp->pid,complete_rq_count,io_size,max_id_time,max_dc_time,max_idc_time,max_real_dc_time,max_hctx_list_rq_count,rq_inflght_issue_queue,rq_inflght_issue_driver,rq_inflght_done_queue,rq_inflght_done_driver,avg_id_time,avg_dc_time,avg_idc_time);
		
		    if(p_process_io_info_tmp->rq_empty_count != 0)
		    {
		        p_process_io_info_tmp->rq_empty_count = 0;//来了IO请求则对rq_empty_count清0
		    }
                }
		else//这1s时间1个IO都没传输，连续5次说明进程IO空闲，就考虑剔除进程对应的process_io_info
		{
			p_process_io_info_tmp->rq_empty_count ++;
			//释放没有IO请求进程的process_io_info 结构。有没有可能在释放 process_io_info时，此时对应进程来了新的IO请求呢，就使用无效 process_io_info了。不会，由 spin_lock_irq加锁保护
			if(p_process_io_info_tmp->rq_empty_count == 5)
			{
			     //????????????这里不能delete p_process_io_info_tmp,因为下个for循环，要依赖 p_process_io_info_tmp.next找到下一个链表成员，p_process_io_info_tmp删除后p_process_io_info_tmp.next就可能是个非法值.看下 list_for_each_entry源码就清楚了
		             //list_del(&p_process_io_info_tmp->process_io_info_list);
		             //kmem_cache_free(p_process_io_tmp->process_io_info_cachep, p_process_io_info_tmp);
			     //p_process_io_info_del = p_process_io_info_tmp;
			     
			     //blk_mq_sched_request_inserted()中会同时向process_io_control_head链表插入process_io_info，这里又要剔除process_io_info，同时多个writer，因此要加锁
	                     //先从process_io_control_head链表剔除当前的process_io_info但不立即释放
	                     spin_lock_irq(&(p_process_io_tmp->process_lock_list));
			     //如果p_process_io_info_tmp->rq_count是0才能真正从链表剔除p_process_io_info_tmp。因为可能blk_mq_sched_request_inserted()此时正好访问到p_process_io_info_tmp，此时
			     //就不能从链表剔除p_process_io_info_tmp了，更不能释放从链表剔除p_process_io_info_tmp，会导致blk_mq_sched_request_inserted()用的p_process_io_info_tmp是非法内存
			     //这里spin lock锁加锁防止了这个问题，具体blk_mq_sched_request_inserted()有详细解释
		             if(atomic_read(&(p_process_io_info_tmp->rq_count)) == 0){
		                  p_process_io_info_tmp->has_deleted = 1;
	                          list_del_rcu(&p_process_io_info_tmp->process_io_info_list);
		             }else{
				  //这里跳到for...循环头，最初忘记spin unlock了，态SB，内核处处都是坑????????????????????????????
	                          spin_unlock_irq(&(p_process_io_tmp->process_lock_list));
			          continue;
			     }
	                     spin_unlock_irq(&(p_process_io_tmp->process_lock_list));

			     /*p_process_io_info_tmp从process_io_control_head链表删除后不能立即添加到process_io_control_head_del链表，因为p_process_io_info_tmp指向的process_io_info可能
			     有进程正在使用，因此不能修改 p_process_io_info_tmp->next值，故不能添加到新链表，这个知识点隐藏的太深了???解决方法是用p_process_io_info_tmp的process_io_info_list_del把
			     p_process_io_info_tmp添加到 process_io_control_head_del链表*/
			     //list_add(&p_process_io_info_tmp->process_io_info_list,&(p_process_io_tmp->process_io_control_head_del));
			     list_add(&p_process_io_info_tmp->process_io_info_del,&(p_process_io_tmp->process_io_control_head_del));
			}
		}

	}
        atomic_dec(&(p_process_io_tmp->read_lock_count));//类似 rcu_read_unlock()结束宽限期Grace Period

	/*???????这段代码需要放到加锁里，因为可能此时正好p_process_io_info_del这个process_io_info绑定的进程传输IO，使用了p_process_io_info_del。而现在加了锁，这个进程根本访问不到p_process_io_info_del这个process_io_info。犯了一个错误，错误以为在 list_for_each_entry遍历链表结束后，p_process_io_info_del指向的是链表头结点，而不是链表成员process_io_info，因此这里不能再delete p_process_io_info_del。理解错了，因为链表结束后，p_process_io_info_del指向的是链表最后一个成员
	  */
	/*if(p_process_io_info_del)
	{
	    list_del(&p_process_io_info_del->process_io_info_list);
	    kmem_cache_free(p_process_io_tmp->process_io_info_cachep, p_process_io_info_del);
	}*/
        //spin_unlock_irq(&(p_process_io_tmp->process_lock));
	
	while(i ++ < 2)//加锁while循环是一旦 p_process_io_tmp->read_lock_count 不是0，先等等
	{
	    //当 read_lock_count 是0，说明process_io_control_head_del上的process_io_info在从process_io_control_head链表剔除后，遍历 process_io_control_head链表的进程都退出了宽限期，可以放心释放了
	    if(atomic_read(&(p_process_io_tmp->read_lock_count)) == 0){
		list_for_each_entry_safe(p_process_io_info_tmp,p_process_io_info_tmp_copy,&(p_process_io_tmp->process_io_control_head_del), process_io_info_del){
		    printk(KERN_DEBUG"%s %s %d process_io_info delete\n",__func__,p_process_io_info_tmp->comm,p_process_io_info_tmp->pid);
		    list_del(&p_process_io_info_tmp->process_io_info_del);
		    //真正释放process_io_info结构
		    kmem_cache_free(p_process_io_tmp->process_io_info_cachep,p_process_io_info_tmp);
		}
		break;
	    }
	    msleep(5);
        }
}
EXPORT_SYMBOL(print_process_io_info);

void blk_mq_requeue_request(struct request *rq, bool kick_requeue_list)
{
	__blk_mq_requeue_request(rq);

	/* this request will be re-inserted to io scheduler queue */
	blk_mq_sched_requeue_request(rq);

	BUG_ON(!list_empty(&rq->queuelist));
	blk_mq_add_to_requeue_list(rq, true, kick_requeue_list);
}
EXPORT_SYMBOL(blk_mq_requeue_request);

static void blk_mq_requeue_work(struct work_struct *work)
{
	struct request_queue *q =
		container_of(work, struct request_queue, requeue_work.work);
	LIST_HEAD(rq_list);
	struct request *rq, *next;

	spin_lock_irq(&q->requeue_lock);
	list_splice_init(&q->requeue_list, &rq_list);
	spin_unlock_irq(&q->requeue_lock);

	list_for_each_entry_safe(rq, next, &rq_list, queuelist) {
		if (!(rq->rq_flags & (RQF_SOFTBARRIER | RQF_DONTPREP)))
			continue;

		rq->rq_flags &= ~RQF_SOFTBARRIER;
		list_del_init(&rq->queuelist);
		/*
		 * If RQF_DONTPREP, rq has contained some driver specific
		 * data, so insert it to hctx dispatch list to avoid any
		 * merge.
		 */
		if (rq->rq_flags & RQF_DONTPREP)
			blk_mq_request_bypass_insert(rq, false, false);
		else
			blk_mq_sched_insert_request(rq, true, false, false);
	}

	while (!list_empty(&rq_list)) {
		rq = list_entry(rq_list.next, struct request, queuelist);
		list_del_init(&rq->queuelist);
		blk_mq_sched_insert_request(rq, false, false, false);
	}

	blk_mq_run_hw_queues(q, false);
}

void blk_mq_add_to_requeue_list(struct request *rq, bool at_head,
				bool kick_requeue_list)
{
	struct request_queue *q = rq->q;
	unsigned long flags;

	/*
	 * We abuse this flag that is otherwise used by the I/O scheduler to
	 * request head insertion from the workqueue.
	 */
	BUG_ON(rq->rq_flags & RQF_SOFTBARRIER);

	spin_lock_irqsave(&q->requeue_lock, flags);
	if (at_head) {
		rq->rq_flags |= RQF_SOFTBARRIER;
		list_add(&rq->queuelist, &q->requeue_list);
	} else {
		list_add_tail(&rq->queuelist, &q->requeue_list);
	}
	spin_unlock_irqrestore(&q->requeue_lock, flags);

	if (kick_requeue_list)
		blk_mq_kick_requeue_list(q);
}

void blk_mq_kick_requeue_list(struct request_queue *q)
{
	kblockd_mod_delayed_work_on(WORK_CPU_UNBOUND, &q->requeue_work, 0);
}
EXPORT_SYMBOL(blk_mq_kick_requeue_list);

void blk_mq_delay_kick_requeue_list(struct request_queue *q,
				    unsigned long msecs)
{
	kblockd_mod_delayed_work_on(WORK_CPU_UNBOUND, &q->requeue_work,
				    msecs_to_jiffies(msecs));
}
EXPORT_SYMBOL(blk_mq_delay_kick_requeue_list);

struct request *blk_mq_tag_to_rq(struct blk_mq_tags *tags, unsigned int tag)
{
	if (tag < tags->nr_tags) {
		prefetch(tags->rqs[tag]);
		return tags->rqs[tag];
	}

	return NULL;
}
EXPORT_SYMBOL(blk_mq_tag_to_rq);

static bool blk_mq_rq_inflight(struct blk_mq_hw_ctx *hctx, struct request *rq,
			       void *priv, bool reserved)
{
	/*
	 * If we find a request that isn't idle and the queue matches,
	 * we know the queue is busy. Return false to stop the iteration.
	 */
	if (blk_mq_request_started(rq) && rq->q == hctx->queue) {
		bool *busy = priv;

		*busy = true;
		return false;
	}

	return true;
}

bool blk_mq_queue_inflight(struct request_queue *q)
{
	bool busy = false;

	blk_mq_queue_tag_busy_iter(q, blk_mq_rq_inflight, &busy);
	return busy;
}
EXPORT_SYMBOL_GPL(blk_mq_queue_inflight);

static void blk_mq_rq_timed_out(struct request *req, bool reserved)
{
	req->rq_flags |= RQF_TIMED_OUT;
	if (req->q->mq_ops->timeout) {
		enum blk_eh_timer_return ret;

		ret = req->q->mq_ops->timeout(req, reserved);
		if (ret == BLK_EH_DONE)
			return;
		WARN_ON_ONCE(ret != BLK_EH_RESET_TIMER);
	}

	blk_add_timer(req);
}

static bool blk_mq_req_expired(struct request *rq, unsigned long *next)
{
	unsigned long deadline;

	if (blk_mq_rq_state(rq) != MQ_RQ_IN_FLIGHT)
		return false;
	if (rq->rq_flags & RQF_TIMED_OUT)
		return false;

	deadline = READ_ONCE(rq->deadline);
	if (time_after_eq(jiffies, deadline))
		return true;

	if (*next == 0)
		*next = deadline;
	else if (time_after(*next, deadline))
		*next = deadline;
	return false;
}

static bool blk_mq_check_expired(struct blk_mq_hw_ctx *hctx,
		struct request *rq, void *priv, bool reserved)
{
	unsigned long *next = priv;

	/*
	 * Just do a quick check if it is expired before locking the request in
	 * so we're not unnecessarilly synchronizing across CPUs.
	 */
	if (!blk_mq_req_expired(rq, next))
		return true;

	/*
	 * We have reason to believe the request may be expired. Take a
	 * reference on the request to lock this request lifetime into its
	 * currently allocated context to prevent it from being reallocated in
	 * the event the completion by-passes this timeout handler.
	 *
	 * If the reference was already released, then the driver beat the
	 * timeout handler to posting a natural completion.
	 */
	if (!refcount_inc_not_zero(&rq->ref))
		return true;

	/*
	 * The request is now locked and cannot be reallocated underneath the
	 * timeout handler's processing. Re-verify this exact request is truly
	 * expired; if it is not expired, then the request was completed and
	 * reallocated as a new request.
	 */
	if (blk_mq_req_expired(rq, next))
		blk_mq_rq_timed_out(rq, reserved);

	if (is_flush_rq(rq, hctx))
		rq->end_io(rq, 0);
	else if (refcount_dec_and_test(&rq->ref))
		__blk_mq_free_request(rq);

	return true;
}

static void blk_mq_timeout_work(struct work_struct *work)
{
	struct request_queue *q =
		container_of(work, struct request_queue, timeout_work);
	unsigned long next = 0;
	struct blk_mq_hw_ctx *hctx;
	int i;

	/* A deadlock might occur if a request is stuck requiring a
	 * timeout at the same time a queue freeze is waiting
	 * completion, since the timeout code would not be able to
	 * acquire the queue reference here.
	 *
	 * That's why we don't use blk_queue_enter here; instead, we use
	 * percpu_ref_tryget directly, because we need to be able to
	 * obtain a reference even in the short window between the queue
	 * starting to freeze, by dropping the first reference in
	 * blk_freeze_queue_start, and the moment the last request is
	 * consumed, marked by the instant q_usage_counter reaches
	 * zero.
	 */
	if (!percpu_ref_tryget(&q->q_usage_counter))
		return;

	blk_mq_queue_tag_busy_iter(q, blk_mq_check_expired, &next);

	if (next != 0) {
		mod_timer(&q->timeout, next);
	} else {
		/*
		 * Request timeouts are handled as a forward rolling timer. If
		 * we end up here it means that no requests are pending and
		 * also that no request has been pending for a while. Mark
		 * each hctx as idle.
		 */
		queue_for_each_hw_ctx(q, hctx, i) {
			/* the hctx may be unmapped, so check it here */
			if (blk_mq_hw_queue_mapped(hctx))
				blk_mq_tag_idle(hctx);
		}
	}
	blk_queue_exit(q);
}

struct flush_busy_ctx_data {
	struct blk_mq_hw_ctx *hctx;
	struct list_head *list;
};

static bool flush_busy_ctx(struct sbitmap *sb, unsigned int bitnr, void *data)
{
	struct flush_busy_ctx_data *flush_data = data;
	struct blk_mq_hw_ctx *hctx = flush_data->hctx;
	struct blk_mq_ctx *ctx = hctx->ctxs[bitnr];
	enum hctx_type type = hctx->type;

	spin_lock(&ctx->lock);
	list_splice_tail_init(&ctx->rq_lists[type], flush_data->list);
	sbitmap_clear_bit(sb, bitnr);
	spin_unlock(&ctx->lock);
	return true;
}

/*
 * Process software queues that have been marked busy, splicing them
 * to the for-dispatch
 */
void blk_mq_flush_busy_ctxs(struct blk_mq_hw_ctx *hctx, struct list_head *list)
{
	struct flush_busy_ctx_data data = {
		.hctx = hctx,
		.list = list,
	};

	sbitmap_for_each_set(&hctx->ctx_map, flush_busy_ctx, &data);
}
EXPORT_SYMBOL_GPL(blk_mq_flush_busy_ctxs);

struct dispatch_rq_data {
	struct blk_mq_hw_ctx *hctx;
	struct request *rq;
};

static bool dispatch_rq_from_ctx(struct sbitmap *sb, unsigned int bitnr,
		void *data)
{
	struct dispatch_rq_data *dispatch_data = data;
	struct blk_mq_hw_ctx *hctx = dispatch_data->hctx;
	struct blk_mq_ctx *ctx = hctx->ctxs[bitnr];
	enum hctx_type type = hctx->type;

	spin_lock(&ctx->lock);
	if (!list_empty(&ctx->rq_lists[type])) {
		dispatch_data->rq = list_entry_rq(ctx->rq_lists[type].next);
		list_del_init(&dispatch_data->rq->queuelist);
		if (list_empty(&ctx->rq_lists[type]))
			sbitmap_clear_bit(sb, bitnr);
	}
	spin_unlock(&ctx->lock);

	return !dispatch_data->rq;
}

struct request *blk_mq_dequeue_from_ctx(struct blk_mq_hw_ctx *hctx,
					struct blk_mq_ctx *start)
{
	unsigned off = start ? start->index_hw[hctx->type] : 0;
	struct dispatch_rq_data data = {
		.hctx = hctx,
		.rq   = NULL,
	};

	__sbitmap_for_each_set(&hctx->ctx_map, off,
			       dispatch_rq_from_ctx, &data);

	return data.rq;
}

static inline unsigned int queued_to_index(unsigned int queued)
{
	if (!queued)
		return 0;

	return min(BLK_MQ_MAX_DISPATCH_ORDER - 1, ilog2(queued) + 1);
}

static int blk_mq_dispatch_wake(wait_queue_entry_t *wait, unsigned mode,
				int flags, void *key)
{
	struct blk_mq_hw_ctx *hctx;

	hctx = container_of(wait, struct blk_mq_hw_ctx, dispatch_wait);

	spin_lock(&hctx->dispatch_wait_lock);
	if (!list_empty(&wait->entry)) {
		struct sbitmap_queue *sbq;

		list_del_init(&wait->entry);
		sbq = &hctx->tags->bitmap_tags;
		atomic_dec(&sbq->ws_active);
	}
	spin_unlock(&hctx->dispatch_wait_lock);

	blk_mq_run_hw_queue(hctx, true);
	return 1;
}

/*
 * Mark us waiting for a tag. For shared tags, this involves hooking us into
 * the tag wakeups. For non-shared tags, we can simply mark us needing a
 * restart. For both cases, take care to check the condition again after
 * marking us as waiting.
 */
static bool blk_mq_mark_tag_wait(struct blk_mq_hw_ctx *hctx,
				 struct request *rq)
{
	struct sbitmap_queue *sbq = &hctx->tags->bitmap_tags;
	struct wait_queue_head *wq;
	wait_queue_entry_t *wait;
	bool ret;

	if (!(hctx->flags & BLK_MQ_F_TAG_SHARED)) {
		blk_mq_sched_mark_restart_hctx(hctx);

		/*
		 * It's possible that a tag was freed in the window between the
		 * allocation failure and adding the hardware queue to the wait
		 * queue.
		 *
		 * Don't clear RESTART here, someone else could have set it.
		 * At most this will cost an extra queue run.
		 */
		return blk_mq_get_driver_tag(rq);
	}

	wait = &hctx->dispatch_wait;
	if (!list_empty_careful(&wait->entry))
		return false;

	wq = &bt_wait_ptr(sbq, hctx)->wait;

	spin_lock_irq(&wq->lock);
	spin_lock(&hctx->dispatch_wait_lock);
	if (!list_empty(&wait->entry)) {
		spin_unlock(&hctx->dispatch_wait_lock);
		spin_unlock_irq(&wq->lock);
		return false;
	}

	atomic_inc(&sbq->ws_active);
	wait->flags &= ~WQ_FLAG_EXCLUSIVE;
	__add_wait_queue(wq, wait);

	/*
	 * It's possible that a tag was freed in the window between the
	 * allocation failure and adding the hardware queue to the wait
	 * queue.
	 */
	ret = blk_mq_get_driver_tag(rq);
	if (!ret) {
		spin_unlock(&hctx->dispatch_wait_lock);
		spin_unlock_irq(&wq->lock);
		return false;
	}

	/*
	 * We got a tag, remove ourselves from the wait queue to ensure
	 * someone else gets the wakeup.
	 */
	list_del_init(&wait->entry);
	atomic_dec(&sbq->ws_active);
	spin_unlock(&hctx->dispatch_wait_lock);
	spin_unlock_irq(&wq->lock);

	return true;
}

#define BLK_MQ_DISPATCH_BUSY_EWMA_WEIGHT  8
#define BLK_MQ_DISPATCH_BUSY_EWMA_FACTOR  4
/*
 * Update dispatch busy with the Exponential Weighted Moving Average(EWMA):
 * - EWMA is one simple way to compute running average value
 * - weight(7/8 and 1/8) is applied so that it can decrease exponentially
 * - take 4 as factor for avoiding to get too small(0) result, and this
 *   factor doesn't matter because EWMA decreases exponentially
 */
static void blk_mq_update_dispatch_busy(struct blk_mq_hw_ctx *hctx, bool busy)
{
	unsigned int ewma;

	if (hctx->queue->elevator)
		return;

	ewma = hctx->dispatch_busy;

	if (!ewma && !busy)
		return;

	ewma *= BLK_MQ_DISPATCH_BUSY_EWMA_WEIGHT - 1;
	if (busy)
		ewma += 1 << BLK_MQ_DISPATCH_BUSY_EWMA_FACTOR;
	ewma /= BLK_MQ_DISPATCH_BUSY_EWMA_WEIGHT;

	hctx->dispatch_busy = ewma;
}

#define BLK_MQ_RESOURCE_DELAY	3		/* ms units */

/*
 * Returns true if we did some work AND can potentially do more.
 */
bool blk_mq_dispatch_rq_list(struct request_queue *q, struct list_head *list,
			     bool got_budget)
{
	struct blk_mq_hw_ctx *hctx;
	struct request *rq, *nxt;
	bool no_tag = false;
	int errors, queued;
	blk_status_t ret = BLK_STS_OK;
	bool no_budget_avail = false;
        int rq_pid = -1;
	struct process_rq_stat *p_process_rq_stat_tmp = NULL;
        struct process_io_info *p_process_io_info_tmp = NULL;
	LIST_HEAD(tmp_list);

	if (list_empty(list))
		return false;

	WARN_ON(!list_is_singular(list) && got_budget);

	/*
	 * Now process all the entries, sending them to the driver.
	 */
	errors = queued = 0;
	do {
		struct blk_mq_queue_data bd;

		rq = list_first_entry(list, struct request, queuelist);

		hctx = rq->mq_hctx;
		if (!got_budget && !blk_mq_get_dispatch_budget(hctx)) {
			blk_mq_put_driver_tag(rq);
			no_budget_avail = true;
			break;
		}
		if(0 /*q->high_io_prio_enable*/){
		    if((rq->rq_flags & RQF_HIGH_PRIO) && (0 == q->high_io_prio_mode)){
		        q->high_io_prio_mode = 1;
		    }

		    if(q->high_io_prio_mode){
		        if(rq->rq_flags & RQF_HIGH_PRIO){
		            q->high_io_prio_limit = 0;	
			}else{
			    q->high_io_prio_limit ++;
			    //如果q->high_io_prio_limit过大，说明已经一段时间没有高优先级IO了，则在else分支清理high_io_prio_mode
                            if(q->high_io_prio_limit < 60){
				//如果驱动队列的IO数大于16这个阀值，则不再派发非高优先级IO
		                if(atomic_read(&(rq->q->rq_in_diver_count)) > 19){
		                    //spin_lock(&hctx->lock);
				    //把非高优先级rq从list链表剔除，并移动到hctx->dispatch延迟派发,这个过程要加锁
		                    //list_move(&rq->queuelist, &hctx->dispatch);
		                    //spin_unlock(&hctx->lock);
		                    list_move(&rq->queuelist, &tmp_list);
				    printk("%s %s %d do not dispatch rq:%llx for high prio io list:%d tmp_list:%d\n",__func__,current->comm,current->pid,(u64)rq,list_empty(list),list_empty(&tmp_list));
				    //如果此req是最后一个IO请求，则不再根据优先级派发IO，这是遵循mq派发驱动的规则，最后一个必须bd.last = true传递给磁盘驱
				    if(list_empty(list)){
		                        list_move(&rq->queuelist, list);
				    }
				    else{
	                                blk_mq_put_driver_tag(rq);
				        continue;
				    }
			         }
			    }
			    else
			    {
			        //q->high_io_prio_limit = 0;
				q->high_io_prio_mode = 0;
			    }
			}

		    }
		}

		if (!blk_mq_get_driver_tag(rq)) {
			/*
			 * The initial allocation attempt failed, so we need to
			 * rerun the hardware queue when a tag is freed. The
			 * waitqueue takes care of that. If the queue is run
			 * before we add this entry back on the dispatch list,
			 * we'll re-run it below.
			 */
			if (!blk_mq_mark_tag_wait(hctx, rq)) {
				blk_mq_put_dispatch_budget(hctx);
				/*
				 * For non-shared tags, the RESTART check
				 * will suffice.
				 */
				if (hctx->flags & BLK_MQ_F_TAG_SHARED)
					no_tag = true;
				break;
			}
		}

		list_del_init(&rq->queuelist);

		bd.rq = rq;


		/******process_rq_stat***************/
                if(rq->rq_disk && rq->rq_disk->process_io.enable && rq->p_process_rq_stat){
		    //先保存rq所属进程PID
		    rq_pid = rq->p_process_rq_stat->p_process_io_info->pid;
	            p_process_rq_stat_tmp = rq->p_process_rq_stat;
		    p_process_io_info_tmp = rq->p_process_rq_stat->p_process_io_info;
		    /*printk(KERN_DEBUG"1:%s %s %d\n",__func__,current->comm,current->pid);*/
		}
		/*
		 * Flag last if we have no more requests, or if we have more
		 * but can't assign a driver tag to it.
		 */
		if (list_empty(list))
			bd.last = true;
		else {
			nxt = list_first_entry(list, struct request, queuelist);
			bd.last = !blk_mq_get_driver_tag(nxt);
		}
                //从该函数返回BLK_STS_OK，有时rq派发成功，有时失败，随机
		ret = q->mq_ops->queue_rq(hctx, &bd);
		if (ret == BLK_STS_RESOURCE || ret == BLK_STS_DEV_RESOURCE) {
			/*
			 * If an I/O scheduler has been configured and we got a
			 * driver tag for the next request already, free it
			 * again.
			 */
			if (!list_empty(list)) {
				nxt = list_first_entry(list, struct request, queuelist);
				blk_mq_put_driver_tag(nxt);
			}
			list_add(&rq->queuelist, list);
			__blk_mq_requeue_request(rq);
			break;
		}

		if (unlikely(ret != BLK_STS_OK)) {
			errors++;
			blk_mq_end_request(rq, BLK_STS_IOERR);
			continue;
		}

		/******process_rq_stat***************/
		//smp_rmb(); 
                //if(rq->rq_disk && rq->rq_disk->process_io.enable && rq->p_process_rq_stat && rq->p_process_rq_stat->p_process_io_info){
                if(rq->rq_disk && rq->rq_disk->process_io.enable && p_process_rq_stat_tmp && p_process_io_info_tmp){
	            //struct process_rq_stat *p_process_rq_stat_tmp = rq->p_process_rq_stat;
		    //struct process_io_info *p_process_io_info_tmp = rq->p_process_rq_stat->p_process_io_info;

		    //??????????执行到这里，rq这个IO请求可能还没派发完成，此时执行blk_account_io_done()里的p_process_rq_stat_tmp->real_dc_time = ktime_to_us(ktime_get()) - p_process_rq_stat_tmp->rq_real_issue_time计算的real_dc_time是准确的。
		    //但是也可能已经派发完成并执行了 blk_account_io_done()里的计算real_dc_time的代码，此时p_process_rq_stat_tmp->rq_real_issue_time还没在下边赋值，是0。
		    //当然这种情况计算的real_dc_time是有问题的，非常大。并且这个rq很快又被新的进程分配，并分配rq->p_process_rq_stat。此时执行到这里，就会错误执行
		    //p_process_rq_stat_tmp->rq_real_issue_time = ktime_to_us(ktime_get()) 赋值。此时这个rq被赋值了新的进程，只是插入IO队列，还没有派发！简单说rq->p_process_rq_stat
		    //已经是新分配的了。等这个rq接下来真的被派发，执行到这里就会发现p_process_rq_stat_tmp->rq_real_issue_time不是0，这个奇怪的现象就可以解释了
		    //
		    //还有一种情况是，执行到这里时，rq正好派发完成执行blk_account_io_done()函数，rq->p_process_rq_stat还没清NULL，
		    //因此这里的if(rq->rq_disk && rq->rq_disk->process_io.enable && rq->p_process_rq_stat)成立，这里的p_process_rq_stat_tmp->rq_real_issue_time = ktime_to_us(ktime_get());
		    //和 blk_account_io_done()里的p_process_rq_stat_tmp->real_dc_time = ktime_to_us(ktime_get())-p_process_rq_stat_tmp->rq_real_issue_time就会同时执行，谁前谁后不一定。
		    //这样可能blk_account_io_done()里先ktime_to_us(ktime_get())，然后该函数这里对p_process_rq_stat_tmp->rq_real_issue_time赋值，此时real_dc_time就是负数。要加锁保护
		    //
		    
		    //printk(KERN_DEBUG"1:%s rp_process_io_info:0x%llx\n",__func__,(u64)(rq->p_process_rq_stat->p_process_io_info));
		    //smp_rmb(); 
                    //这里的printk打印p_process_io_info是0xffff9fec1e15d100，但是if(rq_pid == rq->p_process_rq_stat->p_process_io_info->pid)却因p_process_io_info NULL而crash了
		    //printk(KERN_DEBUG"2:%s p_process_io_info:0x%llx\n",__func__,(u64)(rq->p_process_rq_stat->p_process_io_info));
		    
		    /*printk(KERN_DEBUG"2:%s %s %d\n",__func__,current->comm,current->pid);*/

		    spin_lock_irq(&(p_process_io_info_tmp->io_data_lock));
		    if(rq->p_process_rq_stat &&(rq_pid == rq->p_process_rq_stat->p_process_io_info->pid))//rq如果派发的快，到这里可能已经传输完成并被新的进程分配，这个判断限制rq不能被新的分配
		    {
		        if(p_process_rq_stat_tmp->rq_real_issue_time == 0){
			    //spin_lock_irq(&(p_process_io_info_tmp->io_data_lock)); 
		            p_process_rq_stat_tmp->rq_real_issue_time = ktime_to_us(ktime_get());
			    //smp_mb();
		            //printk("%s %s %d rq:0x%llx process_rq_stat:0x%llx rq_real_issue_time:%lld p_process_io_info_tmp:0x%llx pid:%d\n",__func__,current->comm,current->pid,(u64)rq,(u64)(rq->p_process_rq_stat),p_process_rq_stat_tmp->rq_real_issue_time,(u64)p_process_io_info_tmp,p_process_io_info_tmp->pid);
			    //spin_unlock_irq(&(p_process_io_info_tmp->io_data_lock));
			}
		        else
			{
			    //走到这里说明发生了错误，把 rq_real_issue_time 清0，表示本轮采集的rq_real_issue_time无效
			    p_process_rq_stat_tmp->rq_real_issue_time = 0;
		            //printk(KERN_ERR"!!!!!!!!!!%s %s %d rq:0x%llx process_rq_stat:0x%llx p_process_io_info_tmp:0x%llx pid:%d rq_real_issue_time:%llu rq_issue_time:%llu rq_inset_time:%llu\n",__func__,current->comm,current->pid,(u64)rq,(u64)(rq->p_process_rq_stat),(u64)p_process_io_info_tmp,p_process_io_info_tmp->pid,p_process_rq_stat_tmp->rq_real_issue_time,p_process_rq_stat_tmp->rq_issue_time,p_process_rq_stat_tmp->rq_inset_time);
		//	    dump_stack();
		        }
		    }
		    spin_unlock_irq(&(p_process_io_info_tmp->io_data_lock));
		}
		queued++;
	} while (!list_empty(list));

	hctx->dispatched[queued_to_index(queued)]++;

	if (0 /*!list_empty(&tmp_list)*/){
	    printk("%s %s %d move tmp_list to hctx->dispatch ret:%d queued:%d errors:%d\n",__func__,current->comm,current->pid,ret,queued,errors);
	    spin_lock(&hctx->lock);
	    //把非高优先级rq从list链表剔除，并移动到hctx->dispatch延迟派发,这个过程要加锁
	    list_splice_init(&tmp_list, &hctx->dispatch);
	    spin_unlock(&hctx->lock);
	}

	/*
	 * Any items that need requeuing? Stuff them into hctx->dispatch,
	 * that is where we will continue on next queue run.
	 */
	if (!list_empty(list)) {
		bool needs_restart;

		/*
		 * If we didn't flush the entire list, we could have told
		 * the driver there was more coming, but that turned out to
		 * be a lie.
		 */
		if (q->mq_ops->commit_rqs && queued)
			q->mq_ops->commit_rqs(hctx);

		spin_lock(&hctx->lock);
		list_splice_tail_init(list, &hctx->dispatch);
		spin_unlock(&hctx->lock);

		/*
		 * Order adding requests to hctx->dispatch and checking
		 * SCHED_RESTART flag. The pair of this smp_mb() is the one
		 * in blk_mq_sched_restart(). Avoid restart code path to
		 * miss the new added requests to hctx->dispatch, meantime
		 * SCHED_RESTART is observed here.
		 */
		smp_mb();

		/*
		 * If SCHED_RESTART was set by the caller of this function and
		 * it is no longer set that means that it was cleared by another
		 * thread and hence that a queue rerun is needed.
		 *
		 * If 'no_tag' is set, that means that we failed getting
		 * a driver tag with an I/O scheduler attached. If our dispatch
		 * waitqueue is no longer active, ensure that we run the queue
		 * AFTER adding our entries back to the list.
		 *
		 * If no I/O scheduler has been configured it is possible that
		 * the hardware queue got stopped and restarted before requests
		 * were pushed back onto the dispatch list. Rerun the queue to
		 * avoid starvation. Notes:
		 * - blk_mq_run_hw_queue() checks whether or not a queue has
		 *   been stopped before rerunning a queue.
		 * - Some but not all block drivers stop a queue before
		 *   returning BLK_STS_RESOURCE. Two exceptions are scsi-mq
		 *   and dm-rq.
		 *
		 * If driver returns BLK_STS_RESOURCE and SCHED_RESTART
		 * bit is set, run queue after a delay to avoid IO stalls
		 * that could otherwise occur if the queue is idle.  We'll do
		 * similar if we couldn't get budget and SCHED_RESTART is set.
		 */
		needs_restart = blk_mq_sched_needs_restart(hctx);
		if (!needs_restart ||
		    (no_tag && list_empty_careful(&hctx->dispatch_wait.entry)))
			blk_mq_run_hw_queue(hctx, true);
		else if (needs_restart && (ret == BLK_STS_RESOURCE ||
					   no_budget_avail))
			blk_mq_delay_run_hw_queue(hctx, BLK_MQ_RESOURCE_DELAY);

		blk_mq_update_dispatch_busy(hctx, true);
		return false;
	} else
		blk_mq_update_dispatch_busy(hctx, false);
        
	if(queued > 1){
	    printk("%s %s %d ret:%d queued:%d errors:%d\n",__func__,current->comm,current->pid,ret,queued,errors);
	    dump_stack();
	}
	/*
	 * If the host/device is unable to accept more work, inform the
	 * caller of that.
	 */
	if (ret == BLK_STS_RESOURCE || ret == BLK_STS_DEV_RESOURCE)
		return false;

	return (queued + errors) != 0;
}

static void __blk_mq_run_hw_queue(struct blk_mq_hw_ctx *hctx)
{
	int srcu_idx;

	/*
	 * We should be running this queue from one of the CPUs that
	 * are mapped to it.
	 *
	 * There are at least two related races now between setting
	 * hctx->next_cpu from blk_mq_hctx_next_cpu() and running
	 * __blk_mq_run_hw_queue():
	 *
	 * - hctx->next_cpu is found offline in blk_mq_hctx_next_cpu(),
	 *   but later it becomes online, then this warning is harmless
	 *   at all
	 *
	 * - hctx->next_cpu is found online in blk_mq_hctx_next_cpu(),
	 *   but later it becomes offline, then the warning can't be
	 *   triggered, and we depend on blk-mq timeout handler to
	 *   handle dispatched requests to this hctx
	 */
	if (!cpumask_test_cpu(raw_smp_processor_id(), hctx->cpumask) &&
		cpu_online(hctx->next_cpu)) {
		printk(KERN_WARNING "run queue from wrong CPU %d, hctx %s\n",
			raw_smp_processor_id(),
			cpumask_empty(hctx->cpumask) ? "inactive": "active");
		dump_stack();
	}

	/*
	 * We can't run the queue inline with ints disabled. Ensure that
	 * we catch bad users of this early.
	 */
	WARN_ON_ONCE(in_interrupt());

	might_sleep_if(hctx->flags & BLK_MQ_F_BLOCKING);

	hctx_lock(hctx, &srcu_idx);
	blk_mq_sched_dispatch_requests(hctx);
	hctx_unlock(hctx, srcu_idx);
}

static inline int blk_mq_first_mapped_cpu(struct blk_mq_hw_ctx *hctx)
{
	int cpu = cpumask_first_and(hctx->cpumask, cpu_online_mask);

	if (cpu >= nr_cpu_ids)
		cpu = cpumask_first(hctx->cpumask);
	return cpu;
}

/*
 * It'd be great if the workqueue API had a way to pass
 * in a mask and had some smarts for more clever placement.
 * For now we just round-robin here, switching for every
 * BLK_MQ_CPU_WORK_BATCH queued items.
 */
static int blk_mq_hctx_next_cpu(struct blk_mq_hw_ctx *hctx)
{
	bool tried = false;
	int next_cpu = hctx->next_cpu;

	if (hctx->queue->nr_hw_queues == 1)
		return WORK_CPU_UNBOUND;

	if (--hctx->next_cpu_batch <= 0) {
select_cpu:
		next_cpu = cpumask_next_and(next_cpu, hctx->cpumask,
				cpu_online_mask);
		if (next_cpu >= nr_cpu_ids)
			next_cpu = blk_mq_first_mapped_cpu(hctx);
		hctx->next_cpu_batch = BLK_MQ_CPU_WORK_BATCH;
	}

	/*
	 * Do unbound schedule if we can't find a online CPU for this hctx,
	 * and it should only happen in the path of handling CPU DEAD.
	 */
	if (!cpu_online(next_cpu)) {
		if (!tried) {
			tried = true;
			goto select_cpu;
		}

		/*
		 * Make sure to re-select CPU next time once after CPUs
		 * in hctx->cpumask become online again.
		 */
		hctx->next_cpu = next_cpu;
		hctx->next_cpu_batch = 1;
		return WORK_CPU_UNBOUND;
	}

	hctx->next_cpu = next_cpu;
	return next_cpu;
}

static void __blk_mq_delay_run_hw_queue(struct blk_mq_hw_ctx *hctx, bool async,
					unsigned long msecs)
{
	if (unlikely(blk_mq_hctx_stopped(hctx)))
		return;

	if (!async && !(hctx->flags & BLK_MQ_F_BLOCKING)) {
		int cpu = get_cpu();
		if (cpumask_test_cpu(cpu, hctx->cpumask)) {
			__blk_mq_run_hw_queue(hctx);
			put_cpu();
			return;
		}

		put_cpu();
	}

	kblockd_mod_delayed_work_on(blk_mq_hctx_next_cpu(hctx), &hctx->run_work,
				    msecs_to_jiffies(msecs));
}

void blk_mq_delay_run_hw_queue(struct blk_mq_hw_ctx *hctx, unsigned long msecs)
{
	__blk_mq_delay_run_hw_queue(hctx, true, msecs);
}
EXPORT_SYMBOL(blk_mq_delay_run_hw_queue);

void blk_mq_run_hw_queue(struct blk_mq_hw_ctx *hctx, bool async)
{
	int srcu_idx;
	bool need_run;

	/*
	 * When queue is quiesced, we may be switching io scheduler, or
	 * updating nr_hw_queues, or other things, and we can't run queue
	 * any more, even __blk_mq_hctx_has_pending() can't be called safely.
	 *
	 * And queue will be rerun in blk_mq_unquiesce_queue() if it is
	 * quiesced.
	 */
	hctx_lock(hctx, &srcu_idx);
	need_run = !blk_queue_quiesced(hctx->queue) &&
		blk_mq_hctx_has_pending(hctx);
	hctx_unlock(hctx, srcu_idx);

	if (need_run)
		__blk_mq_delay_run_hw_queue(hctx, async, 0);
}
EXPORT_SYMBOL(blk_mq_run_hw_queue);

void blk_mq_run_hw_queues(struct request_queue *q, bool async)
{
	struct blk_mq_hw_ctx *hctx;
	int i;

	queue_for_each_hw_ctx(q, hctx, i) {
		if (blk_mq_hctx_stopped(hctx))
			continue;

		blk_mq_run_hw_queue(hctx, async);
	}
}
EXPORT_SYMBOL(blk_mq_run_hw_queues);

/**
 * blk_mq_delay_run_hw_queues - Run all hardware queues asynchronously.
 * @q: Pointer to the request queue to run.
 * @msecs: Microseconds of delay to wait before running the queues.
 */
void blk_mq_delay_run_hw_queues(struct request_queue *q, unsigned long msecs)
{
	struct blk_mq_hw_ctx *hctx;
	int i;

	queue_for_each_hw_ctx(q, hctx, i) {
		if (blk_mq_hctx_stopped(hctx))
			continue;

		blk_mq_delay_run_hw_queue(hctx, msecs);
	}
}
EXPORT_SYMBOL(blk_mq_delay_run_hw_queues);

/**
 * blk_mq_queue_stopped() - check whether one or more hctxs have been stopped
 * @q: request queue.
 *
 * The caller is responsible for serializing this function against
 * blk_mq_{start,stop}_hw_queue().
 */
bool blk_mq_queue_stopped(struct request_queue *q)
{
	struct blk_mq_hw_ctx *hctx;
	int i;

	queue_for_each_hw_ctx(q, hctx, i)
		if (blk_mq_hctx_stopped(hctx))
			return true;

	return false;
}
EXPORT_SYMBOL(blk_mq_queue_stopped);

/*
 * This function is often used for pausing .queue_rq() by driver when
 * there isn't enough resource or some conditions aren't satisfied, and
 * BLK_STS_RESOURCE is usually returned.
 *
 * We do not guarantee that dispatch can be drained or blocked
 * after blk_mq_stop_hw_queue() returns. Please use
 * blk_mq_quiesce_queue() for that requirement.
 */
void blk_mq_stop_hw_queue(struct blk_mq_hw_ctx *hctx)
{
	cancel_delayed_work(&hctx->run_work);

	set_bit(BLK_MQ_S_STOPPED, &hctx->state);
}
EXPORT_SYMBOL(blk_mq_stop_hw_queue);

/*
 * This function is often used for pausing .queue_rq() by driver when
 * there isn't enough resource or some conditions aren't satisfied, and
 * BLK_STS_RESOURCE is usually returned.
 *
 * We do not guarantee that dispatch can be drained or blocked
 * after blk_mq_stop_hw_queues() returns. Please use
 * blk_mq_quiesce_queue() for that requirement.
 */
void blk_mq_stop_hw_queues(struct request_queue *q)
{
	struct blk_mq_hw_ctx *hctx;
	int i;

	queue_for_each_hw_ctx(q, hctx, i)
		blk_mq_stop_hw_queue(hctx);
}
EXPORT_SYMBOL(blk_mq_stop_hw_queues);

void blk_mq_start_hw_queue(struct blk_mq_hw_ctx *hctx)
{
	clear_bit(BLK_MQ_S_STOPPED, &hctx->state);

	blk_mq_run_hw_queue(hctx, false);
}
EXPORT_SYMBOL(blk_mq_start_hw_queue);

void blk_mq_start_hw_queues(struct request_queue *q)
{
	struct blk_mq_hw_ctx *hctx;
	int i;

	queue_for_each_hw_ctx(q, hctx, i)
		blk_mq_start_hw_queue(hctx);
}
EXPORT_SYMBOL(blk_mq_start_hw_queues);

void blk_mq_start_stopped_hw_queue(struct blk_mq_hw_ctx *hctx, bool async)
{
	if (!blk_mq_hctx_stopped(hctx))
		return;

	clear_bit(BLK_MQ_S_STOPPED, &hctx->state);
	blk_mq_run_hw_queue(hctx, async);
}
EXPORT_SYMBOL_GPL(blk_mq_start_stopped_hw_queue);

void blk_mq_start_stopped_hw_queues(struct request_queue *q, bool async)
{
	struct blk_mq_hw_ctx *hctx;
	int i;

	queue_for_each_hw_ctx(q, hctx, i)
		blk_mq_start_stopped_hw_queue(hctx, async);
}
EXPORT_SYMBOL(blk_mq_start_stopped_hw_queues);

static void blk_mq_run_work_fn(struct work_struct *work)
{
	struct blk_mq_hw_ctx *hctx;

	hctx = container_of(work, struct blk_mq_hw_ctx, run_work.work);

	/*
	 * If we are stopped, don't run the queue.
	 */
	if (test_bit(BLK_MQ_S_STOPPED, &hctx->state))
		return;

	__blk_mq_run_hw_queue(hctx);
}

static inline void __blk_mq_insert_req_list(struct blk_mq_hw_ctx *hctx,
					    struct request *rq,
					    bool at_head)
{
	struct blk_mq_ctx *ctx = rq->mq_ctx;
	enum hctx_type type = hctx->type;

	lockdep_assert_held(&ctx->lock);

	trace_block_rq_insert(hctx->queue, rq);

	if (at_head)
		list_add(&rq->queuelist, &ctx->rq_lists[type]);
	else
		list_add_tail(&rq->queuelist, &ctx->rq_lists[type]);
}

void __blk_mq_insert_request(struct blk_mq_hw_ctx *hctx, struct request *rq,
			     bool at_head)
{
	struct blk_mq_ctx *ctx = rq->mq_ctx;

	lockdep_assert_held(&ctx->lock);

	__blk_mq_insert_req_list(hctx, rq, at_head);
	blk_mq_hctx_mark_pending(hctx, ctx);
}

/*
 * Should only be used carefully, when the caller knows we want to
 * bypass a potential IO scheduler on the target device.
 */
void blk_mq_request_bypass_insert(struct request *rq, bool at_head,
				  bool run_queue)
{
	struct blk_mq_hw_ctx *hctx = rq->mq_hctx;

	spin_lock(&hctx->lock);
	if (at_head)
		list_add(&rq->queuelist, &hctx->dispatch);
	else
		list_add_tail(&rq->queuelist, &hctx->dispatch);
	spin_unlock(&hctx->lock);

	if (run_queue)
		blk_mq_run_hw_queue(hctx, false);
}

void blk_mq_insert_requests(struct blk_mq_hw_ctx *hctx, struct blk_mq_ctx *ctx,
			    struct list_head *list)

{
	struct request *rq;
	enum hctx_type type = hctx->type;

	/*
	 * preemption doesn't flush plug list, so it's possible ctx->cpu is
	 * offline now
	 */
	list_for_each_entry(rq, list, queuelist) {
		BUG_ON(rq->mq_ctx != ctx);
		trace_block_rq_insert(hctx->queue, rq);
	}

	spin_lock(&ctx->lock);
	list_splice_tail_init(list, &ctx->rq_lists[type]);
	blk_mq_hctx_mark_pending(hctx, ctx);
	spin_unlock(&ctx->lock);
}

static int plug_rq_cmp(void *priv, struct list_head *a, struct list_head *b)
{
	struct request *rqa = container_of(a, struct request, queuelist);
	struct request *rqb = container_of(b, struct request, queuelist);

	if (rqa->mq_ctx < rqb->mq_ctx)
		return -1;
	else if (rqa->mq_ctx > rqb->mq_ctx)
		return 1;
	else if (rqa->mq_hctx < rqb->mq_hctx)
		return -1;
	else if (rqa->mq_hctx > rqb->mq_hctx)
		return 1;

	return blk_rq_pos(rqa) > blk_rq_pos(rqb);
}

void blk_mq_flush_plug_list(struct blk_plug *plug, bool from_schedule)
{
	struct blk_mq_hw_ctx *this_hctx;
	struct blk_mq_ctx *this_ctx;
	struct request_queue *this_q;
	struct request *rq;
	LIST_HEAD(list);
	LIST_HEAD(rq_list);
	unsigned int depth;

	list_splice_init(&plug->mq_list, &list);

	if (plug->rq_count > 2 && plug->multiple_queues)
		list_sort(NULL, &list, plug_rq_cmp);

	plug->rq_count = 0;

	this_q = NULL;
	this_hctx = NULL;
	this_ctx = NULL;
	depth = 0;

	while (!list_empty(&list)) {
		rq = list_entry_rq(list.next);
		list_del_init(&rq->queuelist);
		BUG_ON(!rq->q);
		if (rq->mq_hctx != this_hctx || rq->mq_ctx != this_ctx) {
			if (this_hctx) {
				trace_block_unplug(this_q, depth, !from_schedule);
				blk_mq_sched_insert_requests(this_hctx, this_ctx,
								&rq_list,
								from_schedule);
			}

			this_q = rq->q;
			this_ctx = rq->mq_ctx;
			this_hctx = rq->mq_hctx;
			depth = 0;
		}

		depth++;
		list_add_tail(&rq->queuelist, &rq_list);
	}

	/*
	 * If 'this_hctx' is set, we know we have entries to complete
	 * on 'rq_list'. Do those.
	 */
	if (this_hctx) {
		trace_block_unplug(this_q, depth, !from_schedule);
		blk_mq_sched_insert_requests(this_hctx, this_ctx, &rq_list,
						from_schedule);
	}
}

static void blk_mq_bio_to_request(struct request *rq, struct bio *bio)
{
	if (bio->bi_opf & REQ_RAHEAD)
		rq->cmd_flags |= REQ_FAILFAST_MASK;

	rq->__sector = bio->bi_iter.bi_sector;
	rq->write_hint = bio->bi_write_hint;
	blk_rq_bio_prep(rq->q, rq, bio);

	blk_account_io_start(rq, true);
}

static blk_status_t __blk_mq_issue_directly(struct blk_mq_hw_ctx *hctx,
					    struct request *rq,
					    blk_qc_t *cookie, bool last)
{
	struct request_queue *q = rq->q;
	struct blk_mq_queue_data bd = {
		.rq = rq,
		.last = last,
	};
	blk_qc_t new_cookie;
	blk_status_t ret;

	new_cookie = request_to_qc_t(hctx, rq);

	/*
	 * For OK queue, we are done. For error, caller may kill it.
	 * Any other error (busy), just add it to our list as we
	 * previously would have done.
	 */
	ret = q->mq_ops->queue_rq(hctx, &bd);
	switch (ret) {
	case BLK_STS_OK:
		blk_mq_update_dispatch_busy(hctx, false);
		*cookie = new_cookie;
		break;
	case BLK_STS_RESOURCE:
	case BLK_STS_DEV_RESOURCE:
		blk_mq_update_dispatch_busy(hctx, true);
		__blk_mq_requeue_request(rq);
		break;
	default:
		blk_mq_update_dispatch_busy(hctx, false);
		*cookie = BLK_QC_T_NONE;
		break;
	}

	return ret;
}

static blk_status_t __blk_mq_try_issue_directly(struct blk_mq_hw_ctx *hctx,
						struct request *rq,
						blk_qc_t *cookie,
						bool bypass_insert, bool last)
{
	struct request_queue *q = rq->q;
	bool run_queue = true;

	/*
	 * RCU or SRCU read lock is needed before checking quiesced flag.
	 *
	 * When queue is stopped or quiesced, ignore 'bypass_insert' from
	 * blk_mq_request_issue_directly(), and return BLK_STS_OK to caller,
	 * and avoid driver to try to dispatch again.
	 */
	if (blk_mq_hctx_stopped(hctx) || blk_queue_quiesced(q)) {
		run_queue = false;
		bypass_insert = false;
		goto insert;
	}

	if (q->elevator && !bypass_insert)
		goto insert;

	if (!blk_mq_get_dispatch_budget(hctx))
		goto insert;

	if (!blk_mq_get_driver_tag(rq)) {
		blk_mq_put_dispatch_budget(hctx);
		goto insert;
	}

	return __blk_mq_issue_directly(hctx, rq, cookie, last);
insert:
	if (bypass_insert)
		return BLK_STS_RESOURCE;

	blk_mq_request_bypass_insert(rq, false, run_queue);
	return BLK_STS_OK;
}

static void blk_mq_try_issue_directly(struct blk_mq_hw_ctx *hctx,
		struct request *rq, blk_qc_t *cookie)
{
	blk_status_t ret;
	int srcu_idx;

	might_sleep_if(hctx->flags & BLK_MQ_F_BLOCKING);

	hctx_lock(hctx, &srcu_idx);

	ret = __blk_mq_try_issue_directly(hctx, rq, cookie, false, true);
	if (ret == BLK_STS_RESOURCE || ret == BLK_STS_DEV_RESOURCE)
		blk_mq_request_bypass_insert(rq, false, true);
	else if (ret != BLK_STS_OK)
		blk_mq_end_request(rq, ret);

	hctx_unlock(hctx, srcu_idx);
}

blk_status_t blk_mq_request_issue_directly(struct request *rq, bool last)
{
	blk_status_t ret;
	int srcu_idx;
	blk_qc_t unused_cookie;
	struct blk_mq_hw_ctx *hctx = rq->mq_hctx;

	hctx_lock(hctx, &srcu_idx);
	ret = __blk_mq_try_issue_directly(hctx, rq, &unused_cookie, true, last);
	hctx_unlock(hctx, srcu_idx);

	return ret;
}

void blk_mq_try_issue_list_directly(struct blk_mq_hw_ctx *hctx,
		struct list_head *list)
{
	int queued = 0;

	while (!list_empty(list)) {
		blk_status_t ret;
		struct request *rq = list_first_entry(list, struct request,
				queuelist);

		list_del_init(&rq->queuelist);
		ret = blk_mq_request_issue_directly(rq, list_empty(list));
		if (ret != BLK_STS_OK) {
			if (ret == BLK_STS_RESOURCE ||
					ret == BLK_STS_DEV_RESOURCE) {
				blk_mq_request_bypass_insert(rq, false,
							list_empty(list));
				break;
			}
			blk_mq_end_request(rq, ret);
		} else
			queued++;
	}

	/*
	 * If we didn't flush the entire list, we could have told
	 * the driver there was more coming, but that turned out to
	 * be a lie.
	 */
	if (!list_empty(list) && hctx->queue->mq_ops->commit_rqs && queued)
		hctx->queue->mq_ops->commit_rqs(hctx);
}

static void blk_add_rq_to_plug(struct blk_plug *plug, struct request *rq)
{
	list_add_tail(&rq->queuelist, &plug->mq_list);
	plug->rq_count++;
	if (!plug->multiple_queues && !list_is_singular(&plug->mq_list)) {
		struct request *tmp;

		tmp = list_first_entry(&plug->mq_list, struct request,
						queuelist);
		if (tmp->q != rq->q)
			plug->multiple_queues = true;
	}
}

static blk_qc_t blk_mq_make_request(struct request_queue *q, struct bio *bio)
{
	const int is_sync = op_is_sync(bio->bi_opf);
	const int is_flush_fua = op_is_flush(bio->bi_opf);
	struct blk_mq_alloc_data data = {
		.q		= q,
		.bio		= bio,
	};
	struct request *rq;
	struct blk_plug *plug;
	struct request *same_queue_rq = NULL;
	blk_qc_t cookie;

	blk_queue_bounce(q, &bio);

	blk_queue_split(q, &bio);

	if (!bio_integrity_prep(bio))
		return BLK_QC_T_NONE;

	if (!is_flush_fua && !blk_queue_nomerges(q) &&
	    blk_attempt_plug_merge(q, bio, &same_queue_rq))
		return BLK_QC_T_NONE;

	if (blk_mq_sched_bio_merge(q, bio))
		return BLK_QC_T_NONE;

	rq_qos_throttle(q, bio);

	data.cmd_flags = bio->bi_opf;
	blk_queue_enter_live(q);
	rq = __blk_mq_alloc_request(&data);
	if (unlikely(!rq)) {
		blk_queue_exit(q);
		rq_qos_cleanup(q, bio);
		if (bio->bi_opf & REQ_NOWAIT)
			bio_wouldblock_error(bio);
		return BLK_QC_T_NONE;
	}

	trace_block_getrq(q, bio, bio->bi_opf);

	rq_qos_track(q, rq, bio);

	cookie = request_to_qc_t(data.hctx, rq);

	blk_mq_bio_to_request(rq, bio);

	plug = blk_mq_plug(q, bio);
	if (unlikely(is_flush_fua)) {
		/* bypass scheduler for flush rq */
		blk_insert_flush(rq);
		blk_mq_run_hw_queue(data.hctx, true);
	} else if (plug && (q->nr_hw_queues == 1 || q->mq_ops->commit_rqs ||
				!blk_queue_nonrot(q))) {
		/*
		 * Use plugging if we have a ->commit_rqs() hook as well, as
		 * we know the driver uses bd->last in a smart fashion.
		 *
		 * Use normal plugging if this disk is slow HDD, as sequential
		 * IO may benefit a lot from plug merging.
		 */
		unsigned int request_count = plug->rq_count;
		struct request *last = NULL;

		if (!request_count)
			trace_block_plug(q);
		else
			last = list_entry_rq(plug->mq_list.prev);

		if (request_count >= BLK_MAX_REQUEST_COUNT || (last &&
		    blk_rq_bytes(last) >= BLK_PLUG_FLUSH_SIZE)) {
			blk_flush_plug_list(plug, false);
			trace_block_plug(q);
		}

		blk_add_rq_to_plug(plug, rq);
	} else if (q->elevator) {
		blk_mq_sched_insert_request(rq, false, true, true);
	} else if (plug && !blk_queue_nomerges(q)) {
		/*
		 * We do limited plugging. If the bio can be merged, do that.
		 * Otherwise the existing request in the plug list will be
		 * issued. So the plug list will have one request at most
		 * The plug list might get flushed before this. If that happens,
		 * the plug list is empty, and same_queue_rq is invalid.
		 */
		if (list_empty(&plug->mq_list))
			same_queue_rq = NULL;
		if (same_queue_rq) {
			list_del_init(&same_queue_rq->queuelist);
			plug->rq_count--;
		}
		blk_add_rq_to_plug(plug, rq);
		trace_block_plug(q);

		if (same_queue_rq) {
			data.hctx = same_queue_rq->mq_hctx;
			trace_block_unplug(q, 1, true);
			blk_mq_try_issue_directly(data.hctx, same_queue_rq,
					&cookie);
		}
	} else if ((q->nr_hw_queues > 1 && is_sync) ||
			!data.hctx->dispatch_busy) {
		blk_mq_try_issue_directly(data.hctx, rq, &cookie);
	} else {
		blk_mq_sched_insert_request(rq, false, true, true);
	}

	return cookie;
}

void blk_mq_free_rqs(struct blk_mq_tag_set *set, struct blk_mq_tags *tags,
		     unsigned int hctx_idx)
{
	struct page *page;

	if (tags->rqs && set->ops->exit_request) {
		int i;

		for (i = 0; i < tags->nr_tags; i++) {
			struct request *rq = tags->static_rqs[i];

			if (!rq)
				continue;
			set->ops->exit_request(set, rq, hctx_idx);
			tags->static_rqs[i] = NULL;
		}
	}

	while (!list_empty(&tags->page_list)) {
		page = list_first_entry(&tags->page_list, struct page, lru);
		list_del_init(&page->lru);
		/*
		 * Remove kmemleak object previously allocated in
		 * blk_mq_init_rq_map().
		 */
		kmemleak_free(page_address(page));
		__free_pages(page, page->private);
	}
}

void blk_mq_free_rq_map(struct blk_mq_tags *tags)
{
	kfree(tags->rqs);
	tags->rqs = NULL;
	kfree(tags->static_rqs);
	tags->static_rqs = NULL;

	blk_mq_free_tags(tags);
}

struct blk_mq_tags *blk_mq_alloc_rq_map(struct blk_mq_tag_set *set,
					unsigned int hctx_idx,
					unsigned int nr_tags,
					unsigned int reserved_tags)
{
	struct blk_mq_tags *tags;
	int node;

	node = blk_mq_hw_queue_to_node(&set->map[HCTX_TYPE_DEFAULT], hctx_idx);
	if (node == NUMA_NO_NODE)
		node = set->numa_node;

	tags = blk_mq_init_tags(nr_tags, reserved_tags, node,
				BLK_MQ_FLAG_TO_ALLOC_POLICY(set->flags));
	if (!tags)
		return NULL;

	tags->rqs = kcalloc_node(nr_tags, sizeof(struct request *),
				 GFP_NOIO | __GFP_NOWARN | __GFP_NORETRY,
				 node);
	if (!tags->rqs) {
		blk_mq_free_tags(tags);
		return NULL;
	}

	tags->static_rqs = kcalloc_node(nr_tags, sizeof(struct request *),
					GFP_NOIO | __GFP_NOWARN | __GFP_NORETRY,
					node);
	if (!tags->static_rqs) {
		kfree(tags->rqs);
		blk_mq_free_tags(tags);
		return NULL;
	}

	return tags;
}

static size_t order_to_size(unsigned int order)
{
	return (size_t)PAGE_SIZE << order;
}

static int blk_mq_init_request(struct blk_mq_tag_set *set, struct request *rq,
			       unsigned int hctx_idx, int node)
{
	int ret;

	if (set->ops->init_request) {
		ret = set->ops->init_request(set, rq, hctx_idx, node);
		if (ret)
			return ret;
	}

	WRITE_ONCE(rq->state, MQ_RQ_IDLE);
	return 0;
}

int blk_mq_alloc_rqs(struct blk_mq_tag_set *set, struct blk_mq_tags *tags,
		     unsigned int hctx_idx, unsigned int depth)
{
	unsigned int i, j, entries_per_page, max_order = 4;
	size_t rq_size, left;
	int node;

	node = blk_mq_hw_queue_to_node(&set->map[HCTX_TYPE_DEFAULT], hctx_idx);
	if (node == NUMA_NO_NODE)
		node = set->numa_node;

	INIT_LIST_HEAD(&tags->page_list);

	/*
	 * rq_size is the size of the request plus driver payload, rounded
	 * to the cacheline size
	 */
	rq_size = round_up(sizeof(struct request_aux) + sizeof(struct request) + set->cmd_size,
				cache_line_size());
	left = rq_size * depth;

	for (i = 0; i < depth; ) {
		int this_order = max_order;
		struct page *page;
		int to_do;
		void *p;

		while (this_order && left < order_to_size(this_order - 1))
			this_order--;

		do {
			page = alloc_pages_node(node,
				GFP_NOIO | __GFP_NOWARN | __GFP_NORETRY | __GFP_ZERO,
				this_order);
			if (page)
				break;
			if (!this_order--)
				break;
			if (order_to_size(this_order) < rq_size)
				break;
		} while (1);

		if (!page)
			goto fail;

		page->private = this_order;
		list_add_tail(&page->lru, &tags->page_list);

		p = page_address(page);
		/*
		 * Allow kmemleak to scan these pages as they contain pointers
		 * to additional allocations like via ops->init_request().
		 */
		kmemleak_alloc(p, order_to_size(this_order), 1, GFP_NOIO);
		entries_per_page = order_to_size(this_order) / rq_size;
		to_do = min(entries_per_page, depth - i);
		left -= to_do * rq_size;
		for (j = 0; j < to_do; j++) {
			struct request *rq = p + sizeof(struct request_aux);

			tags->static_rqs[i] = rq;
			if (blk_mq_init_request(set, rq, hctx_idx, node)) {
				tags->static_rqs[i] = NULL;
				goto fail;
			}

			p += rq_size;
			i++;
		}
	}
	return 0;

fail:
	blk_mq_free_rqs(set, tags, hctx_idx);
	return -ENOMEM;
}

struct rq_iter_data {
	struct blk_mq_hw_ctx *hctx;
	bool has_rq;
};

static bool blk_mq_has_request(struct request *rq, void *data, bool reserved)
{
	struct rq_iter_data *iter_data = data;

	if (rq->mq_hctx != iter_data->hctx)
		return true;
	iter_data->has_rq = true;
	return false;
}

static bool blk_mq_hctx_has_requests(struct blk_mq_hw_ctx *hctx)
{
	struct blk_mq_tags *tags = hctx->sched_tags ?
			hctx->sched_tags : hctx->tags;
	struct rq_iter_data data = {
		.hctx	= hctx,
	};

	blk_mq_all_tag_iter(tags, blk_mq_has_request, &data);
	return data.has_rq;
}

static inline bool blk_mq_last_cpu_in_hctx(unsigned int cpu,
		struct blk_mq_hw_ctx *hctx)
{
	if (cpumask_next_and(-1, hctx->cpumask, cpu_online_mask) != cpu)
		return false;
	if (cpumask_next_and(cpu, hctx->cpumask, cpu_online_mask) < nr_cpu_ids)
		return false;
	return true;
}

static int blk_mq_hctx_notify_offline(unsigned int cpu, struct hlist_node *node)
{
	struct blk_mq_hw_ctx *hctx = hlist_entry_safe(node,
			struct blk_mq_hw_ctx, cpuhp_online);

	if (!cpumask_test_cpu(cpu, hctx->cpumask) ||
	    !blk_mq_last_cpu_in_hctx(cpu, hctx))
		return 0;

	/*
	 * Prevent new request from being allocated on the current hctx.
	 *
	 * The smp_mb__after_atomic() Pairs with the implied barrier in
	 * test_and_set_bit_lock in sbitmap_get().  Ensures the inactive flag is
	 * seen once we return from the tag allocator.
	 */
	set_bit(BLK_MQ_S_INACTIVE, &hctx->state);
	smp_mb__after_atomic();

	/*
	 * Try to grab a reference to the queue and wait for any outstanding
	 * requests.  If we could not grab a reference the queue has been
	 * frozen and there are no requests.
	 */
	if (percpu_ref_tryget(&hctx->queue->q_usage_counter)) {
		while (blk_mq_hctx_has_requests(hctx))
			msleep(5);
		percpu_ref_put(&hctx->queue->q_usage_counter);
	}

	return 0;
}

static int blk_mq_hctx_notify_online(unsigned int cpu, struct hlist_node *node)
{
	struct blk_mq_hw_ctx *hctx = hlist_entry_safe(node,
			struct blk_mq_hw_ctx, cpuhp_online);

	if (cpumask_test_cpu(cpu, hctx->cpumask))
		clear_bit(BLK_MQ_S_INACTIVE, &hctx->state);
	return 0;
}

/*
 * 'cpu' is going away. splice any existing rq_list entries from this
 * software queue to the hw queue dispatch list, and ensure that it
 * gets run.
 */
static int blk_mq_hctx_notify_dead(unsigned int cpu, struct hlist_node *node)
{
	struct blk_mq_hw_ctx *hctx;
	struct blk_mq_ctx *ctx;
	LIST_HEAD(tmp);
	enum hctx_type type;

	hctx = hlist_entry_safe(node, struct blk_mq_hw_ctx, cpuhp_dead);
	if (!cpumask_test_cpu(cpu, hctx->cpumask))
		return 0;

	ctx = __blk_mq_get_ctx(hctx->queue, cpu);
	type = hctx->type;

	spin_lock(&ctx->lock);
	if (!list_empty(&ctx->rq_lists[type])) {
		list_splice_init(&ctx->rq_lists[type], &tmp);
		blk_mq_hctx_clear_pending(hctx, ctx);
	}
	spin_unlock(&ctx->lock);

	if (list_empty(&tmp))
		return 0;

	spin_lock(&hctx->lock);
	list_splice_tail_init(&tmp, &hctx->dispatch);
	spin_unlock(&hctx->lock);

	blk_mq_run_hw_queue(hctx, true);
	return 0;
}

static void blk_mq_remove_cpuhp(struct blk_mq_hw_ctx *hctx)
{
	if (!(hctx->flags & BLK_MQ_F_STACKING) && blk_mq_online >= 0)
		cpuhp_state_remove_instance_nocalls(blk_mq_online,
						    &hctx->cpuhp_online);
	cpuhp_state_remove_instance_nocalls(CPUHP_BLK_MQ_DEAD,
					    &hctx->cpuhp_dead);
}

/* hctx->ctxs will be freed in queue's release handler */
static void blk_mq_exit_hctx(struct request_queue *q,
		struct blk_mq_tag_set *set,
		struct blk_mq_hw_ctx *hctx, unsigned int hctx_idx)
{
	if (blk_mq_hw_queue_mapped(hctx))
		blk_mq_tag_idle(hctx);

	if (set->ops->exit_request)
		set->ops->exit_request(set, hctx->fq->flush_rq, hctx_idx);

	if (set->ops->exit_hctx)
		set->ops->exit_hctx(hctx, hctx_idx);

	blk_mq_remove_cpuhp(hctx);

	spin_lock(&q->unused_hctx_lock);
	list_add(&hctx->hctx_list, &q->unused_hctx_list);
	spin_unlock(&q->unused_hctx_lock);
}

static void blk_mq_exit_hw_queues(struct request_queue *q,
		struct blk_mq_tag_set *set, int nr_queue)
{
	struct blk_mq_hw_ctx *hctx;
	unsigned int i;

	queue_for_each_hw_ctx(q, hctx, i) {
		if (i == nr_queue)
			break;
		blk_mq_debugfs_unregister_hctx(hctx);
		blk_mq_exit_hctx(q, set, hctx, i);
	}
}

static int blk_mq_hw_ctx_size(struct blk_mq_tag_set *tag_set)
{
	int hw_ctx_size = sizeof(struct blk_mq_hw_ctx);

	BUILD_BUG_ON(ALIGN(offsetof(struct blk_mq_hw_ctx, srcu),
			   __alignof__(struct blk_mq_hw_ctx)) !=
		     sizeof(struct blk_mq_hw_ctx));

	if (tag_set->flags & BLK_MQ_F_BLOCKING)
		hw_ctx_size += sizeof(struct srcu_struct);

	return hw_ctx_size;
}

static int blk_mq_init_hctx(struct request_queue *q,
		struct blk_mq_tag_set *set,
		struct blk_mq_hw_ctx *hctx, unsigned hctx_idx)
{
	hctx->queue_num = hctx_idx;

	if (!(hctx->flags & BLK_MQ_F_STACKING) && blk_mq_online >= 0)
		cpuhp_state_add_instance_nocalls(blk_mq_online,
				&hctx->cpuhp_online);
	cpuhp_state_add_instance_nocalls(CPUHP_BLK_MQ_DEAD, &hctx->cpuhp_dead);

	hctx->tags = set->tags[hctx_idx];

	if (set->ops->init_hctx &&
	    set->ops->init_hctx(hctx, set->driver_data, hctx_idx))
		goto unregister_cpu_notifier;

	if (blk_mq_init_request(set, hctx->fq->flush_rq, hctx_idx,
				hctx->numa_node))
		goto exit_hctx;
	return 0;

 exit_hctx:
	if (set->ops->exit_hctx)
		set->ops->exit_hctx(hctx, hctx_idx);
 unregister_cpu_notifier:
	blk_mq_remove_cpuhp(hctx);
	return -1;
}

static struct blk_mq_hw_ctx *
blk_mq_alloc_hctx(struct request_queue *q, struct blk_mq_tag_set *set,
		int node)
{
	struct blk_mq_hw_ctx *hctx;
	gfp_t gfp = GFP_NOIO | __GFP_NOWARN | __GFP_NORETRY;

	hctx = kzalloc_node(blk_mq_hw_ctx_size(set), gfp, node);
	if (!hctx)
		goto fail_alloc_hctx;

	if (!zalloc_cpumask_var_node(&hctx->cpumask, gfp, node))
		goto free_hctx;

	atomic_set(&hctx->nr_active, 0);
	if (node == NUMA_NO_NODE)
		node = set->numa_node;
	hctx->numa_node = node;

	INIT_DELAYED_WORK(&hctx->run_work, blk_mq_run_work_fn);
	spin_lock_init(&hctx->lock);
	INIT_LIST_HEAD(&hctx->dispatch);
	hctx->queue = q;
	hctx->flags = set->flags & ~BLK_MQ_F_TAG_SHARED;

	INIT_LIST_HEAD(&hctx->hctx_list);

	/*
	 * Allocate space for all possible cpus to avoid allocation at
	 * runtime
	 */
	hctx->ctxs = kmalloc_array_node(nr_cpu_ids, sizeof(void *),
			gfp, node);
	if (!hctx->ctxs)
		goto free_cpumask;

	if (sbitmap_init_node(&hctx->ctx_map, nr_cpu_ids, ilog2(8),
				gfp, node))
		goto free_ctxs;
	hctx->nr_ctx = 0;

	spin_lock_init(&hctx->dispatch_wait_lock);
	init_waitqueue_func_entry(&hctx->dispatch_wait, blk_mq_dispatch_wake);
	INIT_LIST_HEAD(&hctx->dispatch_wait.entry);

	hctx->fq = blk_alloc_flush_queue(q, hctx->numa_node, set->cmd_size,
			gfp);
	if (!hctx->fq)
		goto free_bitmap;

	if (hctx->flags & BLK_MQ_F_BLOCKING)
		init_srcu_struct(hctx->srcu);
	blk_mq_hctx_kobj_init(hctx);

	return hctx;

 free_bitmap:
	sbitmap_free(&hctx->ctx_map);
 free_ctxs:
	kfree(hctx->ctxs);
 free_cpumask:
	free_cpumask_var(hctx->cpumask);
 free_hctx:
	kfree(hctx);
 fail_alloc_hctx:
	return NULL;
}

static void blk_mq_init_cpu_queues(struct request_queue *q,
				   unsigned int nr_hw_queues)
{
	struct blk_mq_tag_set *set = q->tag_set;
	unsigned int i, j;

	for_each_possible_cpu(i) {
		struct blk_mq_ctx *__ctx = per_cpu_ptr(q->queue_ctx, i);
		struct blk_mq_hw_ctx *hctx;
		int k;

		__ctx->cpu = i;
		spin_lock_init(&__ctx->lock);
		for (k = HCTX_TYPE_DEFAULT; k < HCTX_MAX_TYPES; k++)
			INIT_LIST_HEAD(&__ctx->rq_lists[k]);

		__ctx->queue = q;

		/*
		 * Set local node, IFF we have more than one hw queue. If
		 * not, we remain on the home node of the device
		 */
		for (j = 0; j < set->nr_maps; j++) {
			hctx = blk_mq_map_queue_type(q, j, i);
			if (nr_hw_queues > 1 && hctx->numa_node == NUMA_NO_NODE)
				hctx->numa_node = local_memory_node(cpu_to_node(i));
		}
	}
}

static bool __blk_mq_alloc_rq_map(struct blk_mq_tag_set *set, int hctx_idx)
{
	int ret = 0;

	set->tags[hctx_idx] = blk_mq_alloc_rq_map(set, hctx_idx,
					set->queue_depth, set->reserved_tags);
	if (!set->tags[hctx_idx])
		return false;

	ret = blk_mq_alloc_rqs(set, set->tags[hctx_idx], hctx_idx,
				set->queue_depth);
	if (!ret)
		return true;

	blk_mq_free_rq_map(set->tags[hctx_idx]);
	set->tags[hctx_idx] = NULL;
	return false;
}

static void blk_mq_free_map_and_requests(struct blk_mq_tag_set *set,
					 unsigned int hctx_idx)
{
	if (set->tags && set->tags[hctx_idx]) {
		blk_mq_free_rqs(set, set->tags[hctx_idx], hctx_idx);
		blk_mq_free_rq_map(set->tags[hctx_idx]);
		set->tags[hctx_idx] = NULL;
	}
}

static void blk_mq_map_swqueue(struct request_queue *q)
{
	unsigned int i, j, hctx_idx;
	struct blk_mq_hw_ctx *hctx;
	struct blk_mq_ctx *ctx;
	struct blk_mq_tag_set *set = q->tag_set;

	queue_for_each_hw_ctx(q, hctx, i) {
		cpumask_clear(hctx->cpumask);
		hctx->nr_ctx = 0;
		hctx->dispatch_from = NULL;
	}

	/*
	 * Map software to hardware queues.
	 *
	 * If the cpu isn't present, the cpu is mapped to first hctx.
	 */
	for_each_possible_cpu(i) {

		ctx = per_cpu_ptr(q->queue_ctx, i);
		for (j = 0; j < set->nr_maps; j++) {
			if (!set->map[j].nr_queues) {
				ctx->hctxs[j] = blk_mq_map_queue_type(q,
						HCTX_TYPE_DEFAULT, i);
				continue;
			}
			hctx_idx = set->map[j].mq_map[i];
			/* unmapped hw queue can be remapped after CPU topo changed */
			if (!set->tags[hctx_idx] &&
			    !__blk_mq_alloc_rq_map(set, hctx_idx)) {
				/*
				 * If tags initialization fail for some hctx,
				 * that hctx won't be brought online.  In this
				 * case, remap the current ctx to hctx[0] which
				 * is guaranteed to always have tags allocated
				 */
				set->map[j].mq_map[i] = 0;
			}

			hctx = blk_mq_map_queue_type(q, j, i);
			ctx->hctxs[j] = hctx;
			/*
			 * If the CPU is already set in the mask, then we've
			 * mapped this one already. This can happen if
			 * devices share queues across queue maps.
			 */
			if (cpumask_test_cpu(i, hctx->cpumask))
				continue;

			cpumask_set_cpu(i, hctx->cpumask);
			hctx->type = j;
			ctx->index_hw[hctx->type] = hctx->nr_ctx;
			hctx->ctxs[hctx->nr_ctx++] = ctx;

			/*
			 * If the nr_ctx type overflows, we have exceeded the
			 * amount of sw queues we can support.
			 */
			BUG_ON(!hctx->nr_ctx);
		}

		for (; j < HCTX_MAX_TYPES; j++)
			ctx->hctxs[j] = blk_mq_map_queue_type(q,
					HCTX_TYPE_DEFAULT, i);
	}

	queue_for_each_hw_ctx(q, hctx, i) {
		/*
		 * If no software queues are mapped to this hardware queue,
		 * disable it and free the request entries.
		 */
		if (!hctx->nr_ctx) {
			/* Never unmap queue 0.  We need it as a
			 * fallback in case of a new remap fails
			 * allocation
			 */
			if (i && set->tags[i])
				blk_mq_free_map_and_requests(set, i);

			hctx->tags = NULL;
			continue;
		}

		hctx->tags = set->tags[i];
		WARN_ON(!hctx->tags);

		/*
		 * Set the map size to the number of mapped software queues.
		 * This is more accurate and more efficient than looping
		 * over all possibly mapped software queues.
		 */
		sbitmap_resize(&hctx->ctx_map, hctx->nr_ctx);

		/*
		 * Initialize batch roundrobin counts
		 */
		hctx->next_cpu = blk_mq_first_mapped_cpu(hctx);
		hctx->next_cpu_batch = BLK_MQ_CPU_WORK_BATCH;
	}
}

/*
 * Caller needs to ensure that we're either frozen/quiesced, or that
 * the queue isn't live yet.
 */
static void queue_set_hctx_shared(struct request_queue *q, bool shared)
{
	struct blk_mq_hw_ctx *hctx;
	int i;

	queue_for_each_hw_ctx(q, hctx, i) {
		if (shared)
			hctx->flags |= BLK_MQ_F_TAG_SHARED;
		else
			hctx->flags &= ~BLK_MQ_F_TAG_SHARED;
	}
}

static void blk_mq_update_tag_set_depth(struct blk_mq_tag_set *set,
					bool shared)
{
	struct request_queue *q;

	lockdep_assert_held(&set->tag_list_lock);

	list_for_each_entry(q, &set->tag_list, tag_set_list) {
		blk_mq_freeze_queue(q);
		queue_set_hctx_shared(q, shared);
		blk_mq_unfreeze_queue(q);
	}
}

static void blk_mq_del_queue_tag_set(struct request_queue *q)
{
	struct blk_mq_tag_set *set = q->tag_set;

	mutex_lock(&set->tag_list_lock);
	list_del_rcu(&q->tag_set_list);
	if (list_is_singular(&set->tag_list)) {
		/* just transitioned to unshared */
		set->flags &= ~BLK_MQ_F_TAG_SHARED;
		/* update existing queue */
		blk_mq_update_tag_set_depth(set, false);
	}
	mutex_unlock(&set->tag_list_lock);
	INIT_LIST_HEAD(&q->tag_set_list);
}

static void blk_mq_add_queue_tag_set(struct blk_mq_tag_set *set,
				     struct request_queue *q)
{
	mutex_lock(&set->tag_list_lock);

	/*
	 * Check to see if we're transitioning to shared (from 1 to 2 queues).
	 */
	if (!list_empty(&set->tag_list) &&
	    !(set->flags & BLK_MQ_F_TAG_SHARED)) {
		set->flags |= BLK_MQ_F_TAG_SHARED;
		/* update existing queue */
		blk_mq_update_tag_set_depth(set, true);
	}
	if (set->flags & BLK_MQ_F_TAG_SHARED)
		queue_set_hctx_shared(q, true);
	list_add_tail_rcu(&q->tag_set_list, &set->tag_list);

	mutex_unlock(&set->tag_list_lock);
}

/* All allocations will be freed in release handler of q->mq_kobj */
static int blk_mq_alloc_ctxs(struct request_queue *q)
{
	struct blk_mq_ctxs *ctxs;
	int cpu;

	ctxs = kzalloc(sizeof(*ctxs), GFP_KERNEL);
	if (!ctxs)
		return -ENOMEM;

	ctxs->queue_ctx = alloc_percpu(struct blk_mq_ctx);
	if (!ctxs->queue_ctx)
		goto fail;

	for_each_possible_cpu(cpu) {
		struct blk_mq_ctx *ctx = per_cpu_ptr(ctxs->queue_ctx, cpu);
		ctx->ctxs = ctxs;
	}

	q->mq_kobj = &ctxs->kobj;
	q->queue_ctx = ctxs->queue_ctx;

	return 0;
 fail:
	kfree(ctxs);
	return -ENOMEM;
}

/*
 * It is the actual release handler for mq, but we do it from
 * request queue's release handler for avoiding use-after-free
 * and headache because q->mq_kobj shouldn't have been introduced,
 * but we can't group ctx/kctx kobj without it.
 */
void blk_mq_release(struct request_queue *q)
{
	struct blk_mq_hw_ctx *hctx, *next;
	int i;

	queue_for_each_hw_ctx(q, hctx, i)
		WARN_ON_ONCE(hctx && list_empty(&hctx->hctx_list));

	/* all hctx are in .unused_hctx_list now */
	list_for_each_entry_safe(hctx, next, &q->unused_hctx_list, hctx_list) {
		list_del_init(&hctx->hctx_list);
		kobject_put(&hctx->kobj);
	}

	kfree(q->queue_hw_ctx);

	/*
	 * release .mq_kobj and sw queue's kobject now because
	 * both share lifetime with request queue.
	 */
	blk_mq_sysfs_deinit(q);
}

struct request_queue *blk_mq_init_queue(struct blk_mq_tag_set *set)
{
	struct request_queue *uninit_q, *q;

	uninit_q = blk_alloc_queue_node(GFP_KERNEL, set->numa_node);
	if (!uninit_q)
		return ERR_PTR(-ENOMEM);

	/*
	 * Initialize the queue without an elevator. device_add_disk() will do
	 * the initialization.
	 */
	q = blk_mq_init_allocated_queue(set, uninit_q, false);
	if (IS_ERR(q))
		blk_cleanup_queue(uninit_q);

	return q;
}
EXPORT_SYMBOL(blk_mq_init_queue);

/*
 * Helper for setting up a queue with mq ops, given queue depth, and
 * the passed in mq ops flags.
 */
struct request_queue *blk_mq_init_sq_queue(struct blk_mq_tag_set *set,
					   const struct blk_mq_ops *ops,
					   unsigned int queue_depth,
					   unsigned int set_flags)
{
	struct request_queue *q;
	int ret;

	memset(set, 0, sizeof(*set));
	set->ops = ops;
	set->nr_hw_queues = 1;
	set->nr_maps = 1;
	set->queue_depth = queue_depth;
	set->numa_node = NUMA_NO_NODE;
	set->flags = set_flags;

	ret = blk_mq_alloc_tag_set(set);
	if (ret)
		return ERR_PTR(ret);

	q = blk_mq_init_queue(set);
	if (IS_ERR(q)) {
		blk_mq_free_tag_set(set);
		return q;
	}

	return q;
}
EXPORT_SYMBOL(blk_mq_init_sq_queue);

static struct blk_mq_hw_ctx *blk_mq_alloc_and_init_hctx(
		struct blk_mq_tag_set *set, struct request_queue *q,
		int hctx_idx, int node)
{
	struct blk_mq_hw_ctx *hctx = NULL, *tmp;

	/* reuse dead hctx first */
	spin_lock(&q->unused_hctx_lock);
	list_for_each_entry(tmp, &q->unused_hctx_list, hctx_list) {
		if (tmp->numa_node == node) {
			hctx = tmp;
			break;
		}
	}
	if (hctx)
		list_del_init(&hctx->hctx_list);
	spin_unlock(&q->unused_hctx_lock);

	if (!hctx)
		hctx = blk_mq_alloc_hctx(q, set, node);
	if (!hctx)
		goto fail;

	if (blk_mq_init_hctx(q, set, hctx, hctx_idx))
		goto free_hctx;

	return hctx;

 free_hctx:
	kobject_put(&hctx->kobj);
 fail:
	return NULL;
}

static void blk_mq_realloc_hw_ctxs(struct blk_mq_tag_set *set,
						struct request_queue *q)
{
	int i, j, end;
	struct blk_mq_hw_ctx **hctxs = q->queue_hw_ctx;

	if (q->nr_hw_queues < set->nr_hw_queues) {
		struct blk_mq_hw_ctx **new_hctxs;

		new_hctxs = kcalloc_node(set->nr_hw_queues,
				       sizeof(*new_hctxs), GFP_KERNEL,
				       set->numa_node);
		if (!new_hctxs)
			return;
		if (hctxs)
			memcpy(new_hctxs, hctxs, q->nr_hw_queues *
			       sizeof(*hctxs));
		q->queue_hw_ctx = new_hctxs;
		kfree(hctxs);
		hctxs = new_hctxs;
	}

	/* protect against switching io scheduler  */
	mutex_lock(&q->sysfs_lock);
	for (i = 0; i < set->nr_hw_queues; i++) {
		int node;
		struct blk_mq_hw_ctx *hctx;

		node = blk_mq_hw_queue_to_node(&set->map[HCTX_TYPE_DEFAULT], i);
		/*
		 * If the hw queue has been mapped to another numa node,
		 * we need to realloc the hctx. If allocation fails, fallback
		 * to use the previous one.
		 */
		if (hctxs[i] && (hctxs[i]->numa_node == node))
			continue;

		hctx = blk_mq_alloc_and_init_hctx(set, q, i, node);
		if (hctx) {
			if (hctxs[i])
				blk_mq_exit_hctx(q, set, hctxs[i], i);
			hctxs[i] = hctx;
		} else {
			if (hctxs[i])
				pr_warn("Allocate new hctx on node %d fails,\
						fallback to previous one on node %d\n",
						node, hctxs[i]->numa_node);
			else
				break;
		}
	}
	/*
	 * Increasing nr_hw_queues fails. Free the newly allocated
	 * hctxs and keep the previous q->nr_hw_queues.
	 */
	if (i != set->nr_hw_queues) {
		j = q->nr_hw_queues;
		end = i;
	} else {
		j = i;
		end = q->nr_hw_queues;
		q->nr_hw_queues = set->nr_hw_queues;
	}

	for (; j < end; j++) {
		struct blk_mq_hw_ctx *hctx = hctxs[j];

		if (hctx) {
			if (hctx->tags)
				blk_mq_free_map_and_requests(set, j);
			blk_mq_exit_hctx(q, set, hctx, j);
			hctxs[j] = NULL;
		}
	}
	mutex_unlock(&q->sysfs_lock);
}

struct request_queue *blk_mq_init_allocated_queue(struct blk_mq_tag_set *set,
						  struct request_queue *q,
						  bool elevator_init)
{
	/* mark the queue as mq asap */
	q->mq_ops = set->ops;

	q->poll_cb = blk_stat_alloc_callback(blk_mq_poll_stats_fn,
					     blk_mq_poll_stats_bkt,
					     BLK_MQ_POLL_STATS_BKTS, q);
	if (!q->poll_cb)
		goto err_exit;

	if (blk_mq_alloc_ctxs(q))
		goto err_exit;

	/* init q->mq_kobj and sw queues' kobjects */
	blk_mq_sysfs_init(q);

	INIT_LIST_HEAD(&q->unused_hctx_list);
	spin_lock_init(&q->unused_hctx_lock);

	blk_mq_realloc_hw_ctxs(set, q);
	if (!q->nr_hw_queues)
		goto err_hctxs;

	INIT_WORK(&q->timeout_work, blk_mq_timeout_work);
	blk_queue_rq_timeout(q, set->timeout ? set->timeout : 30 * HZ);

	q->tag_set = set;

	q->queue_flags |= QUEUE_FLAG_MQ_DEFAULT;
	if (set->nr_maps > HCTX_TYPE_POLL &&
	    set->map[HCTX_TYPE_POLL].nr_queues)
		blk_queue_flag_set(QUEUE_FLAG_POLL, q);

	q->sg_reserved_size = INT_MAX;

	INIT_DELAYED_WORK(&q->requeue_work, blk_mq_requeue_work);
	INIT_LIST_HEAD(&q->requeue_list);
	spin_lock_init(&q->requeue_lock);

	blk_queue_make_request(q, blk_mq_make_request);

	/*
	 * Do this after blk_queue_make_request() overrides it...
	 */
	q->nr_requests = set->queue_depth;

	/*
	 * Default to classic polling
	 */
	q->poll_nsec = BLK_MQ_POLL_CLASSIC;

	blk_mq_init_cpu_queues(q, set->nr_hw_queues);
	blk_mq_add_queue_tag_set(set, q);
	blk_mq_map_swqueue(q);

	if (elevator_init)
		elevator_init_mq(q);

	return q;

err_hctxs:
	kfree(q->queue_hw_ctx);
	q->nr_hw_queues = 0;
	blk_mq_sysfs_deinit(q);
err_exit:
	q->mq_ops = NULL;
	return ERR_PTR(-ENOMEM);
}
EXPORT_SYMBOL(blk_mq_init_allocated_queue);

/* tags can _not_ be used after returning from blk_mq_exit_queue */
void blk_mq_exit_queue(struct request_queue *q)
{
	struct blk_mq_tag_set	*set = q->tag_set;

	blk_mq_del_queue_tag_set(q);
	blk_mq_exit_hw_queues(q, set, set->nr_hw_queues);
}

static int __blk_mq_alloc_rq_maps(struct blk_mq_tag_set *set)
{
	int i;

	for (i = 0; i < set->nr_hw_queues; i++)
		if (!__blk_mq_alloc_rq_map(set, i))
			goto out_unwind;

	return 0;

out_unwind:
	while (--i >= 0)
		blk_mq_free_map_and_requests(set, i);

	return -ENOMEM;
}

/*
 * Allocate the request maps associated with this tag_set. Note that this
 * may reduce the depth asked for, if memory is tight. set->queue_depth
 * will be updated to reflect the allocated depth.
 */
static int blk_mq_alloc_rq_maps(struct blk_mq_tag_set *set)
{
	unsigned int depth;
	int err;

	depth = set->queue_depth;
	do {
		err = __blk_mq_alloc_rq_maps(set);
		if (!err)
			break;

		set->queue_depth >>= 1;
		if (set->queue_depth < set->reserved_tags + BLK_MQ_TAG_MIN) {
			err = -ENOMEM;
			break;
		}
	} while (set->queue_depth);

	if (!set->queue_depth || err) {
		pr_err("blk-mq: failed to allocate request map\n");
		return -ENOMEM;
	}

	if (depth != set->queue_depth)
		pr_info("blk-mq: reduced tag depth (%u -> %u)\n",
						depth, set->queue_depth);

	return 0;
}

static int blk_mq_update_queue_map(struct blk_mq_tag_set *set)
{
	/*
	 * blk_mq_map_queues() and multiple .map_queues() implementations
	 * expect that set->map[HCTX_TYPE_DEFAULT].nr_queues is set to the
	 * number of hardware queues.
	 */
	if (set->nr_maps == 1)
		set->map[HCTX_TYPE_DEFAULT].nr_queues = set->nr_hw_queues;

	if (set->ops->map_queues && !is_kdump_kernel()) {
		int i;

		/*
		 * transport .map_queues is usually done in the following
		 * way:
		 *
		 * for (queue = 0; queue < set->nr_hw_queues; queue++) {
		 * 	mask = get_cpu_mask(queue)
		 * 	for_each_cpu(cpu, mask)
		 * 		set->map[x].mq_map[cpu] = queue;
		 * }
		 *
		 * When we need to remap, the table has to be cleared for
		 * killing stale mapping since one CPU may not be mapped
		 * to any hw queue.
		 */
		for (i = 0; i < set->nr_maps; i++)
			blk_mq_clear_mq_map(&set->map[i]);

		return set->ops->map_queues(set);
	} else {
		BUG_ON(set->nr_maps > 1);
		return blk_mq_map_queues(&set->map[HCTX_TYPE_DEFAULT]);
	}
}

static int blk_mq_realloc_tag_set_tags(struct blk_mq_tag_set *set,
				  int cur_nr_hw_queues, int new_nr_hw_queues)
{
	struct blk_mq_tags **new_tags;

	if (cur_nr_hw_queues >= new_nr_hw_queues)
		return 0;

	new_tags = kcalloc_node(new_nr_hw_queues, sizeof(struct blk_mq_tags *),
				GFP_KERNEL, set->numa_node);
	if (!new_tags)
		return -ENOMEM;

	if (set->tags)
		memcpy(new_tags, set->tags, cur_nr_hw_queues *
		       sizeof(*set->tags));
	kfree(set->tags);
	set->tags = new_tags;
	set->nr_hw_queues = new_nr_hw_queues;

	return 0;
}

/*
 * Alloc a tag set to be associated with one or more request queues.
 * May fail with EINVAL for various error conditions. May adjust the
 * requested depth down, if it's too large. In that case, the set
 * value will be stored in set->queue_depth.
 */
int blk_mq_alloc_tag_set(struct blk_mq_tag_set *set)
{
	int i, ret;

	BUILD_BUG_ON(BLK_MQ_MAX_DEPTH > 1 << BLK_MQ_UNIQUE_TAG_BITS);

	if (!set->nr_hw_queues)
		return -EINVAL;
	if (!set->queue_depth)
		return -EINVAL;
	if (set->queue_depth < set->reserved_tags + BLK_MQ_TAG_MIN)
		return -EINVAL;

	if (!set->ops->queue_rq)
		return -EINVAL;

	if (!set->ops->get_budget ^ !set->ops->put_budget)
		return -EINVAL;

	if (set->queue_depth > BLK_MQ_MAX_DEPTH) {
		pr_info("blk-mq: reduced tag depth to %u\n",
			BLK_MQ_MAX_DEPTH);
		set->queue_depth = BLK_MQ_MAX_DEPTH;
	}

	if (!set->nr_maps)
		set->nr_maps = 1;
	else if (set->nr_maps > HCTX_MAX_TYPES)
		return -EINVAL;

	/*
	 * If a crashdump is active, then we are potentially in a very
	 * memory constrained environment. Limit us to 1 queue and
	 * 64 tags to prevent using too much memory.
	 */
	if (is_kdump_kernel()) {
		set->nr_hw_queues = 1;
		set->nr_maps = 1;
		set->queue_depth = min(64U, set->queue_depth);
	}
	/*
	 * There is no use for more h/w queues than cpus if we just have
	 * a single map
	 */
	if (set->nr_maps == 1 && set->nr_hw_queues > nr_cpu_ids)
		set->nr_hw_queues = nr_cpu_ids;

	if (blk_mq_realloc_tag_set_tags(set, 0, set->nr_hw_queues) < 0)
		return -ENOMEM;

	ret = -ENOMEM;
	for (i = 0; i < set->nr_maps; i++) {
		set->map[i].mq_map = kcalloc_node(nr_cpu_ids,
						  sizeof(struct blk_mq_queue_map),
						  GFP_KERNEL, set->numa_node);
		if (!set->map[i].mq_map)
			goto out_free_mq_map;
		set->map[i].nr_queues = is_kdump_kernel() ? 1 : set->nr_hw_queues;
	}

	ret = blk_mq_update_queue_map(set);
	if (ret)
		goto out_free_mq_map;

	ret = blk_mq_alloc_rq_maps(set);
	if (ret)
		goto out_free_mq_map;

	mutex_init(&set->tag_list_lock);
	INIT_LIST_HEAD(&set->tag_list);

	return 0;

out_free_mq_map:
	for (i = 0; i < set->nr_maps; i++) {
		kfree(set->map[i].mq_map);
		set->map[i].mq_map = NULL;
	}
	kfree(set->tags);
	set->tags = NULL;
	return ret;
}
EXPORT_SYMBOL(blk_mq_alloc_tag_set);

void blk_mq_free_tag_set(struct blk_mq_tag_set *set)
{
	int i, j;

	for (i = 0; i < set->nr_hw_queues; i++)
		blk_mq_free_map_and_requests(set, i);

	for (j = 0; j < set->nr_maps; j++) {
		kfree(set->map[j].mq_map);
		set->map[j].mq_map = NULL;
	}

	kfree(set->tags);
	set->tags = NULL;
}
EXPORT_SYMBOL(blk_mq_free_tag_set);

int blk_mq_update_nr_requests(struct request_queue *q, unsigned int nr)
{
	struct blk_mq_tag_set *set = q->tag_set;
	struct blk_mq_hw_ctx *hctx;
	int i, ret;

	if (!set)
		return -EINVAL;

	if (q->nr_requests == nr)
		return 0;

	blk_mq_freeze_queue(q);
	blk_mq_quiesce_queue(q);

	ret = 0;
	queue_for_each_hw_ctx(q, hctx, i) {
		if (!hctx->tags)
			continue;
		/*
		 * If we're using an MQ scheduler, just update the scheduler
		 * queue depth. This is similar to what the old code would do.
		 */
		if (!hctx->sched_tags) {
			ret = blk_mq_tag_update_depth(hctx, &hctx->tags, nr,
							false);
		} else {
			ret = blk_mq_tag_update_depth(hctx, &hctx->sched_tags,
							nr, true);
		}
		if (ret)
			break;
		if (q->elevator && q->elevator->type->ops.depth_updated)
			q->elevator->type->ops.depth_updated(hctx);
	}

	if (!ret)
		q->nr_requests = nr;

	blk_mq_unquiesce_queue(q);
	blk_mq_unfreeze_queue(q);

	return ret;
}

/*
 * request_queue and elevator_type pair.
 * It is just used by __blk_mq_update_nr_hw_queues to cache
 * the elevator_type associated with a request_queue.
 */
struct blk_mq_qe_pair {
	struct list_head node;
	struct request_queue *q;
	struct elevator_type *type;
};

/*
 * Cache the elevator_type in qe pair list and switch the
 * io scheduler to 'none'
 */
static bool blk_mq_elv_switch_none(struct list_head *head,
		struct request_queue *q)
{
	struct blk_mq_qe_pair *qe;

	if (!q->elevator)
		return true;

	qe = kmalloc(sizeof(*qe), GFP_NOIO | __GFP_NOWARN | __GFP_NORETRY);
	if (!qe)
		return false;

	INIT_LIST_HEAD(&qe->node);
	qe->q = q;
	qe->type = q->elevator->type;
	list_add(&qe->node, head);

	mutex_lock(&q->sysfs_lock);
	/*
	 * After elevator_switch_mq, the previous elevator_queue will be
	 * released by elevator_release. The reference of the io scheduler
	 * module get by elevator_get will also be put. So we need to get
	 * a reference of the io scheduler module here to prevent it to be
	 * removed.
	 */
	__module_get(qe->type->elevator_owner);
	elevator_switch_mq(q, NULL);
	mutex_unlock(&q->sysfs_lock);

	return true;
}

static void blk_mq_elv_switch_back(struct list_head *head,
		struct request_queue *q)
{
	struct blk_mq_qe_pair *qe;
	struct elevator_type *t = NULL;

	list_for_each_entry(qe, head, node)
		if (qe->q == q) {
			t = qe->type;
			break;
		}

	if (!t)
		return;

	list_del(&qe->node);
	kfree(qe);

	mutex_lock(&q->sysfs_lock);
	elevator_switch_mq(q, t);
	mutex_unlock(&q->sysfs_lock);
}

static void __blk_mq_update_nr_hw_queues(struct blk_mq_tag_set *set,
							int nr_hw_queues)
{
	struct request_queue *q;
	LIST_HEAD(head);
	int prev_nr_hw_queues;

	lockdep_assert_held(&set->tag_list_lock);

	if (set->nr_maps == 1 && nr_hw_queues > nr_cpu_ids)
		nr_hw_queues = nr_cpu_ids;
	if (nr_hw_queues < 1 || nr_hw_queues == set->nr_hw_queues)
		return;

	list_for_each_entry(q, &set->tag_list, tag_set_list)
		blk_mq_freeze_queue(q);
	/*
	 * Switch IO scheduler to 'none', cleaning up the data associated
	 * with the previous scheduler. We will switch back once we are done
	 * updating the new sw to hw queue mappings.
	 */
	list_for_each_entry(q, &set->tag_list, tag_set_list)
		if (!blk_mq_elv_switch_none(&head, q))
			goto switch_back;

	list_for_each_entry(q, &set->tag_list, tag_set_list) {
		blk_mq_debugfs_unregister_hctxs(q);
		blk_mq_sysfs_unregister(q);
	}

	prev_nr_hw_queues = set->nr_hw_queues;
	if (blk_mq_realloc_tag_set_tags(set, set->nr_hw_queues, nr_hw_queues) <
	    0)
		goto reregister;

	set->nr_hw_queues = nr_hw_queues;
fallback:
	blk_mq_update_queue_map(set);
	list_for_each_entry(q, &set->tag_list, tag_set_list) {
		blk_mq_realloc_hw_ctxs(set, q);
		if (q->nr_hw_queues != set->nr_hw_queues) {
			pr_warn("Increasing nr_hw_queues to %d fails, fallback to %d\n",
					nr_hw_queues, prev_nr_hw_queues);
			set->nr_hw_queues = prev_nr_hw_queues;
			blk_mq_map_queues(&set->map[HCTX_TYPE_DEFAULT]);
			goto fallback;
		}
		blk_mq_map_swqueue(q);
	}

reregister:
	list_for_each_entry(q, &set->tag_list, tag_set_list) {
		blk_mq_sysfs_register(q);
		blk_mq_debugfs_register_hctxs(q);
	}

switch_back:
	list_for_each_entry(q, &set->tag_list, tag_set_list)
		blk_mq_elv_switch_back(&head, q);

	list_for_each_entry(q, &set->tag_list, tag_set_list)
		blk_mq_unfreeze_queue(q);
}

void blk_mq_update_nr_hw_queues(struct blk_mq_tag_set *set, int nr_hw_queues)
{
	mutex_lock(&set->tag_list_lock);
	__blk_mq_update_nr_hw_queues(set, nr_hw_queues);
	mutex_unlock(&set->tag_list_lock);
}
EXPORT_SYMBOL_GPL(blk_mq_update_nr_hw_queues);

/* Enable polling stats and return whether they were already enabled. */
static bool blk_poll_stats_enable(struct request_queue *q)
{
	if (test_bit(QUEUE_FLAG_POLL_STATS, &q->queue_flags) ||
	    blk_queue_flag_test_and_set(QUEUE_FLAG_POLL_STATS, q))
		return true;
	blk_stat_add_callback(q, q->poll_cb);
	return false;
}

static void blk_mq_poll_stats_start(struct request_queue *q)
{
	/*
	 * We don't arm the callback if polling stats are not enabled or the
	 * callback is already active.
	 */
	if (!test_bit(QUEUE_FLAG_POLL_STATS, &q->queue_flags) ||
	    blk_stat_is_active(q->poll_cb))
		return;

	blk_stat_activate_msecs(q->poll_cb, 100);
}

static void blk_mq_poll_stats_fn(struct blk_stat_callback *cb)
{
	struct request_queue *q = cb->data;
	int bucket;

	for (bucket = 0; bucket < BLK_MQ_POLL_STATS_BKTS; bucket++) {
		if (cb->stat[bucket].nr_samples)
			q->poll_stat[bucket] = cb->stat[bucket];
	}
}

static unsigned long blk_mq_poll_nsecs(struct request_queue *q,
				       struct request *rq)
{
	unsigned long ret = 0;
	int bucket;

	/*
	 * If stats collection isn't on, don't sleep but turn it on for
	 * future users
	 */
	if (!blk_poll_stats_enable(q))
		return 0;

	/*
	 * As an optimistic guess, use half of the mean service time
	 * for this type of request. We can (and should) make this smarter.
	 * For instance, if the completion latencies are tight, we can
	 * get closer than just half the mean. This is especially
	 * important on devices where the completion latencies are longer
	 * than ~10 usec. We do use the stats for the relevant IO size
	 * if available which does lead to better estimates.
	 */
	bucket = blk_mq_poll_stats_bkt(rq);
	if (bucket < 0)
		return ret;

	if (q->poll_stat[bucket].nr_samples)
		ret = (q->poll_stat[bucket].mean + 1) / 2;

	return ret;
}

static bool blk_mq_poll_hybrid_sleep(struct request_queue *q,
				     struct request *rq)
{
	struct hrtimer_sleeper hs;
	enum hrtimer_mode mode;
	unsigned int nsecs;
	ktime_t kt;

	if (rq->rq_flags & RQF_MQ_POLL_SLEPT)
		return false;

	/*
	 * If we get here, hybrid polling is enabled. Hence poll_nsec can be:
	 *
	 *  0:	use half of prev avg
	 * >0:	use this specific value
	 */
	if (q->poll_nsec > 0)
		nsecs = q->poll_nsec;
	else
		nsecs = blk_mq_poll_nsecs(q, rq);

	if (!nsecs)
		return false;

	rq->rq_flags |= RQF_MQ_POLL_SLEPT;

	/*
	 * This will be replaced with the stats tracking code, using
	 * 'avg_completion_time / 2' as the pre-sleep target.
	 */
	kt = nsecs;

	mode = HRTIMER_MODE_REL;
	hrtimer_init_sleeper_on_stack(&hs, CLOCK_MONOTONIC, mode);
	hrtimer_set_expires(&hs.timer, kt);

	do {
		if (blk_mq_rq_state(rq) == MQ_RQ_COMPLETE)
			break;
		set_current_state(TASK_UNINTERRUPTIBLE);
		hrtimer_sleeper_start_expires(&hs, mode);
		if (hs.task)
			io_schedule();
		hrtimer_cancel(&hs.timer);
		mode = HRTIMER_MODE_ABS;
	} while (hs.task && !signal_pending(current));

	__set_current_state(TASK_RUNNING);
	destroy_hrtimer_on_stack(&hs.timer);
	return true;
}

static bool blk_mq_poll_hybrid(struct request_queue *q,
			       struct blk_mq_hw_ctx *hctx, blk_qc_t cookie)
{
	struct request *rq;

	if (q->poll_nsec == BLK_MQ_POLL_CLASSIC)
		return false;

	if (!blk_qc_t_is_internal(cookie))
		rq = blk_mq_tag_to_rq(hctx->tags, blk_qc_t_to_tag(cookie));
	else {
		rq = blk_mq_tag_to_rq(hctx->sched_tags, blk_qc_t_to_tag(cookie));
		/*
		 * With scheduling, if the request has completed, we'll
		 * get a NULL return here, as we clear the sched tag when
		 * that happens. The request still remains valid, like always,
		 * so we should be safe with just the NULL check.
		 */
		if (!rq)
			return false;
	}

	return blk_mq_poll_hybrid_sleep(q, rq);
}

/**
 * blk_poll - poll for IO completions
 * @q:  the queue
 * @cookie: cookie passed back at IO submission time
 * @spin: whether to spin for completions
 *
 * Description:
 *    Poll for completions on the passed in queue. Returns number of
 *    completed entries found. If @spin is true, then blk_poll will continue
 *    looping until at least one completion is found, unless the task is
 *    otherwise marked running (or we need to reschedule).
 */
int blk_poll(struct request_queue *q, blk_qc_t cookie, bool spin)
{
	struct blk_mq_hw_ctx *hctx;
	long state;

	if (!blk_qc_t_valid(cookie) ||
	    !test_bit(QUEUE_FLAG_POLL, &q->queue_flags))
		return 0;

	if (current->plug)
		blk_flush_plug_list(current->plug, false);

	hctx = q->queue_hw_ctx[blk_qc_t_to_queue_num(cookie)];

	/*
	 * If we sleep, have the caller restart the poll loop to reset
	 * the state. Like for the other success return cases, the
	 * caller is responsible for checking if the IO completed. If
	 * the IO isn't complete, we'll get called again and will go
	 * straight to the busy poll loop.
	 */
	if (blk_mq_poll_hybrid(q, hctx, cookie))
		return 1;

	hctx->poll_considered++;

	state = current->state;
	do {
		int ret;

		hctx->poll_invoked++;

		ret = q->mq_ops->poll(hctx);
		if (ret > 0) {
			hctx->poll_success++;
			__set_current_state(TASK_RUNNING);
			return ret;
		}

		if (signal_pending_state(state, current))
			__set_current_state(TASK_RUNNING);

		if (current->state == TASK_RUNNING)
			return 1;
		if (ret < 0 || !spin)
			break;
		cpu_relax();
	} while (!need_resched());

	__set_current_state(TASK_RUNNING);
	return 0;
}
EXPORT_SYMBOL_GPL(blk_poll);

unsigned int blk_mq_rq_cpu(struct request *rq)
{
	return rq->mq_ctx->cpu;
}
EXPORT_SYMBOL(blk_mq_rq_cpu);

static int __init blk_mq_init(void)
{
	cpuhp_setup_state_multi(CPUHP_BLK_MQ_DEAD, "block/mq:dead", NULL,
				blk_mq_hctx_notify_dead);
	blk_mq_online = cpuhp_setup_state_multi(CPUHP_AP_ONLINE_DYN, "block/mq:online",
				blk_mq_hctx_notify_online,
				blk_mq_hctx_notify_offline);
	return 0;
}
subsys_initcall(blk_mq_init);
