/*
 * blk-mq scheduling framework
 *
 * Copyright (C) 2016 Jens Axboe
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/blk-mq.h>

#include <trace/events/block.h>

#include "blk.h"
#include "blk-mq.h"
#include "blk-mq-debugfs.h"
#include "blk-mq-sched.h"
#include "blk-mq-tag.h"
#include "blk-wbt.h"

void blk_mq_sched_free_hctx_data(struct request_queue *q,
				 void (*exit)(struct blk_mq_hw_ctx *))
{
	struct blk_mq_hw_ctx *hctx;
	int i;

	queue_for_each_hw_ctx(q, hctx, i) {
		if (exit && hctx->sched_data)
			exit(hctx);
		kfree(hctx->sched_data);
		hctx->sched_data = NULL;
	}
}
EXPORT_SYMBOL_GPL(blk_mq_sched_free_hctx_data);

void blk_mq_sched_assign_ioc(struct request *rq)
{
	struct request_queue *q = rq->q;
	struct io_context *ioc;
	struct io_cq *icq;

	/*
	 * May not have an IO context if it's a passthrough request
	 */
	ioc = current->io_context;
	if (!ioc)
		return;

	spin_lock_irq(&q->queue_lock);
	icq = ioc_lookup_icq(ioc, q);
	spin_unlock_irq(&q->queue_lock);

	if (!icq) {
		icq = ioc_create_icq(ioc, q, GFP_ATOMIC);
		if (!icq)
			return;
	}
	get_io_context(icq->ioc);
	rq->elv.icq = icq;
}

/*
 * Mark a hardware queue as needing a restart. For shared queues, maintain
 * a count of how many hardware queues are marked for restart.
 */
void blk_mq_sched_mark_restart_hctx(struct blk_mq_hw_ctx *hctx)
{
	if (test_bit(BLK_MQ_S_SCHED_RESTART, &hctx->state))
		return;

	set_bit(BLK_MQ_S_SCHED_RESTART, &hctx->state);
}
EXPORT_SYMBOL_GPL(blk_mq_sched_mark_restart_hctx);

void blk_mq_sched_restart(struct blk_mq_hw_ctx *hctx)
{
	if (!test_bit(BLK_MQ_S_SCHED_RESTART, &hctx->state))
		return;
	clear_bit(BLK_MQ_S_SCHED_RESTART, &hctx->state);

	/*
	 * Order clearing SCHED_RESTART and list_empty_careful(&hctx->dispatch)
	 * in blk_mq_run_hw_queue(). Its pair is the barrier in
	 * blk_mq_dispatch_rq_list(). So dispatch code won't see SCHED_RESTART,
	 * meantime new request added to hctx->dispatch is missed to check in
	 * blk_mq_run_hw_queue().
	 */
	smp_mb();

	blk_mq_run_hw_queue(hctx, true);
}

#define BLK_MQ_BUDGET_DELAY	3		/* ms units */

/*
 * Only SCSI implements .get_budget and .put_budget, and SCSI restarts
 * its queue by itself in its completion handler, so we don't need to
 * restart queue if .get_budget() returns BLK_STS_NO_RESOURCE.
 *
 * Returns -EAGAIN if hctx->dispatch was found non-empty and run_work has to
 * be run again.  This is necessary to avoid starving flushes.
 */
static int blk_mq_do_dispatch_sched(struct blk_mq_hw_ctx *hctx)
{
	struct request_queue *q = hctx->queue;
	struct elevator_queue *e = q->elevator;
	LIST_HEAD(rq_list);
	int ret = 0;

	do {
		struct request *rq;

		if (e->type->ops.has_work && !e->type->ops.has_work(hctx))
			break;

		if (!list_empty_careful(&hctx->dispatch)) {
			ret = -EAGAIN;
			break;
		}

		if (!blk_mq_get_dispatch_budget(hctx))
			break;

		rq = e->type->ops.dispatch_request(hctx);
		if (!rq) {
			blk_mq_put_dispatch_budget(hctx);
			/*
			 * We're releasing without dispatching. Holding the
			 * budget could have blocked any "hctx"s with the
			 * same queue and if we didn't dispatch then there's
			 * no guarantee anyone will kick the queue.  Kick it
			 * ourselves.
			 */
			blk_mq_delay_run_hw_queues(q, BLK_MQ_BUDGET_DELAY);
			break;
		}

		/*
		 * Now this rq owns the budget which has to be released
		 * if this rq won't be queued to driver via .queue_rq()
		 * in blk_mq_dispatch_rq_list().
		 */
		list_add(&rq->queuelist, &rq_list);
	} while (blk_mq_dispatch_rq_list(q, &rq_list, true));

	return ret;
}

static struct blk_mq_ctx *blk_mq_next_ctx(struct blk_mq_hw_ctx *hctx,
					  struct blk_mq_ctx *ctx)
{
	unsigned short idx = ctx->index_hw[hctx->type];

	if (++idx == hctx->nr_ctx)
		idx = 0;

	return hctx->ctxs[idx];
}

/*
 * Only SCSI implements .get_budget and .put_budget, and SCSI restarts
 * its queue by itself in its completion handler, so we don't need to
 * restart queue if .get_budget() returns BLK_STS_NO_RESOURCE.
 *
 * Returns -EAGAIN if hctx->dispatch was found non-empty and run_work has to
 * to be run again.  This is necessary to avoid starving flushes.
 */
static int blk_mq_do_dispatch_ctx(struct blk_mq_hw_ctx *hctx)
{
	struct request_queue *q = hctx->queue;
	LIST_HEAD(rq_list);
	struct blk_mq_ctx *ctx = READ_ONCE(hctx->dispatch_from);
	int ret = 0;

	do {
		struct request *rq;

		if (!list_empty_careful(&hctx->dispatch)) {
			ret = -EAGAIN;
			break;
		}

		if (!sbitmap_any_bit_set(&hctx->ctx_map))
			break;

		if (!blk_mq_get_dispatch_budget(hctx))
			break;

		rq = blk_mq_dequeue_from_ctx(hctx, ctx);
		if (!rq) {
			blk_mq_put_dispatch_budget(hctx);
			/*
			 * We're releasing without dispatching. Holding the
			 * budget could have blocked any "hctx"s with the
			 * same queue and if we didn't dispatch then there's
			 * no guarantee anyone will kick the queue.  Kick it
			 * ourselves.
			 */
			blk_mq_delay_run_hw_queues(q, BLK_MQ_BUDGET_DELAY);
			break;
		}

		/*
		 * Now this rq owns the budget which has to be released
		 * if this rq won't be queued to driver via .queue_rq()
		 * in blk_mq_dispatch_rq_list().
		 */
		list_add(&rq->queuelist, &rq_list);

		/* round robin for fair dispatch */
		ctx = blk_mq_next_ctx(hctx, rq->mq_ctx);

	} while (blk_mq_dispatch_rq_list(q, &rq_list, true));

	WRITE_ONCE(hctx->dispatch_from, ctx);
	return ret;
}
static void dispatch_list_rq_count(struct list_head *hctx_list_head)
{
    struct request *rq = list_first_entry(hctx_list_head, struct request, queuelist);
    if(rq->rq_disk && rq->rq_disk->process_io.enable && rq->p_process_rq_stat){
        int count = 0;
        struct request *tmp;
	struct process_io_info *p_process_io_info_tmp = rq->p_process_rq_stat->p_process_io_info;

	//?????? hctx->dispatch ?????????????????????req???????????????????????????max_hctx_list_rq_count
        list_for_each_entry(tmp,hctx_list_head,queuelist){
            count ++;
        }
	spin_lock_irq(&(p_process_io_info_tmp->io_data_lock));
        if(count > p_process_io_info_tmp->max_hctx_list_rq_count){
	    p_process_io_info_tmp->max_hctx_list_rq_count = count;
	}
	spin_unlock_irq(&(p_process_io_info_tmp->io_data_lock));
	if(count > 0)
	    printk("%s count:%d\n",__func__,count);
    }
}
int __blk_mq_sched_dispatch_requests(struct blk_mq_hw_ctx *hctx)
{
	struct request_queue *q = hctx->queue;
	struct elevator_queue *e = q->elevator;
	const bool has_sched_dispatch = e && e->type->ops.dispatch_request;
	int ret = 0;
	LIST_HEAD(rq_list);

	/*
	 * If we have previous entries on our dispatch list, grab them first for
	 * more fair dispatch.
	 */
	if (!list_empty_careful(&hctx->dispatch)) {
		spin_lock(&hctx->lock);
		if (!list_empty(&hctx->dispatch))
	        {
			/******process_rq_stat***************/
			dispatch_list_rq_count(&hctx->dispatch);

			list_splice_init(&hctx->dispatch, &rq_list);
		}
		spin_unlock(&hctx->lock);
	}

	/*
	 * Only ask the scheduler for requests, if we didn't have residual
	 * requests from the dispatch list. This is to avoid the case where
	 * we only ever dispatch a fraction of the requests available because
	 * of low device queue depth. Once we pull requests out of the IO
	 * scheduler, we can no longer merge or sort them. So it's best to
	 * leave them there for as long as we can. Mark the hw queue as
	 * needing a restart in that case.
	 *
	 * We want to dispatch from the scheduler if there was nothing
	 * on the dispatch list or we were able to dispatch from the
	 * dispatch list.
	 */
	if (!list_empty(&rq_list)) {
		blk_mq_sched_mark_restart_hctx(hctx);
		if (blk_mq_dispatch_rq_list(q, &rq_list, false)) {
			if (has_sched_dispatch)
				ret = blk_mq_do_dispatch_sched(hctx);
			else
				ret = blk_mq_do_dispatch_ctx(hctx);
		}
	} else if (has_sched_dispatch) {
		ret = blk_mq_do_dispatch_sched(hctx);
	} else if (hctx->dispatch_busy) {
		/* dequeue request one by one from sw queue if queue is busy */
		ret = blk_mq_do_dispatch_ctx(hctx);
	} else {
		blk_mq_flush_busy_ctxs(hctx, &rq_list);
		blk_mq_dispatch_rq_list(q, &rq_list, false);
	}

	return ret;
}

void blk_mq_sched_dispatch_requests(struct blk_mq_hw_ctx *hctx)
{
	struct request_queue *q = hctx->queue;

	/* RCU or SRCU read lock is needed before checking quiesced flag */
	if (unlikely(blk_mq_hctx_stopped(hctx) || blk_queue_quiesced(q)))
		return;

	hctx->run++;

	/*
	 * A return of -EAGAIN is an indication that hctx->dispatch is not
	 * empty and we must run again in order to avoid starving flushes.
	 */
	if (__blk_mq_sched_dispatch_requests(hctx) == -EAGAIN) {
		if (__blk_mq_sched_dispatch_requests(hctx) == -EAGAIN)
			blk_mq_run_hw_queue(hctx, true);
	}
}

bool blk_mq_sched_try_merge(struct request_queue *q, struct bio *bio,
			    struct request **merged_request)
{
	struct request *rq;

	switch (elv_merge(q, &rq, bio)) {
	case ELEVATOR_BACK_MERGE:
		if (!blk_mq_sched_allow_merge(q, rq, bio))
			return false;
		if (!bio_attempt_back_merge(q, rq, bio))
			return false;
		*merged_request = attempt_back_merge(q, rq);
		if (!*merged_request)
			elv_merged_request(q, rq, ELEVATOR_BACK_MERGE);
		return true;
	case ELEVATOR_FRONT_MERGE:
		if (!blk_mq_sched_allow_merge(q, rq, bio))
			return false;
		if (!bio_attempt_front_merge(q, rq, bio))
			return false;
		*merged_request = attempt_front_merge(q, rq);
		if (!*merged_request)
			elv_merged_request(q, rq, ELEVATOR_FRONT_MERGE);
		return true;
	case ELEVATOR_DISCARD_MERGE:
		return bio_attempt_discard_merge(q, rq, bio);
	default:
		return false;
	}
}
EXPORT_SYMBOL_GPL(blk_mq_sched_try_merge);

/*
 * Iterate list of requests and see if we can merge this bio with any
 * of them.
 */
bool blk_mq_bio_list_merge(struct request_queue *q, struct list_head *list,
			   struct bio *bio)
{
	struct request *rq;
	int checked = 8;

	list_for_each_entry_reverse(rq, list, queuelist) {
		bool merged = false;

		if (!checked--)
			break;

		if (!blk_rq_merge_ok(rq, bio))
			continue;

		switch (blk_try_merge(rq, bio)) {
		case ELEVATOR_BACK_MERGE:
			if (blk_mq_sched_allow_merge(q, rq, bio))
				merged = bio_attempt_back_merge(q, rq, bio);
			break;
		case ELEVATOR_FRONT_MERGE:
			if (blk_mq_sched_allow_merge(q, rq, bio))
				merged = bio_attempt_front_merge(q, rq, bio);
			break;
		case ELEVATOR_DISCARD_MERGE:
			merged = bio_attempt_discard_merge(q, rq, bio);
			break;
		default:
			continue;
		}

		return merged;
	}

	return false;
}
EXPORT_SYMBOL_GPL(blk_mq_bio_list_merge);

/*
 * Reverse check our software queue for entries that we could potentially
 * merge with. Currently includes a hand-wavy stop count of 8, to not spend
 * too much time checking for merges.
 */
static bool blk_mq_attempt_merge(struct request_queue *q,
				 struct blk_mq_hw_ctx *hctx,
				 struct blk_mq_ctx *ctx, struct bio *bio)
{
	enum hctx_type type = hctx->type;

	lockdep_assert_held(&ctx->lock);

	if (blk_mq_bio_list_merge(q, &ctx->rq_lists[type], bio)) {
		ctx->rq_merged++;
		return true;
	}

	return false;
}

bool __blk_mq_sched_bio_merge(struct request_queue *q, struct bio *bio)
{
	struct elevator_queue *e = q->elevator;
	struct blk_mq_ctx *ctx = blk_mq_get_ctx(q);
	struct blk_mq_hw_ctx *hctx = blk_mq_map_queue(q, bio->bi_opf, ctx);
	bool ret = false;
	enum hctx_type type;

	if (e && e->type->ops.bio_merge)
		return e->type->ops.bio_merge(hctx, bio);

	type = hctx->type;
	if ((hctx->flags & BLK_MQ_F_SHOULD_MERGE) &&
			!list_empty_careful(&ctx->rq_lists[type])) {
		/* default per sw-queue merge */
		spin_lock(&ctx->lock);
		ret = blk_mq_attempt_merge(q, hctx, ctx, bio);
		spin_unlock(&ctx->lock);
	}

	return ret;
}

bool blk_mq_sched_try_insert_merge(struct request_queue *q, struct request *rq)
{
	return rq_mergeable(rq) && elv_attempt_insert_merge(q, rq);
}
EXPORT_SYMBOL_GPL(blk_mq_sched_try_insert_merge);

void blk_mq_sched_request_inserted(struct request *rq)
{
	//int cycle_count = 0;
	trace_block_rq_insert(rq->q, rq);
        /******process_rq_stat***************/
	//if(rq->rq_disk && rq->rq_disk->process_io.enable && (process_io_count < 50)){
	rq->p_process_rq_stat = NULL;//???rq->p_process_rq_stat????????????NULL???rq->p_process_rq_stat?????????rq??????process_io_info??????
	if(rq->rq_disk && rq->rq_disk->process_io.enable){
		struct process_rq_stat *p_process_rq_stat_tmp = NULL;
	        struct process_io_info *p_process_io_info_tmp = NULL;
	        int find = 0;
		
		if(!rq->rq_disk->process_io.process_io_info_cachep || !rq->rq_disk->process_io.process_rq_stat_cachep){
		    printk(KERN_DEBUG"%s %p %p\n",__func__,rq->rq_disk->process_io.process_io_info_cachep,rq->rq_disk->process_io.process_rq_stat_cachep);
		}
                //spin_lock_irq(&(rq->rq_disk->process_io.process_lock));
                atomic_inc(&(rq->rq_disk->process_io.read_lock_count));//?????? rcu_read_lock()???????????????Grace Period
		//list_for_each_entry(p_process_io_info_tmp, &(rq->rq_disk->process_io.process_io_control_head), process_io_info_list){
		list_for_each_entry_rcu(p_process_io_info_tmp, &(rq->rq_disk->process_io.process_io_control_head), process_io_info_list){

			if(p_process_io_info_tmp->pid == current->pid){
		              //????????p_process_io_info_tmp->rq_count ++?????????????????????.???????????????????????????????????????p_process_io_info_tmp->rq_count???0?????????p_process_io_info_tmp->rq_empty_count???4???process_io_info???
			      //????????????????????????p_process_io_info_tmp->rq_count ++????????????process_io???????????????print_process_io_info()?????????p_process_io_info_tmp->rq_count??????0?????????p_process_io_info_tmp->rq_empty_count++??????5???
			      //???????????????p_process_io_info_tmp??????????????????blk_mq_sched_request_inserted()?????????????????????p_process_io_info_tmp->rq_count ++??????crash???????????????p_process_io_info_tmp??????process_io_info???????????????
	                      spin_lock_irq(&(rq->rq_disk->process_io.process_lock_list));
			      //??????p_process_io_info_tmp->has_deleted ???1?????????p_process_io_info_tmp???print_process_io_info()????????????process_io_control_head????????????????????????????????????
			      //???????????????????????????process_io_info????????????p_process_io_info_tmp->rq_count???1?????????print_process_io_info()????????????p_process_io_info_tmp->rq_count??????0??????????????????
			      //process_io_control_head??????????????????p_process_io_info_tmp???????????????spin_lock_irq????????????????????????????????? ??? print_process_io_info()????????????process_io_control_head????????????
			      //p_process_io_info_tmp????????????????????????????????????????????????print_process_io_info()?????????spin_lock?????????p_process_io_info_tmp?????????????????????p_process_io_info_tmp->has_deleted???1???
			      //?????????????????????spin lock??????p_process_io_info_tmp->rq_count???1?????????print_process_io_info()??????p_process_io_info_tmp->rq_count??????0???????????????????????????p_process_io_info_tmp???
			      //?????????????????????????????????????????? print_process_io_info()???process_io_control_head???????????????p_process_io_info_tmp????????????????????????????????????p_process_io_info_tmp???????????????????????????
			      if(p_process_io_info_tmp->has_deleted){//?????????????????????????????????
				  //?????????process_io_info ??????delete?????????break????????????spin unlock??????????????????SB?????????????????????
				  spin_unlock_irq(&(rq->rq_disk->process_io.process_lock_list));
			          break;
			      }
			      else
			      {
				  //?????????????????????spin lock??????,????????????????????????????????????spin lock????????????????????????????????????????????????????????????????????? ??? ??????print_process_io_info()???????????????
				  //????????????rq_count spin lock?????????????????????????????????p_process_io_info_tmp->has_deleted ???0???????????????spin lock??????????????????
				  //atomic_inc(&(p_process_io_info_tmp->rq_count))???1???????????????1??????spin unlock????????????print_process_io_info()????????????rq_count???0???
				  //?????????p_process_io_info_tmp?????????process_io_info?????????????????????????????????atomic_inc(&(p_process_io_info_tmp->rq_count))?????????????????????process_io_info???
				  //??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
                                  atomic_inc(&(p_process_io_info_tmp->rq_count));
			          //atomic_inc(&(rq->rq_disk->process_io.rq_in_queue));

			          /*spin_lock_irq(&(rq->rq_disk->process_io.io_data_lock));
				  //p_process_io_info_tmp->rq_count ????????????io_data_lock?????????????????????blk_account_io_done???rq_count--?????????io_data_lock?????????
		                  p_process_io_info_tmp->rq_count ++;
		                  rq->rq_disk->process_io.rq_in_queue ++;
		                  spin_unlock_irq(&(rq->rq_disk->process_io.io_data_lock));*/
		              }
	                      spin_unlock_irq(&(rq->rq_disk->process_io.process_lock_list));

			      atomic_inc(&(rq->rq_disk->process_io.rq_in_queue));
                              //atomic_inc(&(p_process_io_info_tmp->rq_count));
                              find = 1;
			      break;
			}
		}
		//spin_unlock_irq(&(rq->rq_disk->process_io.process_lock));
                atomic_dec(&(rq->rq_disk->process_io.read_lock_count));//?????? rcu_read_unlock()???????????????Grace Period
				
		if(0 == find){//??????????????????????????????
		    p_process_io_info_tmp = kmem_cache_alloc(rq->rq_disk->process_io.process_io_info_cachep,GFP_ATOMIC);
		    if(!p_process_io_info_tmp)
			    goto fail;
			
			memset(p_process_io_info_tmp,0,sizeof(struct process_io_info));
			atomic_set(&(p_process_io_info_tmp->rq_count),0);
			spin_lock_init(&(p_process_io_info_tmp->io_data_lock));
                        //???process_io_control_head????????????process_io_info????????????,???????????????print_process_io_info()???????????? process_io_control_head????????????process_io_info???????????????writer???????????????
                        spin_lock_irq(&(rq->rq_disk->process_io.process_lock_list));
			//??????????????????IO?????????1
		        //p_process_io_info_tmp->rq_count ++;
			//???IO?????????IO????????????1
			//rq->rq_disk->process_io.rq_in_queue ++;
			list_add_rcu(&p_process_io_info_tmp->process_io_info_list,&(rq->rq_disk->process_io.process_io_control_head));//???spin lock?????????????????????rcu add???????????????
		        spin_unlock_irq(&(rq->rq_disk->process_io.process_lock_list));
                
			atomic_inc(&(rq->rq_disk->process_io.rq_in_queue));
                        atomic_inc(&(p_process_io_info_tmp->rq_count));
			/*spin_lock_irq(&(rq->rq_disk->process_io.io_data_lock));
		        p_process_io_info_tmp->rq_count ++;
		        rq->rq_disk->process_io.rq_in_queue ++;
		        spin_unlock_irq(&(rq->rq_disk->process_io.io_data_lock));*/
	        }

		p_process_rq_stat_tmp = kmem_cache_alloc(rq->rq_disk->process_io.process_rq_stat_cachep,GFP_ATOMIC);
		if(!p_process_rq_stat_tmp)
			goto fail;
		memset(p_process_rq_stat_tmp,0,sizeof(struct process_rq_stat));
		
		p_process_io_info_tmp->pid = current->pid;
		strncpy(p_process_io_info_tmp->comm,current->comm,COMM_LEN-1);
		
		p_process_rq_stat_tmp->p_process_io_info = p_process_io_info_tmp;
		smp_mb();
		p_process_rq_stat_tmp->rq_inset_time = ktime_to_us(ktime_get());
		rq->p_process_rq_stat = p_process_rq_stat_tmp;
		printk(KERN_DEBUG"%s rq:0x%llx process_rq_stat:0x%llx rq_inset_time:%lld  p_process_io_info_tmp:0x%llx pid:%d rq_real_issue_time:%lld\n",__func__,(u64)rq,(u64)(rq->p_process_rq_stat),p_process_rq_stat_tmp->rq_inset_time,(u64)p_process_io_info_tmp,p_process_io_info_tmp->pid,p_process_rq_stat_tmp->rq_real_issue_time);

		return;
	fail:
                if(p_process_rq_stat_tmp)
   	            kmem_cache_free(rq->rq_disk->process_io.process_rq_stat_cachep, p_process_rq_stat_tmp);
	        if(p_process_io_info_tmp)
	            kmem_cache_free(rq->rq_disk->process_io.process_io_info_cachep, p_process_io_info_tmp);
	}
}
EXPORT_SYMBOL_GPL(blk_mq_sched_request_inserted);

static bool blk_mq_sched_bypass_insert(struct blk_mq_hw_ctx *hctx,
				       bool has_sched,
				       struct request *rq)
{
	/*
	 * dispatch flush and passthrough rq directly
	 *
	 * passthrough request has to be added to hctx->dispatch directly.
	 * For some reason, device may be in one situation which can't
	 * handle FS request, so STS_RESOURCE is always returned and the
	 * FS request will be added to hctx->dispatch. However passthrough
	 * request may be required at that time for fixing the problem. If
	 * passthrough request is added to scheduler queue, there isn't any
	 * chance to dispatch it given we prioritize requests in hctx->dispatch.
	 */
	if ((rq->rq_flags & RQF_FLUSH_SEQ) || blk_rq_is_passthrough(rq))
		return true;

	if (has_sched)
		rq->rq_flags |= RQF_SORTED;

	return false;
}

void blk_mq_sched_insert_request(struct request *rq, bool at_head,
				 bool run_queue, bool async)
{
	struct request_queue *q = rq->q;
	struct elevator_queue *e = q->elevator;
	struct blk_mq_ctx *ctx = rq->mq_ctx;
	struct blk_mq_hw_ctx *hctx = rq->mq_hctx;

	/* flush rq in flush machinery need to be dispatched directly */
	if (!(rq->rq_flags & RQF_FLUSH_SEQ) && op_is_flush(rq->cmd_flags)) {
		blk_insert_flush(rq);
		goto run;
	}

	WARN_ON(e && (rq->tag != -1));

	if (blk_mq_sched_bypass_insert(hctx, !!e, rq)) {
		blk_mq_request_bypass_insert(rq, at_head, false);
		goto run;
	}

	if (e && e->type->ops.insert_requests) {
		LIST_HEAD(list);

		list_add(&rq->queuelist, &list);
		e->type->ops.insert_requests(hctx, &list, at_head);
	} else {
		spin_lock(&ctx->lock);
		__blk_mq_insert_request(hctx, rq, at_head);
		spin_unlock(&ctx->lock);
	}

run:
	if (run_queue)
		blk_mq_run_hw_queue(hctx, async);
}

void blk_mq_sched_insert_requests(struct blk_mq_hw_ctx *hctx,
				  struct blk_mq_ctx *ctx,
				  struct list_head *list, bool run_queue_async)
{
	struct elevator_queue *e;
	struct request_queue *q = hctx->queue;

	/*
	 * blk_mq_sched_insert_requests() is called from flush plug
	 * context only, and hold one usage counter to prevent queue
	 * from being released.
	 */
	percpu_ref_get(&q->q_usage_counter);

	e = hctx->queue->elevator;
	if (e && e->type->ops.insert_requests)
		e->type->ops.insert_requests(hctx, list, false);
	else {
		/*
		 * try to issue requests directly if the hw queue isn't
		 * busy in case of 'none' scheduler, and this way may save
		 * us one extra enqueue & dequeue to sw queue.
		 */
		if (!hctx->dispatch_busy && !e && !run_queue_async) {
			blk_mq_try_issue_list_directly(hctx, list);
			if (list_empty(list))
				goto out;
		}
		blk_mq_insert_requests(hctx, ctx, list);
	}

	blk_mq_run_hw_queue(hctx, run_queue_async);
 out:
	percpu_ref_put(&q->q_usage_counter);
}

static void blk_mq_sched_free_tags(struct blk_mq_tag_set *set,
				   struct blk_mq_hw_ctx *hctx,
				   unsigned int hctx_idx)
{
	if (hctx->sched_tags) {
		blk_mq_free_rqs(set, hctx->sched_tags, hctx_idx);
		blk_mq_free_rq_map(hctx->sched_tags);
		hctx->sched_tags = NULL;
	}
}

static int blk_mq_sched_alloc_tags(struct request_queue *q,
				   struct blk_mq_hw_ctx *hctx,
				   unsigned int hctx_idx)
{
	struct blk_mq_tag_set *set = q->tag_set;
	int ret;

	hctx->sched_tags = blk_mq_alloc_rq_map(set, hctx_idx, q->nr_requests,
					       set->reserved_tags);
	if (!hctx->sched_tags)
		return -ENOMEM;

	ret = blk_mq_alloc_rqs(set, hctx->sched_tags, hctx_idx, q->nr_requests);
	if (ret)
		blk_mq_sched_free_tags(set, hctx, hctx_idx);

	return ret;
}

/* called in queue's release handler, tagset has gone away */
static void blk_mq_sched_tags_teardown(struct request_queue *q)
{
	struct blk_mq_hw_ctx *hctx;
	int i;

	queue_for_each_hw_ctx(q, hctx, i) {
		if (hctx->sched_tags) {
			blk_mq_free_rq_map(hctx->sched_tags);
			hctx->sched_tags = NULL;
		}
	}
}

int blk_mq_init_sched(struct request_queue *q, struct elevator_type *e)
{
	struct blk_mq_hw_ctx *hctx;
	struct elevator_queue *eq;
	unsigned int i;
	int ret;

	if (!e) {
		q->elevator = NULL;
		q->nr_requests = q->tag_set->queue_depth;
		return 0;
	}

	/*
	 * Default to double of smaller one between hw queue_depth and 128,
	 * since we don't split into sync/async like the old code did.
	 * Additionally, this is a per-hw queue depth.
	 */
	q->nr_requests = 2 * min_t(unsigned int, q->tag_set->queue_depth,
				   BLKDEV_MAX_RQ);

	queue_for_each_hw_ctx(q, hctx, i) {
		ret = blk_mq_sched_alloc_tags(q, hctx, i);
		if (ret)
			goto err;
	}

	ret = e->ops.init_sched(q, e);
	if (ret)
		goto err;

	blk_mq_debugfs_register_sched(q);

	queue_for_each_hw_ctx(q, hctx, i) {
		if (e->ops.init_hctx) {
			ret = e->ops.init_hctx(hctx, i);
			if (ret) {
				eq = q->elevator;
				blk_mq_sched_free_requests(q);
				blk_mq_exit_sched(q, eq);
				kobject_put(&eq->kobj);
				return ret;
			}
		}
		blk_mq_debugfs_register_sched_hctx(q, hctx);
	}

	return 0;

err:
	blk_mq_sched_free_requests(q);
	blk_mq_sched_tags_teardown(q);
	q->elevator = NULL;
	return ret;
}

/*
 * called in either blk_queue_cleanup or elevator_switch, tagset
 * is required for freeing requests
 */
void blk_mq_sched_free_requests(struct request_queue *q)
{
	struct blk_mq_hw_ctx *hctx;
	int i;

	queue_for_each_hw_ctx(q, hctx, i) {
		if (hctx->sched_tags)
			blk_mq_free_rqs(q->tag_set, hctx->sched_tags, i);
	}
}

void blk_mq_exit_sched(struct request_queue *q, struct elevator_queue *e)
{
	struct blk_mq_hw_ctx *hctx;
	unsigned int i;

	queue_for_each_hw_ctx(q, hctx, i) {
		blk_mq_debugfs_unregister_sched_hctx(hctx);
		if (e->type->ops.exit_hctx && hctx->sched_data) {
			e->type->ops.exit_hctx(hctx, i);
			hctx->sched_data = NULL;
		}
	}
	blk_mq_debugfs_unregister_sched(q);
	if (e->type->ops.exit_sched)
		e->type->ops.exit_sched(e);
	blk_mq_sched_tags_teardown(q);
	q->elevator = NULL;
}
