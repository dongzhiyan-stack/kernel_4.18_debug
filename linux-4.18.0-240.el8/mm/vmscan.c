// SPDX-License-Identifier: GPL-2.0
/*
 *  linux/mm/vmscan.c
 *
 *  Copyright (C) 1991, 1992, 1993, 1994  Linus Torvalds
 *
 *  Swap reorganised 29.12.95, Stephen Tweedie.
 *  kswapd added: 7.1.96  sct
 *  Removed kswapd_ctl limits, and swap out as many pages as needed
 *  to bring the system back to freepages.high: 2.4.97, Rik van Riel.
 *  Zone aware kswapd started 02/00, Kanoj Sarcar (kanoj@sgi.com).
 *  Multiqueue VM started 5.8.00, Rik van Riel.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/mm.h>
#include <linux/sched/mm.h>
#include <linux/module.h>
#include <linux/gfp.h>
#include <linux/kernel_stat.h>
#include <linux/swap.h>
#include <linux/pagemap.h>
#include <linux/init.h>
#include <linux/highmem.h>
#include <linux/vmpressure.h>
#include <linux/vmstat.h>
#include <linux/file.h>
#include <linux/writeback.h>
#include <linux/blkdev.h>
#include <linux/buffer_head.h>	/* for try_to_release_page(),
					buffer_heads_over_limit */
#include <linux/mm_inline.h>
#include <linux/backing-dev.h>
#include <linux/rmap.h>
#include <linux/topology.h>
#include <linux/cpu.h>
#include <linux/cpuset.h>
#include <linux/compaction.h>
#include <linux/notifier.h>
#include <linux/rwsem.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/memcontrol.h>
#include <linux/delayacct.h>
#include <linux/sysctl.h>
#include <linux/oom.h>
#include <linux/pagevec.h>
#include <linux/prefetch.h>
#include <linux/printk.h>
#include <linux/dax.h>
#include <linux/psi.h>

#include <asm/tlbflush.h>
#include <asm/div64.h>

#include <linux/swapops.h>
#include <linux/balloon_compaction.h>

#include "internal.h"

#define CREATE_TRACE_POINTS
#include <trace/events/vmscan.h>
/**********************************************************************/
extern int open_shrink_printk;
extern int open_shrink_printk1;
int async_shrink_enable = 0;
int hot_file_shrink_enable = 0;
static int async_shrink_memory(void *p);
void inline update_async_shrink_page(struct page *page);

int hot_file_init(void);
int hot_file_thread_enable = 0;

struct scan_control {
	/* How many pages shrink_list() should reclaim */
	unsigned long nr_to_reclaim;

	/* This context's GFP mask */
	gfp_t gfp_mask;

	/* Allocation order */
	int order;

	/*
	 * Nodemask of nodes allowed by the caller. If NULL, all nodes
	 * are scanned.
	 */
	nodemask_t	*nodemask;

	/*
	 * The memory cgroup that hit its limit and as a result is the
	 * primary target of this reclaim invocation.
	 */
	struct mem_cgroup *target_mem_cgroup;

	/* Scan (total_size >> priority) pages at once */
	int priority;

	/* The highest zone to isolate pages for reclaim from */
	enum zone_type reclaim_idx;

	/* Writepage batching in laptop mode; RECLAIM_WRITE */
	unsigned int may_writepage:1;

	/* Can mapped pages be reclaimed? */
	unsigned int may_unmap:1;

	/* Can pages be swapped as part of reclaim? */
	unsigned int may_swap:1;

	/*
	 * Cgroups are not reclaimed below their configured memory.low,
	 * unless we threaten to OOM. If any cgroups are skipped due to
	 * memory.low and nothing was reclaimed, go back for memory.low.
	 */
	unsigned int memcg_low_reclaim:1;
	unsigned int memcg_low_skipped:1;

	unsigned int hibernation_mode:1;

	/* One of the zones is ready for compaction */
	unsigned int compaction_ready:1;

	/* Incremented by the number of inactive pages that were scanned */
	unsigned long nr_scanned;

	/* Number of pages freed so far during a call to shrink_zones() */
	unsigned long nr_reclaimed;

	struct {
		unsigned int dirty;
		unsigned int unqueued_dirty;
		unsigned int congested;
		unsigned int writeback;
		unsigned int immediate;
		unsigned int file_taken;
		unsigned int taken;
	} nr;
};

#ifdef ARCH_HAS_PREFETCH
#define prefetch_prev_lru_page(_page, _base, _field)			\
	do {								\
		if ((_page)->lru.prev != _base) {			\
			struct page *prev;				\
									\
			prev = lru_to_page(&(_page->lru));		\
			prefetch(&prev->_field);			\
		}							\
	} while (0)
#else
#define prefetch_prev_lru_page(_page, _base, _field) do { } while (0)
#endif

#ifdef ARCH_HAS_PREFETCHW
#define prefetchw_prev_lru_page(_page, _base, _field)			\
	do {								\
		if ((_page)->lru.prev != _base) {			\
			struct page *prev;				\
									\
			prev = lru_to_page(&(_page->lru));		\
			prefetchw(&prev->_field);			\
		}							\
	} while (0)
#else
#define prefetchw_prev_lru_page(_page, _base, _field) do { } while (0)
#endif

/*
 * From 0 .. 100.  Higher means more swappy.
 */
int vm_swappiness = 60;
/*
 * The total number of pages which are beyond the high watermark within all
 * zones.
 */
unsigned long vm_total_pages;

static LIST_HEAD(shrinker_list);
static DECLARE_RWSEM(shrinker_rwsem);

#ifdef CONFIG_MEMCG_KMEM
static DEFINE_IDR(shrinker_idr);
static int shrinker_nr_max;

static int prealloc_memcg_shrinker(struct shrinker *shrinker)
{
	int id, ret = -ENOMEM;

	down_write(&shrinker_rwsem);
	/* This may call shrinker, so it must use down_read_trylock() */
	id = idr_alloc(&shrinker_idr, shrinker, 0, 0, GFP_KERNEL);
	if (id < 0)
		goto unlock;

	if (id >= shrinker_nr_max) {
		if (memcg_expand_shrinker_maps(id)) {
			idr_remove(&shrinker_idr, id);
			goto unlock;
		}

		shrinker_nr_max = id + 1;
	}
	shrinker->id = id;
	ret = 0;
unlock:
	up_write(&shrinker_rwsem);
	return ret;
}

static void unregister_memcg_shrinker(struct shrinker *shrinker)
{
	int id = shrinker->id;

	BUG_ON(id < 0);

	down_write(&shrinker_rwsem);
	idr_remove(&shrinker_idr, id);
	up_write(&shrinker_rwsem);
}
#else /* CONFIG_MEMCG_KMEM */
static int prealloc_memcg_shrinker(struct shrinker *shrinker)
{
	return 0;
}

static void unregister_memcg_shrinker(struct shrinker *shrinker)
{
}
#endif /* CONFIG_MEMCG_KMEM */

#ifdef CONFIG_MEMCG
static bool global_reclaim(struct scan_control *sc)
{
	return !sc->target_mem_cgroup;
}

/**
 * sane_reclaim - is the usual dirty throttling mechanism operational?
 * @sc: scan_control in question
 *
 * The normal page dirty throttling mechanism in balance_dirty_pages() is
 * completely broken with the legacy memcg and direct stalling in
 * shrink_page_list() is used for throttling instead, which lacks all the
 * niceties such as fairness, adaptive pausing, bandwidth proportional
 * allocation and configurability.
 *
 * This function tests whether the vmscan currently in progress can assume
 * that the normal dirty throttling mechanism is operational.
 */
static bool sane_reclaim(struct scan_control *sc)
{
	struct mem_cgroup *memcg = sc->target_mem_cgroup;

	if (!memcg)
		return true;
#ifdef CONFIG_CGROUP_WRITEBACK
	if (cgroup_subsys_on_dfl(memory_cgrp_subsys))
		return true;
#endif
	return false;
}

static void set_memcg_congestion(pg_data_t *pgdat,
				struct mem_cgroup *memcg,
				bool congested)
{
	struct mem_cgroup_per_node *mn;

	if (!memcg)
		return;

	mn = mem_cgroup_nodeinfo(memcg, pgdat->node_id);
	WRITE_ONCE(mn->congested, congested);
}

static bool memcg_congested(pg_data_t *pgdat,
			struct mem_cgroup *memcg)
{
	struct mem_cgroup_per_node *mn;

	mn = mem_cgroup_nodeinfo(memcg, pgdat->node_id);
	return READ_ONCE(mn->congested);

}
#else
static bool global_reclaim(struct scan_control *sc)
{
	return true;
}

static bool sane_reclaim(struct scan_control *sc)
{
	return true;
}

static inline void set_memcg_congestion(struct pglist_data *pgdat,
				struct mem_cgroup *memcg, bool congested)
{
}

static inline bool memcg_congested(struct pglist_data *pgdat,
			struct mem_cgroup *memcg)
{
	return false;

}
#endif

/*
 * This misses isolated pages which are not accounted for to save counters.
 * As the data only determines if reclaim or compaction continues, it is
 * not expected that isolated pages will be a dominating factor.
 */
unsigned long zone_reclaimable_pages(struct zone *zone)
{
	unsigned long nr;

	nr = zone_page_state_snapshot(zone, NR_ZONE_INACTIVE_FILE) +
		zone_page_state_snapshot(zone, NR_ZONE_ACTIVE_FILE);
	if (get_nr_swap_pages() > 0)
		nr += zone_page_state_snapshot(zone, NR_ZONE_INACTIVE_ANON) +
			zone_page_state_snapshot(zone, NR_ZONE_ACTIVE_ANON);

	return nr;
}

/**
 * lruvec_lru_size -  Returns the number of pages on the given LRU list.
 * @lruvec: lru vector
 * @lru: lru to use
 * @zone_idx: zones to consider (use MAX_NR_ZONES for the whole LRU list)
 */
unsigned long lruvec_lru_size(struct lruvec *lruvec, enum lru_list lru, int zone_idx)
{
	unsigned long lru_size = 0;
	int zid;

	if (!mem_cgroup_disabled()) {
		for (zid = 0; zid < MAX_NR_ZONES; zid++)
			lru_size += mem_cgroup_get_zone_lru_size(lruvec, lru, zid);
	} else
		lru_size = node_page_state(lruvec_pgdat(lruvec), NR_LRU_BASE + lru);

	for (zid = zone_idx + 1; zid < MAX_NR_ZONES; zid++) {
		struct zone *zone = &lruvec_pgdat(lruvec)->node_zones[zid];
		unsigned long size;

		if (!managed_zone(zone))
			continue;

		if (!mem_cgroup_disabled())
			size = mem_cgroup_get_zone_lru_size(lruvec, lru, zid);
		else
			size = zone_page_state(&lruvec_pgdat(lruvec)->node_zones[zid],
				       NR_ZONE_LRU_BASE + lru);
		lru_size -= min(size, lru_size);
	}

	return lru_size;

}

/*
 * Add a shrinker callback to be called from the vm.
 */
int prealloc_shrinker(struct shrinker *shrinker)
{
	size_t size = sizeof(*shrinker->nr_deferred);

	if (shrinker->flags & SHRINKER_NUMA_AWARE)
		size *= nr_node_ids;

	shrinker->nr_deferred = kzalloc(size, GFP_KERNEL);
	if (!shrinker->nr_deferred)
		return -ENOMEM;

	/*
	 * There is a window between prealloc_shrinker()
	 * and register_shrinker_prepared(). We don't want
	 * to clear bit of a shrinker in such the state
	 * in shrink_slab_memcg(), since this will impose
	 * restrictions on a code registering a shrinker
	 * (they would have to guarantee, their LRU lists
	 * are empty till shrinker is completely registered).
	 * So, we differ the situation, when 1)a shrinker
	 * is semi-registered (id is assigned, but it has
	 * not yet linked to shrinker_list) and 2)shrinker
	 * is not registered (id is not assigned).
	 */
	INIT_LIST_HEAD(&shrinker->list);

	if (shrinker->flags & SHRINKER_MEMCG_AWARE) {
		if (prealloc_memcg_shrinker(shrinker))
			goto free_deferred;
	}

	return 0;

free_deferred:
	kfree(shrinker->nr_deferred);
	shrinker->nr_deferred = NULL;
	return -ENOMEM;
}

void free_prealloced_shrinker(struct shrinker *shrinker)
{
	if (!shrinker->nr_deferred)
		return;

	if (shrinker->flags & SHRINKER_MEMCG_AWARE)
		unregister_memcg_shrinker(shrinker);

	kfree(shrinker->nr_deferred);
	shrinker->nr_deferred = NULL;
}

void register_shrinker_prepared(struct shrinker *shrinker)
{
	down_write(&shrinker_rwsem);
	list_add_tail(&shrinker->list, &shrinker_list);
	up_write(&shrinker_rwsem);
}

int register_shrinker(struct shrinker *shrinker)
{
	int err = prealloc_shrinker(shrinker);

	if (err)
		return err;
	register_shrinker_prepared(shrinker);
	return 0;
}
EXPORT_SYMBOL(register_shrinker);

/*
 * Remove one
 */
void unregister_shrinker(struct shrinker *shrinker)
{
	if (!shrinker->nr_deferred)
		return;
	if (shrinker->flags & SHRINKER_MEMCG_AWARE)
		unregister_memcg_shrinker(shrinker);
	down_write(&shrinker_rwsem);
	list_del(&shrinker->list);
	up_write(&shrinker_rwsem);
	kfree(shrinker->nr_deferred);
	shrinker->nr_deferred = NULL;
}
EXPORT_SYMBOL(unregister_shrinker);

#define SHRINK_BATCH 128

static unsigned long do_shrink_slab(struct shrink_control *shrinkctl,
				    struct shrinker *shrinker, int priority)
{
	unsigned long freed = 0;
	unsigned long long delta;
	long total_scan;
	long freeable;
	long nr;
	long new_nr;
	int nid = shrinkctl->nid;
	long batch_size = shrinker->batch ? shrinker->batch
					  : SHRINK_BATCH;
	long scanned = 0, next_deferred;

	freeable = shrinker->count_objects(shrinker, shrinkctl);
	if (freeable == 0 || freeable == SHRINK_EMPTY)
		return freeable;

	/*
	 * copy the current shrinker scan count into a local variable
	 * and zero it so that other concurrent shrinker invocations
	 * don't also do this scanning work.
	 */
	nr = atomic_long_xchg(&shrinker->nr_deferred[nid], 0);

	total_scan = nr;
	delta = freeable >> priority;
	delta *= 4;
	do_div(delta, shrinker->seeks);
	total_scan += delta;
	if (total_scan < 0) {
		pr_err("shrink_slab: %pF negative objects to delete nr=%ld\n",
		       shrinker->scan_objects, total_scan);
		total_scan = freeable;
		next_deferred = nr;
	} else
		next_deferred = total_scan;

	/*
	 * We need to avoid excessive windup on filesystem shrinkers
	 * due to large numbers of GFP_NOFS allocations causing the
	 * shrinkers to return -1 all the time. This results in a large
	 * nr being built up so when a shrink that can do some work
	 * comes along it empties the entire cache due to nr >>>
	 * freeable. This is bad for sustaining a working set in
	 * memory.
	 *
	 * Hence only allow the shrinker to scan the entire cache when
	 * a large delta change is calculated directly.
	 */
	if (delta < freeable / 4)
		total_scan = min(total_scan, freeable / 2);

	/*
	 * Avoid risking looping forever due to too large nr value:
	 * never try to free more than twice the estimate number of
	 * freeable entries.
	 */
	if (total_scan > freeable * 2)
		total_scan = freeable * 2;

	trace_mm_shrink_slab_start(shrinker, shrinkctl, nr,
				   freeable, delta, total_scan, priority);

	/*
	 * Normally, we should not scan less than batch_size objects in one
	 * pass to avoid too frequent shrinker calls, but if the slab has less
	 * than batch_size objects in total and we are really tight on memory,
	 * we will try to reclaim all available objects, otherwise we can end
	 * up failing allocations although there are plenty of reclaimable
	 * objects spread over several slabs with usage less than the
	 * batch_size.
	 *
	 * We detect the "tight on memory" situations by looking at the total
	 * number of objects we want to scan (total_scan). If it is greater
	 * than the total number of objects on slab (freeable), we must be
	 * scanning at high prio and therefore should try to reclaim as much as
	 * possible.
	 */
	while (total_scan >= batch_size ||
	       total_scan >= freeable) {
		unsigned long ret;
		unsigned long nr_to_scan = min(batch_size, total_scan);

		shrinkctl->nr_to_scan = nr_to_scan;
		shrinkctl->nr_scanned = nr_to_scan;
		ret = shrinker->scan_objects(shrinker, shrinkctl);
		if (ret == SHRINK_STOP)
			break;
		freed += ret;

		count_vm_events(SLABS_SCANNED, shrinkctl->nr_scanned);
		total_scan -= shrinkctl->nr_scanned;
		scanned += shrinkctl->nr_scanned;

		cond_resched();
	}

	if (next_deferred >= scanned)
		next_deferred -= scanned;
	else
		next_deferred = 0;
	/*
	 * move the unused scan count back into the shrinker in a
	 * manner that handles concurrent updates. If we exhausted the
	 * scan, there is no need to do an update.
	 */
	if (next_deferred > 0)
		new_nr = atomic_long_add_return(next_deferred,
						&shrinker->nr_deferred[nid]);
	else
		new_nr = atomic_long_read(&shrinker->nr_deferred[nid]);

	trace_mm_shrink_slab_end(shrinker, nid, freed, nr, new_nr, total_scan);
	return freed;
}

#ifdef CONFIG_MEMCG_KMEM
static unsigned long shrink_slab_memcg(gfp_t gfp_mask, int nid,
			struct mem_cgroup *memcg, int priority)
{
	struct memcg_shrinker_map *map;
	unsigned long freed = 0;
	int ret, i;

	if (!memcg_kmem_enabled() || !mem_cgroup_online(memcg))
		return 0;

	if (!down_read_trylock(&shrinker_rwsem))
		return 0;

	map = rcu_dereference_protected(memcg->nodeinfo[nid]->shrinker_map,
					true);
	if (unlikely(!map))
		goto unlock;

	for_each_set_bit(i, map->map, shrinker_nr_max) {
		struct shrink_control sc = {
			.gfp_mask = gfp_mask,
			.nid = nid,
			.memcg = memcg,
		};
		struct shrinker *shrinker;

		shrinker = idr_find(&shrinker_idr, i);
		if (unlikely(!shrinker)) {
			clear_bit(i, map->map);
			continue;
		}

		/* See comment in prealloc_shrinker() */
		if (unlikely(list_empty(&shrinker->list)))
			continue;

		ret = do_shrink_slab(&sc, shrinker, priority);
		if (ret == SHRINK_EMPTY) {
			clear_bit(i, map->map);
			/*
			 * After the shrinker reported that it had no objects to
			 * free, but before we cleared the corresponding bit in
			 * the memcg shrinker map, a new object might have been
			 * added. To make sure, we have the bit set in this
			 * case, we invoke the shrinker one more time and reset
			 * the bit if it reports that it is not empty anymore.
			 * The memory barrier here pairs with the barrier in
			 * memcg_set_shrinker_bit():
			 *
			 * list_lru_add()     shrink_slab_memcg()
			 *   list_add_tail()    clear_bit()
			 *   <MB>               <MB>
			 *   set_bit()          do_shrink_slab()
			 */
			smp_mb__after_atomic();
			ret = do_shrink_slab(&sc, shrinker, priority);
			if (ret == SHRINK_EMPTY)
				ret = 0;
			else
				memcg_set_shrinker_bit(memcg, nid, i);
		}
		freed += ret;

		if (rwsem_is_contended(&shrinker_rwsem)) {
			freed = freed ? : 1;
			break;
		}
	}
unlock:
	up_read(&shrinker_rwsem);
	return freed;
}
#else /* CONFIG_MEMCG_KMEM */
static unsigned long shrink_slab_memcg(gfp_t gfp_mask, int nid,
			struct mem_cgroup *memcg, int priority)
{
	return 0;
}
#endif /* CONFIG_MEMCG_KMEM */

/**
 * shrink_slab - shrink slab caches
 * @gfp_mask: allocation context
 * @nid: node whose slab caches to target
 * @memcg: memory cgroup whose slab caches to target
 * @priority: the reclaim priority
 *
 * Call the shrink functions to age shrinkable caches.
 *
 * @nid is passed along to shrinkers with SHRINKER_NUMA_AWARE set,
 * unaware shrinkers will receive a node id of 0 instead.
 *
 * @memcg specifies the memory cgroup to target. Unaware shrinkers
 * are called only if it is the root cgroup.
 *
 * @priority is sc->priority, we take the number of objects and >> by priority
 * in order to get the scan target.
 *
 * Returns the number of reclaimed slab objects.
 */
static unsigned long shrink_slab(gfp_t gfp_mask, int nid,
				 struct mem_cgroup *memcg,
				 int priority)
{
	struct shrinker *shrinker;
	unsigned long freed = 0;
	int ret;

	/*
	 * The root memcg might be allocated even though memcg is disabled
	 * via "cgroup_disable=memory" boot parameter.  This could make
	 * mem_cgroup_is_root() return false, then just run memcg slab
	 * shrink, but skip global shrink.  This may result in premature
	 * oom.
	 */
	if (!mem_cgroup_disabled() && !mem_cgroup_is_root(memcg))
		return shrink_slab_memcg(gfp_mask, nid, memcg, priority);

	if (!down_read_trylock(&shrinker_rwsem))
		goto out;

	list_for_each_entry(shrinker, &shrinker_list, list) {
		struct shrink_control sc = {
			.gfp_mask = gfp_mask,
			.nid = nid,
			.memcg = memcg,
		};

		if (!(shrinker->flags & SHRINKER_NUMA_AWARE))
			sc.nid = 0;

		ret = do_shrink_slab(&sc, shrinker, priority);
		if (ret == SHRINK_EMPTY)
			ret = 0;
		freed += ret;
		/*
		 * Bail out if someone want to register a new shrinker to
		 * prevent the regsitration from being stalled for long periods
		 * by parallel ongoing shrinking.
		 */
		if (rwsem_is_contended(&shrinker_rwsem)) {
			freed = freed ? : 1;
			break;
		}
	}

	up_read(&shrinker_rwsem);
out:
	cond_resched();
	return freed;
}

void drop_slab_node(int nid)
{
	unsigned long freed;

	do {
		struct mem_cgroup *memcg = NULL;

		freed = 0;
		memcg = mem_cgroup_iter(NULL, NULL, NULL);
		do {
			freed += shrink_slab(GFP_KERNEL, nid, memcg, 0);
		} while ((memcg = mem_cgroup_iter(NULL, memcg, NULL)) != NULL);
	} while (freed > 10);
}

void drop_slab(void)
{
	int nid;

	for_each_online_node(nid)
		drop_slab_node(nid);
}

static inline int is_page_cache_freeable(struct page *page)
{
	/*
	 * A freeable page cache page is referenced only by the caller
	 * that isolated the page, the page cache and optional buffer
	 * heads at page->private.
	 */
	int page_cache_pins = PageTransHuge(page) && PageSwapCache(page) ?
		HPAGE_PMD_NR : 1;
	return page_count(page) - page_has_private(page) == 1 + page_cache_pins;
}

static int may_write_to_inode(struct inode *inode, struct scan_control *sc)
{
	if (current->flags & PF_SWAPWRITE)
		return 1;
	if (!inode_write_congested(inode))
		return 1;
	if (inode_to_bdi(inode) == current->backing_dev_info)
		return 1;
	return 0;
}

/*
 * We detected a synchronous write error writing a page out.  Probably
 * -ENOSPC.  We need to propagate that into the address_space for a subsequent
 * fsync(), msync() or close().
 *
 * The tricky part is that after writepage we cannot touch the mapping: nothing
 * prevents it from being freed up.  But we have a ref on the page and once
 * that page is locked, the mapping is pinned.
 *
 * We're allowed to run sleeping lock_page() here because we know the caller has
 * __GFP_FS.
 */
static void handle_write_error(struct address_space *mapping,
				struct page *page, int error)
{
	lock_page(page);
	if (page_mapping(page) == mapping)
		mapping_set_error(mapping, error);
	unlock_page(page);
}

/* possible outcome of pageout() */
typedef enum {
	/* failed to write page out, page is locked */
	PAGE_KEEP,
	/* move page to the active list, page is locked */
	PAGE_ACTIVATE,
	/* page has been sent to the disk successfully, page is unlocked */
	PAGE_SUCCESS,
	/* page is clean and locked */
	PAGE_CLEAN,
} pageout_t;

/*
 * pageout is called by shrink_page_list() for each dirty page.
 * Calls ->writepage().
 */
static pageout_t pageout(struct page *page, struct address_space *mapping,
			 struct scan_control *sc)
{
	/*
	 * If the page is dirty, only perform writeback if that write
	 * will be non-blocking.  To prevent this allocation from being
	 * stalled by pagecache activity.  But note that there may be
	 * stalls if we need to run get_block().  We could test
	 * PagePrivate for that.
	 *
	 * If this process is currently in __generic_file_write_iter() against
	 * this page's queue, we can perform writeback even if that
	 * will block.
	 *
	 * If the page is swapcache, write it back even if that would
	 * block, for some throttling. This happens by accident, because
	 * swap_backing_dev_info is bust: it doesn't reflect the
	 * congestion state of the swapdevs.  Easy to fix, if needed.
	 */
	if (!is_page_cache_freeable(page))
		return PAGE_KEEP;
	if (!mapping) {
		/*
		 * Some data journaling orphaned pages can have
		 * page->mapping == NULL while being dirty with clean buffers.
		 */
		if (page_has_private(page)) {
			if (try_to_free_buffers(page)) {
				ClearPageDirty(page);
				pr_info("%s: orphaned page\n", __func__);
				return PAGE_CLEAN;
			}
		}
		return PAGE_KEEP;
	}
	if (mapping->a_ops->writepage == NULL)
		return PAGE_ACTIVATE;
	if (!may_write_to_inode(mapping->host, sc))
		return PAGE_KEEP;

	if (clear_page_dirty_for_io(page)) {
		int res;
		struct writeback_control wbc = {
			.sync_mode = WB_SYNC_NONE,
			.nr_to_write = SWAP_CLUSTER_MAX,
			.range_start = 0,
			.range_end = LLONG_MAX,
			.for_reclaim = 1,
		};

		SetPageReclaim(page);
		res = mapping->a_ops->writepage(page, &wbc);
		if (res < 0)
			handle_write_error(mapping, page, res);
		if (res == AOP_WRITEPAGE_ACTIVATE) {
			ClearPageReclaim(page);
			return PAGE_ACTIVATE;
		}

		if (!PageWriteback(page)) {
			/* synchronous write or broken a_ops? */
			ClearPageReclaim(page);
		}
		trace_mm_vmscan_writepage(page);
		inc_node_page_state(page, NR_VMSCAN_WRITE);
		return PAGE_SUCCESS;
	}

	return PAGE_CLEAN;
}

/*
 * Same as remove_mapping, but if the page is removed from the mapping, it
 * gets returned with a refcount of 0.
 */
static int __remove_mapping(struct address_space *mapping, struct page *page,
			    bool reclaimed)
{
	unsigned long flags;
	int refcount;

	BUG_ON(!PageLocked(page));
	BUG_ON(mapping != page_mapping(page));

	xa_lock_irqsave(&mapping->i_pages, flags);
	/*
	 * The non racy check for a busy page.
	 *
	 * Must be careful with the order of the tests. When someone has
	 * a ref to the page, it may be possible that they dirty it then
	 * drop the reference. So if PageDirty is tested before page_count
	 * here, then the following race may occur:
	 *
	 * get_user_pages(&page);
	 * [user mapping goes away]
	 * write_to(page);
	 *				!PageDirty(page)    [good]
	 * SetPageDirty(page);
	 * put_page(page);
	 *				!page_count(page)   [good, discard it]
	 *
	 * [oops, our write_to data is lost]
	 *
	 * Reversing the order of the tests ensures such a situation cannot
	 * escape unnoticed. The smp_rmb is needed to ensure the page->flags
	 * load is not satisfied before that of page->_refcount.
	 *
	 * Note that if SetPageDirty is always performed via set_page_dirty,
	 * and thus under the i_pages lock, then this ordering is not required.
	 */
	if (unlikely(PageTransHuge(page)) && PageSwapCache(page))
		refcount = 1 + HPAGE_PMD_NR;
	else
		refcount = 2;
	if (!page_ref_freeze(page, refcount))
		goto cannot_free;
	/* note: atomic_cmpxchg in page_freeze_refs provides the smp_rmb */
	if (unlikely(PageDirty(page))) {
		page_ref_unfreeze(page, refcount);
		goto cannot_free;
	}

	if (PageSwapCache(page)) {
		swp_entry_t swap = { .val = page_private(page) };
		mem_cgroup_swapout(page, swap);
		__delete_from_swap_cache(page);
		xa_unlock_irqrestore(&mapping->i_pages, flags);
		put_swap_page(page, swap);
	} else {
		void (*freepage)(struct page *);
		void *shadow = NULL;

		freepage = mapping->a_ops->freepage;
		/*
		 * Remember a shadow entry for reclaimed file cache in
		 * order to detect refaults, thus thrashing, later on.
		 *
		 * But don't store shadows in an address space that is
		 * already exiting.  This is not just an optizimation,
		 * inode reclaim needs to empty out the radix tree or
		 * the nodes are lost.  Don't plant shadows behind its
		 * back.
		 *
		 * We also don't store shadows for DAX mappings because the
		 * only page cache pages found in these are zero pages
		 * covering holes, and because we don't want to mix DAX
		 * exceptional entries and shadow exceptional entries in the
		 * same address_space.
		 */
		if (reclaimed && page_is_file_cache(page) &&
		    !mapping_exiting(mapping) && !dax_mapping(mapping))
			shadow = workingset_eviction(mapping, page);
		__delete_from_page_cache(page, shadow);
		xa_unlock_irqrestore(&mapping->i_pages, flags);

		if (freepage != NULL)
			freepage(page);
	}

	return 1;

cannot_free:
	xa_unlock_irqrestore(&mapping->i_pages, flags);
	return 0;
}

/*
 * Attempt to detach a locked page from its ->mapping.  If it is dirty or if
 * someone else has a ref on the page, abort and return 0.  If it was
 * successfully detached, return 1.  Assumes the caller has a single ref on
 * this page.
 */
int remove_mapping(struct address_space *mapping, struct page *page)
{
	if (__remove_mapping(mapping, page, false)) {
		/*
		 * Unfreezing the refcount with 1 rather than 2 effectively
		 * drops the pagecache ref for us without requiring another
		 * atomic operation.
		 */
		page_ref_unfreeze(page, 1);
		return 1;
	}
	return 0;
}

/**
 * putback_lru_page - put previously isolated page onto appropriate LRU list
 * @page: page to be put back to appropriate lru list
 *
 * Add previously isolated @page to appropriate LRU list.
 * Page may still be unevictable for other reasons.
 *
 * lru_lock must not be held, interrupts must be enabled.
 */
void putback_lru_page(struct page *page)
{
	lru_cache_add(page);
	put_page(page);		/* drop ref from isolate */
}

enum page_references {
	PAGEREF_RECLAIM,
	PAGEREF_RECLAIM_CLEAN,
	PAGEREF_KEEP,
	PAGEREF_ACTIVATE,
};

static enum page_references page_check_references(struct page *page,
						  struct scan_control *sc)
{
	int referenced_ptes, referenced_page;
	unsigned long vm_flags;

	referenced_ptes = page_referenced(page, 1, sc->target_mem_cgroup,
					  &vm_flags);
	referenced_page = TestClearPageReferenced(page);

	/*
	 * Mlock lost the isolation race with us.  Let try_to_unmap()
	 * move the page to the unevictable list.
	 */
	if (vm_flags & VM_LOCKED){
                if(open_shrink_printk)
		    printk("1:%s %s %d page:0x%llx page->flags:0x%lx referenced_ptes:%d referenced_page:%d vm_flags & VM_LOCKED :PAGEREF_RECLAIM\n",__func__,current->comm,current->pid,(u64)page,page->flags,referenced_ptes,referenced_page);
		return PAGEREF_RECLAIM;
	}

	if (referenced_ptes) {
		if (PageSwapBacked(page)){
                        if(open_shrink_printk)
		            printk("2:%s %s %d page:0x%llx page->flags:0x%lx referenced_ptes:%d referenced_page:%d PageSwapBacked:PAGEREF_ACTIVATE\n",__func__,current->comm,current->pid,(u64)page,page->flags,referenced_ptes,referenced_page);
			return PAGEREF_ACTIVATE;
		}
		/*
		 * All mapped pages start out with page table
		 * references from the instantiating fault, so we need
		 * to look twice if a mapped file page is used more
		 * than once.
		 *
		 * Mark it and spare it for another trip around the
		 * inactive list.  Another page table reference will
		 * lead to its activation.
		 *
		 * Note: the mark is set for activated pages as well
		 * so that recently deactivated but used pages are
		 * quickly recovered.
		 */
		SetPageReferenced(page);

		if (referenced_page || referenced_ptes > 1){
                        if(open_shrink_printk)
		            printk("3:%s %s %d page:0x%llx page->flags:0x%lx referenced_ptes:%d referenced_page:%d PAGEREF_ACTIVATE\n",__func__,current->comm,current->pid,(u64)page,page->flags,referenced_ptes,referenced_page);
			return PAGEREF_ACTIVATE;
		}

		/*
		 * Activate file-backed executable pages after first usage.
		 */
		if (vm_flags & VM_EXEC){
                        if(open_shrink_printk)
		            printk("4:%s %s %d page:0x%llx page->flags:0x%lx referenced_ptes:%d referenced_page:%d vm_flags & VM_EXEC:PAGEREF_ACTIVATE\n",__func__,current->comm,current->pid,(u64)page,page->flags,referenced_ptes,referenced_page);
			return PAGEREF_ACTIVATE;
                }
                if(open_shrink_printk)
		       printk("6:%s %s %d page:0x%llx page->flags:0x%lx referenced_ptes:%d referenced_page:%d PAGEREF_KEEP\n",__func__,current->comm,current->pid,(u64)page,page->flags,referenced_ptes,referenced_page);
		return PAGEREF_KEEP;
	}

	/* Reclaim if clean, defer dirty pages to writeback */
	if (referenced_page && !PageSwapBacked(page)){
                if(open_shrink_printk)
		    printk("7:%s %s %d page:0x%llx page->flags:0x%lx referenced_ptes:%d referenced_page:%d PAGEREF_RECLAIM_CLEAN\n",__func__,current->comm,current->pid,(u64)page,page->flags,referenced_ptes,referenced_page);
		return PAGEREF_RECLAIM_CLEAN;
        }
        
	if(open_shrink_printk)
            printk("8:%s %s %d page:0x%llx page->flags:0x%lx referenced_ptes:%d referenced_page:%d PAGEREF_RECLAIM\n",__func__,current->comm,current->pid,(u64)page,page->flags,referenced_ptes,referenced_page);
	return PAGEREF_RECLAIM;
}

/* Check if a page is dirty or under writeback */
static void page_check_dirty_writeback(struct page *page,
				       bool *dirty, bool *writeback)
{
	struct address_space *mapping;

	/*
	 * Anonymous pages are not handled by flushers and must be written
	 * from reclaim context. Do not stall reclaim based on them
	 */
	if (!page_is_file_cache(page) ||
	    (PageAnon(page) && !PageSwapBacked(page))) {
		*dirty = false;
		*writeback = false;
		return;
	}

	/* By default assume that the page flags are accurate */
	*dirty = PageDirty(page);
	*writeback = PageWriteback(page);

	/* Verify dirty/writeback state if the filesystem supports it */
	if (!page_has_private(page))
		return;

	mapping = page_mapping(page);
	if (mapping && mapping->a_ops->is_dirty_writeback)
		mapping->a_ops->is_dirty_writeback(page, dirty, writeback);
}

/*
 * shrink_page_list() returns the number of reclaimed pages
 */
static unsigned long shrink_page_list(struct list_head *page_list,
				      struct pglist_data *pgdat,
				      struct scan_control *sc,
				      enum ttu_flags ttu_flags,
				      struct reclaim_stat *stat,
				      bool force_reclaim)
{
	LIST_HEAD(ret_pages);
	LIST_HEAD(free_pages);
	int pgactivate = 0;
	unsigned nr_unqueued_dirty = 0;
	unsigned nr_dirty = 0;
	unsigned nr_congested = 0;
	unsigned nr_reclaimed = 0;
	unsigned nr_writeback = 0;
	unsigned nr_immediate = 0;
	unsigned nr_ref_keep = 0;
	unsigned nr_unmap_fail = 0;

	cond_resched();
        if(open_shrink_printk)
	    printk("1:%s %s %d\n",__func__,current->comm,current->pid);

	while (!list_empty(page_list)) {
		struct address_space *mapping;
		struct page *page;
		int may_enter_fs;
		enum page_references references = PAGEREF_RECLAIM_CLEAN;
		bool dirty, writeback;

		cond_resched();

		page = lru_to_page(page_list);
		list_del(&page->lru);
                if(open_shrink_printk)
		    printk("2:%s %s %d page:0x%llx page->flags:0x%lx page_evictable:%d page_mapped:%d PageWriteback:%d PageDirty:%d PageLocked:%d\n",__func__,current->comm,current->pid,(u64)page,page->flags,page_evictable(page),page_mapped(page),PageWriteback(page),PageDirty(page),PageLocked(page));
		if (!trylock_page(page)){
                        if(open_shrink_printk)
		            printk("2_1:%s %s %d page:0x%llx page->flags:0x%lx trylock_page\n",__func__,current->comm,current->pid,(u64)page,page->flags);
			goto keep;
                }
		VM_BUG_ON_PAGE(PageActive(page), page);

		sc->nr_scanned++;

		if (unlikely(!page_evictable(page)))
			goto activate_locked;

		if (!sc->may_unmap && page_mapped(page)){
                        if(open_shrink_printk)
		            printk("3:%s %s %d page:0x%llx page->flags:0x%lx trylock_page\n",__func__,current->comm,current->pid,(u64)page,page->flags);
			goto keep_locked;
                }
		/* Double the slab pressure for mapped and swapcache pages */
		if ((page_mapped(page) || PageSwapCache(page)) &&
		    !(PageAnon(page) && !PageSwapBacked(page)))
			sc->nr_scanned++;

		may_enter_fs = (sc->gfp_mask & __GFP_FS) ||
			(PageSwapCache(page) && (sc->gfp_mask & __GFP_IO));

		/*
		 * The number of dirty pages determines if a node is marked
		 * reclaim_congested which affects wait_iff_congested. kswapd
		 * will stall and start writing pages if the tail of the LRU
		 * is all dirty unqueued pages.
		 */
		page_check_dirty_writeback(page, &dirty, &writeback);
		if (dirty || writeback)
			nr_dirty++;

		if (dirty && !writeback)
			nr_unqueued_dirty++;

		/*
		 * Treat this page as congested if the underlying BDI is or if
		 * pages are cycling through the LRU so quickly that the
		 * pages marked for immediate reclaim are making it to the
		 * end of the LRU a second time.
		 */
		mapping = page_mapping(page);
		if (((dirty || writeback) && mapping &&
		     inode_write_congested(mapping->host)) ||
		    (writeback && PageReclaim(page)))
			nr_congested++;

		/*
		 * If a page at the tail of the LRU is under writeback, there
		 * are three cases to consider.
		 *
		 * 1) If reclaim is encountering an excessive number of pages
		 *    under writeback and this page is both under writeback and
		 *    PageReclaim then it indicates that pages are being queued
		 *    for IO but are being recycled through the LRU before the
		 *    IO can complete. Waiting on the page itself risks an
		 *    indefinite stall if it is impossible to writeback the
		 *    page due to IO error or disconnected storage so instead
		 *    note that the LRU is being scanned too quickly and the
		 *    caller can stall after page list has been processed.
		 *
		 * 2) Global or new memcg reclaim encounters a page that is
		 *    not marked for immediate reclaim, or the caller does not
		 *    have __GFP_FS (or __GFP_IO if it's simply going to swap,
		 *    not to fs). In this case mark the page for immediate
		 *    reclaim and continue scanning.
		 *
		 *    Require may_enter_fs because we would wait on fs, which
		 *    may not have submitted IO yet. And the loop driver might
		 *    enter reclaim, and deadlock if it waits on a page for
		 *    which it is needed to do the write (loop masks off
		 *    __GFP_IO|__GFP_FS for this reason); but more thought
		 *    would probably show more reasons.
		 *
		 * 3) Legacy memcg encounters a page that is already marked
		 *    PageReclaim. memcg does not have any dirty pages
		 *    throttling so we could easily OOM just because too many
		 *    pages are in writeback and there is nothing else to
		 *    reclaim. Wait for the writeback to complete.
		 *
		 * In cases 1) and 2) we activate the pages to get them out of
		 * the way while we continue scanning for clean pages on the
		 * inactive list and refilling from the active list. The
		 * observation here is that waiting for disk writes is more
		 * expensive than potentially causing reloads down the line.
		 * Since they're marked for immediate reclaim, they won't put
		 * memory pressure on the cache working set any longer than it
		 * takes to write them to disk.
		 */
		if (PageWriteback(page)) {
                        if(open_shrink_printk)
		            printk("4:%s %s %d page:0x%llx page->flags:0x%lx PageWriteback:%d\n",__func__,current->comm,current->pid,(u64)page,page->flags,PageWriteback(page));
			/* Case 1 above */
			if (current_is_kswapd() &&
			    PageReclaim(page) &&
			    test_bit(PGDAT_WRITEBACK, &pgdat->flags)) {
                                if(open_shrink_printk)
   		                    printk("5:%s %s %d page:0x%llx page->flags:0x%lx PageWriteback Case 1\n",__func__,current->comm,current->pid,(u64)page,page->flags);

				nr_immediate++;
				goto activate_locked;

			/* Case 2 above */
			} else if (sane_reclaim(sc) ||
			    !PageReclaim(page) || !may_enter_fs) {
				/*
				 * This is slightly racy - end_page_writeback()
				 * might have just cleared PageReclaim, then
				 * setting PageReclaim here end up interpreted
				 * as PageReadahead - but that does not matter
				 * enough to care.  What we do want is for this
				 * page to have PageReclaim set next time memcg
				 * reclaim reaches the tests above, so it will
				 * then wait_on_page_writeback() to avoid OOM;
				 * and it's also appropriate in global reclaim.
				 */
                                if(open_shrink_printk)
   		                    printk("6:%s %s %d page:0x%llx page->flags:0x%lx PageWriteback Case 2\n",__func__,current->comm,current->pid,(u64)page,page->flags);
				SetPageReclaim(page);
				nr_writeback++;
				goto activate_locked;

			/* Case 3 above */
			} else {
                                if(open_shrink_printk)
   		                    printk("7:%s %s %d page:0x%llx page->flags:0x%lx PageWriteback Case 3\n",__func__,current->comm,current->pid,(u64)page,page->flags);
				unlock_page(page);
				wait_on_page_writeback(page);
				/* then go back and try same page again */
				list_add_tail(&page->lru, page_list);
				continue;
			}
		}

		if (!force_reclaim)
			references = page_check_references(page, sc);

                //if(open_shrink_printk)
   		//    printk("8:%s %s %d page:0x%llx page->flags:0x%lx references:%d\n",__func__,current->comm,current->pid,(u64)page,page->flags,references);
		switch (references) {
		case PAGEREF_ACTIVATE:
			goto activate_locked;
		case PAGEREF_KEEP:
			nr_ref_keep++;
			goto keep_locked;
		case PAGEREF_RECLAIM:
		case PAGEREF_RECLAIM_CLEAN:
			; /* try to reclaim the page below */
		}

		/*
		 * Anonymous process memory has backing store?
		 * Try to allocate it some swap space here.
		 * Lazyfree page could be freed directly
		 */
		if (PageAnon(page) && PageSwapBacked(page)) {
			if (!PageSwapCache(page)) {
				if (!(sc->gfp_mask & __GFP_IO))
					goto keep_locked;
				if (PageTransHuge(page)) {
					/* cannot split THP, skip it */
					if (!can_split_huge_page(page, NULL))
						goto activate_locked;
					/*
					 * Split pages without a PMD map right
					 * away. Chances are some or all of the
					 * tail pages can be freed without IO.
					 */
					if (!compound_mapcount(page) &&
					    split_huge_page_to_list(page,
								    page_list))
						goto activate_locked;
				}
				if (!add_to_swap(page)) {
					if (!PageTransHuge(page))
						goto activate_locked;
					/* Fallback to swap normal pages */
					if (split_huge_page_to_list(page,
								    page_list))
						goto activate_locked;
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
					count_vm_event(THP_SWPOUT_FALLBACK);
#endif
					if (!add_to_swap(page))
						goto activate_locked;
				}

				may_enter_fs = 1;

				/* Adding to swap updated mapping */
				mapping = page_mapping(page);
			}
		} else if (unlikely(PageTransHuge(page))) {
			/* Split file THP */
			if (split_huge_page_to_list(page, page_list))
				goto keep_locked;
		}

		/*
		 * The page is mapped into the page tables of one or more
		 * processes. Try to unmap it here.
		 */
		if (page_mapped(page)) {
			enum ttu_flags flags = ttu_flags | TTU_BATCH_FLUSH;
                        if(open_shrink_printk)
   		        printk("8_1:%s %s %d page:0x%llx page->flags:0x%lx page_mapped\n",__func__,current->comm,current->pid,(u64)page,page->flags);

			if (unlikely(PageTransHuge(page)))
				flags |= TTU_SPLIT_HUGE_PMD;
			if (!try_to_unmap(page, flags)) {
				nr_unmap_fail++;
				goto activate_locked;
			}
		}

		if (PageDirty(page)) {
                         if(open_shrink_printk)
   		             printk("9:%s %s %d page:0x%llx page->flags:0x%lx PageDirty;%d\n",__func__,current->comm,current->pid,(u64)page,page->flags,PageDirty(page));
			/*
			 * Only kswapd can writeback filesystem pages
			 * to avoid risk of stack overflow. But avoid
			 * injecting inefficient single-page IO into
			 * flusher writeback as much as possible: only
			 * write pages when we've encountered many
			 * dirty pages, and when we've already scanned
			 * the rest of the LRU for clean pages and see
			 * the same dirty pages again (PageReclaim).
			 */
			if (page_is_file_cache(page) &&
			    (!current_is_kswapd() || !PageReclaim(page) ||
			     !test_bit(PGDAT_DIRTY, &pgdat->flags))) {
				/*
				 * Immediately reclaim when written back.
				 * Similar in principal to deactivate_page()
				 * except we already have the page isolated
				 * and know it's dirty
				 */
                                if(open_shrink_printk)
   		                    printk("10:%s %s %d page:0x%llx page->flags:0x%lx PageDirty ->activate_locked\n",__func__,current->comm,current->pid,(u64)page,page->flags);
				inc_node_page_state(page, NR_VMSCAN_IMMEDIATE);
				SetPageReclaim(page);

				goto activate_locked;
			}

			if (references == PAGEREF_RECLAIM_CLEAN)
				goto keep_locked;
			if (!may_enter_fs)
				goto keep_locked;
			if (!sc->may_writepage)
				goto keep_locked;

                        if(open_shrink_printk)
   		            printk("11:%s %s %d page:0x%llx page->flags:0x%lx PageDirty ->pageout\n",__func__,current->comm,current->pid,(u64)page,page->flags);
			/*
			 * Page is dirty. Flush the TLB if a writable entry
			 * potentially exists to avoid CPU writes after IO
			 * starts and then write it out here.
			 */
			try_to_unmap_flush_dirty();
			switch (pageout(page, mapping, sc)) {
			case PAGE_KEEP:
                                if(open_shrink_printk)
   		                    printk("12:%s %s %d page:0x%llx page->flags:0x%lx PageDirty ->keep_locked\n",__func__,current->comm,current->pid,(u64)page,page->flags);
				goto keep_locked;
			case PAGE_ACTIVATE:
                                if(open_shrink_printk)
   		                    printk("13:%s %s %d page:0x%llx page->flags:0x%lx PageDirty ->activate_locked\n",__func__,current->comm,current->pid,(u64)page,page->flags);
				goto activate_locked;
			case PAGE_SUCCESS:
                                if(open_shrink_printk)
   		                    printk("14:%s %s %d page:0x%llx page->flags:0x%lx PageDirty PageWriteback:%d PageDirty:%d PG_locked:%d\n",__func__,current->comm,current->pid,(u64)page,page->flags,PageWriteback(page),PageDirty(page),PageLocked(page));
				if (PageWriteback(page))
					goto keep;
				if (PageDirty(page))
					goto keep;

				/*
				 * A synchronous write - probably a ramdisk.  Go
				 * ahead and try to reclaim the page.
				 */
				if (!trylock_page(page))
					goto keep;
				if (PageDirty(page) || PageWriteback(page))
					goto keep_locked;
				mapping = page_mapping(page);
                                if(open_shrink_printk)
   		                    printk("15:%s %s %d page:0x%llx page->flags:0x%lx \n",__func__,current->comm,current->pid,(u64)page,page->flags);
			case PAGE_CLEAN:
                                if(open_shrink_printk)
   		                    printk("16:%s %s %d page:0x%llx page->flags:0x%lx PageDirty PAGE_CLEAN\n",__func__,current->comm,current->pid,(u64)page,page->flags);
				; /* try to free the page below */
			}
		}

		/*
		 * If the page has buffers, try to free the buffer mappings
		 * associated with this page. If we succeed we try to free
		 * the page as well.
		 *
		 * We do this even if the page is PageDirty().
		 * try_to_release_page() does not perform I/O, but it is
		 * possible for a page to have PageDirty set, but it is actually
		 * clean (all its buffers are clean).  This happens if the
		 * buffers were written out directly, with submit_bh(). ext3
		 * will do this, as well as the blockdev mapping.
		 * try_to_release_page() will discover that cleanness and will
		 * drop the buffers and mark the page clean - it can be freed.
		 *
		 * Rarely, pages can have buffers and no ->mapping.  These are
		 * the pages which were not successfully invalidated in
		 * truncate_complete_page().  We try to drop those buffers here
		 * and if that worked, and the page is no longer mapped into
		 * process address space (page_count == 1) it can be freed.
		 * Otherwise, leave the page on the LRU so it is swappable.
		 */
		if (page_has_private(page)) {
                        if(open_shrink_printk)
   		            printk("17:%s %s %d page:0x%llx page->flags:0x%lx mapping:0x%llx page_has_private\n",__func__,current->comm,current->pid,(u64)page,page->flags,(u64)mapping);

			if (!try_to_release_page(page, sc->gfp_mask)){
                                if(open_shrink_printk)
   		                    printk("18:%s %s %d page:0x%llx page->flags:0x%lx activate_locked\n",__func__,current->comm,current->pid,(u64)page,page->flags);
				goto activate_locked;
			}
			if (!mapping && page_count(page) == 1) {
				unlock_page(page);
				if (put_page_testzero(page)){
                                        if(open_shrink_printk)
   		                            printk("18_1:%s %s %d page:0x%llx page->flags:0x%lx put_page_testzero\n",__func__,current->comm,current->pid,(u64)page,page->flags);
					goto free_it;
				}
				else {
                                        if(open_shrink_printk)
   		                            printk("18_2:%s %s %d page:0x%llx page->flags:0x%lx page_has_private\n",__func__,current->comm,current->pid,(u64)page,page->flags);
					/*
					 * rare race with speculative reference.
					 * the speculative reference will free
					 * this page shortly, so we may
					 * increment nr_reclaimed here (and
					 * leave it off the LRU).
					 */
					nr_reclaimed++;
					continue;
				}
			}
		}

		if (PageAnon(page) && !PageSwapBacked(page)) {
			/* follow __remove_mapping for reference */
			if (!page_ref_freeze(page, 1))
				goto keep_locked;
			if (PageDirty(page)) {
				page_ref_unfreeze(page, 1);
				goto keep_locked;
			}

			count_vm_event(PGLAZYFREED);
			count_memcg_page_event(page, PGLAZYFREED);
		} else if (!mapping || !__remove_mapping(mapping, page, true)){
                        if(open_shrink_printk)
   		            printk("19:%s %s %d page:0x%llx page->flags:0x%lx mapping:0x%llx keep_locked\n",__func__,current->comm,current->pid,(u64)page,page->flags,(u64)mapping);
			goto keep_locked;
                }
		unlock_page(page);
free_it:
		nr_reclaimed++;
                if(open_shrink_printk)
   		    printk("20:%s %s %d page:0x%llx page->flags:0x%lx free_it nr_reclaimed++\n",__func__,current->comm,current->pid,(u64)page,page->flags);

		/*
		 * Is there need to periodically free_page_list? It would
		 * appear not as the counts should be low
		 */
		if (unlikely(PageTransHuge(page))) {
			mem_cgroup_uncharge(page);
			(*get_compound_page_dtor(page))(page);
		} else
			list_add(&page->lru, &free_pages);
		continue;

activate_locked:
                if(open_shrink_printk)
   		    printk("21:%s %s %d page:0x%llx page->flags:0x%lx activate_locked:\n",__func__,current->comm,current->pid,(u64)page,page->flags);
		/* Not a candidate for swapping, so reclaim swap space. */
		if (PageSwapCache(page) && (mem_cgroup_swap_full(page) ||
						PageMlocked(page)))
			try_to_free_swap(page);
		VM_BUG_ON_PAGE(PageActive(page), page);
		if (!PageMlocked(page)) {
			SetPageActive(page);
			pgactivate++;
			count_memcg_page_event(page, PGACTIVATE);
		}
keep_locked:
                if(open_shrink_printk)
   		    printk("22:%s %s %d page:0x%llx page->flags:0x%lx keep_locked:\n",__func__,current->comm,current->pid,(u64)page,page->flags);
		unlock_page(page);
keep:
                if(open_shrink_printk)
   		    printk("23:%s %s %d page:0x%llx page->flags:0x%lx keep:\n",__func__,current->comm,current->pid,(u64)page,page->flags);
		list_add(&page->lru, &ret_pages);
		VM_BUG_ON_PAGE(PageLRU(page) || PageUnevictable(page), page);
	}

	mem_cgroup_uncharge_list(&free_pages);
	try_to_unmap_flush();
	free_unref_page_list(&free_pages);

	list_splice(&ret_pages, page_list);
	count_vm_events(PGACTIVATE, pgactivate);

	if (stat) {
		stat->nr_dirty = nr_dirty;
		stat->nr_congested = nr_congested;
		stat->nr_unqueued_dirty = nr_unqueued_dirty;
		stat->nr_writeback = nr_writeback;
		stat->nr_immediate = nr_immediate;
		stat->nr_activate = pgactivate;
		stat->nr_ref_keep = nr_ref_keep;
		stat->nr_unmap_fail = nr_unmap_fail;
	}
	return nr_reclaimed;
}

unsigned long reclaim_clean_pages_from_list(struct zone *zone,
					    struct list_head *page_list)
{
	struct scan_control sc = {
		.gfp_mask = GFP_KERNEL,
		.priority = DEF_PRIORITY,
		.may_unmap = 1,
	};
	unsigned long ret;
	struct page *page, *next;
	LIST_HEAD(clean_pages);

	list_for_each_entry_safe(page, next, page_list, lru) {
		if (page_is_file_cache(page) && !PageDirty(page) &&
		    !__PageMovable(page) && !PageUnevictable(page)) {
			ClearPageActive(page);
			list_move(&page->lru, &clean_pages);
		}
	}

	ret = shrink_page_list(&clean_pages, zone->zone_pgdat, &sc,
			TTU_IGNORE_ACCESS, NULL, true);
	list_splice(&clean_pages, page_list);
	mod_node_page_state(zone->zone_pgdat, NR_ISOLATED_FILE, -ret);
	return ret;
}

/*
 * Attempt to remove the specified page from its LRU.  Only take this page
 * if it is of the appropriate PageActive status.  Pages which are being
 * freed elsewhere are also ignored.
 *
 * page:	page to consider
 * mode:	one of the LRU isolation modes defined above
 *
 * returns 0 on success, -ve errno on failure.
 */
int __isolate_lru_page(struct page *page, isolate_mode_t mode)
{
	int ret = -EINVAL;

	/* Only take pages on the LRU. */
	if (!PageLRU(page))
		return ret;

	/* Compaction should not handle unevictable pages but CMA can do so */
	if (PageUnevictable(page) && !(mode & ISOLATE_UNEVICTABLE))
		return ret;

	ret = -EBUSY;

	/*
	 * To minimise LRU disruption, the caller can indicate that it only
	 * wants to isolate pages it will be able to operate on without
	 * blocking - clean pages for the most part.
	 *
	 * ISOLATE_ASYNC_MIGRATE is used to indicate that it only wants to pages
	 * that it is possible to migrate without blocking
	 */
	if (mode & ISOLATE_ASYNC_MIGRATE) {
		/* All the caller can do on PageWriteback is block */
		if (PageWriteback(page))
			return ret;

		if (PageDirty(page)) {
			struct address_space *mapping;
			bool migrate_dirty;

			/*
			 * Only pages without mappings or that have a
			 * ->migratepage callback are possible to migrate
			 * without blocking. However, we can be racing with
			 * truncation so it's necessary to lock the page
			 * to stabilise the mapping as truncation holds
			 * the page lock until after the page is removed
			 * from the page cache.
			 */
			if (!trylock_page(page))
				return ret;

			mapping = page_mapping(page);
			migrate_dirty = !mapping || mapping->a_ops->migratepage;
			unlock_page(page);
			if (!migrate_dirty)
				return ret;
		}
	}

	if ((mode & ISOLATE_UNMAPPED) && page_mapped(page))
		return ret;

	if (likely(get_page_unless_zero(page))) {
		/*
		 * Be careful not to clear PageLRU until after we're
		 * sure the page is not being freed elsewhere -- the
		 * page release code relies on it.
		 */
		ClearPageLRU(page);
		ret = 0;
	}

	return ret;
}


/*
 * Update LRU sizes after isolating pages. The LRU size updates must
 * be complete before mem_cgroup_update_lru_size due to a santity check.
 */
static __always_inline void update_lru_sizes(struct lruvec *lruvec,
			enum lru_list lru, unsigned long *nr_zone_taken)
{
	int zid;

	for (zid = 0; zid < MAX_NR_ZONES; zid++) {
		if (!nr_zone_taken[zid])
			continue;

		__update_lru_size(lruvec, lru, zid, -nr_zone_taken[zid]);
#ifdef CONFIG_MEMCG
		mem_cgroup_update_lru_size(lruvec, lru, zid, -nr_zone_taken[zid]);
#endif
	}

}

/*
 * zone_lru_lock is heavily contended.  Some of the functions that
 * shrink the lists perform better by taking out a batch of pages
 * and working on them outside the LRU lock.
 *
 * For pagecache intensive workloads, this function is the hottest
 * spot in the kernel (apart from copy_*_user functions).
 *
 * Appropriate locks must be held before calling this function.
 *
 * @nr_to_scan:	The number of eligible pages to look through on the list.
 * @lruvec:	The LRU vector to pull pages from.
 * @dst:	The temp list to put pages on to.
 * @nr_scanned:	The number of pages that were scanned.
 * @sc:		The scan_control struct for this reclaim session
 * @mode:	One of the LRU isolation modes
 * @lru:	LRU list id for isolating
 *
 * returns how many pages were moved onto *@dst.
 */
static unsigned long isolate_lru_pages(unsigned long nr_to_scan,
		struct lruvec *lruvec, struct list_head *dst,
		unsigned long *nr_scanned, struct scan_control *sc,
		isolate_mode_t mode, enum lru_list lru)
{
	struct list_head *src = &lruvec->lists[lru];
	unsigned long nr_taken = 0;
	unsigned long nr_zone_taken[MAX_NR_ZONES] = { 0 };
	unsigned long nr_skipped[MAX_NR_ZONES] = { 0, };
	unsigned long skipped = 0;
	unsigned long scan, total_scan, nr_pages;
	LIST_HEAD(pages_skipped);

	scan = 0;
	for (total_scan = 0;
	     scan < nr_to_scan && nr_taken < nr_to_scan && !list_empty(src);
	     total_scan++) {
		struct page *page;

		page = lru_to_page(src);
		prefetchw_prev_lru_page(page, src, flags);

                if(open_shrink_printk)
		    printk("1:%s %s %d page:0x%llx page->flags:0x%lx page_zonenum(page):%d sc->reclaim_idx:%d\n",__func__,current->comm,current->pid,(u64)page,page->flags,page_zonenum(page),sc->reclaim_idx);

		VM_BUG_ON_PAGE(!PageLRU(page), page);

		if (page_zonenum(page) > sc->reclaim_idx) {
			//如果page要从file lru链表剔除，并且page是async_shrink_page，则要从file lru链表取出page的上一个page更新到async_shrink_page
			update_async_shrink_page(page);
			list_move(&page->lru, &pages_skipped);
			nr_skipped[page_zonenum(page)]++;
			continue;
		}

		/*
		 * Do not count skipped pages because that makes the function
		 * return with no isolated pages if the LRU mostly contains
		 * ineligible pages.  This causes the VM to not reclaim any
		 * pages, triggering a premature OOM.
		 */
		scan++;
		switch (__isolate_lru_page(page, mode)) {
		case 0:
			nr_pages = hpage_nr_pages(page);
			nr_taken += nr_pages;
			nr_zone_taken[page_zonenum(page)] += nr_pages;
			//如果page要从file lru链表剔除，并且page是async_shrink_page，则要从file lru链表取出page的上一个page更新到async_shrink_page
			update_async_shrink_page(page);
			list_move(&page->lru, dst);
			break;

		case -EBUSY:
                        if(open_shrink_printk)
		            printk("2:%s %s %d page:0x%llx page->flags:0x%lx EBUSY\n",__func__,current->comm,current->pid,(u64)page,page->flags);
			//如果page要从file lru链表剔除，并且page是async_shrink_page，则要从file lru链表取出page的上一个page更新到async_shrink_page
			update_async_shrink_page(page);
			/* else it is being freed elsewhere */
			list_move(&page->lru, src);
			continue;

		default:
			BUG();
		}
	}

	/*
	 * Splice any skipped pages to the start of the LRU list. Note that
	 * this disrupts the LRU order when reclaiming for lower zones but
	 * we cannot splice to the tail. If we did then the SWAP_CLUSTER_MAX
	 * scanning would soon rescan the same pages to skip and put the
	 * system at risk of premature OOM.
	 */
	if (!list_empty(&pages_skipped)) {
		int zid;

                if(open_shrink_printk)
		    printk("3:%s %s %d\n",__func__,current->comm,current->pid);
		list_splice(&pages_skipped, src);
		for (zid = 0; zid < MAX_NR_ZONES; zid++) {
			if (!nr_skipped[zid])
				continue;

			__count_zid_vm_events(PGSCAN_SKIP, zid, nr_skipped[zid]);
			skipped += nr_skipped[zid];
		}
	}
	*nr_scanned = total_scan;
	trace_mm_vmscan_lru_isolate(sc->reclaim_idx, sc->order, nr_to_scan,
				    total_scan, skipped, nr_taken, mode, lru);
	update_lru_sizes(lruvec, lru, nr_zone_taken);
	return nr_taken;
}

/**
 * isolate_lru_page - tries to isolate a page from its LRU list
 * @page: page to isolate from its LRU list
 *
 * Isolates a @page from an LRU list, clears PageLRU and adjusts the
 * vmstat statistic corresponding to whatever LRU list the page was on.
 *
 * Returns 0 if the page was removed from an LRU list.
 * Returns -EBUSY if the page was not on an LRU list.
 *
 * The returned page will have PageLRU() cleared.  If it was found on
 * the active list, it will have PageActive set.  If it was found on
 * the unevictable list, it will have the PageUnevictable bit set. That flag
 * may need to be cleared by the caller before letting the page go.
 *
 * The vmstat statistic corresponding to the list on which the page was
 * found will be decremented.
 *
 * Restrictions:
 *
 * (1) Must be called with an elevated refcount on the page. This is a
 *     fundamentnal difference from isolate_lru_pages (which is called
 *     without a stable reference).
 * (2) the lru_lock must not be held.
 * (3) interrupts must be enabled.
 */
int isolate_lru_page(struct page *page)
{
	int ret = -EBUSY;

	VM_BUG_ON_PAGE(!page_count(page), page);
	WARN_RATELIMIT(PageTail(page), "trying to isolate tail page");

	if (PageLRU(page)) {
		struct zone *zone = page_zone(page);
		struct lruvec *lruvec;

		spin_lock_irq(zone_lru_lock(zone));
		lruvec = mem_cgroup_page_lruvec(page, zone->zone_pgdat);
		if (PageLRU(page)) {
			int lru = page_lru(page);
			get_page(page);
			ClearPageLRU(page);
			del_page_from_lru_list(page, lruvec, lru);
			ret = 0;
		}
		spin_unlock_irq(zone_lru_lock(zone));
	}
	return ret;
}

/*
 * A direct reclaimer may isolate SWAP_CLUSTER_MAX pages from the LRU list and
 * then get resheduled. When there are massive number of tasks doing page
 * allocation, such sleeping direct reclaimers may keep piling up on each CPU,
 * the LRU list will go small and be scanned faster than necessary, leading to
 * unnecessary swapping, thrashing and OOM.
 */
static int too_many_isolated(struct pglist_data *pgdat, int file,
		struct scan_control *sc)
{
	unsigned long inactive, isolated;

	if (current_is_kswapd())
		return 0;

	if (!sane_reclaim(sc))
		return 0;

	if (file) {
		inactive = node_page_state(pgdat, NR_INACTIVE_FILE);
		isolated = node_page_state(pgdat, NR_ISOLATED_FILE);
	} else {
		inactive = node_page_state(pgdat, NR_INACTIVE_ANON);
		isolated = node_page_state(pgdat, NR_ISOLATED_ANON);
	}

	/*
	 * GFP_NOIO/GFP_NOFS callers are allowed to isolate more pages, so they
	 * won't get blocked by normal direct-reclaimers, forming a circular
	 * deadlock.
	 */
	if ((sc->gfp_mask & (__GFP_IO | __GFP_FS)) == (__GFP_IO | __GFP_FS))
		inactive >>= 3;

	return isolated > inactive;
}

static noinline_for_stack void
putback_inactive_pages(struct lruvec *lruvec, struct list_head *page_list)
{
	struct zone_reclaim_stat *reclaim_stat = &lruvec->reclaim_stat;
	struct pglist_data *pgdat = lruvec_pgdat(lruvec);
	LIST_HEAD(pages_to_free);

	/*
	 * Put back any unfreeable pages.
	 */
	while (!list_empty(page_list)) {
		struct page *page = lru_to_page(page_list);
		int lru;

                if(open_shrink_printk)
		    printk("1:%s %s %d page:0x%llx page->flags:0x%lx\n",__func__,current->comm,current->pid,(u64)page,page->flags);
		VM_BUG_ON_PAGE(PageLRU(page), page);
		list_del(&page->lru);
		if (unlikely(!page_evictable(page))) {
			spin_unlock_irq(&pgdat->lru_lock);
			putback_lru_page(page);
			spin_lock_irq(&pgdat->lru_lock);
			continue;
		}

		lruvec = mem_cgroup_page_lruvec(page, pgdat);

		SetPageLRU(page);
		lru = page_lru(page);
		add_page_to_lru_list(page, lruvec, lru);

		if (is_active_lru(lru)) {
			int file = is_file_lru(lru);
			int numpages = hpage_nr_pages(page);
			reclaim_stat->recent_rotated[file] += numpages;
		}
		if (put_page_testzero(page)) {
                        if(open_shrink_printk)
		            printk("2:%s %s %d put_page_testzero page:0x%llx page->flags:0x%lx PageCompound:%d\n",__func__,current->comm,current->pid,(u64)page,page->flags,PageCompound(page));
			__ClearPageLRU(page);
			__ClearPageActive(page);
			del_page_from_lru_list(page, lruvec, lru);

			if (unlikely(PageCompound(page))) {
				spin_unlock_irq(&pgdat->lru_lock);
				mem_cgroup_uncharge(page);
				(*get_compound_page_dtor(page))(page);
				spin_lock_irq(&pgdat->lru_lock);
			} else
				list_add(&page->lru, &pages_to_free);
		}
	}

	/*
	 * To save our caller's stack, now use input list for pages to free.
	 */
	list_splice(&pages_to_free, page_list);
}

/*
 * If a kernel thread (such as nfsd for loop-back mounts) services
 * a backing device by writing to the page cache it sets PF_LESS_THROTTLE.
 * In that case we should only throttle if the backing device it is
 * writing to is congested.  In other cases it is safe to throttle.
 */
static int current_may_throttle(void)
{
	return !(current->flags & PF_LESS_THROTTLE) ||
		current->backing_dev_info == NULL ||
		bdi_write_congested(current->backing_dev_info);
}

/*
 * shrink_inactive_list() is a helper for shrink_node().  It returns the number
 * of reclaimed pages
 */
static noinline_for_stack unsigned long
shrink_inactive_list(unsigned long nr_to_scan, struct lruvec *lruvec,
		     struct scan_control *sc, enum lru_list lru)
{
	LIST_HEAD(page_list);
	unsigned long nr_scanned;
	unsigned long nr_reclaimed = 0;
	unsigned long nr_taken;
	struct reclaim_stat stat = {};
	isolate_mode_t isolate_mode = 0;
	int file = is_file_lru(lru);
	struct pglist_data *pgdat = lruvec_pgdat(lruvec);
	struct zone_reclaim_stat *reclaim_stat = &lruvec->reclaim_stat;
	bool stalled = false;

        if(open_shrink_printk)
	    printk("1:%s %s %d nr_to_scan:%ld lru:%d lruvec:0x%llx isolate_mode:0x%x\n",__func__,current->comm,current->pid,nr_to_scan,lru,(u64)lruvec,isolate_mode);
	while (unlikely(too_many_isolated(pgdat, file, sc))) {
		if (stalled)
			return 0;

		/* wait a bit for the reclaimer. */
		msleep(100);
		stalled = true;

		/* We are about to die and free our memory. Return now. */
		if (fatal_signal_pending(current))
			return SWAP_CLUSTER_MAX;
	}

	lru_add_drain();

	if (!sc->may_unmap)
		isolate_mode |= ISOLATE_UNMAPPED;

	spin_lock_irq(&pgdat->lru_lock);

	nr_taken = isolate_lru_pages(nr_to_scan, lruvec, &page_list,
				     &nr_scanned, sc, isolate_mode, lru);

	__mod_node_page_state(pgdat, NR_ISOLATED_ANON + file, nr_taken);
	reclaim_stat->recent_scanned[file] += nr_taken;

	if (current_is_kswapd()) {
		if (global_reclaim(sc))
			__count_vm_events(PGSCAN_KSWAPD, nr_scanned);
		count_memcg_events(lruvec_memcg(lruvec), PGSCAN_KSWAPD,
				   nr_scanned);
	} else {
		if (global_reclaim(sc))
			__count_vm_events(PGSCAN_DIRECT, nr_scanned);
		count_memcg_events(lruvec_memcg(lruvec), PGSCAN_DIRECT,
				   nr_scanned);
	}
	spin_unlock_irq(&pgdat->lru_lock);

	if (nr_taken == 0)
		return 0;

	nr_reclaimed = shrink_page_list(&page_list, pgdat, sc, 0,
				&stat, false);

	spin_lock_irq(&pgdat->lru_lock);

	if (current_is_kswapd()) {
		if (global_reclaim(sc))
			__count_vm_events(PGSTEAL_KSWAPD, nr_reclaimed);
		count_memcg_events(lruvec_memcg(lruvec), PGSTEAL_KSWAPD,
				   nr_reclaimed);
	} else {
		if (global_reclaim(sc))
			__count_vm_events(PGSTEAL_DIRECT, nr_reclaimed);
		count_memcg_events(lruvec_memcg(lruvec), PGSTEAL_DIRECT,
				   nr_reclaimed);
	}
        if(open_shrink_printk)
	    printk("2:%s %s %d nr_reclaimed:%ld ->putback_inactive_pages()\n",__func__,current->comm,current->pid,nr_reclaimed);
	putback_inactive_pages(lruvec, &page_list);

	__mod_node_page_state(pgdat, NR_ISOLATED_ANON + file, -nr_taken);

	spin_unlock_irq(&pgdat->lru_lock);

	mem_cgroup_uncharge_list(&page_list);
	free_unref_page_list(&page_list);

	/*
	 * If dirty pages are scanned that are not queued for IO, it
	 * implies that flushers are not doing their job. This can
	 * happen when memory pressure pushes dirty pages to the end of
	 * the LRU before the dirty limits are breached and the dirty
	 * data has expired. It can also happen when the proportion of
	 * dirty pages grows not through writes but through memory
	 * pressure reclaiming all the clean cache. And in some cases,
	 * the flushers simply cannot keep up with the allocation
	 * rate. Nudge the flusher threads in case they are asleep.
	 */
	if (stat.nr_unqueued_dirty == nr_taken)
		wakeup_flusher_threads(WB_REASON_VMSCAN);

	sc->nr.dirty += stat.nr_dirty;
	sc->nr.congested += stat.nr_congested;
	sc->nr.unqueued_dirty += stat.nr_unqueued_dirty;
	sc->nr.writeback += stat.nr_writeback;
	sc->nr.immediate += stat.nr_immediate;
	sc->nr.taken += nr_taken;
	if (file)
		sc->nr.file_taken += nr_taken;
        
	if(open_shrink_printk)
	    printk("3:%s %s %d sc->nr.dirty:%d sc->nr.congested:%d sc->nr.unqueued_dirty:%d sc->nr.writeback:%d sc->nr.immediate:%d sc->nr.taken:%d\n",__func__,current->comm,current->pid,sc->nr.dirty,sc->nr.congested,sc->nr.unqueued_dirty,sc->nr.writeback,sc->nr.immediate,sc->nr.taken);
	trace_mm_vmscan_lru_shrink_inactive(pgdat->node_id,
			nr_scanned, nr_reclaimed, &stat, sc->priority, file);
	return nr_reclaimed;
}

/*
 * This moves pages from the active list to the inactive list.
 *
 * We move them the other way if the page is referenced by one or more
 * processes, from rmap.
 *
 * If the pages are mostly unmapped, the processing is fast and it is
 * appropriate to hold zone_lru_lock across the whole operation.  But if
 * the pages are mapped, the processing is slow (page_referenced()) so we
 * should drop zone_lru_lock around each page.  It's impossible to balance
 * this, so instead we remove the pages from the LRU while processing them.
 * It is safe to rely on PG_active against the non-LRU pages in here because
 * nobody will play with that bit on a non-LRU page.
 *
 * The downside is that we have to touch page->_refcount against each page.
 * But we had to alter page->flags anyway.
 *
 * Returns the number of pages moved to the given lru.
 */

static unsigned move_active_pages_to_lru(struct lruvec *lruvec,
				     struct list_head *list,
				     struct list_head *pages_to_free,
				     enum lru_list lru)
{
	struct pglist_data *pgdat = lruvec_pgdat(lruvec);
	struct page *page;
	int nr_pages;
	int nr_moved = 0;

	while (!list_empty(list)) {
		page = lru_to_page(list);
                if(open_shrink_printk)
		    printk("1:%s %s %d lru:%d page:0x%llx page->flags:0x%lx\n",__func__,current->comm,current->pid,lru,(u64)page,page->flags);
		lruvec = mem_cgroup_page_lruvec(page, pgdat);

		VM_BUG_ON_PAGE(PageLRU(page), page);
		SetPageLRU(page);

		nr_pages = hpage_nr_pages(page);
		update_lru_size(lruvec, lru, page_zonenum(page), nr_pages);
		list_move(&page->lru, &lruvec->lists[lru]);

		if (put_page_testzero(page)) {
                        if(open_shrink_printk)
		            printk("2:%s %s %d put_page_testzero page:0x%llx page->flags:0x%lx PageCompound:%d\n",__func__,current->comm,current->pid,(u64)page,page->flags,PageCompound(page));
			__ClearPageLRU(page);
			__ClearPageActive(page);
			del_page_from_lru_list(page, lruvec, lru);

			if (unlikely(PageCompound(page))) {
				spin_unlock_irq(&pgdat->lru_lock);
				mem_cgroup_uncharge(page);
				(*get_compound_page_dtor(page))(page);
				spin_lock_irq(&pgdat->lru_lock);
			} else
				list_add(&page->lru, pages_to_free);
		} else {
			nr_moved += nr_pages;
		}
	}

	if (!is_active_lru(lru)) {
		__count_vm_events(PGDEACTIVATE, nr_moved);
		count_memcg_events(lruvec_memcg(lruvec), PGDEACTIVATE,
				   nr_moved);
	}

	return nr_moved;
}

static void shrink_active_list(unsigned long nr_to_scan,
			       struct lruvec *lruvec,
			       struct scan_control *sc,
			       enum lru_list lru)
{
	unsigned long nr_taken;
	unsigned long nr_scanned;
	unsigned long vm_flags;
	LIST_HEAD(l_hold);	/* The pages which were snipped off */
	LIST_HEAD(l_active);
	LIST_HEAD(l_inactive);
	struct page *page;
	struct zone_reclaim_stat *reclaim_stat = &lruvec->reclaim_stat;
	unsigned nr_deactivate, nr_activate;
	unsigned nr_rotated = 0;
	isolate_mode_t isolate_mode = 0;
	int file = is_file_lru(lru);
	struct pglist_data *pgdat = lruvec_pgdat(lruvec);

	lru_add_drain();

	if (!sc->may_unmap)
		isolate_mode |= ISOLATE_UNMAPPED;

        if(open_shrink_printk)
		printk("1:%s %s %d nr_to_scan:%ld isolate_mode:%d\n",__func__,current->comm,current->pid,nr_to_scan,isolate_mode);
	spin_lock_irq(&pgdat->lru_lock);

	nr_taken = isolate_lru_pages(nr_to_scan, lruvec, &l_hold,
				     &nr_scanned, sc, isolate_mode, lru);

	__mod_node_page_state(pgdat, NR_ISOLATED_ANON + file, nr_taken);
	reclaim_stat->recent_scanned[file] += nr_taken;

	__count_vm_events(PGREFILL, nr_scanned);
	count_memcg_events(lruvec_memcg(lruvec), PGREFILL, nr_scanned);

	spin_unlock_irq(&pgdat->lru_lock);

	while (!list_empty(&l_hold)) {
		cond_resched();
		page = lru_to_page(&l_hold);
		list_del(&page->lru);

		if (unlikely(!page_evictable(page))) {
			putback_lru_page(page);
			continue;
		}

		if (unlikely(buffer_heads_over_limit)) {
			if (page_has_private(page) && trylock_page(page)) {
				if (page_has_private(page))
					try_to_release_page(page, 0);
				unlock_page(page);
			}
		}

		if (page_referenced(page, 0, sc->target_mem_cgroup,
				    &vm_flags)) {
			nr_rotated += hpage_nr_pages(page);
			/*
			 * Identify referenced, file-backed active pages and
			 * give them one more trip around the active list. So
			 * that executable code get better chances to stay in
			 * memory under moderate memory pressure.  Anon pages
			 * are not likely to be evicted by use-once streaming
			 * IO, plus JVM can create lots of anon VM_EXEC pages,
			 * so we ignore them here.
			 */
			if ((vm_flags & VM_EXEC) && page_is_file_cache(page)) {
				list_add(&page->lru, &l_active);
				continue;
			}
		}

		ClearPageActive(page);	/* we are de-activating */
		SetPageWorkingset(page);
		list_add(&page->lru, &l_inactive);
	}

	/*
	 * Move pages back to the lru list.
	 */
	spin_lock_irq(&pgdat->lru_lock);
	/*
	 * Count referenced pages from currently used mappings as rotated,
	 * even though only some of them are actually re-activated.  This
	 * helps balance scan pressure between file and anonymous pages in
	 * get_scan_count.
	 */
	reclaim_stat->recent_rotated[file] += nr_rotated;

        if(open_shrink_printk)
		printk("2:%s %s %d ->move_active_pages_to_lru()\n",__func__,current->comm,current->pid);
	nr_activate = move_active_pages_to_lru(lruvec, &l_active, &l_hold, lru);
	nr_deactivate = move_active_pages_to_lru(lruvec, &l_inactive, &l_hold, lru - LRU_ACTIVE);
	__mod_node_page_state(pgdat, NR_ISOLATED_ANON + file, -nr_taken);
	spin_unlock_irq(&pgdat->lru_lock);

	mem_cgroup_uncharge_list(&l_hold);
	free_unref_page_list(&l_hold);
	trace_mm_vmscan_lru_shrink_active(pgdat->node_id, nr_taken, nr_activate,
			nr_deactivate, nr_rotated, sc->priority, file);
}

/*
 * The inactive anon list should be small enough that the VM never has
 * to do too much work.
 *
 * The inactive file list should be small enough to leave most memory
 * to the established workingset on the scan-resistant active list,
 * but large enough to avoid thrashing the aggregate readahead window.
 *
 * Both inactive lists should also be large enough that each inactive
 * page has a chance to be referenced again before it is reclaimed.
 *
 * If that fails and refaulting is observed, the inactive list grows.
 *
 * The inactive_ratio is the target ratio of ACTIVE to INACTIVE pages
 * on this LRU, maintained by the pageout code. An inactive_ratio
 * of 3 means 3:1 or 25% of the pages are kept on the inactive list.
 *
 * total     target    max
 * memory    ratio     inactive
 * -------------------------------------
 *   10MB       1         5MB
 *  100MB       1        50MB
 *    1GB       3       250MB
 *   10GB      10       0.9GB
 *  100GB      31         3GB
 *    1TB     101        10GB
 *   10TB     320        32GB
 */
static bool inactive_list_is_low(struct lruvec *lruvec, bool file,
				 struct scan_control *sc, bool trace)
{
	enum lru_list active_lru = file * LRU_FILE + LRU_ACTIVE;
	struct pglist_data *pgdat = lruvec_pgdat(lruvec);
	enum lru_list inactive_lru = file * LRU_FILE;
	unsigned long inactive, active;
	unsigned long inactive_ratio;
	unsigned long refaults;
	unsigned long gb;

	/*
	 * If we don't have swap space, anonymous page deactivation
	 * is pointless.
	 */
	if (!file && !total_swap_pages)
		return false;

	inactive = lruvec_lru_size(lruvec, inactive_lru, sc->reclaim_idx);
	active = lruvec_lru_size(lruvec, active_lru, sc->reclaim_idx);

	/*
	 * When refaults are being observed, it means a new workingset
	 * is being established. Disable active list protection to get
	 * rid of the stale workingset quickly.
	 */
	refaults = lruvec_page_state_local(lruvec, WORKINGSET_ACTIVATE);
	if (file && lruvec->refaults != refaults) {
		inactive_ratio = 0;
	} else {
		gb = (inactive + active) >> (30 - PAGE_SHIFT);
		if (gb)
			inactive_ratio = int_sqrt(10 * gb);
		else
			inactive_ratio = 1;
	}

	if (trace)
		trace_mm_vmscan_inactive_list_is_low(pgdat->node_id, sc->reclaim_idx,
			lruvec_lru_size(lruvec, inactive_lru, MAX_NR_ZONES), inactive,
			lruvec_lru_size(lruvec, active_lru, MAX_NR_ZONES), active,
			inactive_ratio, file);

	if(open_shrink_printk)
	    printk("1:%s %s %d inactive:%ld inactive_ratio:%ld active:%ld\n",__func__,current->comm,current->pid,inactive,inactive_ratio,active);
	return inactive * inactive_ratio < active;
}

static unsigned long shrink_list(enum lru_list lru, unsigned long nr_to_scan,
				 struct lruvec *lruvec, struct scan_control *sc)
{
	if (is_active_lru(lru)) {
		if (inactive_list_is_low(lruvec, is_file_lru(lru), sc, true))
			shrink_active_list(nr_to_scan, lruvec, sc, lru);
		return 0;
	}

	return shrink_inactive_list(nr_to_scan, lruvec, sc, lru);
}

enum scan_balance {
	SCAN_EQUAL,
	SCAN_FRACT,
	SCAN_ANON,
	SCAN_FILE,
};

/*
 * Determine how aggressively the anon and file LRU lists should be
 * scanned.  The relative value of each set of LRU lists is determined
 * by looking at the fraction of the pages scanned we did rotate back
 * onto the active list instead of evict.
 *
 * nr[0] = anon inactive pages to scan; nr[1] = anon active pages to scan
 * nr[2] = file inactive pages to scan; nr[3] = file active pages to scan
 */
static void get_scan_count(struct lruvec *lruvec, struct mem_cgroup *memcg,
			   struct scan_control *sc, unsigned long *nr,
			   unsigned long *lru_pages)
{
	int swappiness = mem_cgroup_swappiness(memcg);
	struct zone_reclaim_stat *reclaim_stat = &lruvec->reclaim_stat;
	u64 fraction[2];
	u64 denominator = 0;	/* gcc */
	struct pglist_data *pgdat = lruvec_pgdat(lruvec);
	unsigned long anon_prio, file_prio;
	enum scan_balance scan_balance;
	unsigned long anon, file;
	unsigned long ap, fp;
	enum lru_list lru;

	/* If we have no swap space, do not bother scanning anon pages. */
	if (!sc->may_swap || mem_cgroup_get_nr_swap_pages(memcg) <= 0) {
		if(open_shrink_printk)
		    printk("1:%s %s %d sc->may_swap:%d\n",__func__,current->comm,current->pid,sc->may_swap);
		scan_balance = SCAN_FILE;
		goto out;
	}

	/*
	 * Global reclaim will swap to prevent OOM even with no
	 * swappiness, but memcg users want to use this knob to
	 * disable swapping for individual groups completely when
	 * using the memory controller's swap limit feature would be
	 * too expensive.
	 */
	if (!global_reclaim(sc) && !swappiness) {
		if(open_shrink_printk)
		    printk("2:%s %s %d\n",__func__,current->comm,current->pid);
		scan_balance = SCAN_FILE;
		goto out;
	}

	/*
	 * Do not apply any pressure balancing cleverness when the
	 * system is close to OOM, scan both anon and file equally
	 * (unless the swappiness setting disagrees with swapping).
	 */
	if (!sc->priority && swappiness) {
		if(open_shrink_printk)
		    printk("3:%s %s %d\n",__func__,current->comm,current->pid);
		scan_balance = SCAN_EQUAL;
		goto out;
	}

	/*
	 * Prevent the reclaimer from falling into the cache trap: as
	 * cache pages start out inactive, every cache fault will tip
	 * the scan balance towards the file LRU.  And as the file LRU
	 * shrinks, so does the window for rotation from references.
	 * This means we have a runaway feedback loop where a tiny
	 * thrashing file LRU becomes infinitely more attractive than
	 * anon pages.  Try to detect this based on file LRU size.
	 */
	if (global_reclaim(sc)) {
		unsigned long pgdatfile;
		unsigned long pgdatfree;
		int z;
		unsigned long total_high_wmark = 0;

		if(open_shrink_printk)
		    printk("4:%s %s %d\n",__func__,current->comm,current->pid);
		pgdatfree = sum_zone_node_page_state(pgdat->node_id, NR_FREE_PAGES);
		pgdatfile = node_page_state(pgdat, NR_ACTIVE_FILE) +
			   node_page_state(pgdat, NR_INACTIVE_FILE);

		for (z = 0; z < MAX_NR_ZONES; z++) {
			struct zone *zone = &pgdat->node_zones[z];
			if (!managed_zone(zone))
				continue;

			total_high_wmark += high_wmark_pages(zone);
		}

		if (unlikely(pgdatfile + pgdatfree <= total_high_wmark)) {
			/*
			 * Force SCAN_ANON if there are enough inactive
			 * anonymous pages on the LRU in eligible zones.
			 * Otherwise, the small LRU gets thrashed.
			 */
			if (!inactive_list_is_low(lruvec, false, sc, false) &&
			    lruvec_lru_size(lruvec, LRU_INACTIVE_ANON, sc->reclaim_idx)
					>> sc->priority) {
				scan_balance = SCAN_ANON;
				goto out;
			}
		}
	}

	/*
	 * If there is enough inactive page cache, i.e. if the size of the
	 * inactive list is greater than that of the active list *and* the
	 * inactive list actually has some pages to scan on this priority, we
	 * do not reclaim anything from the anonymous working set right now.
	 * Without the second condition we could end up never scanning an
	 * lruvec even if it has plenty of old anonymous pages unless the
	 * system is under heavy pressure.
	 */
	if (!inactive_list_is_low(lruvec, true, sc, false) &&
	    lruvec_lru_size(lruvec, LRU_INACTIVE_FILE, sc->reclaim_idx) >> sc->priority) {
		scan_balance = SCAN_FILE;
		goto out;
	}

	scan_balance = SCAN_FRACT;

	/*
	 * With swappiness at 100, anonymous and file have the same priority.
	 * This scanning priority is essentially the inverse of IO cost.
	 */
	anon_prio = swappiness;
	file_prio = 200 - anon_prio;

	/*
	 * OK, so we have swap space and a fair amount of page cache
	 * pages.  We use the recently rotated / recently scanned
	 * ratios to determine how valuable each cache is.
	 *
	 * Because workloads change over time (and to avoid overflow)
	 * we keep these statistics as a floating average, which ends
	 * up weighing recent references more than old ones.
	 *
	 * anon in [0], file in [1]
	 */

	anon  = lruvec_lru_size(lruvec, LRU_ACTIVE_ANON, MAX_NR_ZONES) +
		lruvec_lru_size(lruvec, LRU_INACTIVE_ANON, MAX_NR_ZONES);
	file  = lruvec_lru_size(lruvec, LRU_ACTIVE_FILE, MAX_NR_ZONES) +
		lruvec_lru_size(lruvec, LRU_INACTIVE_FILE, MAX_NR_ZONES);

	spin_lock_irq(&pgdat->lru_lock);
	if (unlikely(reclaim_stat->recent_scanned[0] > anon / 4)) {
		reclaim_stat->recent_scanned[0] /= 2;
		reclaim_stat->recent_rotated[0] /= 2;
	}

	if (unlikely(reclaim_stat->recent_scanned[1] > file / 4)) {
		reclaim_stat->recent_scanned[1] /= 2;
		reclaim_stat->recent_rotated[1] /= 2;
	}

	/*
	 * The amount of pressure on anon vs file pages is inversely
	 * proportional to the fraction of recently scanned pages on
	 * each list that were recently referenced and in active use.
	 */
	ap = anon_prio * (reclaim_stat->recent_scanned[0] + 1);
	ap /= reclaim_stat->recent_rotated[0] + 1;

	fp = file_prio * (reclaim_stat->recent_scanned[1] + 1);
	fp /= reclaim_stat->recent_rotated[1] + 1;
	spin_unlock_irq(&pgdat->lru_lock);

	fraction[0] = ap;
	fraction[1] = fp;
	denominator = ap + fp + 1;
	if(open_shrink_printk)
	    printk("5:%s %s %d ap:%ld fp:%ld\n",__func__,current->comm,current->pid,ap,fp);
out:
	*lru_pages = 0;
	for_each_evictable_lru(lru) {
		int file = is_file_lru(lru);
		unsigned long lruvec_size;
		unsigned long scan;
		unsigned long protection;

		lruvec_size = lruvec_lru_size(lruvec, lru, sc->reclaim_idx);
		protection = mem_cgroup_protection(memcg,
						   sc->memcg_low_reclaim);

		if (protection) {
			/*
			 * Scale a cgroup's reclaim pressure by proportioning
			 * its current usage to its memory.low or memory.min
			 * setting.
			 *
			 * This is important, as otherwise scanning aggression
			 * becomes extremely binary -- from nothing as we
			 * approach the memory protection threshold, to totally
			 * nominal as we exceed it.  This results in requiring
			 * setting extremely liberal protection thresholds. It
			 * also means we simply get no protection at all if we
			 * set it too low, which is not ideal.
			 *
			 * If there is any protection in place, we reduce scan
			 * pressure by how much of the total memory used is
			 * within protection thresholds.
			 *
			 * There is one special case: in the first reclaim pass,
			 * we skip over all groups that are within their low
			 * protection. If that fails to reclaim enough pages to
			 * satisfy the reclaim goal, we come back and override
			 * the best-effort low protection. However, we still
			 * ideally want to honor how well-behaved groups are in
			 * that case instead of simply punishing them all
			 * equally. As such, we reclaim them based on how much
			 * memory they are using, reducing the scan pressure
			 * again by how much of the total memory used is under
			 * hard protection.
			 */
			unsigned long cgroup_size = mem_cgroup_size(memcg);

			/* Avoid TOCTOU with earlier protection check */
			cgroup_size = max(cgroup_size, protection);

			scan = lruvec_size - lruvec_size * protection /
				cgroup_size;

			/*
			 * Minimally target SWAP_CLUSTER_MAX pages to keep
			 * reclaim moving forwards, avoiding decremeting
			 * sc->priority further than desirable.
			 */
			scan = max(scan, SWAP_CLUSTER_MAX);
		} else {
			scan = lruvec_size;
		}

		scan >>= sc->priority;

		/*
		 * If the cgroup's already been deleted, make sure to
		 * scrape out the remaining cache.
		 */
		if (!scan && !mem_cgroup_online(memcg))
			scan = min(lruvec_size, SWAP_CLUSTER_MAX);

		switch (scan_balance) {
		case SCAN_EQUAL:
			/* Scan lists relative to size */
			break;
		case SCAN_FRACT:
			/*
			 * Scan types proportional to swappiness and
			 * their relative recent reclaim efficiency.
			 * Make sure we don't miss the last page on
			 * the offlined memory cgroups because of a
			 * round-off error.
			 */
			scan = mem_cgroup_online(memcg) ?
			       div64_u64(scan * fraction[file], denominator) :
			       DIV64_U64_ROUND_UP(scan * fraction[file],
						  denominator);
			break;
		case SCAN_FILE:
		case SCAN_ANON:
			/* Scan one type exclusively */
			if ((scan_balance == SCAN_FILE) != file) {
				lruvec_size = 0;
				scan = 0;
			}
			break;
		default:
			/* Look ma, no brain */
			BUG();
		}

		*lru_pages += lruvec_size;
		nr[lru] = scan;
		if(open_shrink_printk)
		    printk("6:%s %s %d nr[%d]:%ld scan:%ld sc->priority:%d lruvec_size:%ld scan_balance:%d protection:%ld\n",__func__,current->comm,current->pid,lru,nr[lru],scan,sc->priority,lruvec_size,scan_balance,protection);
	}
}

/*
 * This is a basic per-node page freer.  Used by both kswapd and direct reclaim.
 */
static void shrink_node_memcg(struct pglist_data *pgdat, struct mem_cgroup *memcg,
			      struct scan_control *sc, unsigned long *lru_pages)
{
	struct lruvec *lruvec = mem_cgroup_lruvec(memcg, pgdat);
	unsigned long nr[NR_LRU_LISTS];
	unsigned long targets[NR_LRU_LISTS];
	unsigned long nr_to_scan;
	enum lru_list lru;
	unsigned long nr_reclaimed = 0;
	unsigned long nr_to_reclaim = sc->nr_to_reclaim;
	struct blk_plug plug;
	bool scan_adjusted;
		
	if(open_shrink_printk)
	    printk("1:%s %s %d\n",__func__,current->comm,current->pid);

	get_scan_count(lruvec, memcg, sc, nr, lru_pages);

	/* Record the original scan target for proportional adjustments later */
	memcpy(targets, nr, sizeof(nr));

	/*
	 * Global reclaiming within direct reclaim at DEF_PRIORITY is a normal
	 * event that can occur when there is little memory pressure e.g.
	 * multiple streaming readers/writers. Hence, we do not abort scanning
	 * when the requested number of pages are reclaimed when scanning at
	 * DEF_PRIORITY on the assumption that the fact we are direct
	 * reclaiming implies that kswapd is not keeping up and it is best to
	 * do a batch of work at once. For memcg reclaim one check is made to
	 * abort proportional reclaim if either the file or anon lru has already
	 * dropped to zero at the first pass.
	 */
	scan_adjusted = (global_reclaim(sc) && !current_is_kswapd() &&
			 sc->priority == DEF_PRIORITY);

	blk_start_plug(&plug);
	while (nr[LRU_INACTIVE_ANON] || nr[LRU_ACTIVE_FILE] ||
					nr[LRU_INACTIVE_FILE]) {
		unsigned long nr_anon, nr_file, percentage;
		unsigned long nr_scanned;

		for_each_evictable_lru(lru) {
			if (nr[lru]) {
				nr_to_scan = min(nr[lru], SWAP_CLUSTER_MAX);
				nr[lru] -= nr_to_scan;
	                        if(open_shrink_printk)
	                            printk("2:%s %s %d nr_to_scan:%ld lru:%d scan_adjusted:%d lruvec:0x%llx ->shrink_list()\n",__func__,current->comm,current->pid,nr_to_scan,lru,scan_adjusted,(u64)lruvec);

				nr_reclaimed += shrink_list(lru, nr_to_scan,
							    lruvec, sc);
			}
		}

		cond_resched();

		if (nr_reclaimed < nr_to_reclaim || scan_adjusted)
			continue;

		/*
		 * For kswapd and memcg, reclaim at least the number of pages
		 * requested. Ensure that the anon and file LRUs are scanned
		 * proportionally what was requested by get_scan_count(). We
		 * stop reclaiming one LRU and reduce the amount scanning
		 * proportional to the original scan target.
		 */
		nr_file = nr[LRU_INACTIVE_FILE] + nr[LRU_ACTIVE_FILE];
		nr_anon = nr[LRU_INACTIVE_ANON] + nr[LRU_ACTIVE_ANON];

		/*
		 * It's just vindictive to attack the larger once the smaller
		 * has gone to zero.  And given the way we stop scanning the
		 * smaller below, this makes sure that we only make one nudge
		 * towards proportionality once we've got nr_to_reclaim.
		 */
		if (!nr_file || !nr_anon)
			break;

		if (nr_file > nr_anon) {
			unsigned long scan_target = targets[LRU_INACTIVE_ANON] +
						targets[LRU_ACTIVE_ANON] + 1;
			lru = LRU_BASE;
			percentage = nr_anon * 100 / scan_target;
		} else {
			unsigned long scan_target = targets[LRU_INACTIVE_FILE] +
						targets[LRU_ACTIVE_FILE] + 1;
			lru = LRU_FILE;
			percentage = nr_file * 100 / scan_target;
		}

		/* Stop scanning the smaller of the LRU */
		nr[lru] = 0;
		nr[lru + LRU_ACTIVE] = 0;

		/*
		 * Recalculate the other LRU scan count based on its original
		 * scan target and the percentage scanning already complete
		 */
		lru = (lru == LRU_FILE) ? LRU_BASE : LRU_FILE;
		nr_scanned = targets[lru] - nr[lru];
		nr[lru] = targets[lru] * (100 - percentage) / 100;
		nr[lru] -= min(nr[lru], nr_scanned);

		lru += LRU_ACTIVE;
		nr_scanned = targets[lru] - nr[lru];
		nr[lru] = targets[lru] * (100 - percentage) / 100;
		nr[lru] -= min(nr[lru], nr_scanned);

		scan_adjusted = true;
	}
	blk_finish_plug(&plug);
	sc->nr_reclaimed += nr_reclaimed;

	/*
	 * Even if we did not try to evict anon pages at all, we want to
	 * rebalance the anon lru active/inactive ratio.
	 */
	if (inactive_list_is_low(lruvec, false, sc, true)){
	        if(open_shrink_printk)
	            printk("3:%s %s %d ->shrink_active_list\n",__func__,current->comm,current->pid);
		shrink_active_list(SWAP_CLUSTER_MAX, lruvec,
				   sc, LRU_ACTIVE_ANON);
	}
}

/* Use reclaim/compaction for costly allocs or under memory pressure */
static bool in_reclaim_compaction(struct scan_control *sc)
{
	if (IS_ENABLED(CONFIG_COMPACTION) && sc->order &&
			(sc->order > PAGE_ALLOC_COSTLY_ORDER ||
			 sc->priority < DEF_PRIORITY - 2))
		return true;

	return false;
}

/*
 * Reclaim/compaction is used for high-order allocation requests. It reclaims
 * order-0 pages before compacting the zone. should_continue_reclaim() returns
 * true if more pages should be reclaimed such that when the page allocator
 * calls try_to_compact_zone() that it will have enough free pages to succeed.
 * It will give up earlier than that if there is difficulty reclaiming pages.
 */
static inline bool should_continue_reclaim(struct pglist_data *pgdat,
					unsigned long nr_reclaimed,
					struct scan_control *sc)
{
	unsigned long pages_for_compaction;
	unsigned long inactive_lru_pages;
	int z;

	/* If not in reclaim/compaction mode, stop */
	if (!in_reclaim_compaction(sc))
		return false;

	/*
	 * Stop if we failed to reclaim any pages from the last SWAP_CLUSTER_MAX
	 * number of pages that were scanned. This will return to the caller
	 * with the risk reclaim/compaction and the resulting allocation attempt
	 * fails. In the past we have tried harder for __GFP_RETRY_MAYFAIL
	 * allocations through requiring that the full LRU list has been scanned
	 * first, by assuming that zero delta of sc->nr_scanned means full LRU
	 * scan, but that approximation was wrong, and there were corner cases
	 * where always a non-zero amount of pages were scanned.
	 */
	if (!nr_reclaimed)
		return false;

	/* If compaction would go ahead or the allocation would succeed, stop */
	for (z = 0; z <= sc->reclaim_idx; z++) {
		struct zone *zone = &pgdat->node_zones[z];
		if (!managed_zone(zone))
			continue;

		switch (compaction_suitable(zone, sc->order, 0, sc->reclaim_idx)) {
		case COMPACT_SUCCESS:
		case COMPACT_CONTINUE:
			return false;
		default:
			/* check next zone */
			;
		}
	}

	/*
	 * If we have not reclaimed enough pages for compaction and the
	 * inactive lists are large enough, continue reclaiming
	 */
	pages_for_compaction = compact_gap(sc->order);
	inactive_lru_pages = node_page_state(pgdat, NR_INACTIVE_FILE);
	if (get_nr_swap_pages() > 0)
		inactive_lru_pages += node_page_state(pgdat, NR_INACTIVE_ANON);

	return inactive_lru_pages > pages_for_compaction;
}

static bool pgdat_memcg_congested(pg_data_t *pgdat, struct mem_cgroup *memcg)
{
	return test_bit(PGDAT_CONGESTED, &pgdat->flags) ||
		(memcg && memcg_congested(pgdat, memcg));
}

static bool shrink_node(pg_data_t *pgdat, struct scan_control *sc)
{
	struct reclaim_state *reclaim_state = current->reclaim_state;
	unsigned long nr_reclaimed, nr_scanned;
	bool reclaimable = false;

	do {
		struct mem_cgroup *root = sc->target_mem_cgroup;
		unsigned long node_lru_pages = 0;
		struct mem_cgroup *memcg;

		memset(&sc->nr, 0, sizeof(sc->nr));

		nr_reclaimed = sc->nr_reclaimed;
		nr_scanned = sc->nr_scanned;

		memcg = mem_cgroup_iter(root, NULL, NULL);
		do {
			unsigned long lru_pages;
			unsigned long reclaimed;
			unsigned long scanned;

                        if(open_shrink_printk)
    		            printk("1:%s %s %d memcg:0x%llx root:0x%llx sc->nr_reclaimed:%ld sc->nr_scanned:%ld\n",__func__,current->comm,current->pid,(u64)memcg,(u64)root,sc->nr_reclaimed,sc->nr_scanned);
			switch (mem_cgroup_protected(root, memcg)) {
			case MEMCG_PROT_MIN:
				/*
				 * Hard protection.
				 * If there is no reclaimable memory, OOM.
				 */
				continue;
			case MEMCG_PROT_LOW:
				/*
				 * Soft protection.
				 * Respect the protection only as long as
				 * there is an unprotected supply
				 * of reclaimable memory from other cgroups.
				 */
				if (!sc->memcg_low_reclaim) {
					sc->memcg_low_skipped = 1;
					continue;
				}
				memcg_memory_event(memcg, MEMCG_LOW);
				break;
			case MEMCG_PROT_NONE:
				/*
				 * All protection thresholds breached. We may
				 * still choose to vary the scan pressure
				 * applied based on by how much the cgroup in
				 * question has exceeded its protection
				 * thresholds (see get_scan_count).
				 */
				break;
			}

			reclaimed = sc->nr_reclaimed;
			scanned = sc->nr_scanned;
			shrink_node_memcg(pgdat, memcg, sc, &lru_pages);
			node_lru_pages += lru_pages;

			shrink_slab(sc->gfp_mask, pgdat->node_id,
				    memcg, sc->priority);

			/* Record the group's reclaim efficiency */
			vmpressure(sc->gfp_mask, memcg, false,
				   sc->nr_scanned - scanned,
				   sc->nr_reclaimed - reclaimed);

		} while ((memcg = mem_cgroup_iter(root, memcg, NULL)));

		if (reclaim_state) {
			sc->nr_reclaimed += reclaim_state->reclaimed_slab;
			reclaim_state->reclaimed_slab = 0;
		}

		/* Record the subtree's reclaim efficiency */
		vmpressure(sc->gfp_mask, sc->target_mem_cgroup, true,
			   sc->nr_scanned - nr_scanned,
			   sc->nr_reclaimed - nr_reclaimed);

		if (sc->nr_reclaimed - nr_reclaimed)
			reclaimable = true;

		if (current_is_kswapd()) {
			/*
			 * If reclaim is isolating dirty pages under writeback,
			 * it implies that the long-lived page allocation rate
			 * is exceeding the page laundering rate. Either the
			 * global limits are not being effective at throttling
			 * processes due to the page distribution throughout
			 * zones or there is heavy usage of a slow backing
			 * device. The only option is to throttle from reclaim
			 * context which is not ideal as there is no guarantee
			 * the dirtying process is throttled in the same way
			 * balance_dirty_pages() manages.
			 *
			 * Once a node is flagged PGDAT_WRITEBACK, kswapd will
			 * count the number of pages under pages flagged for
			 * immediate reclaim and stall if any are encountered
			 * in the nr_immediate check below.
			 */
			if (sc->nr.writeback && sc->nr.writeback == sc->nr.taken)
				set_bit(PGDAT_WRITEBACK, &pgdat->flags);

			/*
			 * Tag a node as congested if all the dirty pages
			 * scanned were backed by a congested BDI and
			 * wait_iff_congested will stall.
			 */
			if (sc->nr.dirty && sc->nr.dirty == sc->nr.congested)
				set_bit(PGDAT_CONGESTED, &pgdat->flags);

			/* Allow kswapd to start writing pages during reclaim.*/
			if (sc->nr.unqueued_dirty == sc->nr.file_taken)
				set_bit(PGDAT_DIRTY, &pgdat->flags);

			/*
			 * If kswapd scans pages marked marked for immediate
			 * reclaim and under writeback (nr_immediate), it
			 * implies that pages are cycling through the LRU
			 * faster than they are written so also forcibly stall.
			 */
			if (sc->nr.immediate)
				congestion_wait(BLK_RW_ASYNC, HZ/10);
		}

		/*
		 * Legacy memcg will stall in page writeback so avoid forcibly
		 * stalling in wait_iff_congested().
		 */
		if (!global_reclaim(sc) && sane_reclaim(sc) &&
		    sc->nr.dirty && sc->nr.dirty == sc->nr.congested)
			set_memcg_congestion(pgdat, root, true);

		/*
		 * Stall direct reclaim for IO completions if underlying BDIs
		 * and node is congested. Allow kswapd to continue until it
		 * starts encountering unqueued dirty pages or cycling through
		 * the LRU too quickly.
		 */
		if (!sc->hibernation_mode && !current_is_kswapd() &&
		   current_may_throttle() && pgdat_memcg_congested(pgdat, root))
			wait_iff_congested(BLK_RW_ASYNC, HZ/10);

	} while (should_continue_reclaim(pgdat, sc->nr_reclaimed - nr_reclaimed,
					 sc));

	/*
	 * Kswapd gives up on balancing particular nodes after too
	 * many failures to reclaim anything from them and goes to
	 * sleep. On reclaim progress, reset the failure counter. A
	 * successful direct reclaim run will revive a dormant kswapd.
	 */
	if (reclaimable)
		pgdat->kswapd_failures = 0;

	return reclaimable;
}

/*
 * Returns true if compaction should go ahead for a costly-order request, or
 * the allocation would already succeed without compaction. Return false if we
 * should reclaim first.
 */
static inline bool compaction_ready(struct zone *zone, struct scan_control *sc)
{
	unsigned long watermark;
	enum compact_result suitable;

	suitable = compaction_suitable(zone, sc->order, 0, sc->reclaim_idx);
	if (suitable == COMPACT_SUCCESS)
		/* Allocation should succeed already. Don't reclaim. */
		return true;
	if (suitable == COMPACT_SKIPPED)
		/* Compaction cannot yet proceed. Do reclaim. */
		return false;

	/*
	 * Compaction is already possible, but it takes time to run and there
	 * are potentially other callers using the pages just freed. So proceed
	 * with reclaim to make a buffer of free pages available to give
	 * compaction a reasonable chance of completing and allocating the page.
	 * Note that we won't actually reclaim the whole buffer in one attempt
	 * as the target watermark in should_continue_reclaim() is lower. But if
	 * we are already above the high+gap watermark, don't reclaim at all.
	 */
	watermark = high_wmark_pages(zone) + compact_gap(sc->order);

	return zone_watermark_ok_safe(zone, 0, watermark, sc->reclaim_idx);
}

/*
 * This is the direct reclaim path, for page-allocating processes.  We only
 * try to reclaim pages from zones which will satisfy the caller's allocation
 * request.
 *
 * If a zone is deemed to be full of pinned pages then just give it a light
 * scan then give up on it.
 */
static void shrink_zones(struct zonelist *zonelist, struct scan_control *sc)
{
	struct zoneref *z;
	struct zone *zone;
	unsigned long nr_soft_reclaimed;
	unsigned long nr_soft_scanned;
	gfp_t orig_mask;
	pg_data_t *last_pgdat = NULL;

	/*
	 * If the number of buffer_heads in the machine exceeds the maximum
	 * allowed level, force direct reclaim to scan the highmem zone as
	 * highmem pages could be pinning lowmem pages storing buffer_heads
	 */
        if(open_shrink_printk){
            printk("1:%s %s %d\n",__func__,current->comm,current->pid);
	    dump_stack();
	}
	orig_mask = sc->gfp_mask;
	if (buffer_heads_over_limit) {
		sc->gfp_mask |= __GFP_HIGHMEM;
		sc->reclaim_idx = gfp_zone(sc->gfp_mask);
	}

	for_each_zone_zonelist_nodemask(zone, z, zonelist,
					sc->reclaim_idx, sc->nodemask) {
		/*
		 * Take care memory controller reclaiming has small influence
		 * to global LRU.
		 */
		if (global_reclaim(sc)) {
			if (!cpuset_zone_allowed(zone,
						 GFP_KERNEL | __GFP_HARDWALL))
				continue;

			/*
			 * If we already have plenty of memory free for
			 * compaction in this zone, don't free any more.
			 * Even though compaction is invoked for any
			 * non-zero order, only frequent costly order
			 * reclamation is disruptive enough to become a
			 * noticeable problem, like transparent huge
			 * page allocations.
			 */
			if (IS_ENABLED(CONFIG_COMPACTION) &&
			    sc->order > PAGE_ALLOC_COSTLY_ORDER &&
			    compaction_ready(zone, sc)) {
				sc->compaction_ready = true;
				continue;
			}

                        if(open_shrink_printk)
		            printk("2:%s %s %d zone:0x%llx %s\n",__func__,current->comm,current->pid,(u64)zone,zone->name);
			/*
			 * Shrink each node in the zonelist once. If the
			 * zonelist is ordered by zone (not the default) then a
			 * node may be shrunk multiple times but in that case
			 * the user prefers lower zones being preserved.
			 */
			if (zone->zone_pgdat == last_pgdat)
				continue;

			/*
			 * This steals pages from memory cgroups over softlimit
			 * and returns the number of reclaimed pages and
			 * scanned pages. This works for global memory pressure
			 * and balancing, not for a memcg's limit.
			 */
			nr_soft_scanned = 0;
			nr_soft_reclaimed = mem_cgroup_soft_limit_reclaim(zone->zone_pgdat,
						sc->order, sc->gfp_mask,
						&nr_soft_scanned);
			sc->nr_reclaimed += nr_soft_reclaimed;
			sc->nr_scanned += nr_soft_scanned;
			/* need some check for avoid more shrink_zone() */
		}

		/* See comment about same check for global reclaim above */
		if (zone->zone_pgdat == last_pgdat)
			continue;
		last_pgdat = zone->zone_pgdat;
		shrink_node(zone->zone_pgdat, sc);
	}

	/*
	 * Restore to original mask to avoid the impact on the caller if we
	 * promoted it to __GFP_HIGHMEM.
	 */
	sc->gfp_mask = orig_mask;
}

static void snapshot_refaults(struct mem_cgroup *root_memcg, pg_data_t *pgdat)
{
	struct mem_cgroup *memcg;

	memcg = mem_cgroup_iter(root_memcg, NULL, NULL);
	do {
		unsigned long refaults;
		struct lruvec *lruvec;

		lruvec = mem_cgroup_lruvec(memcg, pgdat);
		refaults = lruvec_page_state_local(lruvec, WORKINGSET_ACTIVATE);
		lruvec->refaults = refaults;
	} while ((memcg = mem_cgroup_iter(root_memcg, memcg, NULL)));
}

/*
 * This is the main entry point to direct page reclaim.
 *
 * If a full scan of the inactive list fails to free enough memory then we
 * are "out of memory" and something needs to be killed.
 *
 * If the caller is !__GFP_FS then the probability of a failure is reasonably
 * high - the zone may be full of dirty or under-writeback pages, which this
 * caller can't do much about.  We kick the writeback threads and take explicit
 * naps in the hope that some of these pages can be written.  But if the
 * allocating task holds filesystem locks which prevent writeout this might not
 * work, and the allocation attempt will fail.
 *
 * returns:	0, if no pages reclaimed
 * 		else, the number of pages reclaimed
 */
static unsigned long do_try_to_free_pages(struct zonelist *zonelist,
					  struct scan_control *sc)
{
	int initial_priority = sc->priority;
	pg_data_t *last_pgdat;
	struct zoneref *z;
	struct zone *zone;
retry:
	delayacct_freepages_start();

	if (global_reclaim(sc))
		__count_zid_vm_events(ALLOCSTALL, sc->reclaim_idx, 1);

	do {
		vmpressure_prio(sc->gfp_mask, sc->target_mem_cgroup,
				sc->priority);
		sc->nr_scanned = 0;
		shrink_zones(zonelist, sc);

		if (sc->nr_reclaimed >= sc->nr_to_reclaim)
			break;

		if (sc->compaction_ready)
			break;

		/*
		 * If we're getting trouble reclaiming, start doing
		 * writepage even in laptop mode.
		 */
		if (sc->priority < DEF_PRIORITY - 2)
			sc->may_writepage = 1;
	} while (--sc->priority >= 0);

	last_pgdat = NULL;
	for_each_zone_zonelist_nodemask(zone, z, zonelist, sc->reclaim_idx,
					sc->nodemask) {
		if (zone->zone_pgdat == last_pgdat)
			continue;
		last_pgdat = zone->zone_pgdat;
		snapshot_refaults(sc->target_mem_cgroup, zone->zone_pgdat);
		set_memcg_congestion(last_pgdat, sc->target_mem_cgroup, false);
	}

	delayacct_freepages_end();

	if (sc->nr_reclaimed)
		return sc->nr_reclaimed;

	/* Aborted reclaim to try compaction? don't OOM, then */
	if (sc->compaction_ready)
		return 1;

	/* Untapped cgroup reserves?  Don't OOM, retry. */
	if (sc->memcg_low_skipped) {
		sc->priority = initial_priority;
		sc->memcg_low_reclaim = 1;
		sc->memcg_low_skipped = 0;
		goto retry;
	}

	return 0;
}

static bool allow_direct_reclaim(pg_data_t *pgdat)
{
	struct zone *zone;
	unsigned long pfmemalloc_reserve = 0;
	unsigned long free_pages = 0;
	int i;
	bool wmark_ok;

	if (pgdat->kswapd_failures >= MAX_RECLAIM_RETRIES)
		return true;

	for (i = 0; i <= ZONE_NORMAL; i++) {
		zone = &pgdat->node_zones[i];
		if (!managed_zone(zone))
			continue;

		if (!zone_reclaimable_pages(zone))
			continue;

		pfmemalloc_reserve += min_wmark_pages(zone);
		free_pages += zone_page_state(zone, NR_FREE_PAGES);
	}

	/* If there are no reserves (unexpected config) then do not throttle */
	if (!pfmemalloc_reserve)
		return true;

	wmark_ok = free_pages > pfmemalloc_reserve / 2;

	/* kswapd must be awake if processes are being throttled */
	if (!wmark_ok && waitqueue_active(&pgdat->kswapd_wait)) {
		pgdat->kswapd_classzone_idx = min(pgdat->kswapd_classzone_idx,
						(enum zone_type)ZONE_NORMAL);
		wake_up_interruptible(&pgdat->kswapd_wait);
	}

	return wmark_ok;
}

/*
 * Throttle direct reclaimers if backing storage is backed by the network
 * and the PFMEMALLOC reserve for the preferred node is getting dangerously
 * depleted. kswapd will continue to make progress and wake the processes
 * when the low watermark is reached.
 *
 * Returns true if a fatal signal was delivered during throttling. If this
 * happens, the page allocator should not consider triggering the OOM killer.
 */
static bool throttle_direct_reclaim(gfp_t gfp_mask, struct zonelist *zonelist,
					nodemask_t *nodemask)
{
	struct zoneref *z;
	struct zone *zone;
	pg_data_t *pgdat = NULL;

	/*
	 * Kernel threads should not be throttled as they may be indirectly
	 * responsible for cleaning pages necessary for reclaim to make forward
	 * progress. kjournald for example may enter direct reclaim while
	 * committing a transaction where throttling it could forcing other
	 * processes to block on log_wait_commit().
	 */
	if (current->flags & PF_KTHREAD)
		goto out;

	/*
	 * If a fatal signal is pending, this process should not throttle.
	 * It should return quickly so it can exit and free its memory
	 */
	if (fatal_signal_pending(current))
		goto out;

	/*
	 * Check if the pfmemalloc reserves are ok by finding the first node
	 * with a usable ZONE_NORMAL or lower zone. The expectation is that
	 * GFP_KERNEL will be required for allocating network buffers when
	 * swapping over the network so ZONE_HIGHMEM is unusable.
	 *
	 * Throttling is based on the first usable node and throttled processes
	 * wait on a queue until kswapd makes progress and wakes them. There
	 * is an affinity then between processes waking up and where reclaim
	 * progress has been made assuming the process wakes on the same node.
	 * More importantly, processes running on remote nodes will not compete
	 * for remote pfmemalloc reserves and processes on different nodes
	 * should make reasonable progress.
	 */
	for_each_zone_zonelist_nodemask(zone, z, zonelist,
					gfp_zone(gfp_mask), nodemask) {
		if (zone_idx(zone) > ZONE_NORMAL)
			continue;

		/* Throttle based on the first usable node */
		pgdat = zone->zone_pgdat;
		if (allow_direct_reclaim(pgdat))
			goto out;
		break;
	}

	/* If no zone was usable by the allocation flags then do not throttle */
	if (!pgdat)
		goto out;

	/* Account for the throttling */
	count_vm_event(PGSCAN_DIRECT_THROTTLE);

	/*
	 * If the caller cannot enter the filesystem, it's possible that it
	 * is due to the caller holding an FS lock or performing a journal
	 * transaction in the case of a filesystem like ext[3|4]. In this case,
	 * it is not safe to block on pfmemalloc_wait as kswapd could be
	 * blocked waiting on the same lock. Instead, throttle for up to a
	 * second before continuing.
	 */
	if (!(gfp_mask & __GFP_FS)) {
		wait_event_interruptible_timeout(pgdat->pfmemalloc_wait,
			allow_direct_reclaim(pgdat), HZ);

		goto check_pending;
	}

	/* Throttle until kswapd wakes the process */
	wait_event_killable(zone->zone_pgdat->pfmemalloc_wait,
		allow_direct_reclaim(pgdat));

check_pending:
	if (fatal_signal_pending(current))
		return true;

out:
	return false;
}

unsigned long try_to_free_pages(struct zonelist *zonelist, int order,
				gfp_t gfp_mask, nodemask_t *nodemask)
{
	unsigned long nr_reclaimed;
	struct scan_control sc = {
		.nr_to_reclaim = SWAP_CLUSTER_MAX,
		.gfp_mask = current_gfp_context(gfp_mask),
		.reclaim_idx = gfp_zone(gfp_mask),
		.order = order,
		.nodemask = nodemask,
		.priority = DEF_PRIORITY,
		.may_writepage = !laptop_mode,
		.may_unmap = 1,
		.may_swap = 1,
	};

	/*
	 * Do not enter reclaim if fatal signal was delivered while throttled.
	 * 1 is returned so that the page allocator does not OOM kill at this
	 * point.
	 */
	if (throttle_direct_reclaim(sc.gfp_mask, zonelist, nodemask))
		return 1;

	trace_mm_vmscan_direct_reclaim_begin(order,
				sc.may_writepage,
				sc.gfp_mask,
				sc.reclaim_idx);

	nr_reclaimed = do_try_to_free_pages(zonelist, &sc);

	trace_mm_vmscan_direct_reclaim_end(nr_reclaimed);

	return nr_reclaimed;
}

#ifdef CONFIG_MEMCG

unsigned long mem_cgroup_shrink_node(struct mem_cgroup *memcg,
						gfp_t gfp_mask, bool noswap,
						pg_data_t *pgdat,
						unsigned long *nr_scanned)
{
	struct scan_control sc = {
		.nr_to_reclaim = SWAP_CLUSTER_MAX,
		.target_mem_cgroup = memcg,
		.may_writepage = !laptop_mode,
		.may_unmap = 1,
		.reclaim_idx = MAX_NR_ZONES - 1,
		.may_swap = !noswap,
	};
	unsigned long lru_pages;

	sc.gfp_mask = (gfp_mask & GFP_RECLAIM_MASK) |
			(GFP_HIGHUSER_MOVABLE & ~GFP_RECLAIM_MASK);

	trace_mm_vmscan_memcg_softlimit_reclaim_begin(sc.order,
						      sc.may_writepage,
						      sc.gfp_mask,
						      sc.reclaim_idx);

	/*
	 * NOTE: Although we can get the priority field, using it
	 * here is not a good idea, since it limits the pages we can scan.
	 * if we don't reclaim here, the shrink_node from balance_pgdat
	 * will pick up pages from other mem cgroup's as well. We hack
	 * the priority and make it zero.
	 */
	shrink_node_memcg(pgdat, memcg, &sc, &lru_pages);

	trace_mm_vmscan_memcg_softlimit_reclaim_end(sc.nr_reclaimed);

	*nr_scanned = sc.nr_scanned;
	return sc.nr_reclaimed;
}

unsigned long try_to_free_mem_cgroup_pages(struct mem_cgroup *memcg,
					   unsigned long nr_pages,
					   gfp_t gfp_mask,
					   bool may_swap)
{
	unsigned long nr_reclaimed;
	unsigned long pflags;
	unsigned int noreclaim_flag;
	struct scan_control sc = {
		.nr_to_reclaim = max(nr_pages, SWAP_CLUSTER_MAX),
		.gfp_mask = (current_gfp_context(gfp_mask) & GFP_RECLAIM_MASK) |
				(GFP_HIGHUSER_MOVABLE & ~GFP_RECLAIM_MASK),
		.reclaim_idx = MAX_NR_ZONES - 1,
		.target_mem_cgroup = memcg,
		.priority = DEF_PRIORITY,
		.may_writepage = !laptop_mode,
		.may_unmap = 1,
		.may_swap = may_swap,
	};
	/*
	 * Traverse the ZONELIST_FALLBACK zonelist of the current node to put
	 * equal pressure on all the nodes. This is based on the assumption that
	 * the reclaim does not bail out early.
	 */
	struct zonelist *zonelist = node_zonelist(numa_node_id(), sc.gfp_mask);

	trace_mm_vmscan_memcg_reclaim_begin(0,
					    sc.may_writepage,
					    sc.gfp_mask,
					    sc.reclaim_idx);

	psi_memstall_enter(&pflags);
	noreclaim_flag = memalloc_noreclaim_save();

	nr_reclaimed = do_try_to_free_pages(zonelist, &sc);

	memalloc_noreclaim_restore(noreclaim_flag);
	psi_memstall_leave(&pflags);

	trace_mm_vmscan_memcg_reclaim_end(nr_reclaimed);

	return nr_reclaimed;
}
#endif

static void age_active_anon(struct pglist_data *pgdat,
				struct scan_control *sc)
{
	struct mem_cgroup *memcg;

	if (!total_swap_pages)
		return;

	memcg = mem_cgroup_iter(NULL, NULL, NULL);
	do {
		struct lruvec *lruvec = mem_cgroup_lruvec(memcg, pgdat);

		if (inactive_list_is_low(lruvec, false, sc, true))
			shrink_active_list(SWAP_CLUSTER_MAX, lruvec,
					   sc, LRU_ACTIVE_ANON);

		memcg = mem_cgroup_iter(NULL, memcg, NULL);
	} while (memcg);
}

/*
 * Returns true if there is an eligible zone balanced for the request order
 * and classzone_idx
 */
static bool pgdat_balanced(pg_data_t *pgdat, int order, int classzone_idx)
{
	int i;
	unsigned long mark = -1;
	struct zone *zone;

	for (i = 0; i <= classzone_idx; i++) {
		zone = pgdat->node_zones + i;

		if (!managed_zone(zone))
			continue;

		mark = high_wmark_pages(zone);
		if (zone_watermark_ok_safe(zone, order, mark, classzone_idx))
			return true;
	}

	/*
	 * If a node has no populated zone within classzone_idx, it does not
	 * need balancing by definition. This can happen if a zone-restricted
	 * allocation tries to wake a remote kswapd.
	 */
	if (mark == -1)
		return true;

	return false;
}

/* Clear pgdat state for congested, dirty or under writeback. */
static void clear_pgdat_congested(pg_data_t *pgdat)
{
	clear_bit(PGDAT_CONGESTED, &pgdat->flags);
	clear_bit(PGDAT_DIRTY, &pgdat->flags);
	clear_bit(PGDAT_WRITEBACK, &pgdat->flags);
}

/*
 * Prepare kswapd for sleeping. This verifies that there are no processes
 * waiting in throttle_direct_reclaim() and that watermarks have been met.
 *
 * Returns true if kswapd is ready to sleep
 */
static bool prepare_kswapd_sleep(pg_data_t *pgdat, int order, int classzone_idx)
{
	/*
	 * The throttled processes are normally woken up in balance_pgdat() as
	 * soon as allow_direct_reclaim() is true. But there is a potential
	 * race between when kswapd checks the watermarks and a process gets
	 * throttled. There is also a potential race if processes get
	 * throttled, kswapd wakes, a large process exits thereby balancing the
	 * zones, which causes kswapd to exit balance_pgdat() before reaching
	 * the wake up checks. If kswapd is going to sleep, no process should
	 * be sleeping on pfmemalloc_wait, so wake them now if necessary. If
	 * the wake up is premature, processes will wake kswapd and get
	 * throttled again. The difference from wake ups in balance_pgdat() is
	 * that here we are under prepare_to_wait().
	 */
	if (waitqueue_active(&pgdat->pfmemalloc_wait))
		wake_up_all(&pgdat->pfmemalloc_wait);

	/* Hopeless node, leave it to direct reclaim */
	if (pgdat->kswapd_failures >= MAX_RECLAIM_RETRIES)
		return true;

	if (pgdat_balanced(pgdat, order, classzone_idx)) {
		clear_pgdat_congested(pgdat);
		return true;
	}

	return false;
}

/*
 * kswapd shrinks a node of pages that are at or below the highest usable
 * zone that is currently unbalanced.
 *
 * Returns true if kswapd scanned at least the requested number of pages to
 * reclaim or if the lack of progress was due to pages under writeback.
 * This is used to determine if the scanning priority needs to be raised.
 */
static bool kswapd_shrink_node(pg_data_t *pgdat,
			       struct scan_control *sc)
{
	struct zone *zone;
	int z;

	/* Reclaim a number of pages proportional to the number of zones */
	sc->nr_to_reclaim = 0;
	for (z = 0; z <= sc->reclaim_idx; z++) {
		zone = pgdat->node_zones + z;
		if (!managed_zone(zone))
			continue;

		sc->nr_to_reclaim += max(high_wmark_pages(zone), SWAP_CLUSTER_MAX);
	}

	/*
	 * Historically care was taken to put equal pressure on all zones but
	 * now pressure is applied based on node LRU order.
	 */
	shrink_node(pgdat, sc);

	/*
	 * Fragmentation may mean that the system cannot be rebalanced for
	 * high-order allocations. If twice the allocation size has been
	 * reclaimed then recheck watermarks only at order-0 to prevent
	 * excessive reclaim. Assume that a process requested a high-order
	 * can direct reclaim/compact.
	 */
	if (sc->order && sc->nr_reclaimed >= compact_gap(sc->order))
		sc->order = 0;

	return sc->nr_scanned >= sc->nr_to_reclaim;
}

/*
 * For kswapd, balance_pgdat() will reclaim pages across a node from zones
 * that are eligible for use by the caller until at least one zone is
 * balanced.
 *
 * Returns the order kswapd finished reclaiming at.
 *
 * kswapd scans the zones in the highmem->normal->dma direction.  It skips
 * zones which have free_pages > high_wmark_pages(zone), but once a zone is
 * found to have free_pages <= high_wmark_pages(zone), any page is that zone
 * or lower is eligible for reclaim until at least one usable zone is
 * balanced.
 */
static int balance_pgdat(pg_data_t *pgdat, int order, int classzone_idx)
{
	int i;
	unsigned long nr_soft_reclaimed;
	unsigned long nr_soft_scanned;
	unsigned long pflags;
	struct zone *zone;
	struct scan_control sc = {
		.gfp_mask = GFP_KERNEL,
		.order = order,
		.priority = DEF_PRIORITY,
		.may_writepage = !laptop_mode,
		.may_unmap = 1,
		.may_swap = 1,
	};

	psi_memstall_enter(&pflags);
	__fs_reclaim_acquire();

	count_vm_event(PAGEOUTRUN);

	do {
		unsigned long nr_reclaimed = sc.nr_reclaimed;
		bool raise_priority = true;
		bool ret;

		sc.reclaim_idx = classzone_idx;

		/*
		 * If the number of buffer_heads exceeds the maximum allowed
		 * then consider reclaiming from all zones. This has a dual
		 * purpose -- on 64-bit systems it is expected that
		 * buffer_heads are stripped during active rotation. On 32-bit
		 * systems, highmem pages can pin lowmem memory and shrinking
		 * buffers can relieve lowmem pressure. Reclaim may still not
		 * go ahead if all eligible zones for the original allocation
		 * request are balanced to avoid excessive reclaim from kswapd.
		 */
		if (buffer_heads_over_limit) {
			for (i = MAX_NR_ZONES - 1; i >= 0; i--) {
				zone = pgdat->node_zones + i;
				if (!managed_zone(zone))
					continue;

				sc.reclaim_idx = i;
				break;
			}
		}

		/*
		 * Only reclaim if there are no eligible zones. Note that
		 * sc.reclaim_idx is not used as buffer_heads_over_limit may
		 * have adjusted it.
		 */
		if (pgdat_balanced(pgdat, sc.order, classzone_idx))
			goto out;

		/*
		 * Do some background aging of the anon list, to give
		 * pages a chance to be referenced before reclaiming. All
		 * pages are rotated regardless of classzone as this is
		 * about consistent aging.
		 */
		age_active_anon(pgdat, &sc);

		/*
		 * If we're getting trouble reclaiming, start doing writepage
		 * even in laptop mode.
		 */
		if (sc.priority < DEF_PRIORITY - 2)
			sc.may_writepage = 1;

		/* Call soft limit reclaim before calling shrink_node. */
		sc.nr_scanned = 0;
		nr_soft_scanned = 0;
		nr_soft_reclaimed = mem_cgroup_soft_limit_reclaim(pgdat, sc.order,
						sc.gfp_mask, &nr_soft_scanned);
		sc.nr_reclaimed += nr_soft_reclaimed;

		/*
		 * There should be no need to raise the scanning priority if
		 * enough pages are already being scanned that that high
		 * watermark would be met at 100% efficiency.
		 */
		if (kswapd_shrink_node(pgdat, &sc))
			raise_priority = false;

		/*
		 * If the low watermark is met there is no need for processes
		 * to be throttled on pfmemalloc_wait as they should not be
		 * able to safely make forward progress. Wake them
		 */
		if (waitqueue_active(&pgdat->pfmemalloc_wait) &&
				allow_direct_reclaim(pgdat))
			wake_up_all(&pgdat->pfmemalloc_wait);

		/* Check if kswapd should be suspending */
		__fs_reclaim_release();
		ret = try_to_freeze();
		__fs_reclaim_acquire();
		if (ret || kthread_should_stop())
			break;

		/*
		 * Raise priority if scanning rate is too low or there was no
		 * progress in reclaiming pages
		 */
		nr_reclaimed = sc.nr_reclaimed - nr_reclaimed;
		if (raise_priority || !nr_reclaimed)
			sc.priority--;
	} while (sc.priority >= 1);

	if (!sc.nr_reclaimed)
		pgdat->kswapd_failures++;

out:
	snapshot_refaults(NULL, pgdat);
	__fs_reclaim_release();
	psi_memstall_leave(&pflags);
	/*
	 * Return the order kswapd stopped reclaiming at as
	 * prepare_kswapd_sleep() takes it into account. If another caller
	 * entered the allocator slow path while kswapd was awake, order will
	 * remain at the higher level.
	 */
	return sc.order;
}

/*
 * The pgdat->kswapd_classzone_idx is used to pass the highest zone index to be
 * reclaimed by kswapd from the waker. If the value is MAX_NR_ZONES which is not
 * a valid index then either kswapd runs for first time or kswapd couldn't sleep
 * after previous reclaim attempt (node is still unbalanced). In that case
 * return the zone index of the previous kswapd reclaim cycle.
 */
static enum zone_type kswapd_classzone_idx(pg_data_t *pgdat,
					   enum zone_type prev_classzone_idx)
{
	if (pgdat->kswapd_classzone_idx == MAX_NR_ZONES)
		return prev_classzone_idx;
	return pgdat->kswapd_classzone_idx;
}

static void kswapd_try_to_sleep(pg_data_t *pgdat, int alloc_order, int reclaim_order,
				unsigned int classzone_idx)
{
	long remaining = 0;
	DEFINE_WAIT(wait);

	if (freezing(current) || kthread_should_stop())
		return;

	prepare_to_wait(&pgdat->kswapd_wait, &wait, TASK_INTERRUPTIBLE);

	/*
	 * Try to sleep for a short interval. Note that kcompactd will only be
	 * woken if it is possible to sleep for a short interval. This is
	 * deliberate on the assumption that if reclaim cannot keep an
	 * eligible zone balanced that it's also unlikely that compaction will
	 * succeed.
	 */
	if (prepare_kswapd_sleep(pgdat, reclaim_order, classzone_idx)) {
		/*
		 * Compaction records what page blocks it recently failed to
		 * isolate pages from and skips them in the future scanning.
		 * When kswapd is going to sleep, it is reasonable to assume
		 * that pages and compaction may succeed so reset the cache.
		 */
		reset_isolation_suitable(pgdat);

		/*
		 * We have freed the memory, now we should compact it to make
		 * allocation of the requested order possible.
		 */
		wakeup_kcompactd(pgdat, alloc_order, classzone_idx);

		remaining = schedule_timeout(HZ/10);

		/*
		 * If woken prematurely then reset kswapd_classzone_idx and
		 * order. The values will either be from a wakeup request or
		 * the previous request that slept prematurely.
		 */
		if (remaining) {
			pgdat->kswapd_classzone_idx = kswapd_classzone_idx(pgdat, classzone_idx);
			pgdat->kswapd_order = max(pgdat->kswapd_order, reclaim_order);
		}

		finish_wait(&pgdat->kswapd_wait, &wait);
		prepare_to_wait(&pgdat->kswapd_wait, &wait, TASK_INTERRUPTIBLE);
	}

	/*
	 * After a short sleep, check if it was a premature sleep. If not, then
	 * go fully to sleep until explicitly woken up.
	 */
	if (!remaining &&
	    prepare_kswapd_sleep(pgdat, reclaim_order, classzone_idx)) {
		trace_mm_vmscan_kswapd_sleep(pgdat->node_id);

		/*
		 * vmstat counters are not perfectly accurate and the estimated
		 * value for counters such as NR_FREE_PAGES can deviate from the
		 * true value by nr_online_cpus * threshold. To avoid the zone
		 * watermarks being breached while under pressure, we reduce the
		 * per-cpu vmstat threshold while kswapd is awake and restore
		 * them before going back to sleep.
		 */
		set_pgdat_percpu_threshold(pgdat, calculate_normal_threshold);

		if (!kthread_should_stop())
			schedule();

		set_pgdat_percpu_threshold(pgdat, calculate_pressure_threshold);
	} else {
		if (remaining)
			count_vm_event(KSWAPD_LOW_WMARK_HIT_QUICKLY);
		else
			count_vm_event(KSWAPD_HIGH_WMARK_HIT_QUICKLY);
	}
	finish_wait(&pgdat->kswapd_wait, &wait);
}

/*
 * The background pageout daemon, started as a kernel thread
 * from the init process.
 *
 * This basically trickles out pages so that we have _some_
 * free memory available even if there is no other activity
 * that frees anything up. This is needed for things like routing
 * etc, where we otherwise might have all activity going on in
 * asynchronous contexts that cannot page things out.
 *
 * If there are applications that are active memory-allocators
 * (most normal use), this basically shouldn't matter.
 */
static int kswapd(void *p)
{
	unsigned int alloc_order, reclaim_order;
	unsigned int classzone_idx = MAX_NR_ZONES - 1;
	pg_data_t *pgdat = (pg_data_t*)p;
	struct task_struct *tsk = current;

	struct reclaim_state reclaim_state = {
		.reclaimed_slab = 0,
	};
	const struct cpumask *cpumask = cpumask_of_node(pgdat->node_id);

	if (!cpumask_empty(cpumask))
		set_cpus_allowed_ptr(tsk, cpumask);
	current->reclaim_state = &reclaim_state;

	/*
	 * Tell the memory management that we're a "memory allocator",
	 * and that if we need more memory we should get access to it
	 * regardless (see "__alloc_pages()"). "kswapd" should
	 * never get caught in the normal page freeing logic.
	 *
	 * (Kswapd normally doesn't need memory anyway, but sometimes
	 * you need a small amount of memory in order to be able to
	 * page out something else, and this flag essentially protects
	 * us from recursively trying to free more memory as we're
	 * trying to free the first piece of memory in the first place).
	 */
	tsk->flags |= PF_MEMALLOC | PF_SWAPWRITE | PF_KSWAPD;
	set_freezable();

	pgdat->kswapd_order = 0;
	pgdat->kswapd_classzone_idx = MAX_NR_ZONES;
	for ( ; ; ) {
		bool ret;

		alloc_order = reclaim_order = pgdat->kswapd_order;
		classzone_idx = kswapd_classzone_idx(pgdat, classzone_idx);

kswapd_try_sleep:
		kswapd_try_to_sleep(pgdat, alloc_order, reclaim_order,
					classzone_idx);

		/* Read the new order and classzone_idx */
		alloc_order = reclaim_order = pgdat->kswapd_order;
		classzone_idx = kswapd_classzone_idx(pgdat, classzone_idx);
		pgdat->kswapd_order = 0;
		pgdat->kswapd_classzone_idx = MAX_NR_ZONES;

		ret = try_to_freeze();
		if (kthread_should_stop())
			break;

		/*
		 * We can speed up thawing tasks if we don't call balance_pgdat
		 * after returning from the refrigerator
		 */
		if (ret)
			continue;

		/*
		 * Reclaim begins at the requested order but if a high-order
		 * reclaim fails then kswapd falls back to reclaiming for
		 * order-0. If that happens, kswapd will consider sleeping
		 * for the order it finished reclaiming at (reclaim_order)
		 * but kcompactd is woken to compact for the original
		 * request (alloc_order).
		 */
		trace_mm_vmscan_kswapd_wake(pgdat->node_id, classzone_idx,
						alloc_order);
		reclaim_order = balance_pgdat(pgdat, alloc_order, classzone_idx);
		if (reclaim_order < alloc_order)
			goto kswapd_try_sleep;
	}

	tsk->flags &= ~(PF_MEMALLOC | PF_SWAPWRITE | PF_KSWAPD);
	current->reclaim_state = NULL;

	return 0;
}

/*
 * A zone is low on free memory or too fragmented for high-order memory.  If
 * kswapd should reclaim (direct reclaim is deferred), wake it up for the zone's
 * pgdat.  It will wake up kcompactd after reclaiming memory.  If kswapd reclaim
 * has failed or is not needed, still wake up kcompactd if only compaction is
 * needed.
 */
void wakeup_kswapd(struct zone *zone, gfp_t gfp_flags, int order,
		   enum zone_type classzone_idx)
{
	pg_data_t *pgdat;

	if (!managed_zone(zone))
		return;

	if (!cpuset_zone_allowed(zone, gfp_flags))
		return;
	pgdat = zone->zone_pgdat;

	if (pgdat->kswapd_classzone_idx == MAX_NR_ZONES)
		pgdat->kswapd_classzone_idx = classzone_idx;
	else
		pgdat->kswapd_classzone_idx = max(pgdat->kswapd_classzone_idx,
						  classzone_idx);
	pgdat->kswapd_order = max(pgdat->kswapd_order, order);
	if (!waitqueue_active(&pgdat->kswapd_wait))
		return;

	/* Hopeless node, leave it to direct reclaim if possible */
	if (pgdat->kswapd_failures >= MAX_RECLAIM_RETRIES ||
	    pgdat_balanced(pgdat, order, classzone_idx)) {
		/*
		 * There may be plenty of free memory available, but it's too
		 * fragmented for high-order allocations.  Wake up kcompactd
		 * and rely on compaction_suitable() to determine if it's
		 * needed.  If it fails, it will defer subsequent attempts to
		 * ratelimit its work.
		 */
		if (!(gfp_flags & __GFP_DIRECT_RECLAIM))
			wakeup_kcompactd(pgdat, order, classzone_idx);
		return;
	}

	trace_mm_vmscan_wakeup_kswapd(pgdat->node_id, classzone_idx, order,
				      gfp_flags);
	wake_up_interruptible(&pgdat->kswapd_wait);
}

#ifdef CONFIG_HIBERNATION
/*
 * Try to free `nr_to_reclaim' of memory, system-wide, and return the number of
 * freed pages.
 *
 * Rather than trying to age LRUs the aim is to preserve the overall
 * LRU order by reclaiming preferentially
 * inactive > active > active referenced > active mapped
 */
unsigned long shrink_all_memory(unsigned long nr_to_reclaim)
{
	struct reclaim_state reclaim_state;
	struct scan_control sc = {
		.nr_to_reclaim = nr_to_reclaim,
		.gfp_mask = GFP_HIGHUSER_MOVABLE,
		.reclaim_idx = MAX_NR_ZONES - 1,
		.priority = DEF_PRIORITY,
		.may_writepage = 1,
		.may_unmap = 1,
		.may_swap = 1,
		.hibernation_mode = 1,
	};
	struct zonelist *zonelist = node_zonelist(numa_node_id(), sc.gfp_mask);
	struct task_struct *p = current;
	unsigned long nr_reclaimed;
	unsigned int noreclaim_flag;

	fs_reclaim_acquire(sc.gfp_mask);
	noreclaim_flag = memalloc_noreclaim_save();
	reclaim_state.reclaimed_slab = 0;
	p->reclaim_state = &reclaim_state;

	nr_reclaimed = do_try_to_free_pages(zonelist, &sc);

	p->reclaim_state = NULL;
	memalloc_noreclaim_restore(noreclaim_flag);
	fs_reclaim_release(sc.gfp_mask);

	return nr_reclaimed;
}
#endif /* CONFIG_HIBERNATION */

/* It's optimal to keep kswapds on the same CPUs as their memory, but
   not required for correctness.  So if the last cpu in a node goes
   away, we get changed to run anywhere: as the first one comes back,
   restore their cpu bindings. */
static int kswapd_cpu_online(unsigned int cpu)
{
	int nid;

	for_each_node_state(nid, N_MEMORY) {
		pg_data_t *pgdat = NODE_DATA(nid);
		const struct cpumask *mask;

		mask = cpumask_of_node(pgdat->node_id);

		if (cpumask_any_and(cpu_online_mask, mask) < nr_cpu_ids)
			/* One of our CPUs online: restore mask */
			set_cpus_allowed_ptr(pgdat->kswapd, mask);
	}
	return 0;
}

/*
 * This kswapd start function will be called by init and node-hot-add.
 * On node-hot-add, kswapd will moved to proper cpus if cpus are hot-added.
 */
int kswapd_run(int nid)
{
	pg_data_t *pgdat = NODE_DATA(nid);
	int ret = 0;

	if (pgdat->kswapd)
		return 0;

	pgdat->kswapd = kthread_run(kswapd, pgdat, "kswapd%d", nid);
	if (IS_ERR(pgdat->kswapd)) {
		/* failure at boot is fatal */
		BUG_ON(system_state < SYSTEM_RUNNING);
		pr_err("Failed to start kswapd on node %d\n", nid);
		ret = PTR_ERR(pgdat->kswapd);
		pgdat->kswapd = NULL;
	}
        /*******async_shrink_thread********************************************/
	pgdat->async_shrink_thread = kthread_run(async_shrink_memory,pgdat, "async_shrink_thread%d",nid);
	if(IS_ERR(pgdat->async_shrink_thread)){
		pr_err("Failed to start async_shrink_thread on node %d\n", nid);
		ret = PTR_ERR(pgdat->async_shrink_thread);
		pgdat->async_shrink_thread = NULL;
	}
	return ret;
}

/*
 * Called by memory hotplug when all memory in a node is offlined.  Caller must
 * hold mem_hotplug_begin/end().
 */
void kswapd_stop(int nid)
{
	struct task_struct *kswapd = NODE_DATA(nid)->kswapd;

	if (kswapd) {
		kthread_stop(kswapd);
		NODE_DATA(nid)->kswapd = NULL;
	}
}

static int __init kswapd_init(void)
{
	int nid, ret;

	swap_setup();
	for_each_node_state(nid, N_MEMORY)
 		kswapd_run(nid);
	ret = cpuhp_setup_state_nocalls(CPUHP_AP_ONLINE_DYN,
					"mm/vmscan:online", kswapd_cpu_online,
					NULL);

	WARN_ON(ret < 0);
        /******hot_file_init**************************************/
	hot_file_init();
	return 0;
}

module_init(kswapd_init)

#ifdef CONFIG_NUMA
/*
 * Node reclaim mode
 *
 * If non-zero call node_reclaim when the number of free pages falls below
 * the watermarks.
 */
int node_reclaim_mode __read_mostly;

#define RECLAIM_OFF 0
#define RECLAIM_ZONE (1<<0)	/* Run shrink_inactive_list on the zone */
#define RECLAIM_WRITE (1<<1)	/* Writeout pages during reclaim */
#define RECLAIM_UNMAP (1<<2)	/* Unmap pages during reclaim */

/*
 * Priority for NODE_RECLAIM. This determines the fraction of pages
 * of a node considered for each zone_reclaim. 4 scans 1/16th of
 * a zone.
 */
#define NODE_RECLAIM_PRIORITY 4

/*
 * Percentage of pages in a zone that must be unmapped for node_reclaim to
 * occur.
 */
int sysctl_min_unmapped_ratio = 1;

/*
 * If the number of slab pages in a zone grows beyond this percentage then
 * slab reclaim needs to occur.
 */
int sysctl_min_slab_ratio = 5;

static inline unsigned long node_unmapped_file_pages(struct pglist_data *pgdat)
{
	unsigned long file_mapped = node_page_state(pgdat, NR_FILE_MAPPED);
	unsigned long file_lru = node_page_state(pgdat, NR_INACTIVE_FILE) +
		node_page_state(pgdat, NR_ACTIVE_FILE);

	/*
	 * It's possible for there to be more file mapped pages than
	 * accounted for by the pages on the file LRU lists because
	 * tmpfs pages accounted for as ANON can also be FILE_MAPPED
	 */
	return (file_lru > file_mapped) ? (file_lru - file_mapped) : 0;
}

/* Work out how many page cache pages we can reclaim in this reclaim_mode */
static unsigned long node_pagecache_reclaimable(struct pglist_data *pgdat)
{
	unsigned long nr_pagecache_reclaimable;
	unsigned long delta = 0;

	/*
	 * If RECLAIM_UNMAP is set, then all file pages are considered
	 * potentially reclaimable. Otherwise, we have to worry about
	 * pages like swapcache and node_unmapped_file_pages() provides
	 * a better estimate
	 */
	if (node_reclaim_mode & RECLAIM_UNMAP)
		nr_pagecache_reclaimable = node_page_state(pgdat, NR_FILE_PAGES);
	else
		nr_pagecache_reclaimable = node_unmapped_file_pages(pgdat);

	/* If we can't clean pages, remove dirty pages from consideration */
	if (!(node_reclaim_mode & RECLAIM_WRITE))
		delta += node_page_state(pgdat, NR_FILE_DIRTY);

	/* Watch for any possible underflows due to delta */
	if (unlikely(delta > nr_pagecache_reclaimable))
		delta = nr_pagecache_reclaimable;

	return nr_pagecache_reclaimable - delta;
}

/*
 * Try to free up some pages from this node through reclaim.
 */
static int __node_reclaim(struct pglist_data *pgdat, gfp_t gfp_mask, unsigned int order)
{
	/* Minimum pages needed in order to stay on node */
	const unsigned long nr_pages = 1 << order;
	struct task_struct *p = current;
	struct reclaim_state reclaim_state;
	unsigned int noreclaim_flag;
	struct scan_control sc = {
		.nr_to_reclaim = max(nr_pages, SWAP_CLUSTER_MAX),
		.gfp_mask = current_gfp_context(gfp_mask),
		.order = order,
		.priority = NODE_RECLAIM_PRIORITY,
		.may_writepage = !!(node_reclaim_mode & RECLAIM_WRITE),
		.may_unmap = !!(node_reclaim_mode & RECLAIM_UNMAP),
		.may_swap = 1,
		.reclaim_idx = gfp_zone(gfp_mask),
	};

	cond_resched();
	fs_reclaim_acquire(sc.gfp_mask);
	/*
	 * We need to be able to allocate from the reserves for RECLAIM_UNMAP
	 * and we also need to be able to write out pages for RECLAIM_WRITE
	 * and RECLAIM_UNMAP.
	 */
	noreclaim_flag = memalloc_noreclaim_save();
	p->flags |= PF_SWAPWRITE;
	reclaim_state.reclaimed_slab = 0;
	p->reclaim_state = &reclaim_state;

	if (node_pagecache_reclaimable(pgdat) > pgdat->min_unmapped_pages) {
		/*
		 * Free memory by calling shrink node with increasing
		 * priorities until we have enough memory freed.
		 */
		do {
			shrink_node(pgdat, &sc);
		} while (sc.nr_reclaimed < nr_pages && --sc.priority >= 0);
	}

	p->reclaim_state = NULL;
	current->flags &= ~PF_SWAPWRITE;
	memalloc_noreclaim_restore(noreclaim_flag);
	fs_reclaim_release(sc.gfp_mask);
	return sc.nr_reclaimed >= nr_pages;
}

int node_reclaim(struct pglist_data *pgdat, gfp_t gfp_mask, unsigned int order)
{
	int ret;

	/*
	 * Node reclaim reclaims unmapped file backed pages and
	 * slab pages if we are over the defined limits.
	 *
	 * A small portion of unmapped file backed pages is needed for
	 * file I/O otherwise pages read by file I/O will be immediately
	 * thrown out if the node is overallocated. So we do not reclaim
	 * if less than a specified percentage of the node is used by
	 * unmapped file backed pages.
	 */
	if (node_pagecache_reclaimable(pgdat) <= pgdat->min_unmapped_pages &&
	    node_page_state(pgdat, NR_SLAB_RECLAIMABLE) <= pgdat->min_slab_pages)
		return NODE_RECLAIM_FULL;

	/*
	 * Do not scan if the allocation should not be delayed.
	 */
	if (!gfpflags_allow_blocking(gfp_mask) || (current->flags & PF_MEMALLOC))
		return NODE_RECLAIM_NOSCAN;

	/*
	 * Only run node reclaim on the local node or on nodes that do not
	 * have associated processors. This will favor the local processor
	 * over remote processors and spread off node memory allocations
	 * as wide as possible.
	 */
	if (node_state(pgdat->node_id, N_CPU) && pgdat->node_id != numa_node_id())
		return NODE_RECLAIM_NOSCAN;

	if (test_and_set_bit(PGDAT_RECLAIM_LOCKED, &pgdat->flags))
		return NODE_RECLAIM_NOSCAN;

	ret = __node_reclaim(pgdat, gfp_mask, order);
	clear_bit(PGDAT_RECLAIM_LOCKED, &pgdat->flags);

	if (!ret)
		count_vm_event(PGSCAN_ZONE_RECLAIM_FAILED);

	return ret;
}
#endif

/*
 * page_evictable - test whether a page is evictable
 * @page: the page to test
 *
 * Test whether page is evictable--i.e., should be placed on active/inactive
 * lists vs unevictable list.
 *
 * Reasons page might not be evictable:
 * (1) page's mapping marked unevictable
 * (2) page is part of an mlocked VMA
 *
 */
int page_evictable(struct page *page)
{
	int ret;

	/* Prevent address_space of inode and swap cache from being freed */
	rcu_read_lock();
	ret = !mapping_unevictable(page_mapping(page)) && !PageMlocked(page);
	rcu_read_unlock();
	return ret;
}

/**
 * check_move_unevictable_pages - check pages for evictability and move to
 * appropriate zone lru list
 * @pvec: pagevec with lru pages to check
 *
 * Checks pages for evictability, if an evictable page is in the unevictable
 * lru list, moves it to the appropriate evictable lru list. This function
 * should be only used for lru pages.
 */
void check_move_unevictable_pages(struct pagevec *pvec)
{
	struct lruvec *lruvec;
	struct pglist_data *pgdat = NULL;
	int pgscanned = 0;
	int pgrescued = 0;
	int i;

	for (i = 0; i < pvec->nr; i++) {
		struct page *page = pvec->pages[i];
		struct pglist_data *pagepgdat = page_pgdat(page);

		pgscanned++;
		if (pagepgdat != pgdat) {
			if (pgdat)
				spin_unlock_irq(&pgdat->lru_lock);
			pgdat = pagepgdat;
			spin_lock_irq(&pgdat->lru_lock);
		}
		lruvec = mem_cgroup_page_lruvec(page, pgdat);

		if (!PageLRU(page) || !PageUnevictable(page))
			continue;

		if (page_evictable(page)) {
			enum lru_list lru = page_lru_base_type(page);

			VM_BUG_ON_PAGE(PageActive(page), page);
			ClearPageUnevictable(page);
			del_page_from_lru_list(page, lruvec, LRU_UNEVICTABLE);
			add_page_to_lru_list(page, lruvec, lru);
			pgrescued++;
		}
	}

	if (pgdat) {
		__count_vm_events(UNEVICTABLE_PGRESCUED, pgrescued);
		__count_vm_events(UNEVICTABLE_PGSCANNED, pgscanned);
		spin_unlock_irq(&pgdat->lru_lock);
	}
}
EXPORT_SYMBOL_GPL(check_move_unevictable_pages);
/***********************************************************************************/
static unsigned int async_shrink_free_page(struct pglist_data *pgdat,struct lruvec *lruvec,struct list_head *page_list,
		                           struct scan_control *sc,struct reclaim_stat *stat)
{
    LIST_HEAD(ret_pages);
    LIST_HEAD(free_pages);
    int pgactivate = 0;

    unsigned nr_unqueued_dirty = 0;
    unsigned nr_dirty = 0;
    unsigned nr_congested = 0;
    unsigned nr_reclaimed = 0;
    unsigned nr_writeback = 0;
    unsigned nr_immediate = 0;
    unsigned nr_ref_keep = 0;
    unsigned nr_unmap_fail = 0;

    while (!list_empty(page_list)) {
        struct address_space *mapping;
        struct page *page;
	int may_enter_fs;

        cond_resched();

	page = lru_to_page(page_list);
	list_del(&page->lru);

        //内存回收过程，page又被访问过了，则不能释放该page
	if((!PageIdle(page) || PageYoung(page)))
            goto keep;

	if (!trylock_page(page))
	    goto keep;

        mapping = page_mapping(page);
        may_enter_fs = (sc->gfp_mask & __GFP_FS);

	/****page是witeback标记*********************/
	if (PageWriteback(page)) {
	    if(PageReclaim(page)){
	        nr_immediate++;
	    }
	    else{
	        SetPageReclaim(page);
	        nr_writeback++;
	    }
		if(open_shrink_printk)
		    printk("1:%s %s %d page:0x%llx page->flags:0x%lx PageWriteback;%d\n",__func__,current->comm,current->pid,(u64)page,page->flags,PageWriteback(page));
	    goto activate_locked;
	}

	/****page是脏页*********************/
	if (PageDirty(page)) {
		if(open_shrink_printk)
		    printk("9:%s %s %d page:0x%llx page->flags:0x%lx PageDirty;%d\n",__func__,current->comm,current->pid,(u64)page,page->flags,PageDirty(page));
                nr_dirty++;
	        /*if (page_is_file_cache(page) &&
		    (!current_is_kswapd() || !PageReclaim(page) ||
		     !test_bit(PGDAT_DIRTY, &pgdat->flags))) {

		     if(open_shrink_printk)
			    printk("10:%s %s %d page:0x%llx page->flags:0x%lx PageDirty ->activate_locked\n",__func__,current->comm,current->pid,(u64)page,page->flags);
			inc_node_page_state(page, NR_VMSCAN_IMMEDIATE);
			SetPageReclaim(page);

			goto activate_locked;
		}*/

		//if (references == PAGEREF_RECLAIM_CLEAN)
		//	goto keep_locked;
		if (!may_enter_fs)
			goto keep_locked;
		//if (!sc->may_writepage)
		//	goto keep_locked;

		
		try_to_unmap_flush_dirty();
		switch (pageout(page, mapping, sc)) {
		case PAGE_KEEP:
			if(open_shrink_printk)
			    printk("12:%s %s %d page:0x%llx page->flags:0x%lx PageDirty ->keep_locked\n",__func__,current->comm,current->pid,(u64)page,page->flags);
			goto keep_locked;
		case PAGE_ACTIVATE:
			if(open_shrink_printk)
			    printk("13:%s %s %d page:0x%llx page->flags:0x%lx PageDirty ->activate_locked\n",__func__,current->comm,current->pid,(u64)page,page->flags);
			goto activate_locked;
		case PAGE_SUCCESS:
			if(open_shrink_printk)
			    printk("14:%s %s %d page:0x%llx page->flags:0x%lx PageDirty PageWriteback:%d PageDirty:%d PG_locked:%d\n",__func__,current->comm,current->pid,(u64)page,page->flags,PageWriteback(page),PageDirty(page),PageLocked(page));
			if (PageWriteback(page))
				goto keep;
			if (PageDirty(page))
				goto keep;

			if (!trylock_page(page))
				goto keep;
			if (PageDirty(page) || PageWriteback(page))
				goto keep_locked;
			mapping = page_mapping(page);
			if(open_shrink_printk)
			    printk("15:%s %s %d page:0x%llx page->flags:0x%lx \n",__func__,current->comm,current->pid,(u64)page,page->flags);
		case PAGE_CLEAN:
			if(open_shrink_printk)
			    printk("16:%s %s %d page:0x%llx page->flags:0x%lx PageDirty PAGE_CLEAN\n",__func__,current->comm,current->pid,(u64)page,page->flags);
			; /* try to free the page below */
		}
	}

	/*******释放page的bh******************/
	if (page_has_private(page)) {
		if(open_shrink_printk)
		    printk("17:%s %s %d page:0x%llx page->flags:0x%lx mapping:0x%llx page_has_private\n",__func__,current->comm,current->pid,(u64)page,page->flags,(u64)mapping);

		if (!try_to_release_page(page,sc->gfp_mask)){
			if(open_shrink_printk)
			    printk("18:%s %s %d page:0x%llx page->flags:0x%lx activate_locked\n",__func__,current->comm,current->pid,(u64)page,page->flags);
			goto activate_locked;
		}
		if (!mapping && page_count(page) == 1) {
			unlock_page(page);
			if (put_page_testzero(page)){
				if(open_shrink_printk)
				    printk("18_1:%s %s %d page:0x%llx page->flags:0x%lx put_page_testzero\n",__func__,current->comm,current->pid,(u64)page,page->flags);
				goto free_it;
			}
			else {
				if(open_shrink_printk)
				    printk("18_2:%s %s %d page:0x%llx page->flags:0x%lx page_has_private\n",__func__,current->comm,current->pid,(u64)page,page->flags);

				nr_reclaimed++;
				continue;
			}
		}
	}
        /********把page从radix tree剔除************************/
        if (!mapping || !__remove_mapping(mapping, page, true)){
            if(open_shrink_printk)
            printk("19:%s %s %d page:0x%llx page->flags:0x%lx mapping:0x%llx keep_locked\n",__func__,current->comm,current->pid,(u64)page,page->flags,(u64)mapping);
	    goto keep_locked;
        }


	unlock_page(page);
free_it:
	nr_reclaimed++;
	list_add(&page->lru, &free_pages);
	continue;
activate_locked:
	if (!PageMlocked(page)) {
	     SetPageActive(page);
	     pgactivate++;
	     /*page要添加到active lru链表，这里增加对应的memory cgroup中在active lru链表的page统计数-------------*/
	     count_memcg_page_event(page, PGACTIVATE);
	}
keep_locked:
	unlock_page(page);
keep:
        list_add(&page->lru, &ret_pages);
    }
    mem_cgroup_uncharge_list(&free_pages);
    try_to_unmap_flush();
    free_unref_page_list(&free_pages);

    /*共有pgactivate个page要添加到active lru链表，这里增加全局的在active lru链表的page统计数---------------*/
    list_splice(&ret_pages, page_list);
    count_vm_events(PGACTIVATE, pgactivate);

    if (stat) {
	stat->nr_dirty = nr_dirty;
	stat->nr_congested = nr_congested;
	stat->nr_unqueued_dirty = nr_unqueued_dirty;
	stat->nr_writeback = nr_writeback;
	stat->nr_immediate = nr_immediate;
	stat->nr_activate = pgactivate;
	stat->nr_ref_keep = nr_ref_keep;
	stat->nr_unmap_fail = nr_unmap_fail;
    }
    return nr_reclaimed;
}
void inline update_async_shrink_page(struct page *page)
{
    struct page *async_shrink_page = page_pgdat(page)->async_shrink_page;
    if(unlikely(async_shrink_page && async_shrink_page == page)){
	//当前page要从lru链表剔除，于是要获取page的上一个page保存到async_shrink_page，从而不中断check_idle_page_and_clear 函数中list_for_each_entry_from_reverse遍历链表
        page_pgdat(page)->async_shrink_page = list_prev_entry(page,lru);
	smp_wmb();//把async_shrink_page最新赋值通知给所有cpu，防止其他cpu还是async_shrink_page老的数据
    }
}
static inline int async_isolate_lru_pages(struct scan_control *sc,isolate_mode_t mode,struct page *page,
	                                  struct lruvec *lruvec,enum lru_list lru,struct list_head *dst)
{
        //struct list_head *src = &lruvec->lists[lru];
        unsigned long nr_taken = 0;
	unsigned long nr_zone_taken[MAX_NR_ZONES] = { 0 };
	unsigned long nr_skipped[MAX_NR_ZONES] = { 0, };
	unsigned long skipped = 0;
	unsigned long /*scan, total_scan,*/ nr_pages;
	//LIST_HEAD(pages_skipped);
        int ret = 0;

	//page = lru_to_page(src);
	//prefetchw_prev_lru_page(page, src, flags);
	VM_BUG_ON_PAGE(!PageLRU(page), page);

	/*在这里的isolate_lru_pages函数里，不再把skip page移动来移动去，该page只会在原始lru file链表不动。只会把__count_zid_vm_events统计PGSCAN_SKIP的代码移动到这里*/
	//打印证实，page_zonenum(page)都是0，sc->reclaim_idx如果是默认值的话，就一个内存page都无法回收了!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	if (page_zonenum(page) > sc->reclaim_idx) {
	        int zid;	

  	        if(open_shrink_printk)
		    printk("1:%s %s %d page:0x%llx page->flags:0x%lx page_zonenum(page):%d sc->reclaim_idx:%d\n",__func__,current->comm,current->pid,(u64)page,page->flags,page_zonenum(page),sc->reclaim_idx);
		//list_move(&page->lru, &pages_skipped);
		nr_skipped[page_zonenum(page)]++;
		//continue;
		for (zid = 0; zid < MAX_NR_ZONES; zid++) {
		    if (!nr_skipped[zid])
			    continue;

		    __count_zid_vm_events(PGSCAN_SKIP, zid, nr_skipped[zid]);
		    skipped += nr_skipped[zid];
		}

		return ret;
	}
        /*这里边，__isolate_lru_page里清除page的PageLRU属性，因为要把page从lru链表剔除了*/
	switch (__isolate_lru_page(page, mode)) {
	case 0:
		nr_pages = hpage_nr_pages(page);
		nr_taken += nr_pages;
		nr_zone_taken[page_zonenum(page)] += nr_pages;
		//page原本在lru链表，现在要移动到其他链表，要把page在链表的上一个page保存到async_shrink_page
		update_async_shrink_page(page);
		list_move(&page->lru, dst);
		ret = 1;
		break;

	case -EBUSY:
		if(open_shrink_printk)
		    printk("2:%s %s %d page:0x%llx page->flags:0x%lx EBUSY\n",__func__,current->comm,current->pid,(u64)page,page->flags);
		/* else it is being freed elsewhere */
		//list_move(&page->lru, src);
		break;

	default:
            if(open_shrink_printk)
                printk("3:%s %s %d PageUnevictable:%d PageLRU:%d\n",__func__,current->comm,current->pid,PageUnevictable(page),PageLRU(page));

		BUG();
	}

	/*if (!list_empty(&pages_skipped)) {
		int zid;
		list_splice(&pages_skipped, src);
		for (zid = 0; zid < MAX_NR_ZONES; zid++) {
			if (!nr_skipped[zid])
				continue;

			__count_zid_vm_events(PGSCAN_SKIP, zid, nr_skipped[zid]);
			skipped += nr_skipped[zid];
		}
	}*/

	/*更新 acitve/inactive file 链入链表的page数，减少nr_taken个，因为page将要从lru链表移除*/
	update_lru_sizes(lruvec, lru, nr_zone_taken);
	return ret;
}
/*检查page的空闲状态，返回值 
 * -1:page不是file cache
 *  0:page 是file cache，但page不空闲
 *  1:page 是file cache，且page空闲
 */
static inline int check_page_idle(struct page *page)
{
    if(!PageCompound(page) && page_is_file_cache(page) && !page_mapped(page) && !PageUnevictable(page)){
       if(PageIdle(page) && !PageYoung(page))
           return 1;
       else
	   return 0;
    }
    if(PageUnevictable(page) || PageCompound(page)){
        if(open_shrink_printk)
            printk("1:%s %s %d PageUnevictable:%d PageCompound:%d\n",__func__,current->comm,current->pid,PageUnevictable(page),PageCompound(page));
    }
    return -1;
}
static int async_shrink_active_inactive_list(struct pglist_data *pgdat,struct lruvec *lruvec,enum lru_list lru,
	                                     struct mem_cgroup *memcg)
{
    struct list_head *page_list_head; 
    struct page *page = NULL;
    struct page *prev_page = NULL;
    //get_lru_prev_page置1说明page已经用pgdat->async_shrink_page更新过了，是最新的老的page在lru链表上的上一个page，不要再执行while循环下边的page = list_prev_entry(page, lru)再更新page了。否则get_lru_prev_page是0就要按照老规则执行while循环下边的page = list_prev_entry(page, lru)用老page在lru链表上的上一个page更新page，作为下一轮内存回收while循环中判断的page.
    unsigned int page_idle_status,get_lru_prev_page;
    unsigned int free_page_count,all_page_count,scan_page_count,sleep_count,check_scan_count;
    unsigned long start_time,dx;
    isolate_mode_t isolate_mode = 0;
    LIST_HEAD(free_list);
    unsigned long nr_reclaimed = 0;
    struct reclaim_stat stat = {};
    struct address_space *mapping;

    struct scan_control sc = {
	.gfp_mask = GFP_KERNEL,
	.order = 1,
	.priority = DEF_PRIORITY,
	.may_writepage = 1,
	.may_unmap = 0,
	.may_swap = 0,
	.reclaim_idx = MAX_NR_ZONES - 1
    };


    if(!(lru == LRU_INACTIVE_FILE || lru == LRU_ACTIVE_FILE))
       return 0;

    //all_page_count = lruvec_lru_size(lruvec,lru, MAX_NR_ZONES);
    //if(open_shrink_printk)
    //    printk("1:%s %s %d all_page_count:%d\n",__func__,current->comm,current->pid,all_page_count);

    page_list_head = &lruvec->lists[lru];
    if(list_empty(page_list_head))
        return 0;

    if (!sc.may_unmap)
        isolate_mode |= ISOLATE_UNMAPPED;

    start_time = jiffies;
    free_page_count = 0;
    scan_page_count = 0;
    check_scan_count = 0;
    get_lru_prev_page = 0;
    spin_lock_irq(&pgdat->lru_lock);
    //all_page_count放在加锁里，
    all_page_count = lruvec_lru_size(lruvec,lru, MAX_NR_ZONES);
    pgdat->async_shrink_page = NULL;
    page = lru_to_page(page_list_head);//获取链表尾的page
    //遍历链表，从链表尾开始遍历。不能用 list_for_each_entry_from_reverse()。举例，如果async_isolate_lru_pages函数隔离成功page，则要执行update_async_shrink_page()里的list_prev_entry(page,lru)
    //把page早lru链表的上一个page保存到async_shrink_page。然后page=async_shrink_page。最后再次执行list_for_each_entry_from_reverse()里的
    //list_prev_entry(page,lru)又一次执行把page在lru链表的上一个保存到page。相当于重复执行了list_prev_entry()，跳过了一个page！这样遍历page就乱了，要错过很多page!!!!!!!!!
    //list_for_each_entry_from_reverse(page,page_list_head,lru){
    while(&page->lru != page_list_head && scan_page_count < all_page_count){//当前的page是lru链表头结点时，结束循环 或者 遍历page数超过最大数，结束遍历page
	scan_page_count ++;
	get_lru_prev_page = 0;
	//如果前后两次遍历的page是同一个则触发crash，遇到过这种错误
	if(prev_page == page){
	    panic("prev_page == page :0x%llx\n",(u64)page);
	}
        prev_page = page;

	//如果遍历到的page是free_list头结点则触发crash，遇到过这种问题
	if(&page->lru == &free_list){
	    panic("page is free_list:0x%llx\n",(u64)page);
	}

	//只有page所属块设备的主次设备号是async_shrink_enable指定的，page才允许回收。这是防止内存回收代码bug导致异常回收，损坏重要文件系统
	//page 的mapping 存在是NULL的情况，要做防护
	if(open_shrink_printk1)
	    printk("1:%s %s %d free_page_count:%d scan_page_count:%d page:0x%llx flag:0x%lx page_mapping(page):0x%llx lru:%d page_list_head:0x%llx,all_page_count:%d nr_reclaimed:%ld\n",__func__,current->comm,current->pid,free_page_count,scan_page_count,(u64)page,page->flags,(u64)page_mapping(page),lru,(u64)page_list_head,all_page_count,nr_reclaimed);
        mapping = page_mapping(page);
	if(!mapping || !mapping->host ||async_shrink_enable != mapping->host->i_sb->s_dev)
	//下边这样代码会导致crash，原因是page_mapping(page)返回的mapping没事，但是page_mapping(page)->host->i_sb->s_dev时，page_mapping(page)返回的maping是NULL
	//神奇了，这么近的代码也会发生这种情况。可能这之间page的mapping的文件释放了!!!!!!!!!!!
	//if(!page_mapping(page) || !page_mapping(page)->host ||async_shrink_enable != page_mapping(page)->host->i_sb->s_dev)
	    goto expire_check;

	//检测page是否空闲
	page_idle_status = check_page_idle(page);
	if(page_idle_status == 1){
	    if(open_shrink_printk1)
	        printk("2:%s %s %d free_page_count:%d scan_page_count:%d page:0x%llx flag:0x%lx\n",__func__,current->comm,current->pid,free_page_count,scan_page_count,(u64)page,page->flags);

	    //如果page在async_isolate_lru_pages()中隔离成功到free_list链表，则会把page在lru链表的上一个page保存到pgdat->async_shrink_page
	    pgdat->async_shrink_page = page;
	    //检测该page能否符合内存回收隔离条件，ok的话清理page的lru属性,把page添加到free_list链表，并且令lru链表减少page数，page引用计数加1,返回1
            if(async_isolate_lru_pages(&sc,isolate_mode,page,lruvec,lru,&free_list)){
	        if(PageActive(page))//如果是active page则清理掉
		    ClearPageActive(page);\
                //置1说明async_isolate_lru_pages函数里page隔离成功，已经把page在lru链表上一个page保存到async_shrink_page。
                //get_lru_prev_page = 1;

		//更新全局 NR_ISOLATED_FILE 隔离page数计数，这个不用添加
		//__mod_node_page_state(pgdat, NR_ISOLATED_FILE, nr_taken);
		//reclaim_stat->recent_scanned[1] += nr_taken;------这个与内存回收get_scan_count()统计scan 的page数有关，不添加

		//把page从原链表剔除再添加到free_list链表
		//list_move(&page->lru, &free_list);------移动到了async_isolate_lru_pages函数
		free_page_count ++;
	    }
	}
	//page_idle_status说明page是page cache，但该page不空闲，则设置page空闲
	else if(page_idle_status == 0){
	    //标记page idle
	    SetPageIdle(page);
	    //清理page的Young标记
	    TestClearPageYoung(page);
	}

expire_check:
	dx = jiffies_to_msecs(jiffies - start_time);
	//遍历链表过了20ms，检查一下是否有进程卡在lru_lock锁。但这个方案可能会造成比较大的延迟，于是参考get_scan_count()和isolate_lru_pages()遍历lru链表上的page，改成
	//遍历固定数目的page后，就检测是否有进程卡在lru_lock锁，有的话就休眠
	//if(dx > 20){
	if(check_scan_count++ == 128){
	    check_scan_count = 0;
	    //有其他进程卡在lru_lock锁，就需要释放lru_lock了，然后休眠几十ms
	    //if(atomic_read(&pgdat->shrink_spin_lock_count) > 0){
	    if(spin_is_contended(&pgdat->lru_lock))//spin_is_contended()返回true，说明有一个进程是有lock锁，然后至少还有一个进程在等待释放锁
	    {
		if(open_shrink_printk)
		    printk("3:%s %s %d spin_is_contended:%d free_page_count:%d scan_page_count:%d page:0x%llx page->flags:0x%lx\n",__func__,current->comm,current->pid,spin_is_contended(&pgdat->lru_lock),free_page_count,scan_page_count,(u64)page,page->flags);

		//对async_shrink_page赋值必须放到spin lock锁里。此时其他进程卡在lru_lock锁上。等下边spin unlock后，pgdat->async_shrink_page保存的page就生效了。
		//在msleep休眠这段时间，其他进程要是把pgdat->async_shrink_page保存的page从lru链表剔除，就会执行update_async_shrink_page()把pgdat->async_shrink_page
		//在lru file链表的上一个page保存到pgdat->async_shrink_page。
		if (PageLRU(page))
		    pgdat->async_shrink_page = page;
		else//如果当前page已经在前边async_isolate_lru_pages()被隔离到free_list链表了，则当前page不在lru链表，是无效的，要使用它在lru链表的上一个有效的page，即pgdat->async_shrink_page
                {
		    //走到这里，page肯定是隔离成功的，get_lru_prev_page不可能是0
		    //BUG_ON(!get_lru_prev_page);
		    page = pgdat->async_shrink_page;
		    get_lru_prev_page = 1;
		}
		spin_unlock_irq(&pgdat->lru_lock);
		sleep_count = 0;
		//休眠等待所有进程全释放lru_lock锁，但是如果休眠时间过长就退出休眠抢占lru_lock锁
		//while(atomic_read(&pgdat->shrink_spin_lock_count) > 0 && sleep_count++ < 5){
		while(spin_is_contended(&pgdat->lru_lock) && sleep_count++ < 5){
		    msleep(10);
		}

		//page = pgdat->async_shrink_page;
		spin_lock_irq(&pgdat->lru_lock);

                /*如果在前边休眠时page被从lru链表剔除，则要把它在lru链表上的上一个page保存到pgdat->async_shrink_page，此时 if(pgdat->async_shrink_page != page)成立。
		 *于是就要把pgdat->async_shrink_page保存到page，作为下一轮循环的page。get_lru_prev_page = 1 是说page已经更新过了，不要再执行while循环下边的page = list_prev_entry(page, lru)
		 *再更新page了。本来page = pgdat->async_shrink_page是要放到while循环最后位置，但是判断有问题。原因是：前边休眠时page被从lru链表剔除了，并保存了它在lru上前一个page到
		 *pgdat->async_shrink_page。如果此时这个page已经不再添加到lru链表，执行到下边的if(free_page_count >= 1024)分支，if (PageLRU(page))的else分支成立，它会执行
		 *page = pgdat->async_shrink_page令page保存最新有效的page。但是如果这个page从lru链表剔除后，又被添加到lru链表(内核内存回收就可能这么做)，此时执行到
		 *if(free_page_count >= 1024)分支，if (PageLRU(page))成立，则执行 pgdat->async_shrink_page = page，这个page已经打乱我的异步内存回收顺序了。
		 *
		 *故总结，在异步内存回收线程spin_unlock_irq lru_lock后，休眠，发生从lru链表剔除page而保存它在lru链表上的上一个page到pgdat->async_shrink_page后，要立即把pgdat->async_shrink_page
		 *保存到当前page，并且get_lru_prev_page = 1表示page已经更新过了。这个规则下边也同样用到
		 */
	        if(pgdat->async_shrink_page != page){//要放到spin lock加锁里
	            page = pgdat->async_shrink_page;
		    get_lru_prev_page = 1;
	        }
		start_time = jiffies;
		//page = pgdat->async_shrink_page;
		//pgdat->async_shrink_page = NULL;
	    }
	}
	//遍历链表过了100ms，休眠一段时间
	else if(dx > 100) {
	        if (PageLRU(page))//如果page在lru链表才能按照预定规则保证释放lru_lock后其他进程要是从lru链表剔除该page能更新它在lru链表的上一个page到async_shrink_page。否则pgdat->async_shrink_page要操持NULL
	            pgdat->async_shrink_page = page;
                else//如果当前page已经在前边async_isolate_lru_pages()被隔离到free_list链表了，则当前page不在lru链表，是无效的，要使用它在lru链表的上一个的有效的page，即pgdat->async_shrink_page
		{    
		    //走到这里，page肯定是隔离成功的，get_lru_prev_page不可能是0
		    //BUG_ON(!get_lru_prev_page);
		    page = pgdat->async_shrink_page;
		    get_lru_prev_page = 1;
                }
		spin_unlock_irq(&pgdat->lru_lock);

		msleep(20);
		//在休眠这段时间，老的page可能被del了，shrink_page记录最新的有效page
		//page = pgdat->async_shrink_page;
		//pgdat->async_shrink_page = NULL;

		if(!list_empty(&free_list)){
		    //每次休眠后，先尝试释放free_list上的page，主要是防止这些page长时间得不到释放-------是否有必要在加个规则，当free_list上的page达到128时，直接强制释放free_list上的page呢？
		    nr_reclaimed += async_shrink_free_page(pgdat,lruvec,&free_list,&sc,&stat);
		    free_page_count = 0;
		}

		//从msleep唤醒后，pgdat->async_shrink_page就要被立即赋值NULL，不能再影响page从lru file链表剔除。但是从msleep重新运行后到spin_lock_irq获取锁这段期间,pgdat->async_shrink_page
		//因为有效，一直在page从从lru剔除掉时而会执行到update_async_shrink_page函数，一直更新pgdat->async_shrink_page。这个没办法避免，但是page = pgdat->async_shrink_page赋值必须
		//放到spin lock锁之前，而pgdat->async_shrink_page = NULL赋值必须放到spin lock锁后边。如果pgdat->async_shrink_page = NULL赋值spin lock锁之前，page = pgdat->async_shrink_page后
		//接着就pgdat->async_shrink_page = NULL。其他进程把要最新的page从lru file链表剔除了，而pgdat->async_shrink_page 是NULL，就无法执行update_async_shrink_page函数把最新的page的上一个
		//page保存到pgdat->async_shrink_page。这样第一次page = pgdat->async_shrink_page得到的page就是一个无效的page了，从链表剔除了.因此，page = pgdat->async_shrink_page也必须放到
		//spin_lock_irq获取锁之后，因为从msleep唤醒到spin_lock_irq成功获取锁，pgdat->async_shrink_page指向的page可能被从lru链表剔除，因此pgdat->async_shrink_page会被实时更新为
		//上一个page。而获取锁成功后，其他进程就无法再从lru链表剔除page了，pgdat->async_shrink_page就不再变化了，也是最新的。ok，此时page = pgdat->async_shrink_page后，page是最新有效的
		//,然后回到list_for_each_entry_from_reverse(page,page_list_head,lru),继续从休眠前中断的page继续向file lru链表遍历page
		spin_lock_irq(&pgdat->lru_lock);
		//在休眠这段时间，老的page可能被del了，shrink_page记录最新的有效page
		//page = pgdat->async_shrink_page;
		//pgdat->async_shrink_page = NULL;

		if(!list_empty(&free_list)){
                    //把free_list残存的不能释放的page再添加到lru链表头，这个过程需要加锁
		    putback_inactive_pages(lruvec, &free_list);        
		    spin_unlock_irq(&pgdat->lru_lock);

                    //释放应用计数是0的page-------------这个过程需要释放锁，然后再加锁，加锁释放锁太频繁了
                    mem_cgroup_uncharge_list(&free_list);
                    free_unref_page_list(&free_list);
		    //if(!list_empty(&free_list))
		    //    panic("free_list not empty 1\n");
                }
  	        if(pgdat->async_shrink_page != page){//要放到spin lock加锁里
	            page = pgdat->async_shrink_page;
		    get_lru_prev_page = 1;
	        }

		//重新初始化free_list链表，因为它上边可能他残留无效的page
	        INIT_LIST_HEAD(&free_list);
		//重置起始记录时间,但要放到spin_lock_irq锁后，因为在spin_lock_irq获取锁时可能获取锁失败而阻塞一段时间
		start_time = jiffies;
		if(open_shrink_printk1)
		    printk("4:%s %s %d free_page_count:%d scan_page_count:%d\n",__func__,current->comm,current->pid,free_page_count,scan_page_count);
	}
	
	//可释放的page数达到1024个则强制释放一次
	if(free_page_count >= 1024){
	    free_page_count  = 0;

            //释放lru_lock锁前把page保存到async_shrink_page，其他进程抢占lru_lock锁后，其他进程要是把pgdat->async_shrink_page保存的page从lru链表剔除，执行update_async_shrink_page()把pgdat->async_shrink_page
	    //逼了，画蛇添足了，这个操作不应该有。因为这个当前的page要是已经在free_list链表，已经不在lru链表了。此时释放page后，if(unlikely(pgdat->async_shrink_page))成立，就会让当前的page作为下一次循环的
	    //page，但这个page已经不在lru链表了，很有可能已经被释放了。正确的做法是判断这个page是否在lru链表，在的话才能pgdat->async_shrink_page = page;操作，然后保证释放lru锁试试更新page的上一个page到async_shrink_page
	    if (PageLRU(page))//如果page在lru链表才能按照预定规则保证释放lru_lock后其他进程要是从lru链表剔除该page能更新它在lru链表的上一个page到async_shrink_page。否则pgdat->async_shrink_page要操持NULL
	        pgdat->async_shrink_page = page;
            else//如果当前page已经在前边async_isolate_lru_pages()被隔离到free_list链表了，则当前page不在lru链表，是无效的，要使用它在lru链表的上一个的有效的page，即pgdat->async_shrink_page
	    {//这个else分支是可能成立的:在当前page成功隔离后，上边的if...else分支都不成立，直接到这里else分支就成立
		//BUG_ON(!get_lru_prev_page);走到这里，page肯定是隔离成功的，get_lru_prev_page不可能是0
		page = pgdat->async_shrink_page;
		get_lru_prev_page = 1;
	    }

	    spin_unlock_irq(&pgdat->lru_lock);

	    //释放free_list链表上的page，不符合释放条件的page再移回free_list链表
	    nr_reclaimed += async_shrink_free_page(pgdat,lruvec,&free_list,&sc,&stat);

	    spin_lock_irq(&pgdat->lru_lock);
	    if(!list_empty(&free_list)){
		//把free_list残留的链表再移动回 inactive/active lru链表，同时令这些page引用计数减1。这个过程需要加锁
                putback_inactive_pages(lruvec, &free_list);
		spin_unlock_irq(&pgdat->lru_lock);

                //释放应用计数是0的page-------------这个过程需要释放锁，然后再加锁，加锁释放锁太频繁了
                mem_cgroup_uncharge_list(&free_list);
                free_unref_page_list(&free_list);
		//if(!list_empty(&free_list))---free_unref_page_list函数并不会从free_list链表剔除page，到这里free_list链表肯定不会空
		//    panic("free_list not empty 2\n");
	        spin_lock_irq(&pgdat->lru_lock);
	    }
	    if(pgdat->async_shrink_page != page){//要放到spin lock加锁里
	        page = pgdat->async_shrink_page;
		get_lru_prev_page = 1;
	    }
	    /*free_unref_page_list函数并不会从free_list链表剔除page，到这里free_list链表不会空。但是实际到这里free_list上的page已经转移到了lru链表等。
	      free_list这个头结点的next指针指向的第一个page，但该page的prev指针不是指向free_list头结点，因为该page在lru链表上。此时free_list这个链表
	      就是有问题的。等将来执行async_isolate_lru_pages->list_move()把符合内存回收条件的page添加到free_list链表，执行到__list_add()会报告
	      "list_add corruption. next->prev should be prev"错误而内核crash。因为free_list这个头结点的next指针指向的page，该page的prev指针指向的不是
	      free_list头结点。解决方法就是重新初始化free_list，它上边的page此时都是无效的。*/
	    INIT_LIST_HEAD(&free_list);
	}
#if 0
        //page和 async_shrink_page不一样，说明前边释放lru_lock后，休眠，这段时间其他把休眠前的page从lru链表剔除了，执行update_async_shrink_page()，
	//把这个page在lru链表的上一个page已经保存到了pgdat->async_shrink_page，此时下边的if成立，pgdat->async_shrink_page保存到page，作为下一次循环的判断的page。
	//如果if不成立，则要按照正常流程执行list_prev_entry(page, lru)把page在lru链表的上一个page保存到page，作为下一次循环的page
	smp_rmb();//如果其他cpu执行update_async_shrink_page对async_shrink_page有更新，这里获取最新的async_shrink_page数据
        if(unlikely(pgdat->async_shrink_page)){
	    if(pgdat->async_shrink_page != page)
	        page = pgdat->async_shrink_page;
	    else if(get_lru_prev_page == 0)
	    //这里成立说明上边有休眠的场景，休眠前把page保存到async_shrink_page，然后休眠。休眠过程page没有被剔除lru链表，于是这里就取出page在lru链表的上一个page，作为下次循环判断的page
		page = list_prev_entry(page, lru);

	    pgdat->async_shrink_page = NULL;
	}
	else
            page = list_prev_entry(page, lru);
#else
	//get_lru_prev_page是0就要按照老规则执行while循环下边的page = list_prev_entry(page, lru)用老page在lru链表上的上一个page更新page，作为下一轮内存回收while循环中判断的page.
	//get_lru_prev_page置1说明page已经用pgdat->async_shrink_page更新过了，是最新的老的page在lru链表上的上一个page，不要再执行while循环下边的page = list_prev_entry(page, lru)再更新page了
	if(get_lru_prev_page == 0){
	    //这个if成立有两种情况，1:是当前的page不是空闲状态而pgdat->async_shrink_page始终是NULL。2:page是空闲状态，执行了pgdat->async_shrink_page = page，但是page不符合隔离条件，
	    //然后expire_check分支的if....else两个分支 和 if(free_page_count >= 1024)分支都不成立,执行到这里get_lru_prev_page是0，并且pgdat->async_shrink_page == page成立。
	    if(!pgdat->async_shrink_page || pgdat->async_shrink_page == page)//async_shrink_page没有更新过
		page = list_prev_entry(page, lru);
	    else{
		//走到这里的else分支，说明该page首先async_isolate_lru_pages函数里隔离成功，但是前边的expire_check分支的if....else两个分支 和 if(free_page_count >= 1024)分支都不成立，
	 	//没有机会把老的page在lru链表上的上一个page即pgdat->async_shrink_page赋值给当前的page,于是就在这里赋值。如果没有这个赋值，就出现过问题。老的代码里，if(get_lru_prev_page == 0)
		//里只有page = list_prev_entry(page, lru)，则此时page因为是在free_list链表上，执行page = list_prev_entry(page, lru)后，page指向的free_list头结点了。相当于下轮循环判断的page
		//是free_list头结点。因为page是无效的，get_lru_prev_page一直是0，然后直接又执行到if(get_lru_prev_page == 0)里的page = list_prev_entry(page, lru)，此时page就指向老的page了
		//但是该page已经隔离成功，但是这个page作为下次循环判断的page，就会在async_isolate_lru_pages()触发BUG_ON错误，因为这个page已经隔离过了!!!!!!!!!!!
		page = pgdat->async_shrink_page;
		if(!pgdat->async_shrink_page)
		    panic("async_shrink_page is NULL\n");
	    }
	}
	if(pgdat->async_shrink_page)
	    pgdat->async_shrink_page = NULL;
#endif
    }

    spin_unlock_irq(&pgdat->lru_lock);
    if(!list_empty(&free_list)){
        //释放page，不符合释放条件的page再移回free_list链表---------------------------如果free_list上的page太多，async_shrink_free_page函数耗时会很长，可以每回收几十个page休眠一段时间
        nr_reclaimed += async_shrink_free_page(pgdat,lruvec,&free_list,&sc,&stat);
 
        if(!list_empty(&free_list)){
            spin_lock_irq(&pgdat->lru_lock);
            //把free_list残留的链表再移动回 inactive/active lru链表，同时令这些page引用计数减1
            putback_inactive_pages(lruvec, &free_list);
            spin_unlock_irq(&pgdat->lru_lock);
	}
    }
    if(open_shrink_printk && nr_reclaimed)
        printk("5:%s %s %d free_page_count:%d all_page_count:%d scan_page_count:%d nr_reclaimed:%ld memcg:0x%llx %s\n",__func__,current->comm,current->pid,free_page_count,all_page_count,scan_page_count,nr_reclaimed,(u64)memcg,lru == LRU_INACTIVE_FILE ?"inactive file lru":"active file lru");
    else if(open_shrink_printk1)
        printk("5:%s %s %d free_page_count:%d all_page_count:%d scan_page_count:%d nr_reclaimed:%ld memcg:0x%llx %s\n",__func__,current->comm,current->pid,free_page_count,all_page_count,scan_page_count,nr_reclaimed,(u64)memcg,lru == LRU_INACTIVE_FILE ?"inactive file lru":"active file lru");

    //释放应用计数是0的page
    mem_cgroup_uncharge_list(&free_list);
    free_unref_page_list(&free_list);

    //if(!list_empty(&free_list))
    //    panic("free_list not empty 3\n");
    return free_page_count;
}
static int async_shrink_memory(void *p){
    pg_data_t *pgdat = (pg_data_t*)p;
    struct mem_cgroup *memcg,*root = NULL;
    int sleep_count = 0;
    int ret;

    while(1){
	sleep_count = 0;
        while(!async_shrink_enable || sleep_count ++ < 10)
            msleep(1000);

	root = NULL;
	memcg = mem_cgroup_iter(root, NULL, NULL);
        
	ret = 0;
        do {
            struct lruvec *lruvec = mem_cgroup_lruvec(memcg, pgdat);
	    //只回收active/inactive file
            ret =  async_shrink_active_inactive_list(pgdat,lruvec,LRU_INACTIVE_FILE,memcg);
            ret += async_shrink_active_inactive_list(pgdat,lruvec,LRU_ACTIVE_FILE,memcg);
	}while ((memcg = mem_cgroup_iter(root, memcg, NULL)));

        if (kthread_should_stop())
	    break;
    }
    return 0;
}
/*********************************************************************************************************************************/
#if 0
#define HOT_FILE_AREA_CACHE_COUNT 6
#define HOT_FILE_AARE_RANGE 3
//文件热点区域信息头结点，每片热点区域的头结点
-struct hot_file_area_hot
{
    //1:hot_file_area_hot后边的区域是hot_file_area结构体
    //0:保存的是分配的4k内存page指针，这些page内存保存的是hot_file_area。此时说明hot_file_stat->hot_file_area_cache指向内存的hot_file_area全用完了。只能分配新的内存page了，用来保存
    //后续更多的hot_file_area结构，用来保存这些文件热点区域数据
    int file_area_magic;
    //文件热点区域个数
    unsigned int file_area_count;
    //最小起始文件page索引
    pgoff_t min_start_index;
    //最大起始文件page索引
    pgoff_t max_end_index;
}
//文件每个热点区域信息结构体，一个热点区域一个该结构体
-struct hot_file_area
{
    pgoff_t start_index;
    pgoff_t end_index;
    unsigned int area_access_count;
}
//热点文件统计信息，一个文件一个
-struct hot_file_stat
{
    struct address_space *mapping;
    struct list_head hot_file_list;
    struct async_shrink_file 
    unsigned int file_access_count;
    unsigned char *hot_file_area_cache;
}
//热点文件统计信息全局结构体
-struct hot_file_global
{
    struct list_head hot_file_head;
    struct list_head cold_file_head_temp;
    unsigned long hot_file_count;
    unsigned long cold_file_count;
    struct kmem_cache *hot_file_cachep;
    struct kmem_cache *hot_file_area_cachep;
    spinlock_t hot_file_lock;
}
-struct hot_file_global hot_file_global_info;
int async_shrink_file_init()
{
    unsigned int hot_file_area_cache_size = sizeof(struct hot_file_area)*HOT_FILE_AREA_CACHE_COUNT + sizeof(struct hot_file_area_hot);
    hot_file_global_info.hot_file_stat_cachep = KMEM_CACHE(hot_file_stat,0);
    hot_file_global_info.hot_file_area_cachep = kmem_cache_create("hot_file_area",hot_file_area_cache_size,0,0,NULL);
    INIT_LIST_HEAD(&hot_file_global_info.hot_file_head);
    INIT_LIST_HEAD(&hot_file_global_info.cold_file_head_temp);
    spin_lock_init(&hot_file_global_info.hot_file_lock);
}
//hot_file_area_start是保存文件热点区域结构体hot_file_area的首地址，vaild_hot_file_area_count是这片内存有效文件热点区域个数，all_hot_file_area_count是总热点区域个数
//page_index是本次要匹配查找的文件page索引。
//利用二分法查找包含索引index的hot_file_area
struct hot_file_area *find_match_hot_file_area(struct hot_file_area *hot_file_area_start,unsigned int vaild_hot_file_area_count,unsigned int all_hot_file_area_count
	                                      pgoff_t page_index,int *new_hot_file_area_index)
{
    int left,middle,right;
    struct hot_file_area *hot_file_area_middle;
    int search_count;
    /*举例
     0   1     2     3     4      5
     0-5 10-15 20-30 35-40 50-60 70-80

case1: page_index=16
step 1:left=0 right=6 middle=left + (right - 1)/2=2 则hot_file_area="20-30"
       page_index < hot_file_area.start_index(20)，则right = middle - 1=1

step 2:left=0 right=1 middle=left + (right - 1)/2=0 则hot_file_area="0-5"
       page_index > hot_file_area.end_index(5)，则left = middle + 1=1

step 3:left=1 right=1 middle=left + (right - 1)/2=1 则hot_file_area="10-15"
       page_index > hot_file_area.end_index(15)，则left = middle + 1=2
       因为left>right导致while(left <= right) 不成立退出循环,middle此时是1，指向hot_file_area="10-15",
       middle+1=2指向的hot_file_area="20-30",因为page_index=16与hot_file_area.start_index(20)相差大于HOT_FILE_AARE_RANGE(3),
       则本次的page_index不能合并到hot_file_area="20-30"

case2: page_index=51
step 1:left=0 right=6 middle=left + (right - 1)/2=2 则hot_file_area="20-30"
       page_index > hot_file_area.end_index(30)，则left = middle + 1=3

step 2:left=3 right=6 middle=left + (right - 1)/2=5 则hot_file_area="70-80"
       page_index < hot_file_area.start_index(70)，则right = middle - 1=4

case 3:left=3 right=4 middle=left + (right - 1)/2=4 则hot_file_area="50-60"
     page_index 在 则hot_file_area="50-60"范围内找到匹配的,成功返回

case3: page_index=69
step 1:left=0 right=6 middle=left + (right - 1)/2=2 则hot_file_area="20-30"
       page_index > hot_file_area.end_index(30)，则left = middle + 1=3

step 2:left=3 right=6 middle=left + (right - 1)/2=5 则hot_file_area="70-80"
       page_index < hot_file_area.start_index(70)，则right = middle - 1=4

case 3:left=3 right=4 middle=left + (right - 1)/2=4 则hot_file_area="50-60"
     page_index >hot_file_area.end_index(60),则 left = middle + 1=5
     因为left>right导致while(left <= right) 不成立退出循环,middle此时是4，指向hot_file_area="50-60",
     middle+1=5指向的hot_file_area="70-80",因为page_index=69与hot_file_area.start_index(70)相差小于HOT_FILE_AARE_RANGE(3),
     则本次的page_index=69可以合并到hot_file_area="20-30"!!!!!!!!!!!!!
     */
    *new_hot_file_area_index = -1; 
    right = vaild_hot_file_area_count;
    search_count = 0;
    while(left <= right){
        middle = left + (right - 1)/2;
        search_count ++;
	//得到中间的hot_file_area
        hot_file_area_middle = hot_file_area_start + middle;
	//待查找的索引index 小于中间区域hot_file_area的起始索引，要去文件热点区域更左半部分搜索，于是right = m - 1令右边界减少一半
        if(index < hot_file_area_middle->start_index)
	    right = middle - 1;
	//待查找的索引index 大于中间区域hot_file_area的结束索引，要去文件热点区域更右半部分搜索，于是left = m + 1令左边界增大一半
	else if(index > hot_file_area_middle->end_index)
	    left = middle + 1;
	else{//到这里肯定待查找的索引在当前hot_file_area包含的索引范围内
	    break;
	}
    }
    //middle不可能大于
    if(middle >= vaild_hot_file_area_count){
        panic("middle:%d %d error\n",vaild_hot_file_area_count,all_hot_file_area_count)
    }
    if(open_shrink_printk)
        printk("%s %s %d hot_file_area_count:%d index:%d search_count:%d\n",__func__,current->comm,current->pid,hot_file_area_count,index,search_count);
    
    //找到包含page_index索引的的hot_file_area则返回它
    if(page_index >= hot_file_area_middle->start_index && page_index <= hot_file_area_middle->end_index){
	return hot_file_area_middle;
    }
    else{
        /*case1 ****************************************************/
	/*
         0   1     2     3     4      5-----------原始文件hot_file_stat的hot_file_area_cache指向的内存只能容纳下6个hot_file_area。
         0-5 10-15 20-30 35-40 50-60 70-80
	 举例，vaild_hot_file_area_count=6，当page_index=69，经历上边的循环后middle=4，middle+1=5指向的hot_file_area="70-80"的start_index(70)与page_index=69差距
	 小于HOT_FILE_AARE_RANGE(3)，则本次的索引page_index=69就可以合并到hot_file_area="70-80"。因为本次访问的索引是69，按照经验下次访问的page索引
	 很有可能是70，这符合文件访问经验，依次向后访问。下边这个if做的就是这件事，
	 当前middle必须小于vaild_hot_file_area_count - 1，这样才能保证至少
	 有一个空闲的hot_file_area槽位。比如hot_file_area_count=6，当前内存区只有6个hot_file_area结构。走到这个分支，说明找不到
	 */
        //if(vaild_hot_file_area_count <= all_hot_file_area_count){
	    //比如 middle=4 vaild_hot_file_area_count=6，看middle+1=5指向的hot_file_area.start_index是否与page_index很接近。
	    //middle指向倒数第2个有效的hot_file_area，middle+1=5指向的 hot_file_area是最后一个有效的hot_file_area，看page_index能否合并到middle+1=5指向的 hot_file_area
	    if(middle < vaild_hot_file_area_count -1){//比如 middle=4 vaild_hot_file_area_count=6，看middle+5指向的hot_file_area.start_index是否与page_index很接近
		//middle指向middle后边那个的hot_file_area，看page_index与这个hot_file_area.start_index是否很接近，很接近就可以合并
		hot_file_area_middle = hot_file_area_start + middle + 1;
		if(hot_file_area_middle->start_index - page_index <= HOT_FILE_AARE_RANGE){
		    //更新hot_file_area的start_index 为 page_index，相当于把page_index何必到了当前的hot_file_area
		    hot_file_area_middle->start_index = page_index;
		    return hot_file_area_middle;
		}
	    }
	
        /*case2 ****************************************************/
        //执行到这里，说明没有找到没有找到匹配page_index的hot_file_area。但是还有剩余的空间，可以分配一个hot_file_area，保存本次的page->index
	if(vaild_hot_file_area_count < all_hot_file_area_count){
	    //分配一个新的hot_file_area，存本次的page->index
	    hot_file_area_middle = hot_file_area_start + vaild_hot_file_area_count;
	    hot_file_area_middle->start_index = page_index;
	    hot_file_area_middle->end_index = page_index + HOT_FILE_AARE_RANGE;
	    return hot_file_area_middle;
	}

        /*case3 ****************************************************/
	//执行到这里，说明 说明没有找到没有找到匹配page_index的hot_file_area，但是没有剩余的空间可以分配一个hot_file_area保存本次的page->index了。
	//那只能分配一个新的4K page内存，分配新的hot_file_area，保存本次的page->index
	if(vaild_hot_file_area_count >= all_hot_file_area_count){
	    return NULL;
	}

	/*以上是能想到的几种情况，但是还有隐藏很深的问题，看如下来自
	 
	 0   1     2     3     4      5-------------------索引地址必须由左向右依次增大
         0-5 10-15 20-30 35-40 50-60 75-80

	 vaild_hot_file_area_count=6，当page_index=65，经历上边的循环后middle=4，middle+1=5指向的hot_file_area="75-80"的start_index(75)与page_index=65
	 差距大于HOT_FILE_AARE_RANGE(3)，则本次的索引page_index=65 无法合并到hot_file_area="75-80"。怎么办？
	 要把弄一个新的hot_file_area ，保存page_index=65，然后把它插入到 hot_file_area="50-60"和hot_file_area="75-80"之间，
         具体操作起来非常麻烦，要先做成这样
	 0   1     2     3     4      5
         0-5 10-15 20-30 35-40 50-60 65-68
	 hot_file_area="75-80"就被挤走了，只能分配一个新的4K page内存，然后把hot_file_area="75-80"移动到这个4K内存page。
 	 是吗，并不是，按照预期算法，实际要把原有的在hot_file_area_cache指向的内存的6个hot_file_area也移动到这个4K内存page，如下
	 0   1     2     3     4      5     6
         0-5 10-15 20-30 35-40 50-60 65-68  75-80

	 然后 hot_file_area_cache指向的内存不再保存hot_file_area，而是变成索引，比如第一片内存指向前边分配的4K内存page，索引范围是0-80
	 0     1   2   3   4  5
	 0-80

         
	 还有一种情况,如下6片hot_file_area保存在4K page内存
	 0   1     2     3     4      6      7 
         0-5 10-15 20-30 35-40 50-60  75-80  90-100-------------------索引地址必须由左向右依次增大
         假设此时 page_index=69 要插入这里，最后m=4，则要分配一个新的 hot_file_area，保存page_index=69，然后把插入到里边如下
	 0   1     2     3     4      6      
         0-5 10-15 20-30 35-40 50-60  69-72
         然后把原有的 hot_file_area="75-80"和hot_file_area="90-100"复制，向后移动一个hot_file_area位置，整体变成如下:
	 0   1     2     3     4      6     6      7  
         0-5 10-15 20-30 35-40 50-60  69-72 75-80  90-100
         

	 还有一种情况
	 0   1     2     3     4      6      7 
         0-5 10-15 20-30 35-40 50-60  63-80  90-100-------------------索引地址必须由左向右依次增大
         此时 page_index=61要插入到里边
	 0   1     2     3     4      6      7 
         0-5 10-15 20-30 35-40 50-60  61-80  90-100
	 然后是否要把hot_file_area="61-80"合并到 hot_file_area="50-60 "，并且把 hot_file_area="90-100"向前移动一个 hot_file_area位置
	 0   1     2     3     4      7 
         0-5 10-15 20-30 35-40 50-80  90-100

    方案2	 
         这样的算法太复杂了！会因为发生插入新的hot_file_area或者老的hot_file_area合并到其他hot_file_area，导致频繁向前或者向
	 后复制移动N个 hot_file_area结构数据，浪费cpu！并且可能令 hot_file_area的索引范围无限扩大，比如的hot_file_area索引范围达到100，
	 这样就不太合适了，这种page索引范围太大了。粒度太大了！可以发现，原有的算法会遇到各种ext4 extent麻烦，比较浪费cpu。并且
	 hot_file_area的索引范围不受控制，大是很大，小时很小(将导致分配很多hot_file_area结构)。没办法，只能改进算法。
	 令每个hot_file_area的索引范围固定，比如每个hot_file_area的索引范围固定是5，也是从左向右排列，禁止hot_file_area向前或向后复制
	 数据结构，不再令相邻hot_file_area合并。
	 0   1    2     3     4     5    
         1-5 6-10 11-15 16-20 21-25 26-30-------------------索引地址必须由左向右依次增大
	 现在一个 hot_file_area索引范围是5，当文件很大时，弹性令hot_file_area索引范围增大到10,甚至20。总体觉得，这种算法更优，
	 更简单，避免繁琐的ext4 extent的合并、分割、赋值 操作。

	 当hot_file_area很多时，就分配4K的page内存，保存更多的hot_file_area

	 0-30 31-60 61-90 91-120 121-150  151-180--------原始文件hot_file_stat的hot_file_area_cache指向的内存只能容纳下6个hot_file_area空间，现在变成索引
         |
	 |
	 0   1    2     3     4     5     6     7     ........
         1-5 6-10 11-15 16-20 21-25 26-30 31-35 36-40 ........ -------------------4k page内存能容纳很多个hot_file_area

        这个方案看着貌似合理，但其实也有很大问题：一个全新的文件，10G，现在开始访问文件，但是直接访问文件索引 10000，这种情况线上是有的
	，并不是所有文件都是从文件0地址开始访问！

	这种情况要分配很多无效的中间索引page
	
	0-10000 10001-20000 *-* *-* *-*  50001-60000--------原始文件hot_file_stat的hot_file_area_cache指向的内存只能容纳下6个hot_file_area空间，现在变成索引
        
	0--5000  5001-10000   ------这里的两个page内存，都是索引，每个page的所有包含的索引总范围是5000
                  |
	          |
                  10000-10003 10004-10006 10007-10009 10011-10013  ----这个page内存才是有效的hot_file_area，包含了本次的文件索引10000

        看到没，为了找到第一次访问的page索引10000，就要无端分配3个page，浪费了内存。极端情况，分配的无效的page只会更多，这个方案也不行

    方案3	
        radix tree +ext4 extent

	把中间索引节点"0--5000"和 "5001-10000" 两个4K内存page，换成类似radix tree的radix_tree_node节点就行，一个节点消耗不了多少内存。
        而radix_tree_node的成员void *slots[64]保存一个个hot_file_area结构指针，保存热点索引区域
        */

	//否则就要把page_index插入到 middle指向的热点区域hot_file_area，原来位置的向后移动
        //new_hot_file_area_index = middle;
        if(open_shrink_printk)
	    printk("%s %s %d find error\n",__func__,current->comm,current->pid);
	return NULL;
    }
}
int async_shirnk_update_file_status(struct *page){
    struct address_space *mapping;
    int ret = 0;
    struct hot_file_stat * p_hot_file_stat = NULL;
    unsigned char *hot_file_area_cache = NULL;

    mapping = page_mapping(page);
    if(mapping){
        struct hot_file_area_hot *p_hot_file_area_hot;
	struct hot_file_area *p_hot_file_area; 

	if(!mapping->hot_file_stat){
            unsigned int hot_file_area_cache_size = sizeof(struct hot_file_area)*HOT_FILE_AREA_CACHE_COUNT + sizeof(struct hot_file_area_hot);

	    if(!hot_file_global_info.hot_file_stat_cachep || !hot_file_global_info.hot_file_area_cachep){
	        ret =  -ENOMEM;
		goto err;
	    }
		
	    //新的文件分配hot_file_stat,一个文件一个，保存文件热点区域访问数据
	    p_hot_file_stat = kmem_cache_alloc(hot_file_global_info.hot_file_stat_cachep,GFP_KERNE);
            if (!p_hot_file_stat) {
	        printk("%s hot_file_stat alloc fail\n",__func__);
	        ret =  -ENOMEM;
		goto err;
	    }
	    memset(p_hot_file_stat,sizeof(struct hot_file_stat),0);
            //新的文件，先分配hot_file_area_cache,这片区域是1个hot_file_head结构 + 6个hot_file_area结构
	    hot_file_area_cache = kmem_cache_alloc(hot_file_global_info.hot_file_area_cachep,GFP_KERNE);
            if (!p_hot_file_area) {
	        printk("%s hot_file_area alloc fail\n",__func__);
	        ret =  -ENOMEM;
		goto err;
            }
	    memset(hot_file_area_cache,hot_file_area_cache_size,0);
	    //mapping->hot_file_stat记录该文件绑定的hot_file_stat结构，将来判定是否对该文件分配了hot_file_stat
	    mapping->hot_file_stat = p_hot_file_stat;
	    //hot_file_stat记录mapping结构
	    p_hot_file_stat->mapping = mapping;
	    //文件访问次数加1
	    p_hot_file_stat->file_access_count++;
            //p_hot_file_area_hot指向hot_file_area_cache第一片区域，即hot_file_area_hot。往后还有6片hot_file_area结构
            p_hot_file_area_hot = (struct hot_file_area_hot*)hot_file_area_cache;
	    //p_hot_file_area_hot指向的区域内文件热点区域加1
            p_hot_file_area_hot->file_area_count ++;
	    p_hot_file_area_hot->file_area_magic = 0;
            //hot_file_stat->hot_file_area_cache指向头结点区域
	    p_hot_file_stat->hot_file_area_cache = p_hot_file_area_hot;

	    //p_hot_file_area指向hot_file_area_cache第一个hot_file_area结构，为新文件分配的hot_file_stat，肯定要在第一片hot_file_area结构记录第一个该文件的热点区域
	    p_hot_file_area = (struct hot_file_area*)(p_hot_file_area_hot + sizeof(struct hot_file_area_hot));
	    //p_hot_file_area记录文件热点区域的起始、结束文件索引，默认page->index后的5个page都是热点区域
	    p_hot_file_area->start_index = page->index;
	    p_hot_file_area->end_index = page->index + HOT_FILE_AARE_RANGE;
	    //p_hot_file_area热点区域访问数加1
	    p_hot_file_area->area_access_count ++;

            p_hot_file_area_hot->min_start_index = p_hot_file_area->start_index;
            p_hot_file_area_hot->max_start_index = p_hot_file_area->end_index;
	    
            spin_lock(&hot_file_global_info.hot_file_lock);
	    list_add_rcu(p_hot_file_stat->hot_file_list,hot_file_global_info.hot_file_head);
	    spin_unlock(&hot_file_global_info.hot_file_lock);
	}
	else//走到这个分支，说明之前为当前访问的文件分配了hot_file_stat结构
	{
	    //从mapping得到该文件绑定的hot_file_stat结构
	    p_hot_file_stat = mapping->hot_file_stat;
	    //从文件绑定的hot_file_stat结构的成员hot_file_area_cache得到保存文件热点区域的内存地址，保存到p_hot_file_area_hot。该内存的数据是
	    //1个hot_file_area_hot结构+6个hot_file_area结构。但是如果文件热点区域hot_file_area大于6个，则这片内存的数据调整为是
	    //1个hot_file_area_hot结构+N个page指针，这些page的内存保存文件热点区域hot_file_area结构
            p_hot_file_area_hot = (struct hot_file_area_hot *)p_hot_file_stat->hot_file_area_cache;
	    //令p_hot_file_area第一个hot_file_area结构
	    p_hot_file_area = (struct hot_file_area *)(p_hot_file_area_hot + sizeof(struct hot_file_area_hot));

            //文件的ot_file_stat的hot_file_area_cache指向的内存保存的是文件热点区域结构hot_file_area
	    if(p_hot_file_area_hot->file_area_magic == 0)
	    {
                //本次的文件页page索引在hot_file_area_hot指向的热点区域范围内
	        //if(page->index > p_hot_file_area_hot->min_start_index && page->index < p_hot_file_area_hot->max_start_index)
		
		//找到包含page->index的文件热点区域hot_file_area则返回它，否则返回NULL。
		p_hot_file_area = find_match_hot_file_area(p_hot_file_area,hot_file_area_count,page->index);
                if(p_hot_file_area){
		    //该文件热点区域访问次数加1
		    p_hot_file_area->area_access_count ++;
		}
		else{
		    //文件绑定的hot_file_stat结构的成员hot_file_area_cache还有空闲的hot_file_area保存本次文件的热点区域
		    if(p_hot_file_area_hot->file_area_count < HOT_FILE_AREA_CACHE_COUNT){
			//令p_hot_file_are指向hot_file_area_cache空闲的hot_file_area，由于hot_file_area_cache里的6个hot_file_area，从左到右保存的文件索引
			//
		        p_hot_file_area = p_hot_file_area_hot + sizeof(struct hot_file_area)*(p_hot_file_area_hot->file_area_count;
			p_hot_file_area_hot->file_area_count ++;

		    }else{
		        /*到这里，说明hot_file_area_cache里的6个hot_file_area用完了，要分配内存page保存新的hot_file_area结构了*/

			
		    }

		}
	    }else
	    {
	    
	    }

	    //该文件的访问次数加1
	    p_hot_file_stat->file_access_count++;
            
        }
    }

    return 0;

err:
    if(p_hot_file_stat)
	kmem_cache_free(hot_file_global_info.hot_file_stat_cachep,p_hot_file_stat);
    if(p_hot_file_area)
	kmem_cache_free(hot_file_global_info.hot_file_area_cachep,p_hot_file_area);
    return ret;
}

#else
//一个 hot_file_area 包含的page数，默认6个
#define PAGE_COUNT_IN_AREA_SHIFT 3
#define PAGE_COUNT_IN_AREA (1UL << PAGE_COUNT_IN_AREA_SHIFT)

#define TREE_MAP_SHIFT	6
#define TREE_MAP_SIZE	(1UL << TREE_MAP_SHIFT)
#define TREE_MAP_MASK (TREE_MAP_SIZE - 1)

#define TREE_ENTRY_MASK 3
#define TREE_INTERNAL_NODE 1

//file_area在 GOLD_FILE_AREA_LEVAL 个周期内没有被访问则被判定是冷file_area，然后释放这个file_area的page
#define GOLD_FILE_AREA_LEVAL  5

#define FILE_AREA_HOT_BIT (1 << 0)//hot_file_area的bit0是1表示是热的file_area_hot,是0则是冷的。bit1是1表示是热的大文件，是0则是小文件
//一个冷hot_file_area，如果经过HOT_FILE_AREA_FREE_LEVEL个周期，仍然没有被访问，则释放掉hot_file_area结构
#define HOT_FILE_AREA_FREE_LEVEL  6
//当一个hot_file_area在一个周期内访问超过FILE_AREA_HOT_LEVEL次数，则判定是热的hot_file_area
#define FILE_AREA_HOT_LEVEL 3
//一个hot_file_area表示了一片page范围(默认6个page)的冷热情况，比如page索引是0~5、6~11、12~17各用一个hot_file_area来表示
struct hot_file_area
{
    //每次hot_file_stat的hot_file_area_free链表上的hot_file_area，每次遍历cold_time加1，如果cold_time达到阀值就释放掉hot_file_area结构。
    //如果在这个过程中hot_file_area又被访问了，则cold_time清0，并且把hot_file_area移动到hot_file_area_temp链表。
    //unsigned char cold_time;
    //不同取值表示hot_file_area当前处于哪种链表，hot_file_area_temp:0 hot_file_area_hot:1 hot_file_area_cold:2 hot_file_area_free_temp:3 hot_file_area_free:4 hot_file_area_refault:5
    unsigned char file_area_state;
    //该hot_file_area 上轮被访问的次数
    //unsigned int last_access_count;
    //该file_area最新依次被访问时的global_age，global_age - file_area_age差值大于 GOLD_FILE_AREA_LEVAL，则判定file_area是冷file_area，然后释放该file_area的page
    unsigned long file_area_age;
    //该hot_file_area当前周期被访问的次数
    unsigned int area_access_count;
    //该hot_file_area里的某个page最近一次被回收的时间点，单位秒
    unsigned int shrink_time;
    //hot_file_area通过hot_file_area_list添加hot_file_stat的各种链表
    struct list_head hot_file_area_list;
    //指向父hot_file_area_tree_node节点
    struct hot_file_area_tree_node *parent;
    //该hot_file_area代表的N个连续page的起始page索引
    pgoff_t start_index;
};
struct hot_file_area_tree_node
{
    //与该节点树下最多能保存多少个page指针有关
    unsigned char   shift;
    //在节点在父节点中的偏移
    unsigned char   offset;
    //指向父节点
    struct hot_file_area_tree_node *parent;
    //该节点下有多少个成员
    unsigned int    count;
    //是叶子节点时保存hot_file_area结构，是索引节点时保存子节点指针
    void    *slots[TREE_MAP_SIZE];
};
struct hot_file_area_tree_root
{
    unsigned int  height;//树高度
    struct hot_file_area_tree_node __rcu *root_node;
};
//热点文件统计信息，一个文件一个
struct hot_file_stat
{
    struct address_space *mapping;
    //hot_file_stat通过hot_file_list添加到hot_file_global的hot_file_head链表
    struct list_head hot_file_list;
    unsigned char file_stat_status;//bit0表示冷文件还是热文件，bit1表示大文件还是小文件
    unsigned int file_area_count;//总hot_file_area结构个数
    unsigned int file_area_hot_count;//热hot_file_area结构个数
//  unsigned char *hot_file_area_cache;
    struct hot_file_area_tree_root hot_file_area_tree_root_node;
    spinlock_t hot_file_stat_lock;
    //频繁被访问的文件page对应的hot_file_area存入这个头结点
    struct list_head hot_file_area_hot;
    //不冷不热处于中间状态的hot_file_area结构添加到这个链表，新分配的hot_file_area就添加到这里
    struct list_head hot_file_area_temp;
    //访问很少的文件page对应的hot_file_area，移动到该链表
    struct list_head hot_file_area_cold;
    //每轮扫描被释放内存page的hot_file_area结构临时先添加到这个链表。hot_file_area_free_temp有存在的必要
    struct list_head hot_file_area_free_temp;
    //所有被释放内存page的hot_file_area结构最后添加到这个链表，如果长时间还没被访问，就释放hot_file_area结构。
    struct list_head hot_file_area_free;
    //hot_file_area的page被释放后，但很快又被访问，发生了refault，于是要把这种page添加到hot_file_area_refault链表，短时间内不再考虑扫描和释放
    struct list_head hot_file_area_refault;
    //本轮扫描移动到hot_file_area_cold链表的file_area个数
    //unsigned int file_area_count_in_cold_list;
    //上一轮扫描移动到hot_file_area_cold链表的file_area个数
    //unsigned int old_file_area_count_in_cold_list;
};
struct hot_file_node_pgdat
{
    pg_data_t *pgdat;
    struct list_head pgdat_page_list;
};
//热点文件统计信息全局结构体
struct hot_file_global
{
    //被判定是热文本的hot_file_stat添加到hot_file_head链表
    struct list_head hot_file_head;
    //新分配的文件hot_file_stat默认添加到hot_file_head_temp链表
    struct list_head hot_file_head_temp;
    //如果文件file_stat上的page cache数超过1G，则把file_stat移动到这个链表。将来内存回收时，优先遍历这种file_stat，因为file_area足够多，能遍历到更多的冷file_area，回收到内存page
    struct list_head hot_file_head_temp_large;
    //当file_stat的file_area个数达到file_area_count_for_large_file时，表示该文件的page cache数达到1G。因为一个file_area包含了多个page，一个file_area并不能填满page，
    //因此实际file_stat的file_area个数达到file_area_count_for_large_file时，实际该文件的的page cache数应该小于1G
    int file_area_count_for_large_file;

    struct list_head cold_file_head;
    //在cold_fiLe_head链表的file_stat个数
    //unsigned int file_stat_count_in_cold_list;
    unsigned int hot_file_count;
    unsigned int cold_file_count;
    unsigned long global_age;//每个周期加1
    struct kmem_cache *hot_file_stat_cachep;
    struct kmem_cache *hot_file_area_cachep;
    struct kmem_cache *hot_file_area_tree_node_cachep;
    spinlock_t hot_file_lock;
    struct hot_file_node_pgdat *p_hot_file_node_pgdat;
    struct task_struct *hot_file_thead;
    int node_count;
};
struct hot_file_global hot_file_global_info;
#endif

#if 0
//返回1说明hot_file_area结构处于hot_file_area_temp链表，不冷不热
static inline int file_area_in_temp_list(struct hot_file_area *p_hot_file_area)
{
    return (0 == p_hot_file_area->file_area_state);
}
//设置 p_hot_file_area->file_area_state = 0表示该 hot_file_area处于hot_file_area_temp链表
static inline void set_file_area_in_temp_list(struct hot_file_area *p_hot_file_area)
{
    p_hot_file_area->file_area_state  = 0;
    smp_wmb();
}

//返回1说明hot_file_area结构处于hot_file_area_cold链表，冷file_file_area
static inline int file_area_in_cold_list(struct hot_file_area *p_hot_file_area)
{
    return (1 == p_hot_file_area->file_area_state);
}
//设置 p_hot_file_area->file_area_state = 1表示该 hot_file_area处于hot_file_area_cold链表
static inline void set_file_area_in_cold_list(struct hot_file_area *p_hot_file_area)
{
    p_hot_file_area->file_area_state  = 1;
    smp_wmb();
}

//返回1说明hot_file_area结构处于hot_file_area_hot链表，是热hot_file_area
static inline int file_area_in_hot_list(struct hot_file_area *p_hot_file_area)
{
    return (2 == p_hot_file_area->file_area_state);
}
//设置 p_hot_file_area->file_area_state = 2表示该 hot_file_area处于hot_file_area_hot链表
static inline void set_file_area_in_hot_list(struct hot_file_area *p_hot_file_area)
{
    p_hot_file_area->file_area_state  = 2;
    smp_wmb();
}

//返回1说明hot_file_area结构处于hot_file_area_refault链表
static inline int file_area_in_refault_list(struct hot_file_area *p_hot_file_area)
{
    return (5 == p_hot_file_area->file_area_state);
}
//设置 p_hot_file_area->file_area_state = 5 表示该 hot_file_area处于hot_file_area_refault链表
static inline void set_file_area_in_refault_list(struct hot_file_area *p_hot_file_area)
{
    p_hot_file_area->file_area_state  = 5;
    smp_wmb();
}

//返回1说明hot_file_area结构处于hot_file_area_free_temp链表
static inline int file_area_in_free_temp_list(struct hot_file_area *p_hot_file_area)
{
    return (3 == p_hot_file_area->file_area_state);
}
//设置 p_hot_file_area->file_area_state = 3 表示该 hot_file_area处于hot_file_area_free_temp链表
static inline void set_file_area_in_free_temp_list(struct hot_file_area *p_hot_file_area)
{
    p_hot_file_area->file_area_state  = 3;
    smp_wmb();
}

//返回1说明hot_file_area结构处于hot_file_area_free链表，不冷不热
static inline int file_area_in_free_list(struct hot_file_area *p_hot_file_area)
{
    return (4 == p_hot_file_area->file_area_state);
}
//设置 p_hot_file_area->file_area_state = 4 表示该 hot_file_area处于hot_file_area_free链表
static inline void set_file_area_in_free_list(struct hot_file_area *p_hot_file_area)
{
    p_hot_file_area->file_area_state  = 4;
    smp_wmb();
}
#else
enum file_area_status{
    F_file_area_in_temp_list,
    F_file_area_in_cold_list,
    F_file_area_in_hot_list,
    F_file_area_in_free_temp_list,
    F_file_area_in_free_list,
    F_file_area_in_refault_list
};
//不能使用 clear_bit、set_bit、test_bit，因为要求p_hot_file_area->file_area_state是64位数据，但实际只是u8型数据

//设置file_area的状态，在哪个链表
#define CLEAR_FILE_AREA_STATUS(list_name) \
static inline void clear_file_area_in_##list_name(struct hot_file_area *p_hot_file_area)\
      { p_hot_file_area->file_area_state &= ~(1 << F_file_area_in_##list_name);}
//    {clear_bit(file_area_in_##list_name,p_hot_file_area->file_area_state);}
//清理file_area在哪个链表的状态
#define SET_FILE_AREA_STATUS(list_name) \
static inline void set_file_area_in_##list_name(struct hot_file_area *p_hot_file_area)\
    { p_hot_file_area->file_area_state |= (1 << F_file_area_in_##list_name);}
    //{set_bit(file_area_in_##list_name,p_hot_file_area->file_area_state);}
//测试file_area在哪个链表
#define TEST_FILE_AREA_STATUS(list_name) \
static inline int file_area_in_##list_name(struct hot_file_area *p_hot_file_area)\
    {return p_hot_file_area->file_area_state & (1 << F_file_area_in_##list_name);}
    //{return test_bit(file_area_in_##list_name,p_hot_file_area->file_area_state);}

#define FILE_AREA_STATUS(list_name)     \
        CLEAR_FILE_AREA_STATUS(list_name) \
        SET_FILE_AREA_STATUS(list_name)  \
        TEST_FILE_AREA_STATUS(list_name)

FILE_AREA_STATUS(temp_list)
FILE_AREA_STATUS(cold_list)
FILE_AREA_STATUS(hot_list)
FILE_AREA_STATUS(free_temp_list)
FILE_AREA_STATUS(free_list)
FILE_AREA_STATUS(refault_list)
#endif

#if 0
//返回1说明hot_file_stat处于walk_throuth_all_hot_file_area()函数中的临时链表
static inline int file_stat_in_hot_file_other_list(struct hot_file_stat *p_hot_file_stat)
{
   return (0 == p_hot_file_stat->file_stat_status);
}
//设置hot_file_stat处于walk_throuth_all_hot_file_area()函数中的临时链表
static inline void set_file_stat_in_other_list(struct hot_file_stat *p_hot_file_stat)
{
    p_hot_file_stat->file_stat_status = 0;
    smp_wmb();
}

//返回1说明hot_file_stat处于global hot_file_head_temp链表
static inline int file_stat_in_hot_file_head_temp(struct hot_file_stat *p_hot_file_stat)
{
   return (1 == p_hot_file_stat->file_stat_status);
}
//设置hot_file_stat处于global hot_file_head_temp链表
static inline void set_file_stat_in_head_temp_list(struct hot_file_stat *p_hot_file_stat)
{
    p_hot_file_stat->file_stat_status = 1;
    smp_wmb();
}

//返回1说明hot_file_stat处于global hot_file_head链表
static inline int file_stat_in_hot_file_head(struct hot_file_stat *p_hot_file_stat)
{
   return (2 == p_hot_file_stat->file_stat_status);
}
//设置hot_file_stat处于global hot_file_head链表
static inline void set_file_stat_in_hot_file_head(struct hot_file_stat *p_hot_file_stat)
{
    p_hot_file_stat->file_stat_status = 2;
    smp_wmb();
}

//设置hot_file_stat是大文件
static inline void set_file_stat_in_hot_file_head_temp_large(struct hot_file_stat *p_hot_file_stat)
{
    p_hot_file_stat->file_stat_status = 3;
    smp_wmb();
}
//返回1说明是large file
static inline int file_stat_in_hot_file_head_temp_large(struct hot_file_stat *p_hot_file_stat)
{
   return (3 == p_hot_file_stat->file_stat_status);
}
#else
enum file_stat_status{
    F_file_stat_in_hot_file_head_list,
    F_file_stat_in_hot_file_head_temp_list,
    F_file_stat_in_large_file,
};
//不能使用 clear_bit、set_bit、test_bit，因为要求p_hot_file_stat->file_stat_status是64位数据，但这里只是u8型数据

//设置file_stat的状态，在哪个链表
#define CLEAR_FILE_STAT_STATUS(name)\
static inline void clear_file_stat_in_##name##_list(struct hot_file_stat *p_hot_file_stat)\
    {p_hot_file_stat->file_stat_status &= ~(1 << F_file_stat_in_##name##_list);}
//    {clear_bit(file_stat_in_##list_name,p_hot_file_stat->file_stat_status);}
//清理file_stat在哪个链表的状态
#define SET_FILE_STAT_STATUS(name)\
static inline void set_file_stat_in_##name##_list(struct hot_file_stat *p_hot_file_stat)\
    {p_hot_file_stat->file_stat_status |= (1 << F_file_stat_in_##name##_list);}
//    {set_bit(file_stat_in_##list_name,p_hot_file_stat->file_stat_status);}
//测试file_stat在哪个链表
#define TEST_FILE_STAT_STATUS(name)\
static inline int file_stat_in_##name##_list(struct hot_file_stat *p_hot_file_stat)\
    {return (p_hot_file_stat->file_stat_status | (1 << F_file_stat_in_##name##_list));}
//    {return test_bit(file_stat_in_##list_name,p_hot_file_stat->file_stat_status);}

#define FILE_STAT_STATUS(name) \
    CLEAR_FILE_STAT_STATUS(name) \
    SET_FILE_STAT_STATUS(name) \
    TEST_FILE_STAT_STATUS(name)

FILE_STAT_STATUS(hot_file_head)
FILE_STAT_STATUS(hot_file_head_temp)
//FILE_STAT_STATUS(large_file)
    
//设置文件的状态，大小文件等
#define CLEAR_FILE_STATUS(name)\
static inline void clear_file_stat_in_##name(struct hot_file_stat *p_hot_file_stat)\
    {p_hot_file_stat->file_stat_status &= ~(1 << F_file_stat_in_##name);}
//清理文件的状态，大小文件等
#define SET_FILE_STATUS(name)\
static inline void set_file_stat_in_##name(struct hot_file_stat *p_hot_file_stat)\
    {p_hot_file_stat->file_stat_status |= (1 << F_file_stat_in_##name);}
//测试文件的状态，大小文件等
#define TEST_FILE_STATUS(name)\
static inline int file_stat_in_##name(struct hot_file_stat *p_hot_file_stat)\
    {return (p_hot_file_stat->file_stat_status | (1 << F_file_stat_in_##name));}

#define FILE_STATUS(name) \
    CLEAR_FILE_STATUS(name) \
    SET_FILE_STATUS(name) \
    TEST_FILE_STATUS(name)

FILE_STATUS(large_file)
#endif

static inline unsigned long hot_file_area_tree_shift_maxindex(unsigned int shift)
{
    return (TREE_MAP_SIZE << shift) - 1;
}
//计算以当前节点node为基准，它下边的子树能容纳多少个page有关的hot_file_area。如果是跟节点，则表示整个tree最多容纳多少个hot_file_area
static inline unsigned long hot_file_area_tree_node_maxindex(struct hot_file_area_tree_node *node)
{
    return  hot_file_area_tree_shift_maxindex(node->shift);
}
static inline bool hot_file_area_tree_is_internal_node(void *ptr)
{
    return ((unsigned long)ptr & TREE_ENTRY_MASK) == TREE_INTERNAL_NODE;
}
static inline struct hot_file_area_tree_node *entry_to_node(void *ptr)
{
    return (void *)((unsigned long)ptr & ~TREE_INTERNAL_NODE);
}
static inline void *node_to_entry(void *ptr)
{
    return (void *)((unsigned long)ptr | TREE_INTERNAL_NODE);
}
int hot_file_area_tree_extend(struct hot_file_area_tree_root *root,unsigned long area_index,unsigned int shift)
{
    struct hot_file_area_tree_node *slot;
    unsigned int maxshift;
    
    maxshift = shift;
    //hot_file_area_tree要扩增1层时，这个循环不成立.扩增2层时循环成立1次，其他类推
    while (area_index > hot_file_area_tree_shift_maxindex(maxshift))
	maxshift += TREE_MAP_SHIFT;
    
    slot = root->root_node;
    if (!slot)
        goto out;

    do {
	//在分配radix tree node前，是spin lock加了hot_file_stat->hot_file_stat_lock锁的，故这里分配内存禁止休眠，否则低内存场景就会占着spin锁休眠，然后导致其他进程获取spin lock失败而soft lockup
        //struct hot_file_area_tree_node* node = kmem_cache_alloc(hot_file_global_info.hot_file_area_tree_node_cachep,GFP_KERNEL);
        struct hot_file_area_tree_node* node = kmem_cache_alloc(hot_file_global_info.hot_file_area_tree_node_cachep,GFP_ATOMIC);
	if (!node)
	    return -ENOMEM;
	memset(node,0,sizeof(struct hot_file_area_tree_node));
        node->shift = shift;
	node->offset = 0;
	node->count = 1;
	node->parent = NULL;
	if (hot_file_area_tree_is_internal_node(slot))
	    entry_to_node(slot)->parent = node;
	//当hot_file_area tree只保存索引是0的hot_file_area时，hot_file_area指针是保存在root->root_node指针里。后续hot_file_area tree添加其他成员时，就需要增加tree层数，就在这个循环完成。
	//可能hot_file_area tree一次只增加一层，或者增加多层。这行代码是限制，当第一层增加tree层数时，slot是root->root_node，并且slot保存的是索引是0的hot_file_area指针，不是节点。
	//则hot_file_area_tree_is_internal_node(slot)返回flase，然后执行slot->parent = node令索引是0的hot_file_area的parent指向父节点。没有这样代码，该hot_file_area就成没有父亲的孤儿了，后续释放tree就会有问题
        else if(slot == root->root_node && !hot_file_area_tree_is_internal_node(slot))
	    //此时根节点root->root_node保存的是hot_file_area指针，并不是hot_file_area_tree_node指针，要强制转换成hot_file_area指针并令其parent成员指向父节点。否则还是以
	    //hot_file_area_tree_node->parent=node形式赋值，实际赋值到了hot_file_area->file_area_age成员那里，内存越界了,导致它很大!!!!!!!!!!!
	    //slot->parent = node; 此时根节点root->root_node保存的是hot_file_area指针，并不是hot_file_area_tree_node指针，要强制转换成hot_file_area指针并令
	    ((struct hot_file_area *)slot)->parent = node;

	node->slots[0] = slot;
	slot = node_to_entry(node);
	rcu_assign_pointer(root->root_node, slot);
	shift += TREE_MAP_SHIFT;
    }while (shift <= maxshift);
out:
    return maxshift + RADIX_TREE_MAP_SHIFT;    
}
struct hot_file_area_tree_node *hot_file_area_tree_lookup_and_create(struct hot_file_area_tree_root *root,
	                                                 unsigned long area_index,void ***page_slot_in_tree)
{
    unsigned int shift, offset = 0;
    unsigned long max_area_index;
    struct hot_file_area_tree_node *node = NULL, *child;
    void **slot = (void **)&root->root_node;
    int ret;
    //hot_file_area_tree根节点，radix tree原本用的是rcu_dereference_raw，为什么?????????????需要研究下
    node = rcu_dereference_raw(root->root_node);

    //hot_file_area_tree至少有一层，不是空的树
    if (likely(hot_file_area_tree_is_internal_node(node))){
        //hot_file_area_tree根节点的的shift+6
        shift = node->shift + TREE_MAP_SHIFT;
        max_area_index = hot_file_area_tree_shift_maxindex(node->shift);
    }
    else//到这里说明hot_file_area_tree 是空的，没有根节点
    {
	shift = 0;
	max_area_index = 0;
    }
    //此时child指向根节点
    child = node;
    //这里再赋值NULL是为了保证shift=0的场景，就是tree没有一个节点，只有索引是0的成员保存在root->root_node根节点，此时到这里shift是0，下边的while (shift > 0)不成立。
    //此时该函数返回的父节点node应是NULL，因为返回的slot就指向根节点的root->root_node，它的父节点是NULL
    node = NULL;

    //当本次查找的hot_file_area索引太大，hot_file_area_tree树能容纳的最大hot_file_area索引不能容纳本次要查找的hot_file_area索引
    if(area_index > max_area_index){//hot_file_area_tree 是空树时，这里不成立，二者都是0
        ret = hot_file_area_tree_extend(root,area_index,shift);
	if (ret < 0)
	    return ERR_PTR(ret);
	shift = ret;
	child = root->root_node;
    }
    
    //node是父节点，slot指向父节点node的某个槽位，这个槽位保存child这个节点指针 或者hot_file_area_tree树最下层节点的file_area_tree指针
    while (shift > 0) {
        shift -= TREE_MAP_SHIFT;

	//当前遍历指向radix tree层数的节点是NULL则分配一个新的节点，这里的child肯定是hot_file_area_tree的节点
	if (child == NULL) {
	    //在分配radix tree node前，是spin lock加了hot_file_stat->hot_file_stat_lock锁的，故这里分配内存禁止休眠，否则低内存场景就会占着spin锁休眠，然后导致其他进程获取spin lock失败而soft lockup
            //child = kmem_cache_alloc(hot_file_global_info.hot_file_area_tree_node_cachep,GFP_KERNEL);
            child = kmem_cache_alloc(hot_file_global_info.hot_file_area_tree_node_cachep,GFP_ATOMIC);
	    if (!child)
	        return ERR_PTR(-ENOMEM);
	    memset(child,0,sizeof(struct hot_file_area_tree_node));

	    child->shift = shift;
	    child->offset = offset;
	    child->parent = node;
	    //slot指向child所在父节点的槽位，这里是把新分配的节点hot_file_area_tree_node指针保存到父节点的槽位
	    rcu_assign_pointer(*slot, node_to_entry(child));
	    if (node)
		node->count++;//父节点的子成员树加1
	}
	//这里成立说明child不是hot_file_area_tree的节点，而是树最下层的节点保存的数据
	else if (!hot_file_area_tree_is_internal_node(child))
	    break;

	node = entry_to_node(child);
	//根据area_index索引计算在父节点的槽位索引offset
	offset = (area_index >> node->shift) & TREE_MAP_MASK;
        //根据area_index索引计算在父节点的槽位索引offset，找到在父节点的槽位保存的数据，可能是子节点 或者 保存在hot_file_area_tree树最下层节点的hot_file_area指针
	child = rcu_dereference_raw(node->slots[offset]);
        //根据area_index索引计算在父节点的槽位索引offset，令slot指向在父节点的槽位
	slot = &node->slots[offset];
        /*下轮循环，node= child 成为新的父节点。slot指向父节点node的某个槽位，这个槽位保存child这个节点指针 或者hot_file_area_tree树最下层节点的file_area_tree指针*/
    }
    //page_slot_in_tree是3重指针，*page_slot_in_tree 和 slot 是2重指针，*page_slot_in_tree和slot才能彼此赋值。赋值后*page_slot_in_tree保存的是槽位的地址
    *page_slot_in_tree = slot;
    return node;
}
//释放hot_file_area结构，返回0说明释放成功，返回1说明hot_file_area此时又被访问了，没有释放
int hot_file_area_detele(struct hot_file_global *p_hot_file_global,struct hot_file_stat * p_hot_file_stat,struct hot_file_area *p_hot_file_area)
{
    struct hot_file_area_tree_node *p_hot_file_area_tree_node = p_hot_file_area->parent;
    //取出hot_file_area在父节点的槽位号
    int slot_number = p_hot_file_area->start_index & TREE_MAP_MASK;

    //在释放hot_file_area时，可能正有进程执行hot_file_update_file_status()遍历hot_file_area_tree树中p_hot_file_area指向的hot_file_area结构，
    //这里又在释放hot_file_area结构，因此需要加锁
    spin_lock_irq(&p_hot_file_stat->hot_file_stat_lock);
    if(hot_file_global_info.global_age - p_hot_file_area->file_area_age < HOT_FILE_AREA_FREE_LEVEL - 3){
        spin_unlock_irq(&p_hot_file_stat->hot_file_stat_lock);
	return 1;
    }
    //从hot_file_area tree释放hot_file_area结构，同时也要从hot_file_area_list链表剔除，这个过程还要p_hot_file_stat->hot_file_stat_lock加锁
    list_del(&p_hot_file_area->hot_file_area_list);
    kmem_cache_free(p_hot_file_global->hot_file_area_cachep,p_hot_file_area);
    p_hot_file_area_tree_node->slots[slot_number] = NULL;
    p_hot_file_area_tree_node->count --;//父节点的子成员数减1

    //如果 p_hot_file_area_tree_node没有成员了，则释放p_hot_file_area_tree_node节点，并且向上逐层没有成员的hot_file_area_tree_node父节点
    while(p_hot_file_area_tree_node->count == 0){
        kmem_cache_free(p_hot_file_global->hot_file_area_tree_node_cachep,p_hot_file_area_tree_node);
        p_hot_file_area_tree_node = p_hot_file_area_tree_node->parent;
    }
    spin_unlock_irq(&p_hot_file_stat->hot_file_stat_lock);

    return 0;
}
//如果一个文件file_stat超过一定比例(比如50%)的file_area都是热的，则判定该文件file_stat是热文件，file_stat要移动到global hot_file_head链表。返回1是热文件
int is_file_stat_hot_file(struct hot_file_global *p_hot_file_global,struct hot_file_stat * p_hot_file_stat){
    int ret;

    //如果文件file_stat的file_area个数比较少，则比例按照50%计算
    if(p_hot_file_stat->file_area_count < p_hot_file_global->file_area_count_for_large_file){
        //超过50%的file_area是热的，则判定文件file_stat是热文件
        //if(div64_u64((u64)p_hot_file_stat->file_area_count*100,(u64)p_hot_file_stat->file_area_hot_count) > 50)
	if(p_hot_file_stat->file_area_hot_count > p_hot_file_stat->file_area_count>>1)
	    ret = 1;
        else
	    ret = 0;
    }else{
	//否则，文件很大，则必须热file_area超过文件总file_area数的很多很多，才能判定是热文件。因为此时file_area很多，冷file_area的数目有很多，应该遍历回收这种file_area的page
        if(p_hot_file_stat->file_area_hot_count > (p_hot_file_stat->file_area_count - (p_hot_file_stat->file_area_count >>2)))
	   ret  = 1;
	else
	   ret =  0;
    }
    return ret;
}
//当文件file_stat的file_area个数超过阀值则判定是大文件
int inline is_file_stat_large_file(struct hot_file_global *p_hot_file_global,struct hot_file_stat * p_hot_file_stat)
{
    if(p_hot_file_stat->file_area_hot_count > hot_file_global_info.file_area_count_for_large_file)
	return 1;
    else
	return 0;
}
//模仿page_mapping()判断是否是page cache
inline struct address_space * hot_file_page_mapping(struct page *page)
{
    struct address_space *mapping;
    if (unlikely(PageSlab(page)) || unlikely(PageSwapCache(page)) || PageAnon(page) || page_mapped(page))
        return NULL;

    mapping = page->mapping;
    if ((unsigned long)mapping & PAGE_MAPPING_ANON)
        return NULL;

    return (void *)((unsigned long)mapping & ~PAGE_MAPPING_FLAGS);
}
int hot_file_update_file_status(struct page *page)
{
    struct address_space *mapping;
    int ret = 0;
    struct hot_file_stat * p_hot_file_stat = NULL;
    struct hot_file_area *p_hot_file_area = NULL; 

    //mapping = page_mapping(page);-----这个针对swapcache也是返回非NULL，不能用
    mapping = hot_file_page_mapping(page);
    if(mapping){
        void **page_slot_in_tree = NULL;
	//page所在的hot_file_area的索引
	unsigned int area_index_for_page;
        struct hot_file_area_tree_node *parent_node;

	//如果两个进程同时访问同一个文件的page0和page1，这就就有问题了，因为这个if会同时成立。然后下边针对
	if(!mapping->hot_file_stat){

	    if(!hot_file_global_info.hot_file_stat_cachep || !hot_file_global_info.hot_file_area_cachep){
	        ret =  -ENOMEM;
		goto err;
	    }
            
	    //这里有个问题，hot_file_global_info.hot_file_lock有个全局大锁，每个进程执行到这里就会获取到。合理的是
	    //应该用每个文件自己的spin lock锁!比如hot_file_stat里的spin lock锁，但是在这里，每个文件的hot_file_stat结构还没分配!!!!!!!!!!!!
            spin_lock(&hot_file_global_info.hot_file_lock);
	    //如果两个进程同时访问一个文件，同时执行到这里，需要加锁。第1个进程加锁成功后，分配hot_file_stat并赋值给
	    //mapping->hot_file_stat，第2个进程获取锁后执行到这里mapping->hot_file_stat就会成立
	    if(mapping->hot_file_stat){
	        spin_unlock(&hot_file_global_info.hot_file_lock);
	        goto already_alloc;  
	    }
	    //新的文件分配hot_file_stat,一个文件一个，保存文件热点区域访问数据
	    p_hot_file_stat = kmem_cache_alloc(hot_file_global_info.hot_file_stat_cachep,GFP_ATOMIC);
            if (!p_hot_file_stat) {
	        spin_unlock(&hot_file_global_info.hot_file_lock);
	        printk("%s hot_file_stat alloc fail\n",__func__);
	        ret =  -ENOMEM;
		goto err;
	    }
	    memset(p_hot_file_stat,0,sizeof(struct hot_file_stat));
	    //初始化hot_file_area_hot头结点
	    INIT_LIST_HEAD(&p_hot_file_stat->hot_file_area_hot);
	    INIT_LIST_HEAD(&p_hot_file_stat->hot_file_area_temp);
	    INIT_LIST_HEAD(&p_hot_file_stat->hot_file_area_cold);
	    INIT_LIST_HEAD(&p_hot_file_stat->hot_file_area_free_temp);
	    INIT_LIST_HEAD(&p_hot_file_stat->hot_file_area_free);
	    INIT_LIST_HEAD(&p_hot_file_stat->hot_file_area_refault);

	    //mapping->hot_file_stat记录该文件绑定的hot_file_stat结构，将来判定是否对该文件分配了hot_file_stat
	    mapping->hot_file_stat = (void *)p_hot_file_stat;
	    //hot_file_stat记录mapping结构
	    p_hot_file_stat->mapping = mapping;
	    //把针对该文件分配的hot_file_stat结构添加到hot_file_global_info的hot_file_head_temp链表
	    list_add(&p_hot_file_stat->hot_file_list,&hot_file_global_info.hot_file_head_temp);
	    //新分配的file_stat必须设置in_hot_file_head_temp_list链表
	    set_file_stat_in_hot_file_head_temp_list(p_hot_file_stat);
            spin_lock_init(&p_hot_file_stat->hot_file_stat_lock);

	    spin_unlock(&hot_file_global_info.hot_file_lock);
	}

already_alloc:	    
	    //根据page索引找到所在的hot_file_area的索引，二者关系默认是 hot_file_area的索引 = page索引/6
            area_index_for_page =  page->index >> PAGE_COUNT_IN_AREA_SHIFT;

	    p_hot_file_stat = mapping->hot_file_stat;
            spin_lock_irq(&p_hot_file_stat->hot_file_stat_lock);
	    //根据page索引的hot_file_area的索引，找到对应在file area tree树的槽位，page_slot_in_tree双重指针指向这个槽位。
	    //下边分配真正的hot_file_area结构，把hot_file_area指针保存到这个操作
	    parent_node = hot_file_area_tree_lookup_and_create(&p_hot_file_stat->hot_file_area_tree_root_node,area_index_for_page,&page_slot_in_tree);
            if(IS_ERR(parent_node)){
	        spin_unlock_irq(&p_hot_file_stat->hot_file_stat_lock);
	        printk("%s hot_file_area_tree_insert fail\n",__func__);
		goto err;
	    }
	    //两个进程并发执行该函数时，进程1获取hot_file_stat_lock锁成功，执行hot_file_area_tree_insert()查找page绑定的hot_file_area的
	    //在file_area_tree的槽位，*page_slot_in_tree 是NULL，然后对它赋值。进程2获取hot_file_stat_lock锁后，*page_slot_in_tree就不是NULL了
	    if(*page_slot_in_tree == NULL){//针对当前page索引的hot_file_area结构还没有分配,page_slot_in_tree是槽位地址，*page_slot_in_tree是槽位里的数据，就是hot_file_area指针
		//针对本次page索引，分配hot_file_area一个结构，于是该hot_file_area就代表了page
		p_hot_file_area = kmem_cache_alloc(hot_file_global_info.hot_file_area_cachep,GFP_ATOMIC);
		if (!p_hot_file_area) {
	            spin_unlock_irq(&p_hot_file_stat->hot_file_stat_lock);
		    printk("%s hot_file_area alloc fail\n",__func__);
		    ret =  -ENOMEM;
		    goto err;
		}
		memset(p_hot_file_area,0,sizeof(struct hot_file_area));
	        //把根据page索引分配的hot_file_area结构指针保存到file area tree指定的槽位
	        rcu_assign_pointer(*page_slot_in_tree,p_hot_file_area);

		//set_file_area_in_temp_list(p_hot_file_area);
	        //把新分配的hot_file_area添加到hot_file_area_temp链表
	        list_add(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_temp);
		//保存该hot_file_area对应的起始page索引，一个hot_file_area默认包含6个索引挨着依次增大page，start_index保存其中第一个page的索引
		p_hot_file_area->start_index = area_index_for_page * PAGE_COUNT_IN_AREA;
		//新分配的hot_file_area指向其在hot_file_area_tree的父节点node
		p_hot_file_area->parent = parent_node;
		//如果第一次把索引是0的hot_file_area插入hot_file_area tree，是把该hot_file_area指针保存到hot_file_area tree的根节点，此时parent_node是NULL
		if(parent_node)
		    parent_node->count ++;//父节点下的hot_file_area个数加1
		//令新创建的hot_file_area的last_access_count为1，跟area_access_count相等。如果将来walk_throuth_all_hot_file_area()扫描到hot_file_area
		//的last_access_count和area_access_count都是1，说明后续该hot_file_area就没被访问过。
		//p_hot_file_area->last_access_count = 1;
		
		p_hot_file_stat->file_area_count ++;//文件file_stat的file_area个数加1
		set_file_area_in_temp_list(p_hot_file_area);//新分配的file_area必须设置in_temp_list链表
            }
	    p_hot_file_area = *page_slot_in_tree;
	    //hot_file_global_info.global_age更新了，把最新的global age更新到本次访问的hot_file_area->file_area_age。并对hot_file_area->area_access_count清0，本周期被访问1次则加1
	    if(p_hot_file_area->file_area_age < hot_file_global_info.global_age){
		p_hot_file_area->file_area_age = hot_file_global_info.global_age;
	        p_hot_file_area->area_access_count = 0;
	    }
	    //file_area区域的page被访问的次数加1
	    p_hot_file_area->area_access_count ++;
#if 0
	    //如果p_hot_file_area在当前周期第1次被访问，则把移动到hot_file_area_temp链表头，该链表头的hot_file_area访问比较频繁，链表尾的hot_file_area很少访问。
	    //将来扫描释放page时，也是从hot_file_area_temp链表尾扫描hot_file_area看哪些可以释放
            if(file_area_in_temp_list(p_hot_file_area) && 
		    //p_hot_file_area->area_access_count - p_hot_file_area->last_access_count == 1)
		p_hot_file_area->area_access_count == 1)
	    {
		//如果p_hot_file_area不在hot_file_area_temp链表头，才把它添加到hot_file_area_temp链表头
	        if(p_hot_file_area->hot_file_area_list.prev != &p_hot_file_stat->hot_file_area_temp){
		    list_move(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_temp);
		}
	    }
#endif
	    //如果p_hot_file_area在当前周期第1次被访问，则把移动到hot_file_area_hot链表头，该链表头的hot_file_area访问比较频繁，链表尾的hot_file_area很少访问。
	    //将来walk_throuth_all_hot_file_area()函数扫描释放page时过程，遍历到file_area所处的file_stat并释放内存page时，遍历这些file_stat的hot_file_area_hot
	    //链表尾巴的file_area，如果这些file_area在移动到hot_file_area_hot链表后,很少访问了，则把把这些file_area再降级移动回hot_file_area_temp链表头
            if(p_hot_file_area->area_access_count == 1)
	    {
		//如果p_hot_file_area不在hot_file_area_hot或hot_file_area_temp链表头，才把它添加到hot_file_area_hot或hot_file_area_temp链表头
		//file_stat的hot_file_area_hot或hot_file_area_temp链表头的file_area是最频繁访问的，链表尾的file_area访问频次低，内存回收光顾这些链表尾的file_area
                
		if(file_area_in_temp_list(p_hot_file_area)){
		    if(!list_is_first(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_temp))
		        list_move(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_temp);
		}else if(file_area_in_hot_list(p_hot_file_area)){
		    if(!list_is_first(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_hot))
		        list_move(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_hot);
		}else if(file_area_in_refault_list(p_hot_file_area)){//在refault链表的file_area如果被访问了也移动到链表头
		        list_move(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_refault);
		}
	    }

            //如果p_hot_file_area是冷热不定的，并且hot_file_area的本轮访问次数大于阀值，则设置hot_file_area热，并且把该hot_file_area移动到hot_file_area_hot链表
	    if(file_area_in_temp_list(p_hot_file_area) &&  
		    //p_hot_file_area->area_access_count - p_hot_file_area->last_access_count >= FILE_AREA_HOT_LEVEL){
		p_hot_file_area->area_access_count > FILE_AREA_HOT_LEVEL){

		clear_file_area_in_temp_list(p_hot_file_area);
                //设置hot_file_area 处于 hot_file_area_hot链表
	        set_file_area_in_hot_list(p_hot_file_area);
	        list_move(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_hot);
		/*//hot_file_area->last_access_count保存当前的hot_file_area->area_access_count值。如果hot_file_area移动到hot_file_area_hot链表后
		//hot_file_area还是经常被访问，area_access_count还会一直增加，则这种hot_file_area一直停留在ot_file_area_hot链表。否则area_access_count不再增加，
		//后续扫描到 hot_file_area_hot链表有这种hot_file_area，就要把它再移动回hot_file_area_temp链表
		p_hot_file_area->last_access_count = p_hot_file_area->area_access_count;*/

		//该文件的热hot_file_stat数加1
                p_hot_file_stat->file_area_hot_count ++;
                
		//如果文件file_stat的file_area很多都是热的，判定file_stat是热文件，则把hot_file_stat移动到global hot_file_head链表，
		//global hot_file_head链表上的hot_file_stat不再扫描上边的hot_file_area，有没有必要这样做??????????????????????
		if(is_file_stat_hot_file(&hot_file_global_info,p_hot_file_stat)){
		    //外层有spin_lock_irq(&p_hot_file_stat->hot_file_stat_lock)，这里不应该再关中断，只能spin_lock加锁!!!!!!!!!!!!!!
                    spin_lock(&hot_file_global_info.hot_file_lock);
		    clear_file_stat_in_hot_file_head_temp_list(p_hot_file_stat);
		    set_file_stat_in_hot_file_head_list(p_hot_file_stat);
	            list_move(&p_hot_file_stat->hot_file_list,&hot_file_global_info.hot_file_head);
                    spin_unlock(&hot_file_global_info.hot_file_lock);
		}
	    }

	    //如果file_area处于file_stat的free_list或free_temp_list链表
            if(file_area_in_free_list(p_hot_file_area) || file_area_in_free_temp_list(p_hot_file_area)){
		if(file_area_in_free_list(p_hot_file_area))
		    clear_file_area_in_free_list(p_hot_file_area);
		else
		    clear_file_area_in_free_temp_list(p_hot_file_area);

                //file_area 的page被内存回收后，过了仅1s左右就又被访问则发生了refault，把该hot_file_area移动到hot_file_area_refault链表，
		//不再参与内存回收扫描!!!!需要设个保护期限制
		smp_rmb();
    		if(p_hot_file_area->shrink_time && (ktime_to_ms(ktime_get()) - (p_hot_file_area->shrink_time << 10) < 1000)){
		    p_hot_file_area->shrink_time = 0;
	            set_file_area_in_refault_list(p_hot_file_area);
		    list_move(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_refault);
                }else{
	            //file_area此时正在被内存回收而移动到了file_stat的free_list或free_temp_list链表，则直接移动到hot_file_stat->hot_file_area_temp链表头
		    set_file_area_in_temp_list(p_hot_file_area);
		    //if(file_area_in_free_list(p_hot_file_area))
	            //    list_move(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_temp_large);
		    //else
			list_move(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_temp);
		}
	    }
            //如果file_area处于file_area链表，但是p_hot_file_area->shrink_time不是0.这说明该file_area在之前walk_throuth_all_hot_file_area()函数中扫描
	    //判定该file_area是冷的，然后回收内存page。但是回收内存时，正好这个file_area又被访问了，则把file_area移动到file_stat->hot_file_area_temp链表。
	    //但是内存回收流程执行到hot_file_isolate_lru_pages()函数因并发问题没发现该file_area最近被访问了，只能继续回收该file_area的page。需要避免回收这种
	    //热file_area的page。于是等该file_area下次被访问，执行到这里，if成立，把该file_area移动到file_stat->hot_file_area_refault链表。这样未来一段较长时间
	    //可以避免再次回收该file_area的page。具体详情看hot_file_isolate_lru_pages()函数里的注释
	    if(file_area_in_temp_list(p_hot_file_area) && (p_hot_file_area->shrink_time != 0)){
	        p_hot_file_area->shrink_time = 0;
		clear_file_area_in_temp_list(p_hot_file_area);
	        set_file_area_in_refault_list(p_hot_file_area);
		list_move(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_refault);
	    }
	    spin_unlock_irq(&p_hot_file_stat->hot_file_stat_lock);

	    //文件file_stat的file_area个数大于阀值则移动到global hot_file_head_large_file_temp链表
	    if(is_file_stat_large_file(&hot_file_global_info,p_hot_file_stat)){
		smp_rmb();
		//walk_throuth_all_hot_file_area()函数中也有的大量的访问file_stat或file_area状态的，他们需要smp_rmb()吗，需要留意???????????????????????????????????????
		if(!file_stat_in_large_file(p_hot_file_stat)){
                    spin_lock(&hot_file_global_info.hot_file_lock);
		    //设置file_stat是大文件
		    set_file_stat_in_large_file(p_hot_file_stat);
	            list_move(&p_hot_file_stat->hot_file_list,&hot_file_global_info.hot_file_head_temp_large);
                    spin_unlock(&hot_file_global_info.hot_file_lock);
		}
	    }
	     
	    if(open_shrink_printk)
	        printk("%s %s %d p_hot_file_stat:0x%llx status:0x%x p_hot_file_area:0x%llx status:0x%x hot_file_area->area_access_count:%d hot_file_area->file_area_age:%lu page:0x%llx page->index:%ld file_area_hot_count:%d file_area_count:%d shrink_time:%d start_index:%ld page_slot_in_tree:0x%llx tree-height:%d\n",__func__,current->comm,current->pid,(u64)p_hot_file_stat,p_hot_file_stat->file_stat_status,(u64)p_hot_file_area,p_hot_file_area->file_area_state,p_hot_file_area->area_access_count,p_hot_file_area->file_area_age,(u64)page,page->index,p_hot_file_stat->file_area_hot_count,p_hot_file_stat->file_area_count,p_hot_file_area->shrink_time,p_hot_file_area->start_index,(u64)page_slot_in_tree,p_hot_file_stat->hot_file_area_tree_root_node.height);
	   
	    if(p_hot_file_area->file_area_age > hot_file_global_info.global_age)
	        panic("p_hot_file_area->file_area_age:%ld > hot_file_global_info.global_age:%ld\n",p_hot_file_area->file_area_age,hot_file_global_info.global_age);
/*	    
	}	
	else//走到这个分支，说明之前为当前访问的文件分配了hot_file_stat结构
	{
            struct hot_file_area_tree_node *parent_node;
	    //从mapping得到该文件绑定的hot_file_stat结构
	    p_hot_file_stat = mapping->hot_file_stat;
	   
	    //根据page索引找到所在的hot_file_area的索引，二者关系默认是 hot_file_area的索引 = page索引/6
            area_index_for_page =  page->index >> PAGE_COUNT_IN_AREA_SHIFT;
	    //需要加锁，多个进程同时访问同一个page索引，得到同一个hot_file_area结构，同时令 p_hot_file_area->area_access_count ++，那就出问题了，多线程同时对同一个变量修改必须加锁
	    spin_lock_irq(&p_hot_file_stat->hot_file_stat_lock);
	    //根据page索引的hot_file_area的索引，找到对应在file area tree树的槽位，page_slot_in_tree双重指针指向这个槽位。
	    //下边分配真正的hot_file_area结构，把hot_file_area指针保存到这个操作
	    parent_node = hot_file_area_tree_lookup_and_create(&mapping->hot_file_stat->root_node,area_index_for_page,&page_slot_in_tree);
            if(IS_ERR(parent_node)){
	        printk("%s hot_file_area_tree_insert fail\n",__func__);
		goto err;
	    }
	    if(*page_slot_in_tree == NULL){//针对当前page索引的hot_file_area结构还没有分配
		//针对本次page索引，分配hot_file_area一个结构，于是该hot_file_area就代表了page
		p_hot_file_area = kmem_cache_alloc(hot_file_global_info.hot_file_area_cachep,GFP_ATOMIC);
		if (!p_hot_file_area) {
		    printk("%s hot_file_area alloc fail\n",__func__);
		    ret =  -ENOMEM;
		    goto err;
		}
		memset(hot_file_area_cache,sizeof(hot_file_area),0);
	        //把根据page索引分配的hot_file_area结构指针保存到file area tree指定的槽位
	        rcu_assign_pointer(page_slot_in_tree,p_hot_file_area);
	        //把新分配的hot_file_area添加到hot_file_area_temp链表
	        list_add_rcu(p_hot_file_area->hot_file_area_list,p_hot_file_stat->hot_file_area_temp);
		//保存该hot_file_area对应的起始page索引，一个hot_file_area默认包含6个索引挨着依次增大page，start_index保存其中第一个page的索引
		p_hot_file_area->start_index = (page->index >> PAGE_COUNT_IN_AREA_SHIFT) * PAGE_COUNT_IN_AREA;
		//新分配的hot_file_area指向其在hot_file_area_tree的父节点node
		p_hot_file_area->parent = parent_node;
            }
	    p_hot_file_area = *page_slot_in_tree; 
	    //hot_file_area热点区域访问数加1，表示这个hot_file_area的区域的page被访问的次数加1
	    p_hot_file_area->area_access_count ++;
	    //该文件访问次数加1
	    p_hot_file_stat->file_access_count++;
	    spin_unlock_irq(&p_hot_file_stat->hot_file_stat_lock);

        }
*/
    }

    return 0;

err:
    //不能因为走了err分支，就释放p_hot_file_stat和p_hot_file_area结构。二者都已经添加到ot_file_global_info.hot_file_head 或 p_hot_file_stat->hot_file_area_temp链表，
    //不能释放二者的数据结构。是这样吗，得再考虑一下???????????????????????
    if(p_hot_file_stat){
	//kmem_cache_free(hot_file_global_info.hot_file_stat_cachep,p_hot_file_stat);
    }
    if(p_hot_file_area){
	//kmem_cache_free(hot_file_global_info.hot_file_area_cachep,p_hot_file_area);
    }
    return ret;
}
EXPORT_SYMBOL(hot_file_update_file_status);
/*
//把hot_file_area对应的page从lru链表剔除，然后添加到链表dst,这个过程需要
inline int get_page_from_hot_file_area(struct hot_file_stat * p_hot_file_stat,struct hot_file_area *p_hot_file_area,struct list_head *dst)
{
    int i;
    struct address_space *mapping = p_hot_file_stat->mapping;
    //得到hot_file_area对应的page
    for(i = 0;i < PAGE_COUNT_IN_AREA;i ++){
        page = xa_load(&mapping->i_pages, hot_file_area->start_index + i)
	if (page && !xa_is_value(page)) {
            list_move(&page->lru,dst);
	}
    }
}*/
static int __hot_file_isolate_lru_pages(pg_data_t *pgdat,struct page * page,struct list_head *dst,isolate_mode_t mode)
{
    struct lruvec *lruvec;
    int lru;

    lruvec = mem_cgroup_lruvec(page->mem_cgroup, pgdat);
    lru = page_lru_base_type(page);

    /*__isolate_lru_page里清除page的PageLRU属性，因为要把page从lru链表剔除了，并且令page的引用计数加1*/
    switch (__isolate_lru_page(page, mode)) {
    case 0:
	    //nr_pages = hpage_nr_pages(page);
	    //nr_taken += nr_pages;
	    //nr_zone_taken[page_zonenum(page)] += nr_pages;
	    //page原本在lru链表，现在要移动到其他链表，要把page在链表的上一个page保存到async_shrink_page
	    //update_async_shrink_page(page);
	    //list_move(&page->lru, dst);

	    //把page从lru链表剔除，并减少page所属lru链表的page数
	    del_page_from_lru_list(page, lruvec, lru + PageActive(page));
	    //再把page添加到dst临时链表
	    list_add(&page->lru,dst);
	    return 0;

    case -EBUSY:
	    if(open_shrink_printk)
		printk("2:%s %s %d page:0x%llx page->flags:0x%lx EBUSY\n",__func__,current->comm,current->pid,(u64)page,page->flags);
	    break;

    default:
	if(open_shrink_printk)
	    printk("3:%s %s %d PageUnevictable:%d PageLRU:%d\n",__func__,current->comm,current->pid,PageUnevictable(page),PageLRU(page));

	    BUG();
    }
    
    /*更新 acitve/inactive file 链入链表的page数，减少nr_taken个，因为page将要从lru链表移除*/
    //update_lru_sizes(lruvec, lru, nr_zone_taken);------
    return -1;
}
//遍历p_hot_file_stat对应文件的hot_file_area_free链表上的hot_file_area结构，找到这些hot_file_area结构对应的page，这些page被判定是冷页，可以回收
static unsigned long hot_file_isolate_lru_pages(struct hot_file_global *p_hot_file_global,struct hot_file_stat * p_hot_file_stat,
	                               struct list_head *hot_file_area_free)
{
    struct hot_file_area *p_hot_file_area,*tmp_hot_file_area;
    int i;
    struct address_space *mapping = p_hot_file_stat->mapping;
    //unsigned long nr_zone_taken[MAX_NR_ZONES] = { 0 };
    isolate_mode_t mode = 0;
    pg_data_t *pgdat = NULL;
    struct page *page;
    unsigned int isolate_pages = 0;
    struct list_head *dst;
    
    list_for_each_entry_safe(p_hot_file_area,tmp_hot_file_area,hot_file_area_free,hot_file_area_list){
        if(open_shrink_printk)
	    printk("%s %s %d p_hot_file_global:0x%llx p_hot_file_stat:0x%llx status:0x%x p_hot_file_area:0x%llx status:0x%x\n",__func__,current->comm,current->pid,(u64)p_hot_file_global,(u64)p_hot_file_stat,p_hot_file_stat->file_stat_status,(u64)p_hot_file_area,p_hot_file_area->file_area_state);

#if 0 
	//--------这段注释不要删除-------------------很重要

	/*这里要对p_hot_file_area->shrink_time的赋值需要加锁。
	  情况1：这里先加锁。对p_hot_file_area->shrink_time赋值，然后1s内执行hot_file_update_file_status()获取锁，访问到该file_area，则判定该file_area是refault file_area。
	  情况2:hot_file_update_file_status()先加锁，访问该file_area，令p_hot_file_global->global_age和p_hot_file_area->file_area_age相等，则
	        这里直接continue，不再释放hot_file_area的page。

	  有了hot_file_stat_lock加锁，完美解决p_hot_file_area->shrink_time在这里的赋值 和 在hot_file_update_file_status()函数的访问 时，数据不同步的问题，但是
	  这个加锁真的有必要吗????????要多次加锁,太浪费性能了，影响hot_file_update_file_status()函数的spin_lock_irq(&p_hot_file_stat->hot_file_stat_lock)加锁
	 */
	spin_lock_irq(&p_hot_file_stat->hot_file_stat_lock);
	//如果此时hot_file_area又被访问了，则不再释放，并移动回hot_file_area_temp链表
	//if(p_hot_file_area->area_access_count - p_hot_file_area->last_access_count  0){
	if(p_hot_file_global->global_age == p_hot_file_area->file_area_age){
            list_move(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_temp);
	    set_file_area_in_temp_list(p_hot_file_area);
	    spin_unlock_irq(&p_hot_file_stat->hot_file_stat_lock);
	    continue;
	}
        //获取hot_file_area内存回收的时间，ktime_to_ms获取的时间是ms，右移10近似除以1000，变成单位秒
	p_hot_file_area->shrink_time = ktime_to_ms(ktime_get()) >> 10;
	spin_unlock_irq(&p_hot_file_stat->hot_file_stat_lock);
#else
	/*对p_hot_file_area->shrink_time的赋值不再加锁，
	 *情况1:如果这里先对p_hot_file_area->shrink_time赋值，然后1s内hot_file_update_file_status()函数访问该file_area，则file_area被判定是refault file_area。
	 *情况2:先有hot_file_update_file_status()函数访问该file_area,但p_hot_file_area->shrink_time还是0，则file_area无法被判定是refault file_area.
          但因为file_area处于file_stat->hot_file_area_free_temp链表上，故把file_area移动到file_stat->hot_file_area_temp链表。然后这里执行到
	  if(!file_area_in_free_list(p_hot_file_area))，if成立，则不再不再回收该file_area的page。这种情况也没事

	 *情况3:如果这里快要对p_hot_file_area->shrink_time赋值，但是先有hot_file_update_file_status()函数访问该file_area，但p_hot_file_area->shrink_time还是0，
	        则file_area无法被判定是refault file_area.但因为file_area处于file_stat->hot_file_area_free_temp链表上，故把file_area移动到file_stat->hot_file_area_temp链表。
		但是，在把file_area移动到file_stat->hot_file_area_free_temp链表上前，这里并发先执行了对p_hot_file_area->shrink_time赋值当前时间和
		if(!file_area_in_free_list(p_hot_file_area))，但if不成立。然后该file_area的page还要继续走内存回收流程。相当于刚访问过的file_area却被回收内存page了.
		这种情况没有办法。只有在hot_file_update_file_status()函数中，再次访问该file_area时，发现p_hot_file_area->shrink_time不是0，说明刚该file_area经历过一次
		重度refault现象，于是也要把file_area移动到refault链表。注意，此时file_area处于file_stat->hot_file_area_free_temp链表。
	 * */

    	//获取hot_file_area内存回收的时间，ktime_to_ms获取的时间是ms，右移10近似除以1000，变成单位秒
	p_hot_file_area->shrink_time = ktime_to_ms(ktime_get()) >> 10;
	smp_mb();
	//正常此时file_area处于file_stat->hot_file_area_free_temp链表，但如果正好此时该file_area被访问了，则就要移动到file_stat->hot_file_area_temp链表。
	//这种情况file_area的page就不能被释放了
	if(!file_area_in_free_list(p_hot_file_area)){
	    p_hot_file_area->shrink_time = 0;
	    continue;
	}
#endif
	//设置 hot_file_area的状态为 in_free_list
	//set_file_area_in_free_list(p_hot_file_area);------这里不再设置set_file_area_in_free_list的状态，因为设置需要hot_file_stat_lock加锁，浪费性能
	
	//得到hot_file_area对应的page
	for(i = 0;i < PAGE_COUNT_IN_AREA;i ++){
	    page = xa_load(&mapping->i_pages, p_hot_file_area->start_index + i);
	    if (page && !xa_is_value(page)) {
		//正常情况每个文件的page cache的page都应该属于同一个node，进行一次spin_lock_irq(&pgdat->lru_lock)就行，但是也有可能属于不同的内存节点node，
		//那就需要每次出现新的page所属的内存节点node的pgdat=page_pgdat(page)时，那就把老的pgdat=page_pgdat(page)解锁，对新的pgdat=page_pgdat(page)加锁
		//pgdat != page_pgdat(page)成立说明前后两个page所属node不一样，那就要把前一个page所属pgdat spin unlock，然后对新的page所属pgdat spin lock
                if(unlikely(pgdat != page_pgdat(page)))
		{
		    //第一次进入这个if，pgdat是NULL，此时不用spin unlock，只有后续的page才需要
		    if(pgdat){
			//对之前page所属pgdat进行spin unlock
                        spin_unlock_irq(&pgdat->lru_lock);
		    }
		    //pgdat最新的page所属node节点对应的pgdat
		    pgdat = page_pgdat(page);
		    if(pgdat != p_hot_file_global->p_hot_file_node_pgdat[pgdat->node_id].pgdat)
	                panic("pgdat not equal\n");
		    //对新的page所属的pgdat进行spin lock
		    spin_lock_irq(&pgdat->lru_lock);
		}
		//在把page从lru链表移动到dst临时链表时，必须spin_lock_irq(&pgdat->lru_lock)加锁
		//list_move(&page->lru,dst);-----在下边的hot_file_area_isolate_lru_pages实现
		
                /*这里又是另外一个核心点。由于现在前后两次的page不能保证处于同一个内存node、同一个memory、同一个lruvec，因此
		 * 只能每来一个page，都执行类似原版内存回收的isolate_lru_pages，判断能否隔离，可以隔离的话。再计算当前page所属的
		 * pgdat、lruvec、active/inacitve lru编号，然后把page从lru链表剔除，再令lru链表的page数减1。而原来内存回收的isolate_lru_pages函数，进行隔离的
		 * 多个page一定来自同一个pgdat、lruvec、active/inacitve lru编号，就不用针对隔离的每个page再计算这些参数了。并且把所有page
		 * 都隔离后，同一执行update_lru_sizes()令lru链表的page数减去隔离成功的page数。显然，这样更节省cpu，我的方法稍微有点耗cpu，尤其是隔离page多的情况下*/
		dst = &p_hot_file_global->p_hot_file_node_pgdat[pgdat->node_id].pgdat_page_list;//把page保存到对应node的hot_file_node_pgdat链表上
		if(__hot_file_isolate_lru_pages(pgdat,page,dst,mode) == 0){
		    goto err;
		}
		isolate_pages ++;
	    }
	}
    }
err:    
    if(pgdat)
	spin_unlock_irq(&pgdat->lru_lock);

    return isolate_pages;
}
//static void putback_inactive_pages(struct lruvec *lruvec, struct list_head *page_list)
static void hot_file_putback_inactive_pages(struct pglist_data *pgdat, struct list_head *page_list)
{
	//struct pglist_data *pgdat = lruvec_pgdat(lruvec);
	LIST_HEAD(pages_to_free);
        struct lruvec *lruvec;

	spin_lock_irq(&pgdat->lru_lock);
	/*
	 * Put back any unfreeable pages.
	 */
	while (!list_empty(page_list)) {
		struct page *page = lru_to_page(page_list);
		int lru;

                if(open_shrink_printk)
		    printk("1:%s %s %d page:0x%llx page->flags:0x%lx\n",__func__,current->comm,current->pid,(u64)page,page->flags);

		VM_BUG_ON_PAGE(PageLRU(page), page);
		list_del(&page->lru);
		if (unlikely(!page_evictable(page))) {
			spin_unlock_irq(&pgdat->lru_lock);
			putback_lru_page(page);
			spin_lock_irq(&pgdat->lru_lock);
			continue;
		}
                /*怎么保证这些内存释放失败的page添加会原有的lru链表呢？page->mem_cgroup 是page锁绑定的memcg，再有memcg找到它的lruvec，完美*/
		lruvec = mem_cgroup_page_lruvec(page, pgdat);

		SetPageLRU(page);
		lru = page_lru(page);
		add_page_to_lru_list(page, lruvec, lru);

		/*if (is_active_lru(lru)) {
			int file = is_file_lru(lru);
			int numpages = hpage_nr_pages(page);
			reclaim_stat->recent_rotated[file] += numpages;
		}*/
		if (put_page_testzero(page)) {
                        if(open_shrink_printk)
		            printk("2:%s %s %d put_page_testzero page:0x%llx page->flags:0x%lx PageCompound:%d\n",__func__,current->comm,current->pid,(u64)page,page->flags,PageCompound(page));
			__ClearPageLRU(page);
			__ClearPageActive(page);
			del_page_from_lru_list(page, lruvec, lru);

			if (unlikely(PageCompound(page))) {
				spin_unlock_irq(&pgdat->lru_lock);
				mem_cgroup_uncharge(page);
				(*get_compound_page_dtor(page))(page);
				spin_lock_irq(&pgdat->lru_lock);
			} else
				list_add(&page->lru, &pages_to_free);
		}
	}
        spin_unlock_irq(&pgdat->lru_lock);
	/*
	 * To save our caller's stack, now use input list for pages to free.
	 */
	list_splice(&pages_to_free, page_list);
}

static unsigned long hot_file_shrink_pages(struct hot_file_global *p_hot_file_global)
{
    int i;
    unsigned long nr_reclaimed = 0;
    struct reclaim_stat stat = {};

    struct scan_control sc = {
	.gfp_mask = GFP_KERNEL,
	.order = 1,
	.priority = DEF_PRIORITY,
	.may_writepage = 1,
	.may_unmap = 0,
	.may_swap = 0,
	.reclaim_idx = MAX_NR_ZONES - 1
    };

    struct hot_file_node_pgdat *p_hot_file_node_pgdat = p_hot_file_global->p_hot_file_node_pgdat;
    //遍历每个内存节点上p_hot_file_node_pgdat[i]->pgdat_page_list 上的page，释放它，
    for(i = 0;i < hot_file_global_info.node_count;i ++){
        struct list_head *p_pgdat_page_list = &p_hot_file_node_pgdat[i].pgdat_page_list;
        if(open_shrink_printk)
            printk("1:%s %s %d node:0x%d pgdat:0x%llx\n",__func__,current->comm,current->pid,i,(u64)p_hot_file_node_pgdat[i].pgdat);
        if(!list_empty(p_pgdat_page_list)){
	    //开始释放p_hot_file_node_pgdat[i]->pgdat_page_list链表上的page
            nr_reclaimed += async_shrink_free_page(p_hot_file_node_pgdat[i].pgdat,NULL,p_pgdat_page_list,&sc,&stat);
	    //把p_hot_file_node_pgdat[i]->pgdat_page_list链表上未释放成功的page再移动到lru链表
	    hot_file_putback_inactive_pages(p_hot_file_node_pgdat[i].pgdat,p_pgdat_page_list);

	    //此时p_hot_file_node_pgdat[pgdat->node_id]->pgdat_page_list链表上还残留的page没人再用了，引用计数是0，这里直接释放
	    mem_cgroup_uncharge_list(p_pgdat_page_list);
	    free_unref_page_list(p_pgdat_page_list);
	}
    }
    return nr_reclaimed;
}
#if 0
-int walk_throuth_all_hot_file_area(struct hot_file_global *p_hot_file_global)
{
    struct hot_file_stat * p_hot_file_stat,*p_hot_file_stat_temp;
    struct hot_file_area *p_hot_file_area,*p_hot_file_area_temp;
    LIST_HEAD(hot_file_area_list);
    LIST_HEAD(global_hot_file_head_temp_list);
    LIST_HEAD(hot_file_stat_free_list);
    unsigned int scan_file_area_count  = 0;
    unsigned int scan_file_area_max = 1024;
    unsigned int scan_file_stat_count  = 0;
    unsigned int scan_file_stat_max = 64;
    
    unsigned int file_area_count_in_cold_list = 0;
    unsigned int file_stat_count_in_cold_list = 0;
    unsigned int scan_cold_file_stat_count,scan_cold_file_area_count;
    
    spin_lock_irq(&p_hot_file_global->hot_file_lock);
    //先从global hot_file_head_temp链表尾隔离scan_file_stat_max个hot_file_stat到 global_hot_file_head_temp_list 临时链表
    list_for_each_entry_safe_reverse(p_hot_file_stat,p_hot_file_stat_temp,&p_hot_file_global->hot_file_head_temp,hot_file_list){
	//这里把hot_file_stat 移动到 global_hot_file_head_temp_list 临时链表，用不用清理的file_stat的 file_area_in_temp_list 标记????????????????????????????????????????????
	//这里用不用把hot_file_stat->file_stat_status设置成无效，因为不在hot_file_global的任何链表了?????????????????????????????????????
        list_move(&p_hot_file_stat->hot_file_list,&global_hot_file_head_temp_list);
	if(scan_file_stat_count ++ > scan_file_stat_max)
	    break;
    }
    spin_unlock_irq(&p_hot_file_global->hot_file_lock);

    //在遍历hot_file_global->hot_file_head_temp链表期间，可能创建了新文件并创建了hot_file_stat并添加到hot_file_global->hot_file_head_temp链表，
    //下边遍历hot_file_global->hot_file_head链表成员期间，是否用hot_file_global_info.hot_file_lock加锁？不用，因为遍历链表期间
    //向链表添加成员没事，只要不删除成员！想想我写的内存屏障那片文章讲解list_del_rcu的代码
    //list_for_each_entry_safe_reverse(p_hot_file_stat,&p_hot_file_global->hot_file_head_temp,hot_file_list)//从链表尾开始遍历，链表尾的成员更老，链表头的成员是最新添加的
    list_for_each_entry_safe(p_hot_file_stat,p_hot_file_stat_temp,&global_hot_file_head_temp_list,hot_file_list)//本质就是遍历p_hot_file_global->hot_file_head_temp链表尾的hot_file_stat
    {
        if(!file_stat_in_hot_file_head_temp(p_hot_file_stat))
	    panic("p_hot_file_stat:0x%llx status:%d not in free_temp_list\n",(u64)p_hot_file_stat,p_hot_file_stat->file_stat_status);

	file_area_count_in_cold_list = 0;
	//注意，这里扫描的global hot_file_head_temp上的hot_file_stat肯定有冷hot_file_area，因为hot_file_stat只要50%的hot_file_area是热的，hot_file_stat就要移动到
	//global hot_file_head 链表。
        list_for_each_entry_safe_reverse(p_hot_file_area,p_hot_file_area_temp,&p_hot_file_stat->hot_file_area_temp,hot_file_area_list)//从链表尾开始遍历，链表尾的成员更老，链表头的成员是最新添加的
	{
	    if(!file_area_in_temp_list(p_hot_file_area))
		panic("file_area_in_temp_list:0x%llx status:%d not in hot_file_area_temp\n",(u64)p_hot_file_area,p_hot_file_area->file_area_state);

	    scan_file_area_count ++;
	    //本周期内，该p_hot_file_area 依然没有被访问，移动到hot_file_area_cold链表头
	    if(p_hot_file_area->area_access_count == p_hot_file_area->last_access_count){

	        spin_lock_irq(&p_hot_file_stat->hot_file_stat_lock);
		set_file_area_in_cold_list(p_hot_file_area);
		//需要加锁，此时可能有进程执行hot_file_update_file_status()并发向该p_hot_file_area前或者后插入新的hot_file_area，这里是把该p_hot_file_area从hot_file_area_temp链表剔除，存在同时修改该p_hot_file_area在hot_file_area_temp链表前的hot_file_area结构的next指针和在链表后的hot_file_area结构的prev指针，并发修改同一个变量就需要加锁。
                list_move(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_cold);
	        spin_unlock_irq(&p_hot_file_stat->hot_file_stat_lock);
		file_area_count_in_cold_list ++;
                
		//把有冷hot_file_area的hot_file_stat移动到global cold_file_head链表，并设置file_stat_in_head_temp_list
		if(!file_stat_in_hot_file_head_temp(p_hot_file_stat)){
		    //是否会存在并发设置p_hot_file_stat->file_stat_status的情况??????????????? 这里没有加锁，需要考虑这点???????????????
		    set_file_stat_in_head_temp_list(p_hot_file_stat);
		    //这里不用加锁，此时p_hot_file_stat是在 global_hot_file_head_temp_list临时链表，并且把p_hot_file_stat移动到
		    //global cold_file_head链表，只在walk_throuth_all_hot_file_area()函数单线程操作，不存在并发
		    list_move(&p_hot_file_stat->hot_file_list,&p_hot_file_global->cold_file_head);
		    //本轮扫描移动到global cold_file_head链表头的file_stat个数
		    file_stat_count_in_cold_list ++;
		}
	    }
	    
	    //凡是扫描到的hot_file_area都令last_access_count与area_access_count相等，下轮周期该hot_file_area被访问，则area_access_count就大于last_access_count。
	    //这样做有两个用处。1:该hot_file_area本轮扫描last_access_count与area_access_count就相等，则前边把 p_hot_file_area移动到了hot_file_area_cold链表。
	    //如果后续该 p_hot_file_area又被访问了则last_access_count与area_access_count不等，则把p_hot_file_area移动到hot_file_area_temp链表头。下轮扫描，从
	    //该文件p_hot_file_stat的链表尾扫到的p_hot_file_area都是新的，不会重复扫描。 情况2:本轮扫描p_hot_file_area的last_access_count与area_access_count不等，
	    //说明是热hot_file_area，这里令last_access_count与area_access_count相等，看下轮扫描周期带来时，该p_hot_file_area是否会被再被访问而area_access_count加1，
	    //没被访问那last_access_count与area_access_count下轮扫描就相等，就可以把p_hot_file_area移动到hot_file_area_cold链表了
            p_hot_file_area->last_access_count = p_hot_file_area->area_access_count;
	}
	//把本轮扫描 移动到该文件hot_file_stat的hot_file_area_cold链表上的file_area个数保存到p_hot_file_stat->file_area_count_in_cold_list
        if(file_area_count_in_cold_list > 0)
	    p_hot_file_stat->file_area_count_in_cold_list = file_area_count_in_cold_list;

	if(!list_empty(&p_hot_file_stat->hot_file_area_free)){
	    //hot_file_area_free链表上长时间没访问的hot_file_area释放掉
	    list_for_each_entry_safe_reverse(p_hot_file_area,p_hot_file_area_temp,&p_hot_file_stat->hot_file_area_free,hot_file_area_list){
	        if(p_hot_file_area->area_access_count - p_hot_file_area->last_access_count > 0){
	            spin_lock_irq(&p_hot_file_stat->hot_file_stat_lock);
                    list_move(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_temp);
	            set_file_area_in_temp_list(p_hot_file_area);
	            spin_unlock_irq(&p_hot_file_stat->hot_file_stat_lock);

		    p_hot_file_area->cold_time = 0;//冷却计数清0
 	        }else{
		    p_hot_file_area->cold_time ++;
		    //hot_file_area冷的次数达到阀值则释放掉它
		    if(p_hot_file_area->cold_time > HOT_FILE_AREA_FREE_LEVEL)
		        hot_file_area_detele(p_hot_file_global,p_hot_file_stat,p_hot_file_area);
		}
	    }
	}
        //防止在for循环耗时太长，限制遍历的文件hot_file_stat数。这里两个问题 问题1:单个hot_file_stat上的hot_file_area太多了，只扫描一个hot_file_stat这里就
	//break跳出循环了。这样下边就把global_hot_file_head_temp_list残留的hot_file_stat移动到global hot_file_head_temp链表头了。下轮扫描从
	//global hot_file_head_temp尾就扫描不到该hot_file_stat了。合理的做法是，把这些压根没扫描的hot_file_stat再移动到global hot_file_head_temp尾。问题2：
	//还是 单个hot_file_stat上的hot_file_area太多了，没扫描完，下次再扫描该hot_file_stat时，直接从上次结束的hot_file_area位置处继续扫描，似乎更合理。
	//hot_file_stat断点hot_file_area继续扫描！但是实现起来似乎比较繁琐，算了
	if(scan_file_area_count > scan_file_area_max)
	    break;
    }
    //把global_hot_file_head_temp_list残留的hot_file_stat移动到global hot_file_head_temp链表头。这样做就保证本轮从global hot_file_head_temp尾扫到的
    //hot_file_stat要么移动到了globa cold_file_head链表，要么移动到global hot_file_head_temp链表头。这样下轮从global hot_file_head_temp尾扫到的hot_file_stat之前没扫描过。
    //错了！上边扫描的global hot_file_head_temp链表尾的hot_file_stat肯定有冷hot_file_area。因为hot_file_stat只要50%的hot_file_area是热的，hot_file_stat就要移动到
    //global hot_file_head 链表。global hot_file_head_temp链表上的hot_file_stat肯定有hot_file_area。这里还残留在global_hot_file_head_temp_list上的hot_file_stat,
    //本轮就没有扫描到，因此要移动到global hot_file_head_temp链表尾，下轮扫描继续扫描这些hot_file_stat
    if(!list_empty(&global_hot_file_head_temp_list)){
        spin_lock_irq(&p_hot_file_global->hot_file_lock);
	//set_file_stat_in_head_temp_list(p_hot_file_stat);//不用再设置这些hot_file_stat的状态，这些hot_file_stat没有移动到global hot_file_area_cold链表，没改变状态
        //list_splice(&global_hot_file_head_temp_list,&p_hot_file_global->hot_file_head_temp);//移动到global hot_file_head_temp链表头
        list_splice_tail(&global_hot_file_head_temp_list,&p_hot_file_global->hot_file_head_temp);//global hot_file_head_temp链表尾
        spin_unlock_irq(&p_hot_file_global->hot_file_lock);
    }
    
    //遍历hot_file_area_cold链表尾上p_hot_file_area->old_file_area_count_in_cold_list个hot_file_stat，这些hot_file_stat上一轮扫描因为有
    //冷file_area而移动到了hot_file_stat->hot_file_area_cold链表头。现在新的一轮扫描，再次从hot_file_stat->hot_file_area_cold链表尾巴
    //如果这些hot_file_stat里上一轮被判定是冷的file_area还是冷的，那就释放这些file_area
    /*这个for循环是内存回收的关键，这里要借助上轮扫描在上边的循环中:把有冷file_area的hot_file_stat移动到了global->hot_file_area_cold链表头。现在新的一轮扫描，
     次从hot_file_stat->hot_file_area_cold链表尾巴扫描前一轮扫描移动到hot_file_stat->hot_file_area_cold的hot_file_stat,下边详述

     1:第1轮扫描，上边的循环，从global->hot_file_head_temp链表尾巴扫到64个hot_file_stat，它们全有冷的hot_file_area，于是这些hot_file_stat全移动到了global->hot_file_area_cold链表头,
       注意是，链表头，要记录一共移动了多少个hot_file_stat保存到file_area_count_in_cold_list临时变量.同时呢，这些hot_file_stat链表hot_file_area_temp尾扫到的被判定
       冷的hot_file_area(即area_access_count和last_access_count相等)要移动到hot_file_stat->hot_file_area_cold链表头，注意是链表头，
       同时要记录移动了多少个hot_file_area并保存到p_hot_file_stat->file_area_count_in_cold_list。

       然后，执行到下边for循环，因为是第一次执行 if(free_hot_file_area_count == p_hot_file_area->old_file_area_count_in_cold_list)直接成立，执行
       p_hot_file_area->old_file_area_count_in_cold_list = p_hot_file_area->file_area_count_in_cold_list赋值，然后break跳出for循环。接着
       if(free_hot_file_area_count == p_hot_file_area->old_file_area_count_in_cold_list)成立，执行赋值
       p_hot_file_area->old_file_area_count_in_cold_list = p_hot_file_area->file_area_count_in_cold_list，然后break跳出循环。这就相当于第1轮扫描时，在下边的for循环中，
       p_hot_file_global->cold_file_head上的hot_file_stat 和 p_hot_file_stat->hot_file_area_cold上的hot_file_area一个都没遍历。
       就应该这样，这些都是刚才上边的for循环刚移动到p_hot_file_global->cold_file_head和p_hot_file_stat->hot_file_area_cold上的，要等第2轮扫描再遍历。

      2:第2轮扫描，执行上边的for循环:重复第一步的操作，把global->hot_file_head_temp链表新扫描的hot_file_stat移动到global->hot_file_area_cold链表头,把这些hot_file_stat上被判定
      是冷hot_file_area移动到hot_file_stat的hot_file_area_cold链表头。然后执行下边的for循环，从p_hot_file_stat->hot_file_area_cold链表尾巴遍历
      p_hot_file_area->old_file_area_count_in_cold_list 个hot_file_area，这些个hot_file_area是第1次扫描时上边的for循环从p_hot_file_stat->hot_file_area_temp链表尾移动
      到p_hot_file_area->old_file_area_count_in_cold_list链表头的，现在第2轮扫描，这些hot_file_area在p_hot_file_area->old_file_area_count_in_cold_list链表尾了。为什么？
      。因为现在第2轮扫描，上边的for循环又从p_hot_file_stat->hot_file_area_temp链表移动到了p_hot_file_area->old_file_area_count_in_cold_list 链表头 一些hot_file_area。则第1轮
      扫描移动到p_hot_file_area->old_file_area_count_in_cold_list链表的hot_file_area就成了在p_hot_file_area->old_file_area_count_in_cold_list链表尾了。
      
      继续，在下边大for循环里，从p_hot_file_global->cold_file_head链表尾扫描p_hot_file_stat->file_stat_count_in_cold_list个hot_file_stat后，跳出下边的for循环。
      这 p_hot_file_stat->file_stat_count_in_cold_list个hot_file_stat是第1轮扫描时，上边的for循环从p_hot_file_global->hot_file_head_temp链表尾移动到
      p_hot_file_global->cold_file_head头，现在第2轮扫描，又从p_hot_file_global->hot_file_head_temp链表尾移动的一些hot_file_stat到
      p_hot_file_global->cold_file_head头。于是第1轮扫描时，移动到p_hot_file_global->cold_file_head头的hot_file_stat就成了在
      p_hot_file_global->cold_file_head链表尾。
      
      这个过程解释起来太繁琐了。简单总结说：第1轮扫描，上边的for循环，从global->hot_file_head_temp链表尾移动N1个hot_file_stat到global->hot_file_area_cold链表头。
      同时，遍历这N1个hot_file_stat的hot_file_area_temp链表尾N2个冷hot_file_area，并移动到hot_file_stat的hot_file_area_cold链表头。
      紧接着第2轮扫描，上边的fro循环重复第1轮的操作，再次向global->hot_file_area_cold链表头移动N1_1个hot_file_stat，向这些hot_file_stat的hot_file_area_cold链表头移动N2_1
      个hot_file_area。然后第2轮扫描，在下边for循环，从global->hot_file_area_cold链表尾巴遍历N1个第1轮扫描移动的hot_file_stat，再遍历这些hot_file_stat的hot_file_area_cold
      链表尾巴上N2个第1轮扫描移动的冷hot_file_area。这些hot_file_area第1轮扫描已经判定是冷hot_file_area，现在第2轮扫描，这些hot_file_area如果还是冷的，那就把这些
      hot_file_area移动到hot_file_stat的hot_file_area_free链表，然后就释放这些hot_file_area对应的page。说到底，绕来绕去的，就是保证一个hot_file_area必须经过
      两轮扫描判定都是冷的hot_file_area，才能释放这些hot_file_area对应的page。
      */
    scan_cold_file_stat_count = 0;
    list_for_each_entry_safe_reverse(p_hot_file_stat,p_hot_file_stat_temp,&p_hot_file_global->cold_file_head,hot_file_list)
    {
	/*//该if成立，说明上一轮扫描移动到global cold_file_head链表头的p_hot_file_stat->file_stat_count_in_cold_list个hot_file_stat已经遍历完了，不能继续向前
	//扫描hot_file_stat了，因为再向前的hot_file_stat是本轮扫描移动到global cold_file_head链表的。这个if判断要放到for循环开头,因为第一次执行到这里，
	//scan_cold_file_stat_count和p_hot_file_stat->file_stat_count_in_cold_list都是0-------------不行，这样就无法执行里边的for循环代码:
	//if(free_hot_file_area_count == p_hot_file_area->old_file_area_count_in_cold_list)里的
	//p_hot_file_area->old_file_area_count_in_cold_list = p_hot_file_area->file_area_count_in_cold_list;这个赋值了!!!!!!!!!!。给要移动到后边
        if(scan_cold_file_stat_count == p_hot_file_stat->file_stat_count_in_cold_list){
	    //把本轮扫描移动到global cold_file_head链表的file_stat个数保存到p_hot_file_stat->file_stat_count_in_cold_list
            p_hot_file_global->file_stat_count_in_cold_list = file_stat_count_in_cold_list;
	    break;
	}*/

        if(!file_stat_in_hot_file_head_temp(p_hot_file_stat))
	    panic("p_hot_file_stat:0x%llx status:%d not in free_temp_list\n",(u64)p_hot_file_stat,p_hot_file_stat->file_stat_status);

	scan_cold_file_area_count = 0;
        list_for_each_entry_safe_reverse(p_hot_file_area,p_hot_file_area_temp,&p_hot_file_stat->hot_file_area_cold,hot_file_area_list)
	{
	    //该if成立，说明上一轮扫描该p_hot_file_area被判定是冷hot_file_area而移动到p_hot_file_stat->hot_file_area_cold链表的p_hot_file_area->old_file_area_count_in_cold_list
	    //个hot_file_area已经都扫描完了，不能再向前扫描了，因为再向前的hot_file_area是本轮扫描移动到p_hot_file_stat->hot_file_area_cold链表的。这if判断要放到for循环最开头，因为
	    //第一次扫描时scan_cold_file_area_count是0，p_hot_file_area->old_file_area_count_in_cold_list也是0
            if(scan_cold_file_area_count == p_hot_file_stat->old_file_area_count_in_cold_list){
		p_hot_file_stat->old_file_area_count_in_cold_list = p_hot_file_stat->file_area_count_in_cold_list;
	        break;
	    }
	    //scan_cold_file_area_count++要放到if判断后边，因为第一次扫描执行到if判断，free_hot_file_area_count 和 p_hot_file_area->old_file_area_count_in_cold_list 都是0，得break跳出
	    scan_cold_file_area_count++;

	    if(!file_area_in_temp_list(p_hot_file_area))
	        panic("file_area_in_temp_list:0x%llx status:%d not in hot_file_area_temp\n",(u64)p_hot_file_area,p_hot_file_area->file_area_state);

	    //file_area 依然没有被访问，就释放 hot_file_stat 对应的page了
	    if(p_hot_file_area->area_access_count == p_hot_file_area->last_access_count){
                list_move(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_free_temp);
	    }
	    //file_area 又被访问了，则把hot_file_area添加到hot_file_area_temp临时链表
	    else{
		//需要加锁，hot_file_update_file_status()函数中会并发向该文件p_hot_file_stat->hot_file_area_temp添加新的hot_file_area结构
	        spin_lock_irq(&p_hot_file_stat->hot_file_stat_lock);
                list_move(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_temp);
		set_file_area_in_temp_list(p_hot_file_area);
		//有没有必要用area_access_count重置last_access_count，重置的话，后续该file_area不再被访问就又要把从hot_file_area_temp移动到hot_file_area_cold链表
		//p_hot_file_area->last_access_count = p_hot_file_area->area_access_count;??????????????????????????????
	        spin_unlock_irq(&p_hot_file_stat->hot_file_stat_lock);
	    }
        }
	//if成立，说明当前p_hot_file_stat，有冷hot_file_area添加到了p_hot_file_stat->hot_file_area_free链表
	if(0 != scan_cold_file_area_count){
            //把有冷hot_file_area的文件的hot_file_stat移动到hot_file_stat_free_list链表
	    //这里用不用把hot_file_stat->file_stat_status设置成无效，因为不在hot_file_global的任何链表了?????????????????????????????????????
            list_move(&p_hot_file_stat->hot_file_list,&hot_file_stat_free_list);
	}
	//该if成立，说明上一轮扫描移动到global cold_file_head链表头的p_hot_file_stat->file_stat_count_in_cold_list个hot_file_stat已经遍历完了，不能继续向前
	//扫描hot_file_stat了，因为再向前的hot_file_stat是本轮扫描移动到global cold_file_head链表的hot_file_stat
        if(scan_cold_file_stat_count == p_hot_file_global->file_stat_count_in_cold_list){
	    //把本轮扫描移动到global cold_file_head链表的file_stat个数保存到p_hot_file_stat->file_stat_count_in_cold_list
            p_hot_file_global->file_stat_count_in_cold_list = file_stat_count_in_cold_list;
	    break;
	}
	//scan_cold_file_stat_count++要放到if判断后边，因为第1轮扫描时，没有要扫描的hot_file_stat，scan_cold_file_stat_count和p_hot_file_stat->file_stat_count_in_cold_list都是0
	//上边直接break条春大的ffor循环
        scan_cold_file_stat_count ++;
    }
    //遍历hot_file_stat_free_list上的hot_file_stat，这些hot_file_stat有两轮扫描都判定是冷的hot_file_area，于是释放这些
    list_for_each_entry_safe_reverse(p_hot_file_stat,p_hot_file_stat_temp,&hot_file_stat_free_list,hot_file_list)
    {
        //对hot_file_area_free_temp上的hot_file_stat上的hot_file_area对应的page进行隔离，隔离成功的移动到p_hot_file_global->hot_file_node_pgdat->pgdat_page_list对应内存节点链表上
        hot_file_isolate_lru_pages(p_hot_file_global,p_hot_file_stat,&p_hot_file_stat->hot_file_area_free_temp);
	//这里真正释放内存page
	hot_file_shrink_pages(p_hot_file_global);
   
        /*注意，hot_file_stat->hot_file_area_free_temp 和 hot_file_stat->hot_file_area_free 各有用处。hot_file_area_free_temp保存每次扫描释放的page的hot_file_area。
	  释放后把这些hot_file_area移动到hot_file_area_free链表，hot_file_area_free保存的是每轮扫描释放page的hot_file_area，是所有的!!!!!!!!!!!!!!*/

	//把hot_file_area_free_temp链表上的hot_file_area结构再移动到hot_file_area_free链表，hot_file_area_free链表上的hot_file_area结构要长时间也没被访问就释放掉
        if(!list_empty(&p_hot_file_stat->hot_file_area_free_temp)){
            list_splice(&p_hot_file_stat->hot_file_area_free_temp,&p_hot_file_stat->hot_file_area_free);
        }
    }

    //把本轮扫描并释放page的hot_file_stat再移动后p_hot_file_global->hot_file_head链表头。注意是链表头，因为上边扫描时是从p_hot_file_global->hot_file_head链表尾
    //开始扫描的。这样做是保证下轮扫描不再扫描到这些hot_file_stat，而是扫描其他p_hot_file_global->hot_file_head链表尾的hot_file_stat
    if(!list_empty(&hot_file_stat_free_list)){
        list_for_each_entry(p_hot_file_stat,&hot_file_stat_free_list,hot_file_list)
            set_file_stat_in_head_temp_list(p_hot_file_stat);//设置hot_file_stat状态为head_temp_list

        spin_lock_irq(&p_hot_file_global->hot_file_lock);
	//把这些hot_file_stat移动回p_hot_file_global->hot_file_head_temp链表头
        list_splice(&hot_file_stat_free_list,&p_hot_file_global->hot_file_head_temp);
	spin_unlock_irq(&p_hot_file_global->hot_file_lock);
    }


    return 0;
}
#else
//遍历hot_file_global->hot_file_head_temp_large或hot_file_head_temp链表尾巴上边的文件file_stat，然后遍历这些file_stat的hot_file_stat->hot_file_area_temp链表尾巴上的file_area，
//被判定是冷的file_area则移动到hot_file_stat->hot_file_area_free_temp链表。把有冷file_area的file_stat移动到file_stat_free_list临时链表。返回值是遍历到的冷file_area个数
static unsigned int get_file_area_from_file_stat_list(struct hot_file_global *p_hot_file_global,unsigned int scan_file_area_max,unsigned int scan_file_stat_max,
	                                 //hot_file_head_temp来自 hot_file_global->hot_file_head_temp 或 hot_file_global->hot_file_head_temp_large 链表
          	                         struct list_head *hot_file_head_temp,struct list_head *file_stat_free_list){
    struct hot_file_stat * p_hot_file_stat,*p_hot_file_stat_temp;
    struct hot_file_area *p_hot_file_area,*p_hot_file_area_temp;

    unsigned int scan_file_area_count  = 0;
    unsigned int scan_file_stat_count  = 0;
    unsigned int scan_cold_file_area_count = 0;
    unsigned int cold_file_area_for_file_stat = 0;
    unsigned int file_stat_count_in_cold_list = 0;
    unsigned int serial_hot_file_area = 0;
    LIST_HEAD(file_stat_list_temp);
    //暂存从hot_file_global->hot_file_head_temp 或 hot_file_global->hot_file_head_temp_large 链表链表尾扫描到的file_stat
    LIST_HEAD(global_hot_file_head_temp_list);

     /*必须要先从hot_file_head_temp或hot_file_head_temp_large隔离多个file_stat，然后去遍历这些file_stat上的file_area，这样只用开关一次hot_file_global->hot_file_lock锁.
      * 否则每遍历一个file_stat，都开关一次hot_file_global->hot_file_lock锁，太损耗性能。*/
    spin_lock_irq(&p_hot_file_global->hot_file_lock);
    //先从global hot_file_head_temp链表尾隔离scan_file_stat_max个hot_file_stat到 global_hot_file_head_temp_list 临时链表
    list_for_each_entry_safe_reverse(p_hot_file_stat,p_hot_file_stat_temp,hot_file_head_temp,hot_file_list){
	//这里把hot_file_stat 移动到 global_hot_file_head_temp_list 临时链表，用不用清理的file_stat的 in_hot_file_head_temp 标记，需要的。因为hot_file_update_file_status()
	//函数中会并发因为file_stat的 in_hot_file_head_temp 标记，而移动到file_stat的hot_file_head链表，不能有这种并发操作
	if(!file_stat_in_hot_file_head_temp_list(p_hot_file_stat)){
	    panic("hot_file_stat:0x%llx not int hot_file_head_temp status:0x%x\n",(u64)p_hot_file_stat,p_hot_file_stat->file_stat_status);
	}

	/*hot_file_head_temp来自 hot_file_global->hot_file_head_temp 或 hot_file_global->hot_file_head_temp_large 链表，当是hot_file_global->hot_file_head_temp_large
	 * 时，file_stat_in_large_file(p_hot_file_stat)才会成立*/

        //当file_stat上有些file_area长时间没有被访问则会释放掉file_are结构。此时原本在hot_file_global->hot_file_head_temp_large 链表的大文件file_stat则会因
	//file_area数量减少而需要降级移动到hot_file_global->hot_file_head_temp链表.这个判断起始可以放到hot_file_update_file_status()函数，算了降低损耗
	if(!is_file_stat_large_file(&hot_file_global_info,p_hot_file_stat) && file_stat_in_large_file(p_hot_file_stat)){
	    if(open_shrink_printk)
	        printk("1:%s %s %d p_hot_file_global:0x%llx p_hot_file_stat:0x%llx status:0x%x not is_file_stat_large_file\n",__func__,current->comm,current->pid,(u64)p_hot_file_global,(u64)p_hot_file_stat,p_hot_file_stat->file_stat_status);

            clear_file_stat_in_large_file(p_hot_file_stat);
	    //不用现在把file_stat移动到global hot_file_head_temp链表。等该file_stat的file_area经过内存回收后，该file_stat会因为clear_file_stat_in_large_file而移动到hot_file_head_temp链表
	    //想了想，还是现在就移动到file_stat->hot_file_head_temp链表尾，否则内存回收再移动更麻烦。要移动到链表尾，这样紧接着就会从hot_file_head_temp链表链表尾扫描到该file_stat
	    list_move_tail(&p_hot_file_stat->hot_file_list,&p_hot_file_global->hot_file_head_temp);
	    continue;
	}

	//需要设置这些hot_file_stat不再处于hot_file_head_temp链表，否则之后hot_file_update_file_status()会因该file_stat的热file_area很多而移动到global hot_file_head_temp链表
	clear_file_stat_in_hot_file_head_temp_list(p_hot_file_stat);
        //扫描到的file_stat先移动到global_hot_file_head_temp_list临时链表，下边就开始遍历这些file_stat上的file_area
        list_move(&p_hot_file_stat->hot_file_list,&global_hot_file_head_temp_list);
	if(scan_file_stat_count ++ > scan_file_stat_max)
	    break;
    }
    spin_unlock_irq(&p_hot_file_global->hot_file_lock);

    //在遍历hot_file_global->hot_file_head_temp链表期间，可能创建了新文件并创建了hot_file_stat并添加到hot_file_global->hot_file_head_temp链表，
    //下边遍历hot_file_global->hot_file_head链表成员期间，是否用hot_file_global_info.hot_file_lock加锁？不用，因为遍历链表期间
    //向链表添加成员没事，只要不删除成员！想想我写的内存屏障那片文章讲解list_del_rcu的代码
    //list_for_each_entry_safe_reverse(p_hot_file_stat,&p_hot_file_global->hot_file_head_temp,hot_file_list)//从链表尾开始遍历，链表尾的成员更老，链表头的成员是最新添加的
    list_for_each_entry_safe(p_hot_file_stat,p_hot_file_stat_temp,&global_hot_file_head_temp_list,hot_file_list)//本质就是遍历p_hot_file_global->hot_file_head_temp链表尾的hot_file_stat
    {
        if(!file_stat_in_hot_file_head_temp_list(p_hot_file_stat))
	    panic("p_hot_file_stat:0x%llx status:%d not in free_temp_list\n",(u64)p_hot_file_stat,p_hot_file_stat->file_stat_status);

	cold_file_area_for_file_stat = 0;
	serial_hot_file_area = 0;
	//注意，这里扫描的global hot_file_head_temp上的hot_file_stat肯定有冷hot_file_area，因为hot_file_stat只要50%的hot_file_area是热的，hot_file_stat就要移动到
	//global hot_file_head 链表。
        list_for_each_entry_safe_reverse(p_hot_file_area,p_hot_file_area_temp,&p_hot_file_stat->hot_file_area_temp,hot_file_area_list)//从链表尾开始遍历，链表尾的成员更老，链表头的成员是最新添加的
	{
	    if(!file_area_in_temp_list(p_hot_file_area))
		panic("file_area_in_temp_list:0x%llx status:%d not in hot_file_area_temp\n",(u64)p_hot_file_area,p_hot_file_area->file_area_state);

	    scan_file_area_count ++;
	    //本周期内，该p_hot_file_area 依然没有被访问，移动到hot_file_area_cold链表头
	    //if(p_hot_file_area->area_access_count == p_hot_file_area->last_access_count){
	    
            //hot_file_area经过GOLD_FILE_AREA_LEVAL个周期还没有被访问，则被判定是冷file_area，然后就释放该file_area的page
	    if(p_hot_file_global->global_age - p_hot_file_area->file_area_age > GOLD_FILE_AREA_LEVAL){
                //每遍历到一个就加一次锁，浪费性能，可以先移动到一个临时链表上，循环结束后加一次锁，然后把这些file_area或file_stat移动到目标链表??????????????
	        spin_lock_irq(&p_hot_file_stat->hot_file_stat_lock);
		//为什么hot_file_stat_lock加锁后要再判断一次file_area是不是被访问了。因为可能有这种情况:上边的if成立，此时file_area还没被访问。但是此时有进程
		//先执行hot_file_update_file_status()获取hot_file_stat_lock锁，然后访问当前file_area，file_area不再冷了。当前进程此时获取hot_file_stat_lock锁失败。
		//等获取hot_file_stat_lock锁成功后，file_area的file_area_age就和global_age相等了。一次，变量加减后的判断，在spin_lock前后各判断一次有必要的!!!!!!!!!!!!!!!!!!!!!!!!
                if(p_hot_file_global->global_age - p_hot_file_area->file_area_age > GOLD_FILE_AREA_LEVAL){
                   continue;
		   spin_unlock_irq(&p_hot_file_stat->hot_file_stat_lock);    
		}
	        if(open_shrink_printk)
	            printk("2:%s %s %d p_hot_file_global:0x%llx p_hot_file_stat:0x%llx status:0x%x p_hot_file_area:0x%llx status:0x%x is cold file_area\n",__func__,current->comm,current->pid,(u64)p_hot_file_global,(u64)p_hot_file_stat,p_hot_file_stat->file_stat_status,(u64)p_hot_file_area,p_hot_file_area->file_area_state);

                serial_hot_file_area = 0;
		clear_file_area_in_temp_list(p_hot_file_area);
		//设置file_area处于file_stat的free_temp_list链表。这里设定，不管file_area处于hot_file_stat->hot_file_area_free_temp还是hot_file_stat->hot_file_area_free
		//链表，都是file_area_in_free_list状态，没有必要再区分二者。主要设置file_area的状态需要遍历每个file_area并hot_file_stat_lock加锁，
		//再多设置一次set_file_area_in_free_temp_list状态浪费性能。这点需注意!!!!!!!!!!!!!!!!!!!!!!!
		set_file_area_in_free_list(p_hot_file_area);
		//需要加锁，此时可能有进程执行hot_file_update_file_status()并发向该p_hot_file_area前或者后插入新的hot_file_area，这里是把该p_hot_file_area从hot_file_area_temp链表剔除，存在同时修改该p_hot_file_area在hot_file_area_temp链表前的hot_file_area结构的next指针和在链表后的hot_file_area结构的prev指针，并发修改同一个变量就需要加锁。
                //list_move(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_cold);
                list_move(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_free_temp);
	        spin_unlock_irq(&p_hot_file_stat->hot_file_stat_lock);
		//file_area_count_in_cold_list ++;
                
		//把有冷hot_file_area的hot_file_stat移动到file_stat_free_list临时链表.此时的file_sata已经不在hot_file_head_temp链表，上边已经清理掉
		if(cold_file_area_for_file_stat == 0){
		    //是否会存在并发设置p_hot_file_stat->file_stat_status的情况??????????????? 这里没有加锁，需要考虑这点???????????????
		    //set_file_stat_in_head_temp_list(p_hot_file_stat);
		    //这里不用加锁，此时p_hot_file_stat是在 global_hot_file_head_temp_list临时链表，并且把p_hot_file_stat移动到
		    //global cold_file_head链表，只在walk_throuth_all_hot_file_area()函数单线程操作，不存在并发
		    //list_move(&p_hot_file_stat->hot_file_list,&p_hot_file_global->cold_file_head);

		    list_move(&p_hot_file_stat->hot_file_list,file_stat_free_list);
		    //本轮扫描移动到global cold_file_head链表头的file_stat个数
		    file_stat_count_in_cold_list ++;
		}

		cold_file_area_for_file_stat ++;
	    }
	    //else if(p_hot_file_global->global_age == p_hot_file_area->file_area_age)
	    else //否则就停止遍历hot_file_stat->hot_file_area_temp链表上的file_area，因为该链表上的file_area从左向右，访问频率由大向小递增，这个需要实际测试?????????????????????????
	    {
		//如果hot_file_stat->hot_file_area_temp链表尾连续扫到3个file_area都是热的，才停止扫描该hot_file_stat上的file_area。因为此时hot_file_stat->hot_file_area_temp链表尾
		//上的file_area可能正在被访问，hot_file_area->file_area_age=hot_file_global->global_age，但是file_area还没被移动到hot_file_stat->hot_file_area_temp链表头。
		//这个判断是为了过滤掉这种瞬时的热file_area干扰
		if(serial_hot_file_area ++ > 2)
   		    break;
	    }
	}
	/*
	//hot_file_area_free链表上长时间没访问的hot_file_area释放掉
	if(!list_empty(&p_hot_file_stat->hot_file_area_free)){
		//hot_file_area_free链表上长时间没访问的hot_file_area释放掉
		list_for_each_entry_safe_reverse(p_hot_file_area,p_hot_file_area_temp,&p_hot_file_stat->hot_file_area_free,hot_file_area_list){
		    if(p_hot_file_global->global_age - p_hot_file_area->file_area_age < GOLD_FILE_AREA_LEVAL -1){//hot_file_area又被访问了，添加回hot_file_area_temp链表
			spin_lock_irq(&p_hot_file_stat->hot_file_stat_lock);
			list_move(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_temp);
			set_file_area_in_temp_list(p_hot_file_area);
			spin_unlock_irq(&p_hot_file_stat->hot_file_stat_lock);

			//p_hot_file_area->cold_time = 0;//冷却计数清0
		    }else{
			//hot_file_area冷的次数达到阀值则释放掉它
			//if(p_hot_file_area->cold_time > HOT_FILE_AREA_FREE_LEVEL)
			if(p_hot_file_global->global_age - p_hot_file_area->file_area_age > (GOLD_FILE_AREA_LEVAL + 6))
			    hot_file_area_detele(p_hot_file_global,p_hot_file_stat,p_hot_file_area);
		    }
		}
	    }
        */
	//累计遍历到的冷file_area个数
        scan_cold_file_area_count += cold_file_area_for_file_stat;

        //防止在for循环耗时太长，限制遍历的文件hot_file_stat数。这里两个问题 问题1:单个hot_file_stat上的hot_file_area太多了，只扫描一个hot_file_stat这里就
	//break跳出循环了。这样下边就把global_hot_file_head_temp_list残留的hot_file_stat移动到global hot_file_head_temp链表头了。下轮扫描从
	//global hot_file_head_temp尾就扫描不到该hot_file_stat了。合理的做法是，把这些压根没扫描的hot_file_stat再移动到global hot_file_head_temp尾。问题2：
	//还是 单个hot_file_stat上的hot_file_area太多了，没扫描完，下次再扫描该hot_file_stat时，直接从上次结束的hot_file_area位置处继续扫描，似乎更合理。
	//hot_file_stat断点hot_file_area继续扫描！但是实现起来似乎比较繁琐，算了
	if(scan_file_area_count > scan_file_area_max)
	    break;
    }
    //把global_hot_file_head_temp_list没遍历到的hot_file_stat移动到global hot_file_head_temp链表头。这样做就保证本轮从global hot_file_head_temp尾扫到的
    //hot_file_stat要么移动到了globa cold_file_head链表，要么移动到global hot_file_head_temp链表头。这样下轮从global hot_file_head_temp尾扫到的hot_file_stat之前没扫描过。
    //错了！上边扫描的global hot_file_head_temp链表尾的hot_file_stat肯定有冷hot_file_area。因为hot_file_stat只要50%的hot_file_area是热的，hot_file_stat就要移动到
    //global hot_file_head 链表。global hot_file_head_temp链表上的hot_file_stat肯定有hot_file_area。这里还残留在global_hot_file_head_temp_list上的hot_file_stat,
    //本轮就没有扫描到，因此要移动到global hot_file_head_temp链表尾，下轮扫描继续扫描这些hot_file_stat
    if(!list_empty(&global_hot_file_head_temp_list)){

        spin_lock_irq(&p_hot_file_global->hot_file_lock);
	//设置file_stat状态要加锁
	list_for_each_entry(p_hot_file_stat,&global_hot_file_head_temp_list,hot_file_list)
	    set_file_stat_in_hot_file_head_temp_list(p_hot_file_stat);//设置hot_file_stat状态为head_temp_list 
	//set_file_stat_in_head_temp_list(p_hot_file_stat);//不用再设置这些hot_file_stat的状态，这些hot_file_stat没有移动到global hot_file_area_cold链表，没改变状态
        //list_splice(&global_hot_file_head_temp_list,&p_hot_file_global->hot_file_head_temp);//移动到global hot_file_head_temp链表头
        //list_splice_tail(&global_hot_file_head_temp_list,&p_hot_file_global->hot_file_head_temp);//移动到 global hot_file_head_temp链表尾
	
	//把未遍历的file_stat再移动回hot_file_global->hot_file_head_temp或hot_file_global->hot_file_head_temp_large 链表尾巴
        list_splice_tail(&global_hot_file_head_temp_list,hot_file_head_temp);//移动到 global hot_file_head_temp 或 hot_file_head_temp_large 链表尾
        spin_unlock_irq(&p_hot_file_global->hot_file_lock);
    }

    if(open_shrink_printk)
        printk("3:%s %s %d p_hot_file_global:0x%llx scan_file_stat_count:%d scan_file_area_count:%d scan_cold_file_area_count:%d file_stat_count_in_cold_list:%d\n",__func__,current->comm,current->pid,(u64)p_hot_file_global,scan_file_stat_count,scan_file_area_count,scan_cold_file_area_count,file_stat_count_in_cold_list);

    return scan_cold_file_area_count;
}
/*该函数主要有3个作用
 * 1：释放file_stat_free_list链表上的file_stat的hot_file_area_free_temp链表上冷file_area的page。释放这些page后，把这些file_area移动到file_stat->hot_file_area_free链表头
 * 2：遍历file_stat_free_list链表上的file_stat的hot_file_area_hot链表尾上的热file_area，如果长时间没有被访问，说明变成冷file_area了，则移动到file_stat->hot_file_area_temp链表头
 * 3：遍历file_stat_free_list链表上的file_stat的hot_file_area_free链表尾上的file_area，如果还是长时间没有被访问，则释放掉这些file_area结构
 * 4: 遍历file_stat_free_list链表上的file_stat的hot_file_area_refault链表尾巴的file_area，如果长时间没有被访问，则移动到file_stat->hot_file_area_temp链表头
 * 5: 把file_stat_free_list链表上的file_stat再移动回hot_file_head_temp链表(即global hot_file_head_temp或hot_file_head_temp_large)头，这样下轮walk_throuth_all_hot_file_area()
 * 再扫描，从global hot_file_head_temp或hot_file_head_temp_large链表尾巴扫到的file_stat都是最近没有被扫描过的，避免重复扫描
 */
//file_stat_free_list链表上的file_stat来自本轮扫描从global hot_file_head_temp或hot_file_head_temp_large链表尾获取到的
//hot_file_head_temp是global hot_file_head_temp或hot_file_head_temp_large
unsigned long free_page_from_file_area(struct hot_file_global *p_hot_file_global,struct list_head * file_stat_free_list,struct list_head *hot_file_head_temp)
{
    unsigned int free_pages = 0;
    struct hot_file_stat * p_hot_file_stat/*,*p_hot_file_stat_temp*/;
    struct hot_file_area *p_hot_file_area,*p_hot_file_area_temp;
    unsigned int cold_file_area_count;
    unsigned int hot_file_area_count;
    unsigned int isolate_lru_pages = 0;


    /*同一个文件file_stat的file_area对应的page，更大可能是属于同一个内存节点node，所以要基于一个个文件的file_stat来扫描file_area，避免频繁开关内存节点锁pgdat->lru_lock锁*/  

    //遍历file_stat_free_list临时链表上的hot_file_stat，释放这些file_stat的hot_file_stat->hot_file_area_free_temp链表上的冷file_area的page
    list_for_each_entry(p_hot_file_stat,file_stat_free_list,hot_file_list)
    {
        //对hot_file_area_free_temp上的hot_file_stat上的hot_file_area对应的page进行隔离，隔离成功的移动到p_hot_file_global->hot_file_node_pgdat->pgdat_page_list对应内存节点链表上
        isolate_lru_pages += hot_file_isolate_lru_pages(p_hot_file_global,p_hot_file_stat,&p_hot_file_stat->hot_file_area_free_temp);
	//这里真正释放p_hot_file_global->hot_file_node_pgdat->pgdat_page_list链表上的内存page
	free_pages += hot_file_shrink_pages(p_hot_file_global);
	
	if(open_shrink_printk)
	    printk("1:%s %s %d p_hot_file_global:0x%llx p_hot_file_stat:0x%llx status:0x%x free_pages:%d\n",__func__,current->comm,current->pid,(u64)p_hot_file_global,(u64)p_hot_file_stat,p_hot_file_stat->file_stat_status,free_pages);
   
        /*注意，hot_file_stat->hot_file_area_free_temp 和 hot_file_stat->hot_file_area_free 各有用处。hot_file_area_free_temp保存每次扫描释放的page的hot_file_area。
	  释放后把这些hot_file_area移动到hot_file_area_free链表，hot_file_area_free保存的是每轮扫描释放page的所有hot_file_area，是所有的!!!!!!!!!!!!!!*/

	//p_hot_file_stat->hot_file_area_free_temp上的file_area的冷内存page释放过后,则把hot_file_area_free_temp链表上的hot_file_area结构再移动到hot_file_area_free链表头，
	//hot_file_area_free链表上的hot_file_area结构要长时间也没被访问就释放掉
        if(!list_empty(&p_hot_file_stat->hot_file_area_free_temp)){
	    //hot_file_update_file_status()函数中会并发把file_area从hot_file_stat->hot_file_area_free_temp链表移动到hot_file_stat->hot_file_area_free_temp链表.
	    //这里把hot_file_stat->hot_file_area_free_temp链表上的file_area移动到hot_file_stat->hot_file_area_free，需要加锁
	    spin_lock_irq(&p_hot_file_stat->hot_file_stat_lock);
            list_splice(&p_hot_file_stat->hot_file_area_free_temp,&p_hot_file_stat->hot_file_area_free);
	    spin_unlock_irq(&p_hot_file_stat->hot_file_stat_lock);
        }
    }
    //需要调度的话休眠一下
    cond_resched();
    
    /*这里有个隐藏很深但很重要的问题：在walk_throuth_all_hot_file_area()内存回收过程执行到该函数，把file_area移动到了hot_file_stat->hot_file_area_free_temp
     *或者hot_file_stat->hot_file_area_free链表后，此时hot_file_update_file_status()函数中又访问到这些file_area了，怎么办？这种情况完全有可能！
     *为了减少spin_lock_irq(&p_hot_file_stat->hot_file_stat_lock)锁的使用。目前设定只有file_area在file_stat的hot_file_area_hot、hot_file_area_temp、hot_file_area_temp_large
     *这3个有关的链表之间移动来移动去时，才会使用spin_lock_irq(&p_hot_file_stat->hot_file_stat_lock)。file_area从hot_file_stat->hot_file_area_free_temp移动到
     *hot_file_stat->hot_file_area_free链表上是没有解锁的！
     
     *如果file_area移动到了hot_file_stat->hot_file_area_free_temp或者hot_file_stat->hot_file_area_free链表后，此时hot_file_update_file_status()函数中又访问到这些file_area了，
     *如果直接hot_file_update_file_status()函数中把这些file_area直接移动到file_stat的hot_file_area_temp链表，那就又得spin_lock_irq(&p_hot_file_stat->hot_file_stat_lock)
     *加锁了，并且file_area从hot_file_stat->hot_file_area_free_temp移动到hot_file_stat->hot_file_area_free链表也得hot_file_stat_lock加锁。可以这样吗??????????
     *最后妥协了，就这样改吧。但是允许 hot_file_update_file_status()函数把file_area从hot_file_stat->hot_file_area_free_temp或hot_file_area_free链表移动到
     *file_stat的hot_file_area_temp链表后。hot_file_update_file_status()函数移动时需要spin_lock_irq(&p_hot_file_stat->hot_file_stat_lock)加锁，
     *该函数中把file_area从hot_file_stat->hot_file_area_free_temp移动到hot_file_stat->hot_file_area_free，也需要hot_file_stat_lock加锁；并且，从hot_file_stat->hot_file_area_free
     *释放长时间没有被访问的file_area时，也需要hot_file_stat_lock加锁!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
     */

    //遍历file_stat_free_list临时链表上的hot_file_stat，然后遍历着这些hot_file_stat->hot_file_area_hot链表尾巴上热file_area。这些file_area之前被判定是热file_area
    //而被移动到了hot_file_stat->hot_file_area_hot链表。之后，hot_file_stat->hot_file_area_hot链表头的file_area访问频繁，链表尾巴的file_area就会变冷。则把这些
    //hot_file_stat->hot_file_area_hot尾巴上长时间未被访问的file_area再降级移动回file_stat->hot_file_area_temp链表头
    list_for_each_entry(p_hot_file_stat,file_stat_free_list,hot_file_list){
        cold_file_area_count = 0;
        list_for_each_entry_safe_reverse(p_hot_file_area,p_hot_file_area_temp,&p_hot_file_stat->hot_file_area_hot,hot_file_area_list){
	    //hot_file_stat->hot_file_area_hot尾巴上长时间未被访问的file_area再降级移动回file_stat->hot_file_area_temp链表头
            if(p_hot_file_global->global_age - p_hot_file_area->file_area_age > GOLD_FILE_AREA_LEVAL + 3){
		cold_file_area_count = 0;
	        if(open_shrink_printk)
	            printk("2:%s %s %d p_hot_file_global:0x%llx p_hot_file_stat:0x%llx status:0x%x p_hot_file_area:0x%llx status:0x%x in file_stat->hot_file_area_hot\n",__func__,current->comm,current->pid,(u64)p_hot_file_global,(u64)p_hot_file_stat,p_hot_file_stat->file_stat_status,(u64)p_hot_file_area,p_hot_file_area->file_area_state);
                //每遍历到一个就加一次锁，浪费性能，可以先移动到一个临时链表上，循环结束后加一次锁，然后把这些file_area或file_stat移动到目标链表??????????????
	        spin_lock_irq(&p_hot_file_stat->hot_file_stat_lock);
		clear_file_area_in_hot_list(p_hot_file_area);
		set_file_area_in_temp_list(p_hot_file_area);
	        list_move(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_temp);
                spin_unlock_irq(&p_hot_file_stat->hot_file_stat_lock);	    
	    }else{//到这里，file_area被判定还是热file_area，还是继续存在hot_file_stat->hot_file_area_hot链表

	//如果hot_file_stat->hot_file_area_hot尾巴上连续出现2个file_area还是热file_area，则说明hot_file_stat->hot_file_area_hot链表尾巴上的冷file_area都遍历完了,遇到链表头的热
	//file_area了，则停止遍历。hot_file_stat->hot_file_area_hot链表头到链表尾，file_area是由热到冷顺序排布的。之所以要限制连续碰到两个热file_area再break，是因为hot_file_stat->
	//hot_file_area_hot尾巴上的冷file_area可能此时hot_file_update_file_status()中并发被频繁访问，变成热file_area，但还没来得及移动到hot_file_stat->hot_file_area_hot链表头
	        if(cold_file_area_count ++ > 1)
		    break;
	    }
	}
    }
     
    //需要调度的话休眠一下
    cond_resched();
   
    //遍历file_stat_free_list临时链表上的file_stat，然后看这些file_stat的hot_file_area_free链表上的哪些file_area长时间未被访问，抓到的话就释放掉file_area结构
    //如果hot_file_stat->hot_file_area_free链表上有很多file_area导致这里遍历时间很长怎么办？需要考虑一下??????????????????????????
    list_for_each_entry(p_hot_file_stat,file_stat_free_list,hot_file_list){
	hot_file_area_count = 0;
	list_for_each_entry_safe_reverse(p_hot_file_area,p_hot_file_area_temp,&p_hot_file_stat->hot_file_area_free,hot_file_area_list){
        #if 0
            //如果hot_file_stat->hot_file_area_free链表上的file_area最近又被访问了，hot_file_update_file_status()函数会把该file_area移动回
	    //global hot_file_head_temp或hot_file_head_temp_large链表，这里就不用再重复操作了

	    //hot_file_area又被访问了，添加回hot_file_area_temp链表
	    if(p_hot_file_global->global_age - p_hot_file_area->file_area_age < GOLD_FILE_AREA_LEVAL -1){
		
		//每遍历到一个就加一次锁，浪费性能，可以先移动到一个临时链表上，循环结束后加一次锁，然后把这些file_area或file_stat移动到目标链表?????????
		spin_lock_irq(&p_hot_file_stat->hot_file_stat_lock);
		list_move(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_temp);
		clear_file_area_in_free_list(p_hot_file_area);
		set_file_area_in_temp_list(p_hot_file_area);
		spin_unlock_irq(&p_hot_file_stat->hot_file_stat_lock);
                //如果hot_file_stat->hot_file_area_free链表尾连续出现3个file_area最近被访问过，则结束遍历该hot_file_stat->hot_file_area_free上的file_area
		if(hot_file_area_count ++ > 2)
		    break;
	    }else{
		hot_file_area_count = 0;
		//hot_file_area_free链表上长时间没访问的hot_file_area释放掉
		if(p_hot_file_global->global_age - p_hot_file_area->file_area_age > (GOLD_FILE_AREA_LEVAL + 3))
		    hot_file_area_detele(p_hot_file_global,p_hot_file_stat,p_hot_file_area);
	    }
        #else
	    //如果hot_file_stat->hot_file_area_free链表上的file_area长时间没有被访问则释放掉file_area结构
            if(p_hot_file_global->global_age - p_hot_file_area->file_area_age > GOLD_FILE_AREA_LEVAL + 5){

	        if(open_shrink_printk)
	            printk("3:%s %s %d p_hot_file_global:0x%llx p_hot_file_stat:0x%llx status:0x%x p_hot_file_area:0x%llx status:0x%x in hot_file_stat->hot_file_area_free\n",__func__,current->comm,current->pid,(u64)p_hot_file_global,(u64)p_hot_file_stat,p_hot_file_stat->file_stat_status,(u64)p_hot_file_area,p_hot_file_area->file_area_state);
		hot_file_area_count = 0;
	        //hot_file_update_file_status()函数中会并发把file_area从hot_file_stat->hot_file_area_free链表移动到hot_file_stat->hot_file_area_free_temp链表.
	        //这里把hot_file_stat->hot_file_area_free链表上的file_area剔除掉并释放掉，需要spin_lock_irq(&p_hot_file_stat->hot_file_stat_lock)加锁，这个函数里有加锁
	        hot_file_area_detele(p_hot_file_global,p_hot_file_stat,p_hot_file_area);
	    }else{
		//如果hot_file_stat->hot_file_area_free链表尾连续出现3个file_area未达到释放标准,说明可能最近被访问过，则结束遍历该hot_file_stat->hot_file_area_free上的file_area
		//这是防止遍历耗时太长，并且遍历到本轮扫描添加到hot_file_stat->hot_file_area_free上的file_area，浪费
	        if(hot_file_area_count ++ > 2)
		    break;
	    }
	#endif
	}
    }

    //遍历 file_stat_free_list临时链表上的file_stat，然后看这些file_stat的hot_file_area_refault链表上的file_area，如果长时间没有被访问，
    //则要移动到hot_file_stat->hot_file_area_temp链表
    list_for_each_entry(p_hot_file_stat,file_stat_free_list,hot_file_list){
	hot_file_area_count = 0;
        list_for_each_entry_safe_reverse(p_hot_file_area,p_hot_file_area_temp,&p_hot_file_stat->hot_file_area_refault,hot_file_area_list){
	    //hot_file_stat->hot_file_area_hot尾巴上长时间未被访问的file_area再降级移动回file_stat->hot_file_area_temp链表头
            if(p_hot_file_global->global_age - p_hot_file_area->file_area_age > GOLD_FILE_AREA_LEVAL + 3){
	        if(open_shrink_printk)
	            printk("4:%s %s %d p_hot_file_global:0x%llx p_hot_file_stat:0x%llx status:0x%x p_hot_file_area:0x%llx status:0x%x in file_stat->hot_file_area_refault\n",__func__,current->comm,current->pid,(u64)p_hot_file_global,(u64)p_hot_file_stat,p_hot_file_stat->file_stat_status,(u64)p_hot_file_area,p_hot_file_area->file_area_state);

		hot_file_area_count = 0;
                //每遍历到一个就加一次锁，浪费性能，可以先移动到一个临时链表上，循环结束后加一次锁，然后把这些file_area或file_stat移动到目标链表??????????????
	        spin_lock_irq(&p_hot_file_stat->hot_file_stat_lock);
		clear_file_area_in_refault_list(p_hot_file_area);
		set_file_area_in_temp_list(p_hot_file_area);
		/*if(file_stat_in_large_file(p_hot_file_stat))
                    list_move(&p_hot_file_stat->hot_file_list,&p_hot_file_global->hot_file_head_temp_large);
		else
                    list_move(&p_hot_file_stat->hot_file_list,&p_hot_file_global->hot_file_head_temp);*/
		list_move(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_temp);
                spin_unlock_irq(&p_hot_file_stat->hot_file_stat_lock);	    
	    }else{
	//如果hot_file_stat->hot_file_area_refault尾巴上连续出现2个file_area还是热file_area，则说明hot_file_stat->hot_file_area_hot链表尾巴上的冷file_area都遍历完了,遇到链表头的热
	//file_area了，则停止遍历。hot_file_stat->hot_file_area_refault链表头到链表尾，file_area是由热到冷顺序排布的。之所以要限制连续碰到两个热file_area再break，是因为hot_file_stat->
	//hot_file_area_refault尾巴上的冷file_area可能此时hot_file_update_file_status()中并发被频繁访问，变成热file_area，但还没来得及移动到hot_file_area_refault链表头
	        if(hot_file_area_count ++ >2)
		    break;
	    }
	}
    }
   
    /*-------这是遍历全局hot_file_global->hot_file_head上的file_stat，不遍历file_stat_free_list上的file_stat，不应该放在这里
    //遍历hot_file_global->hot_file_head链表上的热文件file_stat，如果哪些file_stat不再是热文件，再要把file_stat移动回global->hot_file_head_temp或hot_file_head_temp_large链表
    list_for_each_entry(p_hot_file_stat,p_hot_file_global->hot_file_head,hot_file_list){
	    //file_stat不再是热文件则移动回hot_file_global->hot_file_head_temp 或 hot_file_global->hot_file_head_temp_large链表
	    if(!is_file_stat_hot_file(p_hot_file_global,p_hot_file_stat)){
	        clear_file_area_in_hot_list(p_hot_file_stat);
	        set_file_stat_in_hot_file_head_temp_list(p_hot_file_stat);//设置hot_file_stat状态为in_head_temp_list
		if(file_stat_in_large_file(p_hot_file_stat))
                    list_move(&p_hot_file_stat->hot_file_list,p_hot_file_global->hot_file_head_temp);
		else
                    list_move(&p_hot_file_stat->hot_file_list,p_hot_file_global->hot_file_head_temp_large);
	    }
        }
    }*/

    //需要调度的话休眠一下
    cond_resched();

    //把file_stat_free_list临时链表上释放过内存page的file_stat再移动回global hot_file_head_temp或hot_file_head_temp_large链表头
    if(!list_empty(file_stat_free_list)){
        spin_lock_irq(&p_hot_file_global->hot_file_lock);
        list_for_each_entry(p_hot_file_stat,file_stat_free_list,hot_file_list){
            set_file_stat_in_hot_file_head_temp_list(p_hot_file_stat);//设置hot_file_stat状态为in_head_temp_list
        }
	//把这些遍历过的hot_file_stat移动回global hot_file_head_temp或hot_file_head_temp_large链表头,注意是链表头。这是因为，把这些遍历过的file_stat移动到 
	//global hot_file_head_temp或hot_file_head_temp_large链表头，下轮扫描才能从global hot_file_head_temp或hot_file_head_temp_large链表尾遍历没有遍历过的的file_stat
        list_splice(file_stat_free_list,hot_file_head_temp);//hot_file_head_temp来自 global hot_file_head_temp或hot_file_head_temp_large链表
	spin_unlock_irq(&p_hot_file_global->hot_file_lock);
    }

    if(open_shrink_printk)
    	printk("5:%s %s %d p_hot_file_global:0x%llx free_pages:%d isolate_lru_pages:%d hot_file_head_temp;0x%llx\n",__func__,current->comm,current->pid,(u64)p_hot_file_global,free_pages,isolate_lru_pages,(u64)hot_file_head_temp);
    return free_pages;
}
int walk_throuth_all_hot_file_area(struct hot_file_global *p_hot_file_global)
{
    struct hot_file_stat * p_hot_file_stat,*p_hot_file_stat_temp;
    //struct hot_file_area *p_hot_file_area,*p_hot_file_area_temp;
    //LIST_HEAD(hot_file_area_list);
    LIST_HEAD(file_stat_free_list_from_head_temp);
    LIST_HEAD(file_stat_free_list_from_head_temp_large);
    unsigned int scan_file_area_max,scan_file_stat_max;
    unsigned int scan_cold_file_area_count = 0;
    unsigned long nr_reclaimed = 0;

    //每个周期global_age加1
    hot_file_global_info.global_age ++;

    scan_file_stat_max = 10;
    scan_file_area_max = 1024;
    //遍历hot_file_global->hot_file_head_temp_large链表尾巴上边的大文件file_stat，然后遍历这些大文件file_stat的hot_file_stat->hot_file_area_temp链表尾巴上的file_area，被判定是冷的
    //file_area则移动到hot_file_stat->hot_file_area_free_temp链表。把有冷file_area的file_stat移动到file_stat_free_list_from_head_temp_large临时链表。返回值是遍历到的冷file_area个数
    scan_cold_file_area_count += get_file_area_from_file_stat_list(p_hot_file_global,scan_file_area_max,scan_file_stat_max, 
	                               &p_hot_file_global->hot_file_head_temp_large,&file_stat_free_list_from_head_temp_large);
    //需要调度的话休眠一下
    cond_resched();
    scan_file_stat_max = 64;
    scan_file_area_max = 1024;
    //遍历hot_file_global->hot_file_head_temp链表尾巴上边的小文件file_stat，然后遍历这些小文件file_stat的hot_file_stat->hot_file_area_temp链表尾巴上的file_area，被判定是冷的
    //file_area则移动到hot_file_stat->hot_file_area_free_temp链表。把有冷file_area的file_stat移动到file_stat_free_list_from_head_temp临时链表。返回值是遍历到的冷file_area个数
    scan_cold_file_area_count += get_file_area_from_file_stat_list(p_hot_file_global,scan_file_area_max,scan_file_stat_max, 
	                               &p_hot_file_global->hot_file_head_temp,&file_stat_free_list_from_head_temp);
#if 0 
    scan_cold_file_stat_count = 0;
    list_for_each_entry_safe_reverse(p_hot_file_stat,p_hot_file_stat_temp,&p_hot_file_global->cold_file_head,hot_file_list)
    {
	/*//该if成立，说明上一轮扫描移动到global cold_file_head链表头的p_hot_file_stat->file_stat_count_in_cold_list个hot_file_stat已经遍历完了，不能继续向前
	//扫描hot_file_stat了，因为再向前的hot_file_stat是本轮扫描移动到global cold_file_head链表的。这个if判断要放到for循环开头,因为第一次执行到这里，
	//scan_cold_file_stat_count和p_hot_file_stat->file_stat_count_in_cold_list都是0-------------不行，这样就无法执行里边的for循环代码:
	//if(free_hot_file_area_count == p_hot_file_area->old_file_area_count_in_cold_list)里的
	//p_hot_file_area->old_file_area_count_in_cold_list = p_hot_file_area->file_area_count_in_cold_list;这个赋值了!!!!!!!!!!。给要移动到后边
        if(scan_cold_file_stat_count == p_hot_file_stat->file_stat_count_in_cold_list){
	    //把本轮扫描移动到global cold_file_head链表的file_stat个数保存到p_hot_file_stat->file_stat_count_in_cold_list
            p_hot_file_global->file_stat_count_in_cold_list = file_stat_count_in_cold_list;
	    break;
	}*/

        if(!file_stat_in_hot_file_head_temp(p_hot_file_stat))
	    panic("p_hot_file_stat:0x%llx status:%d not in free_temp_list\n",(u64)p_hot_file_stat,p_hot_file_stat->file_stat_status);

	scan_cold_file_area_count = 0;
        list_for_each_entry_safe_reverse(p_hot_file_area,p_hot_file_area_temp,&p_hot_file_stat->hot_file_area_cold,hot_file_area_list)
	{
	    //该if成立，说明上一轮扫描该p_hot_file_area被判定是冷hot_file_area而移动到p_hot_file_stat->hot_file_area_cold链表的p_hot_file_area->old_file_area_count_in_cold_list
	    //个hot_file_area已经都扫描完了，不能再向前扫描了，因为再向前的hot_file_area是本轮扫描移动到p_hot_file_stat->hot_file_area_cold链表的。这if判断要放到for循环最开头，因为
	    //第一次扫描时scan_cold_file_area_count是0，p_hot_file_area->old_file_area_count_in_cold_list也是0
            if(scan_cold_file_area_count == p_hot_file_stat->old_file_area_count_in_cold_list){
		p_hot_file_stat->old_file_area_count_in_cold_list = p_hot_file_stat->file_area_count_in_cold_list;
	        break;
	    }
	    //scan_cold_file_area_count++要放到if判断后边，因为第一次扫描执行到if判断，free_hot_file_area_count 和 p_hot_file_area->old_file_area_count_in_cold_list 都是0，得break跳出
	    scan_cold_file_area_count++;

	    if(!file_area_in_temp_list(p_hot_file_area))
	        panic("file_area_in_temp_list:0x%llx status:%d not in hot_file_area_temp\n",(u64)p_hot_file_area,p_hot_file_area->file_area_state);

	    //file_area 依然没有被访问，就释放 hot_file_stat 对应的page了
	    if(p_hot_file_area->area_access_count == p_hot_file_area->last_access_count){
                list_move(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_free_temp);
	    }
	    //file_area 又被访问了，则把hot_file_area添加到hot_file_area_temp临时链表
	    else{
		//需要加锁，hot_file_update_file_status()函数中会并发向该文件p_hot_file_stat->hot_file_area_temp添加新的hot_file_area结构
	        spin_lock_irq(&p_hot_file_stat->hot_file_stat_lock);
                list_move(&p_hot_file_area->hot_file_area_list,&p_hot_file_stat->hot_file_area_temp);
		set_file_area_in_temp_list(p_hot_file_area);
		//有没有必要用area_access_count重置last_access_count，重置的话，后续该file_area不再被访问就又要把从hot_file_area_temp移动到hot_file_area_cold链表
		//p_hot_file_area->last_access_count = p_hot_file_area->area_access_count;??????????????????????????????
	        spin_unlock_irq(&p_hot_file_stat->hot_file_stat_lock);
	    }
        }
	//if成立，说明当前p_hot_file_stat，有冷hot_file_area添加到了p_hot_file_stat->hot_file_area_free链表
	if(0 != scan_cold_file_area_count){
            //把有冷hot_file_area的文件的hot_file_stat移动到hot_file_stat_free_list链表
	    //这里用不用把hot_file_stat->file_stat_status设置成无效，因为不在hot_file_global的任何链表了?????????????????????????????????????
            list_move(&p_hot_file_stat->hot_file_list,&hot_file_stat_free_list);
	}
	//该if成立，说明上一轮扫描移动到global cold_file_head链表头的p_hot_file_stat->file_stat_count_in_cold_list个hot_file_stat已经遍历完了，不能继续向前
	//扫描hot_file_stat了，因为再向前的hot_file_stat是本轮扫描移动到global cold_file_head链表的hot_file_stat
        if(scan_cold_file_stat_count == p_hot_file_global->file_stat_count_in_cold_list){
	    //把本轮扫描移动到global cold_file_head链表的file_stat个数保存到p_hot_file_stat->file_stat_count_in_cold_list
            p_hot_file_global->file_stat_count_in_cold_list = file_stat_count_in_cold_list;
	    break;
	}
	//scan_cold_file_stat_count++要放到if判断后边，因为第1轮扫描时，没有要扫描的hot_file_stat，scan_cold_file_stat_count和p_hot_file_stat->file_stat_count_in_cold_list都是0
	//上边直接break条春大的ffor循环
        scan_cold_file_stat_count ++;
    }
#endif  
    /*该函数主要有5个作用
 * 1：释放file_stat_free_list_from_head_temp_large链表上的file_stat的hot_file_area_free_temp链表上冷file_area的page。释放这些page后，把这些file_area移动到file_stat->hot_file_area_free链表头
 * 2：遍历file_stat_free_list_from_head_temp_large的hot_file_area_hot链表尾上的热file_area，如果长时间没有被访问，说明变成冷file_area了，则移动到file_stat->hot_file_area_temp链表头
 * 3：遍历file_stat_free_list_from_head_temp_large链表上的file_stat的hot_file_area_free链表尾上的file_area，如果还是长时间没有被访问，则释放掉这些file_area结构
 * 4: 遍历file_stat_free_list_from_head_temp_large链表上的file_stat的hot_file_area_refault链表尾巴的file_area，如果长时间没有被访问，则移动到file_stat->hot_file_area_temp链表头
 * 5: 把file_stat_free_list_from_head_temp_large链表上的file_stat再移动回hot_file_head_temp链表(即global hot_file_head_temp或hot_file_head_temp_large)头，这样下轮walk_throuth_all_hot_file_area()再扫描，从global hot_file_head_temp或hot_file_head_temp_large链表尾巴扫到的file_stat都是最近没有被扫描过的，避免重复扫描
 */
    nr_reclaimed =  free_page_from_file_area(p_hot_file_global,&file_stat_free_list_from_head_temp_large,&p_hot_file_global->hot_file_head_temp_large); 
    nr_reclaimed += free_page_from_file_area(p_hot_file_global,&file_stat_free_list_from_head_temp,&p_hot_file_global->hot_file_head_temp); 

 //遍历hot_file_global->hot_file_head链表上的热文件file_stat，如果哪些file_stat不再是热文件，再要把file_stat移动回global->hot_file_head_temp或hot_file_head_temp_large链表
    list_for_each_entry_safe_reverse(p_hot_file_stat,p_hot_file_stat_temp,&p_hot_file_global->hot_file_head,hot_file_list){
	//file_stat不再是热文件则移动回hot_file_global->hot_file_head_temp 或 hot_file_global->hot_file_head_temp_large链表
	if(!is_file_stat_hot_file(p_hot_file_global,p_hot_file_stat)){

            spin_lock_irq(&p_hot_file_global->hot_file_lock);
	    clear_file_stat_in_hot_file_head_list(p_hot_file_stat);
	    set_file_stat_in_hot_file_head_temp_list(p_hot_file_stat);//设置hot_file_stat状态为in_head_temp_list
	    if(file_stat_in_large_file(p_hot_file_stat))
		list_move(&p_hot_file_stat->hot_file_list,&p_hot_file_global->hot_file_head_temp_large);
	    else
		list_move(&p_hot_file_stat->hot_file_list,&p_hot_file_global->hot_file_head_temp);
            spin_lock_irq(&p_hot_file_global->hot_file_lock);
	}
    }

    return 0;
}
#endif
static int hot_file_thread(void *p){
    struct hot_file_global *p_hot_file_global = (struct hot_file_global *)p;
    int sleep_count = 0;

    while(1){
	sleep_count = 0;
        while(!hot_file_shrink_enable || sleep_count ++ < 30)
            msleep(1000);

	walk_throuth_all_hot_file_area(p_hot_file_global);
	if (kthread_should_stop())
	    break;
    }
    return 0;
}
int hot_file_init(void)
{
    int node_count,i;
    //hot_file_global_info.hot_file_stat_cachep = KMEM_CACHE(hot_file_stat,0);
    hot_file_global_info.hot_file_stat_cachep = kmem_cache_create("hot_file_stat",sizeof(struct hot_file_stat),0,0,NULL);
    hot_file_global_info.hot_file_area_cachep = kmem_cache_create("hot_file_area",sizeof(struct hot_file_area),0,0,NULL);
    hot_file_global_info.hot_file_area_tree_node_cachep = kmem_cache_create("hot_file_area_tree_node",sizeof(struct hot_file_area_tree_node),0,0,NULL);

    INIT_LIST_HEAD(&hot_file_global_info.hot_file_head);
    INIT_LIST_HEAD(&hot_file_global_info.hot_file_head_temp);
    INIT_LIST_HEAD(&hot_file_global_info.hot_file_head_temp_large);

    INIT_LIST_HEAD(&hot_file_global_info.cold_file_head);
    spin_lock_init(&hot_file_global_info.hot_file_lock);

    //1G的page cache对应多少个file_area
    hot_file_global_info.file_area_count_for_large_file = (1024*1024*1024)/(4096 *PAGE_COUNT_IN_AREA);
    node_count = 0;
    for_each_node_state(i, N_MEMORY)
	node_count ++;

    hot_file_global_info.node_count = node_count;
    //按照内存节点数node_count分配node_count个hot_file_node_pgdat结构体，保存到数组
    hot_file_global_info.p_hot_file_node_pgdat = (struct hot_file_node_pgdat *)kmalloc(node_count*sizeof(struct hot_file_node_pgdat),GFP_KERNEL);
    for(i = 0;i < node_count;i++){
	//保存每个内存节点的pgdat指针
        hot_file_global_info.p_hot_file_node_pgdat[i].pgdat = NODE_DATA(i);
	//初始化每个内存节点的pgdat_page_list链表，将来内存回收时，把每个内存节点要回收的内存保存到pgdat_page_list链表上
        INIT_LIST_HEAD(&hot_file_global_info.p_hot_file_node_pgdat[i].pgdat_page_list);
    }

    hot_file_global_info.hot_file_thead = kthread_run(hot_file_thread,&hot_file_global_info, "hot_file_thread");
    if (IS_ERR(hot_file_global_info.hot_file_thead)) {
	pr_err("Failed to start  hot_file_thead\n");
	return -1;
    }
    return 0;
}
