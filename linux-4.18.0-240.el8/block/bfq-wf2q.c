/*
 * Hierarchical Budget Worst-case Fair Weighted Fair Queueing
 * (B-WF2Q+): hierarchical scheduling algorithm by which the BFQ I/O
 * scheduler schedules generic entities. The latter can represent
 * either single bfq queues (associated with processes) or groups of
 * bfq queues (associated with cgroups).
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation; either version 2 of the
 *  License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */
#include "bfq-iosched.h"

extern int open_bfqq_printk;
extern int open_bfqq_printk1;
int vim_pid = -2;
/**
 * bfq_gt - compare two timestamps.
 * @a: first ts.
 * @b: second ts.
 *
 * Return @a > @b, dealing with wrapping correctly.
 */
static int bfq_gt(u64 a, u64 b)
{
	return (s64)(a - b) > 0;
}

static struct bfq_entity *bfq_root_active_entity(struct rb_root *tree)
{
	struct rb_node *node = tree->rb_node;

	return rb_entry(node, struct bfq_entity, rb_node);
}

static unsigned int bfq_class_idx(struct bfq_entity *entity)
{
	struct bfq_queue *bfqq = bfq_entity_to_bfqq(entity);

	return bfqq ? bfqq->ioprio_class - 1 :
		BFQ_DEFAULT_GRP_CLASS - 1;
}

unsigned int bfq_tot_busy_queues(struct bfq_data *bfqd)
{
	return bfqd->busy_queues[0] + bfqd->busy_queues[1] +
		bfqd->busy_queues[2];
}

static struct bfq_entity *bfq_lookup_next_entity(struct bfq_sched_data *sd,
						 bool expiration);

static bool bfq_update_parent_budget(struct bfq_entity *next_in_service);

/**
 * bfq_update_next_in_service - update sd->next_in_service
 * @sd: sched_data for which to perform the update.
 * @new_entity: if not NULL, pointer to the entity whose activation,
 *		requeueing or repositionig triggered the invocation of
 *		this function.
 * @expiration: id true, this function is being invoked after the
 *             expiration of the in-service entity
 *
 * This function is called to update sd->next_in_service, which, in
 * its turn, may change as a consequence of the insertion or
 * extraction of an entity into/from one of the active trees of
 * sd. These insertions/extractions occur as a consequence of
 * activations/deactivations of entities, with some activations being
 * 'true' activations, and other activations being requeueings (i.e.,
 * implementing the second, requeueing phase of the mechanism used to
 * reposition an entity in its active tree; see comments on
 * __bfq_activate_entity and __bfq_requeue_entity for details). In
 * both the last two activation sub-cases, new_entity points to the
 * just activated or requeued entity.
 *
 * Returns true if sd->next_in_service changes in such a way that
 * entity->parent may become the next_in_service for its parent
 * entity.
 */
static bool bfq_update_next_in_service(struct bfq_sched_data *sd,
				       struct bfq_entity *new_entity,
				       bool expiration)
{
	struct bfq_entity *next_in_service = sd->next_in_service;
	bool parent_sched_may_change = false;
	bool change_without_lookup = false;
	struct bfq_queue *bfqq = NULL;

	/*
	 * If this update is triggered by the activation, requeueing
	 * or repositiong of an entity that does not coincide with
	 * sd->next_in_service, then a full lookup in the active tree
	 * can be avoided. In fact, it is enough to check whether the
	 * just-modified entity has the same priority as
	 * sd->next_in_service, is eligible and has a lower virtual
	 * finish time than sd->next_in_service. If this compound
	 * condition holds, then the new entity becomes the new
	 * next_in_service. Otherwise no change is needed.
	 */
        if(open_bfqq_printk)
	       printk("1:%s %d %s %d new_entity:%llx sd->next_in_service:%llx\n",__func__,__LINE__,current->comm,current->pid,(u64)new_entity,(u64)(sd->next_in_service));
	if (new_entity && new_entity != sd->next_in_service) {
		/*
		 * Flag used to decide whether to replace
		 * sd->next_in_service with new_entity. Tentatively
		 * set to true, and left as true if
		 * sd->next_in_service is NULL.
		 */
		change_without_lookup = true;
		if(open_bfqq_printk)
	            printk("2:%s %d %s %d new_entity:%llx if (new_entity && new_entity != sd->next_in_service)\n",__func__,__LINE__,current->comm,current->pid,(u64)new_entity);

		/*
		 * If there is already a next_in_service candidate
		 * entity, then compare timestamps to decide whether
		 * to replace sd->service_tree with new_entity.
		 */
		if (next_in_service) {
			unsigned int new_entity_class_idx =
				bfq_class_idx(new_entity);
			struct bfq_service_tree *st =
				sd->service_tree + new_entity_class_idx;

			change_without_lookup =
				(new_entity_class_idx ==
				 bfq_class_idx(next_in_service)
				 &&
				 !bfq_gt(new_entity->start, st->vtime)
				 &&
				 bfq_gt(next_in_service->finish,
					new_entity->finish));
                        if(open_bfqq_printk)
	                    printk("3:%s %d %s %d change_without_lookup:%d new_entity->start:%llu st->vtime:%llu next_in_service->finish:%llu new_entity->finish:%llu\n",__func__,__LINE__,current->comm,current->pid,change_without_lookup,new_entity->start,st->vtime,next_in_service->finish,new_entity->finish);
		}

		if (change_without_lookup)
			next_in_service = new_entity;
	}

	if (!change_without_lookup){ /* lookup needed */
		if(open_bfqq_printk)
			  printk("4:%s %d %s %d expiration:%d ->bfq_lookup_next_entity()\n",__func__,__LINE__,current->comm,current->pid,expiration);
		next_in_service = bfq_lookup_next_entity(sd, expiration);
	}

	if (next_in_service) {
		bool new_budget_triggers_change =
			bfq_update_parent_budget(next_in_service);

		parent_sched_may_change = !sd->next_in_service ||
			new_budget_triggers_change;
		if(open_bfqq_printk)
			  printk("5:%s %d %s %d if (next_in_service)\n",__func__,__LINE__,current->comm,current->pid);
	}

        if(open_bfqq_printk)
	     printk("6:%s %d %s %d old sd->next_in_service entity:%llx pid:%d  new sd->next_in_service entity:%llx pid:%d\n",__func__,__LINE__,current->comm,current->pid,(u64)(sd->next_in_service),(sd->next_in_service == NULL)? 0: bfq_entity_to_bfqq(sd->next_in_service)->pid,(u64)(next_in_service),(next_in_service == NULL)? 0:bfq_entity_to_bfqq(next_in_service)->pid);

	sd->next_in_service = next_in_service;

	if (!next_in_service)
		return parent_sched_may_change;

	/*************************************/
	bfqq = bfq_entity_to_bfqq(next_in_service);
	//到这里，如果bfqq已经添加到了 deadline_head 链表则要剔除掉。说明该bfqq在截至时间到来时就已经得到派发机会了
        if(bfqq->deadline_list.next != LIST_POISON1 && bfqq->deadline_list.prev != LIST_POISON2){
	    //printk("%s %s %d bfqq:%llx bfqq->pid:%d list_del(&bfqq->deadline_list)************\n",__func__,current->comm,current->pid,(u64)bfqq,bfqq->pid);
	    //dump_stack();
	    list_del(&bfqq->deadline_list);
	}

	return parent_sched_may_change;
}

#ifdef CONFIG_BFQ_GROUP_IOSCHED

struct bfq_group *bfq_bfqq_to_bfqg(struct bfq_queue *bfqq)
{
	struct bfq_entity *group_entity = bfqq->entity.parent;

	if (!group_entity)
		group_entity = &bfqq->bfqd->root_group->entity;

	return container_of(group_entity, struct bfq_group, entity);
}

/*
 * Returns true if this budget changes may let next_in_service->parent
 * become the next_in_service entity for its parent entity.
 */
static bool bfq_update_parent_budget(struct bfq_entity *next_in_service)
{
	struct bfq_entity *bfqg_entity;
	struct bfq_group *bfqg;
	struct bfq_sched_data *group_sd;
	bool ret = false;

	group_sd = next_in_service->sched_data;

	bfqg = container_of(group_sd, struct bfq_group, sched_data);
	/*
	 * bfq_group's my_entity field is not NULL only if the group
	 * is not the root group. We must not touch the root entity
	 * as it must never become an in-service entity.
	 */
	bfqg_entity = bfqg->my_entity;
	if (bfqg_entity) {
		if (bfqg_entity->budget > next_in_service->budget)
			ret = true;
		bfqg_entity->budget = next_in_service->budget;
	}

	return ret;
}

/*
 * This function tells whether entity stops being a candidate for next
 * service, according to the restrictive definition of the field
 * next_in_service. In particular, this function is invoked for an
 * entity that is about to be set in service.
 *
 * If entity is a queue, then the entity is no longer a candidate for
 * next service according to the that definition, because entity is
 * about to become the in-service queue. This function then returns
 * true if entity is a queue.
 *
 * In contrast, entity could still be a candidate for next service if
 * it is not a queue, and has more than one active child. In fact,
 * even if one of its children is about to be set in service, other
 * active children may still be the next to serve, for the parent
 * entity, even according to the above definition. As a consequence, a
 * non-queue entity is not a candidate for next-service only if it has
 * only one active child. And only if this condition holds, then this
 * function returns true for a non-queue entity.
 */
static bool bfq_no_longer_next_in_service(struct bfq_entity *entity)
{
	struct bfq_group *bfqg;

	if(open_bfqq_printk)
		  printk("1:%s %d %s %d entity->my_sched_data:%llx\n",__func__,__LINE__,current->comm,current->pid,(u64)(entity->my_sched_data));
	if (bfq_entity_to_bfqq(entity))
		return true;

	bfqg = container_of(entity, struct bfq_group, entity);

	if(open_bfqq_printk)
		  printk("2:%s %d %s %d bfqg->active_entities:%d\n",__func__,__LINE__,current->comm,current->pid,bfqg->active_entities);
	/*
	 * The field active_entities does not always contain the
	 * actual number of active children entities: it happens to
	 * not account for the in-service entity in case the latter is
	 * removed from its active tree (which may get done after
	 * invoking the function bfq_no_longer_next_in_service in
	 * bfq_get_next_queue). Fortunately, here, i.e., while
	 * bfq_no_longer_next_in_service is not yet completed in
	 * bfq_get_next_queue, bfq_active_extract has not yet been
	 * invoked, and thus active_entities still coincides with the
	 * actual number of active entities.
	 */
	if (bfqg->active_entities == 1)
		return true;

	return false;
}

#else /* CONFIG_BFQ_GROUP_IOSCHED */

struct bfq_group *bfq_bfqq_to_bfqg(struct bfq_queue *bfqq)
{
	return bfqq->bfqd->root_group;
}

static bool bfq_update_parent_budget(struct bfq_entity *next_in_service)
{
	return false;
}

static bool bfq_no_longer_next_in_service(struct bfq_entity *entity)
{
	return true;
}

#endif /* CONFIG_BFQ_GROUP_IOSCHED */

/*
 * Shift for timestamp calculations.  This actually limits the maximum
 * service allowed in one timestamp delta (small shift values increase it),
 * the maximum total weight that can be used for the queues in the system
 * (big shift values increase it), and the period of virtual time
 * wraparounds.
 */
#define WFQ_SERVICE_SHIFT	22

struct bfq_queue *bfq_entity_to_bfqq(struct bfq_entity *entity)
{
	struct bfq_queue *bfqq = NULL;

	if (!entity->my_sched_data)
		bfqq = container_of(entity, struct bfq_queue, entity);

	return bfqq;
}


/**
 * bfq_delta - map service into the virtual time domain.
 * @service: amount of service.
 * @weight: scale factor (weight of an entity or weight sum).
 */
static u64 bfq_delta(unsigned long service, unsigned long weight)
{
	return div64_ul((u64)service << WFQ_SERVICE_SHIFT, weight);
}

/**
 * bfq_calc_finish - assign the finish time to an entity.
 * @entity: the entity to act upon.
 * @service: the service to be charged to the entity.
 */
static void bfq_calc_finish(struct bfq_entity *entity, unsigned long service)
{
	struct bfq_queue *bfqq = bfq_entity_to_bfqq(entity);

	entity->finish = entity->start +
		bfq_delta(service, entity->weight);
        if(open_bfqq_printk)
	     printk("%s %d %s %d entity:%llx service:%ld entity->start:%llu entity->finish:%llu entity->weight:%d bfq_delta:%llu\n",__func__,__LINE__,current->comm,current->pid,(u64)entity,service,entity->start,entity->finish,entity->weight,bfq_delta(service, entity->weight));
	if (bfqq) {
		bfq_log_bfqq(bfqq->bfqd, bfqq,
			"calc_finish: serv %lu, w %d",
			service, entity->weight);
		bfq_log_bfqq(bfqq->bfqd, bfqq,
			"calc_finish: start %llu, finish %llu, delta %llu",
			entity->start, entity->finish,
			bfq_delta(service, entity->weight));
	}
}

/**
 * bfq_entity_of - get an entity from a node.
 * @node: the node field of the entity.
 *
 * Convert a node pointer to the relative entity.  This is used only
 * to simplify the logic of some functions and not as the generic
 * conversion mechanism because, e.g., in the tree walking functions,
 * the check for a %NULL value would be redundant.
 */
struct bfq_entity *bfq_entity_of(struct rb_node *node)
{
	struct bfq_entity *entity = NULL;

	if (node)
		entity = rb_entry(node, struct bfq_entity, rb_node);

	return entity;
}

/**
 * bfq_extract - remove an entity from a tree.
 * @root: the tree root.
 * @entity: the entity to remove.
 */
static void bfq_extract(struct rb_root *root, struct bfq_entity *entity)
{
	entity->tree = NULL;
	rb_erase(&entity->rb_node, root);
}

/**
 * bfq_idle_extract - extract an entity from the idle tree.
 * @st: the service tree of the owning @entity.
 * @entity: the entity being removed.
 */
static void bfq_idle_extract(struct bfq_service_tree *st,
			     struct bfq_entity *entity)
{
	struct bfq_queue *bfqq = bfq_entity_to_bfqq(entity);
	struct rb_node *next;

	if (entity == st->first_idle) {
		next = rb_next(&entity->rb_node);
		if(open_bfqq_printk)
			  printk("1:%s %d %s %d entity:%llx st->first_idle entity:%llx next st->first_idle entity:%llx\n",__func__,__LINE__,current->comm,current->pid,(u64)entity,(u64)(st->first_idle),(u64)(bfq_entity_of(next)));
		st->first_idle = bfq_entity_of(next);
	}

	if (entity == st->last_idle) {
		next = rb_prev(&entity->rb_node);
	       if(open_bfqq_printk)
			printk("2:%s %d %s %d entity:%llx st->last_idle entity:%llx  next st->last_idle entity:%llx\n",__func__,__LINE__,current->comm,current->pid,(u64)entity,(u64)(st->last_idle),(u64)(bfq_entity_of(next)));
		st->last_idle = bfq_entity_of(next);
	}
        if(open_bfqq_printk)
	      printk("3:%s %d %s %d bfq_service_tree:%llx entity:%llx &st->idle rb_root:%llx\n",__func__,__LINE__,current->comm,current->pid,(u64)st,(u64)entity,(u64)(&st->idle));
	bfq_extract(&st->idle, entity);

	if (bfqq)
		list_del(&bfqq->bfqq_list);
}

/**
 * bfq_insert - generic tree insertion.
 * @root: tree root.
 * @entity: entity to insert.
 *
 * This is used for the idle and the active tree, since they are both
 * ordered by finish time.
 */
static void bfq_insert(struct rb_root *root, struct bfq_entity *entity)
{
	struct bfq_entity *entry;
	struct rb_node **node = &root->rb_node;
	struct rb_node *parent = NULL;

        if(open_bfqq_printk)
	     printk("%s %d %s %d rb_root root:%llx entity:%llx entity->finish:%llu\n",__func__,__LINE__,current->comm,current->pid,(u64)root,(u64)entity,entity->finish);

	while (*node) {
		parent = *node;
		entry = rb_entry(parent, struct bfq_entity, rb_node);

		if (bfq_gt(entry->finish, entity->finish))
			node = &parent->rb_left;
		else
			node = &parent->rb_right;
	}

	rb_link_node(&entity->rb_node, parent, node);
	rb_insert_color(&entity->rb_node, root);

	entity->tree = root;
}

/**
 * bfq_update_min - update the min_start field of a entity.
 * @entity: the entity to update.
 * @node: one of its children.
 *
 * This function is called when @entity may store an invalid value for
 * min_start due to updates to the active tree.  The function  assumes
 * that the subtree rooted at @node (which may be its left or its right
 * child) has a valid min_start value.
 */
static void bfq_update_min(struct bfq_entity *entity, struct rb_node *node)
{
	struct bfq_entity *child;

	if (node) {
		child = rb_entry(node, struct bfq_entity, rb_node);
		if (bfq_gt(entity->min_start, child->min_start)){
			entity->min_start = child->min_start;
	                if(open_bfqq_printk)
                            printk("%s %d %s %d entity->min_start:%lld entity:%llx\n",__func__,__LINE__,current->comm,current->pid,entity->min_start,(u64)entity);
		}
	}
}

/**
 * bfq_update_active_node - recalculate min_start.
 * @node: the node to update.
 *
 * @node may have changed position or one of its children may have moved,
 * this function updates its min_start value.  The left and right subtrees
 * are assumed to hold a correct min_start value.
 */
static void bfq_update_active_node(struct rb_node *node)
{
	struct bfq_entity *entity = rb_entry(node, struct bfq_entity, rb_node);

	if(open_bfqq_printk)
            printk("%s %d %s %d entity->min_start = entity->start:%lld entity:%llx\n",__func__,__LINE__,current->comm,current->pid,entity->start,(u64)entity);
	entity->min_start = entity->start;
	bfq_update_min(entity, node->rb_right);
	bfq_update_min(entity, node->rb_left);
}

/**
 * bfq_update_active_tree - update min_start for the whole active tree.
 * @node: the starting node.
 *
 * @node must be the deepest modified node after an update.  This function
 * updates its min_start using the values held by its children, assuming
 * that they did not change, and then updates all the nodes that may have
 * changed in the path to the root.  The only nodes that may have changed
 * are the ones in the path or their siblings.
 */
static void bfq_update_active_tree(struct rb_node *node)
{
	struct rb_node *parent;

up:
	bfq_update_active_node(node);

	parent = rb_parent(node);
	if (!parent)
		return;

	if (node == parent->rb_left && parent->rb_right)
		bfq_update_active_node(parent->rb_right);
	else if (parent->rb_left)
		bfq_update_active_node(parent->rb_left);

	node = parent;
	goto up;
}

/**
 * bfq_active_insert - insert an entity in the active tree of its
 *                     group/device.
 * @st: the service tree of the entity.
 * @entity: the entity being inserted.
 *
 * The active tree is ordered by finish time, but an extra key is kept
 * per each node, containing the minimum value for the start times of
 * its children (and the node itself), so it's possible to search for
 * the eligible node with the lowest finish time in logarithmic time.
 */
static void bfq_active_insert(struct bfq_service_tree *st,
			      struct bfq_entity *entity)
{
	struct bfq_queue *bfqq = bfq_entity_to_bfqq(entity);
	struct rb_node *node = &entity->rb_node;
#ifdef CONFIG_BFQ_GROUP_IOSCHED
	struct bfq_sched_data *sd = NULL;
	struct bfq_group *bfqg = NULL;
	struct bfq_data *bfqd = NULL;
#endif
	if(open_bfqq_printk)
            printk("%s %d %s %d entity:%llx ->bfq_insert ->bfq_update_active_tree\n",__func__,__LINE__,current->comm,current->pid,(u64)entity);

	bfq_insert(&st->active, entity);

	if (node->rb_left)
		node = node->rb_left;
	else if (node->rb_right)
		node = node->rb_right;

	bfq_update_active_tree(node);

#ifdef CONFIG_BFQ_GROUP_IOSCHED
	sd = entity->sched_data;
	bfqg = container_of(sd, struct bfq_group, sched_data);
	bfqd = (struct bfq_data *)bfqg->bfqd;
#endif
	if (bfqq)
		list_add(&bfqq->bfqq_list, &bfqq->bfqd->active_list);
#ifdef CONFIG_BFQ_GROUP_IOSCHED
	if (bfqg != bfqd->root_group)
		bfqg->active_entities++;
#endif
}

/**
 * bfq_ioprio_to_weight - calc a weight from an ioprio.
 * @ioprio: the ioprio value to convert.
 */
unsigned short bfq_ioprio_to_weight(int ioprio)
{
	return (IOPRIO_BE_NR - ioprio) * BFQ_WEIGHT_CONVERSION_COEFF;
}

/**
 * bfq_weight_to_ioprio - calc an ioprio from a weight.
 * @weight: the weight value to convert.
 *
 * To preserve as much as possible the old only-ioprio user interface,
 * 0 is used as an escape ioprio value for weights (numerically) equal or
 * larger than IOPRIO_BE_NR * BFQ_WEIGHT_CONVERSION_COEFF.
 */
static unsigned short bfq_weight_to_ioprio(int weight)
{
	return max_t(int, 0,
		     IOPRIO_BE_NR * BFQ_WEIGHT_CONVERSION_COEFF - weight);
}

static void bfq_get_entity(struct bfq_entity *entity)
{
	struct bfq_queue *bfqq = bfq_entity_to_bfqq(entity);

	if (bfqq) {
		bfqq->ref++;
		bfq_log_bfqq(bfqq->bfqd, bfqq, "get_entity: %p %d",
			     bfqq, bfqq->ref);
	} else
		bfqg_and_blkg_get(container_of(entity, struct bfq_group,
					       entity));
}

/**
 * bfq_find_deepest - find the deepest node that an extraction can modify.
 * @node: the node being removed.
 *
 * Do the first step of an extraction in an rb tree, looking for the
 * node that will replace @node, and returning the deepest node that
 * the following modifications to the tree can touch.  If @node is the
 * last node in the tree return %NULL.
 */
static struct rb_node *bfq_find_deepest(struct rb_node *node)
{
	struct rb_node *deepest;

	if (!node->rb_right && !node->rb_left)
		deepest = rb_parent(node);
	else if (!node->rb_right)
		deepest = node->rb_left;
	else if (!node->rb_left)
		deepest = node->rb_right;
	else {
		deepest = rb_next(node);
		if (deepest->rb_right)
			deepest = deepest->rb_right;
		else if (rb_parent(deepest) != node)
			deepest = rb_parent(deepest);
	}

	return deepest;
}

/**
 * bfq_active_extract - remove an entity from the active tree.
 * @st: the service_tree containing the tree.
 * @entity: the entity being removed.
 */
static void bfq_active_extract(struct bfq_service_tree *st,
			       struct bfq_entity *entity)
{
	struct bfq_queue *bfqq = bfq_entity_to_bfqq(entity);
	struct rb_node *node;
#ifdef CONFIG_BFQ_GROUP_IOSCHED
	struct bfq_sched_data *sd = NULL;
	struct bfq_group *bfqg = NULL;
	struct bfq_data *bfqd = NULL;
#endif

	node = bfq_find_deepest(&entity->rb_node);
	bfq_extract(&st->active, entity);

        if(open_bfqq_printk)
	     printk("1:%s %d %s %d bfq_service_tree:%llx bfq_entity:%llx bfqq:%llx rb_node:%llx\n",__func__,__LINE__,current->comm,current->pid,(u64)st,(u64)entity,(u64)bfqq,(u64)node);
	if (node)
		bfq_update_active_tree(node);

#ifdef CONFIG_BFQ_GROUP_IOSCHED
	sd = entity->sched_data;
	bfqg = container_of(sd, struct bfq_group, sched_data);
	bfqd = (struct bfq_data *)bfqg->bfqd;
#endif
	if (bfqq)
		list_del(&bfqq->bfqq_list);
#ifdef CONFIG_BFQ_GROUP_IOSCHED
	if (bfqg != bfqd->root_group)
		bfqg->active_entities--;
#endif
}

/**
 * bfq_idle_insert - insert an entity into the idle tree.
 * @st: the service tree containing the tree.
 * @entity: the entity to insert.
 */
static void bfq_idle_insert(struct bfq_service_tree *st,
			    struct bfq_entity *entity)
{
	struct bfq_queue *bfqq = bfq_entity_to_bfqq(entity);
	struct bfq_entity *first_idle = st->first_idle;
	struct bfq_entity *last_idle = st->last_idle;

	if (!first_idle || bfq_gt(first_idle->finish, entity->finish)){
		st->first_idle = entity;
		if(open_bfqq_printk)
			  printk("1:%s %d %s %d st->first_idle entity:%llx\n",__func__,__LINE__,current->comm,current->pid,(u64)(st->first_idle));
	}
	if (!last_idle || bfq_gt(entity->finish, last_idle->finish)){
		st->last_idle = entity;
		if(open_bfqq_printk)
			  printk("2:%s %d %s %d st->last_idle entity:%llx\n",__func__,__LINE__,current->comm,current->pid,(u64)st->last_idle);
	}
        if(open_bfqq_printk)
	    printk("3:%s %d %s %d ->bfq_insert() &st->idle:%llx entity:%llx\n",__func__,__LINE__,current->comm,current->pid,(u64)(&st->idle),(u64)entity);

	bfq_insert(&st->idle, entity);

	if (bfqq)
		list_add(&bfqq->bfqq_list, &bfqq->bfqd->idle_list);
}

/**
 * bfq_forget_entity - do not consider entity any longer for scheduling
 * @st: the service tree.
 * @entity: the entity being removed.
 * @is_in_service: true if entity is currently the in-service entity.
 *
 * Forget everything about @entity. In addition, if entity represents
 * a queue, and the latter is not in service, then release the service
 * reference to the queue (the one taken through bfq_get_entity). In
 * fact, in this case, there is really no more service reference to
 * the queue, as the latter is also outside any service tree. If,
 * instead, the queue is in service, then __bfq_bfqd_reset_in_service
 * will take care of putting the reference when the queue finally
 * stops being served.
 */
static void bfq_forget_entity(struct bfq_service_tree *st,
			      struct bfq_entity *entity,
			      bool is_in_service)
{
	struct bfq_queue *bfqq = bfq_entity_to_bfqq(entity);

	entity->on_st_or_in_serv = false;
	if(open_bfqq_printk)
		  printk("%s %d %s %d st->wsum:%ld entity->weight:%d is_in_service:%d\n",__func__,__LINE__,current->comm,current->pid,st->wsum,entity->weight,is_in_service);
	st->wsum -= entity->weight;
	if (is_in_service)
		return;

	if (bfqq)
		bfq_put_queue(bfqq);
	else
		bfqg_and_blkg_put(container_of(entity, struct bfq_group,
					       entity));
}

/**
 * bfq_put_idle_entity - release the idle tree ref of an entity.
 * @st: service tree for the entity.
 * @entity: the entity being released.
 */
void bfq_put_idle_entity(struct bfq_service_tree *st, struct bfq_entity *entity)
{
	bfq_idle_extract(st, entity);
	bfq_forget_entity(st, entity,
			  entity == entity->sched_data->in_service_entity);
}

/**
 * bfq_forget_idle - update the idle tree if necessary.
 * @st: the service tree to act upon.
 *
 * To preserve the global O(log N) complexity we only remove one entry here;
 * as the idle tree will not grow indefinitely this can be done safely.
 */
static void bfq_forget_idle(struct bfq_service_tree *st)
{
	struct bfq_entity *first_idle = st->first_idle;
	struct bfq_entity *last_idle = st->last_idle;

        if(open_bfqq_printk)
	    printk("1:%s %d %s %d st->vtime:%llu\n",__func__,__LINE__,current->comm,current->pid,st->vtime);
	if (RB_EMPTY_ROOT(&st->active) && last_idle &&
	    !bfq_gt(last_idle->finish, st->vtime)) {
		/*
		 * Forget the whole idle tree, increasing the vtime past
		 * the last finish time of idle entities.
		 */
		if(open_bfqq_printk)
	            printk("2:%s %d %s %d last_idle->finish:%llu\n",__func__,__LINE__,current->comm,current->pid,last_idle->finish);
		st->vtime = last_idle->finish;
	}
	if (first_idle && !bfq_gt(first_idle->finish, st->vtime)){
		if(open_bfqq_printk)
		    printk("3:%s %d %s %d first_idle->finish:%llu\n",__func__,__LINE__,current->comm,current->pid,first_idle->finish);
		bfq_put_idle_entity(st, first_idle);
	}
}

struct bfq_service_tree *bfq_entity_service_tree(struct bfq_entity *entity)
{
	struct bfq_sched_data *sched_data = entity->sched_data;
	unsigned int idx = bfq_class_idx(entity);
        if(open_bfqq_printk)
	  printk("%s %d %s %d bfq_class_idx:%d\n",__func__,__LINE__,current->comm,current->pid,idx);
	return sched_data->service_tree + idx;
}

/*
 * Update weight and priority of entity. If update_class_too is true,
 * then update the ioprio_class of entity too.
 *
 * The reason why the update of ioprio_class is controlled through the
 * last parameter is as follows. Changing the ioprio class of an
 * entity implies changing the destination service trees for that
 * entity. If such a change occurred when the entity is already on one
 * of the service trees for its previous class, then the state of the
 * entity would become more complex: none of the new possible service
 * trees for the entity, according to bfq_entity_service_tree(), would
 * match any of the possible service trees on which the entity
 * is. Complex operations involving these trees, such as entity
 * activations and deactivations, should take into account this
 * additional complexity.  To avoid this issue, this function is
 * invoked with update_class_too unset in the points in the code where
 * entity may happen to be on some tree.
 */
struct bfq_service_tree *
__bfq_entity_update_weight_prio(struct bfq_service_tree *old_st,
				struct bfq_entity *entity,
				bool update_class_too)
{
	struct bfq_service_tree *new_st = old_st;

	if (entity->prio_changed) {
		struct bfq_queue *bfqq = bfq_entity_to_bfqq(entity);
		unsigned int prev_weight, new_weight;
		struct bfq_data *bfqd = NULL;
		struct rb_root_cached *root;
#ifdef CONFIG_BFQ_GROUP_IOSCHED
		struct bfq_sched_data *sd;
		struct bfq_group *bfqg;
#endif

		if(open_bfqq_printk)
		    printk("1:%s %d %s %d entity:%llx bfqq->wr_coeff:%d entity->new_weight:%d entity->orig_weight:%d entity->weight:%d update_class_too:%d\n",__func__,__LINE__,current->comm,current->pid,(u64)entity,bfqq->wr_coeff,entity->new_weight,entity->orig_weight,entity->weight,update_class_too);

		if (bfqq)
			bfqd = bfqq->bfqd;
#ifdef CONFIG_BFQ_GROUP_IOSCHED
		else {
			sd = entity->my_sched_data;
			bfqg = container_of(sd, struct bfq_group, sched_data);
			bfqd = (struct bfq_data *)bfqg->bfqd;
		}
#endif

		/* Matches the smp_wmb() in bfq_group_set_weight. */
		smp_rmb();
		old_st->wsum -= entity->weight;

		if (entity->new_weight != entity->orig_weight) {
			if (entity->new_weight < BFQ_MIN_WEIGHT ||
			    entity->new_weight > BFQ_MAX_WEIGHT) {
				pr_crit("update_weight_prio: new_weight %d\n",
					entity->new_weight);
				if (entity->new_weight < BFQ_MIN_WEIGHT)
					entity->new_weight = BFQ_MIN_WEIGHT;
				else
					entity->new_weight = BFQ_MAX_WEIGHT;
			}
			entity->orig_weight = entity->new_weight;
			if (bfqq)
				bfqq->ioprio =
				  bfq_weight_to_ioprio(entity->orig_weight);
		}

		if (bfqq && update_class_too){
       	                if(open_bfqq_printk)
	                    printk("2:%s %d %s %d entity:%llx bfqq->ioprio_class:%d bfqq->new_ioprio_class:%d\n",__func__,__LINE__,current->comm,current->pid,(u64)entity,bfqq->ioprio_class,bfqq->new_ioprio_class);
			bfqq->ioprio_class = bfqq->new_ioprio_class;
                }
		/*
		 * Reset prio_changed only if the ioprio_class change
		 * is not pending any longer.
		 */
		if (!bfqq || bfqq->ioprio_class == bfqq->new_ioprio_class)
			entity->prio_changed = 0;

		/*
		 * NOTE: here we may be changing the weight too early,
		 * this will cause unfairness.  The correct approach
		 * would have required additional complexity to defer
		 * weight changes to the proper time instants (i.e.,
		 * when entity->finish <= old_st->vtime).
		 */
		new_st = bfq_entity_service_tree(entity);

		prev_weight = entity->weight;
		new_weight = entity->orig_weight *
			     (bfqq ? bfqq->wr_coeff : 1);
		/*
		 * If the weight of the entity changes, and the entity is a
		 * queue, remove the entity from its old weight counter (if
		 * there is a counter associated with the entity).
		 */
		if (prev_weight != new_weight && bfqq) {
			root = &bfqd->queue_weights_tree;
			__bfq_weights_tree_remove(bfqd, bfqq, root);
		}
		entity->weight = new_weight;
		/*
		 * Add the entity, if it is not a weight-raised queue,
		 * to the counter associated with its new weight.
		 */
		if (prev_weight != new_weight && bfqq && bfqq->wr_coeff == 1) {
			/* If we get here, root has been initialized. */
			bfq_weights_tree_add(bfqd, bfqq, root);
		}

		new_st->wsum += entity->weight;

		if (new_st != old_st)
			entity->start = new_st->vtime;
	}
	if(open_bfqq_printk)
	    printk("3:%s %d %s %d entity:%llx entity->weight:%d entity->new_weight:%d entity->orig_weight:%d new_st:%llx new_st->wsum:%ld old_st:%llx update_class_too:%d\n",__func__,__LINE__,current->comm,current->pid,(u64)entity,entity->weight,entity->new_weight,entity->orig_weight,(u64)new_st,new_st->wsum,(u64)old_st,update_class_too);

	return new_st;
}

/**
 * bfq_bfqq_served - update the scheduler status after selection for
 *                   service.
 * @bfqq: the queue being served.
 * @served: bytes to transfer.
 *
 * NOTE: this can be optimized, as the timestamps of upper level entities
 * are synchronized every time a new bfqq is selected for service.  By now,
 * we keep it to better check consistency.
 */
void bfq_bfqq_served(struct bfq_queue *bfqq, int served)
{
	struct bfq_entity *entity = &bfqq->entity;
	struct bfq_service_tree *st;

	if(open_bfqq_printk)
		  printk("1:%s %d %s %d bfqq->first_IO_time:%ld bfqq->service_from_backlogged:%ld jiffies:%ld bfqq->wr_coeff:%d bfqq->service_from_wr:%ld  served:%d\n",__func__,__LINE__,current->comm,current->pid,bfqq->first_IO_time,bfqq->service_from_backlogged,jiffies,bfqq->wr_coeff,bfqq->service_from_wr,served);

	if (!bfqq->service_from_backlogged)
		bfqq->first_IO_time = jiffies;

	if (bfqq->wr_coeff > 1)
		bfqq->service_from_wr += served;

	bfqq->service_from_backlogged += served;
	for_each_entity(entity) {
		st = bfq_entity_service_tree(entity);

                if(open_bfqq_printk)
	            printk("2:%s %d %s %d bfq_service_tree:%llx entity:%llx entity->service:%d st->vtime:%lld st->wsum:%ld bfq_delta:%lld\n",__func__,__LINE__,current->comm,current->pid,(u64)st,(u64)entity,entity->service,st->vtime,st->wsum,bfq_delta(served, st->wsum));

		entity->service += served;

		st->vtime += bfq_delta(served, st->wsum);
		bfq_forget_idle(st);
	}
	bfq_log_bfqq(bfqq->bfqd, bfqq, "bfqq_served %d secs", served);
}

/**
 * bfq_bfqq_charge_time - charge an amount of service equivalent to the length
 *			  of the time interval during which bfqq has been in
 *			  service.
 * @bfqd: the device
 * @bfqq: the queue that needs a service update.
 * @time_ms: the amount of time during which the queue has received service
 *
 * If a queue does not consume its budget fast enough, then providing
 * the queue with service fairness may impair throughput, more or less
 * severely. For this reason, queues that consume their budget slowly
 * are provided with time fairness instead of service fairness. This
 * goal is achieved through the BFQ scheduling engine, even if such an
 * engine works in the service, and not in the time domain. The trick
 * is charging these queues with an inflated amount of service, equal
 * to the amount of service that they would have received during their
 * service slot if they had been fast, i.e., if their requests had
 * been dispatched at a rate equal to the estimated peak rate.
 *
 * It is worth noting that time fairness can cause important
 * distortions in terms of bandwidth distribution, on devices with
 * internal queueing. The reason is that I/O requests dispatched
 * during the service slot of a queue may be served after that service
 * slot is finished, and may have a total processing time loosely
 * correlated with the duration of the service slot. This is
 * especially true for short service slots.
 */
void bfq_bfqq_charge_time(struct bfq_data *bfqd, struct bfq_queue *bfqq,
			  unsigned long time_ms)
{
	struct bfq_entity *entity = &bfqq->entity;
	unsigned long timeout_ms = jiffies_to_msecs(bfq_timeout);
	unsigned long bounded_time_ms = min(time_ms, timeout_ms);
	int serv_to_charge_for_time =
		(bfqd->bfq_max_budget * bounded_time_ms) / timeout_ms;
	int tot_serv_to_charge = max(serv_to_charge_for_time, entity->service);

	if(open_bfqq_printk)
            printk("%s %d %s %d bfqq:%llx entity:%llx tot_serv_to_charge:%d entity->budget:%d entity->service:%d\n",__func__,__LINE__,current->comm,current->pid,(u64)bfqq,(u64)entity,tot_serv_to_charge,entity->budget,entity->service);
	/* Increase budget to avoid inconsistencies */
	if (tot_serv_to_charge > entity->budget)
		entity->budget = tot_serv_to_charge;

	bfq_bfqq_served(bfqq,
			max_t(int, 0, tot_serv_to_charge - entity->service));
}

static void bfq_update_fin_time_enqueue(struct bfq_entity *entity,
					struct bfq_service_tree *st,
					bool backshifted)
{
	struct bfq_queue *bfqq = bfq_entity_to_bfqq(entity);
       
       	if(open_bfqq_printk)
	    printk("1:%s %d %s %d entity:%llx entity->budget:%d backshifted:%d\n",__func__,__LINE__,current->comm,current->pid,(u64)entity,entity->budget,backshifted);
	/*
	 * When this function is invoked, entity is not in any service
	 * tree, then it is safe to invoke next function with the last
	 * parameter set (see the comments on the function).
	 */
	st = __bfq_entity_update_weight_prio(st, entity, true);
	bfq_calc_finish(entity, entity->budget);

	/*
	 * If some queues enjoy backshifting for a while, then their
	 * (virtual) finish timestamps may happen to become lower and
	 * lower than the system virtual time.	In particular, if
	 * these queues often happen to be idle for short time
	 * periods, and during such time periods other queues with
	 * higher timestamps happen to be busy, then the backshifted
	 * timestamps of the former queues can become much lower than
	 * the system virtual time. In fact, to serve the queues with
	 * higher timestamps while the ones with lower timestamps are
	 * idle, the system virtual time may be pushed-up to much
	 * higher values than the finish timestamps of the idle
	 * queues. As a consequence, the finish timestamps of all new
	 * or newly activated queues may end up being much larger than
	 * those of lucky queues with backshifted timestamps. The
	 * latter queues may then monopolize the device for a lot of
	 * time. This would simply break service guarantees.
	 *
	 * To reduce this problem, push up a little bit the
	 * backshifted timestamps of the queue associated with this
	 * entity (only a queue can happen to have the backshifted
	 * flag set): just enough to let the finish timestamp of the
	 * queue be equal to the current value of the system virtual
	 * time. This may introduce a little unfairness among queues
	 * with backshifted timestamps, but it does not break
	 * worst-case fairness guarantees.
	 *
	 * As a special case, if bfqq is weight-raised, push up
	 * timestamps much less, to keep very low the probability that
	 * this push up causes the backshifted finish timestamps of
	 * weight-raised queues to become higher than the backshifted
	 * finish timestamps of non weight-raised queues.
	 */
	if(open_bfqq_printk)
	       printk("2:%s %d %s %d st->vtime:%llu entity->finish:%llu\n",__func__,__LINE__,current->comm,current->pid,st->vtime,entity->finish);
	if (backshifted && bfq_gt(st->vtime, entity->finish)) {
		unsigned long delta = st->vtime - entity->finish;

		if (bfqq)
			delta /= bfqq->wr_coeff;

		entity->start += delta;
		entity->finish += delta;
	        if(open_bfqq_printk)
	            printk("3:%s %d %s %d entity->start:%llu entity->finish:%llu delta:%ld\n",__func__,__LINE__,current->comm,current->pid,entity->start,entity->finish,delta);
	}

	bfq_active_insert(st, entity);
}

/**
 * __bfq_activate_entity - handle activation of entity.
 * @entity: the entity being activated.
 * @non_blocking_wait_rq: true if entity was waiting for a request
 *
 * Called for a 'true' activation, i.e., if entity is not active and
 * one of its children receives a new request.
 *
 * Basically, this function updates the timestamps of entity and
 * inserts entity into its active tree, after possibly extracting it
 * from its idle tree.
 */
static void __bfq_activate_entity(struct bfq_entity *entity,
				  bool non_blocking_wait_rq)
{
	struct bfq_service_tree *st = bfq_entity_service_tree(entity);
	bool backshifted = false;
	unsigned long long min_vstart;

	/************************************************/
	struct bfq_queue *bfqq = bfq_entity_to_bfqq(entity);
	//high prio io的bfqq，记录激活加入st->active tree的时间点。在 high_prio_io_schedule_deadline 时间点到期后，该bfqq必须被调度到派发rq。
	//这是针对从st->idle tree激活添加到st->active tree的bfqq，是否有必要针对__bfq_requeue_entity的bfqq也这样操作呢?????????????????????????????????
	//bfqq->deadline_list->prev 和 next 必须是LIST_POISON2/LIST_POISON1 ，说明没有添加到链表上
	if((bfqq->deadline_list.prev == LIST_POISON2) && (bfqq->deadline_list.next == LIST_POISON1) && (bfqq->wr_coeff == 30 * BFQ_HIGH_PRIO_IO_WEIGHT_FACTOR)){
	   bfqq->high_prio_io_active_time = jiffies;
	   list_add_tail(&bfqq->deadline_list, &bfqq->bfqd->deadline_head);
	}

	/* See comments on bfq_fqq_update_budg_for_activation */
	if (non_blocking_wait_rq && bfq_gt(st->vtime, entity->finish)) {
		backshifted = true;
		min_vstart = entity->finish;
	} else
		min_vstart = st->vtime;
       
	if(open_bfqq_printk)
		  printk("1:%s %d %s %d min_vstart:%lld entity->finish:%lld st->vtime:%lld non_blocking_wait_rq:%d\n",__func__,__LINE__,current->comm,current->pid,min_vstart,entity->finish,st->vtime,non_blocking_wait_rq);

	if (entity->tree == &st->idle) {
		/*
		 * Must be on the idle tree, bfq_idle_extract() will
		 * check for that.
		 */
		if(open_bfqq_printk)
		    printk("2:%s %d %s %d entity:%llx bfq_service_tree:%llx entity->tree:%llx if (entity->tree == &st->idle) ->bfq_idle_extract()\n",__func__,__LINE__,current->comm,current->pid,(u64)entity,(u64)st,(u64)(entity->tree));
		bfq_idle_extract(st, entity);
		entity->start = bfq_gt(min_vstart, entity->finish) ?
			min_vstart : entity->finish;
	} else {
		/*
		 * The finish time of the entity may be invalid, and
		 * it is in the past for sure, otherwise the queue
		 * would have been on the idle tree.
		 */
		entity->start = min_vstart;
		st->wsum += entity->weight;
		/*
		 * entity is about to be inserted into a service tree,
		 * and then set in service: get a reference to make
		 * sure entity does not disappear until it is no
		 * longer in service or scheduled for service.
		 */
		bfq_get_entity(entity);

		entity->on_st_or_in_serv = true;
	}

        if(open_bfqq_printk)
	     printk("3:%s %d %s %d entity->start:%lld st->wsum:%ld entity->weight:%d\n",__func__,__LINE__,current->comm,current->pid,entity->start,st->wsum,entity->weight);
#ifdef CONFIG_BFQ_GROUP_IOSCHED
	if (!bfq_entity_to_bfqq(entity)) { /* bfq_group */
		struct bfq_group *bfqg =
			container_of(entity, struct bfq_group, entity);
		struct bfq_data *bfqd = bfqg->bfqd;

		if (!entity->in_groups_with_pending_reqs) {
			entity->in_groups_with_pending_reqs = true;
			bfqd->num_groups_with_pending_reqs++;
		}
	}
#endif

	bfq_update_fin_time_enqueue(entity, st, backshifted);
}

/**
 * __bfq_requeue_entity - handle requeueing or repositioning of an entity.
 * @entity: the entity being requeued or repositioned.
 *
 * Requeueing is needed if this entity stops being served, which
 * happens if a leaf descendant entity has expired. On the other hand,
 * repositioning is needed if the next_inservice_entity for the child
 * entity has changed. See the comments inside the function for
 * details.
 *
 * Basically, this function: 1) removes entity from its active tree if
 * present there, 2) updates the timestamps of entity and 3) inserts
 * entity back into its active tree (in the new, right position for
 * the new values of the timestamps).
 */
static void __bfq_requeue_entity(struct bfq_entity *entity)
{
	struct bfq_sched_data *sd = entity->sched_data;
	struct bfq_service_tree *st = bfq_entity_service_tree(entity);

	/**************************************************************************/
	//如果bfqq->wr_coeff是30说明是交互式io，执行到这里说明派发这个进程派发的IO太多了，配合消耗完了还没派发完io。此时说明该进程的bfqq需要提升权重，提高游戏那几，作为high prio io.
	struct bfq_queue *bfqq = bfq_entity_to_bfqq(entity);
	if(bfqq && bfqq->bfqd->queue->high_io_prio_enable && bfqq->wr_coeff == 30){
	    bfqq->wr_coeff = 30 * BFQ_HIGH_PRIO_IO_WEIGHT_FACTOR; 
	    //置1表示权重变了，然后才会在bfq_update_fin_time_enqueue->__bfq_entity_update_weight_prio 里真正提升权重
	    entity->prio_changed = 1;
	    //增大权重提升时间为1.5s
	    bfqq->wr_cur_max_time = msecs_to_jiffies(1500);
	    //权重提升时间开始时间为当前时间
	    bfqq->last_wr_start_finish = jiffies;
	    bfqq->entity.completed_size = 0;
	    printk("%s %s %d bfqq:%llx bfqq->pid:%d is high prio io*************\n",__func__,current->comm,current->pid,(u64)bfqq,bfqq->pid);
	}
	if (entity == sd->in_service_entity) {
		if(open_bfqq_printk)
		    printk("1:%s %d %s %d entity:%llx entity->tree:%llx if (entity == sd->in_service_entity)\n",__func__,__LINE__,current->comm,current->pid,(u64)entity,(u64)(entity->tree));
		/*
		 * We are requeueing the current in-service entity,
		 * which may have to be done for one of the following
		 * reasons:
		 * - entity represents the in-service queue, and the
		 *   in-service queue is being requeued after an
		 *   expiration;
		 * - entity represents a group, and its budget has
		 *   changed because one of its child entities has
		 *   just been either activated or requeued for some
		 *   reason; the timestamps of the entity need then to
		 *   be updated, and the entity needs to be enqueued
		 *   or repositioned accordingly.
		 *
		 * In particular, before requeueing, the start time of
		 * the entity must be moved forward to account for the
		 * service that the entity has received while in
		 * service. This is done by the next instructions. The
		 * finish time will then be updated according to this
		 * new value of the start time, and to the budget of
		 * the entity.
		 */
		bfq_calc_finish(entity, entity->service);
		entity->start = entity->finish;
		/*
		 * In addition, if the entity had more than one child
		 * when set in service, then it was not extracted from
		 * the active tree. This implies that the position of
		 * the entity in the active tree may need to be
		 * changed now, because we have just updated the start
		 * time of the entity, and we will update its finish
		 * time in a moment (the requeueing is then, more
		 * precisely, a repositioning in this case). To
		 * implement this repositioning, we: 1) dequeue the
		 * entity here, 2) update the finish time and requeue
		 * the entity according to the new timestamps below.
		 */
		if (entity->tree)
			bfq_active_extract(st, entity);
	} else { /* The entity is already active, and not in service */
		/*
		 * In this case, this function gets called only if the
		 * next_in_service entity below this entity has
		 * changed, and this change has caused the budget of
		 * this entity to change, which, finally implies that
		 * the finish time of this entity must be
		 * updated. Such an update may cause the scheduling,
		 * i.e., the position in the active tree, of this
		 * entity to change. We handle this change by: 1)
		 * dequeueing the entity here, 2) updating the finish
		 * time and requeueing the entity according to the new
		 * timestamps below. This is the same approach as the
		 * non-extracted-entity sub-case above.
		 */
		if(open_bfqq_printk)
			  printk("2:%s %d %s %d ->bfq_active_extract()\n",__func__,__LINE__,current->comm,current->pid);
		bfq_active_extract(st, entity);
	}

	bfq_update_fin_time_enqueue(entity, st, false);
}

static void __bfq_activate_requeue_entity(struct bfq_entity *entity,
					  struct bfq_sched_data *sd,
					  bool non_blocking_wait_rq)
{
	struct bfq_service_tree *st = bfq_entity_service_tree(entity);

	if (sd->in_service_entity == entity || entity->tree == &st->active){
		 /*
		  * in service or already queued on the active tree,
		  * requeue or reposition
		  */
		if(open_bfqq_printk)
			  printk("1:%s %d %s %d entity:%llx bfq_service_tree:%llx ->__bfq_requeue_entity()\n",__func__,__LINE__,current->comm,current->pid,(u64)entity,(u64)st);
		__bfq_requeue_entity(entity);
	}
	else{
		/*
		 * Not in service and not queued on its active tree:
		 * the activity is idle and this is a true activation.
		 */
		
		if(open_bfqq_printk)
			  printk("2:%s %d %s %d entity:%llx bfq_service_tree:%llx non_blocking_wait_rq:%d ->__bfq_activate_entity()\n",__func__,__LINE__,current->comm,current->pid,(u64)entity,(u64)st,non_blocking_wait_rq);
		__bfq_activate_entity(entity, non_blocking_wait_rq);
	}
}


/**
 * bfq_activate_requeue_entity - activate or requeue an entity representing a
 *				 bfq_queue, and activate, requeue or reposition
 *				 all ancestors for which such an update becomes
 *				 necessary.
 * @entity: the entity to activate.
 * @non_blocking_wait_rq: true if this entity was waiting for a request
 * @requeue: true if this is a requeue, which implies that bfqq is
 *	     being expired; thus ALL its ancestors stop being served and must
 *	     therefore be requeued
 * @expiration: true if this function is being invoked in the expiration path
 *             of the in-service queue
 */
static void bfq_activate_requeue_entity(struct bfq_entity *entity,
					bool non_blocking_wait_rq,
					bool requeue, bool expiration)
{
	struct bfq_sched_data *sd;

	for_each_entity(entity) {
		sd = entity->sched_data;
		if(open_bfqq_printk)
			  printk("1:%s %d %s %d bfq_sched_data:%llx entity:%llx expiration:%d non_blocking_wait_rq:%d\n",__func__,__LINE__,current->comm,current->pid,(u64)sd,(u64)entity,expiration,non_blocking_wait_rq);
		__bfq_activate_requeue_entity(entity, sd, non_blocking_wait_rq);

		if (!bfq_update_next_in_service(sd, entity, expiration) &&
		    !requeue)
			break;
	}
}

/**
 * __bfq_deactivate_entity - deactivate an entity from its service tree.
 * @entity: the entity to deactivate.
 * @ins_into_idle_tree: if false, the entity will not be put into the
 *			idle tree.
 *
 * Deactivates an entity, independently of its previous state.  Must
 * be invoked only if entity is on a service tree. Extracts the entity
 * from that tree, and if necessary and allowed, puts it into the idle
 * tree.
 */
bool __bfq_deactivate_entity(struct bfq_entity *entity, bool ins_into_idle_tree)
{
	struct bfq_sched_data *sd = entity->sched_data;
	struct bfq_service_tree *st;
	bool is_in_service;

	if (!entity->on_st_or_in_serv) /*
					* entity never activated, or
					* already inactive
					*/
		return false;

	/*
	 * If we get here, then entity is active, which implies that
	 * bfq_group_set_parent has already been invoked for the group
	 * represented by entity. Therefore, the field
	 * entity->sched_data has been set, and we can safely use it.
	 */
	st = bfq_entity_service_tree(entity);
	is_in_service = entity == sd->in_service_entity;

 	bfq_calc_finish(entity, entity->service);

        if(open_bfqq_printk)
	   printk("1:%s %d %s %d entity:%llx entity->service:%d ins_into_idle_tree:%d sd->in_service_entity:%llx st:%llx\n",__func__,__LINE__,current->comm,current->pid,(u64)entity,entity->service,ins_into_idle_tree,(u64)sd->in_service_entity,(u64)st);

	if (is_in_service)
		sd->in_service_entity = NULL;
	else
		/*
		 * Non in-service entity: nobody will take care of
		 * resetting its service counter on expiration. Do it
		 * now.
		 */
		entity->service = 0;

	if (entity->tree == &st->active){
		if(open_bfqq_printk)
			  printk("2:%s %d %s %d &st->active:%llx ->bfq_active_extract()\n",__func__,__LINE__,current->comm,current->pid,(u64)(&st->active));
		bfq_active_extract(st, entity);
	}
	else if (!is_in_service && entity->tree == &st->idle){
		if(open_bfqq_printk)
			  printk("3:%s %d %s %d &st->idle:%llx ->bfq_idle_extract()\n",__func__,__LINE__,current->comm,current->pid,(u64)(&st->idle));
		bfq_idle_extract(st, entity);
	}

	if(open_bfqq_printk)
	    printk("4:%s %d %s %d entity->finish:%llu st->vtime:%llu\n",__func__,__LINE__,current->comm,current->pid,entity->finish,st->vtime);

	if (!ins_into_idle_tree || !bfq_gt(entity->finish, st->vtime)){
		if(open_bfqq_printk)
			  printk("5:%s %d %s %d ->bfq_forget_entity()\n",__func__,__LINE__,current->comm,current->pid);
		bfq_forget_entity(st, entity, is_in_service);
	}
	else{
		if(open_bfqq_printk)
			  printk("6:%s %d %s %d ->bfq_idle_insert()\n",__func__,__LINE__,current->comm,current->pid);
		bfq_idle_insert(st, entity);
	}

	return true;
}

/**
 * bfq_deactivate_entity - deactivate an entity representing a bfq_queue.
 * @entity: the entity to deactivate.
 * @ins_into_idle_tree: true if the entity can be put into the idle tree
 * @expiration: true if this function is being invoked in the expiration path
 *             of the in-service queue
 */
static void bfq_deactivate_entity(struct bfq_entity *entity,
				  bool ins_into_idle_tree,
				  bool expiration)
{
	struct bfq_sched_data *sd;
	struct bfq_entity *parent = NULL;

	for_each_entity_safe(entity, parent) {
		sd = entity->sched_data;
                if(open_bfqq_printk)
	            printk("1:%s %d %s %d bfq_entity:%llx bfq_sched_data:%llx ins_into_idle_tree:%d sd->next_in_service:%llx\n",__func__,__LINE__,current->comm,current->pid,(u64)entity,(u64)sd,ins_into_idle_tree,(u64)(sd->next_in_service));
		if (!__bfq_deactivate_entity(entity, ins_into_idle_tree)) {
			/*
			 * entity is not in any tree any more, so
			 * this deactivation is a no-op, and there is
			 * nothing to change for upper-level entities
			 * (in case of expiration, this can never
			 * happen).
			 */
			return;
		}

		if (sd->next_in_service == entity){
			/*
			 * entity was the next_in_service entity,
			 * then, since entity has just been
			 * deactivated, a new one must be found.
			 */
			if(open_bfqq_printk)
				  printk("2:%s %d %s %d if (sd->next_in_service == entity)\n",__func__,__LINE__,current->comm,current->pid);
			bfq_update_next_in_service(sd, NULL, expiration);
		}

		if (sd->next_in_service || sd->in_service_entity) {
			/*
			 * The parent entity is still active, because
			 * either next_in_service or in_service_entity
			 * is not NULL. So, no further upwards
			 * deactivation must be performed.  Yet,
			 * next_in_service has changed.	Then the
			 * schedule does need to be updated upwards.
			 *
			 * NOTE If in_service_entity is not NULL, then
			 * next_in_service may happen to be NULL,
			 * although the parent entity is evidently
			 * active. This happens if 1) the entity
			 * pointed by in_service_entity is the only
			 * active entity in the parent entity, and 2)
			 * according to the definition of
			 * next_in_service, the in_service_entity
			 * cannot be considered as
			 * next_in_service. See the comments on the
			 * definition of next_in_service for details.
			 */
			if(open_bfqq_printk)
				  printk("3:%s %d %s %d if (sd->next_in_service || sd->in_service_entity)\n",__func__,__LINE__,current->comm,current->pid);
			break;
		}

		/*
		 * If we get here, then the parent is no more
		 * backlogged and we need to propagate the
		 * deactivation upwards. Thus let the loop go on.
		 */

		/*
		 * Also let parent be queued into the idle tree on
		 * deactivation, to preserve service guarantees, and
		 * assuming that who invoked this function does not
		 * need parent entities too to be removed completely.
		 */
		ins_into_idle_tree = true;
	}

	if(open_bfqq_printk)
	    printk("4:%s %d %s %d\n",__func__,__LINE__,current->comm,current->pid);
	/*
	 * If the deactivation loop is fully executed, then there are
	 * no more entities to touch and next loop is not executed at
	 * all. Otherwise, requeue remaining entities if they are
	 * about to stop receiving service, or reposition them if this
	 * is not the case.
	 */
	entity = parent;
	for_each_entity(entity) {
		/*
		 * Invoke __bfq_requeue_entity on entity, even if
		 * already active, to requeue/reposition it in the
		 * active tree (because sd->next_in_service has
		 * changed)
		 */
		if(open_bfqq_printk)
		    printk("5:%s %d %s %d entity:%llx ->__bfq_requeue_entity()\n",__func__,__LINE__,current->comm,current->pid,(u64)entity);
		__bfq_requeue_entity(entity);

		sd = entity->sched_data;

		if(open_bfqq_printk)
		    printk("6:%s %d %s %d entity:%llx expiration:%d ->bfq_update_next_in_service()\n",__func__,__LINE__,current->comm,current->pid,(u64)entity,expiration);
		if (!bfq_update_next_in_service(sd, entity, expiration) &&
		    !expiration)
			/*
			 * next_in_service unchanged or not causing
			 * any change in entity->parent->sd, and no
			 * requeueing needed for expiration: stop
			 * here.
			 */
			break;
	}
}

/**
 * bfq_calc_vtime_jump - compute the value to which the vtime should jump,
 *                       if needed, to have at least one entity eligible.
 * @st: the service tree to act upon.
 *
 * Assumes that st is not empty.
 */
static u64 bfq_calc_vtime_jump(struct bfq_service_tree *st)
{
	struct bfq_entity *root_entity = bfq_root_active_entity(&st->active);

        if(open_bfqq_printk)
	    printk("1:%s %d %s %d root_entity:%llx root_entity->min_start:%lld st->vtime:%lld\n",__func__,__LINE__,current->comm,current->pid,(u64)root_entity,root_entity->min_start,st->vtime);

	if (bfq_gt(root_entity->min_start, st->vtime))
		return root_entity->min_start;

	return st->vtime;
}

static void bfq_update_vtime(struct bfq_service_tree *st, u64 new_value)
{
        if(open_bfqq_printk)
	    printk("1:%s %d %s %d st:%llx st->vtime:%lld new_value:%lld\n",__func__,__LINE__,current->comm,current->pid,(u64)st,st->vtime,new_value);
	if (new_value > st->vtime) {
		st->vtime = new_value;
		
		if(open_bfqq_printk)
	            printk("2:%s %d %s %d st:%llx st->vtime:%lld\n",__func__,__LINE__,current->comm,current->pid,(u64)st,st->vtime);

		bfq_forget_idle(st);
	}
}

/**
 * bfq_first_active_entity - find the eligible entity with
 *                           the smallest finish time
 * @st: the service tree to select from.
 * @vtime: the system virtual to use as a reference for eligibility
 *
 * This function searches the first schedulable entity, starting from the
 * root of the tree and going on the left every time on this side there is
 * a subtree with at least one eligible (start <= vtime) entity. The path on
 * the right is followed only if a) the left subtree contains no eligible
 * entities and b) no eligible entity has been found yet.
 */
static struct bfq_entity *bfq_first_active_entity(struct bfq_service_tree *st,
						  u64 vtime)
{
	struct bfq_entity *entry, *first = NULL;
	struct rb_node *node = st->active.rb_node;

	while (node) {
		entry = rb_entry(node, struct bfq_entity, rb_node);
left:
		if(open_bfqq_printk)
			    printk("1:%s %d %s %d entry:%llx entry->start:%lld entry->finish:%lld st->vtime:%lld  vtime:%lld pid:%d\n",__func__,__LINE__,current->comm,current->pid,(u64)entry,entry->start,entry->finish,st->vtime,vtime,bfq_entity_to_bfqq(entry)->pid);
		if (!bfq_gt(entry->start, vtime))
			first = entry;

		if (node->rb_left) {
			entry = rb_entry(node->rb_left,
					 struct bfq_entity, rb_node);
		        if(open_bfqq_printk)
			    printk("2:%s %d %s %d entry:%llx entry->min_start:%lld entry->start:%lld entry->finish:%lld vtime:%lld pid:%d\n",__func__,__LINE__,current->comm,current->pid,(u64)entry,entry->min_start,entry->start,entry->finish,vtime,bfq_entity_to_bfqq(entry)->pid);
			if (!bfq_gt(entry->min_start, vtime)) {
				node = node->rb_left;
				goto left;
			}
		}
		if (first)
			break;
		node = node->rb_right;
	}

	return first;
}

/**
 * __bfq_lookup_next_entity - return the first eligible entity in @st.
 * @st: the service tree.
 *
 * If there is no in-service entity for the sched_data st belongs to,
 * then return the entity that will be set in service if:
 * 1) the parent entity this st belongs to is set in service;
 * 2) no entity belonging to such parent entity undergoes a state change
 * that would influence the timestamps of the entity (e.g., becomes idle,
 * becomes backlogged, changes its budget, ...).
 *
 * In this first case, update the virtual time in @st too (see the
 * comments on this update inside the function).
 *
 * In constrast, if there is an in-service entity, then return the
 * entity that would be set in service if not only the above
 * conditions, but also the next one held true: the currently
 * in-service entity, on expiration,
 * 1) gets a finish time equal to the current one, or
 * 2) is not eligible any more, or
 * 3) is idle.
 */
static struct bfq_entity *
__bfq_lookup_next_entity(struct bfq_service_tree *st, bool in_service)
{
	struct bfq_entity *entity;
	u64 new_vtime;

        if(open_bfqq_printk)
	    printk("1:%s %d %s %d in_service:%d st:%llx\n",__func__,__LINE__,current->comm,current->pid,in_service,(u64)st);
	if (RB_EMPTY_ROOT(&st->active))
		return NULL;

	/*
	 * Get the value of the system virtual time for which at
	 * least one entity is eligible.
	 */
	new_vtime = bfq_calc_vtime_jump(st);
        if(open_bfqq_printk)
	    printk("2:%s %d %s %d new_vtime:%lld in_service:%d st:%llx\n",__func__,__LINE__,current->comm,current->pid,new_vtime,in_service,(u64)st);
	/*
	 * If there is no in-service entity for the sched_data this
	 * active tree belongs to, then push the system virtual time
	 * up to the value that guarantees that at least one entity is
	 * eligible. If, instead, there is an in-service entity, then
	 * do not make any such update, because there is already an
	 * eligible entity, namely the in-service one (even if the
	 * entity is not on st, because it was extracted when set in
	 * service).
	 */
	if (!in_service)
		bfq_update_vtime(st, new_vtime);

	entity = bfq_first_active_entity(st, new_vtime);
        if(open_bfqq_printk)
	    printk("3:%s %d %s %d bfq_service_tree:%llx entity:%llx\n",__func__,__LINE__,current->comm,current->pid,(u64)st,(u64)entity);
	return entity;
}

/**
 * bfq_lookup_next_entity - return the first eligible entity in @sd.
 * @sd: the sched_data.
 * @expiration: true if we are on the expiration path of the in-service queue
 *
 * This function is invoked when there has been a change in the trees
 * for sd, and we need to know what is the new next entity to serve
 * after this change.
 */
static struct bfq_entity *bfq_lookup_next_entity(struct bfq_sched_data *sd,
						 bool expiration)
{
	struct bfq_service_tree *st = sd->service_tree;
	struct bfq_service_tree *idle_class_st = st + (BFQ_IOPRIO_CLASSES - 1);
	struct bfq_entity *entity = NULL;
	int class_idx = 0;

        /*****************************************************************/
	struct bfq_queue *bfqq = bfq_entity_to_bfqq(sd->next_in_service);
	struct bfq_data *bfqd = bfqq->bfqd;

	//high prio io的bfqq在加入st->active tree后。high_prio_io_schedule_deadline时间到了，必须立即得到调度派发rq。我这里相当于没
	//执行bfq_lookup_next_entity正常的查找next entity的流程，这样可能会影响bfq调度算法，比如下边的sd->bfq_class_idle_last_service没更新????????????需要后续研究?????????
	
	//不用遍历链表，只有看链表头第一个成员是否超时，第一个没超时，后边的更不会超时
	//list_for_each_entry(bfqq, &bfqd->deadline_head,deadline_list)
	if(!list_empty(&bfqd->deadline_head)){
	    bfqq = list_first_entry(&bfqd->deadline_head, struct bfq_queue,deadline_list);
	    if(time_is_before_jiffies(bfqq->high_prio_io_active_time + bfqd->high_prio_io_schedule_deadline)){
	        entity = &bfqq->entity;
		//要把该bfqq和entity从st->active tree剔除掉，不能在这里bfq_active_extract()从active tree剔除。因为这个entity接下来就是sd->next_in_service
		//在bfq_select_queue->bfq_set_in_service_queue->bfq_get_next_queue函数流程，把该entity赋于sd->in_service_entity时，本身就会执行bfq_active_extract()
                //bfq_active_extract(bfq_entity_service_tree(entity),entity);

		//该bfqq要从 deadline_head 链表剔除
	        list_del(&bfqq->deadline_list);
	        printk("%s %s %d bfqq:%llx bfqq->pid:%d high prio io get schedue active:%ld jifies:%ld************\n",__func__,current->comm,current->pid,(u64)bfqq,bfqq->pid,bfqq->high_prio_io_active_time,jiffies);
		return entity;
	    }
	}

        if(open_bfqq_printk)
	    printk("1:%s %d %s %d\n",__func__,__LINE__,current->comm,current->pid);
	/*
	 * Choose from idle class, if needed to guarantee a minimum
	 * bandwidth to this class (and if there is some active entity
	 * in idle class). This should also mitigate
	 * priority-inversion problems in case a low priority task is
	 * holding file system resources.
	 */
	if (time_is_before_jiffies(sd->bfq_class_idle_last_service +
				   BFQ_CL_IDLE_TIMEOUT)) {
		if (!RB_EMPTY_ROOT(&idle_class_st->active))
			class_idx = BFQ_IOPRIO_CLASSES - 1;
		/* About to be served if backlogged, or not yet backlogged */
		sd->bfq_class_idle_last_service = jiffies;

                if(open_bfqq_printk)
	            printk("2:%s %d %s %d if (time_is_before_jiffies(sd->bfq_class_idle_last_service......\n",__func__,__LINE__,current->comm,current->pid);
	}

	/*
	 * Find the next entity to serve for the highest-priority
	 * class, unless the idle class needs to be served.
	 */
	for (; class_idx < BFQ_IOPRIO_CLASSES; class_idx++) {
		/*
		 * If expiration is true, then bfq_lookup_next_entity
		 * is being invoked as a part of the expiration path
		 * of the in-service queue. In this case, even if
		 * sd->in_service_entity is not NULL,
		 * sd->in_service_entiy at this point is actually not
		 * in service any more, and, if needed, has already
		 * been properly queued or requeued into the right
		 * tree. The reason why sd->in_service_entity is still
		 * not NULL here, even if expiration is true, is that
		 * sd->in_service_entiy is reset as a last step in the
		 * expiration path. So, if expiration is true, tell
		 * __bfq_lookup_next_entity that there is no
		 * sd->in_service_entity.
		 */
                if(open_bfqq_printk)
	            printk("3:%s %d %s %d ->__bfq_lookup_next_entity()\n",__func__,__LINE__,current->comm,current->pid);
		entity = __bfq_lookup_next_entity(st + class_idx,
						  sd->in_service_entity &&
						  !expiration);

		if (entity)
			break;
	}

        if(open_bfqq_printk)
	    printk("3_1:%s %d %s %d entity:%llx\n",__func__,__LINE__,current->comm,current->pid,(u64)entity);

	if (!entity)
		return NULL;
	/*从deadline_head链表剔除bfqq的代码不能放在这里，因为 bfq_activate_requeue_entity->bfq_update_next_in_service() ，把bfqq加入st->active tree后，在bfq_update_next_in_service()
	 *函数直接返回这个bfqq的entity，不会再执行 bfq_update_next_in_service()->bfq_lookup_next_entity()，就不会执行这里的代码，把bfqq从deadline_head链表剔除了。然后这个bfqq的entity就
	 作为sd->in_service_entity，然后bfqq从st->active tree和bfqd->active_list剔除。。后续再执行bfq_lookup_next_entity查找next entity时，就会在bfq_lookup_next_entity()函数开头，
	 从deadline_head链表返回这个bfqq的entity。作为sd->in_serv_entity这就出问题了，因为这个bfqq的entity可能已经从st->active tree和 bfqd->active_list剔除了。此时会再次把bfqq从
	 bfqd->active_list剔除，double 剔除，内核crash*/
	/*
	bfqq = bfq_entity_to_bfqq(entity);
	//到这里，如果bfqq已经添加到了 deadline_head 链表则要剔除掉，因为说明该bfqq在截至时间到来时就已经得到派发机会了
        if(bfqq->deadline_list.next != LIST_POISON1 && bfqq->deadline_list.prev != LIST_POISON2){
	    printk("%s %s %d bfqq:%llx bfqq->pid:%d list_del(&bfqq->deadline_list)************\n",__func__,current->comm,current->pid,(u64)bfqq,bfqq->pid);
	    list_del(&bfqq->deadline_list);
	}
        */
	return entity;
}

bool next_queue_may_preempt(struct bfq_data *bfqd)
{
	struct bfq_sched_data *sd = &bfqd->root_group->sched_data;

	return sd->next_in_service != sd->in_service_entity;
}

/*
 * Get next queue for service.
 */
struct bfq_queue *bfq_get_next_queue(struct bfq_data *bfqd)
{
	struct bfq_entity *entity = NULL;
	struct bfq_sched_data *sd;
	struct bfq_queue *bfqq;

	if(open_bfqq_printk)
            printk("1:%s %d %s %d bfq_tot_busy_queues(bfqd):%d\n",__func__,__LINE__,current->comm,current->pid,bfq_tot_busy_queues(bfqd));
	if (bfq_tot_busy_queues(bfqd) == 0)
		return NULL;

	/*
	 * Traverse the path from the root to the leaf entity to
	 * serve. Set in service all the entities visited along the
	 * way.
	 */
	sd = &bfqd->root_group->sched_data;
	if(open_bfqq_printk)
            printk("2:%s %d %s %d bfq_sched_data:%llx\n",__func__,__LINE__,current->comm,current->pid,(u64)sd);

	for (; sd ; sd = entity->my_sched_data) {
		/*
		 * WARNING. We are about to set the in-service entity
		 * to sd->next_in_service, i.e., to the (cached) value
		 * returned by bfq_lookup_next_entity(sd) the last
		 * time it was invoked, i.e., the last time when the
		 * service order in sd changed as a consequence of the
		 * activation or deactivation of an entity. In this
		 * respect, if we execute bfq_lookup_next_entity(sd)
		 * in this very moment, it may, although with low
		 * probability, yield a different entity than that
		 * pointed to by sd->next_in_service. This rare event
		 * happens in case there was no CLASS_IDLE entity to
		 * serve for sd when bfq_lookup_next_entity(sd) was
		 * invoked for the last time, while there is now one
		 * such entity.
		 *
		 * If the above event happens, then the scheduling of
		 * such entity in CLASS_IDLE is postponed until the
		 * service of the sd->next_in_service entity
		 * finishes. In fact, when the latter is expired,
		 * bfq_lookup_next_entity(sd) gets called again,
		 * exactly to update sd->next_in_service.
		 */

		/* Make next_in_service entity become in_service_entity */
		if(open_bfqq_printk)
	            printk("3:%s %d %s %d bfq_sched_data:%llx sd->next_in_service:%llx sd->in_service_entity:%llx\n",__func__,__LINE__,current->comm,current->pid,(u64)sd,(u64)(sd->next_in_service),(u64)(sd->in_service_entity));
		entity = sd->next_in_service;
		sd->in_service_entity = entity;

		/*
		 * If entity is no longer a candidate for next
		 * service, then it must be extracted from its active
		 * tree, so as to make sure that it won't be
		 * considered when computing next_in_service. See the
		 * comments on the function
		 * bfq_no_longer_next_in_service() for details.
		 */
		if (bfq_no_longer_next_in_service(entity)){
			if(open_bfqq_printk)
			    printk("4:%s %d %s %d if (bfq_no_longer_next_in_service(entity))\n",__func__,__LINE__,current->comm,current->pid);
			bfq_active_extract(bfq_entity_service_tree(entity),
					   entity);
		}

		/*
		 * Even if entity is not to be extracted according to
		 * the above check, a descendant entity may get
		 * extracted in one of the next iterations of this
		 * loop. Such an event could cause a change in
		 * next_in_service for the level of the descendant
		 * entity, and thus possibly back to this level.
		 *
		 * However, we cannot perform the resulting needed
		 * update of next_in_service for this level before the
		 * end of the whole loop, because, to know which is
		 * the correct next-to-serve candidate entity for each
		 * level, we need first to find the leaf entity to set
		 * in service. In fact, only after we know which is
		 * the next-to-serve leaf entity, we can discover
		 * whether the parent entity of the leaf entity
		 * becomes the next-to-serve, and so on.
		 */
	}

	bfqq = bfq_entity_to_bfqq(entity);
        if(open_bfqq_printk)
	    printk("5:%s %d %s %d entity:%llx bfqq:%llx\n",__func__,__LINE__,current->comm,current->pid,(u64)entity,(u64)bfqq);
	/*
	 * We can finally update all next-to-serve entities along the
	 * path from the leaf entity just set in service to the root.
	 */
	for_each_entity(entity) {
		struct bfq_sched_data *sd = entity->sched_data;
                if(open_bfqq_printk)
		    printk("6:%s %d %s %d entity:%llx bfq_sched_data:%llx\n",__func__,__LINE__,current->comm,current->pid,(u64)entity,(u64)sd);
		if (!bfq_update_next_in_service(sd, NULL, false))
			break;
	}

	return bfqq;
}

/* returns true if the in-service queue gets freed */
bool __bfq_bfqd_reset_in_service(struct bfq_data *bfqd)
{
	struct bfq_queue *in_serv_bfqq = bfqd->in_service_queue;
	struct bfq_entity *in_serv_entity = &in_serv_bfqq->entity;
	struct bfq_entity *entity = in_serv_entity;

	bfq_clear_bfqq_wait_request(in_serv_bfqq);
	hrtimer_try_to_cancel(&bfqd->idle_slice_timer);
	bfqd->in_service_queue = NULL;
       if(open_bfqq_printk)
	    printk("1:%s %d %s %d bfqd->in_service_queue:%llx in_serv_entity:%llx\n",__func__,__LINE__,current->comm,current->pid,(u64)(bfqd->in_service_queue),(u64)(in_serv_entity));
	/*
	 * When this function is called, all in-service entities have
	 * been properly deactivated or requeued, so we can safely
	 * execute the final step: reset in_service_entity along the
	 * path from entity to the root.
	 */
	for_each_entity(entity)
		entity->sched_data->in_service_entity = NULL;

	/*
	 * in_serv_entity is no longer in service, so, if it is in no
	 * service tree either, then release the service reference to
	 * the queue it represents (taken with bfq_get_entity).
	 */
	if (!in_serv_entity->on_st_or_in_serv) {
		/*
		 * If no process is referencing in_serv_bfqq any
		 * longer, then the service reference may be the only
		 * reference to the queue. If this is the case, then
		 * bfqq gets freed here.
		 */
		int ref = in_serv_bfqq->ref;
		bfq_put_queue(in_serv_bfqq);
		if (ref == 1)
			return true;
	}

	return false;
}

void bfq_deactivate_bfqq(struct bfq_data *bfqd, struct bfq_queue *bfqq,
			 bool ins_into_idle_tree, bool expiration)
{
	struct bfq_entity *entity = &bfqq->entity;

	bfq_deactivate_entity(entity, ins_into_idle_tree, expiration);
}

void bfq_activate_bfqq(struct bfq_data *bfqd, struct bfq_queue *bfqq)
{
	struct bfq_entity *entity = &bfqq->entity;

	bfq_activate_requeue_entity(entity, bfq_bfqq_non_blocking_wait_rq(bfqq),
				    false, false);
	bfq_clear_bfqq_non_blocking_wait_rq(bfqq);
}

void bfq_requeue_bfqq(struct bfq_data *bfqd, struct bfq_queue *bfqq,
		      bool expiration)
{
	struct bfq_entity *entity = &bfqq->entity;

	bfq_activate_requeue_entity(entity, false,
				    bfqq == bfqd->in_service_queue, expiration);
}

/*
 * Called when the bfqq no longer has requests pending, remove it from
 * the service tree. As a special case, it can be invoked during an
 * expiration.
 */
void bfq_del_bfqq_busy(struct bfq_data *bfqd, struct bfq_queue *bfqq,
		       bool expiration)
{
	bfq_log_bfqq(bfqd, bfqq, "del from busy");

	bfq_clear_bfqq_busy(bfqq);

	bfqd->busy_queues[bfqq->ioprio_class - 1]--;

	if (bfqq->wr_coeff > 1)
		bfqd->wr_busy_queues--;

        if(open_bfqq_printk1 && vim_pid == bfqq->pid){
	    printk("%s %d %s %d bfqq:%llx bfqq->pid:%d expiration:%d bfqq->wr_coeff:%d\n",__func__,__LINE__,current->comm,current->pid,(u64)bfqq,bfqq->pid,expiration,bfqq->wr_coeff);
	}
	bfqg_stats_update_dequeue(bfqq_group(bfqq));

	bfq_deactivate_bfqq(bfqd, bfqq, true, expiration);

	if (!bfqq->dispatched)
		bfq_weights_tree_remove(bfqd, bfqq);
}

/*
 * Called when an inactive queue receives a new request.
 */
void bfq_add_bfqq_busy(struct bfq_data *bfqd, struct bfq_queue *bfqq)
{
	bfq_log_bfqq(bfqd, bfqq, "add to busy");

        if(open_bfqq_printk1 && strcmp("cat",current->comm) == 0 && vim_pid == -2){
		vim_pid = current->pid;
	        printk("%s %d %s %d bfqq:%llx bfqq->pid:%d ->bfq_activate_bfqq()\n",__func__,__LINE__,current->comm,current->pid,(u64)bfqq,bfqq->pid);
        }
	bfq_activate_bfqq(bfqd, bfqq);

	bfq_mark_bfqq_busy(bfqq);
	bfqd->busy_queues[bfqq->ioprio_class - 1]++;

	if (!bfqq->dispatched)
		if (bfqq->wr_coeff == 1)
			bfq_weights_tree_add(bfqd, bfqq,
					     &bfqd->queue_weights_tree);

	if (bfqq->wr_coeff > 1)
		bfqd->wr_busy_queues++;
}
