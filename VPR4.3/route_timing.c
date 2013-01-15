#include <stdio.h>
#include <math.h>
#include "util.h"
#include "vpr_types.h"
#include "globals.h"
#include "route_export.h"
#include "route_common.h"
#include "route_tree_timing.h"
#include "route_timing.h"
#include "heapsort.h"
#include "path_delay.h"
#include "net_delay.h"

/******************** Subroutines local to route_timing.c ********************/

static int get_max_pins_per_net(void);

static void add_route_tree_to_heap(t_rt_node *rt_node, int target_node,
		float target_criticality, float astar_fac);

static void timing_driven_expand_neighbours(struct s_heap *current, int inet,
		float bend_cost, float criticality_fac, int target_node,
		float astar_fac);

static float get_timing_driven_expected_cost(int inode, int target_node,
		float criticality_fac, float R_upstream);

static int get_expected_segs_to_target(int inode, int target_node,
		int * num_segs_ortho_dir_ptr);

static void update_rr_base_costs(int inet, float largest_criticality);
//added by Wei

static void adjust_pin_pos(int inet);
static boolean exchange(int blk, int pin, int tpos, int net, int net_pin);
/************************ Subroutine definitions *****************************/

boolean exchange(int blk, int pin, int tpos, int inet, int net_pin) {
	printf("process pin %d at %s for tpos %d\n", pin, block[blk].name, tpos);
	int tar_pin = -1, ipin;
	int i, j, iclass, pinpos;
	i = block[blk].x;
	j = block[blk].y;
	if (clb[i][j].type != CLB)
		return TRUE;
	if (pinloc[LEFT][pin] == 1)
		pinpos = LEFT;
	else if (pinloc[TOP][pin] == 1)
		pinpos = TOP;
	else if (pinloc[RIGHT][pin] == 1)
		pinpos = RIGHT;
	else if (pinloc[BOTTOM][pin] == 1)
		pinpos = BOTTOM;
	else
		printf("error: not find position.\n");

	if (pinpos == tpos)
		return TRUE;

	iclass = clb_pin_class[pin];
	for (ipin = 0; ipin < pins_per_clb; ipin++) {
		if ((pinloc[tpos][ipin] == 1) && (clb_pin_class[ipin] == iclass))
			if (block[blk].nets[ipin] == OPEN) {
				tar_pin = ipin;
				printf("tar_pin is %d\n", tar_pin);
				break;
			}
	}
	if (tar_pin != -1) {
		block[blk].nets[tar_pin] = inet;
		block[blk].nets[pin] = OPEN;
		net[inet].blk_pin[net_pin] = tar_pin;
		return TRUE;
	} else
		return FALSE;
}

void adjust_pin_pos(int inet) {
	int ipin, dblk, iblk, x, y, x1, y1, blk_pin, pinpos = -1, to_pin;
	int diffx = 0, diffy = 0, tar_pos = -1;
	boolean succ;
	dblk = net[inet].blocks[0];
	x = block[dblk].x;
	y = block[dblk].y;
	blk_pin = net[inet].blk_pin[0];
	if (clb[x][y].type == CLB || clb[x][y].type == MEM) {
		if (pinloc[LEFT][blk_pin] == 1)
			pinpos = LEFT;
		else if (pinloc[TOP][blk_pin] == 1)
			pinpos = TOP;
		else if (pinloc[RIGHT][blk_pin] == 1)
			pinpos = RIGHT;
		else if (pinloc[BOTTOM][blk_pin] == 1)
			pinpos = BOTTOM;
		else
			printf("error: not find position.\n");
	}

	printf("for net: %d stage: %d the pin is %d at block %s on side %d\n", inet,
			net[inet].stage, blk_pin, block[dblk].name, pinpos);

	for (ipin = 1; ipin < net[inet].num_pins; ipin++) {
		iblk = net[inet].blocks[ipin];
		x1 = block[iblk].x;
		y1 = block[iblk].y;
		diffx += x1 - x;
		diffy += y1 - y;
	}
	printf("total diffx %d, total diffy %d\n", diffx, diffy);

	if (clb[x][y].type == CLB) {
		if (abs(diffy) >= abs(diffx)) {
			if (diffy > 0) {
				tar_pos = TOP;
				if (pinpos != TOP) {
					succ = exchange(dblk, blk_pin, TOP, inet, 0);
					printf("try top\n");
				}
			}
			if (diffy < 0) {
				tar_pos = BOTTOM;
				if (pinpos != BOTTOM) {
					succ = exchange(dblk, blk_pin, BOTTOM, inet, 0);
					printf("try bottom\n");
				}
			}
			if (!succ) {
				if (diffx > 0) {
					printf("try right\n");
					succ = exchange(dblk, blk_pin, RIGHT, inet, 0);
				}
				if (diffx < 0) {
					printf("try left\n");
					succ = exchange(dblk, blk_pin, LEFT, inet, 0);
				}
			}
		}
		if (abs(diffx) > abs(diffy)) {
			if (diffx > 0) {
				tar_pos = RIGHT;
				if (pinpos != RIGHT) {
					succ = exchange(dblk, blk_pin, RIGHT, inet, 0);
					printf("try right\n");
				}
			}
			if (diffx < 0) {
				tar_pos = LEFT;
				if (pinpos != LEFT) {
					succ = exchange(dblk, blk_pin, LEFT, inet, 0);
					printf("try left\n");
				}
			}
			if (!succ) {
				if (diffy > 0) {
					succ = exchange(dblk, blk_pin, TOP, inet, 0);
					printf("try top\n");
				}
				if (diffy < 0) {
					succ = exchange(dblk, blk_pin, BOTTOM, inet, 0);
					printf("try bottom\n");
				}
			}
		}
	}
	printf("new pos %d\n", tar_pos);
	switch (tar_pos) {
	case TOP:
		printf("come here top\n");
		for (ipin = 1; ipin < net[inet].num_pins; ipin++) {
			iblk = net[inet].blocks[ipin];
			to_pin = net[inet].blk_pin[ipin];
			x1 = block[iblk].x;
			y1 = block[iblk].y;
			succ = exchange(iblk, to_pin, BOTTOM, inet, ipin);
			if (!succ) {
				if (x1 > x)
					exchange(iblk, to_pin, LEFT, inet, ipin);
				if (x1 < x)
					exchange(iblk, to_pin, RIGHT, inet, ipin);
			}
		}
		break;
	case BOTTOM:
		printf("come here bottom\n");
		for (ipin = 1; ipin < net[inet].num_pins; ipin++) {
			iblk = net[inet].blocks[ipin];
			to_pin = net[inet].blk_pin[ipin];
			x1 = block[iblk].x;
			y1 = block[iblk].y;
			succ = exchange(iblk, to_pin, TOP, inet, ipin);
			if (!succ) {
				if (x1 > x)
					exchange(iblk, to_pin, LEFT, inet, ipin);
				if (x1 < x)
					exchange(iblk, to_pin, RIGHT, inet, ipin);
			}
		}
		break;
	case RIGHT:
		printf("come here right\n");
		for (ipin = 1; ipin < net[inet].num_pins; ipin++) {
			iblk = net[inet].blocks[ipin];
			to_pin = net[inet].blk_pin[ipin];
			x1 = block[iblk].x;
			y1 = block[iblk].y;
			succ = exchange(iblk, to_pin, LEFT, inet, ipin);
			if (!succ) {
				if (y1 > y)
					exchange(iblk, to_pin, BOTTOM, inet, ipin);
				if (y1 < y)
					exchange(iblk, to_pin, TOP, inet, ipin);
			}
		}
		break;
	case LEFT:
		printf("come here left\n");
		for (ipin = 1; ipin < net[inet].num_pins; ipin++) {
			iblk = net[inet].blocks[ipin];
			to_pin = net[inet].blk_pin[ipin];
			x1 = block[iblk].x;
			y1 = block[iblk].y;
			succ = exchange(iblk, to_pin, RIGHT, inet, ipin);
			if (!succ) {
				if (y1 > y)
					exchange(iblk, to_pin, BOTTOM, inet, ipin);
				if (y1 < y)
					exchange(iblk, to_pin, TOP, inet, ipin);
			}
		}
		break;
	default:
		return;
	}
}

boolean try_timing_driven_route(struct s_router_opts router_opts,
		float **net_slack, float **net_delay, t_ivec **clb_opins_used_locally) {

	/* Timing-driven routing algorithm.  The timing graph (includes net_slack)   *
	 * must have already been allocated, and net_delay must have been allocated. *
	 * Returns TRUE if the routing succeeds, FALSE otherwise.                    */

	int itry, inet, ipin;
	boolean success, is_routable, rip_up_local_opins;
	float *pin_criticality; /* [1..max_pins_per_net-1]. */
	int *sink_order; /* [1..max_pins_per_net-1]. */
	t_rt_node **rt_node_of_sink; /* [1..max_pins_per_net-1]. */
	float T_crit, pres_fac;
	//printf("new route\n");
	alloc_timing_driven_route_structs(&pin_criticality, &sink_order,
			&rt_node_of_sink);

	/* First do one routing iteration ignoring congestion and marking all sinks  *
	 * on each net as critical to get reasonable net delay estimates.            */

	for (inet = 0; inet < num_nets; inet++) {
		if (is_folding) {
			if (current_stage != num_stage) {
				if ((inet < num_net_per_stage[current_stage - 1].begin)
						|| (inet >= num_net_per_stage[current_stage].begin))
					continue;
			} else {
				if (inet < num_net_per_stage[current_stage - 1].begin)
					continue;
			}
		}
		//printf("routing net %d\n",inet);
		if (is_global[inet] == FALSE) {
			for (ipin = 1; ipin < net[inet].num_pins; ipin++)
				net_slack[inet][ipin] = 0.;
		} else { /* Set delay of global signals to zero. */
			for (ipin = 1; ipin < net[inet].num_pins; ipin++)
				net_delay[inet][ipin] = 0.;
		}
	}

	T_crit = 1.;
	pres_fac = router_opts.first_iter_pres_fac; /* Typically 0 -> ignore cong. */

	for (itry = 1; itry <= router_opts.max_router_iterations; itry++) {
		//printf("the number of try %d\n", itry);
		for (inet = 0; inet < num_nets; inet++) {
			if (is_folding) {
				if (current_stage != num_stage) {
					if ((inet < num_net_per_stage[current_stage - 1].begin)
							|| (inet >= num_net_per_stage[current_stage].begin))
						continue;
				} else {
					if (inet < num_net_per_stage[current_stage - 1].begin)
						continue;
				}
			}
			// printf("the net is %d (%s)\n",inet, net[inet].name);
			if (is_global[inet] == FALSE) { /* Skip global nets. */
				/* if(!net[inet].adjusted)
				 {net[inet].adjusted = TRUE;
				 adjust_pin_pos(inet);} */
				is_routable = timing_driven_route_net(inet, pres_fac,
						router_opts.max_criticality,
						router_opts.criticality_exp, router_opts.astar_fac,
						router_opts.bend_cost, net_slack[inet], pin_criticality,
						sink_order, rt_node_of_sink, T_crit, net_delay[inet]);

				/* Impossible to route? (disconnected rr_graph) */

				if (!is_routable) {
					printf("Routing failed.\n");
					free_timing_driven_route_structs(pin_criticality,
							sink_order, rt_node_of_sink);
					//  printf("come here\n");
					return (FALSE);
				}
			}
			// else
			// printf("the global net %d(%s) in stage %d\n", inet, net[inet].name, net[inet].stage);
		}

		/* Make sure any CLB OPINs used up by subblocks being hooked directly     *
		 * to them are reserved for that purpose.                                 */

		if (itry == 1)
			rip_up_local_opins = FALSE;
		else
			rip_up_local_opins = TRUE;

		reserve_locally_used_opins(pres_fac, rip_up_local_opins,
				clb_opins_used_locally);

		/* Pathfinder guys quit after finding a feasible route. I may want to keep *
		 * going longer, trying to improve timing.  Think about this some.         */

		success = feasible_routing();
		if (success) {
			printf("Successfully routed after %d routing iterations.\n", itry);
			free_timing_driven_route_structs(pin_criticality, sink_order,
					rt_node_of_sink);
			return (TRUE);
		}

		if (itry == 1) {
			pres_fac = router_opts.initial_pres_fac;
			pathfinder_update_cost(pres_fac, 0.); /* Acc_fac=0 for first iter. */
		} else {
			pres_fac *= router_opts.pres_fac_mult;
			pathfinder_update_cost(pres_fac, router_opts.acc_fac);
		}

		/* Update slack values by doing another timing analysis.                 *
		 * Timing_driven_route_net updated the net delay values.                 */

		load_timing_graph_net_delays(net_delay);
		T_crit = load_net_slack(net_slack, 0, TRUE);

		//printf ("T_crit: %g.\n", T_crit);
		/* printf("After....\n");
		 for (inet=0;inet<num_nets;inet++) {
		 printf("for net %d (%s)\n", inet, net[inet].name);
		 for (ipin=1;ipin<net[inet].num_pins;ipin++)
		 printf("the net slack %g\n",net_slack[inet][ipin]);
		 for (ipin=1;ipin<net[inet].num_pins;ipin++)
		 printf("the net delay %g\n",net_delay[inet][ipin]);
		 }
		 */
	}

	printf("Routing failed.\n");
	free_timing_driven_route_structs(pin_criticality, sink_order,
			rt_node_of_sink);
	return (FALSE);
}

void alloc_timing_driven_route_structs(float **pin_criticality_ptr,
		int **sink_order_ptr, t_rt_node ***rt_node_of_sink_ptr) {

	/* Allocates all the structures needed only by the timing-driven router.   */

	int max_pins_per_net;
	float *pin_criticality;
	int *sink_order;
	t_rt_node **rt_node_of_sink;

	max_pins_per_net = get_max_pins_per_net();

	//printf("the max is %d\n", max_pins_per_net);

	pin_criticality = (float *) my_malloc(
			(max_pins_per_net - 1) * sizeof(float));
	*pin_criticality_ptr = pin_criticality - 1; /* First sink is pin #1. */

	sink_order = (int *) my_malloc((max_pins_per_net - 1) * sizeof(int));
	*sink_order_ptr = sink_order - 1;

	rt_node_of_sink = (t_rt_node **) my_malloc(
			(max_pins_per_net - 1) * sizeof(t_rt_node *));
	*rt_node_of_sink_ptr = rt_node_of_sink - 1;

	alloc_route_tree_timing_structs();
}

void free_timing_driven_route_structs(float *pin_criticality, int *sink_order,
		t_rt_node **rt_node_of_sink) {

	/* Frees all the stuctures needed only by the timing-driven router.        */

	free(pin_criticality + 1); /* Starts at index 1. */
	free(sink_order + 1);
	free(rt_node_of_sink + 1);
	free_route_tree_timing_structs();
}

static int get_max_pins_per_net(void) {

	/* Returns the largest number of pins on any non-global net.    */

	int inet, max_pins_per_net;

	max_pins_per_net = 0;
	for (inet = 0; inet < num_nets; inet++) {

		if (is_global[inet] == FALSE) {
			//printf("the num of pins are %d\n", net[inet].num_pins);
			max_pins_per_net = max (max_pins_per_net, net[inet].num_pins);
		}
	}
	if (max_pins_per_net == 0)
		max_pins_per_net = 2;
	return (max_pins_per_net);
}

boolean timing_driven_route_net(int inet, float pres_fac, float max_criticality,
		float criticality_exp, float astar_fac, float bend_cost,
		float *net_slack, float *pin_criticality, int *sink_order,
		t_rt_node **rt_node_of_sink, float T_crit, float *net_delay) {

	/* Returns TRUE as long is found some way to hook up this net, even if that *
	 * way resulted in overuse of resources (congestion).  If there is no way   *
	 * to route this net, even ignoring congestion, it returns FALSE.  In this  *
	 * case the rr_graph is disconnected and you can give up.                   */

	int ipin, num_sinks, itarget, target_pin, target_node, inode;
	float target_criticality, old_tcost, new_tcost, largest_criticality,
			pin_crit;
	float old_back_cost, new_back_cost;
	t_rt_node *rt_root;
	struct s_heap *current;
	struct s_trace *new_route_start_tptr;
	int i;

	/* Rip-up any old routing. */

	//printf ("Routing inet: %d(%s) at stage %d\n", inet, net[inet].name, net[inet].stage);
	pathfinder_update_one_cost(trace_head[inet], -1, pres_fac);
	//printf("come here\n");
	free_traceback(inet);

	//printf("number of sink:%d\n", ipin<net[inet].num_pins);

	for (ipin = 1; ipin < net[inet].num_pins; ipin++) { /* For all sinks */
		pin_crit = max (max_criticality - net_slack[ipin] / T_crit, 0.);
		pin_crit = pow(pin_crit, criticality_exp);
		pin_crit = min (pin_crit, max_criticality);
		pin_criticality[ipin] = pin_crit;
	}

	//for (i=0; i<num_rr_nodes; i++)
	//printf("node %d, pres_cost %g, path_cost %g, pre_node %d, flag %d\n", i, rr_node_route_inf[i].pres_cost, rr_node_route_inf[i].acc_cost, rr_node_route_inf[i].path_cost, rr_node_route_inf[i].prev_node, rr_node_route_inf[i].target_flag);
	//printf("node %d, back_cost %g\n", i, rr_node_route_inf[i].backward_path_cost);

	num_sinks = net[inet].num_pins - 1;
	heapsort(sink_order, pin_criticality, num_sinks);

	/* Update base costs according to fanout and criticality rules */

	largest_criticality = pin_criticality[sink_order[1]];
	update_rr_base_costs(inet, largest_criticality);

	mark_ends(inet); /* Only needed to check for multiply-connected SINKs */

	rt_root = init_route_tree_to_source(inet);

	for (itarget = 1; itarget <= num_sinks; itarget++) {
		target_pin = sink_order[itarget];
		target_node = net_rr_terminals[inet][target_pin];

		/*    printf ("Target #%d, pin number %d, target_node: %d.\n",
		 itarget, target_pin, target_node);  */

		target_criticality = pin_criticality[target_pin];

		add_route_tree_to_heap(rt_root, target_node, target_criticality,
				astar_fac);
		//printf("check 1\n");
		current = get_heap_head();
		//printf("check 2\n");

		if (current == NULL) { /* Infeasible routing.  No possible path for net. */
			reset_path_costs();
			free_route_tree(rt_root);
			return (FALSE);
		}

		inode = current->index;
		//printf("inode here %d\n", inode);

		while (inode != target_node) {
			old_tcost = rr_node_route_inf[inode].path_cost;
			new_tcost = current->cost;

			if (old_tcost > 0.99 * HUGE_FLOAT) /* First time touched. */
				old_back_cost = HUGE_FLOAT;
			else
				old_back_cost = rr_node_route_inf[inode].backward_path_cost;

			new_back_cost = current->backward_path_cost;

			/* I only re-expand a node if both the "known" backward cost is lower  *
			 * in the new expansion (this is necessary to prevent loops from       *
			 * forming in the routing and causing havoc) *and* the expected total  *
			 * cost to the sink is lower than the old value.  Different R_upstream *
			 * values could make a path with lower back_path_cost less desirable   *
			 * than one with higher cost.  Test whether or not I should disallow   *
			 * re-expansion based on a higher total cost.                          */

			if (old_tcost > new_tcost && old_back_cost > new_back_cost) {
				/*       if (old_tcost > new_tcost) {     */
				rr_node_route_inf[inode].prev_node = current->u.prev_node;
				rr_node_route_inf[inode].prev_edge = current->prev_edge;
				rr_node_route_inf[inode].path_cost = new_tcost;
				rr_node_route_inf[inode].backward_path_cost = new_back_cost;

				if (old_tcost > 0.99 * HUGE_FLOAT) /* First time touched. */
					add_to_mod_list(&rr_node_route_inf[inode].path_cost);

				timing_driven_expand_neighbours(current, inet, bend_cost,
						target_criticality, target_node, astar_fac);
			}

			free_heap_data(current);
			//printf("check 3\n");
			//printf("inode is %d\n", inode);
			current = get_heap_head();
			//printf("check 4\n");
			if (current == NULL) { /* Impossible routing.  No path for net. */
				reset_path_costs();
				free_route_tree(rt_root);
				return (FALSE);
			}

			inode = current->index;
			// printf("inode %d\n", inode);
		}

		/* NB:  In the code below I keep two records of the partial routing:  the   *
		 * traceback and the route_tree.  The route_tree enables fast recomputation *
		 * of the Elmore delay to each node in the partial routing.  The traceback  *
		 * lets me reuse all the routines written for breadth-first routing, which  *
		 * all take a traceback structure as input.  Before this routine exits the  *
		 * route_tree structure is destroyed; only the traceback is needed at that  *
		 * point.                                                                   */

		//printf("arrive here\n");
		rr_node_route_inf[inode].target_flag--; /* Connected to this SINK. */
		new_route_start_tptr = update_traceback(current, inet);
		rt_node_of_sink[target_pin] = update_route_tree(current);
		free_heap_data(current);
		pathfinder_update_one_cost(new_route_start_tptr, 1, pres_fac);
		empty_heap();
		reset_path_costs();
	}

	/* For later timing analysis. */

	update_net_delays_from_route_tree(net_delay, rt_node_of_sink, inet);
	free_route_tree(rt_root);
	return (TRUE);
}

static void add_route_tree_to_heap(t_rt_node *rt_node, int target_node,
		float target_criticality, float astar_fac) {

	/* Puts the entire partial routing below and including rt_node onto the heap *
	 * (except for those parts marked as not to be expanded) by calling itself   *
	 * recursively.                                                              */

	int inode;
	t_rt_node *child_node;
	t_linked_rt_edge *linked_rt_edge;
	float tot_cost, backward_path_cost, R_upstream;
	float expected_cost;

	/* Pre-order depth-first traversal */

	if (rt_node->re_expand) {
		inode = rt_node->inode;
		backward_path_cost = target_criticality * rt_node->Tdel;
		R_upstream = rt_node->R_upstream;
		expected_cost = get_timing_driven_expected_cost(inode, target_node,
				target_criticality, R_upstream);
		if (expected_cost < 0.9 * HUGE_FLOAT) {
			tot_cost = backward_path_cost + astar_fac * expected_cost;
			node_to_heap(inode, tot_cost, NO_PREVIOUS, NO_PREVIOUS,
					backward_path_cost, R_upstream);
		}
	}

	linked_rt_edge = rt_node->u.child_list;

	while (linked_rt_edge != NULL) {
		child_node = linked_rt_edge->child;
		add_route_tree_to_heap(child_node, target_node, target_criticality,
				astar_fac);
		linked_rt_edge = linked_rt_edge->next;
	}
}

static void timing_driven_expand_neighbours(struct s_heap *current, int inet,
		float bend_cost, float criticality_fac, int target_node,
		float astar_fac) {

	/* Puts all the rr_nodes adjacent to current on the heap.  rr_nodes outside *
	 * the expanded bounding box specified in route_bb are not added to the     *
	 * heap.                                                                    */

	int iconn, to_node, num_edges, inode, iswitch, target_x, target_y;
	t_rr_type from_type, to_type;
	float new_tot_cost, old_back_pcost, new_back_pcost, R_upstream;
	float new_R_upstream, Tdel;
	float expected_cost;

	inode = current->index;
	old_back_pcost = current->backward_path_cost;
	//printf("old path cost %g\n", old_back_pcost);
	R_upstream = current->R_upstream;
	num_edges = rr_node[inode].num_edges;

	target_x = rr_node[target_node].xhigh;
	target_y = rr_node[target_node].yhigh;

	for (iconn = 0; iconn < num_edges; iconn++) {
		to_node = rr_node[inode].edges[iconn];
		// printf("to node %d\n", to_node);
		if (rr_node[to_node].xhigh < route_bb[inet].xmin
				|| rr_node[to_node].xlow > route_bb[inet].xmax
				|| rr_node[to_node].yhigh < route_bb[inet].ymin
				|| rr_node[to_node].ylow > route_bb[inet].ymax)
			continue; /* Node is outside (expanded) bounding box. */

		/* Prune away IPINs that lead to blocks other than the target one.  Avoids  *
		 * the issue of how to cost them properly so they don't get expanded before *
		 * more promising routes, but makes route-throughs (via CLBs) impossible.   *
		 * Change this if you want to investigate route-throughs.                   */

		to_type = rr_node[to_node].type;
		//nionio modified
		/*if (to_type == IPIN && (rr_node[to_node].xhigh != target_x ||
		 rr_node[to_node].yhigh != target_y))
		 continue;*/

		/* new_back_pcost stores the "known" part of the cost to this node -- the   *
		 * congestion cost of all the routing resources back to the existing route  *
		 * plus the known delay of the total path back to the source.  new_tot_cost *
		 * is this "known" backward cost + an expected cost to get to the target.   */

		new_back_pcost = old_back_pcost
				+ (1. - criticality_fac) * get_rr_cong_cost(to_node);

		iswitch = rr_node[inode].switches[iconn];

		if (switch_inf[iswitch].buffered) {
			new_R_upstream = switch_inf[iswitch].R;
		} else {
			new_R_upstream = R_upstream + switch_inf[iswitch].R;
		}

		//printf("C %g R %d\n", rr_node[to_node].C, rr_node[to_node].R);
		Tdel = rr_node[to_node].C * (new_R_upstream + 0.5 * rr_node[to_node].R);
		Tdel += switch_inf[iswitch].Tdel;
		new_R_upstream += rr_node[to_node].R;
		new_back_pcost += criticality_fac * Tdel;

		if (bend_cost != 0.) {
			from_type = rr_node[inode].type;
			to_type = rr_node[to_node].type;
			if ((from_type == CHANX && to_type == CHANY)
					|| (from_type == CHANY && to_type == CHANX))
				new_back_pcost += bend_cost;
		}

		expected_cost = get_timing_driven_expected_cost(to_node, target_node,
				criticality_fac, new_R_upstream);

		if (expected_cost < 0.9 * HUGE_FLOAT) {
			new_tot_cost = new_back_pcost + astar_fac * expected_cost;
			//printf("inode %d, tnode %d, back_cost %g, cost %g\n", inode, to_node, new_back_pcost, new_tot_cost);
			node_to_heap(to_node, new_tot_cost, inode, iconn, new_back_pcost,
					new_R_upstream);
		}

	} /* End for all neighbours */
}

static float get_timing_driven_expected_cost(int inode, int target_node,
		float criticality_fac, float R_upstream) {

	/* Determines the expected cost (due to both delay and resouce cost) to reach *
	 * the target node from inode.  It doesn't include the cost of inode --       *
	 * that's already in the "known" path_cost.                                   */

	t_rr_type rr_type;
	int cost_index, ortho_cost_index, num_segs_same_dir, num_segs_ortho_dir;
	float expected_cost, cong_cost, Tdel;

	rr_type = rr_node[inode].type;
	//printf("inode %d tnode %d\n", inode, target_node);

	if (rr_type == CHANX || rr_type == CHANY || rr_type == DIREX
			|| rr_type == DIREY) {
		num_segs_same_dir = get_expected_segs_to_target(inode, target_node,
				&num_segs_ortho_dir);
		if ((rr_type == DIREX || rr_type == DIREY)
				&& (num_segs_same_dir == UNREACH))
			return (HUGE_FLOAT);

		//printf("same_dir %d ortho_dir %d\n", num_segs_same_dir, num_segs_ortho_dir);
		cost_index = rr_node[inode].cost_index;

		ortho_cost_index = rr_indexed_data[cost_index].ortho_cost_index;

		cong_cost = num_segs_same_dir * rr_indexed_data[cost_index].base_cost
				+ num_segs_ortho_dir
						* rr_indexed_data[ortho_cost_index].base_cost;
		cong_cost += rr_indexed_data[IPIN_COST_INDEX].base_cost
				+ rr_indexed_data[SINK_COST_INDEX].base_cost;

		Tdel =
				num_segs_same_dir * rr_indexed_data[cost_index].T_linear
						+ num_segs_ortho_dir
								* rr_indexed_data[ortho_cost_index].T_linear
						+ num_segs_same_dir * num_segs_same_dir
								* rr_indexed_data[cost_index].T_quadratic
						+ num_segs_ortho_dir * num_segs_ortho_dir
								* rr_indexed_data[ortho_cost_index].T_quadratic
						+ R_upstream
								* (num_segs_same_dir
										* rr_indexed_data[cost_index].C_load
										+ num_segs_ortho_dir
												* rr_indexed_data[ortho_cost_index].C_load);

		Tdel += rr_indexed_data[IPIN_COST_INDEX].T_linear;

		expected_cost = criticality_fac * Tdel
				+ (1. - criticality_fac) * cong_cost;
		// printf("cost_index %d, cong_cost %g, Tdel %g, exp_cost %g\n", cost_index, cong_cost, Tdel, expected_cost);
		return (expected_cost);
	}

	else if (rr_type == IPIN) { /* Change if you're allowing route-throughs */
		return (rr_indexed_data[SINK_COST_INDEX].base_cost);
	}

	else { /* Change this if you want to investigate route-throughs */
		return (0.);
	}
}

/* Macro used below to ensure that fractions are rounded up, but floating   *
 * point values very close to an integer are rounded to that integer.       */

#define ROUND_UP(x) (ceil (x - 0.001))

static int get_expected_segs_to_target(int inode, int target_node,
		int * num_segs_ortho_dir_ptr) {

	/* Returns the number of segments the same type as inode that will be needed *
	 * to reach target_node (not including inode) in each direction (the same    *
	 * direction (horizontal or vertical) as inode and the orthogonal direction).*/

	t_rr_type rr_type, target_type;
	int target_x, target_y, num_segs_same_dir, cost_index, ortho_cost_index;
	int no_need_to_pass_by_clb;
	float inv_length, ortho_inv_length, ylow, yhigh, xlow, xhigh;

	target_x = rr_node[target_node].xlow;
	target_y = rr_node[target_node].ylow;
	cost_index = rr_node[inode].cost_index;
	inv_length = rr_indexed_data[cost_index].inv_length;
	ortho_cost_index = rr_indexed_data[cost_index].ortho_cost_index;
	ortho_inv_length = rr_indexed_data[ortho_cost_index].inv_length;
	rr_type = rr_node[inode].type;

	if (rr_type == CHANX) {
		ylow = rr_node[inode].ylow;
		xhigh = rr_node[inode].xhigh;
		xlow = rr_node[inode].xlow;

		/* Count vertical (orthogonal to inode) segs first. */

		if (ylow > target_y) { /* Coming from a row above target? */
			*num_segs_ortho_dir_ptr = ROUND_UP ((ylow - target_y + 1.) *
					ortho_inv_length);
			no_need_to_pass_by_clb = 1;
		} else if (ylow < target_y - 1) { /* Below the CLB bottom? */
			*num_segs_ortho_dir_ptr = ROUND_UP ((target_y - ylow) *
					ortho_inv_length);
			no_need_to_pass_by_clb = 1;
		} else { /* In a row that passes by target CLB */
			*num_segs_ortho_dir_ptr = 0;
			no_need_to_pass_by_clb = 0;
		}

		/* Now count horizontal (same dir. as inode) segs. */

		if (xlow > target_x + no_need_to_pass_by_clb) {
			num_segs_same_dir = ROUND_UP ((xlow - no_need_to_pass_by_clb -
							target_x) * inv_length);
		} else if (xhigh < target_x - no_need_to_pass_by_clb) {
			num_segs_same_dir = ROUND_UP ((target_x - no_need_to_pass_by_clb -
							xhigh) * inv_length);
		} else {
			num_segs_same_dir = 0;
		}
	}

	if (rr_type == CHANY) { /* inode is a CHANY */
		ylow = rr_node[inode].ylow;
		yhigh = rr_node[inode].yhigh;
		xlow = rr_node[inode].xlow;

		/* Count horizontal (orthogonal to inode) segs first. */

		if (xlow > target_x) { /* Coming from a column right of target? */
			*num_segs_ortho_dir_ptr = ROUND_UP ((xlow - target_x + 1.) *
					ortho_inv_length);
			no_need_to_pass_by_clb = 1;
		} else if (xlow < target_x - 1) { /* Left of and not adjacent to the CLB? */
			*num_segs_ortho_dir_ptr = ROUND_UP ((target_x - xlow) *
					ortho_inv_length);
			no_need_to_pass_by_clb = 1;
		} else { /* In a column that passes by target CLB */
			*num_segs_ortho_dir_ptr = 0;
			no_need_to_pass_by_clb = 0;
		}

		/* Now count vertical (same dir. as inode) segs. */

		if (ylow > target_y + no_need_to_pass_by_clb) {
			num_segs_same_dir = ROUND_UP ((ylow - no_need_to_pass_by_clb -
							target_y) * inv_length);
		} else if (yhigh < target_y - no_need_to_pass_by_clb) {
			num_segs_same_dir = ROUND_UP ((target_y - no_need_to_pass_by_clb -
							yhigh) * inv_length);
		} else {
			num_segs_same_dir = 0;
		}
	}
	//printf("target_x:%d, target_y:%d, xlow:%d, ylow:%d\n", target_x, target_y, xlow, ylow);
	if (rr_type == DIREX) {
		xlow = rr_node[inode].xlow;
		ylow = rr_node[inode].ylow;
		/* Count vertical (orthogonal to inode) segs first. */

		if (ylow > target_y) { /* Coming from a row above target? */
			*num_segs_ortho_dir_ptr = ROUND_UP ((ylow - target_y) *
					ortho_inv_length);
			//no_need_to_pass_by_clb = 1;
		} else if (ylow < target_y) { /* Below the CLB bottom? */
			*num_segs_ortho_dir_ptr = ROUND_UP ((target_y - ylow) *
					ortho_inv_length);
			//no_need_to_pass_by_clb = 1;
		} else { /* In a row that passes by target CLB */
			*num_segs_ortho_dir_ptr = 0;
			//no_need_to_pass_by_clb = 0;
		}

		/* Now count horizontal (same dir. as inode) segs. */

		if (xlow > target_x) {
			num_segs_same_dir = ROUND_UP ((xlow - target_x +1 ) * inv_length);
		} else if (xlow < target_x - 1) {
			num_segs_same_dir = ROUND_UP ((target_x - xlow ) * inv_length);
		} else {
			num_segs_same_dir = 1;
		}
	}

	if (rr_type == DIREY) {

		xlow = rr_node[inode].xlow;
		ylow = rr_node[inode].ylow;
		/* Count horizontal (orthogonal to inode) segs first. */

		if (xlow > target_x) { /* Coming from a row above target? */
			*num_segs_ortho_dir_ptr = ROUND_UP ((xlow - target_x) *
					ortho_inv_length);
			//no_need_to_pass_by_clb = 1;
		} else if (xlow < target_x) { /* Below the CLB bottom? */
			*num_segs_ortho_dir_ptr = ROUND_UP ((target_x - xlow) *
					ortho_inv_length);
			//no_need_to_pass_by_clb = 1;
		} else { /* In a row that passes by target CLB */
			*num_segs_ortho_dir_ptr = 0;
			//no_need_to_pass_by_clb = 0;
		}

		/* Now count vertical (same dir. as inode) segs. */

		if (ylow > target_y) {
			num_segs_same_dir = ROUND_UP ((ylow - target_y +1 ) * inv_length);
		} else if (ylow < target_y - 1) {
			num_segs_same_dir = ROUND_UP ((target_y - ylow ) * inv_length);
		} else {
			num_segs_same_dir = 1;
		}
	}

	return (num_segs_same_dir);
}

static void update_rr_base_costs(int inet, float largest_criticality) {

	/* Changes the base costs of different types of rr_nodes according to the  *
	 * criticality, fanout, etc. of the current net being routed (inet).       */

	float fanout, factor;
	int index;

	fanout = net[inet].num_pins - 1.;
	factor = sqrt(fanout);
	/* factor = fanout; */
	/* factor = 1.;    */

	for (index = CHANX_COST_INDEX_START; index < num_rr_indexed_data; index++) {
		if (rr_indexed_data[index].T_quadratic > 0.) { /* pass transistor */
			rr_indexed_data[index].base_cost =
					rr_indexed_data[index].saved_base_cost * factor;
		} else {
			rr_indexed_data[index].base_cost =
					rr_indexed_data[index].saved_base_cost;
		}
	}
}
