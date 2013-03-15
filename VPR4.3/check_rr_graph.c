#include "util.h"
#include "vpr_types.h"
#include "globals.h"
#include "rr_graph.h"
#include "check_rr_graph.h"

/********************** Local defines and types *****************************/

#define BUF_FLAG 1
#define PTRANS_FLAG 2
#define BUF_AND_PTRANS_FLAG 3

/*********************** Subroutines local to this module *******************/

static boolean rr_node_is_global_clb_ipin(int inode);

static void check_pass_transistors(int from_node);

/************************ Subroutine definitions ****************************/

void check_rr_graph(enum e_route_type route_type, int num_switch) {

	/* This routine sanity checks the routing resource graph to see that it has *
	 * no repeated edges between two nodes, that there are no disconnected      *
	 * routing resource nodes, and that the attributes of each node are         *
	 * reasonable.                                                              */

	int *num_edges_from_current_to_node; /* [0..num_rr_nodes-1] */
	int *total_edges_to_node; /* [0..num_rr_nodes-1] */
	char *switch_types_from_current_to_node; /* [0..num_rr_nodes-1] */
	int inode, iedge, to_node, num_edges;
	short switch_type;
	t_rr_type rr_type, to_rr_type;

	total_edges_to_node = (int *) my_calloc(num_rr_nodes, sizeof(int));
	num_edges_from_current_to_node = (int *) my_calloc(num_rr_nodes,
			sizeof(int));
	switch_types_from_current_to_node = (char *) my_calloc(num_rr_nodes,
			sizeof(char));

	for (inode = 0; inode < num_rr_nodes; inode++) {
		rr_type = rr_node[inode].type;
		num_edges = rr_node[inode].num_edges;

		check_node(inode, route_type);

		/* Check all the connectivity (edges, etc.) information.                    */

		for (iedge = 0; iedge < num_edges; iedge++) {
			to_node = rr_node[inode].edges[iedge];

			if (to_node < 0 || to_node >= num_rr_nodes) {
				printf("Error in check_rr_graph:  node %d has an edge %d.\n"
						"Edge is out of range.\n", inode, to_node);
				exit(1);
			}

			if (to_node == 0) {
				printf("%d ", inode);
				printf("\n");
			}

			num_edges_from_current_to_node[to_node]++;
			total_edges_to_node[to_node]++;

			switch_type = rr_node[inode].switches[iedge];

			if (switch_type < 0 || switch_type >= num_switch) {
				printf(
						"Error in check_rr_graph:  node %d has a switch type %d.\n"
								"Switch type is out of range.\n", inode,
						switch_type);
				exit(1);
			}

			if (to_node == 942 && inode == 278) {
				iedge++;
				iedge--;
			}

			if (switch_inf[switch_type].buffered)
				switch_types_from_current_to_node[to_node] |= BUF_FLAG;
			else
				switch_types_from_current_to_node[to_node] |= PTRANS_FLAG;

		} /* End for all edges of node. */

		for (iedge = 0; iedge < num_edges; iedge++) {
			to_node = rr_node[inode].edges[iedge];

			if (num_edges_from_current_to_node[to_node] > 1) {
				to_rr_type = rr_node[to_node].type;

				if ((to_rr_type != CHANX && to_rr_type != CHANY)
						|| (rr_type != CHANX && rr_type != CHANY)) {
					num_edges_from_current_to_node[to_node]++;
					printf(
							"Error in check_rr_graph:  node %d connects to node %d "
									"%d times.\n", inode, to_node,
							num_edges_from_current_to_node[to_node]);
					exit(1);
				}

				/* Between two wire segments.  Two connections are legal only if  *
				 * one connection is a buffer and the other is a pass transistor. */

				else if (num_edges_from_current_to_node[to_node] != 2||
				switch_types_from_current_to_node[to_node] !=
				BUF_AND_PTRANS_FLAG) {printf ("Error in check_rr_graph:  node %d connects to node %d "
						"%d times.\n", inode, to_node,
						num_edges_from_current_to_node[to_node]);
				exit (1);
			}
		}

			num_edges_from_current_to_node[to_node] = 0;
			switch_types_from_current_to_node[to_node] = 0;
		}

		/* Slow test below.  Leave commented out most of the time. */

#ifdef DEBUG
		check_pass_transistors(inode);
#endif

	} /* End for all rr_nodes */

	/* I built a list of how many edges went to everything in the code above -- *
	 * now I check that everything is reachable.                                */

	for (inode = 0; inode < num_rr_nodes; inode++) {
		rr_type = rr_node[inode].type;

		if (rr_type != SOURCE) {
			if (total_edges_to_node[inode] < 1
					&& !rr_node_is_global_clb_ipin(inode)) {

				/* A global CLB input pin will not have any edges, and neither will  *
				 * a SOURCE.  Anything else is an error.                             */

				if (rr_type == DIREX || rr_type == DIREY)
					continue;

				printf("Error in check_rr_graph:  node %d has no fanin.\n",
						inode);
				exit(1);
			}
		}

		else { /* SOURCE.  No fanin for now; change if feedthroughs allowed. */
			if (total_edges_to_node[inode] != 0) {
				printf("Error in check_rr_graph:  SOURCE node %d has a fanin\n"
						"\tof %d, expected 0.\n", inode,
						total_edges_to_node[inode]);
				exit(1);
			}
		}
	}

	free(num_edges_from_current_to_node);
	free(total_edges_to_node);
	free(switch_types_from_current_to_node);
}

static boolean rr_node_is_global_clb_ipin(int inode) {

	/* Returns TRUE if inode refers to a global CLB input pin node.   */

	int ipin;

	if (rr_node[inode].type != IPIN)
		return (FALSE);

	if (rr_node[inode].xlow == 0 || rr_node[inode].xlow == nx + 1
			|| rr_node[inode].ylow == 0 || rr_node[inode].ylow == ny + 1) /* I/O pad */
		return (FALSE);

	ipin = rr_node[inode].ptc_num;

	return (is_global_clb_pin[ipin]);
}

void check_node(int inode, enum e_route_type route_type) {

	/* This routine checks that the rr_node is inside the grid and has a valid  *
	 * pin number, etc.                                                         */

	int xlow, ylow, xhigh, yhigh, ptc_num, capacity;
	t_rr_type rr_type;
	int nodes_per_chan, nodes_direct, tracks_per_node, num_edges, cost_index,
			iclass;
	float C, R;

	for (iclass = 0; iclass < num_class; iclass++) {
		if (class_inf[iclass].type == DRIVER)
			nodes_direct = class_inf[iclass].num_pins;
	}

	rr_type = rr_node[inode].type;
	xlow = rr_node[inode].xlow;
	xhigh = rr_node[inode].xhigh;
	ylow = rr_node[inode].ylow;
	yhigh = rr_node[inode].yhigh;
	ptc_num = rr_node[inode].ptc_num;
	capacity = rr_node[inode].capacity;

	if (xlow > xhigh || ylow > yhigh) {
		printf("Error in check_node:  rr endpoints are (%d,%d) and (%d,%d).\n",
				xlow, ylow, xhigh, yhigh);
		exit(1);
	}

	if (xlow < 0 || xhigh > nx + 1 || ylow < 0 || yhigh > ny + 1) {
		printf("Error in check_node:  rr endpoints, (%d,%d) and (%d,%d), \n"
				"are out of range.\n", xlow, ylow, xhigh, yhigh);
		exit(1);
	}

	if (ptc_num < 0) {
		printf("Error in check_node.  Inode %d (type %d) had a ptc_num\n"
				"of %d.\n", inode, rr_type, ptc_num);
		exit(1);
	}

	/* Check that the segment is within the array and such. */

	switch (rr_type) {

	case SOURCE:
	case SINK:
	case IPIN:
	case OPIN:
		if (xlow != xhigh || ylow != yhigh) {
			printf("Error in check_node:  Node %d (type %d) has endpoints of\n"
					"(%d,%d) and (%d,%d)\n", inode, rr_type, xlow, ylow, xhigh,
					yhigh);
			exit(1);
		}
		if (clb[xlow][ylow].type != CLB && clb[xlow][ylow].type != IO
				&& clb[xlow][ylow].type != MEM && clb[xlow][ylow].type != DSP) {
			printf("Error in check_node:  Node %d (type %d) is at an illegal\n"
					" clb location (%d, %d).\n", inode, rr_type, xlow, ylow);
			exit(1);
		}
		break;

	case CHANX:
		if (xlow < 1 || xhigh > nx || yhigh > ny || yhigh != ylow) {
			printf("Error in check_node:  CHANX out of range.\n");
			printf("Endpoints: (%d,%d) and (%d,%d)\n", xlow, ylow, xhigh,
					yhigh);
			exit(1);
		}
		if (route_type == GLOBAL && xlow != xhigh) {
			printf(
					"Error in check_node:  node %d spans multiple channel segments\n" "which is not allowed with global routing.\n",
					inode);
			exit(1);
		}
		break;

	case CHANY:
		if (xhigh > nx || ylow < 1 || yhigh > ny || xlow != xhigh) {
			printf("Error in check_node:  CHANY out of range.\n");
			printf("Endpoints: (%d,%d) and (%d,%d)\n", xlow, ylow, xhigh,
					yhigh);
			exit(1);
		}
		if (route_type == GLOBAL && ylow != yhigh) {
			printf(
					"Error in check_node:  node %d spans multiple channel segments\n" "which is not allowed with global routing.\n",
					inode);
			exit(1);
		}
		break;
	case DIREX:
		//nionio modified
		//if (xlow < 1 || xhigh > nx-1 || yhigh > ny || yhigh != ylow || ylow<1 || xhigh!=xlow) {
		if (xhigh > nx || ylow < 1 || yhigh > ny || xlow != xhigh) {
			printf("Error in check_node:  DIREX out of range.\n");
			printf("Endpoints: (%d,%d) and (%d,%d)\n", xlow, ylow, xhigh,
					yhigh);
			exit(1);
		}
		if (route_type == GLOBAL && xlow != xhigh) {
			printf(
					"Error in check_node:  node %d spans multiple channel segments\n" "which is not allowed with global routing.\n",
					inode);
			exit(1);
		}
		break;

	case DIREY:
		//nionio modified
		//if (xhigh > nx || ylow < 1 || yhigh > ny-1 || xlow != xhigh || xlow <1 || ylow!=yhigh) {
		if (xlow < 1 || xhigh > nx || yhigh > ny || yhigh != ylow) {
			printf("Error in check_node:  DIREY out of range.\n");
			printf("Endpoints: (%d,%d) and (%d,%d)\n", xlow, ylow, xhigh,
					yhigh);
			exit(1);
		}
		if (route_type == GLOBAL && ylow != yhigh) {
			printf(
					"Error in check_node:  node %d spans multiple channel segments\n" "which is not allowed with global routing.\n",
					inode);
			exit(1);
		}
		break;
	default:
		printf("Error in check_node:  Unexpected segment type: %d\n", rr_type);
		exit(1);
	}

	/* Check that it's capacities and such make sense. */

	switch (rr_type) {

	case SOURCE:
		if (clb[xlow][ylow].type == CLB || clb[xlow][ylow].type == MEM) {
			if (ptc_num >= num_class || class_inf[ptc_num].type != DRIVER) {
				printf(
						"Error in check_node.  Inode %d (type %d) had a ptc_num\n"
								"of %d.\n", inode, rr_type, ptc_num);
				exit(1);
			}
			if (class_inf[ptc_num].num_pins != capacity) {
				printf(
						"Error in check_node.  Inode %d (type %d) had a capacity\n"
								"of %d.\n", inode, rr_type, capacity);
				exit(1);
			}
		} else if (clb[xlow][ylow].type == DSP) {
			if (ptc_num >= dsp_num_class
					|| dsp_class_inf[ptc_num].type != DRIVER) {
				printf(
						"Error in check_node.  Inode %d (type %d) had a ptc_num\n"
								"of %d.\n", inode, rr_type, ptc_num);
				exit(1);
			}
			if (dsp_class_inf[ptc_num].num_pins != capacity) {
				printf(
						"Error in check_node.  Inode %d (type %d) had a capacity\n"
								"of %d.\n", inode, rr_type, capacity);
				exit(1);
			}
		} else { /* IO block */
			if (ptc_num >= io_rat) {
				printf(
						"Error in check_node.  Inode %d (type %d) had a ptc_num\n"
								"of %d.\n", inode, rr_type, ptc_num);
				exit(1);
			}
			if (capacity != 1) {
				printf(
						"Error in check_node:  Inode %d (type %d) had a capacity\n"
								"of %d.\n", inode, rr_type, capacity);
				exit(1);
			}
		}
		break;

	case SINK:
		if (clb[xlow][ylow].type == CLB || clb[xlow][ylow].type == MEM) {
			if (ptc_num >= num_class || class_inf[ptc_num].type != RECEIVER) {
				printf(
						"Error in check_node.  Inode %d (type %d) had a ptc_num\n"
								"of %d.\n", inode, rr_type, ptc_num);
				exit(1);
			}
			if (class_inf[ptc_num].num_pins != capacity) {
				printf(
						"Error in check_node.  Inode %d (type %d) has a capacity\n"
								"of %d.\n", inode, rr_type, capacity);
				exit(1);
			}
		} else if (clb[xlow][ylow].type == DSP) {
			if (ptc_num >= dsp_num_class
					|| dsp_class_inf[ptc_num].type != RECEIVER) {
				printf(
						"Error in check_node.  Inode %d (type %d) had a ptc_num\n"
								"of %d.\n", inode, rr_type, ptc_num);
				exit(1);
			}
			if (dsp_class_inf[ptc_num].num_pins != capacity) {
				printf(
						"Error in check_node.  Inode %d (type %d) had a capacity\n"
								"of %d.\n", inode, rr_type, capacity);
				exit(1);
			}
		} else { /* IO block */
			if (ptc_num >= io_rat) {
				printf(
						"Error in check_node.  Inode %d (type %d) had a ptc_num\n"
								"of %d.\n", inode, rr_type, ptc_num);
				exit(1);
			}
			if (capacity != 1) {
				printf(
						"Error in check_node:  Inode %d (type %d) has a capacity\n"
								"of %d.\n", inode, rr_type, capacity);
				exit(1);
			}
		}
		break;

	case OPIN:
		if (clb[xlow][ylow].type == CLB || clb[xlow][ylow].type == MEM) {
			if (ptc_num >= pins_per_clb
					|| class_inf[clb_pin_class[ptc_num]].type != DRIVER) {
				printf(
						"Error in check_node.  Inode %d (type %d) had a ptc_num\n"
								"of %d.\n", inode, rr_type, ptc_num);
				exit(1);
			}
		} else if (clb[xlow][ylow].type == DSP) {
			if (ptc_num >= pins_per_dsp
					|| dsp_class_inf[dsp_pin_class[ptc_num]].type != DRIVER) {
				printf(
						"Error in check_node.  Inode %d (type %d) had a ptc_num\n"
								"of %d.\n", inode, rr_type, ptc_num);
				exit(1);
			}
		} else { /* IO block */
			if (ptc_num >= io_rat) {
				printf(
						"Error in check_node.  Inode %d (type %d) had a ptc_num\n"
								"of %d.\n", inode, rr_type, ptc_num);
				exit(1);
			}
		}

		if (capacity != 1) {
			printf("Error in check_node:  Inode %d (type %d) has a capacity\n"
					"of %d.\n", inode, rr_type, capacity);
			exit(1);
		}
		break;

	case IPIN:
		if (clb[xlow][ylow].type == CLB || clb[xlow][ylow].type == MEM) {
			if (ptc_num >= pins_per_clb
					|| class_inf[clb_pin_class[ptc_num]].type != RECEIVER) {
				printf(
						"Error in check_node.  Inode %d (type %d) had a ptc_num\n"
								"of %d.\n", inode, rr_type, ptc_num);
				exit(1);
			}
		} else if (clb[xlow][ylow].type == DSP) {
			if (ptc_num >= pins_per_dsp
					|| dsp_class_inf[dsp_pin_class[ptc_num]].type != RECEIVER) {
				printf(
						"Error in check_node.  Inode %d (type %d) had a ptc_num\n"
								"of %d.\n", inode, rr_type, ptc_num);
				exit(1);
			}
		} else { /* IO block */
			if (ptc_num >= io_rat) {
				printf(
						"Error in check_node.  Inode %d (type %d) had a ptc_num\n"
								"of %d.\n", inode, rr_type, ptc_num);
				exit(1);
			}
		}
		if (capacity != 1) {
			printf("Error in check_node:  Inode %d (type %d) has a capacity\n"
					"of %d.\n", inode, rr_type, capacity);
			exit(1);
		}
		break;

	case CHANX:
		if (route_type == DETAILED) {
			nodes_per_chan = chan_width_x[ylow];
			tracks_per_node = 1;
		} else {
			nodes_per_chan = 1;
			tracks_per_node = chan_width_x[ylow];
		}

		if (ptc_num >= nodes_per_chan) {
			printf("Error in check_node:  Inode %d (type %d) has a ptc_num\n"
					"of %d.\n", inode, rr_type, ptc_num);
			exit(1);
		}

		if (capacity != tracks_per_node) {
			printf("Error in check_node:  Inode %d (type %d) has a capacity\n"
					"of %d.\n", inode, rr_type, capacity);
			exit(1);
		}
		break;

	case CHANY:
		if (route_type == DETAILED) {
			nodes_per_chan = chan_width_y[xlow];
			tracks_per_node = 1;
		} else {
			nodes_per_chan = 1;
			tracks_per_node = chan_width_y[xlow];
		}

		if (ptc_num >= nodes_per_chan) {
			printf("Error in check_node:  Inode %d (type %d) has a ptc_num\n"
					"of %d.\n", inode, rr_type, ptc_num);
			exit(1);
		}

		if (capacity != tracks_per_node) {
			printf("Error in check_node:  Inode %d (type %d) has a capacity\n"
					"of %d.\n", inode, rr_type, capacity);
			exit(1);
		}
		break;

		//nionio commented for now

	case DIREX:

		/*if (ptc_num >= nodes_direct) {
		 printf ("Error in check_node:  Inode %d (type %d) has a ptc_num\n"
		 "of %d.\n", inode, rr_type, ptc_num);
		 exit (1);
		 }*/

		if (capacity != 1) {
			printf("Error in check_node:  Inode %d (type %d) has a capacity\n"
					"of %d.\n", inode, rr_type, capacity);
			exit(1);
		}
		break;

	case DIREY:

		/*if (ptc_num >= nodes_direct) {
		 printf ("Error in check_node:  Inode %d (type %d) has a ptc_num\n"
		 "of %d.\n", inode, rr_type, ptc_num);
		 exit (1);
		 }*/

		if (capacity != 1) {
			printf("Error in check_node:  Inode %d (type %d) has a capacity\n"
					"of %d.\n", inode, rr_type, capacity);
			exit(1);
		}
		break;

	default:
		printf("Error in check_node:  Unexpected segment type: %d\n", rr_type);
		exit(1);

	}

	/* Check that the number of (out) edges is reasonable. */

	num_edges = rr_node[inode].num_edges;

	if (rr_type != SINK) {
		if (rr_type == DIREX && clb[xhigh + 1][yhigh].type == DSP) {
			printf(
					"Warning: Direct Link X at [%d][%d] meets a DSP block on its right.\n",
					xhigh, yhigh);
		} else if (rr_type == DIREY && clb[xhigh][yhigh + 1].type == DSP) {
			printf(
					"Warning: Direct Link Y at [%d][%d] meets a DSP block on its top.\n",
					xhigh, yhigh);
		} else if (num_edges <= 0) {
			printf("Error in check_node: node %d has no edges.\n", inode);
			exit(1);
		}
	}

	else { /* SINK -- remove this check if feedthroughs allowed */
		if (num_edges != 0) {
			printf("Error in check_node: node %d is a sink, but has "
					"%d edges.\n", inode, num_edges);
			exit(1);
		}
	}

	/* Check that the capacitance, resistance and cost_index are reasonable. */

	C = rr_node[inode].C;
	R = rr_node[inode].R;

	if (rr_type == CHANX || rr_type == CHANY || rr_type == DIREX
			|| rr_type == DIREY) {
		if (C < 0. || R < 0.) {
			printf("Error in check_node: node %d of type %d has R = %g "
					"and C = %g.\n", inode, rr_type, R, C);
			exit(1);
		}
	}

	else {
		if (C != 0. || R != 0.) {
			printf("Error in check_node: node %d of type %d has R = %g "
					"and C = %g.\n", inode, rr_type, R, C);
			exit(1);
		}
	}

	cost_index = rr_node[inode].cost_index;
	if (cost_index < 0 || cost_index >= num_rr_indexed_data) {
		printf("Error in check_node:  node %d cost index (%d) is out of "
				"range.\n", inode, cost_index);
		exit(1);
	}
}

static void check_pass_transistors(int from_node) {

	/* This routine checks that all pass transistors in the routing truly are  *
	 * bidirectional.  It may be a slow check, so don't use it all the time.   */

	int from_edge, to_node, to_edge, from_num_edges, to_num_edges;
	t_rr_type from_rr_type, to_rr_type;
	short from_switch_type;
	boolean trans_matched;

	from_rr_type = rr_node[from_node].type;
	if (from_rr_type != CHANX && from_rr_type != CHANY)
		return;

	from_num_edges = rr_node[from_node].num_edges;

	for (from_edge = 0; from_edge < from_num_edges; from_edge++) {
		to_node = rr_node[from_node].edges[from_edge];
		to_rr_type = rr_node[to_node].type;

		if (to_rr_type != CHANX && to_rr_type != CHANY)
			continue;

		from_switch_type = rr_node[from_node].switches[from_edge];

		if (switch_inf[from_switch_type].buffered)
			continue;

		/* We know that we have a pass transitor from from_node to to_node.  Now *
		 * check that there is a corresponding edge from to_node back to         *
		 * from_node.                                                            */

		to_num_edges = rr_node[to_node].num_edges;
		trans_matched = FALSE;

		for (to_edge = 0; to_edge < to_num_edges; to_edge++) {
			if (rr_node[to_node].edges[to_edge] == from_node
					&& rr_node[to_node].switches[to_edge] == from_switch_type) {
				trans_matched = TRUE;
				break;
			}
		}

		if (trans_matched == FALSE) {
			printf(
					"Error in check_pass_transistors:  Connection from node %d to\n"
							"node %d uses a pass transistor (switch type %d), but there is\n"
							"no corresponding pass transistor edge in the other direction.\n",
					from_node, to_node, from_switch_type);
			exit(1);
		}

	} /* End for all from_node edges */
}
