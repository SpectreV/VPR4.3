/*#include <stdlib.h> */
#include <stdio.h>
#include <math.h>
#include "util.h"
#include "hash.h"
#include "vpr_types.h"
#include "globals.h"
#include "place.h"
#include "read_place.h"
#include "draw.h"
#include "place_and_route.h"
#include "net_delay.h"
#include "path_delay.h"
#include "timing_place_lookup.h"
#include "timing_place.h"

/************** Types and defines local to place.c ***************************/

#define SMALL_NET 4    /* Cut off for incremental bounding box updates. */
/* 4 is fastest -- I checked.                    */

#define INITIAL_BLOCK_STORAGE 2000

/* For comp_cost.  NORMAL means use the method that generates updateable  *
 * bounding boxes for speed.  CHECK means compute all bounding boxes from *
 * scratch using a very simple routine to allow checks of the other       *
 * costs.                                                                 */

enum cost_methods {
	NORMAL, CHECK
};

#define FROM 0      /* What block connected to a net has moved? */
#define TO 1
#define FROM_AND_TO 2

#define ERROR_TOL .001
#define MAX_MOVES_BEFORE_RECOMPUTE 1000000

#define EMPTY -1
#define PSUDO_CELL -2 /* Added by Wei */   

/********************** Variables local to place.c ***************************/

/* [0..num_nets-1]  0 if net never connects to the same block more than  *
 *  once, otherwise it gives the number of duplicate connections.        */

static int *duplicate_pins;

/* [0..num_nets-1][0..num_unique_blocks-1]  Contains a list of blocks with *
 * no duplicated blocks for ONLY those nets that had duplicates.           */

static int **unique_pin_list;

/* Cost of a net, and a temporary cost of a net used during move assessment. */

static float *net_cost = NULL, *temp_net_cost = NULL; /* [0..num_nets-1] */

/* [0..num_nets-1][1..num_pins-1]. What is the value of the timing   */
/* driven portion of the cost function. These arrays will be set to  */
/* (criticality * delay) for each point to point connection. */
static float **point_to_point_timing_cost = NULL;
static float **temp_point_to_point_timing_cost = NULL;

/* Added by Wei */
static float *stage_timing_cost = NULL; /*[0..num_stage-1] */
static float *stage_delay_cost = NULL;
static float *folding_delay = NULL;
static float *temp_timing = NULL;

/* [0..num_nets-1][1..num_pins-1]. What is the value of the delay */
/* for each connection in the circuit */
static float **point_to_point_delay_cost = NULL;
static float **temp_point_to_point_delay_cost = NULL;

/* [0..num_blocks-1][0..pins_per_clb-1]. Indicates which pin on the net */
/* this block corresponds to, this is only required during timing-driven */
/* placement. It is used to allow us to update individual connections on */
/* each net */
static int **net_pin_index = NULL;


/* [0..num_nets-1].  Store the bounding box coordinates and the number of    *
 * blocks on each of a net's bounding box (to allow efficient updates),      *
 * respectively.                                                             */

static struct s_bb *bb_coords = NULL, *bb_num_on_edges = NULL; 

/* Stores the maximum and expected occupancies, plus the cost, of each   *
 * region in the placement.  Used only by the NONLINEAR_CONG cost        *
 * function.  [0..num_region-1][0..num_region-1].  Place_region_x and    *
 * y give the situation for the x and y directed channels, respectively. */

static struct s_place_region **place_region_x, **place_region_y;

/* Used only with nonlinear congestion.  [0..num_regions].            */

static float *place_region_bounds_x, *place_region_bounds_y;

/* The arrays below are used to precompute the inverse of the average   *
 * number of tracks per channel between [subhigh] and [sublow].  Access *
 * them as chan?_place_cost_fac[subhigh][sublow].  They are used to     *
 * speed up the computation of the cost function that takes the length  *
 * of the net bounding box in each dimension, divided by the average    *
 * number of tracks in that direction; for other cost functions they    *
 * will never be used.                                                  */

static float **chanx_place_cost_fac, **chany_place_cost_fac;


/* Expected crossing counts for nets with different #'s of pins.  From *
 * ICCAD 94 pp. 690 - 695 (with linear interpolation applied by me).   */
 
static const float cross_count[50] = {   /* [0..49] */
1.0, 1.0, 1.0, 1.0828, 1.1536, 1.2206, 1.2823, 1.3385, 1.3991, 1.4493, 1.4974,
		1.5455, 1.5937, 1.6418, 1.6899, 1.7304, 1.7709, 1.8114, 1.8519, 1.8924,
		1.9288, 1.9652, 2.0015, 2.0379, 2.0743, 2.1061, 2.1379, 2.1698, 2.2016,
		2.2334, 2.2646, 2.2958, 2.3271, 2.3583, 2.3895, 2.4187, 2.4479, 2.4772,
		2.5064, 2.5356, 2.5610, 2.5864, 2.6117, 2.6371, 2.6625, 2.6887, 2.7148,
		2.7410, 2.7671, 2.7933 };

/********************* Static subroutines local to place.c *******************/

static void alloc_and_load_unique_pin_list(void);

static void free_unique_pin_list(void);

static void alloc_place_regions(int num_regions);

static void load_place_regions(int num_regions);

static void free_place_regions(int num_regions);

static void alloc_and_load_placement_structs(int place_cost_type,
		int num_regions, float place_cost_exp, float ***old_region_occ_x,
		float ***old_region_occ_y, struct s_placer_opts placer_opts);

static void free_placement_structs(int place_cost_type, int num_regions,
		float **old_region_occ_x, float **old_region_occ_y,
		struct s_placer_opts placer_opts);

static void alloc_and_load_for_fast_cost_update(float place_cost_exp);

static void initial_placement(enum e_pad_loc_type pad_loc_type,
		char *pad_loc_file, int density_x, int density_y);

static void initial_stage_placement(enum e_pad_loc_type pad_loc_type,
		char *pad_loc_file, int density_x, int density_y);

static float comp_bb_cost(int method, int place_cost_type, int num_regions);

static int try_swap(float t, float *cost, float *bb_cost, float *timing_cost,
		float rlim, int *pins_on_block, int place_cost_type,
		float **old_region_occ_x, float **old_region_occ_y, int num_regions,
		boolean fixed_pins, enum e_place_algorithm place_algorithm,
		float timing_tradeoff, float inverse_prev_bb_cost,
		float inverse_prev_timing_cost, float *delay_cost);

static void check_place(float bb_cost, float timing_cost, int place_cost_type,
		int num_regions, enum e_place_algorithm place_algorithm,
		float delay_cost);

static float starting_t(float *cost_ptr, float *bb_cost_ptr,
		float *timing_cost_ptr, int *pins_on_block, int place_cost_type,
		float **old_region_occ_x, float **old_region_occ_y, int num_regions,
		boolean fixed_pins, struct s_annealing_sched annealing_sched,
		int max_moves, float rlim, enum e_place_algorithm place_algorithm,
		float timing_tradeoff, float inverse_prev_bb_cost,
		float inverse_prev_timing_cost, float *delay_cost_ptr);

static void update_t(float *t, float std_dev, float rlim, float success_rat,
		struct s_annealing_sched annealing_sched);

static void update_rlim(float *rlim, float success_rat);

static int exit_crit(float t, float cost,
		struct s_annealing_sched annealing_sched);

static int count_connections(void);

static void compute_net_pin_index_values(void);

static double get_std_dev(int n, double sum_x_squared, double av_x);

static void free_fast_cost_update_structs(void);

static float recompute_bb_cost(int place_cost_type, int num_regions);

static float comp_td_point_to_point_delay(int inet, int ipin);

static void update_td_cost(int *b_from, int *b_to, int num_of_pins, int length);

static void comp_delta_td_cost(int *b_from, int *b_to, int num_of_pins,
		float *delta_timing, float *delta_delay, int length);

static void comp_td_costs(float *timing_cost, float *connection_delay_sum);

static int assess_swap(float delta_c, float t);

static void find_to(int x_from, int y_from, int type, float rlim, int *x_to,
		int *y_to);

static void get_non_updateable_bb(int inet, struct s_bb *bb_coord_new);

static void update_bb(int inet, struct s_bb *bb_coord_new,
		struct s_bb *bb_edge_new, int xold, int yold, int xnew, int ynew);

static int find_affected_nets(int *nets_to_update, int *net_block_moved,
		int *b_from, int *b_to, int num_of_pins, int length);

static float get_net_cost(int inet, struct s_bb *bb_ptr);

static float nonlinear_cong_cost(int num_regions);

static void update_region_occ(int inet, struct s_bb*coords, int add_or_sub,
		int num_regions);

static void save_region_occ(float **old_region_occ_x, float **old_region_occ_y,
		int num_regions);

static void restore_region_occ(float **old_region_occ_x,
		float **old_region_occ_y, int num_regions);

static void get_bb_from_scratch(int inet, struct s_bb *coords,
		struct s_bb *num_on_edges);

static void check_placement();

/*****************************************************************************/

void try_place(struct s_placer_opts placer_opts,
		struct s_annealing_sched annealing_sched,
		t_chan_width_dist chan_width_dist, struct s_router_opts router_opts,
		struct s_det_routing_arch det_routing_arch, t_segment_inf *segment_inf,
		t_timing_inf timing_inf, t_subblock_data *subblock_data_ptr) {

	/* Does almost all the work of placing a circuit.  Width_fac gives the   *
	 * width of the widest channel.  Place_cost_exp says what exponent the   *
	 * width should be taken to when calculating costs.  This allows a       *
	 * greater bias for anisotropic architectures.  Place_cost_type          *
	 * determines which cost function is used.  num_regions is used only     *
	 * the place_cost_type is NONLINEAR_CONG.                                */

	int tot_iter, inner_iter, success_sum, pins_on_block[5];
	int move_lim, moves_since_cost_recompute, width_fac;
	float t, success_rat, rlim, d_max, est_crit;
	float cost, timing_cost, bb_cost, new_bb_cost, new_timing_cost;
	float delay_cost, new_delay_cost, place_delay_value;
	float inverse_prev_bb_cost, inverse_prev_timing_cost;
	float oldt;
	double av_cost, av_bb_cost, av_timing_cost, av_delay_cost, sum_of_squares,
			std_dev;
	float **old_region_occ_x, **old_region_occ_y;
	char msg[BUFSIZE];
	boolean fixed_pins; /* Can pads move or not? */
	int num_connections;
	int inet, ipin, outer_crit_iter_count, inner_crit_iter_count,
			inner_recompute_limit;
	float **net_slack, **net_delay;
	float crit_exponent;
	float first_rlim, final_rlim, inverse_delta_rlim;
	float **remember_net_delay_original_ptr; /*used to free net_delay if it is re-assigned*/

	int istage;

	remember_net_delay_original_ptr = NULL; /*prevents compiler warning*/

	//initialization before the initial placement
	if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
			|| placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
			|| placer_opts.enable_timing_computations) {

		/*do this before the initial placement to avoid messing up the initial placement */

		alloc_lookups_and_criticalities(chan_width_dist, router_opts,
				det_routing_arch, segment_inf, timing_inf, *subblock_data_ptr,
				&net_delay, &net_slack);

		remember_net_delay_original_ptr = net_delay;

		/*#define PRINT_LOWER_BOUND*/
#ifdef PRINT_LOWER_BOUND
		/*print the crit_path, assuming delay between blocks that are*
		 *block_dist apart*/

		if (placer_opts.block_dist <= nx)
		place_delay_value = delta_clb_to_clb[placer_opts.block_dist][0];
		else if (placer_opts.block_dist <= ny)
		place_delay_value = delta_clb_to_clb[0][placer_opts.block_dist];
		else
		place_delay_value = delta_clb_to_clb[nx][ny];

		printf("\nLower bound assuming delay of %g\n", place_delay_value);

		load_constant_net_delay (net_delay, place_delay_value);
		load_timing_graph_net_delays(net_delay);
		d_max = load_net_slack(net_slack, 0, FALSE);

		print_critical_path("Placement_Lower_Bound.echo");
		print_sink_delays("Placement_Lower_Bound_Sink_Delays.echo");

		/*also print sink delays assuming 0 delay between blocks,
		 this tells us how much logic delay is on each path*/

		load_constant_net_delay (net_delay, 0);
		load_timing_graph_net_delays(net_delay);
		d_max = load_net_slack(net_slack, 0, FALSE);

		print_sink_delays("Placement_Logic_Sink_Delays.echo");
#endif

	} //initialization done

	width_fac = placer_opts.place_chan_width;
	if (placer_opts.pad_loc_type == FREE)
		fixed_pins = FALSE;
	else
		fixed_pins = TRUE;

	init_chan(width_fac, chan_width_dist);

	alloc_and_load_placement_structs(placer_opts.place_cost_type,
			placer_opts.num_regions, placer_opts.place_cost_exp,
			&old_region_occ_x, &old_region_occ_y, placer_opts);

	//initial placement
	if (!is_folding)
		initial_placement(placer_opts.pad_loc_type, placer_opts.pad_loc_file,
				subblock_data_ptr->mem_density_x,
				subblock_data_ptr->mem_density_y);
	else {
		initial_stage_placement(placer_opts.pad_loc_type,
				placer_opts.pad_loc_file, subblock_data_ptr->mem_density_x,
				subblock_data_ptr->mem_density_y);
		check_placement();
	}
	init_draw_coords((float) width_fac);

	/* Storing the number of pins on each type of block makes the swap routine *
	 * slightly more efficient.                                                */

	pins_on_block[CLB] = pins_per_clb;
	pins_on_block[MEM] = pins_per_clb;
	pins_on_block[OUTPAD] = 1;
	pins_on_block[INPAD] = 1;
	pins_on_block[DSP] = pins_per_dsp;

	/* Gets initial cost and loads bounding boxes. */

	if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
			|| placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE) {
		bb_cost = comp_bb_cost(NORMAL, placer_opts.place_cost_type,
				placer_opts.num_regions);
		printf("the bb cost is %11.6g \n", bb_cost);
		crit_exponent = placer_opts.td_place_exp_first; /*this will be modified when rlim starts to change*/

		compute_net_pin_index_values();

		num_connections = count_connections();
		printf("\nThere are %d point to point connections in this circuit\n\n",
				num_connections);

		if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE) {
			for (inet = 0; inet < num_nets; inet++)
				for (ipin = 1; ipin < net[inet].num_pins; ipin++)
					timing_place_crit[inet][ipin] = 0; /*dummy crit values*/

			comp_td_costs(&timing_cost, &delay_cost); /*first pass gets delay_cost, which is used
			 in criticality computations in the next call
			 to comp_td_costs.*/
			place_delay_value = delay_cost / num_connections; /*used for computing criticalities */
			load_constant_net_delay(net_delay, place_delay_value);

		} else
			place_delay_value = 0;

		if (placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE) {
			net_delay = point_to_point_delay_cost;/*this keeps net_delay up to date with      *
			 *the same values that the placer is using  *
			 *point_to_point_delay_cost is computed each*
			 *time that comp_td_costs is called, and is *
			 *also updated after any swap is accepted   */
		}

		load_timing_graph_net_delays(net_delay);
		d_max = load_net_slack(net_slack, 0, FALSE);
		printf("the max_slack is %11.6g \n", d_max);
		load_criticalities(placer_opts, net_slack, d_max, crit_exponent);
		outer_crit_iter_count = 1;

		/*now we can properly compute costs  */
		comp_td_costs(&timing_cost, &delay_cost); /*also puts proper values into point_to_point_delay_cost*/
		// printf("the timing cost %11.6g, delay cost %11.6g \n", timing_cost, delay_cost);

		inverse_prev_timing_cost = 1 / timing_cost;
		inverse_prev_bb_cost = 1 / bb_cost;
		cost = 1; /*our new cost function uses normalized values of           */
		/*bb_cost and timing_cost, the value of cost will be reset  */
		/*to 1 at each temperature when *_TIMING_DRIVEN_PLACE is true*/
	} else { /*BOUNDING_BOX_PLACE*/
		cost = bb_cost = comp_bb_cost(NORMAL, placer_opts.place_cost_type,
				placer_opts.num_regions);
		timing_cost = 0;
		delay_cost = 0;
		place_delay_value = 0;
		outer_crit_iter_count = 0;
		num_connections = 0;
		d_max = 0;
		crit_exponent = 0;

		inverse_prev_timing_cost = 0; /*inverses not used */
		inverse_prev_bb_cost = 0;
	}

	if (!is_folding)
		move_lim = (int) (annealing_sched.inner_num * pow(num_blocks, 1.3333));
	else
		move_lim = (int) (annealing_sched.inner_num
				* pow(num_blocks / num_stage, 1.3333));

	if (placer_opts.inner_loop_recompute_divider != 0)
		inner_recompute_limit = (int) (0.5
				+ (float) move_lim
						/ (float) placer_opts.inner_loop_recompute_divider);
	else
		/*don't do an inner recompute */
		inner_recompute_limit = move_lim + 1;

	/* Sometimes I want to run the router with a random placement.  Avoid *
	 * using 0 moves to stop division by 0 and 0 length vector problems,  *
	 * by setting move_lim to 1 (which is still too small to do any       *
	 * significant optimization).                                         */

	if (move_lim <= 0)
		move_lim = 1;

	rlim = (float) max (nx, ny);

	first_rlim = rlim; /*used in timing-driven placement for exponent computation*/
	final_rlim = 1;
	inverse_delta_rlim = 1 / (first_rlim - final_rlim);
	printf("the move_lim %d\n", move_lim);
	printf("the cost is %g\n", cost);

	// printf("the timing trade_off %g\n", placer_opts.timing_tradeoff);
	t = starting_t(&cost, &bb_cost, &timing_cost, pins_on_block,
			placer_opts.place_cost_type, old_region_occ_x, old_region_occ_y,
			placer_opts.num_regions, fixed_pins, annealing_sched, move_lim,
			rlim, placer_opts.place_algorithm, placer_opts.timing_tradeoff,
			inverse_prev_bb_cost, inverse_prev_timing_cost, &delay_cost);

	printf("initial temp: %g\n", t);

	tot_iter = 0;
	moves_since_cost_recompute = 0;
	printf(
			"Initial Placement Cost: %g bb_cost: %g td_cost: %g delay_cost: %g\n\n",
			cost, bb_cost, timing_cost, delay_cost);

#ifndef SPEC
	printf(
			"%11s  %10s %11s  %11s  %11s %11s  %11s %9s %8s  %7s  %7s  %10s  %7s\n",
			"T", "Cost", "Av. BB Cost", "Av. TD Cost", "Av Tot Del",
			"P to P Del", "d_max", "Ac Rate", "Std Dev", "R limit", "Exp",
			"Tot. Moves", "Alpha");
	printf(
			"%11s  %10s %11s  %11s  %11s %11s  %11s %9s %8s  %7s  %7s  %10s  %7s\n",
			"--------", "----------", "-----------", "-----------", "---------",
			"----------", "-----", "-------", "-------", "-------", "-------",
			"----------", "-----");
#endif

	sprintf(msg,
			"Initial Placement.  Cost: %g  BB Cost: %g  TD Cost %g  Delay Cost: %g "
					"\t d_max %g Channel Factor: %d", cost, bb_cost,
			timing_cost, delay_cost, d_max, width_fac);

	/* Modified by Wei */
	if (!is_folding) {
		update_screen(MINOR, msg, PLACEMENT, FALSE);
	} else {
		for (istage = 1; istage <= num_stage; istage++) {
			current_stage = istage;
			update_screen(MINOR, msg, PLACEMENT, FALSE);
		}
	}

	while (exit_crit(t, cost, annealing_sched) == 0) {

		if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
				|| placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE) {
			cost = 1;
		}

		av_cost = 0.;
		av_bb_cost = 0.;
		av_delay_cost = 0.;
		av_timing_cost = 0.;
		sum_of_squares = 0.;
		success_sum = 0;

		//printf("come here\n");
		if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
				|| placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE) {

			if (outer_crit_iter_count >= placer_opts.recompute_crit_iter
					|| placer_opts.inner_loop_recompute_divider != 0) {
#ifdef VERBOSE
				printf("Outer Loop Recompute Criticalities\n");
#endif
				place_delay_value = delay_cost / num_connections;

				if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE)
					load_constant_net_delay(net_delay, place_delay_value);
				/*note, for path_based, the net delay is not updated since it is current,
				 *because it accesses point_to_point_delay array */

				load_timing_graph_net_delays(net_delay);
				d_max = load_net_slack(net_slack, 0, FALSE);
				load_criticalities(placer_opts, net_slack, d_max,
						crit_exponent);
				/*recompute costs from scratch, based on new criticalities*/
				comp_td_costs(&timing_cost, &delay_cost);
				outer_crit_iter_count = 0;
			}
			outer_crit_iter_count++;

			/*at each temperature change we update these values to be used     */
			/*for normalizing the tradeoff between timing and wirelength (bb)  */
			inverse_prev_bb_cost = 1 / bb_cost;
			inverse_prev_timing_cost = 1 / timing_cost;
		}

		inner_crit_iter_count = 1;
		for (inner_iter = 0; inner_iter < move_lim; inner_iter++) {
			if (try_swap(t, &cost, &bb_cost, &timing_cost, rlim, pins_on_block,
					placer_opts.place_cost_type, old_region_occ_x,
					old_region_occ_y, placer_opts.num_regions, fixed_pins,
					placer_opts.place_algorithm, placer_opts.timing_tradeoff,
					inverse_prev_bb_cost, inverse_prev_timing_cost, &delay_cost)
					== 1) {
				success_sum++;
				av_cost += cost;
				av_bb_cost += bb_cost;
				av_timing_cost += timing_cost;
				av_delay_cost += delay_cost;
				sum_of_squares += cost * cost;
			}

			if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
					|| placer_opts.place_algorithm
							== PATH_TIMING_DRIVEN_PLACE) {

				if (inner_crit_iter_count >= inner_recompute_limit
						&& inner_iter != move_lim - 1) { /*on last iteration don't recompute*/

					inner_crit_iter_count = 0;
#ifdef VERBOSE
					printf("Inner Loop Recompute Criticalities\n");
#endif
					if (placer_opts.place_algorithm
							== NET_TIMING_DRIVEN_PLACE) {
						place_delay_value = delay_cost / num_connections;
						load_constant_net_delay(net_delay, place_delay_value);
					}

					load_timing_graph_net_delays(net_delay);
					d_max = load_net_slack(net_slack, 0, FALSE);
					load_criticalities(placer_opts, net_slack, d_max,
							crit_exponent);
					comp_td_costs(&timing_cost, &delay_cost);
				}
				inner_crit_iter_count++;
			}

#ifdef VERBOSE
			printf("t = %g  cost = %g   bb_cost = %g timing_cost = %g move = %d dmax = %g\n",
					t, cost, bb_cost, timing_cost, inner_iter, d_max);
			if (fabs(bb_cost - comp_bb_cost(CHECK, placer_opts.place_cost_type,
									placer_opts.num_regions)) > bb_cost * ERROR_TOL)
			exit(1);
#endif 
		}

		/* Lines below prevent too much round-off error from accumulating *
		 * in the cost over many iterations.  This round-off can lead to  *
		 * error checks failing because the cost is different from what   *
		 * you get when you recompute from scratch.                       */

		moves_since_cost_recompute += move_lim;
		if (moves_since_cost_recompute > MAX_MOVES_BEFORE_RECOMPUTE) {
			new_bb_cost = recompute_bb_cost(placer_opts.place_cost_type,
					placer_opts.num_regions);
			if (fabs(new_bb_cost - bb_cost) > bb_cost * ERROR_TOL) {
				printf(
						"Error in try_place:  new_bb_cost = %g, old bb_cost = %g.\n",
						new_bb_cost, bb_cost);
				exit(1);
			}
			bb_cost = new_bb_cost;

			if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
					|| placer_opts.place_algorithm
							== PATH_TIMING_DRIVEN_PLACE) {
				comp_td_costs(&new_timing_cost, &new_delay_cost);
				if (fabs(
						new_timing_cost
								- timing_cost) > timing_cost * ERROR_TOL) {
					printf(
							"Error in try_place:  new_timing_cost = %g, old timing_cost = %g.\n",
							new_timing_cost, timing_cost);
					exit(1);
				}
				if (fabs(new_delay_cost - delay_cost) > delay_cost * ERROR_TOL) {
					printf(
							"Error in try_place:  new_delay_cost = %g, old delay_cost = %g.\n",
							new_delay_cost, delay_cost);
					exit(1);
				}
				timing_cost = new_timing_cost;
			}

			if (placer_opts.place_algorithm == BOUNDING_BOX_PLACE) {
				cost = new_bb_cost;
			}
			moves_since_cost_recompute = 0;
		}

		tot_iter += move_lim;
		success_rat = ((float) success_sum) / move_lim;
		if (success_sum == 0) {
			av_cost = cost;
			av_bb_cost = bb_cost;
			av_timing_cost = timing_cost;
			av_delay_cost = delay_cost;
		} else {
			av_cost /= success_sum;
			av_bb_cost /= success_sum;
			av_timing_cost /= success_sum;
			av_delay_cost /= success_sum;
		}
		std_dev = get_std_dev(success_sum, sum_of_squares, av_cost);

#ifndef SPEC
		printf(
				"%11.5g  %10.6g %11.6g  %11.6g  %11.6g %11.6g %11.4g %9.4g %8.3g  %7.4g  %7.4g  %10d  ",
				t, av_cost, av_bb_cost, av_timing_cost, av_delay_cost,
				place_delay_value, d_max, success_rat, std_dev, rlim,
				crit_exponent, tot_iter);
#endif

		oldt = t; /* for finding and printing alpha. */
		update_t(&t, std_dev, rlim, success_rat, annealing_sched);

#ifndef SPEC
		printf("%7.4g\n", t / oldt);
#endif
		// printf("success_rat %g, temp %g\n",success_rat,t);

		sprintf(msg,
				"Cost: %g  BB Cost %g  TD Cost %g  Temperature: %g  d_max: %g",
				cost, bb_cost, timing_cost, t, d_max);

		if (!is_folding) {
			update_screen(MINOR, msg, PLACEMENT, FALSE);
		} else {
			for (istage = 1; istage <= num_stage; istage++) {
				current_stage = istage;
				update_screen(MINOR, msg, PLACEMENT, FALSE);
			}
		}
		update_rlim(&rlim, success_rat);

		if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
				|| placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE) {
			crit_exponent = (1 - (rlim - final_rlim) * inverse_delta_rlim)
					* (placer_opts.td_place_exp_last
							- placer_opts.td_place_exp_first)
					+ placer_opts.td_place_exp_first;
		}
		//printf("the out counter %d\n",outer_crit_iter_count);
		//printf("The timing %g, delay %g\n",timing_cost, delay_cost);
#ifdef VERBOSE
		if (!is_folding)
		dump_clbs();
		else
		dump_stage_clbs();
#endif
	}

	t = 0; /* freeze out */
	av_cost = 0.;
	av_bb_cost = 0.;
	av_timing_cost = 0.;
	sum_of_squares = 0.;
	av_delay_cost = 0.;
	success_sum = 0;
	printf("finish out loop\n");
	printf("here timing %g, delay %g, block %g\n", timing_cost, delay_cost,
			bb_cost);
	if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
			|| placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE) {
		/*at each temperature change we update these values to be used     */
		/*for normalizing the tradeoff between timing and wirelength (bb)  */
		if (outer_crit_iter_count >= placer_opts.recompute_crit_iter
				|| placer_opts.inner_loop_recompute_divider != 0) {

#ifdef VERBOSE
			printf("Outer Loop Recompute Criticalities\n");
#endif
			place_delay_value = delay_cost / num_connections;

			if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE)
				load_constant_net_delay(net_delay, place_delay_value);

			load_timing_graph_net_delays(net_delay);
			d_max = load_net_slack(net_slack, 0, FALSE);
			load_criticalities(placer_opts, net_slack, d_max, crit_exponent);
			/*recompute criticaliies */
			comp_td_costs(&timing_cost, &delay_cost);
			outer_crit_iter_count = 0;
		}
		outer_crit_iter_count++;

		inverse_prev_bb_cost = 1 / (bb_cost);
		inverse_prev_timing_cost = 1 / (timing_cost);
	}

	inner_crit_iter_count = 1;

	//printf("the inner limit %d\n", inner_recompute_limit);
	for (inner_iter = 0; inner_iter < move_lim; inner_iter++) {
		if (try_swap(t, &cost, &bb_cost, &timing_cost, rlim, pins_on_block,
				placer_opts.place_cost_type, old_region_occ_x, old_region_occ_y,
				placer_opts.num_regions, fixed_pins,
				placer_opts.place_algorithm, placer_opts.timing_tradeoff,
				inverse_prev_bb_cost, inverse_prev_timing_cost, &delay_cost)
				== 1) {
			success_sum++;
			av_cost += cost;
			av_bb_cost += bb_cost;
			av_delay_cost += delay_cost;
			av_timing_cost += timing_cost;
			sum_of_squares += cost * cost;

			if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
					|| placer_opts.place_algorithm
							== PATH_TIMING_DRIVEN_PLACE) {

				if (inner_crit_iter_count >= inner_recompute_limit
						&& inner_iter != move_lim - 1) {

					inner_crit_iter_count = 0;
#ifdef VERBOSE
					printf("Inner Loop Recompute Criticalities\n");
#endif
					if (placer_opts.place_algorithm
							== NET_TIMING_DRIVEN_PLACE) {
						place_delay_value = delay_cost / num_connections;
						load_constant_net_delay(net_delay, place_delay_value);
					}

					load_timing_graph_net_delays(net_delay);
					d_max = load_net_slack(net_slack, 0, FALSE);
					load_criticalities(placer_opts, net_slack, d_max,
							crit_exponent);
					comp_td_costs(&timing_cost, &delay_cost);
				}
				inner_crit_iter_count++;
			}
		}
#ifdef VERBOSE 
		printf("t = %g  cost = %g   move = %d\n",t, cost, tot_iter);
#endif
	}

	//printf("the timing %g, delay %g\n", timing_cost, delay_cost);

	tot_iter += move_lim;
	success_rat = ((float) success_sum) / move_lim;
	if (success_sum == 0) {
		av_cost = cost;
		av_bb_cost = bb_cost;
		av_delay_cost = delay_cost;
		av_timing_cost = timing_cost;
	} else {
		av_cost /= success_sum;
		av_bb_cost /= success_sum;
		av_delay_cost /= success_sum;
		av_timing_cost /= success_sum;
	}

	std_dev = get_std_dev(success_sum, sum_of_squares, av_cost);

#ifndef SPEC
	printf(
			"%11.5g  %10.6g %11.6g  %11.6g  %11.6g %11.6g %11.4g %9.4g %8.3g  %7.4g  %7.4g  %10d  \n\n",
			t, av_cost, av_bb_cost, av_timing_cost, av_delay_cost,
			place_delay_value, d_max, success_rat, std_dev, rlim, crit_exponent,
			tot_iter);

#endif

#ifdef VERBOSE
	if(!is_folding)
	dump_clbs();
	else
	dump_stage_clbs();
#endif
	printf("here timing %g, delay %g, block %g\n", timing_cost, delay_cost,
			bb_cost);
	check_place(bb_cost, timing_cost, placer_opts.place_cost_type,
			placer_opts.num_regions, placer_opts.place_algorithm, delay_cost);

	if (placer_opts.enable_timing_computations
			&& placer_opts.place_algorithm == BOUNDING_BOX_PLACE) {
		/*need this done since the timing data has not been kept up to date*
		 *in bounding_box mode */
		for (inet = 0; inet < num_nets; inet++)
			for (ipin = 1; ipin < net[inet].num_pins; ipin++)
				timing_place_crit[inet][ipin] = 0; /*dummy crit values*/
		comp_td_costs(&timing_cost, &delay_cost); /*computes point_to_point_delay_cost*/
	}

	if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
			|| placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
			|| placer_opts.enable_timing_computations) {
		net_delay = point_to_point_delay_cost;/*this makes net_delay up to date with    *
		 *the same values that the placer is using*/
		load_timing_graph_net_delays(net_delay);
		est_crit = load_net_slack(net_slack, 0, FALSE);
#ifdef PRINT_SINK_DELAYS
		print_sink_delays("Placement_Sink_Delays.echo");
#endif
#ifdef PRINT_NET_SLACKS
		print_net_slack("Placement_Net_Slacks.echo", net_slack);
#endif
#ifdef PRINT_PLACE_CRIT_PATH
		print_critical_path("Placement_Crit_Path.echo");
#endif
		printf("Placement Estimated Crit Path Delay: %g\n\n", est_crit);
	}

	sprintf(msg,
			"Placement. Cost: %g  bb_cost: %g td_cost: %g Channel Factor: %d d_max: %g",
			cost, bb_cost, timing_cost, width_fac, d_max);
	printf("Placement. Cost: %g  bb_cost: %g  td_cost: %g  delay_cost: %g.\n",
			cost, bb_cost, timing_cost, delay_cost);

	if (!is_folding) {
		update_screen(MINOR, msg, PLACEMENT, FALSE);
	} else {
		for (istage = 1; istage <= num_stage; istage++) {
			current_stage = istage;
			update_screen(MINOR, msg, PLACEMENT, FALSE);
		}
	}

	printf("Total moves attempted: %d.0\n", tot_iter);

	if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
			|| placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
			|| placer_opts.enable_timing_computations) {

		net_delay = remember_net_delay_original_ptr;

		free_placement_structs(placer_opts.place_cost_type,
				placer_opts.num_regions, old_region_occ_x, old_region_occ_y,
				placer_opts);
		free_lookups_and_criticalities(&net_delay, &net_slack);
	}
}

static int count_connections() {
	/*only count non-global connections*/

	int count, inet;

	count = 0;

	for (inet = 0; inet < num_nets; inet++) {

		if (is_global[inet])
			continue;

		count += (net[inet].num_pins - 1);
	}
	return (count);
}

static void compute_net_pin_index_values() {
	/*computes net_pin_index array, this array allows us to quickly*/
	/*find what pin on the net a block pin corresponds to */

	int inet, netpin, blk, iblk, ipin;

	/*initialize values to OPEN */
	for (iblk = 0; iblk < num_blocks; iblk++)
		if (block[iblk].type == DSP) {
			for (ipin = 0; ipin < pins_per_dsp; ipin++)
				net_pin_index[iblk][ipin] = OPEN;
		} else {
			for (ipin = 0; ipin < pins_per_clb; ipin++)
				net_pin_index[iblk][ipin] = OPEN;
		}

	for (inet = 0; inet < num_nets; inet++) {

		if (is_global[inet])
			continue;

		for (netpin = 0; netpin < net[inet].num_pins; netpin++) {
			blk = net[inet].blocks[netpin];
			if (block[blk].type == INPAD)
				net_pin_index[blk][0] = 0; /*there is only one block pin,so it is 0, and it */
			/*is driving the net since this is an INPAD*/
			else if (block[blk].type == OUTPAD)
				net_pin_index[blk][0] = netpin; /*there is only one block pin,it is 0*/
			else {
				net_pin_index[blk][net[inet].blk_pin[netpin]] = netpin;
			}
		}
	}
}

static double get_std_dev(int n, double sum_x_squared, double av_x) {

	/* Returns the standard deviation of data set x.  There are n sample points, *
	 * sum_x_squared is the summation over n of x^2 and av_x is the average x.   *
	 * All operations are done in double precision, since round off error can be *
	 * a problem in the initial temp. std_dev calculation for big circuits.      */

	double std_dev;

	if (n <= 1)
		std_dev = 0.;
	else
		std_dev = (sum_x_squared - n * av_x * av_x) / (double) (n - 1);

	if (std_dev > 0.) /* Very small variances sometimes round negative */
		std_dev = sqrt(std_dev);
	else
		std_dev = 0.;

	return (std_dev);
}

static void update_rlim(float *rlim, float success_rat) {

	/* Update the range limited to keep acceptance prob. near 0.44.  Use *
	 * a floating point rlim to allow gradual transitions at low temps.  */

	float upper_lim;

	*rlim = (*rlim) * (1. - 0.44 + success_rat);
	upper_lim = max(nx,ny);
	*rlim = min(*rlim,upper_lim);
	*rlim = max(*rlim,1.);
	/* *rlim = (float) nx; */
}

static void update_t(float *t, float std_dev, float rlim, float success_rat,
		struct s_annealing_sched annealing_sched) {

	/* Update the temperature according to the annealing schedule selected. */

	/*  float fac; */

	if (annealing_sched.type == USER_SCHED) {
		*t = annealing_sched.alpha_t * (*t);
	}

	/* Old standard deviation based stuff is below.  This bogs down hopelessly *
	 * for big circuits (alu4 and especially bigkey_mod).                      */
	/* #define LAMBDA .7  */
	/* ------------------------------------ */
	/* else if (std_dev == 0.) {
	 *t = 0.;
	 }
	 else {
	 fac = exp (-LAMBDA*(*t)/std_dev);
	 fac = max(0.5,fac);
	 *t = (*t) * fac;
	 }   */
	/* ------------------------------------- */

	else { /* AUTO_SCHED */
		if (success_rat > 0.96) {
			*t = (*t) * 0.5;
		} else if (success_rat > 0.8) {
			*t = (*t) * 0.9;
		} else if (success_rat > 0.15 || rlim > 1.) {
			*t = (*t) * 0.95;
		} else {
			*t = (*t) * 0.8;
		}
	}
}

static int exit_crit(float t, float cost,
		struct s_annealing_sched annealing_sched) {

	/* Return 1 when the exit criterion is met.                        */

	if (annealing_sched.type == USER_SCHED) {
		if (t < annealing_sched.exit_t) {
			return (1);
		} else {
			return (0);
		}
	}

	/* Automatic annealing schedule */

	if (t < 0.05 * cost / num_nets) {
		return (1);
	} else {
		return (0);
	}
}

static float starting_t(float *cost_ptr, float *bb_cost_ptr,
		float *timing_cost_ptr, int *pins_on_block, int place_cost_type,
		float **old_region_occ_x, float **old_region_occ_y, int num_regions,
		boolean fixed_pins, struct s_annealing_sched annealing_sched,
		int max_moves, float rlim, enum e_place_algorithm place_algorithm,
		float timing_tradeoff, float inverse_prev_bb_cost,
		float inverse_prev_timing_cost, float *delay_cost_ptr) {

	/* Finds the starting temperature (hot condition).              */

	int i, num_accepted, move_lim;
	double std_dev, av, sum_of_squares; /* Double important to avoid round off */
	// printf("come here\n");
	if (annealing_sched.type == USER_SCHED)
		return (annealing_sched.init_t);

	if (!is_folding)
		move_lim = min (max_moves, num_blocks);
	else
		move_lim = min (max_moves, num_blocks/num_stage);

	num_accepted = 0;
	av = 0.;
	sum_of_squares = 0.;

	/* Try one move per block.  Set t high so essentially all accepted. */

	for (i = 0; i < move_lim; i++) {
		if (try_swap(1.e30, cost_ptr, bb_cost_ptr, timing_cost_ptr, rlim,
				pins_on_block, place_cost_type, old_region_occ_x,
				old_region_occ_y, num_regions, fixed_pins, place_algorithm,
				timing_tradeoff, inverse_prev_bb_cost, inverse_prev_timing_cost,
				delay_cost_ptr) == 1) {
			num_accepted++;
			av += *cost_ptr;
			sum_of_squares += *cost_ptr * (*cost_ptr);
		}
	}

	/* Initial Temp = 20*std_dev. */

	if (num_accepted != 0)
		av /= num_accepted;
	else
		av = 0.;

	std_dev = get_std_dev(num_accepted, sum_of_squares, av);

#ifdef DEBUG
	if (num_accepted != move_lim) {
		printf("Warning:  Starting t: %d of %d configurations accepted.\n",
				num_accepted, move_lim);
	}
#endif

#ifdef VERBOSE
	printf("std_dev: %g, average cost: %g, starting temp: %g\n",
			std_dev, av, 20. * std_dev);
#endif

	return (20. * std_dev);
	/* return (15.225523	); */
}

static int try_swap(float t, float *cost, float *bb_cost, float *timing_cost,
		float rlim, int *pins_on_block, int place_cost_type,
		float **old_region_occ_x, float **old_region_occ_y, int num_regions,
		boolean fixed_pins, enum e_place_algorithm place_algorithm,
		float timing_tradeoff, float inverse_prev_bb_cost,
		float inverse_prev_timing_cost, float *delay_cost) {

	/* Picks some block and moves it to another spot.  If this spot is   *
	 * occupied, switch the blocks.  Assess the change in cost function  *
	 * and accept or reject the move.  If rejected, return 0.  If        *
	 * accepted return 1.  Pass back the new value of the cost function. *
	 * rlim is the range limiter.  pins_on_block gives the number of     *
	 * pins on each type of block (improves efficiency).                 */

	int b_from, x_to, y_to, x_from, y_from, b_to;
	int off_from, inet, keep_switch, io_num, num_of_pins;
	int num_nets_affected, bb_index;
	float delta_c, bb_delta_c, timing_delta_c, delay_delta_c, newcost;
	static struct s_bb *bb_coord_new = NULL;
	static struct s_bb *bb_edge_new = NULL;
	static int *nets_to_update = NULL, *net_block_moved = NULL;

	struct s_dist {
		int stage;
		int dir;
		int done;
	}*dist_block;
	char *bname;
	struct s_hash *h_ptr;
	int begin, bcount, i, j, k, l, nblk, count, sum, istage, cstage, length,
			currentb;
	boolean add, processed, if_from, if_to, if_out;
	int *bflist, *btlist;
	int *test;

	//nionio added
	boolean pad_swap_success;
	int pad_count;

	if (is_folding)
		check_placement();

	bflist = my_malloc(num_blocks * sizeof(int));
	dist_block = (struct s_dist*) my_malloc(num_blocks * sizeof(struct s_dist));
	btlist = my_malloc(num_blocks * sizeof(int));
	test = my_malloc(num_blocks * sizeof(int));

	/* Allocate the local bb_coordinate storage, etc. only once. */

	if (bb_coord_new == NULL ) {
		bb_coord_new = (struct s_bb *) my_malloc(
				num_nets * sizeof(struct s_bb));
		bb_edge_new = (struct s_bb *) my_malloc(num_nets * sizeof(struct s_bb));
		nets_to_update = (int *) my_malloc(num_nets * sizeof(int));
		net_block_moved = (int *) my_malloc(num_nets * sizeof(int));
	}

	b_from = my_irand(num_blocks - 1);

	/* If the pins are fixed we never move them from their initial    *
	 * random locations.  The code below could be made more efficient *
	 * by using the fact that pins appear first in the block list,    *
	 * but this shouldn't cause any significant slowdown and won't be *
	 * broken if I ever change the parser so that the pins aren't     *
	 * necessarily at the start of the block list.                    */

	if (fixed_pins == TRUE) {
		while (block[b_from].type != CLB || block[b_from].type != MEM) {
			b_from = my_irand(num_blocks - 1);
		}
	}

	x_from = block[b_from].x;
	y_from = block[b_from].y;

	// printf("from x %d y %d with name %s\n", x_from, y_from, block[b_from].name);

	find_to(x_from, y_from, block[b_from].type, rlim, &x_to, &y_to);
	if ((x_from == x_to) && (y_from == y_to))
		return (0);

	//printf(" to x %d y %d \n", x_to, y_to);

	/* Make the switch in order to make computing the new bounding *
	 * box simpler.  If the cost increase is too high, switch them *
	 * back.  (block data structures switched, clbs not switched   *
	 * until success of move is determined.)                       */

	if (!is_folding) {
		if (block[b_from].type == CLB || block[b_from].type == MEM
				|| block[b_from].type == DSP) {
			io_num = -1; /* Don't need, but stops compiler warning. */
			if (clb[x_to][y_to].occ == 1) { /* Occupied -- do a switch */
				b_to = clb[x_to][y_to].u.block;
				block[b_from].x = x_to;
				block[b_from].y = y_to;
				block[b_to].x = x_from;
				block[b_to].y = y_from;
				// printf("to x %d y %d for blk %s\n", x_to, y_to, block[b_to].name);
			} else {
				b_to = EMPTY;
				block[b_from].x = x_to;
				block[b_from].y = y_to;
				//printf("to x %d y %d for empty blk\n", x_to, y_to);
			}
		} else { /* io block was selected for moving */
			io_num = my_irand(io_rat - 1);
			if (io_num >= clb[x_to][y_to].occ) { /* Moving to an empty location */
				b_to = EMPTY;
				block[b_from].x = x_to;
				block[b_from].y = y_to;
				//printf("to x %d y %d for empty blk\n", x_to, y_to);
			} else { /* Swapping two blocks */
				b_to = *(clb[x_to][y_to].u.io_blocks + io_num);
				block[b_to].x = x_from;
				block[b_to].y = y_from;
				block[b_from].x = x_to;
				block[b_from].y = y_to;
				//printf("to x %d y %d for blk %s\n", x_to, y_to, block[b_to].name);
			}
		}
		bflist[0] = b_from;
		btlist[0] = b_to;
		length = 1;
	} else {
		if (block[b_from].type == CLB) {
			count = 0;
			io_num = -1;
			bname = block[b_from].name;
			h_ptr = get_hash_entry(map_table, bname);
			begin = h_ptr->index;
			bcount = h_ptr->count;
			for (k = begin; k < begin + bcount; k++) {
				nblk = blockmap_inf[k];
				cstage = block[nblk].stage;
				dist_block[count].stage = cstage;
				dist_block[count].dir = 1;
				dist_block[count].done = -1;
				count++;
			}
			for (i = 0; i < count; i++) {
				if (dist_block[i].done = -1) {
					cstage = dist_block[i].stage;
					currentb = -1;
					if (dist_block[i].dir == 1) {
						if (stage_clb[cstage - 1][x_to][y_to].occ == 1) {
							currentb =
									stage_clb[cstage - 1][x_to][y_to].u.block;
						}
					}
					if (dist_block[i].dir == 2) {
						if (stage_clb[cstage - 1][x_from][y_from].occ == 1) {
							currentb =
									stage_clb[cstage - 1][x_from][y_from].u.block;
						}
					}
					if (currentb != -1) {
						bname = block[currentb].name;
						h_ptr = get_hash_entry(map_table, bname);
						begin = h_ptr->index;
						bcount = h_ptr->count;
						for (k = begin; k < begin + bcount; k++) {
							nblk = blockmap_inf[k];
							add = TRUE;
							for (j = 0; j < count; j++) {
								if (dist_block[j].stage == block[nblk].stage) {
									add = FALSE;
								}
							}
							if (add) {
								dist_block[count].stage = block[nblk].stage;
								dist_block[count].done = -1;
								if (dist_block[i].dir == 1)
									dist_block[count].dir = 2;
								if (dist_block[i].dir == 2)
									dist_block[count].dir = 1;
								count++;
							}
						}
					}
					dist_block[i].done = 1;
				}
			}
			for (i = 0; i < count; i++) {
				istage = dist_block[i].stage;
				if ((stage_clb[istage - 1][x_from][y_from].occ == 1)
						&& (stage_clb[istage - 1][x_to][y_to].occ == 1)) {
					b_from = stage_clb[istage - 1][x_from][y_from].u.block;
					b_to = stage_clb[istage - 1][x_to][y_to].u.block;
					block[b_from].x = x_to;
					block[b_from].y = y_to;
					block[b_to].x = x_from;
					block[b_to].y = y_from;
				} else if (stage_clb[istage - 1][x_from][y_from].occ == 1) {
					b_from = stage_clb[istage - 1][x_from][y_from].u.block;
					b_to = EMPTY;
					block[b_from].x = x_to;
					block[b_from].y = y_to;
				} else if (stage_clb[istage - 1][x_to][y_to].occ == 1) {
					b_from = EMPTY;
					b_to = stage_clb[istage - 1][x_to][y_to].u.block;
					block[b_to].x = x_from;
					block[b_to].y = y_from;
				} else
					continue;
				bflist[i] = b_from;
				btlist[i] = b_to;
			}
			length = count;
		} else if (block[b_from].type == MEM) {
			sum = 0;
			for (i = 0; i < num_stage; i++) {
				if (stage_clb[i][x_to][y_to].occ == 1
						&& stage_clb[i][x_from][y_from].occ == 1) {
					b_to = stage_clb[i][x_to][y_to].u.block;
					b_from = stage_clb[i][x_from][y_from].u.block;
					block[b_from].x = x_to;
					block[b_from].y = y_to;
					block[b_to].x = x_from;
					block[b_to].y = y_from;
				} else if (stage_clb[i][x_to][y_to].occ == 1) {
					b_to = stage_clb[i][x_to][y_to].u.block;
					b_from = EMPTY;
					block[b_to].x = x_from;
					block[b_to].y = y_from;
				} else if (stage_clb[i][x_from][y_from].occ == 1) {
					b_to = EMPTY;
					b_from = stage_clb[i][x_from][y_from].u.block;
					block[b_from].x = x_to;
					block[b_from].y = y_to;
				} else
					continue;
				bflist[sum] = b_from;
				btlist[sum] = b_to;
				sum++;
			}
			length = sum;
		} else if (block[b_from].type == DSP) {
			sum = 0;
			for (i = 0; i < num_stage; i++) {
				if (stage_clb[i][x_to][y_to].occ == 1
						&& stage_clb[i][x_from][y_from].occ == 1) {
					b_to = stage_clb[i][x_to][y_to].u.block;
					b_from = stage_clb[i][x_from][y_from].u.block;
					block[b_from].x = x_to;
					block[b_from].y = y_to;
					block[b_to].x = x_from;
					block[b_to].y = y_from;
				} else if (stage_clb[i][x_to][y_to].occ == 1) {
					b_to = stage_clb[i][x_to][y_to].u.block;
					b_from = EMPTY;
					block[b_to].x = x_from;
					block[b_to].y = y_from;
				} else if (stage_clb[i][x_from][y_from].occ == 1) {
					b_to = EMPTY;
					b_from = stage_clb[i][x_from][y_from].u.block;
					block[b_from].x = x_to;
					block[b_from].y = y_to;
				} else
					continue;
				bflist[sum] = b_from;
				btlist[sum] = b_to;
				sum++;
			}
			length = sum;
		} else {
			/* io block was selected for moving */
			//printf("swap pad from %d \n", b_from);
			io_num = my_irand(io_rat - 1);
			//printf("to pad slot %d\n", io_num);
			sum = 0;
			if_out = FALSE;
			for (off_from = 0; off_from < stage_clb[0][x_from][y_from].occ;
					off_from++) {
				for (istage = 0; istage < num_stage; istage++) {
					if (stage_clb[istage][x_from][y_from].u.io_blocks[off_from]
							== b_from) {
						if_out = TRUE;
						break;
					}
				}
				if (if_out)
					break;
			}
			if (!if_out) {
				printf("error: cannot find from block\n");
				exit(1);
			}
			if (io_num >= stage_clb[0][x_to][y_to].occ) { /* Moving to an empty location */
				//check stage_io_rat first
				pad_swap_success = TRUE;
				for (i = 0; i < num_stage; i++) {
					if (*(stage_clb[i][x_from][y_from].u.io_blocks + off_from)
							!= EMPTY) {
						pad_count = 0;
						for (j = 0; j < stage_clb[0][x_to][y_to].occ; j++) {
							if (stage_clb[i][x_to][y_to].u.io_blocks[j] != EMPTY)
								pad_count++;
						}
						if (pad_count == stage_io_rat) {
							pad_swap_success = FALSE;
							break;
						}
					}
				}
				if (pad_swap_success) {
					for (i = 0; i < num_stage; i++) {
						if (*(stage_clb[i][x_from][y_from].u.io_blocks
								+ off_from) != EMPTY) {
							b_from = *(stage_clb[i][x_from][y_from].u.io_blocks
									+ off_from);
							b_to = EMPTY;
							block[b_from].x = x_to;
							block[b_from].y = y_to;
							//printf("b_from:%d in stage %d\n", b_from, i);
							bflist[sum] = b_from;
							btlist[sum] = b_to;
							sum++;
						}
					}
				} else {
					free(test);
					free(dist_block);
					free(btlist);
					free(bflist);
					return;
				}
			}

			else { /*Moving to an occupied location */
				//check stage_io_rat first (to)
				pad_swap_success = TRUE;
				for (i = 0; i < num_stage; i++) {
					if (*(stage_clb[i][x_from][y_from].u.io_blocks + off_from)
							!= EMPTY) {
						b_to = *(stage_clb[i][x_to][y_to].u.io_blocks + io_num);
						pad_count = 0;
						for (j = 0; j < stage_clb[0][x_to][y_to].occ; j++) {
							if ((stage_clb[i][x_to][y_to].u.io_blocks[j]
									!= EMPTY)
									&& (stage_clb[i][x_to][y_to].u.io_blocks[j]
											!= b_to))
								pad_count++;
						}
						if (pad_count == stage_io_rat) {
							pad_swap_success = FALSE;
							break;
						}
					}
				}
				//check stage_io_rat first (from)
				if (pad_swap_success) {
					for (i = 0; i < num_stage; i++) {
						if (*(stage_clb[i][x_to][y_to].u.io_blocks + io_num)
								!= EMPTY) {
							b_from = *(stage_clb[i][x_from][y_from].u.io_blocks
									+ off_from);
							pad_count = 0;
							for (j = 0; j < stage_clb[0][x_from][y_from].occ;
									j++) {
								if ((stage_clb[i][x_from][y_from].u.io_blocks[j]
										!= EMPTY)
										&& (stage_clb[i][x_from][y_from].u.io_blocks[j]
												!= b_from))
									pad_count++;
							}
							if (pad_count == stage_io_rat) {
								pad_swap_success = FALSE;
								break;
							}
						}
					}
				}
				if (!pad_swap_success) {
					free(test);
					free(dist_block);
					free(btlist);
					free(bflist);
					return;
				} else {
					for (i = 0; i < num_stage; i++) {
						b_from = *(stage_clb[i][x_from][y_from].u.io_blocks
								+ off_from);
						b_to = *(stage_clb[i][x_to][y_to].u.io_blocks + io_num);
						if ((b_from != EMPTY) && (b_to != EMPTY)) {
							block[b_to].x = x_from;
							block[b_to].y = y_from;
							block[b_from].x = x_to;
							block[b_from].y = y_to;
						} else if (b_from != EMPTY) {
							block[b_from].x = x_to;
							block[b_from].y = y_to;
						} else if (b_to != EMPTY) {
							block[b_to].x = x_from;
							block[b_to].y = y_from;
						} else
							continue;
						//printf("b_from:%d in stage %d\n", b_from, i);
						bflist[sum] = b_from;
						btlist[sum] = b_to;
						sum++;
					}
				}
			}
			length = sum;
		}
	}
	free(test);
	free(dist_block);
	/* Now update the cost function.  May have to do major optimizations *
	 * here later.                                                       */

	/* I'm using negative values of temp_net_cost as a flag, so DO NOT   *
	 * use cost functions that can go negative.                          */

	delta_c = 0; /* Change in cost due to this swap. */
	bb_delta_c = 0;
	timing_delta_c = 0;

	if (is_folding) {
		for (i = 0; i < length; i++) {
			if (bflist[i] != EMPTY) {
				num_of_pins = pins_on_block[block[bflist[i]].type];
				break;
			}
			if (btlist[i] != EMPTY) {
				num_of_pins = pins_on_block[block[btlist[i]].type];
				break;
			}
		}
	} else
		num_of_pins = pins_on_block[block[b_from].type];

	num_nets_affected = find_affected_nets(nets_to_update, net_block_moved,
			bflist, btlist, num_of_pins, length);

	if (place_cost_type == NONLINEAR_CONG) {
		save_region_occ(old_region_occ_x, old_region_occ_y, num_regions);
	}

	bb_index = 0; /* Index of new bounding box. */

	for (k = 0; k < num_nets_affected; k++) {
		inet = nets_to_update[k];

		/* If we swapped two blocks connected to the same net, its bounding box *
		 * doesn't change.                                                      */

		if (net_block_moved[k] == FROM_AND_TO)
			continue;
		if (net[inet].num_pins <= SMALL_NET)
			get_non_updateable_bb(inet, &bb_coord_new[bb_index]);

		else {
			if (net_block_moved[k] == FROM)
				update_bb(inet, &bb_coord_new[bb_index], &bb_edge_new[bb_index],
						x_from, y_from, x_to, y_to);
			else
				update_bb(inet, &bb_coord_new[bb_index], &bb_edge_new[bb_index],
						x_to, y_to, x_from, y_from);
		}

		if (place_cost_type != NONLINEAR_CONG) {
			temp_net_cost[inet] = get_net_cost(inet, &bb_coord_new[bb_index]);
			bb_delta_c += temp_net_cost[inet] - net_cost[inet];
		} else {
			/* Rip up, then replace with new bb. */
			update_region_occ(inet, &bb_coords[inet], -1, num_regions);
			update_region_occ(inet, &bb_coord_new[bb_index], 1, num_regions);
		}

		bb_index++;
	}

	if (place_cost_type == NONLINEAR_CONG) {
		newcost = nonlinear_cong_cost(num_regions);
		bb_delta_c = newcost - *bb_cost;
	}

	if (place_algorithm == NET_TIMING_DRIVEN_PLACE
			|| place_algorithm == PATH_TIMING_DRIVEN_PLACE) {
		/*in this case we redefine delta_c as a combination of timing and bb.  *
		 *additionally, we normalize all values, therefore delta_c is in       *
		 *relation to 1*/

		comp_delta_td_cost(bflist, btlist, num_of_pins, &timing_delta_c,
				&delay_delta_c, length);
		//printf("The delay %11.6g, timing %11.6g \n",delay_delta_c, timing_delta_c);
		delta_c = (1 - timing_tradeoff) * bb_delta_c * inverse_prev_bb_cost
				+ timing_tradeoff * timing_delta_c * inverse_prev_timing_cost;
	} else {
		delta_c = bb_delta_c;
	}

	keep_switch = assess_swap(delta_c, t);

	/* 1 -> move accepted, 0 -> rejected. */

	if (keep_switch) {
		*cost = *cost + delta_c;
		*bb_cost = *bb_cost + bb_delta_c;

		if (place_algorithm == NET_TIMING_DRIVEN_PLACE
				|| place_algorithm == PATH_TIMING_DRIVEN_PLACE) {
			/*update the point_to_point_timing_cost and point_to_point_delay_cost
			 values from the temporary values*/
			*timing_cost = *timing_cost + timing_delta_c;
			*delay_cost = *delay_cost + delay_delta_c;

			update_td_cost(bflist, btlist, num_of_pins, length);
		}

		/* update net cost functions and reset flags. */

		bb_index = 0;

		for (k = 0; k < num_nets_affected; k++) {
			inet = nets_to_update[k];

			/* If we swapped two blocks connected to the same net, its bounding box *
			 * doesn't change.                                                      */

			if (net_block_moved[k] == FROM_AND_TO) {
				temp_net_cost[inet] = -1;
				continue;
			}

			bb_coords[inet] = bb_coord_new[bb_index];
			if (net[inet].num_pins > SMALL_NET)
				bb_num_on_edges[inet] = bb_edge_new[bb_index];

			bb_index++;

			net_cost[inet] = temp_net_cost[inet];
			temp_net_cost[inet] = -1;
		}

		/* Update Clb data structures since we kept the move. */
		if (!is_folding) {
			if (block[b_from].type == CLB || block[b_from].type == MEM) {
				if (b_to != EMPTY) {
					clb[x_from][y_from].u.block = b_to;
					clb[x_to][y_to].u.block = b_from;
				} else {
					clb[x_to][y_to].u.block = b_from;
					clb[x_to][y_to].occ = 1;
					clb[x_from][y_from].occ = 0;
				}
			}

			else { /* io block was selected for moving */

				/* Get the "sub_block" number of the b_from block. */

				for (off_from = 0;; off_from++) {
					if (clb[x_from][y_from].u.io_blocks[off_from] == b_from)
						break;
				}

				if (b_to != EMPTY) { /* Swapped two blocks. */
					clb[x_to][y_to].u.io_blocks[io_num] = b_from;
					clb[x_from][y_from].u.io_blocks[off_from] = b_to;
				} else { /* Moved to an empty location */
					clb[x_to][y_to].u.io_blocks[clb[x_to][y_to].occ] = b_from;
					clb[x_to][y_to].occ++;
					for (k = off_from; k < clb[x_from][y_from].occ - 1; k++) { /* prevent gap  */
						clb[x_from][y_from].u.io_blocks[k] = /* in io_blocks */
						clb[x_from][y_from].u.io_blocks[k + 1];
					}
					clb[x_from][y_from].occ--;
				}
			}
		} else { //for folding case
			if (clb[x_from][y_from].type == CLB
					|| clb[x_from][y_from].type == MEM
					|| clb[x_from][y_from].type == DSP) {
				// printf("length is %d\n", length);
				for (i = 0; i < length; i++) {
					b_from = bflist[i];
					b_to = btlist[i];
					//printf("the bfrom %d\n", b_from);
					//printf("the bto %d\n", b_to);
					if ((b_to != EMPTY) && (b_from != EMPTY)) {
						cstage = block[b_from].stage;
						stage_clb[cstage - 1][x_from][y_from].u.block = b_to;
						stage_clb[cstage - 1][x_to][y_to].u.block = b_from;
					} else if (b_from != EMPTY) {
						cstage = block[b_from].stage;
						stage_clb[cstage - 1][x_to][y_to].u.block = b_from;
						stage_clb[cstage - 1][x_to][y_to].occ = 1;
						stage_clb[cstage - 1][x_from][y_from].occ = 0;
					} else if (b_to != EMPTY) {
						cstage = block[b_to].stage;
						stage_clb[cstage - 1][x_from][y_from].u.block = b_to;
						stage_clb[cstage - 1][x_from][y_from].occ = 1;
						stage_clb[cstage - 1][x_to][y_to].occ = 0;
					}
				}
			} else { // for IO
				if_from = FALSE;
				if_to = FALSE;
				int from_blk;
				for (i = 0; i < length; i++) {
					b_from = bflist[i];
					b_to = btlist[i];
					if (b_from != EMPTY) {
						if_from = TRUE;
						from_blk = b_from;
						// printf("b_from %d(%s)\n", from_blk, block[from_blk].name);
					}
					if (b_to != EMPTY)
						if_to = TRUE;
					//printf("from_x %d, from_y %d\n", x_from, y_from);
				}
				if_out = FALSE;
				if (if_from && if_to) {
					for (off_from = 0;
							off_from < stage_clb[0][x_from][y_from].occ;
							off_from++) {
						for (istage = 0; istage < num_stage; istage++)
							if (stage_clb[istage][x_from][y_from].u.io_blocks[off_from]
									== from_blk) { //printf("sub pad %d \n", off_from);
								if_out = TRUE;
								break;
							}
						if (if_out)
							break;
					}
					if (!if_out) {
						printf("error: cannot find from block\n");
						exit(1);
					}
					for (j = 0; j < num_stage; j++) {
						int temp_from =
								stage_clb[j][x_from][y_from].u.io_blocks[off_from];
						int temp_to =
								stage_clb[j][x_to][y_to].u.io_blocks[io_num];
						stage_clb[j][x_to][y_to].u.io_blocks[io_num] =
								temp_from;
						stage_clb[j][x_from][y_from].u.io_blocks[off_from] =
								temp_to;
					}
				} else if (if_from) { //printf("empty to blk\n");
					for (off_from = 0;; off_from++) {
						for (istage = 0; istage < num_stage; istage++)
							if (stage_clb[istage][x_from][y_from].u.io_blocks[off_from]
									== from_blk) { //printf("sub pad %d \n", off_from);
								if_out = TRUE;
								break;
							}
						if (if_out)
							break;
					}
					for (j = 0; j < num_stage; j++)
						stage_clb[j][x_to][y_to].u.io_blocks[stage_clb[0][x_to][y_to].occ] =
								stage_clb[j][x_from][y_from].u.io_blocks[off_from];

					for (j = 0; j < num_stage; j++)
						stage_clb[j][x_to][y_to].occ++;
					int num_occ = stage_clb[0][x_from][y_from].occ;
					for (j = 0; j < num_stage; j++) {
						for (k = off_from; k < num_occ - 1; k++) { /* prevent gap  */
							stage_clb[j][x_from][y_from].u.io_blocks[k] = /* in io_blocks */
							stage_clb[j][x_from][y_from].u.io_blocks[k + 1];
						}
						stage_clb[j][x_from][y_from].occ--;
					}
				} else {
					printf("error! from block empty\n");
					exit(1);
				}
			}
		}
	}

	else { /* Move was rejected.  */
		/* Reset the net cost function flags first. */
		for (k = 0; k < num_nets_affected; k++) {
			inet = nets_to_update[k];
			temp_net_cost[inet] = -1;
		}

		/* Restore the block data structures to their state before the move. */
		if (!is_folding) {
			block[b_from].x = x_from;
			block[b_from].y = y_from;
			if (b_to != EMPTY) {
				block[b_to].x = x_to;
				block[b_to].y = y_to;
			}
		} else {
			for (i = 0; i < length; i++) {
				b_from = bflist[i];
				b_to = btlist[i];
				if (b_from != EMPTY) {
					block[b_from].x = x_from;
					block[b_from].y = y_from;
				}
				if (b_to != EMPTY) {
					block[b_to].x = x_to;
					block[b_to].y = y_to;
				}
			}
		}

		/* Restore the region occupancies to their state before the move. */
		if (place_cost_type == NONLINEAR_CONG) {
			restore_region_occ(old_region_occ_x, old_region_occ_y, num_regions);
		}
	}

	free(bflist);
	free(btlist);
	// printf("come here\n");
	return (keep_switch);
}

static void save_region_occ(float **old_region_occ_x, float **old_region_occ_y,
		int num_regions) {

	/* Saves the old occupancies of the placement subregions in case the  *
	 * current move is not accepted.  Used only for NONLINEAR_CONG.       */

	int i, j;

	for (i = 0; i < num_regions; i++) {
		for (j = 0; j < num_regions; j++) {
			old_region_occ_x[i][j] = place_region_x[i][j].occupancy;
			old_region_occ_y[i][j] = place_region_y[i][j].occupancy;
		}
	}
}

static void restore_region_occ(float **old_region_occ_x,
		float **old_region_occ_y, int num_regions) {

	/* Restores the old occupancies of the placement subregions when the  *
	 * current move is not accepted.  Used only for NONLINEAR_CONG.       */

	int i, j;

	for (i = 0; i < num_regions; i++) {
		for (j = 0; j < num_regions; j++) {
			place_region_x[i][j].occupancy = old_region_occ_x[i][j];
			place_region_y[i][j].occupancy = old_region_occ_y[i][j];
		}
	}
}

static int find_affected_nets(int *nets_to_update, int *net_block_moved,
		int *b_from, int *b_to, int num_of_pins, int length) {

	/* Puts a list of all the nets connected to b_from and b_to into          *
	 * nets_to_update.  Returns the number of affected nets.  Net_block_moved *
	 * is either FROM, TO or FROM_AND_TO -- the block connected to this net   *
	 * that has moved.                                                        */

	int k, inet, affected_index, count;
	int pair, cb_from, cb_to;
	affected_index = 0;

	for (pair = 0; pair < length; pair++) {
		cb_from = b_from[pair];
		cb_to = b_to[pair];
		if (cb_from != EMPTY) {
			// printf("block %d(%s)\n", cb_from, block[cb_from].name);
			for (k = 0; k < num_of_pins; k++) {
				inet = block[cb_from].nets[k];

				if (inet == OPEN)
					continue;

				if (is_global[inet]) { //printf("global net\n");
					continue;
				}

				// printf("net %d %d(%s)\n", k, inet, net[inet].name);
				/* This is here in case the same block connects to a net twice. */

				if (temp_net_cost[inet] > 0.)
					continue;

				nets_to_update[affected_index] = inet;
				net_block_moved[affected_index] = FROM;
				affected_index++;
				temp_net_cost[inet] = 1.; /* Flag to say we've marked this net. */
			}

			if (cb_to != EMPTY) {
				//printf("block %d(%s)\n", cb_to, block[cb_to].name);

				for (k = 0; k < num_of_pins; k++) {
					inet = block[cb_to].nets[k];

					if (inet == OPEN)
						continue;

					if (is_global[inet])
						continue;
					// printf("net %d %d(%s)\n", k, inet, net[inet].name);
					if (temp_net_cost[inet] > 0.) { /* Net already marked. */
						for (count = 0; count < affected_index; count++) {
							if (nets_to_update[count] == inet) {
								if (net_block_moved[count] == FROM)
									net_block_moved[count] = FROM_AND_TO;
								break;
							}
						}

#ifdef DEBUG
						if (count > affected_index) {
							printf("Error in find_affected_nets -- count = %d,"
									" affected index = %d.\n", count,
									affected_index);
							exit(1);
						}
#endif
					}

					else { /* Net not marked yet. */
						nets_to_update[affected_index] = inet;
						net_block_moved[affected_index] = TO;
						affected_index++;
						temp_net_cost[inet] = 1.; /* Flag means we've  marked net. */
					}
				}
			}
		} else { /* the from block is empty */
			for (k = 0; k < num_of_pins; k++) {
				inet = block[cb_to].nets[k];

				if (inet == OPEN)
					continue;

				if (is_global[inet])
					continue;

				/* This is here in case the same block connects to a net twice. */

				if (temp_net_cost[inet] > 0.)
					continue;
				// printf("net %d %d(%s)\n", k, inet, net[inet].name);
				nets_to_update[affected_index] = inet;
				net_block_moved[affected_index] = TO;
				affected_index++;
				temp_net_cost[inet] = 1.; /* Flag to say we've marked this net. */
			}
		}
	}
	return (affected_index);
}

static void find_to(int x_from, int y_from, int type, float rlim, int *x_to,
		int *y_to) {

	/* Returns the point to which I want to swap, properly range limited. *
	 * rlim must always be between 1 and nx (inclusive) for this routine  *
	 * to work.                                                           */

	int x_rel, y_rel, iside, iplace, rlx, rly, count = 0;
	int dsp_x_rel, dsp_y_rel, dsp_rlx, dsp_rly;

	dsp_rlx = min(nx / dsp_loc_repeat + 1, rlim / dsp_loc_repeat + 1);
	dsp_rly = min(ny / dsp_loc_repeat + 1, rlim / dsp_loc_repeat + 1);

	rlx = min(nx,rlim); /* Only needed when nx < ny. */
	rly = min(ny,rlim); /* Added rly for aspect_ratio != 1 case. */
	//printf("rlim %d, rlx %d, rly %d\n", rlim, rlx, rly);

#ifdef DEBUG
	if (rlx < 1 || rlx > nx) {
		printf("Error in find_to: rlx = %d\n", rlx);
		exit(1);
	}
#endif

	do { /* Until (x_to, y_to) different from (x_from, y_from) */
		count++;
		if (type == CLB || type == MEM) {
			do {
				x_rel = my_irand(2 * rlx);
				y_rel = my_irand(2 * rly);
				*x_to = x_from - rlx + x_rel;
				*y_to = y_from - rly + y_rel;
				if (*x_to > nx)
					*x_to = *x_to - nx; /* better spectral props. */
				if (*x_to < 1)
					*x_to = *x_to + nx; /* than simple min, max   */
				if (*y_to > ny)
					*y_to = *y_to - ny; /* clipping.              */
				if (*y_to < 1)
					*y_to = *y_to + ny;
				// printf("rlx %d, rly %d\n", rlx, rly);
				//printf("x_from %d, y_from %d, x_rel %d, y_rel %d, x_to %d, y_to %d\n", x_from, y_from, x_rel, y_rel, *x_to, *y_to);
			} while (clb[*x_to][*y_to].type != type);

			// printf("final rlx %d, rly %d, x_rel %d, y_rel %d, x_to %d, y_to %d\n", rlx, rly, x_rel, y_rel, *x_to, *y_to);
		} else if (type == DSP) {
			do {
				dsp_x_rel = my_irand(2 * dsp_rlx);
				dsp_y_rel = my_irand(2 * dsp_rlx);
				*x_to = x_from - (dsp_rlx - dsp_x_rel) * dsp_loc_repeat;
				*y_to = y_from - (dsp_rly - dsp_y_rel) * dsp_h;
				if (*x_to > nx)
					*x_to = dsp_loc_start;
				if (*x_to < 1)
					*x_to = nx - (nx - dsp_loc_start + 1) % dsp_loc_repeat;
				if (*y_to < 1)
					*y_to = ny - ny % dsp_h;
				if (*y_to > ny)
					*y_to = 1;
			} while (!(clb[*x_to][*y_to].type == DSP
					&& clb[*x_to][*y_to].x_off == 0
					&& clb[*x_to][*y_to].y_off == 0));
		} else { /* io_block to be moved. */
			if (rlx >= nx) {
				iside = my_irand(3);
				/*                              *
				 *       +-----1----+           *
				 *       |          |           *
				 *       |          |           *
				 *       0          2           *
				 *       |          |           *
				 *       |          |           *
				 *       +-----3----+           *
				 *                              */
				switch (iside) {
				case 0:
					iplace = my_irand(ny - 1) + 1;
					*x_to = 0;
					*y_to = iplace;
					break;
				case 1:
					iplace = my_irand(nx - 1) + 1;
					*x_to = iplace;
					*y_to = ny + 1;
					break;
				case 2:
					iplace = my_irand(ny - 1) + 1;
					*x_to = nx + 1;
					*y_to = iplace;
					break;
				case 3:
					iplace = my_irand(nx - 1) + 1;
					*x_to = iplace;
					*y_to = 0;
					break;
				default:
					printf("Error in find_to.  Unexpected io swap location.\n");
					exit(1);
				}
			} else { /* rlx is less than whole chip */
				if (x_from == 0) {
					iplace = my_irand(2 * rly);
					*y_to = y_from - rly + iplace;
					*x_to = x_from;
					if (*y_to > ny) {
						*y_to = ny + 1;
						*x_to = my_irand(rlx - 1) + 1;
					} else if (*y_to < 1) {
						*y_to = 0;
						*x_to = my_irand(rlx - 1) + 1;
					}
				} else if (x_from == nx + 1) {
					iplace = my_irand(2 * rly);
					*y_to = y_from - rly + iplace;
					*x_to = x_from;
					if (*y_to > ny) {
						*y_to = ny + 1;
						*x_to = nx - my_irand(rlx - 1);
					} else if (*y_to < 1) {
						*y_to = 0;
						*x_to = nx - my_irand(rlx - 1);
					}
				} else if (y_from == 0) {
					iplace = my_irand(2 * rlx);
					*x_to = x_from - rlx + iplace;
					*y_to = y_from;
					if (*x_to > nx) {
						*x_to = nx + 1;
						*y_to = my_irand(rly - 1) + 1;
					} else if (*x_to < 1) {
						*x_to = 0;
						*y_to = my_irand(rly - 1) + 1;
					}
				} else { /* *y_from == ny + 1 */
					iplace = my_irand(2 * rlx);
					*x_to = x_from - rlx + iplace;
					*y_to = y_from;
					if (*x_to > nx) {
						*x_to = nx + 1;
						*y_to = ny - my_irand(rly - 1);
					} else if (*x_to < 1) {
						*x_to = 0;
						*y_to = ny - my_irand(rly - 1);
					}
				}
			} /* End rlx if */
		} /* end type if */
	} while ((x_from == *x_to) && (y_from == *y_to) && (count < 100));

#ifdef DEBUG
	if (*x_to < 0 || *x_to > nx + 1 || *y_to < 0 || *y_to > ny + 1) {
		printf("Error in routine find_to:  (x_to,y_to) = (%d,%d)\n", *x_to,
				*y_to);
		exit(1);
	}

	if (type == CLB) {
		if (clb[*x_to][*y_to].type != CLB) {
			printf("Error: Moving CLB to illegal type block at (%d,%d)\n",
					*x_to, *y_to);
			exit(1);
		}
	} else if (type == MEM) {
		if (clb[*x_to][*y_to].type != MEM) {
			printf("Error: Moving CLB to illegal type block at (%d,%d)\n",
					*x_to, *y_to);
			exit(1);
		}
	} else if (type == DSP) {
		if (clb[*x_to][*y_to].type != DSP) {
			printf("Error: Moving DSP to illegal type block at (%d,%d)\n",
					*x_to, *y_to);
			exit(1);
		}
	} else if (clb[*x_to][*y_to].type != IO) {
		printf("Error: Moving IO block to illegal type location at "
				"(%d,%d)\n", *x_to, *y_to);
		exit(1);
	}
}
#endif

/* printf("(%d,%d) moved to (%d,%d)\n",x_from,y_from,*x_to,*y_to); */

static int assess_swap(float delta_c, float t) {

	/* Returns: 1 -> move accepted, 0 -> rejected. */

	int accept;
	float prob_fac, fnum;

	if (delta_c <= 0) {

#ifdef SPEC          /* Reduce variation in final solution due to round off */
		fnum = my_frand();
#endif

		accept = 1;
		return (accept);
	}

	if (t == 0.)
		return (0);

	fnum = my_frand();
	prob_fac = exp(-delta_c / t);
	if (prob_fac > fnum) {
		accept = 1;
	} else {
		accept = 0;
	}
	return (accept);
}

static float recompute_bb_cost(int place_cost_type, int num_regions) {

	/* Recomputes the cost to eliminate roundoff that may have accrued.  *
	 * This routine does as little work as possible to compute this new  *
	 * cost.                                                             */

	int i, j, inet;
	float cost;

	cost = 0;

	/* Initialize occupancies to zero if regions are being used. */

	if (place_cost_type == NONLINEAR_CONG) {
		for (i = 0; i < num_regions; i++) {
			for (j = 0; j < num_regions; j++) {
				place_region_x[i][j].occupancy = 0.;
				place_region_y[i][j].occupancy = 0.;
			}
		}
	}

	for (inet = 0; inet < num_nets; inet++) { /* for each net ... */

		if (is_global[inet] == FALSE) { /* Do only if not global. */

			/* Bounding boxes don't have to be recomputed; they're correct. */

			if (place_cost_type != NONLINEAR_CONG) {
				cost += net_cost[inet];
			} else { /* Must be nonlinear_cong case. */
				update_region_occ(inet, &bb_coords[inet], 1, num_regions);
			}
		}
	}

	if (place_cost_type == NONLINEAR_CONG) {
		cost = nonlinear_cong_cost(num_regions);
	}

	return (cost);
}

static float comp_td_point_to_point_delay(int inet, int ipin) {

	/*returns the delay of one point to point connection */

	int source_block, sink_block;
	int delta_x, delta_y;
	enum e_block_types source_type, sink_type;
	float delay_source_to_sink;

	delay_source_to_sink = 0.;

	source_block = net[inet].blocks[0];
	source_type = block[source_block].type;
//printf("source block %s\n", block[source_block].name);

	sink_block = net[inet].blocks[ipin];
	sink_type = block[sink_block].type;
//printf("sink block %s\n", block[sink_block].name);

	delta_x = abs(block[sink_block].x - block[source_block].x);
	delta_y = abs(block[sink_block].y - block[source_block].y);
//printf("delta x %d, y %d \n", delta_x, delta_y);

	if (source_type == CLB || source_type == MEM || source_type == DSP) {
		if (sink_type == CLB || sink_type == MEM || sink_type == DSP)
			delay_source_to_sink = delta_clb_to_clb[delta_x][delta_y];
		else if (sink_type == OUTPAD)
			delay_source_to_sink = delta_clb_to_outpad[delta_x][delta_y];
		else {
			printf(
					"Error in comp_td_point_to_point_delay in place.c, bad sink_type\n");
			exit(1);
		}
	} else if (source_type == INPAD) {
		if (sink_type == CLB || sink_type == DSP)
			delay_source_to_sink = delta_inpad_to_clb[delta_x][delta_y];
		else if (sink_type == OUTPAD)
			delay_source_to_sink = delta_inpad_to_outpad[delta_x][delta_y];
		else {
			printf(
					"Error in comp_td_point_to_point_delay in place.c, bad sink_type\n");
			exit(1);
		}
	} else {
		printf(
				"Error in comp_td_point_to_point_delay in place.c, bad source_type\n");
		exit(1);
	}
	if (delay_source_to_sink < 0) {
		printf(
				"Error in comp_td_point_to_point_delay in place.c, bad delay_source_to_sink value\n");
		exit(1);
	}

	if (delay_source_to_sink < 0.) {
		printf(
				"Error in comp_td_point_to_point_delay in place.c, delay is less than 0\n");
		exit(1);
	}

	return (delay_source_to_sink);
}

static void update_td_cost(int *b_from, int *b_to, int num_of_pins, int length) {
	/*update the point_to_point_timing_cost values from the temporary */
	/*values for all connections that have changed */

	int blkpin, net_pin, inet, ipin;
	int pair, i, cb_from, cb_to;

	for (pair = 0; pair < length; pair++) {
		cb_from = b_from[pair];
		cb_to = b_to[pair];
		if (cb_from != EMPTY) {
			for (blkpin = 0; blkpin < num_of_pins; blkpin++) {

				inet = block[cb_from].nets[blkpin];

				if (inet == OPEN)
					continue;

				if (is_global[inet])
					continue;

				net_pin = net_pin_index[cb_from][blkpin];

				if (net_pin != 0) {

					/*the following "if" prevents the value from being updated twice*/
					if (net[inet].blocks[0] != cb_to
							&& net[inet].blocks[0] != cb_from) {

						point_to_point_delay_cost[inet][net_pin] =
								temp_point_to_point_delay_cost[inet][net_pin];
						temp_point_to_point_delay_cost[inet][net_pin] = -1;

						point_to_point_timing_cost[inet][net_pin] =
								temp_point_to_point_timing_cost[inet][net_pin];
						temp_point_to_point_timing_cost[inet][net_pin] = -1;
					}
				} else { /*this net is being driven by a moved block, recompute */
					/*all point to point connections on this net.*/
					for (ipin = 1; ipin < net[inet].num_pins; ipin++) {

						point_to_point_delay_cost[inet][ipin] =
								temp_point_to_point_delay_cost[inet][ipin];
						temp_point_to_point_delay_cost[inet][ipin] = -1;

						point_to_point_timing_cost[inet][ipin] =
								temp_point_to_point_timing_cost[inet][ipin];
						temp_point_to_point_timing_cost[inet][ipin] = -1;
					}
				}
			}
		}
		if (cb_to != EMPTY) {
			for (blkpin = 0; blkpin < num_of_pins; blkpin++) {

				inet = block[cb_to].nets[blkpin];

				if (inet == OPEN)
					continue;

				if (is_global[inet])
					continue;

				net_pin = net_pin_index[cb_to][blkpin];

				if (net_pin != 0) {

					/*the following "if" prevents the value from being updated 2x*/
					if (net[inet].blocks[0] != cb_to
							&& net[inet].blocks[0] != cb_from) {

						point_to_point_delay_cost[inet][net_pin] =
								temp_point_to_point_delay_cost[inet][net_pin];
						temp_point_to_point_delay_cost[inet][net_pin] = -1;

						point_to_point_timing_cost[inet][net_pin] =
								temp_point_to_point_timing_cost[inet][net_pin];
						temp_point_to_point_timing_cost[inet][net_pin] = -1;
					}
				} else { /*this net is being driven by a moved block, recompute */
					/*all point to point connections on this net.*/
					for (ipin = 1; ipin < net[inet].num_pins; ipin++) {

						point_to_point_delay_cost[inet][ipin] =
								temp_point_to_point_delay_cost[inet][ipin];
						temp_point_to_point_delay_cost[inet][ipin] = -1;

						point_to_point_timing_cost[inet][ipin] =
								temp_point_to_point_timing_cost[inet][ipin];
						temp_point_to_point_timing_cost[inet][ipin] = -1;
					}
				}
			}
		}
	}
	if (is_folding) {
		for (i = 0; i < num_stage; i++) {
			stage_delay_cost[i] = folding_delay[i];
			folding_delay[i] = 0;
			stage_timing_cost[i] = temp_timing[i];
			temp_timing[i] = 0;
		}
	}
}

static void comp_delta_td_cost(int *b_from, int *b_to, int num_of_pins,
		float *delta_timing, float *delta_delay, int length) {

	/*a net that is being driven by a moved block must have all of its  */
	/*sink timing costs recomputed. A net that is driving a moved block */
	/*must only have the timing cost on the connection driving the input*/
	/*pin computed*/

	int i, inet, k, net_pin, ipin;
	float delta_timing_cost, delta_delay_cost, temp_delay;
	int pair, cb_from, cb_to, cstage;

	float total_delay, total_timing;
	float old_max_delay, old_max_timing, new_max_delay, new_max_timing;

	total_delay = 0;
	total_timing = 0;

	if (is_folding) {
		for (i = 0; i < num_stage; i++) {
			folding_delay[i] = 0;
			temp_timing[i] = 0;
		}
	}
//printf("length %d\n", length);
	for (pair = 0; pair < length; pair++) {
		cb_from = b_from[pair];
		cb_to = b_to[pair];
		delta_timing_cost = 0.;
		delta_delay_cost = 0.;

		if (cb_from != EMPTY) {
			//printf("the from block num %d for %s\n", cb_from, block[cb_from].name);
			cstage = block[cb_from].stage;
			for (k = 0; k < num_of_pins; k++) {
				inet = block[cb_from].nets[k];

				if (inet == OPEN)
					continue;

				if (is_global[inet])
					continue;
				//printf("the net %d %s\n", inet, net[inet].name);
				net_pin = net_pin_index[cb_from][k];
				//printf("net pin is %d\n", net_pin);
				if (net_pin != 0) { /*this net is driven by a moved block               */

					/*if this net is being driven by a block that has moved, we do not  */
					/*need to compute the change in the timing cost (here) since it will*/
					/*be computed in the fanout of the net on  the driving block, also  */
					/*computing it here would double count the change, and mess up the  */
					/*delta_timing_cost value */
					if (net[inet].blocks[0] != cb_to
							&& net[inet].blocks[0] != cb_from) {
						temp_delay = comp_td_point_to_point_delay(inet,
								net_pin);
						//printf("the temp delay is %11.6g \n", temp_delay);
						temp_point_to_point_delay_cost[inet][net_pin] =
								temp_delay;
						temp_point_to_point_timing_cost[inet][net_pin] =
								timing_place_crit[inet][net_pin] * temp_delay;
						//printf("the crit is %11.6g \n", timing_place_crit[inet][net_pin]);
						//printf("the timing cost is %11.6g \n", temp_point_to_point_timing_cost[inet][net_pin]);
						//printf("the original delay %11.6g \n", point_to_point_delay_cost[inet][net_pin]);
						//printf("old delta delay cost %11.6g", delta_delay_cost);
						delta_delay_cost +=
								temp_point_to_point_delay_cost[inet][net_pin]
										- point_to_point_delay_cost[inet][net_pin];
						//printf("the delta delay %11.6g\n", delta_delay_cost);
						delta_timing_cost +=
								temp_point_to_point_timing_cost[inet][net_pin]
										- point_to_point_timing_cost[inet][net_pin];
						//printf("the original timing %11.6g\n", point_to_point_timing_cost[inet][net_pin]);
						//printf("the delta timing %11.6g\n", delta_timing_cost);
					}
				} else { /*this net is being driven by a moved block, recompute */
					/*all point to point connections on this net.*/
					for (ipin = 1; ipin < net[inet].num_pins; ipin++) {
						temp_delay = comp_td_point_to_point_delay(inet, ipin);
						temp_point_to_point_delay_cost[inet][ipin] = temp_delay;
						//printf("the temp delay is %11.6g \n", temp_point_to_point_delay_cost[inet][ipin]);
						temp_point_to_point_timing_cost[inet][ipin] =
								timing_place_crit[inet][ipin] * temp_delay;
						//printf("here the crit is %11.6g \n", timing_place_crit[inet][ipin]);
						//printf("the timing cost is %11.8g \n", temp_point_to_point_timing_cost[inet][ipin]);
						//printf("old delta delay cost %11.6g", delta_delay_cost);
						delta_delay_cost +=
								temp_point_to_point_delay_cost[inet][ipin]
										- point_to_point_delay_cost[inet][ipin];
						//printf("the original delay %11.6g\n", point_to_point_delay_cost[inet][ipin]);
						//printf("the delta delay %11.6g\n", delta_delay_cost);
						//printf("old delta timing cost %11.6g", delta_timing_cost);
						delta_timing_cost +=
								temp_point_to_point_timing_cost[inet][ipin]
										- point_to_point_timing_cost[inet][ipin];
						//printf("the original timing %11.6g\n", point_to_point_timing_cost[inet][ipin]);
						//printf("the delta timing %11.6g\n", delta_timing_cost);
					}
				}
			}
		}

		// printf("the delta delay %11.6g\n", delta_delay_cost);
		if (cb_to != EMPTY) {
			//printf("the to block num %d for %s\n", cb_to, block[cb_to].name);
			cstage = block[cb_to].stage;
			for (k = 0; k < num_of_pins; k++) {
				inet = block[cb_to].nets[k];
				// printf("the net %d %s\n", inet, net[inet].name);
				if (inet == OPEN)
					continue;

				if (is_global[inet])
					continue;

				net_pin = net_pin_index[cb_to][k];
				//printf("net pin is %d\n", net_pin);
				if (net_pin != 0) { /*this net is driving a moved block*/

					/*if this net is being driven by a block that has moved, we do not */
					/*need to compute the change in the timing cost (here) since it was*/
					/*computed in the fanout of the net on  the driving block, also    */
					/*computing it here would double count the change, and mess up the */
					/*delta_timing_cost value */
					if (net[inet].blocks[0] != cb_to
							&& net[inet].blocks[0] != cb_from) {
						temp_delay = comp_td_point_to_point_delay(inet,
								net_pin);
						// printf("the temp delay is %11.6g \n", temp_delay);
						temp_point_to_point_delay_cost[inet][net_pin] =
								temp_delay;
						temp_point_to_point_timing_cost[inet][net_pin] =
								timing_place_crit[inet][net_pin] * temp_delay;
						//printf("the crit is %11.6g \n", timing_place_crit[inet][net_pin]);
						//printf("the timing cost is %11.6g \n", temp_point_to_point_timing_cost[inet][net_pin]);
						//printf("old delta delay cost %11.6g", delta_delay_cost);
						delta_delay_cost +=
								temp_point_to_point_delay_cost[inet][net_pin]
										- point_to_point_delay_cost[inet][net_pin];
						//printf("the original delay %11.6g \n", point_to_point_delay_cost[inet][net_pin]);
						//printf("the delta delay %11.6g\n", delta_delay_cost);
						// printf("old delta timing cost %11.6g", delta_timing_cost);
						delta_timing_cost +=
								temp_point_to_point_timing_cost[inet][net_pin]
										- point_to_point_timing_cost[inet][net_pin];
						//printf("the original timing %11.6g\n", point_to_point_timing_cost[inet][net_pin]);
						//printf("the delta timing %11.6g\n", delta_timing_cost);
					}
				} else { /*this net is being driven by a moved block, recompute */
					/*all point to point connections on this net.*/
					for (ipin = 1; ipin < net[inet].num_pins; ipin++) {

						temp_delay = comp_td_point_to_point_delay(inet, ipin);
						//printf("the temp delay is %11.6g \n", temp_delay);
						temp_point_to_point_delay_cost[inet][ipin] = temp_delay;
						temp_point_to_point_timing_cost[inet][ipin] =
								timing_place_crit[inet][ipin] * temp_delay;
						//printf("the crit is %11.6g \n", timing_place_crit[inet][ipin]);
						//printf("the timing cost is %11.6g \n", temp_point_to_point_timing_cost[inet][ipin]);
						//printf("old delta delay cost %11.6g", delta_delay_cost);
						delta_delay_cost +=
								temp_point_to_point_delay_cost[inet][ipin]
										- point_to_point_delay_cost[inet][ipin];
						//printf("the original delay %11.6g\n", point_to_point_delay_cost[inet][ipin]);
						//printf("the delta delay %11.6g\n", delta_delay_cost);
						//printf("old delta timing cost %11.6g", delta_timing_cost);
						delta_timing_cost +=
								temp_point_to_point_timing_cost[inet][ipin]
										- point_to_point_timing_cost[inet][ipin];
						//printf("the original timing %11.6g\n", point_to_point_timing_cost[inet][ipin]);
						//printf("the delta timing %11.6g\n", delta_timing_cost);
					}
				}
			}
		}
		//printf("the delta delay %11.6g\n", delta_delay_cost);
		if (is_folding) {
			//printf("stage is %d\n", cstage);
			folding_delay[cstage - 1] += delta_delay_cost;
			temp_timing[cstage - 1] += delta_timing_cost;
		} else {
			total_delay += delta_delay_cost;
			total_timing += delta_timing_cost;
		}
	}

	if (is_folding) {
		for (i = 0; i < num_stage; i++) {
			folding_delay[i] += stage_delay_cost[i];
			temp_timing[i] += stage_timing_cost[i];
			//printf("the orig delay %11.6g, the new delay %11.6g\n", stage_delay_cost[i], folding_delay[i]);
			//printf("the orig timing %11.6g, the new timing %11.6g\n", stage_timing_cost[i], temp_timing[i]);
		}
		old_max_delay = stage_delay_cost[0];
		old_max_timing = stage_timing_cost[0];
		new_max_delay = folding_delay[0];
		new_max_timing = temp_timing[0];
		for (i = 1; i < num_stage; i++) {
			if (old_max_delay < stage_delay_cost[i])
				old_max_delay = stage_delay_cost[i];
			if (old_max_timing < stage_timing_cost[i])
				old_max_timing = stage_timing_cost[i];
			if (new_max_delay < folding_delay[i])
				new_max_delay = folding_delay[i];
			if (new_max_timing < temp_timing[i])
				new_max_timing = temp_timing[i];
		}
		// printf("new delay %11.6g, timing %11.6g, old delay %11.6g, timing %11.6g\n", new_max_delay, new_max_timing, old_max_delay, old_max_timing);
		*delta_timing = num_stage * (new_max_timing - old_max_timing);
		*delta_delay = num_stage * (new_max_delay - old_max_delay);
	} else {
		*delta_timing = total_timing;
		*delta_delay = total_delay;
	}
}

static void comp_td_costs(float *timing_cost, float *connection_delay_sum) {
	/*computes the cost (from scratch) due to the delays and criticalities*
	 *on all point to point connections, we define the timing cost of     *
	 *each connection as criticality*delay */

	int inet, ipin;
	float loc_timing_cost, loc_connection_delay_sum, temp_delay_cost,
			temp_timing_cost;
	int i;
	float max_delay, max_timing;

	loc_timing_cost = 0.;
	loc_connection_delay_sum = 0.;

	if (is_folding) {
		for (i = 0; i < num_stage; i++) {
			stage_delay_cost[i] = 0;
			stage_timing_cost[i] = 0;
		}
	}

	for (inet = 0; inet < num_nets; inet++) { /* for each net ... */
		if (is_global[inet] == FALSE) { /* Do only if not global. */
			//printf("net %d %s", inet, net[inet].name);
			for (ipin = 1; ipin < net[inet].num_pins; ipin++) {

				temp_delay_cost = comp_td_point_to_point_delay(inet, ipin);
				//printf("delay %11.6g \n", temp_delay_cost);
				temp_timing_cost = temp_delay_cost
						* timing_place_crit[inet][ipin];
				//printf("timing %11.6g \n", temp_timing_cost);
				loc_connection_delay_sum += temp_delay_cost;
				point_to_point_delay_cost[inet][ipin] = temp_delay_cost;
				temp_point_to_point_delay_cost[inet][ipin] = -1; /*undefined*/

				point_to_point_timing_cost[inet][ipin] = temp_timing_cost;
				temp_point_to_point_timing_cost[inet][ipin] = -1; /*undefined*/
				loc_timing_cost += temp_timing_cost;

				if (is_folding) {
					for (i = 0; i < num_stage; i++) {
						if ((inet >= num_net_per_stage[i].begin)
								&& (inet
										< num_net_per_stage[i].num
												+ num_net_per_stage[i].begin)) {
							// printf("come here for istage %d \n", i);
							stage_delay_cost[i] += temp_delay_cost;
							stage_timing_cost[i] += temp_timing_cost;
							break;
						}
					}
				}
			}
		}
	}

	/* for (i=0; i<num_stage; i++)
	 {printf("stage %d\n", i);
	 printf("delay %11.6g \n", stage_delay_cost[i]);
	 printf("timing %11.6g \n", stage_timing_cost[i]);}*/
	if (is_folding) {
		max_delay = stage_delay_cost[0];
		max_timing = stage_timing_cost[0];

		for (i = 1; i < num_stage; i++) {
			if (max_delay < stage_delay_cost[i])
				max_delay = stage_delay_cost[i];
			if (max_timing < stage_timing_cost[i])
				max_timing = stage_timing_cost[i];
		}
		*timing_cost = max_timing * num_stage;
		*connection_delay_sum = max_delay * num_stage;
	} else {
		*timing_cost = loc_timing_cost;
		*connection_delay_sum = loc_connection_delay_sum;
	}
}

static float comp_bb_cost(int method, int place_cost_type, int num_regions) {

	/* Finds the cost from scratch.  Done only when the placement   *
	 * has been radically changed (i.e. after initial placement).   *
	 * Otherwise find the cost change incrementally.  If method     *
	 * check is NORMAL, we find bounding boxes that are updateable  *
	 * for the larger nets.  If method is CHECK, all bounding boxes *
	 * are found via the non_updateable_bb routine, to provide a    *
	 * cost which can be used to check the correctness of the       *
	 * other routine.                                               */

	int i, j, k;
	float cost;

	cost = 0;

	/* Initialize occupancies to zero if regions are being used. */

	if (place_cost_type == NONLINEAR_CONG) {
		for (i = 0; i < num_regions; i++) {
			for (j = 0; j < num_regions; j++) {
				place_region_x[i][j].occupancy = 0.;
				place_region_y[i][j].occupancy = 0.;
			}
		}
	}

	for (k = 0; k < num_nets; k++) { /* for each net ... */

		if (is_global[k] == FALSE) { /* Do only if not global. */

			/* Small nets don't use incremental updating on their bounding boxes, *
			 * so they can use a fast bounding box calculator.                    */

			if (net[k].num_pins > SMALL_NET && method == NORMAL) {
				get_bb_from_scratch(k, &bb_coords[k], &bb_num_on_edges[k]);
			} else {
				get_non_updateable_bb(k, &bb_coords[k]);
			}

			if (place_cost_type != NONLINEAR_CONG) {
				net_cost[k] = get_net_cost(k, &bb_coords[k]);
				cost += net_cost[k];
			} else { /* Must be nonlinear_cong case. */
				update_region_occ(k, &bb_coords[k], 1, num_regions);
			}
		}
	}

	if (place_cost_type == NONLINEAR_CONG) {
		cost = nonlinear_cong_cost(num_regions);
	}

	return (cost);
}

static float nonlinear_cong_cost(int num_regions) {

	/* This routine computes the cost of a placement when the NONLINEAR_CONG *
	 * option is selected.  It assumes that the occupancies of all the       *
	 * placement subregions have been properly updated, and simply           *
	 * computes the cost due to these occupancies by summing over all        *
	 * subregions.  This will be inefficient for moves that don't affect     *
	 * many subregions (i.e. small moves late in placement), esp. when there *
	 * are a lot of subregions.  May recode later to update only affected    *
	 * subregions.                                                           */

	float cost, tmp;
	int i, j;

	cost = 0.;

	for (i = 0; i < num_regions; i++) {
		for (j = 0; j < num_regions; j++) {

			/* Many different cost metrics possible.  1st try:  */

			if (place_region_x[i][j].occupancy
					< place_region_x[i][j].capacity) {
				cost += place_region_x[i][j].occupancy
						* place_region_x[i][j].inv_capacity;
			} else { /* Overused region -- penalize. */

				tmp = place_region_x[i][j].occupancy
						* place_region_x[i][j].inv_capacity;
				cost += tmp * tmp;
			}

			if (place_region_y[i][j].occupancy
					< place_region_y[i][j].capacity) {
				cost += place_region_y[i][j].occupancy
						* place_region_y[i][j].inv_capacity;
			} else { /* Overused region -- penalize. */

				tmp = place_region_y[i][j].occupancy
						* place_region_y[i][j].inv_capacity;
				cost += tmp * tmp;
			}

		}
	}

	return (cost);
}

static void update_region_occ(int inet, struct s_bb *coords, int add_or_sub,
		int num_regions) {

	/* Called only when the place_cost_type is NONLINEAR_CONG.  If add_or_sub *
	 * is 1, this uses the new net bounding box to increase the occupancy     *
	 * of some regions.  If add_or_sub = - 1, it decreases the occupancy      *
	 * by that due to this bounding box.                                      */

	float net_xmin, net_xmax, net_ymin, net_ymax, crossing;
	float inv_region_len, inv_region_height;
	float inv_bb_len, inv_bb_height;
	float overlap_xlow, overlap_xhigh, overlap_ylow, overlap_yhigh;
	float y_overlap, x_overlap, x_occupancy, y_occupancy;
	int imin, imax, jmin, jmax, i, j;

	if (net[inet].num_pins > 50) {
		crossing = 2.7933 + 0.02616 * (net[inet].num_pins - 50);
	} else {
		crossing = cross_count[net[inet].num_pins - 1];
	}

	net_xmin = coords->xmin - 0.5;
	net_xmax = coords->xmax + 0.5;
	net_ymin = coords->ymin - 0.5;
	net_ymax = coords->ymax + 0.5;

	/* I could precompute the two values below.  Should consider this. */

	inv_region_len = (float) num_regions / (float) nx;
	inv_region_height = (float) num_regions / (float) ny;

	/* Get integer coordinates defining the rectangular area in which the *
	 * subregions have to be updated.  Formula is as follows:  subtract   *
	 * 0.5 from net_xmin, etc. to get numbers from 0 to nx or ny;         *
	 * divide by nx or ny to scale between 0 and 1; multiply by           *
	 * num_regions to scale between 0 and num_regions; and truncate to    *
	 * get the final answer.                                              */

	imin = (int) (net_xmin - 0.5) * inv_region_len;
	imax = (int) (net_xmax - 0.5) * inv_region_len;
	imax = min (imax, num_regions - 1); /* Watch for weird roundoff */

	jmin = (int) (net_ymin - 0.5) * inv_region_height;
	jmax = (int) (net_ymax - 0.5) * inv_region_height;
	jmax = min (jmax, num_regions - 1); /* Watch for weird roundoff */

	inv_bb_len = 1. / (net_xmax - net_xmin);
	inv_bb_height = 1. / (net_ymax - net_ymin);

	/* See RISA paper (ICCAD '94, pp. 690 - 695) for a description of why *
	 * I use exactly this cost function.                                  */

	for (i = imin; i <= imax; i++) {
		for (j = jmin; j <= jmax; j++) {
			overlap_xlow = max (place_region_bounds_x[i],net_xmin);
			overlap_xhigh = min (place_region_bounds_x[i+1],net_xmax);
			overlap_ylow = max (place_region_bounds_y[j],net_ymin);
			overlap_yhigh = min (place_region_bounds_y[j+1],net_ymax);

			x_overlap = overlap_xhigh - overlap_xlow;
			y_overlap = overlap_yhigh - overlap_ylow;

#ifdef DEBUG

			if (x_overlap < -0.001) {
				printf("Error in update_region_occ:  x_overlap < 0"
						"\n inet = %d, overlap = %g\n", inet, x_overlap);
			}

			if (y_overlap < -0.001) {
				printf("Error in update_region_occ:  y_overlap < 0"
						"\n inet = %d, overlap = %g\n", inet, y_overlap);
			}
#endif

			x_occupancy = crossing * y_overlap * x_overlap * inv_bb_height
					* inv_region_len;
			y_occupancy = crossing * x_overlap * y_overlap * inv_bb_len
					* inv_region_height;

			place_region_x[i][j].occupancy += add_or_sub * x_occupancy;
			place_region_y[i][j].occupancy += add_or_sub * y_occupancy;
		}
	}

}

static void free_place_regions(int num_regions) {

	/* Frees the place_regions data structures needed by the NONLINEAR_CONG *
	 * cost function.                                                       */

	free_matrix(place_region_x, 0, num_regions - 1, 0,
			sizeof(struct s_place_region));

	free_matrix(place_region_y, 0, num_regions - 1, 0,
			sizeof(struct s_place_region));

	free(place_region_bounds_x);
	free(place_region_bounds_y);
}

static void free_placement_structs(int place_cost_type, int num_regions,
		float **old_region_occ_x, float **old_region_occ_y,
		struct s_placer_opts placer_opts) {

	/* Frees the major structures needed by the placer (and not needed       *
	 * elsewhere).   */

	int inet;

	if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
			|| placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
			|| placer_opts.enable_timing_computations) {
		for (inet = 0; inet < num_nets; inet++) {
			/*add one to the address since it is indexed from 1 not 0 */

			point_to_point_delay_cost[inet]++;
			free(point_to_point_delay_cost[inet]);

			point_to_point_timing_cost[inet]++;
			free(point_to_point_timing_cost[inet]);

			temp_point_to_point_delay_cost[inet]++;
			free(temp_point_to_point_delay_cost[inet]);

			temp_point_to_point_timing_cost[inet]++;
			free(temp_point_to_point_timing_cost[inet]);
		}
		free(point_to_point_delay_cost);
		free(temp_point_to_point_delay_cost);

		free(point_to_point_timing_cost);
		free(temp_point_to_point_timing_cost);

		free_matrix(net_pin_index, 0, num_blocks - 1, 0, sizeof(int));
	}

	free(net_cost);
	free(temp_net_cost);
	free(bb_num_on_edges);
	free(bb_coords);

	/* Added by Wei */
	free(stage_delay_cost);
	free(stage_timing_cost);
	free(folding_delay);
	free(temp_timing);

	net_cost = NULL; /* Defensive coding. */
	temp_net_cost = NULL;
	bb_num_on_edges = NULL;
	bb_coords = NULL;

	free_unique_pin_list();

	if (place_cost_type == NONLINEAR_CONG) {
		free_place_regions(num_regions);
		free_matrix(old_region_occ_x, 0, num_regions - 1, 0, sizeof(float));
		free_matrix(old_region_occ_y, 0, num_regions - 1, 0, sizeof(float));
	}

	else if (place_cost_type == LINEAR_CONG) {
		free_fast_cost_update_structs();
	}
}

static void alloc_and_load_placement_structs(int place_cost_type,
		int num_regions, float place_cost_exp, float ***old_region_occ_x,
		float ***old_region_occ_y, struct s_placer_opts placer_opts) {

	/* Allocates the major structures needed only by the placer, primarily for *
	 * computing costs quickly and such.                                       */

	int inet, ipin, iblk;

	if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
			|| placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
			|| placer_opts.enable_timing_computations) {
		/*allocate structures associated with timing driven placement */
		/* [0..num_nets-1][1..num_pins-1]  */
		point_to_point_delay_cost = (float **) my_malloc(
				num_nets * sizeof(float*));
		temp_point_to_point_delay_cost = (float **) my_malloc(
				num_nets * sizeof(float*));

		point_to_point_timing_cost = (float **) my_malloc(
				num_nets * sizeof(float*));
		temp_point_to_point_timing_cost = (float **) my_malloc(
				num_nets * sizeof(float*));

		net_pin_index = (int **) my_malloc(num_blocks * sizeof(int *));
		for (iblk = 0; iblk < num_blocks; iblk++) {
			if (block[iblk].type == DSP)
				net_pin_index[iblk] = (int *) my_malloc(
						pins_per_dsp * sizeof(int));
			else
				net_pin_index[iblk] = (int *) my_malloc(
						pins_per_clb * sizeof(int));
		}

		for (inet = 0; inet < num_nets; inet++) {

			/* in the following, subract one so index starts at *
			 * 1 instead of 0 */
			point_to_point_delay_cost[inet] = (float *) my_malloc(
					(net[inet].num_pins - 1) * sizeof(float));
			point_to_point_delay_cost[inet]--;

			temp_point_to_point_delay_cost[inet] = (float *) my_malloc(
					(net[inet].num_pins - 1) * sizeof(float));
			temp_point_to_point_delay_cost[inet]--;

			point_to_point_timing_cost[inet] = (float *) my_malloc(
					(net[inet].num_pins - 1) * sizeof(float));
			point_to_point_timing_cost[inet]--;

			temp_point_to_point_timing_cost[inet] = (float *) my_malloc(
					(net[inet].num_pins - 1) * sizeof(float));
			temp_point_to_point_timing_cost[inet]--;
		}
		for (inet = 0; inet < num_nets; inet++) {
			for (ipin = 1; ipin < net[inet].num_pins; ipin++) {
				point_to_point_delay_cost[inet][ipin] = 0;
				temp_point_to_point_delay_cost[inet][ipin] = 0;
			}
		}
	}
	/* Added by Wei */
	stage_delay_cost = (float*) my_malloc(num_stage * sizeof(float));
	stage_timing_cost = (float*) my_malloc(num_stage * sizeof(float));
	folding_delay = (float*) my_malloc(num_stage * sizeof(float));
	temp_timing = (float*) my_malloc(num_stage * sizeof(float));

	net_cost = (float *) my_malloc(num_nets * sizeof(float));
	temp_net_cost = (float *) my_malloc(num_nets * sizeof(float));

	/* Used to store costs for moves not yet made and to indicate when a net's   *
	 * cost has been recomputed. temp_net_cost[inet] < 0 means net's cost hasn't *
	 * been recomputed.                                                          */

	for (inet = 0; inet < num_nets; inet++)
		temp_net_cost[inet] = -1.;

	bb_coords = (struct s_bb *) my_malloc(num_nets * sizeof(struct s_bb));
	bb_num_on_edges = (struct s_bb *) my_malloc(num_nets * sizeof(struct s_bb));

	/* Get a list of pins with no duplicates. */

	alloc_and_load_unique_pin_list();

	/* Allocate storage for subregion data, if needed. */

	if (place_cost_type == NONLINEAR_CONG) {
		alloc_place_regions(num_regions);
		load_place_regions(num_regions);
		*old_region_occ_x = (float **) alloc_matrix(0, num_regions - 1, 0,
				num_regions - 1, sizeof(float));
		*old_region_occ_y = (float **) alloc_matrix(0, num_regions - 1, 0,
				num_regions - 1, sizeof(float));
	} else { /* Shouldn't use them; crash hard if I do!   */
		*old_region_occ_x = NULL;
		*old_region_occ_y = NULL;
	}

	if (place_cost_type == LINEAR_CONG) {
		alloc_and_load_for_fast_cost_update(place_cost_exp);
	}
}

static void alloc_place_regions(int num_regions) {

	/* Allocates memory for the regional occupancy, cost, etc. counts *
	 * kept when we're using the NONLINEAR_CONG placement cost        *
	 * function.                                                      */

	place_region_x = (struct s_place_region **) alloc_matrix(0, num_regions - 1,
			0, num_regions - 1, sizeof(struct s_place_region));

	place_region_y = (struct s_place_region **) alloc_matrix(0, num_regions - 1,
			0, num_regions - 1, sizeof(struct s_place_region));

	place_region_bounds_x = (float *) my_malloc(
			(num_regions + 1) * sizeof(float));

	place_region_bounds_y = (float *) my_malloc(
			(num_regions + 1) * sizeof(float));
}

static void load_place_regions(int num_regions) {

	/* Loads the capacity values in each direction for each of the placement *
	 * regions.  The chip is divided into a num_regions x num_regions array. */

	int i, j, low_block, high_block, rnum;
	float low_lim, high_lim, capacity, fac, block_capacity;
	float len_fac, height_fac;

	/* First load up horizontal channel capacities.  */

	for (j = 0; j < num_regions; j++) {
		capacity = 0.;
		low_lim = (float) j / (float) num_regions * ny + 1.;
		high_lim = (float) (j + 1) / (float) num_regions * ny;
		low_block = floor(low_lim);
		low_block = max (1,low_block); /* Watch for weird roundoff effects. */
		high_block = ceil(high_lim);
		high_block = min(high_block, ny);

		block_capacity = (chan_width_x[low_block - 1] + chan_width_x[low_block])
				/ 2.;
		if (low_block == 1)
			block_capacity += chan_width_x[0] / 2.;

		fac = 1. - (low_lim - low_block);
		capacity += fac * block_capacity;

		for (rnum = low_block + 1; rnum < high_block; rnum++) {
			block_capacity = (chan_width_x[rnum - 1] + chan_width_x[rnum]) / 2.;
			capacity += block_capacity;
		}

		block_capacity = (chan_width_x[high_block - 1]
				+ chan_width_x[high_block]) / 2.;
		if (high_block == ny)
			block_capacity += chan_width_x[ny] / 2.;

		fac = 1. - (high_block - high_lim);
		capacity += fac * block_capacity;

		for (i = 0; i < num_regions; i++) {
			place_region_x[i][j].capacity = capacity;
			place_region_x[i][j].inv_capacity = 1. / capacity;
			place_region_x[i][j].occupancy = 0.;
			place_region_x[i][j].cost = 0.;
		}
	}

	/* Now load vertical channel capacities.  */

	for (i = 0; i < num_regions; i++) {
		capacity = 0.;
		low_lim = (float) i / (float) num_regions * nx + 1.;
		high_lim = (float) (i + 1) / (float) num_regions * nx;
		low_block = floor(low_lim);
		low_block = max (1,low_block); /* Watch for weird roundoff effects. */
		high_block = ceil(high_lim);
		high_block = min(high_block, nx);

		block_capacity = (chan_width_y[low_block - 1] + chan_width_y[low_block])
				/ 2.;
		if (low_block == 1)
			block_capacity += chan_width_y[0] / 2.;

		fac = 1. - (low_lim - low_block);
		capacity += fac * block_capacity;

		for (rnum = low_block + 1; rnum < high_block; rnum++) {
			block_capacity = (chan_width_y[rnum - 1] + chan_width_y[rnum]) / 2.;
			capacity += block_capacity;
		}

		block_capacity = (chan_width_y[high_block - 1]
				+ chan_width_y[high_block]) / 2.;
		if (high_block == nx)
			block_capacity += chan_width_y[nx] / 2.;

		fac = 1. - (high_block - high_lim);
		capacity += fac * block_capacity;

		for (j = 0; j < num_regions; j++) {
			place_region_y[i][j].capacity = capacity;
			place_region_y[i][j].inv_capacity = 1. / capacity;
			place_region_y[i][j].occupancy = 0.;
			place_region_y[i][j].cost = 0.;
		}
	}

	/* Finally set up the arrays indicating the limits of each of the *
	 * placement subregions.                                          */

	len_fac = (float) nx / (float) num_regions;
	height_fac = (float) ny / (float) num_regions;

	place_region_bounds_x[0] = 0.5;
	place_region_bounds_y[0] = 0.5;

	for (i = 1; i <= num_regions; i++) {
		place_region_bounds_x[i] = place_region_bounds_x[i - 1] + len_fac;
		place_region_bounds_y[i] = place_region_bounds_y[i - 1] + height_fac;
	}
}

static void free_unique_pin_list(void) {

	/* Frees the unique pin list structures.                               */

	int any_dup, inet;

	any_dup = 0;

	for (inet = 0; inet < num_nets; inet++) {
		if (duplicate_pins[inet] != 0) {
			free(unique_pin_list[inet]);
			any_dup = 1;
		}
	}

	if (any_dup != 0)
		free(unique_pin_list);

	free(duplicate_pins);
}

static void alloc_and_load_unique_pin_list(void) {

	/* This routine looks for multiple pins going to the same block in the *
	 * pinlist of each net.  If it finds any, it marks that net as having  *
	 * duplicate pins, and creates a new pinlist with no duplicates.  This *
	 * is then used by the updatable bounding box calculation routine for  *
	 * efficiency.                                                         */

	int inet, ipin, bnum, num_dup, any_dups, offset;
	int *times_listed; /* [0..num_blocks-1]: number of times a block is   *
	 * listed in the pinlist of a net.  Temp. storage. */

	duplicate_pins = my_calloc(num_nets, sizeof(int));
	times_listed = my_calloc(num_blocks, sizeof(int));
	any_dups = 0;

	for (inet = 0; inet < num_nets; inet++) {

		num_dup = 0;

		for (ipin = 0; ipin < net[inet].num_pins; ipin++) {
			bnum = net[inet].blocks[ipin];
			times_listed[bnum]++;
			if (times_listed[bnum] > 1)
				num_dup++;
		}

		if (num_dup > 0) { /* Duplicates found.  Make unique pin list. */
			duplicate_pins[inet] = num_dup;

			if (any_dups == 0) { /* This is the first duplicate found */
				unique_pin_list = (int **) my_calloc(num_nets, sizeof(int *));
				any_dups = 1;
			}

			unique_pin_list[inet] = my_malloc(
					(net[inet].num_pins - num_dup) * sizeof(int));

			offset = 0;
			for (ipin = 0; ipin < net[inet].num_pins; ipin++) {
				bnum = net[inet].blocks[ipin];
				if (times_listed[bnum] != 0) {
					times_listed[bnum] = 0;
					unique_pin_list[inet][offset] = bnum;
					offset++;
				}
			}
		}

		else { /* No duplicates found.  Reset times_listed. */
			for (ipin = 0; ipin < net[inet].num_pins; ipin++) {
				bnum = net[inet].blocks[ipin];
				times_listed[bnum] = 0;
			}
		}
	}

	free((void *) times_listed);
}

static void get_bb_from_scratch(int inet, struct s_bb *coords,
		struct s_bb *num_on_edges) {

	/* This routine finds the bounding box of each net from scratch (i.e.    *
	 * from only the block location information).  It updates both the       *
	 * coordinate and number of blocks on each edge information.  It         *
	 * should only be called when the bounding box information is not valid. */

	int ipin, bnum, x, y, xmin, xmax, ymin, ymax;
	int xmin_edge, xmax_edge, ymin_edge, ymax_edge;
	int n_pins;
	int *plist;

	/* I need a list of blocks to which this net connects, with no block listed *
	 * more than once, in order to get a proper count of the number on the edge *
	 * of the bounding box.                                                     */

	if (duplicate_pins[inet] == 0) {
		plist = net[inet].blocks;
		n_pins = net[inet].num_pins;
	} else {
		plist = unique_pin_list[inet];
		n_pins = net[inet].num_pins - duplicate_pins[inet];
	}

	x = block[plist[0]].x;
	y = block[plist[0]].y;

	x = max(min(x,nx),1);
	y = max(min(y,ny),1);

	xmin = x;
	ymin = y;
	xmax = x;
	ymax = y;
	xmin_edge = 1;
	ymin_edge = 1;
	xmax_edge = 1;
	ymax_edge = 1;

	for (ipin = 1; ipin < n_pins; ipin++) {

		bnum = plist[ipin];
		x = block[bnum].x;
		y = block[bnum].y;

		/* Code below counts IO blocks as being within the 1..nx, 1..ny clb array. *
		 * This is because channels do not go out of the 0..nx, 0..ny range, and   *
		 * I always take all channels impinging on the bounding box to be within   *
		 * that bounding box.  Hence, this "movement" of IO blocks does not affect *
		 * the which channels are included within the bounding box, and it         *
		 * simplifies the code a lot.                                              */

		x = max(min(x,nx),1);
		y = max(min(y,ny),1);

		if (x == xmin) {
			xmin_edge++;
		}
		if (x == xmax) { /* Recall that xmin could equal xmax -- don't use else */
			xmax_edge++;
		} else if (x < xmin) {
			xmin = x;
			xmin_edge = 1;
		} else if (x > xmax) {
			xmax = x;
			xmax_edge = 1;
		}

		if (y == ymin) {
			ymin_edge++;
		}
		if (y == ymax) {
			ymax_edge++;
		} else if (y < ymin) {
			ymin = y;
			ymin_edge = 1;
		} else if (y > ymax) {
			ymax = y;
			ymax_edge = 1;
		}
	}

	/* Copy the coordinates and number on edges information into the proper   *
	 * structures.                                                            */

	coords->xmin = xmin;
	coords->xmax = xmax;
	coords->ymin = ymin;
	coords->ymax = ymax;

	num_on_edges->xmin = xmin_edge;
	num_on_edges->xmax = xmax_edge;
	num_on_edges->ymin = ymin_edge;
	num_on_edges->ymax = ymax_edge;
}

static float get_net_cost(int inet, struct s_bb *bbptr) {

	/* Finds the cost due to one net by looking at its coordinate bounding  *
	 * box.                                                                 */

	float ncost, crossing;

	/* Get the expected "crossing count" of a net, based on its number *
	 * of pins.  Extrapolate for very large nets.                      */

	if (net[inet].num_pins > 50) {
		crossing = 2.7933 + 0.02616 * (net[inet].num_pins - 50);
		/*    crossing = 3.0;    Old value  */
	} else {
		crossing = cross_count[net[inet].num_pins - 1];
	}

	/* Could insert a check for xmin == xmax.  In that case, assume  *
	 * connection will be made with no bends and hence no x-cost.    *
	 * Same thing for y-cost.                                        */

	/* Cost = wire length along channel * cross_count / average      *
	 * channel capacity.   Do this for x, then y direction and add.  */

	ncost = (bbptr->xmax - bbptr->xmin + 1) * crossing
			* chanx_place_cost_fac[bbptr->ymax][bbptr->ymin - 1];

	ncost += (bbptr->ymax - bbptr->ymin + 1) * crossing
			* chany_place_cost_fac[bbptr->xmax][bbptr->xmin - 1];

	return (ncost);
}

static void get_non_updateable_bb(int inet, struct s_bb *bb_coord_new) {

	/* Finds the bounding box of a net and stores its coordinates in the  *
	 * bb_coord_new data structure.  This routine should only be called   *
	 * for small nets, since it does not determine enough information for *
	 * the bounding box to be updated incrementally later.                *
	 * Currently assumes channels on both sides of the CLBs forming the   *
	 * edges of the bounding box can be used.  Essentially, I am assuming *
	 * the pins always lie on the outside of the bounding box.            */

	int k, xmax, ymax, xmin, ymin, x, y;

	x = block[net[inet].blocks[0]].x;
	y = block[net[inet].blocks[0]].y;

	xmin = x;
	ymin = y;
	xmax = x;
	ymax = y;

	for (k = 1; k < net[inet].num_pins; k++) {
		x = block[net[inet].blocks[k]].x;
		y = block[net[inet].blocks[k]].y;

		if (x < xmin) {
			xmin = x;
		} else if (x > xmax) {
			xmax = x;
		}

		if (y < ymin) {
			ymin = y;
		} else if (y > ymax) {
			ymax = y;
		}
	}

	/* Now I've found the coordinates of the bounding box.  There are no *
	 * channels beyond nx and ny, so I want to clip to that.  As well,   *
	 * since I'll always include the channel immediately below and the   *
	 * channel immediately to the left of the bounding box, I want to    *
	 * clip to 1 in both directions as well (since minimum channel index *
	 * is 0).  See route.c for a channel diagram.                        */

	bb_coord_new->xmin = max(min(xmin,nx),1);
	bb_coord_new->ymin = max(min(ymin,ny),1);
	bb_coord_new->xmax = max(min(xmax,nx),1);
	bb_coord_new->ymax = max(min(ymax,ny),1);
}

static void update_bb(int inet, struct s_bb *bb_coord_new,
		struct s_bb *bb_edge_new, int xold, int yold, int xnew, int ynew) {

	/* Updates the bounding box of a net by storing its coordinates in    *
	 * the bb_coord_new data structure and the number of blocks on each   *
	 * edge in the bb_edge_new data structure.  This routine should only  *
	 * be called for large nets, since it has some overhead relative to   *
	 * just doing a brute force bounding box calculation.  The bounding   *
	 * box coordinate and edge information for inet must be valid before  *
	 * this routine is called.                                            *
	 * Currently assumes channels on both sides of the CLBs forming the   *
	 * edges of the bounding box can be used.  Essentially, I am assuming *
	 * the pins always lie on the outside of the bounding box.            */

	/* IO blocks are considered to be one cell in for simplicity. */

	xnew = max(min(xnew,nx),1);
	ynew = max(min(ynew,ny),1);
	xold = max(min(xold,nx),1);
	yold = max(min(yold,ny),1);

	/* Check if I can update the bounding box incrementally. */

	if (xnew < xold) { /* Move to left. */

		/* Update the xmax fields for coordinates and number of edges first. */

		if (xold == bb_coords[inet].xmax) { /* Old position at xmax. */
			if (bb_num_on_edges[inet].xmax == 1) {
				get_bb_from_scratch(inet, bb_coord_new, bb_edge_new);
				return;
			} else {
				bb_edge_new->xmax = bb_num_on_edges[inet].xmax - 1;
				bb_coord_new->xmax = bb_coords[inet].xmax;
			}
		}

		else { /* Move to left, old postion was not at xmax. */
			bb_coord_new->xmax = bb_coords[inet].xmax;
			bb_edge_new->xmax = bb_num_on_edges[inet].xmax;
		}

		/* Now do the xmin fields for coordinates and number of edges. */

		if (xnew < bb_coords[inet].xmin) { /* Moved past xmin */
			bb_coord_new->xmin = xnew;
			bb_edge_new->xmin = 1;
		}

		else if (xnew == bb_coords[inet].xmin) { /* Moved to xmin */
			bb_coord_new->xmin = xnew;
			bb_edge_new->xmin = bb_num_on_edges[inet].xmin + 1;
		}

		else { /* Xmin unchanged. */
			bb_coord_new->xmin = bb_coords[inet].xmin;
			bb_edge_new->xmin = bb_num_on_edges[inet].xmin;
		}
	} /* End of move to left case. */

	else if (xnew > xold) { /* Move to right. */

		/* Update the xmin fields for coordinates and number of edges first. */

		if (xold == bb_coords[inet].xmin) { /* Old position at xmin. */
			if (bb_num_on_edges[inet].xmin == 1) {
				get_bb_from_scratch(inet, bb_coord_new, bb_edge_new);
				return;
			} else {
				bb_edge_new->xmin = bb_num_on_edges[inet].xmin - 1;
				bb_coord_new->xmin = bb_coords[inet].xmin;
			}
		}

		else { /* Move to right, old position was not at xmin. */
			bb_coord_new->xmin = bb_coords[inet].xmin;
			bb_edge_new->xmin = bb_num_on_edges[inet].xmin;
		}

		/* Now do the xmax fields for coordinates and number of edges. */

		if (xnew > bb_coords[inet].xmax) { /* Moved past xmax. */
			bb_coord_new->xmax = xnew;
			bb_edge_new->xmax = 1;
		}

		else if (xnew == bb_coords[inet].xmax) { /* Moved to xmax */
			bb_coord_new->xmax = xnew;
			bb_edge_new->xmax = bb_num_on_edges[inet].xmax + 1;
		}

		else { /* Xmax unchanged. */
			bb_coord_new->xmax = bb_coords[inet].xmax;
			bb_edge_new->xmax = bb_num_on_edges[inet].xmax;
		}
	} /* End of move to right case. */

	else { /* xnew == xold -- no x motion. */
		bb_coord_new->xmin = bb_coords[inet].xmin;
		bb_coord_new->xmax = bb_coords[inet].xmax;
		bb_edge_new->xmin = bb_num_on_edges[inet].xmin;
		bb_edge_new->xmax = bb_num_on_edges[inet].xmax;
	}

	/* Now account for the y-direction motion. */

	if (ynew < yold) { /* Move down. */

		/* Update the ymax fields for coordinates and number of edges first. */

		if (yold == bb_coords[inet].ymax) { /* Old position at ymax. */
			if (bb_num_on_edges[inet].ymax == 1) {
				get_bb_from_scratch(inet, bb_coord_new, bb_edge_new);
				return;
			} else {
				bb_edge_new->ymax = bb_num_on_edges[inet].ymax - 1;
				bb_coord_new->ymax = bb_coords[inet].ymax;
			}
		}

		else { /* Move down, old postion was not at ymax. */
			bb_coord_new->ymax = bb_coords[inet].ymax;
			bb_edge_new->ymax = bb_num_on_edges[inet].ymax;
		}

		/* Now do the ymin fields for coordinates and number of edges. */

		if (ynew < bb_coords[inet].ymin) { /* Moved past ymin */
			bb_coord_new->ymin = ynew;
			bb_edge_new->ymin = 1;
		}

		else if (ynew == bb_coords[inet].ymin) { /* Moved to ymin */
			bb_coord_new->ymin = ynew;
			bb_edge_new->ymin = bb_num_on_edges[inet].ymin + 1;
		}

		else { /* ymin unchanged. */
			bb_coord_new->ymin = bb_coords[inet].ymin;
			bb_edge_new->ymin = bb_num_on_edges[inet].ymin;
		}
	} /* End of move down case. */

	else if (ynew > yold) { /* Moved up. */

		/* Update the ymin fields for coordinates and number of edges first. */

		if (yold == bb_coords[inet].ymin) { /* Old position at ymin. */
			if (bb_num_on_edges[inet].ymin == 1) {
				get_bb_from_scratch(inet, bb_coord_new, bb_edge_new);
				return;
			} else {
				bb_edge_new->ymin = bb_num_on_edges[inet].ymin - 1;
				bb_coord_new->ymin = bb_coords[inet].ymin;
			}
		}

		else { /* Moved up, old position was not at ymin. */
			bb_coord_new->ymin = bb_coords[inet].ymin;
			bb_edge_new->ymin = bb_num_on_edges[inet].ymin;
		}

		/* Now do the ymax fields for coordinates and number of edges. */

		if (ynew > bb_coords[inet].ymax) { /* Moved past ymax. */
			bb_coord_new->ymax = ynew;
			bb_edge_new->ymax = 1;
		}

		else if (ynew == bb_coords[inet].ymax) { /* Moved to ymax */
			bb_coord_new->ymax = ynew;
			bb_edge_new->ymax = bb_num_on_edges[inet].ymax + 1;
		}

		else { /* ymax unchanged. */
			bb_coord_new->ymax = bb_coords[inet].ymax;
			bb_edge_new->ymax = bb_num_on_edges[inet].ymax;
		}
	} /* End of move up case. */

	else { /* ynew == yold -- no y motion. */
		bb_coord_new->ymin = bb_coords[inet].ymin;
		bb_coord_new->ymax = bb_coords[inet].ymax;
		bb_edge_new->ymin = bb_num_on_edges[inet].ymin;
		bb_edge_new->ymax = bb_num_on_edges[inet].ymax;
	}
}

static void initial_placement(enum e_pad_loc_type pad_loc_type,
		char *pad_loc_file, int mem_density_x, int mem_density_y) {

	/* Randomly places the blocks to create an initial placement.     */

	struct s_pos {
		int x;
		int y;
	}*pos, *mem_pos;

	int i, j, k, count, iblk, choice, tsize, isubblk, mcount;

	tsize = max(nx*ny, 2*(nx+ny));
	int memsize = (int) (nx / mem_density_x) * (ny / mem_density_y);
	pos = (struct s_pos *) my_malloc(tsize * sizeof(struct s_pos));
	mem_pos = (struct s_pos *) my_malloc(memsize * sizeof(struct s_pos));
	/* Initialize all occupancy to zero. */

	for (i = 0; i <= nx + 1; i++) {
		for (j = 0; j <= ny + 1; j++) {
			clb[i][j].occ = 0;
		}
	}
	mcount = 0;
	count = 0;
	if (is_mem) {
		for (i = 1; i <= nx; i++) {
			for (j = 1; j <= ny; j++) {
				if (i % mem_density_x != 0 || j % mem_density_y != 0) {
					pos[count].x = i;
					pos[count].y = j;
					count++;
				} else {
					mem_pos[mcount].x = i;
					mem_pos[mcount].y = j;
					mcount++;
				}
			}
		}
	} else {
		for (i = 1; i <= nx; i++) {
			for (j = 1; j <= ny; j++) {
				pos[count].x = i;
				pos[count].y = j;
				count++;
			}
		}
	}

	for (iblk = 0; iblk < num_blocks; iblk++) {
		if (block[iblk].type == CLB) { /* only place CLBs in center */
			choice = my_irand(count - 1);
			clb[pos[choice].x][pos[choice].y].u.block = iblk;
			clb[pos[choice].x][pos[choice].y].occ = 1;

			/* Ensure randomizer doesn't pick this block again */
			pos[choice] = pos[count - 1]; /* overwrite used block position */
			count--;
		}
		if (block[iblk].type == MEM) { /* only place CLBs in center */
			choice = my_irand(mcount - 1);
			clb[mem_pos[choice].x][mem_pos[choice].y].u.block = iblk;
			clb[mem_pos[choice].x][mem_pos[choice].y].occ = 1;

			/* Ensure randomizer doesn't pick this block again */
			mem_pos[choice] = mem_pos[mcount - 1]; /* overwrite used block position */
			mcount--;
		}
	}

	/* Now do the io blocks around the periphery */

	if (pad_loc_type == USER) {
		read_user_pad_loc(pad_loc_file);
	} else { /* place_randomly. */
		count = 0;
		for (i = 1; i <= nx; i++) {
			pos[count].x = i;
			pos[count].y = 0;
			pos[count + 1].x = i;
			pos[count + 1].y = ny + 1;
			count += 2;
		}

		for (j = 1; j <= ny; j++) {
			pos[count].x = 0;
			pos[count].y = j;
			pos[count + 1].x = nx + 1;
			pos[count + 1].y = j;
			count += 2;
		}

		for (iblk = 0; iblk < num_blocks; iblk++) {
			if (block[iblk].type == INPAD || block[iblk].type == OUTPAD) {
				choice = my_irand(count - 1);
				isubblk = clb[pos[choice].x][pos[choice].y].occ;
				clb[pos[choice].x][pos[choice].y].u.io_blocks[isubblk] = iblk;
				clb[pos[choice].x][pos[choice].y].occ++;
				if (clb[pos[choice].x][pos[choice].y].occ == io_rat) {
					/* Ensure randomizer doesn't pick this block again */
					pos[choice] = pos[count - 1]; /* overwrite used block position */
					count--;
				}
			}
		}
	} /* End randomly place IO blocks branch of if */

	/* All the blocks are placed now.  Make the block array agree with the    *
	 * clb array.                                                             */

	for (i = 0; i <= nx + 1; i++) {
		for (j = 0; j <= ny + 1; j++) {
			if (clb[i][j].type == CLB || clb[i][j].type == MEM) {
				if (clb[i][j].occ == 1) {
					block[clb[i][j].u.block].x = i;
					block[clb[i][j].u.block].y = j;
				}
			} else {
				if (clb[i][j].type == IO) {
					for (k = 0; k < clb[i][j].occ; k++) {
						block[clb[i][j].u.io_blocks[k]].x = i;
						block[clb[i][j].u.io_blocks[k]].y = j;
					}
				}
			}
		}
	}

#ifdef VERBOSE
	printf("At end of initial_placement.\n");
	dump_clbs();
#endif

	free(pos);
}

static void free_fast_cost_update_structs(void) {

	/* Frees the structures used to speed up evaluation of the nonlinear   *
	 * congestion cost function.                                           */

	int i;

	for (i = 0; i <= ny; i++)
		free(chanx_place_cost_fac[i]);

	free(chanx_place_cost_fac);

	for (i = 0; i <= nx; i++)
		free(chany_place_cost_fac[i]);

	free(chany_place_cost_fac);
}

static void alloc_and_load_for_fast_cost_update(float place_cost_exp) {

	/* Allocates and loads the chanx_place_cost_fac and chany_place_cost_fac *
	 * arrays with the inverse of the average number of tracks per channel   *
	 * between [subhigh] and [sublow].  This is only useful for the cost     *
	 * function that takes the length of the net bounding box in each        *
	 * dimension divided by the average number of tracks in that direction.  *
	 * For other cost functions, you don't have to bother calling this       *
	 * routine; when using the cost function described above, however, you   *
	 * must always call this routine after you call init_chan and before     *
	 * you do any placement cost determination.  The place_cost_exp factor   *
	 * specifies to what power the width of the channel should be taken --   *
	 * larger numbers make narrower channels more expensive.                 */

	int low, high, i;

	/* Access arrays below as chan?_place_cost_fac[subhigh][sublow].  Since   *
	 * subhigh must be greater than or equal to sublow, we only need to       *
	 * allocate storage for the lower half of a matrix.                       */

	chanx_place_cost_fac = (float **) my_malloc((ny + 1) * sizeof(float *));
	for (i = 0; i <= ny; i++)
		chanx_place_cost_fac[i] = (float *) my_malloc((i + 1) * sizeof(float));

	chany_place_cost_fac = (float **) my_malloc((nx + 1) * sizeof(float *));
	for (i = 0; i <= nx; i++)
		chany_place_cost_fac[i] = (float *) my_malloc((i + 1) * sizeof(float));

	/* First compute the number of tracks between channel high and channel *
	 * low, inclusive, in an efficient manner.                             */

	chanx_place_cost_fac[0][0] = chan_width_x[0];

	for (high = 1; high <= ny; high++) {
		chanx_place_cost_fac[high][high] = chan_width_x[high];
		for (low = 0; low < high; low++) {
			chanx_place_cost_fac[high][low] =
					chanx_place_cost_fac[high - 1][low] + chan_width_x[high];
		}
	}

	/* Now compute the inverse of the average number of tracks per channel *
	 * between high and low.  The cost function divides by the average     *
	 * number of tracks per channel, so by storing the inverse I convert   *
	 * this to a faster multiplication.  Take this final number to the     *
	 * place_cost_exp power -- numbers other than one mean this is no      *
	 * longer a simple "average number of tracks"; it is some power of     *
	 * that, allowing greater penalization of narrow channels.             */

	for (high = 0; high <= ny; high++)
		for (low = 0; low <= high; low++) {
			chanx_place_cost_fac[high][low] = (high - low + 1.)
					/ chanx_place_cost_fac[high][low];
			chanx_place_cost_fac[high][low] = pow(
					(double) chanx_place_cost_fac[high][low],
					(double) place_cost_exp);
		}

	/* Now do the same thing for the y-directed channels.  First get the  *
	 * number of tracks between channel high and channel low, inclusive.  */

	chany_place_cost_fac[0][0] = chan_width_y[0];

	for (high = 1; high <= nx; high++) {
		chany_place_cost_fac[high][high] = chan_width_y[high];
		for (low = 0; low < high; low++) {
			chany_place_cost_fac[high][low] =
					chany_place_cost_fac[high - 1][low] + chan_width_y[high];
		}
	}

	/* Now compute the inverse of the average number of tracks per channel *
	 * between high and low.  Take to specified power.                     */

	for (high = 0; high <= nx; high++)
		for (low = 0; low <= high; low++) {
			chany_place_cost_fac[high][low] = (high - low + 1.)
					/ chany_place_cost_fac[high][low];
			chany_place_cost_fac[high][low] = pow(
					(double) chany_place_cost_fac[high][low],
					(double) place_cost_exp);
		}
}

static void check_place(float bb_cost, float timing_cost, int place_cost_type,
		int num_regions, enum e_place_algorithm place_algorithm,
		float delay_cost) {

	/* Checks that the placement has not confused our data structures. *
	 * i.e. the clb and block structures agree about the locations of  *
	 * every block, blocks are in legal spots, etc.  Also recomputes   *
	 * the final placement cost from scratch and makes sure it is      *
	 * within roundoff of what we think the cost is.                   */

	static int *bdone;
	int i, j, k, error = 0, bnum, istage;
	float bb_cost_check;
	float timing_cost_check, delay_cost_check;

	bb_cost_check = comp_bb_cost(CHECK, place_cost_type, num_regions);
	printf("bb_cost recomputed from scratch is %g.\n", bb_cost_check);
	if (fabs(bb_cost_check - bb_cost) > bb_cost * ERROR_TOL) {
		printf(
				"Error:  bb_cost_check: %g and bb_cost: %g differ in check_place.\n",
				bb_cost_check, bb_cost);
		error++;
	}

	if (place_algorithm == NET_TIMING_DRIVEN_PLACE
			|| place_algorithm == PATH_TIMING_DRIVEN_PLACE) {
		comp_td_costs(&timing_cost_check, &delay_cost_check);
		printf("timing_cost recomputed from scratch is %g. \n",
				timing_cost_check);
		if (fabs(timing_cost_check - timing_cost) > timing_cost * ERROR_TOL) {
			printf("Error:  timing_cost_check: %g and timing_cost: "
					"%g differ in check_place.\n", timing_cost_check,
					timing_cost);
			error++;
		}
		printf("delay_cost recomputed from scratch is %g. \n",
				delay_cost_check);
		if (fabs(delay_cost_check - delay_cost) > delay_cost * ERROR_TOL) {
			printf("Error:  delay_cost_check: %g and delay_cost: "
					"%g differ in check_place.\n", delay_cost_check,
					delay_cost);
			error++;
		}
	}

	bdone = (int *) my_malloc(num_blocks * sizeof(int));
	for (i = 0; i < num_blocks; i++)
		bdone[i] = 0;

	/* Step through clb array. Check it against block array. */
	if (!is_folding) {
		for (i = 0; i <= nx + 1; i++)
			for (j = 0; j <= ny + 1; j++) {
				if (clb[i][j].occ == 0)
					continue;
				if (clb[i][j].type == CLB) {
					bnum = clb[i][j].u.block;
					if (block[bnum].type != CLB) {
						printf(
								"Error:  block %d type does not match clb(%d,%d) type.\n",
								bnum, i, j);
						error++;
					}
					if ((block[bnum].x != i) || (block[bnum].y != j)) {
						printf(
								"Error:  block %d location conflicts with clb(%d,%d)"
										"data.\n", bnum, i, j);
						error++;
					}
					if (clb[i][j].occ > 1) {
						printf("Error: clb(%d,%d) has occupancy of %d\n", i, j,
								clb[i][j].occ);
						error++;
					}
					bdone[bnum]++;
				} else if (clb[i][j].type == MEM) {
					bnum = clb[i][j].u.block;
					if (block[bnum].type != MEM) {
						printf(
								"Error:  block %d type does not match clb(%d,%d) type.\n",
								bnum, i, j);
						error++;
					}
					if ((block[bnum].x != i) || (block[bnum].y != j)) {
						printf(
								"Error:  block %d location conflicts with clb(%d,%d)"
										"data.\n", bnum, i, j);
						error++;
					}
					if (clb[i][j].occ > 1) {
						printf("Error: clb(%d,%d) has occupancy of %d\n", i, j,
								clb[i][j].occ);
						error++;
					}
					bdone[bnum]++;
				} else if (clb[i][j].type == DSP) {
					if (clb[i][j].x_off == 0 && clb[i][j].y_off == 0) {
						bnum = clb[i][j].u.block;
						if (block[bnum].type != DSP) {
							printf(
									"Error:  block %d type does not match clb(%d,%d) type.\n",
									bnum, i, j);
							error++;
						}
						if ((block[bnum].x != i) || (block[bnum].y != j)) {
							printf(
									"Error:  block %d location conflicts with clb(%d,%d)"
											"data.\n", bnum, i, j);
							error++;
						}
						if (clb[i][j].occ > 1) {
							printf("Error: clb(%d,%d) has occupancy of %d\n", i,
									j, clb[i][j].occ);
							error++;
						}
						bdone[bnum]++;
					}

				} else { /* IO block */
					if (clb[i][j].occ > io_rat) {
						printf("Error:  clb(%d,%d) has occupancy of %d\n", i, j,
								clb[i][j].occ);
						error++;
					}
					for (k = 0; k < clb[i][j].occ; k++) {
						bnum = clb[i][j].u.io_blocks[k];
						if ((block[bnum].type != INPAD)
								&& block[bnum].type != OUTPAD) {
							printf(
									"Error:  block %d type does not match clb(%d,%d) type.\n",
									bnum, i, j);
							error++;
						}
						if ((block[bnum].x != i) || (block[bnum].y != j)) {
							printf(
									"Error:  block %d location conflicts with clb(%d,%d)"
											"data.\n", bnum, i, j);
							error++;
						}
						bdone[bnum]++;
					}
				}
			}

		/* Check that every block exists in the clb and block arrays somewhere. */
		for (i = 0; i < num_blocks; i++) {
			if (bdone[i] != 1) {
				printf("Error:  block %d listed %d times in data structures.\n",
						i, bdone[i]);
				error++;
			}
		}
		free(bdone);

		if (error == 0) {
			printf("\nCompleted placement consistency check successfully.\n\n");
		} else {
			printf(
					"\nCompleted placement consistency check, %d Errors found.\n\n",
					error);
			printf("Aborting program.\n");
			exit(1);
		}
	} else {
		for (istage = 0; istage < num_stage; istage++) {
			for (i = 0; i <= nx + 1; i++) {
				for (j = 0; j <= ny + 1; j++) {
					if (stage_clb[istage][i][j].occ == 0)
						continue;
					if (stage_clb[istage][i][j].type == CLB) {
						//printf("come for CLB\n");
						bnum = stage_clb[istage][i][j].u.block;
						if (block[bnum].type != CLB) {
							printf(
									"Error:  block %d type does not match clb(%d,%d) type.\n",
									bnum, i, j);
							error++;
						}
						if ((block[bnum].x != i) || (block[bnum].y != j)) {
							printf(
									"Error:  block %d location conflicts with clb(%d,%d)"
											"data.\n", bnum, i, j);
							error++;
						}
						if (stage_clb[istage][i][j].occ > 1) {
							printf(
									"Error: clb(%d,%d) in stage %d has occupancy of %d\n",
									i, j, istage, stage_clb[istage][i][j].occ);
							error++;
						}
						bdone[bnum]++;
					} else if (stage_clb[istage][i][j].type == MEM) {
						//printf("come for CLB\n");
						bnum = stage_clb[istage][i][j].u.block;
						if (block[bnum].type != MEM) {
							printf(
									"Error:  block %d type does not match clb(%d,%d) type.\n",
									bnum, i, j);
							error++;
						}
						if ((block[bnum].x != i) || (block[bnum].y != j)) {
							printf(
									"Error:  block %d location conflicts with clb(%d,%d)"
											"data.\n", bnum, i, j);
							error++;
						}
						if (stage_clb[istage][i][j].occ > 1) {
							printf(
									"Error: clb(%d,%d) in stage %d has occupancy of %d\n",
									i, j, istage, stage_clb[istage][i][j].occ);
							error++;
						}
						bdone[bnum]++;
					} else if (stage_clb[istage][i][j].type == DSP) {
						if (stage_clb[istage][i][j].x_off == 0
								&& stage_clb[istage][i][j].y_off == 0) {
							bnum = stage_clb[istage][i][j].u.block;
							if (block[bnum].type != DSP) {
								printf(
										"Error:  block %d type does not match clb(%d,%d) type.\n",
										bnum, i, j);
								error++;
							}
							if ((block[bnum].x != i) || (block[bnum].y != j)) {
								printf(
										"Error:  block %d location conflicts with clb(%d,%d)"
												"data.\n", bnum, i, j);
								error++;
							}
							if (stage_clb[istage][i][j].occ > 1) {
								printf(
										"Error: clb(%d,%d) has occupancy of %d\n",
										i, j, stage_clb[istage][i][j].occ);
								error++;
							}
							bdone[bnum]++;
						}

					} else { /* IO block */
						//printf("come for pad\n");
						if (stage_clb[istage][i][j].occ > io_rat) {
							printf(
									"Error:  clb(%d,%d) in stage %d has occupancy of %d\n",
									i, j, istage, stage_clb[istage][i][j].occ);
							error++;
						}
						for (k = 0; k < stage_clb[istage][i][j].occ; k++) {
							if (stage_clb[istage][i][j].u.io_blocks[k] != EMPTY) {
								bnum = stage_clb[istage][i][j].u.io_blocks[k];
								if ((block[bnum].type != INPAD)
										&& block[bnum].type != OUTPAD) {
									printf(
											"Error:  block %d type does not match clb(%d,%d) type.\n",
											bnum, i, j);
									error++;
								}
								if ((block[bnum].x != i)
										|| (block[bnum].y != j)) {
									printf(
											"Error:  block %d location conflicts with clb(%d,%d)"
													"data.\n", bnum, i, j);
									error++;
								}
								bdone[bnum]++;
							}
						}
					}
				}
			}
		}

		/* Check that every block exists in the clb and block arrays somewhere. */
		for (i = 0; i < num_blocks; i++)
			if (bdone[i] != 1) {
				printf("Error:  block %d listed %d times in data structures.\n",
						i, bdone[i]);
				error++;
			}
		free(bdone);

		if (error == 0) {
			printf("\nCompleted placement consistency check successfully.\n\n");
		} else {
			printf(
					"\nCompleted placement consistency check, %d Errors found.\n\n",
					error);
			printf("Aborting program.\n");
			exit(1);
		}
	}
}

void read_place(char *place_file, char *net_file, char *arch_file,
		struct s_placer_opts placer_opts, struct s_router_opts router_opts,
		t_chan_width_dist chan_width_dist,
		struct s_det_routing_arch det_routing_arch, t_segment_inf *segment_inf,
		t_timing_inf timing_inf, t_subblock_data *subblock_data_ptr) {

	/* Reads in a previously computed placement of the circuit.  It      *
	 * checks that the placement corresponds to the current architecture *
	 * and netlist file.                                                 */

	char msg[BUFSIZE];
	int chan_width_factor, num_connections, inet, ipin;
	float bb_cost, delay_cost, timing_cost, est_crit;
	float **dummy_x, **dummy_y;
	float **net_slack, **net_delay;
	float **remember_net_delay_original_ptr; /*used to free net_delay if it is re-assigned*/

	remember_net_delay_original_ptr = NULL; /*prevents compiler warning*/

	if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
			|| placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
			|| placer_opts.enable_timing_computations) {
		/*this must be called before alloc_and_load_placement_structs *
		 *and parse_placement_file since it modifies the structures*/
		alloc_lookups_and_criticalities(chan_width_dist, router_opts,
				det_routing_arch, segment_inf, timing_inf, *subblock_data_ptr,
				&net_delay, &net_slack);

		num_connections = count_connections();

		remember_net_delay_original_ptr = net_delay;
	} else {
		num_connections = 0;
	}

	/* First read in the placement.   */

	parse_placement_file(place_file, net_file, arch_file);

	/* Load the channel occupancies and cost factors so that:   *
	 * (1) the cost check will be OK, and                       *
	 * (2) the geometry will draw correctly.                    */

	chan_width_factor = placer_opts.place_chan_width;
	init_chan(chan_width_factor, chan_width_dist);

	/* NB:  dummy_x and dummy_y used because I'll never use the old_place_occ *
	 * arrays in this routine.  I need the placement structures loaded for    *
	 * comp_cost and check_place to work.                                     */

	alloc_and_load_placement_structs(placer_opts.place_cost_type,
			placer_opts.num_regions, placer_opts.place_cost_exp, &dummy_x,
			&dummy_y, placer_opts);

	/* Need cost in order to call check_place. */

	bb_cost = comp_bb_cost(NORMAL, placer_opts.place_cost_type,
			placer_opts.num_regions);

	if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
			|| placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
			|| placer_opts.enable_timing_computations) {

		for (inet = 0; inet < num_nets; inet++)
			for (ipin = 1; ipin < net[inet].num_pins; ipin++)
				timing_place_crit[inet][ipin] = 0; /*dummy crit values*/

		comp_td_costs(&timing_cost, &delay_cost); /*set up point_to_point_delay_cost*/

		net_delay = point_to_point_delay_cost; /*this keeps net_delay up to date with the *
		 *same values that the placer is using     */
		load_timing_graph_net_delays(net_delay);
		est_crit = load_net_slack(net_slack, 0, FALSE);

		printf("Placement. bb_cost: %g  delay_cost: %g.\n\n", bb_cost,
				delay_cost);
#ifdef PRINT_SINK_DELAYS
		print_sink_delays("Placement_Sink_Delays.echo");
#endif
#ifdef PRINT_NET_SLACKS
		print_net_slack("Placement_Net_Slacks.echo", net_slack);
#endif
#ifdef PRINT_PLACE_CRIT_PATH
		print_critical_path("Placement_Crit_Path.echo");
#endif
		printf("Placement Estimated Crit Path Delay: %g\n\n", est_crit);
	} else {
		timing_cost = 0;
		delay_cost = 0;
		printf("Placement bb_cost is %g.\n", bb_cost);
	}
	check_place(bb_cost, timing_cost, placer_opts.place_cost_type,
			placer_opts.num_regions, placer_opts.place_algorithm, delay_cost);

	free_placement_structs(placer_opts.place_cost_type, placer_opts.num_regions,
			dummy_x, dummy_y, placer_opts);

	if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
			|| placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
			|| placer_opts.enable_timing_computations) {
		net_delay = remember_net_delay_original_ptr;
		free_lookups_and_criticalities(&net_delay, &net_slack);
	}

	init_draw_coords((float) chan_width_factor);

	sprintf(msg, "Placement from file %s.  bb_cost %g.", place_file, bb_cost);
	update_screen(MAJOR, msg, PLACEMENT, FALSE);
}

/* Added by Wei */
/* Modified by LiangHao */
void fill_stage_arch(int density_x, int density_y) {

	/* Fill some of the FPGA architecture data structures.         */

	int i, j, k, *index;
	int m, n;

	/* allocate io_blocks arrays. Done this way to save storage */

	i = 2 * io_rat * (nx + ny) * num_stage;
	index = (int *) my_malloc(i * sizeof(int));
	for (k = 0; k < num_stage; k++) {
		for (i = 1; i <= nx; i++) {
			stage_clb[k][i][0].u.io_blocks = index;
			index += io_rat;
			stage_clb[k][i][ny + 1].u.io_blocks = index;
			index += io_rat;
		}
		for (i = 1; i <= ny; i++) {
			stage_clb[k][0][i].u.io_blocks = index;
			index += io_rat;
			stage_clb[k][nx + 1][i].u.io_blocks = index;
			index += io_rat;
		}

		/* Initialize type, and occupancy. */
		for (i = 1; i <= nx; i++) {
			stage_clb[k][i][0].type = IO;
			stage_clb[k][i][ny + 1].type = IO; /* perimeter (IO) cells */
			stage_clb[k][i][0].x_off = stage_clb[k][i][0].y_off = 0;
			stage_clb[k][i][ny + 1].x_off = stage_clb[k][i][ny + 1].y_off = 0;
		}

		for (i = 1; i <= ny; i++) {
			stage_clb[k][0][i].type = IO;
			stage_clb[k][nx + 1][i].type = IO;
			stage_clb[k][0][i].x_off = stage_clb[k][0][i].y_off = 0;
			stage_clb[k][nx + 1][i].x_off = stage_clb[k][nx + 1][i].y_off = 0;
		}

		for (i = 1; i <= nx; i++) { /* interior (LUT) cells */
			for (j = 1; j <= ny; j++) {
				if (!is_mem)
					stage_clb[k][i][j].type = CLB;
				else {
					if (i % density_x == 0 && j % density_y == 0)
						stage_clb[k][i][j].type = MEM;
					else
						stage_clb[k][i][j].type = CLB;
				}

				stage_clb[k][i][j].x_off = 0;
				stage_clb[k][i][j].y_off = 0;
			}
		}

		for (i = dsp_loc_start; i <= nx; i += dsp_loc_repeat) {
			for (j = 1; j <= ny; j += dsp_h) {
				if (i + dsp_w - 1 <= nx && j + dsp_h - 1 <= ny) {
					for (m = i; m < i + dsp_w; ++m) {
						for (n = j; n < j + dsp_h; ++n) {
							stage_clb[k][m][n].type = DSP;
							stage_clb[k][m][n].x_off = m - i;
							stage_clb[k][m][n].y_off = n - j;
						}
					}
				}
			}
		}

		/* Nothing goes in the corners.      */

		stage_clb[k][0][0].type = stage_clb[k][nx + 1][0].type = ILLEGAL;
		stage_clb[k][0][ny + 1].type = stage_clb[k][nx + 1][ny + 1].type =
				ILLEGAL;
	}
}

static void initial_stage_placement(enum e_pad_loc_type pad_loc_type,
		char *pad_loc_file, int mem_density_x, int mem_density_y) {

	/* Randomly places the blocks to create an initial placement.     */

	struct s_pos {
		int x;
		int y;
	}**pos, *mem_pos, *dsp_pos;
	int i, j, k, l, count, iblk, choice, tsize, memsize, mem_count, isubblk,
			cstage, dsp_num;
	int *pos_fixed = (int*) my_malloc(num_blocks * sizeof(int));
	tsize = max(nx*ny, 2*(nx+ny));

	memsize = (int) (nx / mem_density_x * ny / mem_density_y);
	dsp_num = ((int) ((nx - dsp_loc_start + 1) / dsp_loc_repeat))
			* ((int) (ny / dsp_h));
	if (((nx - dsp_loc_start + 1) % dsp_loc_repeat) >= dsp_w)
		dsp_num += (int) (ny / dsp_h);
	dsp_pos = (struct s_pos *) my_malloc(dsp_num * sizeof(struct s_pos));
	mem_pos = (struct s_pos *) my_malloc(memsize * sizeof(struct s_pos));
	pos = (struct s_pos **) alloc_matrix(0, num_stage - 1, 0, tsize - 1,
			sizeof(struct s_pos));
	int *count_index = (int*) my_malloc(num_stage * sizeof(int));
	char *name;
	struct s_hash *h_ptr;
	int begin, bcount, nblk, iter, spec_pos;
	int tar_x, tar_y;
	boolean feasible, if_find;

//nionio added
	boolean io_success;
	int pad_count;
	int dsp_count;

	/* Initialize all occupancy to zero. */
	for (i = 0; i < num_blocks; i++) {
		pos_fixed[i] = -1;
		//printf("the block %d(%s)\n", i, block[i].name);
	}

	for (k = 0; k < num_stage; k++) {
		for (i = 0; i <= nx + 1; i++) {
			for (j = 0; j <= ny + 1; j++) {
				stage_clb[k][i][j].occ = 0;
			}
		}
		mem_count = 0;
		count = 0;
		dsp_count = 0;
		for (i = 1; i <= nx; i++) {
			for (j = 1; j <= ny; j++) {
				if (is_mem) {
					if (i % mem_density_x != 0 || j % mem_density_y != 0) {
						pos[k][count].x = i;
						pos[k][count].y = j;
						// printf("count is %d, x %d, y %d\n", count, i, j);
						count++;
					} else {
						mem_pos[mem_count].x = i;
						mem_pos[mem_count].y = j;
						mem_count++;
						//printf("mem_count here %d\n", mem_count);
					}
				} else {
					if (stage_clb[k][i][j].type == DSP
							&& stage_clb[k][i][j].x_off == 0
							&& stage_clb[k][i][j].y_off == 0) {
						dsp_pos[dsp_count].x = i;
						dsp_pos[dsp_count].y = j;
						dsp_count++;
					} else if (stage_clb[k][i][j].type == CLB) {
						pos[k][count].x = i;
						pos[k][count].y = j;
						// printf("count is %d, x %d, y %d\n", count, i, j);
						count++;
					}
				}

			}
		}
		count_index[k] = count;
	}

// for(k=0; k<num_stage; k++)
// printf("count_index %d\n", count_index[k]);

	printf("mem_count %d\n", mem_count);
//printf("nx is %d, ny is %d\n", nx, ny);
	for (iblk = 0; iblk < num_blocks; iblk++) {
		if (block[iblk].type == CLB) { /* only place CLBs in center */
			if (pos_fixed[iblk] == -1) {
				iter = 0;
				feasible = FALSE;
				name = block[iblk].name;
				h_ptr = get_hash_entry(map_table, name);
				begin = h_ptr->index;
				bcount = h_ptr->count;
				cstage = block[iblk].stage;
				// printf("block %d(%s) at stage %d\n", k, block[nblk].name, cstage);
				while (!feasible) {
					feasible = TRUE;
					// printf("block %s, stage %d, count %d\n", block[iblk].name, cstage, count_index[cstage-1]);
					choice = my_irand(count_index[cstage - 1] - 1);
					// printf("the choice is %d\n",choice);
					tar_x = pos[cstage - 1][choice].x;
					tar_y = pos[cstage - 1][choice].y;
					//printf("the x %d, y %d \n",tar_x, tar_y);
					for (k = begin; k < begin + bcount; k++) {
						nblk = blockmap_inf[k];
						cstage = block[nblk].stage;
						if_find = FALSE;
						//printf("block %d(%s) at stage %d\n", k, block[nblk].name, cstage);
						//printf("count index %d\n", count_index[cstage-1]);
						for (i = 0; i < count_index[cstage - 1]; i++) {
							if ((pos[cstage - 1][i].x == tar_x)
									&& (pos[cstage - 1][i].y == tar_y)) {
								if_find = TRUE;
								//printf("find here %d with x %d, y %d\n", i, pos[cstage-1][i].x, pos[cstage-1][i].y);
								break;
							}
						}
						if (!if_find)
							feasible = FALSE;
					}
					iter++;
					if (iter > 1000) {
						printf("cannot find a position\n", iter);
						exit(1);
					}
				}
				// printf("come here\n");
				for (k = begin; k < begin + bcount; k++) {
					nblk = blockmap_inf[k];
					cstage = block[nblk].stage;
					// printf("block %d(%s) at stage %d\n", k, block[nblk].name, cstage);
					//printf("pos %d %d\n", tar_x, tar_y);
					stage_clb[cstage - 1][tar_x][tar_y].u.block = nblk;
					stage_clb[cstage - 1][tar_x][tar_y].occ = 1;
					//printf("here\n");
					for (i = 0; i < count_index[cstage - 1]; i++) {
						if ((pos[cstage - 1][i].x == tar_x)
								&& (pos[cstage - 1][i].y == tar_y)) {
							spec_pos = i;
							//printf("spec pos %d\n", spec_pos);
							break;
						}
					}
					int temp_x, temp_y;
					temp_x = pos[cstage - 1][spec_pos].x;
					temp_y = pos[cstage - 1][spec_pos].y;
					pos[cstage - 1][spec_pos].x =
							pos[cstage - 1][count_index[cstage - 1] - 1].x; /* overwrite used block position */
					pos[cstage - 1][spec_pos].y =
							pos[cstage - 1][count_index[cstage - 1] - 1].y;
					pos[cstage - 1][count_index[cstage - 1] - 1].x = temp_x;
					pos[cstage - 1][count_index[cstage - 1] - 1].y = temp_y;
					count_index[cstage - 1]--;
					pos_fixed[nblk] = 1;
				}
				/* printf("check here\n");
				 for (k=0; k<num_stage; k++){
				 count =0;
				 for (i=1;i<=nx;i++) {
				 for (j=1;j<=ny;j++) {
				 printf("count is %d, x %d, y %d\n", count, pos[k][count].x, pos[k][count].y);
				 count++;
				 }
				 }
				 printf("the count index here %d\n", count_index[k]);
				 }
				 */
			}
		}
		if (block[iblk].type == DSP) { /* only place MEMs in center */
			if (pos_fixed[iblk] == -1) { //printf("memory\n");
										 //printf("mem_count %d\n", mem_count);
				choice = my_irand(dsp_count - 1);
				//printf("come here\n");
				/* for (k=0; k<num_stage; k++)
				 {stage_clb[k][mem_pos[choice].x][mem_pos[choice].y].u.block = EMPTY;
				 stage_clb[k][mem_pos[choice].x][mem_pos[choice].y].occ = 1;
				 }
				 */
				name = block[iblk].name;
				h_ptr = get_hash_entry(map_table, name);
				begin = h_ptr->index;
				bcount = h_ptr->count;
				for (k = begin; k < begin + bcount; k++) {
					nblk = blockmap_inf[k];
					pos_fixed[nblk] = 1;
					cstage = block[nblk].stage;
					stage_clb[cstage - 1][dsp_pos[choice].x][dsp_pos[choice].y].u.block =
							nblk;
					stage_clb[cstage - 1][dsp_pos[choice].x][dsp_pos[choice].y].occ =
							1;
				}
				/* Ensure randomizer doesn't pick this block again */
				dsp_pos[choice] = dsp_pos[dsp_count - 1]; /* overwrite used block position */
				dsp_count--;
			}
		}
		if (block[iblk].type == MEM) { /* only place MEMs in center */
			if (pos_fixed[iblk] == -1) { //printf("memory\n");
										 //printf("mem_count %d\n", mem_count);
				choice = my_irand(mem_count - 1);
				//printf("come here\n");
				/* for (k=0; k<num_stage; k++)
				 {stage_clb[k][mem_pos[choice].x][mem_pos[choice].y].u.block = EMPTY;
				 stage_clb[k][mem_pos[choice].x][mem_pos[choice].y].occ = 1;
				 }
				 */
				name = block[iblk].name;
				h_ptr = get_hash_entry(map_table, name);
				begin = h_ptr->index;
				bcount = h_ptr->count;
				for (k = begin; k < begin + bcount; k++) {
					nblk = blockmap_inf[k];
					pos_fixed[nblk] = 1;
					cstage = block[nblk].stage;
					stage_clb[cstage - 1][mem_pos[choice].x][mem_pos[choice].y].u.block =
							nblk;
					stage_clb[cstage - 1][mem_pos[choice].x][mem_pos[choice].y].occ =
							1;
				}
				/* Ensure randomizer doesn't pick this block again */
				mem_pos[choice] = mem_pos[mem_count - 1]; /* overwrite used block position */
				mem_count--;
			}
		}
	}

	/* Now do the io blocks around the periphery */

	for (i = 0; i < num_blocks; i++)
		pos_fixed[i] = -1;

	if (pad_loc_type == USER) {
		read_user_pad_loc(pad_loc_file);
	} else { /* place_randomly. */
		count = 0;
		for (i = 1; i <= nx; i++) {
			pos[0][count].x = i;
			pos[0][count].y = 0;
			pos[0][count + 1].x = i;
			pos[0][count + 1].y = ny + 1;
			count += 2;
		}

		for (j = 1; j <= ny; j++) {
			pos[0][count].x = 0;
			pos[0][count].y = j;
			pos[0][count + 1].x = nx + 1;
			pos[0][count + 1].y = j;
			count += 2;
		}
		printf("initial placement of PAD\n");
		for (iblk = 0; iblk < num_blocks; iblk++) {
			if (block[iblk].type == INPAD || block[iblk].type == OUTPAD) {
				if (pos_fixed[iblk] != -1)
					continue;
				//place PAD
				name = block[iblk].name;
				h_ptr = get_hash_entry(map_table, name);
				begin = h_ptr->index;
				bcount = h_ptr->count;
				if (strcmp(name, "clk") != 0) {
					io_success = FALSE;
					while (!io_success) {
						choice = my_irand(count - 1);
						//check stage_io_rat
						io_success = TRUE;
						for (k = begin; k < begin + bcount; k++) {
							nblk = blockmap_inf[k];
							cstage = block[nblk].stage;
							pad_count = 0;
							for (isubblk = 0;
									isubblk
											< stage_clb[cstage - 1][pos[0][choice].x][pos[0][choice].y].occ;
									isubblk++) {
								if (stage_clb[cstage - 1][pos[0][choice].x][pos[0][choice].y].u.io_blocks[isubblk]
										!= EMPTY)
									pad_count++;
							}
							if ((pad_count == stage_io_rat)) {
								io_success = FALSE;
								break;
							}
						}
						if (io_success)
							break;
					}
				}

				isubblk = stage_clb[0][pos[0][choice].x][pos[0][choice].y].occ;

				for (k = 0; k < num_stage; k++) {
					stage_clb[k][pos[0][choice].x][pos[0][choice].y].u.io_blocks[isubblk] =
							EMPTY;
					stage_clb[k][pos[0][choice].x][pos[0][choice].y].occ++; //no matter which cycle is, keep #PADs in the block
				}

				//place into the PAD
				for (k = begin; k < begin + bcount; k++) {
					nblk = blockmap_inf[k];
					pos_fixed[nblk] = 1;
					cstage = block[nblk].stage;
					stage_clb[cstage - 1][pos[0][choice].x][pos[0][choice].y].u.io_blocks[isubblk] =
							nblk;
				}

				//check io_rat
				if (stage_clb[0][pos[0][choice].x][pos[0][choice].y].occ
						== io_rat) {
					/* Ensure randomizer doesn't pick this block again */
					pos[0][choice] = pos[0][count - 1]; /* overwrite used block position */
					count--;
				}
			}
		}
	} /* End randomly place IO blocks branch of if */

	/* All the blocks are placed now.  Make the block array agree with the    *
	 * clb array.                                                             */
	for (l = 0; l < num_stage; l++) {
		for (i = 0; i <= nx + 1; i++) {
			for (j = 0; j <= ny + 1; j++) {
				if (stage_clb[l][i][j].type == CLB
						|| stage_clb[l][i][j].type == MEM) {
					if (stage_clb[l][i][j].occ == 1) {
						block[stage_clb[l][i][j].u.block].x = i;
						block[stage_clb[l][i][j].u.block].y = j;
					}
				} else if (stage_clb[l][i][j].type == IO) {
					for (k = 0; k < stage_clb[l][i][j].occ; k++) {
						if (stage_clb[l][i][j].u.io_blocks[k] != EMPTY) {
							block[stage_clb[l][i][j].u.io_blocks[k]].x = i;
							block[stage_clb[l][i][j].u.io_blocks[k]].y = j;
						}
					}
				} else if (stage_clb[l][i][j].type == DSP
						&& stage_clb[l][i][j].x_off == 0
						&& stage_clb[l][i][j].y_off == 0) {
					if (stage_clb[l][i][j].occ == 1) {
						block[stage_clb[l][i][j].u.block].x = i;
						block[stage_clb[l][i][j].u.block].y = j;
					}
				}
			}
		}
	}

#ifdef VERBOSE
	printf("At end of initial_placement.\n");
	dump_stage_clbs();
#endif

	free_matrix(pos, 0, num_stage - 1, 0, sizeof(int));
	free(count_index);
	free(pos_fixed);
	printf("finish placement initialization\n");
}

static void check_placement() {
	int l, i, j, k, off_num;
	int x, y, begin, bcount, cstage;
	boolean if_find, if_out;
	char* name;
	struct s_hash *h_ptr;
	int error = 0;

	for (l = 0; l < num_blocks; l++) {
		int stage = block[l].stage;
		name = block[l].name;
		x = block[l].x;
		y = block[l].y;
		if_find = FALSE;
		if_out = FALSE;

		if (stage_clb[stage - 1][x][y].type == CLB
				|| stage_clb[stage - 1][x][y].type == MEM
				|| stage_clb[stage - 1][x][y].type == DSP) {
			for (i = 0; i < num_stage; i++) {
				if (stage_clb[i][x][y].occ == 1) {
					if (stage_clb[i][x][y].u.block == l)
						if_find = TRUE;
				}
			}
			if (!if_find) {
				printf("error, didn't find in the stage_clb array\n");
				error++;
			}

			if (block[l].type == MEM) {
				h_ptr = get_hash_entry(map_table, name);
				begin = h_ptr->index;
				bcount = h_ptr->count;
				for (k = begin; k < begin + bcount; k++) {
					int nblk = blockmap_inf[k];
					cstage = block[nblk].stage;
					if ((block[nblk].x != x) || (block[nblk].y != y)) {
						printf("error, block %d(%s) not in the same position\n",
								nblk, block[nblk].name);
						error++;
					}
					if (block[nblk].type != MEM) {
						printf("error in CLB type \n");
						error++;
					}
					if ((stage_clb[cstage - 1][x][y].u.block != nblk)
							|| (stage_clb[cstage - 1][x][y].occ == 0)
							|| (stage_clb[cstage - 1][x][y].type != MEM)) {
						printf("error, different %d(%s) in the same position\n",
								stage_clb[cstage - 1][x][y].u.block,
								block[stage_clb[cstage - 1][x][y].u.block].name);
						error++;
					}
				}
			}

			if (block[l].type == CLB) {
				h_ptr = get_hash_entry(map_table, name);
				begin = h_ptr->index;
				bcount = h_ptr->count;
				for (k = begin; k < begin + bcount; k++) {
					int nblk = blockmap_inf[k];
					cstage = block[nblk].stage;
					if ((block[nblk].x != x) || (block[nblk].y != y)) {
						printf("error, block %d(%s) not in the same position\n",
								nblk, block[nblk].name);
						error++;
					}
					if (block[nblk].type != CLB) {
						printf("error in CLB type \n");
						error++;
					}
					if ((stage_clb[cstage - 1][x][y].u.block != nblk)
							|| (stage_clb[cstage - 1][x][y].occ == 0)
							|| (stage_clb[cstage - 1][x][y].type != CLB)) {
						printf("Error, different %d(%s) in the same position\n",
								stage_clb[cstage - 1][x][y].u.block,
								block[stage_clb[cstage - 1][x][y].u.block].name);
						error++;
					}
				}
			}

			if (block[l].type == DSP) {
				h_ptr = get_hash_entry(map_table, name);
				begin = h_ptr->index;
				bcount = h_ptr->count;
				for (k = begin; k < begin + bcount; k++) {
					int nblk = blockmap_inf[k];
					cstage = block[nblk].stage;
					if ((block[nblk].x != x) || (block[nblk].y != y)) {
						printf("error, block %d(%s) not in the same position\n",
								nblk, block[nblk].name);
						error++;
					}
					if (block[nblk].type != DSP) {
						printf("error in CLB type \n");
						error++;
					}
					if ((stage_clb[cstage - 1][x][y].u.block != nblk)
							|| (stage_clb[cstage - 1][x][y].occ == 0)
							|| (stage_clb[cstage - 1][x][y].type != DSP)) {
						printf("Error, different %d(%s) in the same position\n",
								stage_clb[cstage - 1][x][y].u.block,
								block[stage_clb[cstage - 1][x][y].u.block].name);
						error++;
					}
				}
			}
		}
		//for PADs
		else {
			for (off_num = 0; off_num < stage_clb[0][x][y].occ; off_num++) {
				for (i = 0; i < num_stage; i++) {
					if (stage_clb[i][x][y].u.io_blocks[off_num] == l) {
						//printf("find %s at (%d, %d) at stage %d \n", block[l].name, x, y, i);
						if_out = TRUE;
						break;
					}
					if (if_out)
						break;
				}
				if (if_out)
					break;
			}
			if (!if_out) {
				printf("error, didn't find in the stage_clb array\n");
				error++;
			}
			for (i = 0; i < num_stage; i++) {
				if (stage_clb[i][x][y].u.io_blocks[off_num] != EMPTY) {
					if (strcmp(
							block[stage_clb[i][x][y].u.io_blocks[off_num]].name,
							name) != 0) {
						printf(
								"Error, name: %s, there is another name %s in stage %d\n",
								name,
								block[stage_clb[i][x][y].u.io_blocks[off_num]].name,
								i);
						error++;
					}
				}
			}
			h_ptr = get_hash_entry(map_table, name);
			begin = h_ptr->index;
			bcount = h_ptr->count;
			for (k = begin; k < begin + bcount; k++) {
				int nblk = blockmap_inf[k];
				cstage = block[nblk].stage;
				if ((block[nblk].x != x) || (block[nblk].y != y)) {
					printf("Error, block %d(%s) not in the same position\n",
							nblk, block[nblk].name);
					error++;
				}
				if ((block[nblk].type != INPAD)
						&& (block[nblk].type != OUTPAD)) {
					printf("Error in PAD type %d for block %s\n",
							block[nblk].type, block[nblk].name);
					error++;
				}
				if (stage_clb[cstage - 1][x][y].u.io_blocks[off_num] != nblk) {
					printf(
							"Error, name: %s, different %d(%s) in the same position\n",
							name,
							stage_clb[cstage - 1][x][y].u.io_blocks[off_num],
							block[stage_clb[cstage - 1][x][y].u.io_blocks[off_num]].name);
					error++;
				}
			}
		}
	}
	if (error > 0) {
		printf("check the initial here\n");
		for (l = 0; l < num_blocks; l++) {
			printf("the block is %s\n", block[l].name);
			printf("the pos x is %d ", block[l].x);
			printf("the pos y is %d\n", block[l].y);
		}
		for (l = 0; l < num_stage; l++) {
			for (i = 0; i <= nx + 1; i++) {
				for (j = 0; j <= ny + 1; j++) {
					/*if (stage_clb[l][i][j].type == CLB||stage_clb[l][i][j].type == MEM) {
					 if (stage_clb[l][i][j].occ == 1)
					 printf("the block at %d %d in stage %d is %s\n", i, j, l, block[stage_clb[l][i][j].u.block].name);
					 }*/
					if (stage_clb[l][i][j].type == IO) {
						for (k = 0; k < stage_clb[l][i][j].occ; k++) {
							if (stage_clb[l][i][j].u.io_blocks[k] != EMPTY)
								printf(
										"the pad at %d %d %d in stage %d is %s\n",
										i, j, k, l,
										block[stage_clb[l][i][j].u.io_blocks[k]].name);
						}
					}
				}
			}
		}
		exit(1);
	}
}
