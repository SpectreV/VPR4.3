#include <stdio.h>
#include "util.h"
#include "vpr_types.h"
#include "globals.h"
#include "hash.h"
#include "vpr_utils.h"
#include "check_netlist.h"

/**************** Subroutines local to this module **************************/

static int check_connections_to_global_clb_pins(int inet);

static int check_clb_conn(int iblk, int num_conn);

static void check_for_multiple_sink_connections(void);

static int check_for_duplicate_block_names(void);

static int get_num_conn(int bnum);
/* Modified by Wei */
static int check_subblocks(int iblk, t_subblock_data *subblock_data_ptr);
static int check_ffblocks(int iblk, t_subblock_data *subblock_data_ptr);

static int check_connects(int *num_uses_of_clb_pin, int *num_uses_of_sblk_opin,
		int *num_uses_of_ff_opin, t_subblock_data *subblock_data_ptr, int iblk);

static int check_subblock_pin(int clb_pin, int min_val, int max_val,
		enum e_pin_type pin_type, int iblk, int isubblk,
		t_subblock *subblock_inf);

static int check_ffblock_pin(int clb_pin, int min_val, int max_val,
		enum e_pin_type pin_type, int iblk, int iff);

static int check_internal_subblock_connections(
		t_subblock_data *subblock_data_ptr, int iblk,
		int *num_uses_of_sblk_opin, int *num_uses_of_ff_opin);

static int check_clb_to_subblock_connections(int iblk, t_subblock *subblock_inf,
		t_ffblock *ffblock_inf, int num_subblocks, int num_ffblocks,
		int *num_uses_of_clb_pin);

static void check_structure(t_subblock_data *subblock_data_ptr);

/*********************** Subroutine definitions *****************************/

void check_netlist(t_subblock_data *subblock_data_ptr, int *num_driver) {

	/* This routine checks that the netlist makes sense, and sets the num_ff    *
	 * and num_const_gen members of subblock_data.                              */

	// printf("come to check netlist\n");
	int i, error, num_conn, max_sub;
	int *num_uses_of_clb_pin, *num_uses_of_sblk_opin, *num_subblocks_per_block;
	int *num_uses_of_ff_opin; /* Added by Wei */

	max_sub = subblock_data_ptr->max_subblocks_per_block;
	num_subblocks_per_block = subblock_data_ptr->num_subblocks_per_block;
	subblock_data_ptr->num_ff = 0;
	subblock_data_ptr->num_const_gen = 0;

	error = 0;
	num_uses_of_clb_pin = (int *) my_malloc(pins_per_clb * sizeof(int));
	num_uses_of_sblk_opin = (int *) my_malloc(
			subblock_data_ptr->max_subblocks_per_block * sizeof(int));
	num_uses_of_ff_opin = (int *) my_malloc(
			subblock_data_ptr->subblock_ff_size * max_sub * sizeof(int));

	/* Check that nets fanout and have a driver. */

	for (i = 0; i < num_nets; i++) {
		if (num_driver[i] != 1) {
			printf("Error:  net %s has %d signals driving it.\n", net[i].name,
					num_driver[i]);
			error++;
		}

		if ((net[i].num_pins - num_driver[i]) < 1) {
			printf("Error:  net %s has no fanout.\n", net[i].name);
			error++;
		}

		error += check_connections_to_global_clb_pins(i);
	}

	check_for_multiple_sink_connections();
	error += check_for_duplicate_block_names();

	//if(is_folding)
	// check_structure (subblock_data_ptr);

	/* Check that each block makes sense. */

	for (i = 0; i < num_blocks; i++) {
		num_conn = get_num_conn(i);

		if (block[i].type == CLB) {
			error += check_clb_conn(i, num_conn);
			error += check_subblocks(i, subblock_data_ptr);
			/* Modified by Wei */
			error += check_ffblocks(i, subblock_data_ptr);
			error += check_connects(num_uses_of_clb_pin, num_uses_of_sblk_opin,
					num_uses_of_ff_opin, subblock_data_ptr, i);

			//printf("error of CLB %d\n", error);
		} else { /* IO block */

			/* This error check is a redundant double check.                 */
			if (block[i].type == IO) {
				if (num_conn != 1) {
					printf("Error:  io block #%d (%s) of type %d"
							"has %d pins.\n", i, block[i].name, block[i].type,
							num_conn);
					error++;
				}

				/* IO blocks must have no subblock information. */
				if (num_subblocks_per_block[i] != 0) {
					printf("Error:  IO block #%d (%s) contains %d subblocks.\n"
							"Expected 0.\n", i, block[i].name,
							num_subblocks_per_block[i]);
					error++;
				}
				//printf("error of IO %d\n", error);
			}
			if (block[i].type == MEM)
				error += check_clb_conn(i, num_conn);
		}
	}

	free(num_uses_of_clb_pin);
	free(num_uses_of_sblk_opin);
	free(num_uses_of_ff_opin);

	if (error != 0) {
		printf("Found %d fatal Errors in the input netlist.\n", error);
		exit(1);
	}
}

static void check_structure(t_subblock_data *subblock_data_ptr) {
	int iblk, ipin, istage, num_subs, num_ffblocks, inet, j;
	struct s_hash *h_ptr;
	printf("checking structure\n");
	for (iblk = 0; iblk < num_blocks; iblk++) {
		printf("block name %s at stage %d\n", block[iblk].name,
				block[iblk].stage);
		num_ffblocks = subblock_data_ptr->num_ff_per_block[iblk];
		num_subs = subblock_data_ptr->num_subblocks_per_block[iblk];
		printf("num of sub %d, num of ff %d\n", num_subs, num_ffblocks);
		printf("check pin info \n");
		for (ipin = 0; ipin < pins_per_clb; ipin++) {
			inet = block[iblk].nets[ipin];
			if (inet != OPEN)
				printf("net %d(%s) at pin %d\n", inet, net[inet].name, ipin);
		}
		printf("check map table \n");
		h_ptr = get_hash_entry(map_table, block[iblk].name);
		int begin = h_ptr->index;
		for (j = 0; j < h_ptr->count; j++) {
			int block_num = blockmap_inf[begin + j];
			printf("block num %d(%s) at index %d\n", block_num,
					block[block_num].name, begin + j);
		}
	}
	for (istage = 0; istage < num_stage; istage++) {
		printf("net begin %d, num %d at stage %d\n",
				num_net_per_stage[istage].begin, num_net_per_stage[istage].num,
				istage);
		printf("block begin %d, num %d at stage %d\n",
				num_block_per_stage[istage].begin,
				num_block_per_stage[istage].num, istage);
	}
	for (inet = 0; inet < num_nets; inet++) {
		printf("net name %s at stage %d with num of pins %d\n", net[inet].name,
				net[inet].stage, net[inet].num_pins);
		int num_pin = net[inet].num_pins;
		for (ipin = 0; ipin < num_pin; ipin++) {
			iblk = net[inet].blocks[ipin];
			printf("connect to block %d(%s) at pin %d\n", iblk,
					block[iblk].name, net[inet].blk_pin[ipin]);
		}
	}
	return;
}

static int check_connections_to_global_clb_pins(int inet) {

	/* Checks that a global net (inet) connects only to global CLB input pins  *
	 * and that non-global nets never connects to a global CLB pin.  Either    *
	 * global or non-global nets are allowed to connect to pads.               */

	int ipin, num_pins, iblk, blk_pin, error;

	num_pins = net[inet].num_pins;
	error = 0;

	/* For now global signals can be driven by an I/O pad or any CLB output       *
	 * although a CLB output generates a warning.  I could make a global CLB      *
	 * output pin type to allow people to make architectures that didn't have     *
	 * this warning.                                                              */

	for (ipin = 0; ipin < num_pins; ipin++) {
		iblk = net[inet].blocks[ipin];

		if (block[iblk].type == CLB || block[iblk].type == MEM) { /* I/O pads are exempt. */
			blk_pin = net[inet].blk_pin[ipin];

			if (is_global_clb_pin[blk_pin] != is_global[inet]) {

				/* Allow a CLB output pin to drive a global net (warning only). */

				if (ipin == 0 && is_global[inet]) {
					printf("Warning in check_connections_to_global_clb_pins:\n"
							"\tnet #%d (%s) is driven by CLB output pin (#%d)\n"
							"\ton block #%d (%s).\n", inet, net[inet].name,
							blk_pin, iblk, block[iblk].name);
				} else { /* Otherwise -> Error */
					printf(
							"Error in check_connections_to_global_clb_pins:\n"
									"\tpin %d on net #%d (%s) connects to CLB input pin (#%d)\n"
									"\ton block #%d (%s).\n", ipin, inet,
							net[inet].name, blk_pin, iblk, block[iblk].name);
					error++;
				}

				if (is_global[inet])
					printf("\tNet is global, but CLB pin is not.\n\n");
				else
					printf("\tCLB pin is global, but net is not.\n\n");
			}
		}
	} /* End for all pins */

	return (error);
}

static int check_clb_conn(int iblk, int num_conn) {

	/* Checks that the connections into and out of the clb make sense.  */

	int iclass, ipin, error;

	error = 0;

	if (num_conn < 2) {
		//printf("Warning:  logic block #%d (%s) has only %d pin.\n",
		//iblk, block[iblk].name,num_conn);

		/* Allow the case where we have only one OUTPUT pin connected to continue. *
		 * This is used sometimes as a constant generator for a primary output,    *
		 * but I will still warn the user.  If the only pin connected is an input, *
		 * abort.                                                                  */

		if (num_conn == 1) {
			for (ipin = 0; ipin < pins_per_clb; ipin++) {
				if (block[iblk].nets[ipin] != OPEN) {
					iclass = clb_pin_class[ipin];

					if (class_inf[iclass].type != DRIVER) {
						//error++;
					} else {
						printf(
								"\tPin is an output -- may be a constant generator.\n"
										"\tNon-fatal, but check this.\n");
					}

					break;
				}
			}
		} else {
			// error++;
		}
	}

	/* This case should already have been flagged as an error -- this is *
	 * just a redundant double check.                                    */

	if (num_conn > pins_per_clb) {
		printf("Error:  logic block #%d with output %s has %d pins.\n", iblk,
				block[iblk].name, num_conn);
		error++;
	}

	return (error);
}

static int check_for_duplicate_block_names(void) {

	/* Checks that all blocks have duplicate names.  Returns the number of     *
	 * duplicate names.                                                        */

	int error, iblk;
	struct s_hash **block_hash_table, *h_ptr;
	struct s_hash_iterator hash_iterator;
	int stage; /* Added by Wei */

	error = 0;
	block_hash_table = alloc_hash_table();

	for (iblk = 0; iblk < num_blocks; iblk++) {
		if (!is_folding)
			h_ptr = insert_in_hash_table(block_hash_table, block[iblk].name,
					iblk);
		else {
			stage = block[iblk].stage;
			h_ptr = insert_in_hash_table_new(block_hash_table, block[iblk].name,
					iblk, stage);
		}
	}
	hash_iterator = start_hash_table_iterator();
	h_ptr = get_next_hash(block_hash_table, &hash_iterator);

	while (h_ptr != NULL) {
		if (h_ptr->count != 1) {
			printf(
					"Error:  %d blocks are named %s.  Block names must be unique."
							"\n", h_ptr->count, h_ptr->name);
			error++;
		}
		h_ptr = get_next_hash(block_hash_table, &hash_iterator);
	}

	free_hash_table(block_hash_table);
	return (error);
}

/* Modified by Wei */
static int check_ffblocks(int iblk, t_subblock_data *subblock_data_ptr) {

	/* This routine checks the subblocks of iblk (which must be a CLB).  It    *
	 * returns the number of errors found.                                     */

	int error, iff, ipin, clb_pin;
	int num_ffblocks, subblock_ff_size, max_subblock;
	t_ffblock *ffblock_inf;

	error = 0;
	ffblock_inf = subblock_data_ptr->ffblock_inf[iblk];
	num_ffblocks = subblock_data_ptr->num_ff_per_block[iblk];
	subblock_ff_size = subblock_data_ptr->subblock_ff_size;
	max_subblock = subblock_data_ptr->max_subblocks_per_block;

	if (num_ffblocks > subblock_ff_size * max_subblock) {
		printf("Error:  block #%d (%s) contains %d ffs.\n", iblk,
				block[iblk].name, num_ffblocks);
		error++;
	}

	/* Check that all pins connect to the proper type of CLB pin and are in the *
	 * correct range.                                                           */

	for (iff = 0; iff < num_ffblocks; iff++) {

		/* Input pins */
		clb_pin = ffblock_inf[iff].input;
		error += check_ffblock_pin(clb_pin, 0,
				pins_per_clb + max_subblock * (1 + subblock_ff_size) - 1,
				RECEIVER, iblk, iff);

		/* Subblock output pin. */

		clb_pin = ffblock_inf[iff].output;
		error += check_ffblock_pin(clb_pin, 0, pins_per_clb - 1, DRIVER, iblk,
				iff);

		/* Subblock clock pin. */

		clb_pin = ffblock_inf[iff].clock;
		error += check_ffblock_pin(clb_pin, 0,
				pins_per_clb + max_subblock * (1 + subblock_ff_size) - 1,
				RECEIVER, iblk, iff);

	} /* End ffblock for loop. */

	/* If pins out of range, return.  Could get seg faults otherwise. */

	return (error);
}

static int check_subblocks(int iblk, t_subblock_data *subblock_data_ptr) {

	/* This routine checks the subblocks of iblk (which must be a CLB).  It    *
	 * returns the number of errors found.                                     */

	int error, isub, ipin, clb_pin;
	int num_subblocks, max_subblocks_per_block, subblock_lut_size;
	t_subblock *subblock_inf;
	int subblock_ff_size;

	error = 0;
	subblock_inf = subblock_data_ptr->subblock_inf[iblk];
	num_subblocks = subblock_data_ptr->num_subblocks_per_block[iblk];
	max_subblocks_per_block = subblock_data_ptr->max_subblocks_per_block;
	subblock_lut_size = subblock_data_ptr->subblock_lut_size;
	subblock_ff_size = subblock_data_ptr->subblock_ff_size;

	if (num_subblocks > max_subblocks_per_block) {
		printf("Error:  block #%d (%s) contains %d subblocks.\n", iblk,
				block[iblk].name, num_subblocks);
		error++;
	}

	/* Check that all pins connect to the proper type of CLB pin and are in the *
	 * correct range.                                                           */

	for (isub = 0; isub < num_subblocks; isub++) {

		for (ipin = 0; ipin < subblock_lut_size; ipin++) { /* Input pins */
			clb_pin = subblock_inf[isub].inputs[ipin];
			error += check_subblock_pin(clb_pin, 0,
					pins_per_clb
							+ (1 + subblock_ff_size) * max_subblocks_per_block
							- 1, RECEIVER, iblk, isub, subblock_inf);
		}

		/* Subblock output pin. */

		clb_pin = subblock_inf[isub].output;
		error += check_subblock_pin(clb_pin, 0, pins_per_clb - 1, DRIVER, iblk,
				isub, subblock_inf);

		/* Subblock clock pin. */

		clb_pin = subblock_inf[isub].clock;
		error += check_subblock_pin(clb_pin, 0,
				pins_per_clb + num_subblocks - 1, RECEIVER, iblk, isub,
				subblock_inf);

	} /* End subblock for loop. */

	/* If pins out of range, return.  Could get seg faults otherwise. */

	return (error);
}

static int check_connects(int *num_uses_of_clb_pin, int *num_uses_of_sblk_opin,
		int *num_uses_of_ff_opin, t_subblock_data *subblock_data_ptr, int iblk) {
	/* Reset fanout counts. */
	int i, num_subblocks, num_ffblocks, max_sub, subblock_lut_size,
			subblock_ff_size;
	int error = 0;
	t_subblock *subblock_inf;
	t_ffblock *ffblock_inf;

	subblock_inf = subblock_data_ptr->subblock_inf[iblk];
	ffblock_inf = subblock_data_ptr->ffblock_inf[iblk];
	num_subblocks = subblock_data_ptr->num_subblocks_per_block[iblk];
	max_sub = subblock_data_ptr->max_subblocks_per_block;
	num_ffblocks = subblock_data_ptr->num_ff_per_block[iblk];
	subblock_lut_size = subblock_data_ptr->subblock_lut_size;
	subblock_ff_size = subblock_data_ptr->subblock_ff_size;

	for (i = 0; i < pins_per_clb; i++)
		num_uses_of_clb_pin[i] = 0;

	for (i = 0; i < max_sub; i++)
		num_uses_of_sblk_opin[i] = 0;

	for (i = 0; i < max_sub * subblock_ff_size; i++)
		num_uses_of_ff_opin[i] = 0;

	load_one_clb_fanout_count(subblock_lut_size, subblock_inf, num_subblocks,
			num_uses_of_clb_pin, num_uses_of_sblk_opin, iblk, ffblock_inf,
			num_ffblocks, num_uses_of_ff_opin, max_sub);

	error += check_clb_to_subblock_connections(iblk, subblock_inf, ffblock_inf,
			num_subblocks, num_ffblocks, num_uses_of_clb_pin);

	error += check_internal_subblock_connections(subblock_data_ptr, iblk,
			num_uses_of_sblk_opin, num_uses_of_ff_opin);

	return (error);
}

static int check_subblock_pin(int clb_pin, int min_val, int max_val,
		enum e_pin_type pin_type, int iblk, int isubblk,
		t_subblock *subblock_inf) {

	/* Checks that this subblock pin connects to a valid clb pin or BLE output *
	 * within the clb.  Returns the number of errors found.                    */

	int iclass;

	if (clb_pin != OPEN) {
		if (clb_pin < min_val || clb_pin > max_val) {
			printf("Error:  Block #%d (%s) subblock #%d (%s)"
					"connects to nonexistent clb pin #%d.\n", iblk,
					block[iblk].name, isubblk, subblock_inf[isubblk].name,
					clb_pin);
			return (1);
		}

		if (clb_pin < pins_per_clb) { /* clb pin */
			iclass = clb_pin_class[clb_pin];
			if (class_inf[iclass].type != pin_type) {
				printf("Error:  Block #%d (%s) subblock #%d (%s) pin connects\n"
						"\tto clb pin (#%d) of wrong input/output type.\n",
						iblk, block[iblk].name, isubblk,
						subblock_inf[isubblk].name, clb_pin);
				return (1);
			}
		}
	}

	return (0);
}

static int check_ffblock_pin(int clb_pin, int min_val, int max_val,
		enum e_pin_type pin_type, int iblk, int iff) {

	/* Checks that this subblock pin connects to a valid clb pin or BLE output *
	 * within the clb.  Returns the number of errors found.                    */

	int iclass;

	if (clb_pin != OPEN) {
		if (clb_pin < min_val || clb_pin > max_val) {
			printf("Error:  Block #%d (%s) flip-flop #%d "
					"connects to nonexistent clb pin #%d.\n", iblk,
					block[iblk].name, iff, clb_pin);
			return (1);
		}

		if (clb_pin < pins_per_clb) { /* clb pin */
			iclass = clb_pin_class[clb_pin];
			if (class_inf[iclass].type != pin_type) {
				printf("Error:  Block #%d (%s) subblock #%d pin connects\n"
						"\tto clb pin (#%d) of wrong input/output type.\n",
						iblk, block[iblk].name, iff, clb_pin);
				return (1);
			}
		}
	}

	return (0);
}

static void check_for_multiple_sink_connections(void) {

	/* The check is for nets that connect more than once to the same class of  *
	 * pins on the same block.  For LUTs and cluster-based logic blocks that   *
	 * doesn't make sense, although for some logic blocks it does.  The router *
	 * can now handle this case, so maybe I should get rid of this check.      */

	int iblk, ipin, inet, iclass, class_pin, i;
	int *num_pins_connected;
	/* Have to do the check block by block, rather than net by net, for speed. *
	 * This makes the code a bit messy.                                        */
	num_pins_connected = (int*) my_calloc(num_nets, sizeof(int));

	for (i = 0; i < num_nets; i++)
		num_pins_connected[i] = 0;

	for (iblk = 0; iblk < num_blocks; iblk++) {
		if (block[iblk].type == CLB)
			for (iclass = 0; iclass < num_class; iclass++) {

				/* Two DRIVER pins can never connect to the same net (already checked by    *
				 * the multiple driver check) so skip that check.                           */

				if (class_inf[iclass].type == DRIVER)
					continue;
				for (class_pin = 0; class_pin < class_inf[iclass].num_pins;
						class_pin++) {
					ipin = class_inf[iclass].pinlist[class_pin];
					inet = block[iblk].nets[ipin];

					if (inet != OPEN)
						num_pins_connected[inet]++;
				}

				for (class_pin = 0; class_pin < class_inf[iclass].num_pins;
						class_pin++) {

					ipin = class_inf[iclass].pinlist[class_pin];
					inet = block[iblk].nets[ipin];

					if (inet != OPEN) {
						if (num_pins_connected[inet] > 1) {
							printf(
									"Warning:  block %d (%s) at stage %d connects %d pins of class "
											"%d to net %d (%s).\n", iblk,
									block[iblk].name, block[iblk].stage,
									num_pins_connected[inet], iclass, inet,
									net[inet].name);

							printf(
									"\tThis does not make sense for many logic blocks "
											"(e.g. LUTs).\n"
											"\tBe sure you really want this.\n");
						}

						num_pins_connected[inet] = 0;
					}
				}
			}
	}

	free(num_pins_connected);
}

static int get_num_conn(int bnum) {

	/* This routine returns the number of connections to a block. */

	int i, num_conn;

	num_conn = 0;

	for (i = 0; i < pins_per_clb; i++) {
		if (block[bnum].nets[i] != OPEN)
			num_conn++;
	}

	return (num_conn);
}

static int check_internal_subblock_connections(
		t_subblock_data *subblock_data_ptr, int iblk,
		int *num_uses_of_sblk_opin, int *num_uses_of_ff_opin) {

	/* This routine checks that all subblocks in this block are either           *
	 * completely empty (no pins hooked to anything) or have their output used   *
	 * somewhere.  It also counts the number of constant generators (no input    *
	 * sblks) and the number of FFs used in the circuit.                         */

	int num_const_gen = 0, num_ff = 0, isub, ipin, error, iff;
	boolean has_inputs;
	int subblock_lut_size;
	int num_subblocks, num_ffblocks;
	t_subblock *subblock_inf;
	t_ffblock *ffblock_inf;
	int stage, sub_index;

	subblock_lut_size = subblock_data_ptr->subblock_lut_size;
	num_subblocks = subblock_data_ptr->num_subblocks_per_block[iblk];
	num_ffblocks = subblock_data_ptr->num_ff_per_block[iblk];
	subblock_inf = subblock_data_ptr->subblock_inf[iblk];
	ffblock_inf = subblock_data_ptr->ffblock_inf[iblk];

	stage = block[iblk].stage;

	error = 0;

	for (isub = 0; isub < num_subblocks; isub++) {

		has_inputs = FALSE;
		for (ipin = 0; ipin < subblock_lut_size; ipin++) {
			if (subblock_inf[isub].inputs[ipin] != OPEN) {
				has_inputs = TRUE;
				break;
			}
		}

		sub_index = subblock_inf[isub].index;
		//printf("index %d\n", sub_index);
		if (num_uses_of_sblk_opin[sub_index] == 0) { /* Output unused */

			if (has_inputs || subblock_inf[isub].clock != OPEN) {
				printf(
						"Error:  output of subblock #%d (%s) of block #%d (%s) at stage %d is "
								"never used.\n", sub_index,
						subblock_inf[isub].name, iblk, block[iblk].name,
						block[iblk].stage);
				error++;
			}
		} /* End if output unused */

		/* Check that subblocks whose output is used have inputs. */

		else { /* Subblock output is used. */

			if (!has_inputs) { /* No inputs are used */
				if (subblock_inf[isub].clock == OPEN) {
					printf("Warning:  block #%d (%s), subblock #%d (%s) is a "
							"constant generator.\n\t(Has no inputs.)\n", iblk,
							block[iblk].name, sub_index,
							subblock_inf[isub].name);
					num_const_gen++;
				} else {
					printf(
							"Error:  block #%d (%s), subblock #%d (%s) is a CLOCKED "
									"\n\tconstant generator.\n\t(Has no inputs but is clocked.)\n",
							iblk, block[iblk].name, sub_index,
							subblock_inf[isub].name);
					num_const_gen++;
					error++;
				}
			}

			else { /* Both input and output are used */
				if (subblock_inf[isub].clock != OPEN)
					num_ff++;
			}
		}
	} /* End for all subblocks */

	/* for each ffblock */
	for (iff = 0; iff < num_ffblocks; iff++) {
		if (ffblock_inf[iff].clock == OPEN) {
			printf("Error: ffblock #%d of block #%d (%s) is "
					"never used.\n", ffblock_inf[iff].index, iblk,
					block[iblk].name);
			error++;
		} else if (ffblock_inf[iff].input != OPEN)
			num_ff++;
	}

	subblock_data_ptr->num_const_gen += num_const_gen;
	subblock_data_ptr->num_ff += num_ff;

	return (error);
}

static int check_clb_to_subblock_connections(int iblk, t_subblock *subblock_inf,
		t_ffblock *ffblock_inf, int num_subblocks, int num_ffblocks,
		int *num_uses_of_clb_pin) {

	/* This routine checks that each non-OPEN clb input pin connects to some    *
	 * subblock inputs, and that each non-OPEN clb output pin is driven by      *
	 * exactly one subblock output. It returns the number of errors found.      *
	 * Note that num_uses_of_clb_pin is used to store the number of out-edges   *
	 * (fanout) for a CLB ipin, and the number of in-edges (fanin) for a CLB    *
	 * opin.                                                                    */

	int ipin, isub, clb_pin, error, iff;

	error = 0;

	/* Count how many things connect to each clb output pin. */

	for (isub = 0; isub < num_subblocks; isub++) {
		clb_pin = subblock_inf[isub].output;

		if (clb_pin != OPEN) /* Guaranteed to connect to DRIVER pin only */
			num_uses_of_clb_pin[clb_pin]++;
	}
	/* Added by Wei */
	for (iff = 0; iff < num_ffblocks; iff++) {
		clb_pin = ffblock_inf[iff].output;

		if (clb_pin != OPEN) /* Guaranteed to connect to DRIVER pin only */
			num_uses_of_clb_pin[clb_pin]++;
	}

	for (ipin = 0; ipin < pins_per_clb; ipin++) {

		if (block[iblk].nets[ipin] != OPEN) {
			if (is_opin(ipin)) { /* CLB output */
				if (num_uses_of_clb_pin[ipin] == 0) { /* No driver? */
					printf(
							"Error:  output pin %d on block #%d (%s) on stage %d is not driven "
									"by any subblock or ffs.\n", ipin, iblk,
							block[iblk].name, block[iblk].stage);
					error++;
				} else if (num_uses_of_clb_pin[ipin] > 1) { /* Multiple drivers? */
					printf(
							"Error:  output pin %d (%s) on block #%d (%s) on stage %d is driven "
									"by %d subblocks or ffs.\n", ipin,
							net[block[iblk].nets[ipin]].name, iblk,
							block[iblk].name, block[iblk].stage,
							num_uses_of_clb_pin[ipin]);
					error++;
				}
			}

			else { /* CLB ipin */
				if (num_uses_of_clb_pin[ipin] <= 0) { /* Fans out? */
					printf(
							"Error:  pin %d on block #%d (%s) at stage %d does not fanout to any "
									"subblocks or ffs.\n", ipin, iblk,
							block[iblk].name, block[iblk].stage);
					error++;
				}
			}
		} /* End if not OPEN */

		else if (is_opin(ipin)) { /* OPEN CLB output pin */
			if (num_uses_of_clb_pin[ipin] > 1) {
				printf(
						"Error:  pin %d(%s) on block #%d (%s) on stage %d is driven by %d "
								"subblocks or ffs.\n", ipin,
						net[block[iblk].nets[ipin]].name, iblk,
						block[iblk].name, block[iblk].stage,
						num_uses_of_clb_pin[ipin]);
				error++;
			}
		}
	}

	return (error);
}
