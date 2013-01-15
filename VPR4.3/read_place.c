#include <stdio.h> 
#include <string.h> 
#include "util.h" 
#include "vpr_types.h"
#include "globals.h"
#include "hash.h"
#include "read_place.h"

#define EMPTY -1

static int get_subblock(int i, int j, int bnum);
static void read_place_header(FILE *fp, char *net_file, char *arch_file,
		char *buf);

void read_user_pad_loc(char *pad_loc_file) {

	/* Reads in the locations of the IO pads from a file. */

	struct s_hash **hash_table, *h_ptr;
	int iblk, i, j, xtmp, ytmp, bnum, isubblk;
	FILE *fp;
	char buf[BUFSIZE], bname[BUFSIZE], *ptr;
	int istage, istage1;

	printf("\nReading locations of IO pads from %s.\n", pad_loc_file);
	linenum = 0;
	fp = my_fopen(pad_loc_file, "r", 0);

	hash_table = alloc_hash_table();
	for (iblk = 0; iblk < num_blocks; iblk++) {
		if (block[iblk].type == INPAD || block[iblk].type == OUTPAD) {
			if (!is_folding)
				h_ptr = insert_in_hash_table(hash_table, block[iblk].name,
						iblk);
			else
				h_ptr = insert_in_hash_table_new(hash_table, block[iblk].name,
						iblk, block[iblk].stage);
			block[iblk].x = OPEN; /* Mark as not seen yet. */
		}
	}
	if (!is_folding) {
		for (i = 0; i <= nx + 1; i++) {
			for (j = 0; j <= ny + 1; j++) {
				if (clb[i][j].type == IO) {
					for (isubblk = 0; isubblk < io_rat; isubblk++)
						clb[i][j].u.io_blocks[isubblk] = OPEN; /* Flag for err. check */
				}
			}
		}
	} else {
		for (istage = 0; istage < num_stage; istage++) {
			for (i = 0; i <= nx + 1; i++) {
				for (j = 0; j <= ny + 1; j++) {
					if (stage_clb[istage][i][j].type == IO) {
						for (isubblk = 0; isubblk < io_rat; isubblk++)
							stage_clb[istage][i][j].u.io_blocks[isubblk] = EMPTY; /* Flag for err. check */
					}
				}
			}
		}
	}

	ptr = my_fgets(buf, BUFSIZE, fp);

	while (ptr != NULL) {
		ptr = my_strtok(buf, TOKENS, fp, buf);
		if (ptr == NULL) {
			ptr = my_fgets(buf, BUFSIZE, fp);
			continue; /* Skip blank or comment lines. */
		}

		strcpy(bname, ptr);

		ptr = my_strtok(NULL, TOKENS, fp, buf);
		if (ptr == NULL) {
			printf("Error:  line %d is incomplete.\n", linenum);
			exit(1);
		}
		sscanf(ptr, "%d", &xtmp);

		ptr = my_strtok(NULL, TOKENS, fp, buf);
		if (ptr == NULL) {
			printf("Error:  line %d is incomplete.\n", linenum);
			exit(1);
		}
		sscanf(ptr, "%d", &ytmp);

		ptr = my_strtok(NULL, TOKENS, fp, buf);
		if (ptr == NULL) {
			printf("Error:  line %d is incomplete.\n", linenum);
			exit(1);
		}
		sscanf(ptr, "%d", &isubblk);

		ptr = my_strtok(NULL, TOKENS, fp, buf);
		if (ptr != NULL) {
			printf("Error:  extra characters at end of line %d.\n", linenum);
			exit(1);
		}
		if (!is_folding) {
			h_ptr = get_hash_entry(hash_table, bname);
			if (h_ptr == NULL) {
				printf("Error:  block %s on line %d: no such IO pad.\n", bname,
						linenum);
				exit(1);
			}
			bnum = h_ptr->index;
			i = xtmp;
			j = ytmp;

			if (block[bnum].x != OPEN) {
				printf("Error:  line %d.  Block %s listed twice in pad file.\n",
						linenum, bname);
				exit(1);
			}

			if (i < 0 || i > nx + 1 || j < 0 || j > ny + 1) {
				printf("Error:  block #%d (%s) location\n", bnum, bname);
				printf("(%d,%d) is out of range.\n", i, j);
				exit(1);
			}

			block[bnum].x = i; /* Will be reloaded by initial_placement anyway. */
			block[bnum].y = j; /* I need to set .x only as a done flag.         */

			if (clb[i][j].type != IO) {
				printf("Error:  attempt to place IO block %s in \n", bname);
				printf("an illegal location (%d, %d).\n", i, j);
				exit(1);
			}

			if (isubblk >= io_rat || isubblk < 0) {
				printf(
						"Error:  Block %s subblock number (%d) on line %d is out of "
								"range.\n", bname, isubblk, linenum);
				exit(1);
			}
			clb[i][j].u.io_blocks[isubblk] = bnum;
			clb[i][j].occ++;
		} else {
			i = xtmp;
			j = ytmp;
			for (istage1 = 0; istage1 < num_stage; istage1++)
				stage_clb[istage1][i][j].occ++;

			for (istage = 1; istage <= num_stage; istage++) {
				h_ptr = get_hash_entry_new(hash_table, bname, istage);
				if (h_ptr != NULL) {
					bnum = h_ptr->index;

					if (block[bnum].x != OPEN) {
						printf(
								"Error:  line %d.  Block %s listed twice in pad file.\n",
								linenum, bname);
						exit(1);
					}

					if (i < 0 || i > nx + 1 || j < 0 || j > ny + 1) {
						printf("Error:  block #%d (%s) location\n", bnum,
								bname);
						printf("(%d,%d) is out of range.\n", i, j);
						exit(1);
					}

					block[bnum].x = i; /* Will be reloaded by initial_placement anyway. */
					block[bnum].y = j; /* I need to set .x only as a done flag.         */

					if (clb[i][j].type != IO) {
						printf("Error:  attempt to place IO block %s in \n",
								bname);
						printf("an illegal location (%d, %d).\n", i, j);
						exit(1);
					}

					if (isubblk >= io_rat || isubblk < 0) {
						printf(
								"Error:  Block %s subblock number (%d) on line %d is out of "
										"range.\n", bname, isubblk, linenum);
						exit(1);
					}
					stage_clb[istage][i][j].u.io_blocks[isubblk] = bnum;
				}
			}
		}
		ptr = my_fgets(buf, BUFSIZE, fp);
	}

	for (iblk = 0; iblk < num_blocks; iblk++) {
		if ((block[iblk].type == INPAD || block[iblk].type == OUTPAD)
				&& block[iblk].x == OPEN) {
			printf("Error:  IO block %s location was not specified in "
					"the pad file.\n", block[iblk].name);
			exit(1);
		}
	}

	if (!is_folding) {
		for (i = 0; i <= nx + 1; i++) {
			for (j = 0; j <= ny + 1; j++) {
				if (clb[i][j].type == IO) {
					for (isubblk = 0; isubblk < clb[i][j].occ; isubblk++) {
						if (clb[i][j].u.io_blocks[isubblk] == OPEN) {
							printf(
									"Error:  The IO blocks at (%d, %d) do not have \n"
											"consecutive subblock numbers starting at 0.\n",
									i, j);
							exit(1);
						}
					}
				}
			}
		}
	} else {
		for (istage = 0; istage < num_stage; istage++) {
			for (i = 0; i <= nx + 1; i++) {
				for (j = 0; j <= ny + 1; j++) {
					if (stage_clb[istage][i][j].type == IO) {
						for (isubblk = 0; isubblk < stage_clb[istage][i][j].occ;
								isubblk++) {
							if (stage_clb[istage][i][j].u.io_blocks[isubblk]
									== OPEN) {
								printf(
										"Error:  The IO blocks at (%d, %d) do not have \n"
												"consecutive subblock numbers starting at 0.\n",
										i, j);
								exit(1);
							}
						}
					}
				}
			}
		}
	}

	fclose(fp);
	free_hash_table(hash_table);
	printf("Successfully read %s.\n\n", pad_loc_file);
}

void dump_clbs(void) {

	/* Output routine for debugging. */

	int i, j, index;

	for (i = 0; i <= nx + 1; i++) {
		for (j = 0; j <= ny + 1; j++) {
			printf("clb (%d,%d):  type: %d  occ: %d\n", i, j, clb[i][j].type,
					clb[i][j].occ);
			if (clb[i][j].type == CLB || clb[i][j].type == MEM)
				printf("block: %d\n", clb[i][j].u.block);
			if (clb[i][j].type == IO) {
				printf("io_blocks: ");
				for (index = 0; index < clb[i][j].occ; index++)
					printf("%d  ", clb[i][j].u.io_blocks[index]);
				printf("\n");
			}
		}
	}

	for (i = 0; i < num_blocks; i++) {
		printf("block: %d, (i,j): (%d, %d)\n", i, block[i].x, block[i].y);
	}
}

/* Added by Wei */

void dump_stage_clbs(void) {

	/* Output routine for debugging. */

	int i, j, index, istage;

	for (istage = 0; istage < num_stage; istage++) {
		printf("In stage: %d\n", istage + 1);
		for (i = 0; i <= nx + 1; i++) {
			for (j = 0; j <= ny + 1; j++) {
				printf("clb (%d,%d):  type: %d  occ: %d\n", i, j,
						stage_clb[istage][i][j].type,
						stage_clb[istage][i][j].occ);
				if (clb[i][j].type == CLB || clb[i][j].type == MEM)
					printf("block: %d\n", stage_clb[istage][i][j].u.block);
				if (clb[i][j].type == IO) {
					printf("io_blocks: ");
					for (index = 0; index < stage_clb[istage][i][j].occ;
							index++)
						printf("%d  ",
								stage_clb[istage][i][j].u.io_blocks[index]);
					printf("\n");
				}
			}
		}
	}

	for (i = 0; i < num_blocks; i++) {
		printf("block: %d, (i,j): (%d, %d)\n", i, block[i].x, block[i].y);
	}
}

void print_place(char *place_file, char *net_file, char *arch_file) {

	/* Prints out the placement of the circuit.  The architecture and    *
	 * netlist files used to generate this placement are recorded in the *
	 * file to avoid loading a placement with the wrong support files    *
	 * later.                                                            */

	FILE *fp;
	int i, subblock, istage = 0;

	fp = my_fopen(place_file, "w", 0);

	fprintf(fp, "Netlist file: %s   Architecture file: %s\n", net_file,
			arch_file);
	fprintf(fp, "Array size: %d x %d logic blocks\n\n", nx, ny);
	fprintf(fp, "#block name\tx\ty\tsubblk\tblock number\n");
	fprintf(fp, "#----------\t--\t--\t------\t------------\n");

	for (i = 0; i < num_blocks; i++) {

		if (is_folding) {
			if (block[i].stage != istage) {
				istage = block[i].stage;
				fprintf(fp, "stage: %d\n", istage);
			}
		}

		fprintf(fp, "%s\t", block[i].name);
		if (strlen(block[i].name) < 8)
			fprintf(fp, "\t");

		fprintf(fp, "%d\t%d", block[i].x, block[i].y);
		if (block[i].type == CLB || block[i].type == MEM || block[i].type == DSP) {
			fprintf(fp, "\t%d", 0); /* Sub block number not meaningful. */
		} else { /* IO block.  Save sub block number. */
			subblock = get_subblock(block[i].x, block[i].y, i);
			fprintf(fp, "\t%d", subblock);
		}
		fprintf(fp, "\t#%d\n", i);
	}

	fclose(fp);
}

static int get_subblock(int i, int j, int bnum) {

	/* Use this routine only for IO blocks.  It passes back the index of the *
	 * subblock containing block bnum at location (i,j).                     */

	int k;
	int stage;

	if (!is_folding) {
		for (k = 0; k < io_rat; k++) {
			if (clb[i][j].u.io_blocks[k] == bnum)
				return (k);
		}
	} else {
		stage = block[bnum].stage;
		for (k = 0; k < io_rat; k++) {
			if (stage_clb[stage - 1][i][j].u.io_blocks[k] == bnum)
				return (k);
		}
	}

	printf("Error in get_subblock.  Block %d is not at location (%d,%d)\n",
			bnum, i, j);
	exit(1);
}

void parse_placement_file(char *place_file, char *net_file, char *arch_file) {

	/* Reads the blocks in from a previously placed circuit.             */

	FILE *fp;
	char bname[BUFSIZE];
	char buf[BUFSIZE], *ptr;
	struct s_hash **hash_table, *h_ptr;
	int i, j, bnum, isubblk, xtmp, ytmp, isubblock;
	int stage_num, istage;
	int max;

	printf("Reading the placement from file %s.\n", place_file);
	fp = my_fopen(place_file, "r", 0);
	linenum = 0;

	read_place_header(fp, net_file, arch_file, buf);

	for (i = 0; i <= nx + 1; i++) {
		for (j = 0; j <= ny + 1; j++) {
			clb[i][j].occ = 0;
			if (clb[i][j].type == IO) {
				for (isubblk = 0; isubblk < io_rat; isubblk++) {
					clb[i][j].u.io_blocks[isubblk] = OPEN;
				}
			}
		}
	}
	printf("array initialize \n");
	if (is_folding) {
		for (istage = 0; istage < num_stage; istage++) {
			for (i = 0; i <= nx + 1; i++) {
				for (j = 0; j <= ny + 1; j++) {
					stage_clb[istage][i][j].occ = 0;
					if (stage_clb[istage][i][j].type == IO) {
						for (isubblk = 0; isubblk < io_rat; isubblk++)
							stage_clb[istage][i][j].u.io_blocks[isubblk] = EMPTY; /* Flag for err. check */
					}
				}
			}
		}
	}

	for (i = 0; i < num_blocks; i++)
		block[i].x = OPEN; /* Flag to show not read yet. */

	hash_table = alloc_hash_table();
	for (i = 0; i < num_blocks; i++) {
		if (!is_folding)
			h_ptr = insert_in_hash_table(hash_table, block[i].name, i);
		else {
			//printf("insert block %s %d in stage %d\n", block[i].name, i, block[i].stage);
			h_ptr = insert_in_hash_table_new(hash_table, block[i].name, i,
					block[i].stage);
		}
	}

	ptr = my_fgets(buf, BUFSIZE, fp);

	while (ptr != NULL) {
		ptr = my_strtok(buf, TOKENS, fp, buf);
		if (ptr == NULL) {
			ptr = my_fgets(buf, BUFSIZE, fp);
			continue; /* Skip blank or comment lines. */
		}

		if (strcmp("stage:", ptr) == 0) {
			ptr = my_strtok(NULL, TOKENS, fp, buf);
			if (ptr == NULL) {
				printf("Error: line %d is incomplete.\n", linenum);
				exit(1);
			}
			stage_num = atoi(ptr);
			ptr = my_fgets(buf, BUFSIZE, fp);
			continue;
		}

		strcpy(bname, ptr);

		ptr = my_strtok(NULL, TOKENS, fp, buf);
		if (ptr == NULL) {
			printf("Error:  line %d is incomplete.\n", linenum);
			exit(1);
		}
		sscanf(ptr, "%d", &xtmp);

		ptr = my_strtok(NULL, TOKENS, fp, buf);
		if (ptr == NULL) {
			printf("Error:  line %d is incomplete.\n", linenum);
			exit(1);
		}
		sscanf(ptr, "%d", &ytmp);

		ptr = my_strtok(NULL, TOKENS, fp, buf);
		if (ptr == NULL) {
			printf("Error:  line %d is incomplete.\n", linenum);
			exit(1);
		}
		sscanf(ptr, "%d", &isubblock);
		ptr = my_strtok(NULL, TOKENS, fp, buf);
		if (ptr != NULL) {
			printf("Error:  extra characters at end of line %d.\n", linenum);
			exit(1);
		}
		if (!is_folding)
			h_ptr = get_hash_entry(hash_table, bname);
		else {
			h_ptr = get_hash_entry_new(hash_table, bname, stage_num);
			//printf("block %s in stage %d \n", bname, stage_num);
		}

		if (h_ptr == NULL) {
			printf(
					"Error:  block %s on line %d does not exist in the netlist.\n",
					bname, linenum);
			exit(1);
		}
		bnum = h_ptr->index;
		i = xtmp;
		j = ytmp;

		if (block[bnum].x != OPEN) {
			printf(
					"Error:  line %d.  Block %s listed twice in placement file.\n",
					linenum, bname);
			exit(1);
		}

		if (i < 0 || i > nx + 1 || j < 0 || j > ny + 1) {
			printf("Error in read_place.  Block #%d (%s) location\n", bnum,
					bname);
			printf("(%d,%d) is out of range.\n", i, j);
			exit(1);
		}

		block[bnum].x = i;
		block[bnum].y = j;

		if (clb[i][j].type == CLB) {
			if (block[bnum].type != CLB) {
				printf(
						"Error in read_place.  Attempt to place block #%d (%s) in\n",
						bnum, bname);
				printf("a logic block location (%d, %d).\n", i, j);
				exit(1);
			}
			if (!is_folding) {
				clb[i][j].u.block = bnum;
				clb[i][j].occ++;
			} else {
				stage_clb[stage_num - 1][i][j].u.block = bnum;
				stage_clb[stage_num - 1][i][j].occ++;
			}
		} else if (clb[i][j].type == MEM) {
			if (block[bnum].type != MEM) {
				printf(
						"Error in read_place.  Attempt to place block #%d (%s) in\n",
						bnum, bname);
				printf("a memory block position (%d, %d).\n", i, j);
				exit(1);
			}
			if (!is_folding) {
				clb[i][j].u.block = bnum;
				clb[i][j].occ++;
			} else {
				stage_clb[stage_num - 1][i][j].u.block = bnum;
				stage_clb[stage_num - 1][i][j].occ++;
			}
		} else if (clb[i][j].type == IO) {
			if (block[bnum].type != INPAD && block[bnum].type != OUTPAD) {
				printf(
						"Error in read_place.  Attempt to place block #%d (%s) in\n",
						bnum, bname);
				printf("an IO block location (%d, %d).\n", i, j);
				exit(1);
			}
			if (isubblock >= io_rat || isubblock < 0) {
				printf(
						"Error:  Block %s subblock number (%d) on line %d is out of "
								"range.\n", bname, isubblock, linenum);
				exit(1);
			}
			if (!is_folding) {
				clb[i][j].u.io_blocks[isubblock] = bnum;
				clb[i][j].occ++;
			} else {
				printf("set bnum %d in stage %d at (%d, %d)\n", bnum, stage_num,
						i, j);
				stage_clb[stage_num - 1][i][j].u.io_blocks[isubblock] = bnum;
				stage_clb[stage_num - 1][i][j].occ =
						max(isubblock, stage_clb[stage_num-1][i][j].occ)
								+ 1;
			}
		}

		else { /* Block type was ILLEGAL or some unknown value */
			printf("Error in read_place.  Block #%d (%s) is in an illegal ",
					bnum, bname);
			printf("location.\nLocation specified: (%d,%d).\n", i, j);
			exit(1);
		}

		ptr = my_fgets(buf, BUFSIZE, fp);
	}

	free_hash_table(hash_table);
	fclose(fp);

	for (i = 0; i < num_blocks; i++) {
		if (block[i].x == OPEN) {
			printf(
					"Error in read_place:  block %s location was not specified in "
							"the placement file.\n", block[i].name);
			exit(1);
		}
	}

	for (i = 0; i <= nx + 1; i++) {
		for (j = 0; j <= ny + 1; j++) {
			if (clb[i][j].type == IO) {
				if (!is_folding) {
					for (isubblk = 0; isubblk < clb[i][j].occ; isubblk++) {
						if (clb[i][j].u.io_blocks[isubblk] == OPEN) {
							printf(
									"Error:  The IO blocks at (%d, %d) do not have \n"
											"consecutive subblock numbers starting at 0.\n",
									i, j);
							exit(1);
						}
					}
				}
				/*else {
				 for (istage=0; istage<num_stage; istage++){
				 for (isubblk=0;isubblk<stage_clb[istage][i][j].occ;isubblk++) {
				 if (stage_clb[istage][i][j].u.io_blocks[isubblk] == EMPTY) {
				 printf ("Error:  The IO blocks at (%d, %d) do not have \n"
				 "consecutive subblock numbers starting at 0.\n", i, j);
				 //exit (1);
				 }
				 }
				 }
				 }*/
			}
		}
	}
	/* Added by Wei */
	max = 0;
	for (i = 0; i <= nx + 1; i++) {
		for (j = 0; j <= ny + 1; j++) {
			if (clb[i][j].type == IO) {
				for (istage = 0; istage < num_stage; istage++) {
					if (stage_clb[istage][i][j].occ > max)
						max = stage_clb[istage][i][j].occ;
				}
				for (istage = 0; istage < num_stage; istage++)
					stage_clb[istage][i][j].occ = max;
			}
		}
	}

	printf("Successfully read %s.\n", place_file);
}

static void read_place_header(FILE *fp, char *net_file, char *arch_file,
		char *buf) {

	/* Reads the header from the placement file.  Used only to check that this *
	 * placement file matches the current architecture, netlist, etc.          */

	char *line_one_names[] =
			{ "Netlist", "file:", " ", "Architecture", "file:" };
	char *line_two_names[] = { "Array", "size:", " ", "x", " ", "logic",
			"blocks" };
	char net_check[BUFSIZE], arch_check[BUFSIZE], *ptr;
	int nx_check, ny_check, i;

	ptr = my_fgets(buf, BUFSIZE, fp);

	if (ptr == NULL) {
		printf("Error:  netlist file and architecture file used not listed.\n");
		exit(1);
	}

	ptr = my_strtok(buf, TOKENS, fp, buf);
	while (ptr == NULL) { /* Skip blank or comment lines. */
		ptr = my_fgets(buf, BUFSIZE, fp);
		if (ptr == NULL) {
			printf(
					"Error:  netlist file and architecture file used not listed.\n");
			exit(1);
		}
		ptr = my_strtok(buf, TOKENS, fp, buf);
	}

	for (i = 0; i <= 5; i++) {
		if (i == 2) {
			strcpy(net_check, ptr);
		} else if (i == 5) {
			strcpy(arch_check, ptr);
		} else {
			if (strcmp(ptr, line_one_names[i]) != 0) {
				printf("Error on line %d, word %d:  \n"
						"Expected keyword %s, got %s.\n", linenum, i + 1,
						line_one_names[i], ptr);
				exit(1);
			}
		}
		ptr = my_strtok(NULL, TOKENS, fp, buf);
		if (ptr == NULL && i != 5) {
			printf("Error:  Unexpected end of line on line %d.\n", linenum);
			exit(1);
		}
	}

	if (strcmp(net_check, net_file) != 0) {
		printf("Warning:  Placement generated with netlist file %s:\n",
				net_check);
		printf("current net file is %s.\n", net_file);
	}

	if (strcmp(arch_check, arch_file) != 0) {
		printf("Warning:  Placement generated with architecture file %s:\n",
				arch_check);
		printf("current architecture file is %s.\n", arch_file);
	}

	/* Now check the second line (array size). */

	ptr = my_fgets(buf, BUFSIZE, fp);

	if (ptr == NULL) {
		printf("Error:  Array size not listed.\n");
		exit(1);
	}

	ptr = my_strtok(buf, TOKENS, fp, buf);
	while (ptr == NULL) { /* Skip blank or comment lines. */
		ptr = my_fgets(buf, BUFSIZE, fp);
		if (ptr == NULL) {
			printf("Error:  array size not listed.\n");
			exit(1);
		}
		ptr = my_strtok(buf, TOKENS, fp, buf);
	}

	for (i = 0; i <= 6; i++) {
		if (i == 2) {
			sscanf(ptr, "%d", &nx_check);
		} else if (i == 4) {
			sscanf(ptr, "%d", &ny_check);
		} else {
			if (strcmp(ptr, line_two_names[i]) != 0) {
				printf("Error on line %d, word %d:  \n"
						"Expected keyword %s, got %s.\n", linenum, i + 1,
						line_two_names[i], ptr);
				exit(1);
			}
		}
		ptr = my_strtok(NULL, TOKENS, fp, buf);
		if (ptr == NULL && i != 6) {
			printf("Error:  Unexpected end of line on line %d.\n", linenum);
			exit(1);
		}
	}

	if (nx_check != nx || ny_check != ny) {
		printf("Error:  placement file assumes an array size of %d x %d.\n",
				nx_check, ny_check);
		printf("Current size is %d x %d.\n", nx, ny);
		exit(1);
	}
}
