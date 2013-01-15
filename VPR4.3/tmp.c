static int try_swap (float t, float *cost, float *bb_cost, float *timing_cost, 
		     float rlim, int *pins_on_block, 
		     int place_cost_type, float **old_region_occ_x, 
		     float **old_region_occ_y, int num_regions, boolean fixed_pins,
		     enum e_place_algorithm place_algorithm, float timing_tradeoff,
		     float inverse_prev_bb_cost, float inverse_prev_timing_cost,
		     float *delay_cost) {

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
 
 struct s_dist {int stage; int dir; int done;} *dist_block;
 char *bname;
 struct s_hash *h_ptr;
 int begin, bcount, i, j, k, l, nblk, count, sum, istage, cstage, length, currentb;
 boolean add, processed, if_from, if_to, if_out;
 int *bflist, *btlist;
 int *test;

 //nionio added
 boolean pad_swap_success;
 int pad_count;
 
 if(is_folding) check_placement();

 bflist = my_malloc (num_blocks *sizeof(int));
 dist_block = (struct s_dist*)my_malloc (num_blocks*sizeof(struct s_dist));
 btlist = my_malloc (num_blocks*sizeof(int));
 test = my_malloc (num_blocks*sizeof(int));


/* Allocate the local bb_coordinate storage, etc. only once. */

 if (bb_coord_new == NULL) {
    bb_coord_new = (struct s_bb *) my_malloc (num_nets * 
          sizeof (struct s_bb));
    bb_edge_new = (struct s_bb *) my_malloc (num_nets *
          sizeof (struct s_bb));
    nets_to_update = (int *) my_malloc (num_nets * sizeof (int));
    net_block_moved = (int *) my_malloc (num_nets * sizeof (int));
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

 find_to (x_from, y_from, block[b_from].type, rlim, &x_to, &y_to);
 if ((x_from == x_to) && (y_from == y_to))
   return(0);

 //printf(" to x %d y %d \n", x_to, y_to);

/* Make the switch in order to make computing the new bounding *
 * box simpler.  If the cost increase is too high, switch them *
 * back.  (block data structures switched, clbs not switched   *
 * until success of move is determined.)                       */

 if(!is_folding){
   if (block[b_from].type == CLB || block[b_from].type == MEM) {
     io_num = -1;            /* Don't need, but stops compiler warning. */
     if (clb[x_to][y_to].occ == 1) {         /* Occupied -- do a switch */
       b_to = clb[x_to][y_to].u.block;
       block[b_from].x = x_to;
       block[b_from].y = y_to;
       block[b_to].x = x_from;
       block[b_to].y = y_from; 
       // printf("to x %d y %d for blk %s\n", x_to, y_to, block[b_to].name);
     }    
     else {
       b_to = EMPTY;
       block[b_from].x = x_to;
       block[b_from].y = y_to; 
       //printf("to x %d y %d for empty blk\n", x_to, y_to);
     }
   }
   else {   /* io block was selected for moving */
     io_num = my_irand(io_rat - 1);
     if (io_num >= clb[x_to][y_to].occ) {  /* Moving to an empty location */
       b_to = EMPTY;
       block[b_from].x = x_to;
       block[b_from].y = y_to;
       //printf("to x %d y %d for empty blk\n", x_to, y_to);
     }
     else {          /* Swapping two blocks */
       b_to = *(clb[x_to][y_to].u.io_blocks+io_num);
       block[b_to].x = x_from;
       block[b_to].y = y_from;
       block[b_from].x = x_to;
       block[b_from].y = y_to;
       //printf("to x %d y %d for blk %s\n", x_to, y_to, block[b_to].name);
     }
   }
   bflist[0]=b_from;
   btlist[0]=b_to;
   length = 1;
 }
 else {
   if (block[b_from].type == CLB) {
     count =0;
     io_num = -1;
     bname = block[b_from].name;
     h_ptr = get_hash_entry(map_table, bname);
     begin = h_ptr->index;
     bcount = h_ptr->count;
     for (k=begin; k<begin+bcount; k++) {
       nblk = blockmap_inf[k];
       cstage = block[nblk].stage;
       dist_block[count].stage = cstage;
       dist_block[count].dir = 1;
       dist_block[count].done = -1;
       count++;
     }
     for (i=0; i<count; i++) {
       if(dist_block[i].done = -1) {
         cstage = dist_block[i].stage;
	       currentb=-1;
	       if (dist_block[i].dir == 1) {
	         if (stage_clb[cstage-1][x_to][y_to].occ == 1) {
             currentb = stage_clb[cstage-1][x_to][y_to].u.block;
	         }
         }
	       if (dist_block[i].dir == 2) {
	         if (stage_clb[cstage-1][x_from][y_from].occ == 1) {
             currentb = stage_clb[cstage-1][x_from][y_from].u.block;
	         }
         }
    	   if (currentb!=-1) {
           bname = block[currentb].name;
	         h_ptr = get_hash_entry(map_table, bname);
	         begin = h_ptr->index;
	         bcount = h_ptr->count;
	         for (k=begin; k<begin+bcount; k++) {
             nblk = blockmap_inf[k];
	           add = TRUE;
	           for (j=0; j<count; j++) {
	             if(dist_block[j].stage==block[nblk].stage) {
		             add = FALSE;
               }
	           }
	           if(add) {
	             dist_block[count].stage=block[nblk].stage;
	             dist_block[count].done = -1;
	             if(dist_block[i].dir ==1)
		             dist_block[count].dir =2;
	             if(dist_block[i].dir ==2) 
		             dist_block[count].dir =1;
	             count++;
	           }
	         }
	       }
	       dist_block[i].done = 1;
	     }
     }
     for (i=0; i<count; i++) {
       istage = dist_block[i].stage;
       if ((stage_clb[istage-1][x_from][y_from].occ == 1)&&(stage_clb[istage-1][x_to][y_to].occ == 1)) {
	       b_from = stage_clb[istage-1][x_from][y_from].u.block;
	       b_to = stage_clb[istage-1][x_to][y_to].u.block;
	       block[b_from].x = x_to;
	       block[b_from].y = y_to;
	       block[b_to].x = x_from;
	       block[b_to].y = y_from; 
	     }
       else if (stage_clb[istage-1][x_from][y_from].occ == 1) {
	       b_from = stage_clb[istage-1][x_from][y_from].u.block;
	       b_to = EMPTY;
	       block[b_from].x = x_to;
	       block[b_from].y = y_to; 
	     }
       else if (stage_clb[istage-1][x_to][y_to].occ == 1) {
	       b_from = EMPTY;
	       b_to = stage_clb[istage-1][x_to][y_to].u.block;
	       block[b_to].x = x_from;
	       block[b_to].y = y_from;
	     }
       else continue;
       bflist[i]=b_from;
       btlist[i]=b_to;
     }
     length = count;
   }
   else if (block[b_from].type == MEM) {
     sum =0;
     for (i=0; i<num_stage; i++) {  
       if (stage_clb[i][x_to][y_to].occ ==1 && stage_clb[i][x_from][y_from].occ==1) {
         b_to = stage_clb[i][x_to][y_to].u.block;
	       b_from = stage_clb[i][x_from][y_from].u.block;
	       block[b_from].x = x_to;
	       block[b_from].y = y_to;
	       block[b_to].x = x_from;
	       block[b_to].y = y_from;
	     }
       else if (stage_clb[i][x_to][y_to].occ==1) {
         b_to = stage_clb[i][x_to][y_to].u.block;
	       b_from = EMPTY;
	       block[b_to].x = x_from;
	       block[b_to].y = y_from;
	     }
       else if (stage_clb[i][x_from][y_from].occ==1) {
         b_to = EMPTY;
	       b_from = stage_clb[i][x_from][y_from].u.block;
	       block[b_from].x = x_to;
	       block[b_from].y = y_to;
	     }
       else continue;
       bflist[sum]=b_from;
       btlist[sum]=b_to;
       sum++;
     }
   length = sum;
   }

   else {   
     /* io block was selected for moving */
     //printf("swap pad from %d \n", b_from);
     io_num = my_irand(io_rat - 1);
     //printf("to pad slot %d\n", io_num);
     sum =0;
     if_out = FALSE;
     for (off_from=0; off_from<stage_clb[0][x_from][y_from].occ; off_from++) {
       for (istage=0; istage<num_stage; istage++) {
	       if (stage_clb[istage][x_from][y_from].u.io_blocks[off_from] == b_from) {
	         if_out = TRUE;
	         break;
         }
       }
       if(if_out) break;
     }
     if(!if_out) {
       printf("error: cannot find from block\n");
       exit(1);
     }
     if (io_num >= stage_clb[0][x_to][y_to].occ) {  /* Moving to an empty location */
	     //check stage_io_rat first
	     pad_swap_success = TRUE;
	     for (i = 0; i<num_stage; i++) {
	       if(*(stage_clb[i][x_from][y_from].u.io_blocks+off_from)!=EMPTY) {
		       pad_count=0;
		       for (j=0; j<stage_clb[0][x_to][y_to].occ; j++) {
			       if (stage_clb[i][x_to][y_to].u.io_blocks[j]!=EMPTY)
			         pad_count++;
		       }
		       if (pad_count == stage_io_rat) {
		         pad_swap_success = FALSE;
			       break;
		       }
		     }
	     }
	     if (pad_swap_success) {
         for (i = 0; i<num_stage; i++) {
	         if(*(stage_clb[i][x_from][y_from].u.io_blocks+off_from)!=EMPTY) {
             b_from = *(stage_clb[i][x_from][y_from].u.io_blocks+off_from);
	           b_to = EMPTY;
	           block[b_from].x = x_to;
	           block[b_from].y = y_to;
             //printf("b_from:%d in stage %d\n", b_from, i);
	           bflist[sum]=b_from;
	           btlist[sum]=b_to;
	           sum++;
	         }
         }
	     }
       else {
         free(test);
         free(dist_block);
         free(btlist);
         free(bflist);
         return;
       }
     }
     
	   else {  /*Moving to an occupied location */
	     //check stage_io_rat first (to)
	     pad_swap_success = TRUE;
	   for (i = 0; i<num_stage; i++) {
	     if(*(stage_clb[i][x_from][y_from].u.io_blocks+off_from)!=EMPTY) {
		     b_to = *(stage_clb[i][x_to][y_to].u.io_blocks+io_num);
		     pad_count=0;
		     for (j=0; j<stage_clb[0][x_to][y_to].occ; j++) {
			   if ((stage_clb[i][x_to][y_to].u.io_blocks[j]!=EMPTY)&&(stage_clb[i][x_to][y_to].u.io_blocks[j]!=b_to))
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
	     for (i = 0; i<num_stage; i++) {
	       if(*(stage_clb[i][x_to][y_to].u.io_blocks+io_num)!=EMPTY) {
		       b_from = *(stage_clb[i][x_from][y_from].u.io_blocks+off_from);
		       pad_count=0;
		       for (j=0; j<stage_clb[0][x_from][y_from].occ; j++) {
			     if ((stage_clb[i][x_from][y_from].u.io_blocks[j]!=EMPTY)&&(stage_clb[i][x_from][y_from].u.io_blocks[j]!=b_from))
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
     }
	   else {
	     for (i = 0; i<num_stage; i++) {
	       b_from = *(stage_clb[i][x_from][y_from].u.io_blocks+off_from);
	       b_to = *(stage_clb[i][x_to][y_to].u.io_blocks+io_num);
	       if((b_from!=EMPTY)&&(b_to!=EMPTY)) {
	         block[b_to].x = x_from;
	         block[b_to].y = y_from;
	         block[b_from].x = x_to;
	         block[b_from].y = y_to;
	       }
	       else if (b_from!=EMPTY) { 
	         block[b_from].x = x_to;
	         block[b_from].y = y_to;
	       }
	       else if (b_to!=EMPTY) {
	         block[b_to].x = x_from;
	         block[b_to].y = y_from;   
	       }
	       else
	         continue;
	       bflist[sum]=b_from;
	       btlist[sum]=b_to;
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

 delta_c = 0;                    /* Change in cost due to this swap. */
 bb_delta_c = 0;
 timing_delta_c = 0;

 if(is_folding)
   {for (i=0; i< length; i++)
     {if (bflist[i]!=EMPTY)
       {num_of_pins = pins_on_block[block[bflist[i]].type];
       // printf("bflist %d, num_pins %d\n", bflist[i], num_of_pins);
       break;}
     if (btlist[i]!=EMPTY)
       {num_of_pins = pins_on_block[block[btlist[i]].type];
       // printf("btlist %d, num_pins %d\n", btlist[i], num_of_pins);
       break;}
     }
   }
 else
   num_of_pins = pins_on_block[block[b_from].type];    

 // printf("num of pins %d\n", num_of_pins);

 num_nets_affected = find_affected_nets (nets_to_update, net_block_moved, 
     bflist, btlist, num_of_pins, length);

 //printf("net affected %d\n",num_nets_affected); 

 if (place_cost_type == NONLINEAR_CONG) {
    save_region_occ (old_region_occ_x, old_region_occ_y, num_regions);
 }

 bb_index = 0;               /* Index of new bounding box. */


 /* printf("num of nets %d\n", num_nets);
 printf("num of nets affected %d\n", num_nets_affected);
 */
 for (k=0;k<num_nets_affected;k++) {
    inet = nets_to_update[k];

    //printf("inet %d(%s)\n", inet, net[inet].name);
    //printf("the net is %d(%s) for %d \n", inet, net[inet].name, net_block_moved[k]);
/* If we swapped two blocks connected to the same net, its bounding box *
 * doesn't change.                                                      */

    if (net_block_moved[k] == FROM_AND_TO) 
       continue;
    //printf("%d \n", net_block_moved[k]);
    if (net[inet].num_pins <= SMALL_NET) {
     
      get_non_updateable_bb (inet, &bb_coord_new[bb_index]);
      
    }
    else {
       if (net_block_moved[k] == FROM) 
          update_bb (inet, &bb_coord_new[bb_index], &bb_edge_new[bb_index],
             x_from, y_from, x_to, y_to);      
       else
          update_bb (inet, &bb_coord_new[bb_index], &bb_edge_new[bb_index],
             x_to, y_to, x_from, y_from);      
    }

    if (place_cost_type != NONLINEAR_CONG) {
       temp_net_cost[inet] = get_net_cost (inet, &bb_coord_new[bb_index]);
       bb_delta_c += temp_net_cost[inet] - net_cost[inet];
    }
    else {
           /* Rip up, then replace with new bb. */
       update_region_occ (inet, &bb_coords[inet], -1, num_regions);  
       update_region_occ (inet, &bb_coord_new[bb_index],1, num_regions);
    }

    bb_index++;
 }   

 if (place_cost_type == NONLINEAR_CONG) {
    newcost = nonlinear_cong_cost(num_regions);
    bb_delta_c = newcost - *bb_cost;
 }

   
 if (place_algorithm == NET_TIMING_DRIVEN_PLACE ||
     place_algorithm == PATH_TIMING_DRIVEN_PLACE) {   
   /*in this case we redefine delta_c as a combination of timing and bb.  *
    *additionally, we normalize all values, therefore delta_c is in       *
    *relation to 1*/
  
   comp_delta_td_cost(bflist, btlist, num_of_pins, &timing_delta_c,	
		      &delay_delta_c, length);
   //printf("The delay %11.6g, timing %11.6g \n",delay_delta_c, timing_delta_c);
   delta_c = (1-timing_tradeoff) * bb_delta_c * inverse_prev_bb_cost + 
     timing_tradeoff * timing_delta_c * inverse_prev_timing_cost;
 }
 else {
   delta_c = bb_delta_c;
 }

 //printf("bb_delta_c %11.6g, delta_c %11.6g\n", bb_delta_c, delta_c);
 keep_switch = assess_swap (delta_c, t); 

 /* 1 -> move accepted, 0 -> rejected. */ 
 // printf("accepted? %d\n", keep_switch);
 
 //printf("the delta %g\n", bb_delta_c);


 if (keep_switch) {
    *cost = *cost + delta_c;    
    *bb_cost = *bb_cost + bb_delta_c;
    /* float check_bb;
    check_bb = comp_bb_cost(CHECK, place_cost_type, num_regions);
 
    if ((*bb_cost > check_bb+0.01)||(*bb_cost<check_bb-0.01))
      { printf("bb_cost %g\n", *bb_cost);
       printf("new cost %g\n", check_bb);
       printf("num of nets affected %d \n", num_nets_affected);
       exit(1);
      }
    */
    if (place_algorithm == NET_TIMING_DRIVEN_PLACE ||
	place_algorithm == PATH_TIMING_DRIVEN_PLACE) {      
      /*update the point_to_point_timing_cost and point_to_point_delay_cost 
	values from the temporary values*/
      *timing_cost = *timing_cost + timing_delta_c;
      *delay_cost = *delay_cost + delay_delta_c;

      update_td_cost(bflist, btlist, num_of_pins, length);
    }

 /* update net cost functions and reset flags. */

    bb_index = 0;

    for (k=0;k<num_nets_affected;k++) {
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
   if(!is_folding){
    if (block[b_from].type == CLB || block[b_from].type == MEM) {
       if (b_to != EMPTY) {
          clb[x_from][y_from].u.block = b_to; 
          clb[x_to][y_to].u.block = b_from;
       }
       else {
          clb[x_to][y_to].u.block = b_from;   
          clb[x_to][y_to].occ = 1;
          clb[x_from][y_from].occ = 0; 
       }
    }

    else {     /* io block was selected for moving */

     /* Get the "sub_block" number of the b_from block. */

       for (off_from=0;;off_from++) {
          if (clb[x_from][y_from].u.io_blocks[off_from] == b_from) break;
       }

       if (b_to != EMPTY) {   /* Swapped two blocks. */
          clb[x_to][y_to].u.io_blocks[io_num] = b_from;
          clb[x_from][y_from].u.io_blocks[off_from] = b_to;
       }
       else {                 /* Moved to an empty location */
          clb[x_to][y_to].u.io_blocks[clb[x_to][y_to].occ] = b_from;  
          clb[x_to][y_to].occ++;   
          for  (k=off_from;k<clb[x_from][y_from].occ-1;k++) { /* prevent gap  */
             clb[x_from][y_from].u.io_blocks[k] =          /* in io_blocks */
                clb[x_from][y_from].u.io_blocks[k+1];
          }
          clb[x_from][y_from].occ--;
       }
    }
   }
   else{ //for folding case
     if (clb[x_from][y_from].type == CLB || clb[x_from][y_from].type == MEM){
       // printf("length is %d\n", length);
       for(i=0; i<length; i++)
	 {b_from = bflist[i];
	 b_to = btlist[i];
	 //printf("the bfrom %d\n", b_from);
	 //printf("the bto %d\n", b_to);
	 if ((b_to != EMPTY)&&(b_from != EMPTY)) {
	   cstage = block[b_from].stage;
	   stage_clb[cstage-1][x_from][y_from].u.block = b_to; 
	   stage_clb[cstage-1][x_to][y_to].u.block = b_from;
	 }
	 else if (b_from!=EMPTY){
	   cstage = block[b_from].stage;
	   stage_clb[cstage-1][x_to][y_to].u.block = b_from;   
	   stage_clb[cstage-1][x_to][y_to].occ = 1;
	   stage_clb[cstage-1][x_from][y_from].occ = 0; 
	 }
	 else if (b_to!=EMPTY) {
	   cstage = block[b_to].stage;
	   stage_clb[cstage-1][x_from][y_from].u.block = b_to;   
	   stage_clb[cstage-1][x_from][y_from].occ = 1;
	   stage_clb[cstage-1][x_to][y_to].occ = 0; 
    	 }
	 }
     }
     else{ // for IO
       if_from = FALSE;
       if_to = FALSE;
       int from_blk;
       for (i=0; i<length; i++)
	 {b_from = bflist[i];
	 b_to = btlist[i];
	 if (b_from !=EMPTY)
	   {if_from = TRUE;
	   from_blk = b_from;
	   // printf("b_from %d(%s)\n", from_blk, block[from_blk].name);
	   }
	 if (b_to!=EMPTY)
	   if_to = TRUE;
	 //printf("from_x %d, from_y %d\n", x_from, y_from);
	 }
       if_out = FALSE;  
       if (if_from&&if_to)
	 { for (off_from=0;off_from<stage_clb[0][x_from][y_from].occ;off_from++) {
	   for (istage=0; istage<num_stage; istage++)
	     if (stage_clb[istage][x_from][y_from].u.io_blocks[off_from] == from_blk)
	       {//printf("sub pad %d \n", off_from);
		 if_out = TRUE;
		 break;}
	   if(if_out)
	     break;
	 }
	 if(!if_out)
	   {printf("error: cannot find from block\n");
	   exit(1);}
	 for (j=0; j<num_stage; j++){
	   int temp_from = stage_clb[j][x_from][y_from].u.io_blocks[off_from];
	   int temp_to = stage_clb[j][x_to][y_to].u.io_blocks[io_num];
	   stage_clb[j][x_to][y_to].u.io_blocks[io_num] = temp_from;
	   stage_clb[j][x_from][y_from].u.io_blocks[off_from] = temp_to;
	 }
	 }
       else if(if_from)
	 {//printf("empty to blk\n");
	   for (off_from=0;;off_from++) {
	     for (istage=0; istage<num_stage; istage++)
	       if (stage_clb[istage][x_from][y_from].u.io_blocks[off_from] == from_blk)
		 {//printf("sub pad %d \n", off_from);
		   if_out = TRUE;
		   break;}
	     if(if_out)
	       break;
	   }
	   for (j=0; j<num_stage; j++)
	     stage_clb[j][x_to][y_to].u.io_blocks[stage_clb[0][x_to][y_to].occ] = stage_clb[j][x_from][y_from].u.io_blocks[off_from];
	   
	   for (j=0; j<num_stage; j++)
	     stage_clb[j][x_to][y_to].occ++;
	   int num_occ = stage_clb[0][x_from][y_from].occ;
	   for (j =0; j<num_stage; j++){   
	     for (k=off_from;k<num_occ-1;k++) { /* prevent gap  */
	       stage_clb[j][x_from][y_from].u.io_blocks[k] =          /* in io_blocks */
		 stage_clb[j][x_from][y_from].u.io_blocks[k+1];
	     }
	     stage_clb[j][x_from][y_from].occ--;
	   }
	 }
       else
	 {printf("error! from block empty\n");
	 exit(1);}
     }
   }
 }   

 else {    /* Move was rejected.  */
/* Reset the net cost function flags first. */
   for (k=0;k<num_nets_affected;k++) {
     inet = nets_to_update[k];
     temp_net_cost[inet] = -1;
   }
    
 /* Restore the block data structures to their state before the move. */
   if (!is_folding){
     block[b_from].x = x_from;
     block[b_from].y = y_from;
     if (b_to != EMPTY) {
       block[b_to].x = x_to;
       block[b_to].y = y_to;
     }
   }
  else{
    for (i=0; i<length; i++)
      {b_from = bflist[i];
      b_to = btlist[i];
      if (b_from!=EMPTY)
	{block[b_from].x = x_from;
	block[b_from].y = y_from;
	}
      if (b_to!=EMPTY)
	{block[b_to].x = x_to;
	block[b_to].y = y_to;
	}
      }
  }
           	
/* Restore the region occupancies to their state before the move. */
  if (place_cost_type == NONLINEAR_CONG) {
    restore_region_occ (old_region_occ_x, old_region_occ_y, num_regions);
  }
 }

 free(bflist);
 free(btlist);
 // printf("come here\n");
 return(keep_switch);
}
