t_linked_int *allocate_and_load_critical_path (int istage, boolean if_pad) {

/* Finds the critical path and puts a list of the tnodes on the critical    *
 * path in a linked list, from the path SOURCE to the path SINK.            */

 t_linked_int *critical_path_head, *critical_path_node, *curr_crit_node, *prev_crit_node;
 int inode, iedge, to_node, num_at_level, i, crit_node, num_edges, spec_node, from_node, iblk;
 float min_slack, slack;
 t_tedge *tedge;
 float max_delay =0;
 t_linked_int *next_crit_node;
 float Tdel, total_delay =0;
 t_tnode_type type, next_type;

 num_at_level = tnodes_at_level[0].nelem;
 
/* Stops compiler warnings. */
 spec_node = OPEN;

 for (i=0;i<num_at_level;i++) {   /* Path must start at SOURCE (no inputs) */
   inode = tnodes_at_level[0].list[i];
   crit_node = OPEN; 
   total_delay =0;
   /* Modified by Wei */
   if (!is_folding){
     crit_node = inode;
   }
   else{
     if (istage != num_stage){
       if ((inode>=tnode_stage[istage-1])&&(inode<tnode_stage[istage])){
	 crit_node = inode;
       }
     }
     else {
       if (inode>=tnode_stage[istage-1]){
	 crit_node = inode;
       }
     }
   }
   if (crit_node!=OPEN)  
     {      
       num_edges = tnode[crit_node].num_edges;       
       while (num_edges != 0) {   /* Path will end at SINK (no fanout) */
	 from_node = crit_node;
	 type = tnode_descript[from_node].type;
	 tedge = tnode[crit_node].out_edges;
	 min_slack = HUGE_FLOAT;
	 iblk = tnode_descript[from_node].iblk;
	 printf ("Node: %d  %s Block #%d (%s)\n", from_node, tnode_type_names[type], 
           iblk, block[iblk].name);
	 for (iedge=0;iedge<num_edges;iedge++) {
	   to_node = tedge[iedge].to_node;
	   slack = tnode[to_node].T_req - tnode[to_node].T_arr;
	   if (slack < min_slack) {
	     crit_node = to_node;
	     min_slack = slack;
	   } 
	 }
	 next_type= tnode_descript[crit_node].type;
	 iblk = tnode_descript[crit_node].iblk;
	 printf ("Node: %d  %s Block #%d (%s)\n", crit_node, tnode_type_names[next_type], 
           iblk, block[iblk].name);
	 Tdel = tnode[crit_node].T_arr - tnode[from_node].T_arr;
	 printf ("the delay %g\n", Tdel);
	 if((type != INPAD_OPIN)&& (next_type != OUTPAD_IPIN)) 
	   {total_delay = total_delay + Tdel;
	   printf("total delay become %g\n", total_delay);}
	 num_edges = tnode[crit_node].num_edges;
       }
     }
   else
     continue;
   if (total_delay > max_delay)
     {max_delay = total_delay;
     spec_node = inode;
     }
   printf("spec node is %d\n", spec_node);
 }

 if (spec_node == OPEN)
   {printf("error: didn't find critical path\n");
   exit(1);
   }
 else
   {critical_path_head = (t_linked_int *) my_malloc (sizeof (t_linked_int));
   critical_path_head->data = spec_node;
   crit_node = spec_node;
   prev_crit_node = critical_path_head;
   num_edges = tnode[crit_node].num_edges;
   
   while (num_edges != 0) {   /* Path will end at SINK (no fanout) */
     curr_crit_node = (t_linked_int *) my_malloc (sizeof (t_linked_int));
     prev_crit_node->next = curr_crit_node;
     tedge = tnode[crit_node].out_edges;
     min_slack = HUGE_FLOAT;
  
     for (iedge=0;iedge<num_edges;iedge++) {
       to_node = tedge[iedge].to_node;
       slack = tnode[to_node].T_req - tnode[to_node].T_arr;
       
       if (slack < min_slack) {
          crit_node = to_node;
          min_slack = slack;
       } 
    }
     
    curr_crit_node->data = crit_node;
    prev_crit_node = curr_crit_node;
    num_edges = tnode[crit_node].num_edges;
   }
   
   prev_crit_node->next = NULL;
   return (critical_path_head);
   }
}

