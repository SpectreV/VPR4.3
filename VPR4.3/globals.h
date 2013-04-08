/* Netlist to be placed stuff. */
extern int num_nets, num_blocks; 
extern int num_mem, mem_sink, mem_source;
extern int dsp_sink, dsp_source;
extern int num_p_inputs, num_p_outputs, num_clbs, num_globals;
extern struct s_net *net;
extern struct s_block *block;
extern boolean *is_global;

/* Physical FPGA architecture stuff */
extern int nx, ny, io_rat, pins_per_clb, stage_io_rat;
extern int **pinloc;
extern int *clb_pin_class;
extern boolean *is_global_clb_pin;
extern struct s_class *class_inf;
extern int num_class;

/* chan_width_x is the x-directed channel; i.e. between rows */
extern int *chan_width_x, *chan_width_y; /* numerical form */
extern struct s_clb **clb;

/* [0..num_nets-1] of linked list start pointers.  Defines the routing.  */
extern struct s_trace **trace_head, **trace_tail;   

/* Structures to define the routing architecture of the FPGA.           */
extern int num_rr_nodes;
extern t_rr_node *rr_node;                   /* [0..num_rr_nodes-1]          */
extern int num_rr_indexed_data;
extern t_rr_indexed_data *rr_indexed_data;   /* [0 .. num_rr_indexed_data-1] */
extern int **net_rr_terminals;             /* [0..num_nets-1][0..num_pins-1] */
extern struct s_switch_inf *switch_inf; /* [0..det_routing_arch.num_switch-1] */
extern int **rr_clb_source;              /* [0..num_blocks-1][0..num_class-1] */

/* Added by Wei to support logic folding */
extern boolean is_folding;
extern boolean is_random;
extern boolean is_mem;
extern struct s_hash **map_table;
extern int *blockmap_inf;
extern int *tnode_stage;
extern boolean if_level;
extern boolean if_direct;

extern struct p_index *num_net_per_stage;
extern struct p_index *num_block_per_stage;
extern int num_stage;
extern struct s_clb ***stage_clb;
extern int current_stage;
extern int num_pads;
extern int num_smbs;
extern int num_direct;

/*added by nionio*/
extern int fs;
extern int fo;
extern int fi;

/*added by LiangHao*/
extern int pins_per_dsp, dsp_w, dsp_h;
extern int ****dsp_pinloc; /*[0...dsp_w][0...dsp_h][0...3][0...pins_per_dsp-1]*/
extern int *dsp_pin_offset_x, *dsp_pin_offset_y; /*[0...pins_per_dsp-1]*/
extern int ***dsp_num_pin_loc_assignments; /*[0...dsp_w][0...dsp_h][0...3]*/
extern int *dsp_pin_class; /*[0...pins_per_dsp-1]*/
extern boolean *is_global_dsp_pin; /*[0...pins_per_dsp-1]*/
extern struct s_class *dsp_class_inf; /*[0...dsp_num_class-1]*/
extern int dsp_num_class;
extern int dsp_fc;
extern int dsp_loc_start, dsp_loc_repeat;
extern int num_dsp;
extern int num_dsp_direct;
extern int* dsp_pin_to_direct;
