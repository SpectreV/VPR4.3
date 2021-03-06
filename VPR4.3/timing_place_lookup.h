#define IMPOSSIBLE -1 /*indicator of an array location that    */
                      /*should never be accessed*/

void compute_delay_lookup_tables(struct s_router_opts router_opts, 
			   struct s_det_routing_arch det_routing_arch, 
			   t_segment_inf *segment_inf, 
			   t_timing_inf timing_inf, 
			   t_chan_width_dist chan_width_dist, 
                           t_subblock_data subblock_data); 
void free_place_lookup_structs(void);

void print_array(float** array, int x1, int x2, int y1, int y2);

extern float **delta_inpad_to_clb;
extern float **delta_clb_to_clb;
extern float **delta_clb_to_outpad;
extern float **delta_inpad_to_outpad;

