boolean is_opin (int ipin);
/* Mofified by Wei */
void load_one_clb_fanout_count (int subblock_lut_size, t_subblock
         *subblock_inf, int num_subblocks, int *num_uses_of_clb_ipin,
         int *num_uses_of_sblk_opin, int iblk, t_ffblock *ffblock_inf,
         int num_ffs, int *num_uses_of_ff_opin, int max_subblock_per_block);
