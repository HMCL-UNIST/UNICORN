int32 id

float64 s_start
float64 s_end
float64 d_right           # defined as right bound of the obstacle
float64 d_left            # defined as left bound of the obstacle

float64 x_m
float64 y_m

bool is_actually_a_gap    # used by the frenet planner to choos points through which we want to go
float64 s_center
float64 d_center
float64 size
float64 vs
float64 vd
float64 s_var
float64 d_var
float64 vs_var
float64 vd_var
bool is_static
bool is_visible