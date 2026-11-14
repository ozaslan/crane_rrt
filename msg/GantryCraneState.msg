builtin_interfaces/Time stamp

# Iteration number of the simulation
uint64 iteration

# Elapsed simulation time in seconds (iteration * dt)
float64 sim_time

# Inputs (F_x and F_y)
float64[2] input

# Generalized coordinates (x, y, alpha, beta)
float64[4] position

# First time derivatives: dx, dy, dalpha, dbeta
float64[4] velocity

# Second time derivatives: ddx, ddy, ddalpha, ddbeta
float64[4] acceleration

# Rope length
float64 rope_length