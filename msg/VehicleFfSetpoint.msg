# Feedforward setpoint from an external time-parameterized trajectory planner.
# Consumed by fw_ff_control to compute fixed-wing feedforward augmentation terms
# (u = u_ff + u_fb); the existing feedback controllers remain unchanged.

uint64 timestamp # [us] Time since system start

float32 normal_accel_ff # [-] Normal load factor feedforward in the wind frame (n_n), 1.0 in steady level flight
float32 tangential_accel_ff # [-] Tangential load factor feedforward in the wind frame (n_t)

float32 roll_rate_ff # [rad/s] Desired body roll rate feedforward
