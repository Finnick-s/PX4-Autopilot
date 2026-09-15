# Fixed-wing feedforward augmentation terms computed by fw_ff_control from
# vehicle_ff_setpoint. All fields are final additive increments already scaled
# by the FW_*_FF_GAIN parameters and are consumed by fw_att_control
# (roll/pitch rate) and fw_lat_lon_control (equivalent airspeed).

uint64 timestamp # [us] Time since system start

float32 pitch_rate_ff # [rad/s] Body pitch rate feedforward increment (q_ff = FW_NN_FF_GAIN * g/V * (n_n_ff - 1))
float32 roll_rate_ff # [rad/s] Body roll rate feedforward increment (FW_P_FF_GAIN * p_ff)
float32 ste_rate_ff # [m^2/s^3] Specific total energy rate feedforward increment (FW_NT_FF_GAIN * V_TAS * g * n_t_ff)

float32 true_airspeed # [m/s] True airspeed used to compute the increments (for logging/verification)

bool valid # True while the vehicle_ff_setpoint input is being received and the feedforward terms are active
