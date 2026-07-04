uint64 timestamp		# time since system start (microseconds)

# rate controller integrator status
float32 rollspeed_integ
float32 pitchspeed_integ
float32 yawspeed_integ
float32 wheel_rate_integ	# FW only and optional

# RBF-LADRC residual normalized torque compensation.
# Zero when RBF residual compensation is disabled.
float32 rollspeed_rbf
float32 pitchspeed_rbf
float32 yawspeed_rbf

# RBF-LADRC diagnostics.
float32[3] rbf_target		# equivalent residual-disturbance torque learning target
float32[3] rbf_output_raw	# residual network output before output limiting
float32[3] rbf_output_filtered	# residual output after limiting, low-pass and slew-rate limiting
float32 rbf_weight_norm		# L2 norm of active RBF weights
float32 rbf_phi_max		# maximum active basis-function value
bool rbf_learn_flag		# true when at least one axis passes the online learning gate
bool rbf_saturation_flag	# true when RBF output/final injected torque saturates

float32[3] rbf_residual_accel	# filtered residual angular acceleration: angular_accel - (z2 + b0 * u_last)
float32[3] rbf_feature_e		# normalized filtered rate-error feature
float32[3] rbf_feature_uladrc	# normalized LADRC torque feature before RBF injection
float32[3] rbf_feature_udist	# normalized LADRC disturbance compensation feature
bool rbf_freeze_flag		# true when at least one RBF axis is frozen by protection logic
uint8[3] rbf_freeze_reason	# per-axis freeze bitmask: sign, target jump, accel, saturation, setpoint jump, gate
