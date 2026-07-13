# Normalized actuator command from the adaptive controller to a hardware
# driver. Efforts are dimensionless in [-1, 1]: 0 = the actuator's declared
# neutral, +/-1 = the declared safe extremes (birth certificate). The sign
# carries NO semantic (which sign moves the vehicle forward/left is learned
# by the controller, never declared). Drivers map effort to their substrate
# (PWM microseconds, H-bridge duty+direction, ...) with a fixed piecewise
# affine transform and must clamp to [-1, 1].
std_msgs/Header header
float32 steering_effort
float32 drive_effort
