"""Map logical effort (-1..1) to raw /ackermann/cmd_effort with PWM deadband.

Tuning (from PlotJuggler): find the first raw effort where |odom.linear.x| > 0.05 m/s.
Use those as dead_pos / dead_neg. This module applies the inverse so Nav2/cmd_vel
can command "logical" full scale and the driver still gets enough PWM.
"""


def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


def logical_to_raw(u, dead_neg, dead_pos):
    """Desired motion effort -> raw effort sent to ackermann_driver."""
    u = clamp(u)
    if u > 0.0:
        return clamp(dead_pos + u * (1.0 - dead_pos))
    if u < 0.0:
        return clamp(-dead_neg + u * (1.0 - dead_neg))
    return 0.0


def raw_to_logical(u, dead_neg, dead_pos):
    """Raw effort -> motion effort (for log analysis / calibration)."""
    u = clamp(u)
    if u > dead_pos:
        return clamp((u - dead_pos) / (1.0 - dead_pos))
    if u < -dead_neg:
        return clamp((u + dead_neg) / (1.0 - dead_neg))
    return 0.0
