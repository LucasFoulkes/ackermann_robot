"""Compatibility shim: adaptive_model moved to the adaptive_ackermann
package in the 2026-07-13 hardware split. Old experiment scripts and any
external imports keep working through this re-export."""
from adaptive_ackermann.adaptive_model import *          # noqa: F401,F403
from adaptive_ackermann.adaptive_model import (          # noqa: F401
    DelayEstimator, PathGeometry, TrackabilityEstimator)
