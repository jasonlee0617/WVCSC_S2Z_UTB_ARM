"""Alicia-M hand-eye sampling constants.

The hand-eye collector deliberately follows the official Alicia-M calibration
program's fixed joint sequence.  This module owns that immutable contract so
both Gazebo Classic and the physical C10 setup execute exactly the same order.
The collector still subjects every row to its own MoveIt, kinematic, image and
motion-lock safety gates before accepting it as a sample.
"""


# Keep the values and order byte-for-byte equivalent in meaning to
# ``alicia_m_calibration/scripts/hand_eye_calibration.py``.  Tuples make the
# public table immutable and avoid accidental runtime edits by a collector.
ALICIA_M_FIXED_JOINT_SAMPLES = (
    (0.0, -1.09, -0.87, 0.0, -0.77, 0.0),
    (-0.12, -1.09, -0.87, 0.0, -0.77, 0.0),
    (0.12, -1.09, -0.87, 0.0, -0.77, 0.0),
    (-0.20, -1.09, -0.87, 0.0, -0.77, 0.0),
    (0.0, -1.09, -0.87, 0.0, -0.77, 0.4),
    (0.0, -1.09, -0.87, 0.0, -0.77, -0.4),
    (0.0, -1.09, -0.87, 0.0, -0.77, 0.8),
    (0.0, -1.09, -0.87, 0.0, -0.65, 0.0),
    (0.0, -1.09, -0.87, 0.0, -0.90, 0.0),
    (0.0, -1.09, -0.87, 0.0, -0.55, 0.0),
    (0.0, -1.09, -0.87, 0.25, -0.77, 0.0),
    (0.0, -1.09, -0.87, -0.25, -0.77, 0.0),
    (0.0, -1.09, -0.87, 0.4, -0.77, 0.0),
    (0.15, -1.09, -0.87, 0.0, -0.77, 0.5),
    (-0.15, -1.09, -0.87, 0.0, -0.77, -0.5),
    (0.10, -1.09, -0.87, 0.0, -0.77, -0.6),
    (0.0, -1.09, -0.87, 0.2, -0.70, 0.3),
    (0.0, -1.09, -0.87, -0.2, -0.85, -0.3),
    (0.0, -1.09, -0.87, 0.15, -0.60, 0.5),
    (-0.10, -1.09, -0.87, -0.15, -0.70, -0.4),
)


def fixed_joint_samples():
    """Return the official fixed sequence without exposing mutable rows."""
    return ALICIA_M_FIXED_JOINT_SAMPLES
