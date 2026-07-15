import math

import numpy as np

from wvcsc_visual_servo.controllers.pid_controller import (
    PIDController3D,
    ServoControlConfig,
)
from wvcsc_visual_servo.servo.command_limiter import limit_xy_norm, slew
from wvcsc_visual_servo.servo.servo_status_policy import (
    ServoStatusAction,
    ServoStatusPolicy,
)
from wvcsc_visual_servo.servo.target_estimator import SimpleTargetPredictor2D


def test_pid_disables_depth_axis_and_resets():
    controller = PIDController3D(ServoControlConfig(kp_xy=0.2))
    x, y, z, _debug = controller.step([0.1, -0.2, 1.0], 0.02)
    assert x > 0.0 and y < 0.0 and z == 0.0
    controller.reset()


def test_limiter_caps_norm_and_acceleration():
    x, y = limit_xy_norm(3.0, 4.0, 1.0)
    assert math.isclose(math.hypot(x, y), 1.0)
    assert math.isclose(slew(1.0, 0.0, 2.0, 0.1), 0.2)


def test_moveit_servo_status_policy_is_conservative():
    policy = ServoStatusPolicy({1, 3, 6}, {2, 4, 5})
    assert policy.decide(0).action == ServoStatusAction.OK
    assert policy.decide(3).action == ServoStatusAction.DECELERATE
    assert policy.decide(4).action == ServoStatusAction.HALT_RECOVERY
    assert policy.decide(99).action == ServoStatusAction.HALT_RECOVERY


def test_predictor_uses_bounded_horizon():
    predictor = SimpleTargetPredictor2D()
    predictor.update([1.0, 2.0], [2.0, -1.0], 10.0)
    position, velocity = predictor.predict_to(11.0, 0.1)
    assert np.allclose(position, [1.2, 1.9])
    assert np.allclose(velocity, [2.0, -1.0])
