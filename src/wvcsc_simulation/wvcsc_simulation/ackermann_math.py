"""无 ROS 依赖的 Ackermann 几何辅助函数。"""

import math


def yaw_rate_from_steering(speed, steering_angle, wheel_base):
    """根据有符号车速、前轮转向角和轴距计算偏航角速度。"""
    return float(speed) * math.tan(float(steering_angle)) / float(wheel_base)


def yaw_rate_from_twist(speed, yaw_rate, wheel_base, max_steering_angle):
    """将标准 Twist 偏航角速度限制在阿克曼可实现范围内。"""
    maximum = (
        abs(float(speed)) * math.tan(float(max_steering_angle))
        / float(wheel_base)
    )
    return max(-maximum, min(maximum, float(yaw_rate)))


def point_to_segment_distance(point_x, point_y, start, end):
    """Return the Euclidean distance from a 2D point to a finite segment."""
    start_x, start_y = (float(value) for value in start)
    end_x, end_y = (float(value) for value in end)
    delta_x = end_x - start_x
    delta_y = end_y - start_y
    length_squared = delta_x * delta_x + delta_y * delta_y
    if length_squared <= 1e-12:
        return math.hypot(float(point_x) - start_x, float(point_y) - start_y)
    projection = (
        ((float(point_x) - start_x) * delta_x +
         (float(point_y) - start_y) * delta_y) / length_squared)
    projection = max(0.0, min(1.0, projection))
    return math.hypot(
        float(point_x) - (start_x + projection * delta_x),
        float(point_y) - (start_y + projection * delta_y))


def point_to_polyline_distance(point_x, point_y, points):
    """Return the shortest 2D distance from a point to a non-empty polyline."""
    points = tuple((float(x), float(y)) for x, y in points)
    if not points:
        return math.nan
    if len(points) == 1:
        return math.hypot(float(point_x) - points[0][0],
                          float(point_y) - points[0][1])
    return min(
        point_to_segment_distance(point_x, point_y, start, end)
        for start, end in zip(points, points[1:]))
