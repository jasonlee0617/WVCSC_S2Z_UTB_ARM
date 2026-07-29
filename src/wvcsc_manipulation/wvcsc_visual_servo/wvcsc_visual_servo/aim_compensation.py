# 中文说明：喷嘴轴线在相机图像中的标定补偿与投影工具。
# 输入是 camera_color_optical_frame 下的 TF 和工作平面，输出给 AlignTarget 的像素目标；
# 本模块只计算几何，不移动机械臂、不发送继电器命令。
"""Project a calibrated spray-nozzle axis into the C10 image.

The transform supplied to :func:`project_nozzle_axis` must describe the
nozzle frame in ``camera_color_optical_frame``.  ROS optical axes are +X
right, +Y down and +Z forward; the nozzle spray axis is its local +Z axis.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class AimSolution:
    """Validated nozzle projection for one fixed working-distance plane."""

    u_px: float
    v_px: float
    range_m: float
    intersection: tuple
    forward_axis: tuple


def _rotate_z_axis(quaternion):
    """Return ``R(quaternion) * [0, 0, 1]`` for an ``x,y,z,w`` quaternion."""
    x, y, z, w = (float(value) for value in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError('nozzle quaternion is invalid')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return (
        2.0 * (x * z + y * w),
        2.0 * (y * z - x * w),
        1.0 - 2.0 * (x * x + y * y),
    )


def project_nozzle_axis(
        translation, quaternion, camera, range_m, trim=(0.0, 0.0),
        min_forward_axis_z=0.2, image_margin_px=0.0):
    """Project the nozzle +Z ray onto the camera plane ``z=range_m``.

    ``camera`` is ``(fx, fy, cx, cy, width, height)``.  Invalid geometry is
    rejected instead of silently falling back to the image centre, because a
    fallback could command a geometrically wrong spray.
    """
    ox, oy, oz = (float(value) for value in translation)
    fx, fy, cx, cy, width, height = (float(value) for value in camera)
    range_m = float(range_m)
    trim_u, trim_v = (float(value) for value in trim)
    min_forward_axis_z = float(min_forward_axis_z)
    margin = float(image_margin_px)
    values = (
        ox, oy, oz, fx, fy, cx, cy, width, height, range_m,
        trim_u, trim_v, min_forward_axis_z, margin,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('aim compensation contains non-finite values')
    if fx <= 0.0 or fy <= 0.0 or width <= 0.0 or height <= 0.0:
        raise ValueError('camera intrinsics are invalid')
    if range_m <= 0.0 or min_forward_axis_z <= 0.0:
        raise ValueError('working range and forward-axis limit must be positive')
    if margin < 0.0 or 2.0 * margin >= min(width, height):
        raise ValueError('image safety margin is invalid')

    dx, dy, dz = _rotate_z_axis(quaternion)
    if dz <= min_forward_axis_z:
        raise ValueError(
            f'nozzle axis does not face the camera plane: d.z={dz:.6f}')
    scale = (range_m - oz) / dz
    if not math.isfinite(scale) or scale <= 0.0:
        raise ValueError('nozzle ray intersects behind the nozzle origin')
    px = ox + scale * dx
    py = oy + scale * dy
    pz = oz + scale * dz
    if pz <= 0.0:
        raise ValueError('nozzle intersection is behind the camera')
    u_px = fx * px / pz + cx + trim_u
    v_px = fy * py / pz + cy + trim_v
    if not (margin <= u_px < width - margin and
            margin <= v_px < height - margin):
        raise ValueError(
            f'compensated aim pixel ({u_px:.2f}, {v_px:.2f}) is outside '
            'the safe image region')
    return AimSolution(
        u_px=u_px,
        v_px=v_px,
        range_m=range_m,
        intersection=(px, py, pz),
        forward_axis=(dx, dy, dz),
    )


def plane_error_mm(error_u_px, error_v_px, fx, fy, range_m):
    """Estimate image-plane metric error at the configured fixed range."""
    values = tuple(float(value) for value in (
        error_u_px, error_v_px, fx, fy, range_m))
    if not all(math.isfinite(value) for value in values):
        return math.nan
    error_u_px, error_v_px, fx, fy, range_m = values
    if fx <= 0.0 or fy <= 0.0 or range_m <= 0.0:
        return math.nan
    return 1000.0 * math.hypot(
        error_u_px * range_m / fx,
        error_v_px * range_m / fy,
    )
