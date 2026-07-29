# 中文说明：Qt 手动任务编辑器的 RViz Marker 构造器。
# 输入是路线编辑模型和候选停靠位，输出只用于可视化，不参与 Nav2 控制或喷洒判定。
# Marker 的 frame、颜色和命名空间必须与现有 Qt/RViz 操作约定兼容。
"""RViz marker construction for the Qt manual mission editor."""

import math

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from wvcsc_bringup.mission_editor_model import (
    POINT_INSPECT,
    POINT_TRANSIT,
    TREE_CANOPY_RADIUS_M,
    TREE_CANOPY_SEGMENTS,
    TREE_ROOT_RADIUS_M,
    copy_pose,
    vehicle_pose_from_arm_anchor,
)


class ManualMissionMarkerBuilder:
    """Build the unchanged RViz marker set from editor state."""

    def __init__(self, frame_id, stamp_factory):
        self.frame_id = frame_id
        self._stamp_factory = stamp_factory

    def build(self, editor, candidate, pending_dock=None):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        if editor.start_pose is not None:
            markers.markers.extend([
                self.marker(editor.start_pose, 'manual_start', 0, 0.2, 0.9, 0.2),
                self.vehicle_marker(
                    editor.start_pose, 'manual_start_vehicle', 0, 0.2, 0.9, 0.2),
                self.mount_line(
                    editor.start_pose, 'manual_start_mount', 0, 0.2, 0.9, 0.2),
            ])
        for index, point in enumerate(editor.points, start=1):
            color = {
                POINT_TRANSIT: (0.1, 0.6, 1.0),
                POINT_INSPECT: (1.0, 0.75, 0.0),
            }[point.point_type]
            markers.markers.extend([
                self.marker(point.pose, 'manual_target', index, *color),
                self.vehicle_marker(
                    point.pose, 'manual_target_vehicle', index, *color),
                self.mount_line(point.pose, 'manual_target_mount', index, *color),
                self.label(point.pose, index, point),
            ])
            if point.tree_pose is not None:
                markers.markers.extend([
                    self.tree_root_marker(point.tree_pose, index),
                    self.tree_canopy_marker(point.tree_pose, index),
                    self.tree_line(point.pose, point.tree_pose, index),
                    self.tree_distance_label(point.pose, point.tree_pose, index),
                    self.tree_label(point.tree_pose, index),
                ])
        route_anchors = []
        if editor.start_pose is not None:
            route_anchors.append(editor.start_pose)
        route_anchors.extend(point.pose for point in editor.points)
        if len(route_anchors) >= 2:
            markers.markers.append(self.vehicle_route_marker(route_anchors))
        if candidate is not None:
            markers.markers.extend([
                self.marker(candidate, 'manual_candidate', 1000, 1.0, 0.8, 0.0),
                self.vehicle_marker(
                    candidate, 'manual_candidate_vehicle', 1000, 1.0, 0.8, 0.0),
                self.mount_line(
                    candidate, 'manual_candidate_mount', 1000, 1.0, 0.8, 0.0),
            ])
        if pending_dock is not None:
            markers.markers.extend([
                self.marker(
                    pending_dock, 'manual_pending_inspect_dock', 1001,
                    0.75, 0.2, 0.9),
                self.vehicle_marker(
                    pending_dock, 'manual_pending_inspect_vehicle', 1001,
                    0.75, 0.2, 0.9),
            ])
        return markers

    def _marker(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self._stamp_factory()
        marker.action = Marker.ADD
        return marker

    def marker(self, pose, namespace, marker_id, red, green, blue):
        marker = self._marker()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.pose = copy_pose(pose)
        marker.scale.x = 0.55
        marker.scale.y = 0.14
        marker.scale.z = 0.14
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 0.9
        return marker

    def vehicle_marker(self, arm_anchor, namespace, marker_id,
                       red, green, blue):
        marker = self._marker()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.pose = vehicle_pose_from_arm_anchor(arm_anchor)
        marker.scale.x = marker.scale.y = marker.scale.z = 0.16
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 0.55
        return marker

    def mount_line(self, arm_anchor, namespace, marker_id,
                   red, green, blue):
        marker = self._marker()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.025
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 0.65
        marker.points.extend([
            arm_anchor.position,
            vehicle_pose_from_arm_anchor(arm_anchor).position,
        ])
        return marker

    def vehicle_route_marker(self, arm_anchors):
        marker = self._marker()
        marker.ns = 'manual_vehicle_route'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.045
        marker.color.r = 0.0
        marker.color.g = 0.95
        marker.color.b = 0.95
        marker.color.a = 0.90
        for arm_anchor in arm_anchors:
            vehicle_pose = vehicle_pose_from_arm_anchor(arm_anchor)
            marker.points.append(Point(
                x=vehicle_pose.position.x,
                y=vehicle_pose.position.y,
                z=0.06,
            ))
        return marker

    def label(self, pose, index, point):
        marker = self._marker()
        marker.ns = 'manual_target_label'
        marker.id = index
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose = copy_pose(pose)
        marker.pose.position.z += 0.35
        marker.scale.z = 0.25
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        marker.text = f'{index}: {point.point_type}'
        if point.point_type == POINT_INSPECT:
            marker.text += f' {point.work_side}'
        return marker

    def tree_root_marker(self, pose, marker_id):
        marker = self._marker()
        marker.ns = 'manual_tree_root'
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.pose = copy_pose(pose)
        marker.pose.position.z = 0.015
        marker.scale.x = marker.scale.y = TREE_ROOT_RADIUS_M * 2.0
        marker.scale.z = 0.03
        marker.color.r = 0.45
        marker.color.g = 0.20
        marker.color.b = 0.05
        marker.color.a = 0.95
        return marker

    def tree_canopy_marker(self, pose, marker_id):
        marker = self._marker()
        marker.ns = 'manual_tree_canopy'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.025
        marker.color.r = 1.0
        marker.color.g = 0.72
        marker.color.b = 0.05
        marker.color.a = 0.95
        for step in range(TREE_CANOPY_SEGMENTS + 1):
            angle = 2.0 * math.pi * step / TREE_CANOPY_SEGMENTS
            marker.points.append(Point(
                x=pose.position.x + TREE_CANOPY_RADIUS_M * math.cos(angle),
                y=pose.position.y + TREE_CANOPY_RADIUS_M * math.sin(angle),
                z=0.04,
            ))
        return marker

    def tree_distance_label(self, arm_anchor, tree, marker_id):
        marker = self._marker()
        marker.ns = 'manual_tree_distance'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position.x = (arm_anchor.position.x + tree.position.x) / 2.0
        marker.pose.position.y = (arm_anchor.position.y + tree.position.y) / 2.0
        marker.pose.position.z = 0.24
        marker.scale.z = 0.18
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        distance = math.hypot(
            tree.position.x - arm_anchor.position.x,
            tree.position.y - arm_anchor.position.y)
        marker.text = f'ARM-ROOT: {distance:.2f} m'
        return marker

    def tree_label(self, pose, marker_id):
        marker = self._marker()
        marker.ns = 'manual_tree_label'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose = copy_pose(pose)
        marker.pose.position.z = 0.30
        marker.scale.z = 0.16
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.2
        marker.color.a = 1.0
        marker.text = f'ROOT\nCANOPY r={TREE_CANOPY_RADIUS_M:.2f}m'
        return marker

    def tree_line(self, docking, tree, marker_id):
        marker = self._marker()
        marker.ns = 'manual_tree_link'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.04
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 0.85
        marker.points.extend([docking.position, tree.position])
        return marker
