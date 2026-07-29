"""ROS detection conversion and OpenCV rendering for perception results."""

import cv2
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


def instance_to_detection(header, instance):
    """Convert one internal image-space instance to ``Detection2D``."""
    detection = Detection2D()
    detection.header = header
    detection.id = instance.target_id
    detection.bbox.center.position.x = instance.center_u
    detection.bbox.center.position.y = instance.center_v
    detection.bbox.size_x = float(instance.width)
    detection.bbox.size_y = float(instance.height)
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = instance.class_name
    hypothesis.hypothesis.score = instance.confidence
    detection.results = [hypothesis]
    return detection


def instances_to_array(image, instances):
    """Convert instances while preserving the input image header."""
    array = Detection2DArray()
    array.header = image.header
    array.detections = [
        instance_to_detection(image.header, instance)
        for instance in instances
    ]
    return array


def instance_label(instance):
    """Return a compact, operator-readable debug label.

    Full UUIDs remain on ROS messages for machine correlation but are not an
    operator-facing physical identity and therefore do not belong on the image.
    """
    return f'DISEASE {instance.confidence:.2f}'


def annotated_image(
        image, instances, *, draw_diseased_aim_point=False,
        selected_target_id=''):
    """Render boxes, labels, selected-target emphasis and aim points."""
    annotated = image.copy()
    for instance in instances:
        selected = bool(
            selected_target_id and
            instance.target_id == selected_target_id
        )
        color = (255, 255, 0) if selected else (0, 255, 255)
        thickness = 4 if selected else 2
        left, top = round(instance.left), round(instance.top)
        cv2.rectangle(
            annotated,
            (left, top),
            (round(instance.right), round(instance.bottom)),
            color,
            thickness,
        )
        label = instance_label(instance)
        if selected:
            label = f'LOCKED {label}'
        cv2.putText(
            annotated,
            label,
            (left, max(16, top - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
            cv2.LINE_AA,
        )
        if draw_diseased_aim_point:
            cv2.circle(
                annotated,
                (round(instance.aim_u), round(instance.aim_v)),
                5 if selected else 3,
                color,
                -1,
            )
    return annotated
