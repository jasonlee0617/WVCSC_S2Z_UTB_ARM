"""Pure target-template tracking and cross-frame association."""

from dataclasses import dataclass, replace
import math

import cv2
import numpy as np

from .perception_types import Instance


@dataclass(frozen=True)
class TargetTemplate:
    """Appearance patch and target geometry relative to that patch."""

    patch: np.ndarray
    bbox_left: float
    bbox_top: float
    bbox_right: float
    bbox_bottom: float
    aim_u: float
    aim_v: float
    confidence: float


def capture_target_template(
        image, target, padding_ratio=0.50, min_padding_px=6.0):
    """Capture a non-flat target patch for short detector dropouts."""
    height, width = image.shape[:2]
    pad_x = max(float(min_padding_px), target.width * float(padding_ratio))
    pad_y = max(float(min_padding_px), target.height * float(padding_ratio))
    left = max(0, int(math.floor(target.left - pad_x)))
    top = max(0, int(math.floor(target.top - pad_y)))
    right = min(width, int(math.ceil(target.right + pad_x)))
    bottom = min(height, int(math.ceil(target.bottom + pad_y)))
    if right - left < 3 or bottom - top < 3:
        return None
    patch = image[top:bottom, left:right].copy()
    if patch.size == 0 or float(np.std(patch)) < 1.0:
        return None
    return TargetTemplate(
        patch,
        target.left - left,
        target.top - top,
        target.right - left,
        target.bottom - top,
        target.aim_u - left,
        target.aim_v - top,
        target.confidence,
    )


def match_target_template(
        image, template, reference, search_radius_px=80.0,
        min_score=0.55):
    """Locate a captured template near the previous target position."""
    image_height, image_width = image.shape[:2]
    template_height, template_width = template.patch.shape[:2]
    previous_left = reference.left - template.bbox_left
    previous_top = reference.top - template.bbox_top
    radius = max(0.0, float(search_radius_px))
    search_left = max(0, int(math.floor(previous_left - radius)))
    search_top = max(0, int(math.floor(previous_top - radius)))
    search_right = min(
        image_width,
        int(math.ceil(previous_left + template_width + radius)),
    )
    search_bottom = min(
        image_height,
        int(math.ceil(previous_top + template_height + radius)),
    )
    search = image[search_top:search_bottom, search_left:search_right]
    if (search.shape[0] < template_height or
            search.shape[1] < template_width):
        return None
    scores = cv2.matchTemplate(
        search, template.patch, cv2.TM_CCOEFF_NORMED)
    _, score, _, location = cv2.minMaxLoc(scores)
    if not math.isfinite(score) or score < float(min_score):
        return None
    patch_left = search_left + location[0]
    patch_top = search_top + location[1]
    return Instance(
        reference.target_id,
        reference.class_name,
        min(float(template.confidence), float(score)),
        patch_left + template.bbox_left,
        patch_top + template.bbox_top,
        patch_left + template.bbox_right,
        patch_top + template.bbox_bottom,
        patch_left + template.aim_u,
        patch_top + template.aim_v,
    )


def track_matches(instances, tracks, iou_threshold, center_distance_px):
    """Return a one-to-one current-instance to previous-track assignment."""
    candidates = []
    for instance_index, instance in enumerate(instances):
        for track_index, track in enumerate(tracks):
            if instance.class_name != track.instance.class_name:
                continue
            iou = instance.iou(track.instance)
            distance = instance.distance_to(track.instance)
            if iou >= iou_threshold or distance <= center_distance_px:
                candidates.append((
                    0 if iou >= iou_threshold else 1,
                    -iou,
                    distance,
                    instance_index,
                    track_index,
                ))
    matches = {}
    used_tracks = set()
    for _kind, _iou, _distance, instance_index, track_index in sorted(candidates):
        if instance_index in matches or track_index in used_tracks:
            continue
        matches[instance_index] = track_index
        used_tracks.add(track_index)
    return matches


def reassociation_candidate(
        reference, instances, iou_threshold, center_distance_px,
        iou_margin, distance_margin_px, equivalent_aim_distance_px,
        canonical_target_class_name='diseased_target',
        allow_ambiguous_nearest=False):
    """Resolve a selected logical target after detector track-ID churn."""
    scored = []
    for instance in instances:
        if instance.class_name != canonical_target_class_name:
            continue
        iou = instance.iou(reference)
        distance = instance.distance_to(reference)
        if iou >= iou_threshold or distance <= center_distance_px:
            scored.append((instance, iou, distance))
    overlap = sorted(
        (item for item in scored if item[1] >= iou_threshold),
        key=lambda item: (
            -item[1], item[2], item[0].left, item[0].top,
            item[0].right, item[0].bottom),
    )
    if overlap:
        if (len(overlap) > 1 and
                overlap[0][1] - overlap[1][1] < iou_margin):
            if math.hypot(
                    overlap[0][0].aim_u - overlap[1][0].aim_u,
                    overlap[0][0].aim_v - overlap[1][0].aim_v) <= (
                        equivalent_aim_distance_px):
                return overlap[0][0], 'equivalent_reassociation'
            if allow_ambiguous_nearest:
                return overlap[0][0], 'nearest_reassociation'
            return None, 'ambiguous_reassociation'
        return overlap[0][0], 'none'
    nearby = sorted(
        scored,
        key=lambda item: (
            item[2], item[0].left, item[0].top,
            item[0].right, item[0].bottom),
    )
    if not nearby:
        return None, 'selected_id_missing'
    if (len(nearby) > 1 and
            nearby[1][2] - nearby[0][2] < distance_margin_px):
        if math.hypot(
                nearby[0][0].aim_u - nearby[1][0].aim_u,
                nearby[0][0].aim_v - nearby[1][0].aim_v) <= (
                    equivalent_aim_distance_px):
            return nearby[0][0], 'equivalent_reassociation'
        if allow_ambiguous_nearest:
            return nearby[0][0], 'nearest_reassociation'
        return None, 'ambiguous_reassociation'
    return nearby[0][0], 'none'


def smoothed_target(reference, target, alpha):
    """Smooth the target control point while preserving its latest box."""
    alpha = float(alpha)
    return Instance(
        target.target_id,
        target.class_name,
        target.confidence,
        target.left,
        target.top,
        target.right,
        target.bottom,
        (1.0 - alpha) * reference.aim_u + alpha * target.aim_u,
        (1.0 - alpha) * reference.aim_v + alpha * target.aim_v,
    )


def update_target_template(
        current, image, target, *, min_confidence,
        padding_ratio, min_padding_px):
    """Capture and retain the best confidence associated with a template."""
    if image is None or target.confidence < float(min_confidence):
        return current
    captured = capture_target_template(
        image, target, padding_ratio, min_padding_px)
    if captured is None:
        return current
    if current is not None:
        captured = replace(
            captured,
            confidence=max(current.confidence, captured.confidence),
        )
    return captured
