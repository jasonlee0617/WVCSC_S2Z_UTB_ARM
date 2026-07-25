from wvcsc_bringup.qt_image_viewer import (
    PREFERRED_IMAGE_TOPICS,
    image_to_qimage,
    image_topic_names,
)
from types import SimpleNamespace


def test_only_raw_image_topics_are_listed_in_field_order():
    topics = [
        ('/debug/compressed', ['sensor_msgs/msg/CompressedImage']),
        ('/other/image', ['sensor_msgs/msg/Image']),
        ('/vision/diseased_target_debug_image', ['sensor_msgs/msg/Image']),
        ('/camera/color/image_raw', ['sensor_msgs/msg/Image']),
        ('/vision/tree_debug_image', ['sensor_msgs/msg/Image']),
    ]
    assert image_topic_names(topics) == [
        *PREFERRED_IMAGE_TOPICS,
        '/other/image',
    ]


def test_bgr_image_conversion_does_not_require_cv_bridge():
    image = image_to_qimage(SimpleNamespace(
        encoding='bgr8', height=1, width=2, step=6,
        data=bytes((10, 20, 30, 40, 50, 60))))

    assert image.pixelColor(0, 0).getRgb()[:3] == (30, 20, 10)
    assert image.pixelColor(1, 0).getRgb()[:3] == (60, 50, 40)
