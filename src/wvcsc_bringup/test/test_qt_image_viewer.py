from wvcsc_bringup.qt_image_viewer import (
    PREFERRED_IMAGE_TOPICS,
    image_topic_names,
)


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
