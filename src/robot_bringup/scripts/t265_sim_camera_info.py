#!/usr/bin/env python3

import copy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def build_camera_info(
    frame_id: str,
    width: int,
    height: int,
    focal: list[float],
    principal: list[float],
    distortion: list[float],
) -> CameraInfo:
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = width
    msg.height = height
    msg.distortion_model = 'equidistant'
    msg.d = distortion
    msg.k = [
        focal[0], 0.0, principal[0],
        0.0, focal[1], principal[1],
        0.0, 0.0, 1.0,
    ]
    msg.r = [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    ]
    msg.p = [
        focal[0], 0.0, principal[0], 0.0,
        0.0, focal[1], principal[1], 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    return msg


class T265SimCameraInfoPublisher(Node):
    def __init__(self) -> None:
        super().__init__('t265_sim_camera_info')

        self.declare_parameter('left_image_topic', '/t265/fisheye1/image_raw')
        self.declare_parameter('right_image_topic', '/t265/fisheye2/image_raw')
        self.declare_parameter('left_camera_info_topic', '/t265/fisheye1/camera_info')
        self.declare_parameter('right_camera_info_topic', '/t265/fisheye2/camera_info')
        self.declare_parameter('left_frame_id', 't265_fisheye1_optical_frame')
        self.declare_parameter('right_frame_id', 't265_fisheye2_optical_frame')
        self.declare_parameter('left_width', 848)
        self.declare_parameter('left_height', 800)
        self.declare_parameter('right_width', 848)
        self.declare_parameter('right_height', 800)
        self.declare_parameter('left_focal', [284.10400390625, 285.1369934082031])
        self.declare_parameter('left_principal', [421.202392578125, 390.9504089355469])
        self.declare_parameter(
            'left_distortion',
            [-0.00517302006483078, 0.04260534048080444, -0.04044441133737564, 0.00740980077534914],
        )
        self.declare_parameter('right_focal', [285.5444030761719, 286.4206848144531])
        self.declare_parameter('right_principal', [423.2666015625, 389.993408203125])
        self.declare_parameter(
            'right_distortion',
            [-0.0028362609446048737, 0.03916018828749657, -0.03660506010055542, 0.0057794819585978985],
        )

        self._left_template = build_camera_info(
            frame_id=self.get_parameter('left_frame_id').value,
            width=self.get_parameter('left_width').value,
            height=self.get_parameter('left_height').value,
            focal=list(self.get_parameter('left_focal').value),
            principal=list(self.get_parameter('left_principal').value),
            distortion=list(self.get_parameter('left_distortion').value),
        )
        self._right_template = build_camera_info(
            frame_id=self.get_parameter('right_frame_id').value,
            width=self.get_parameter('right_width').value,
            height=self.get_parameter('right_height').value,
            focal=list(self.get_parameter('right_focal').value),
            principal=list(self.get_parameter('right_principal').value),
            distortion=list(self.get_parameter('right_distortion').value),
        )

        sensor_qos = rclpy.qos.qos_profile_sensor_data
        self._left_pub = self.create_publisher(
            CameraInfo, self.get_parameter('left_camera_info_topic').value, 10
        )
        self._right_pub = self.create_publisher(
            CameraInfo, self.get_parameter('right_camera_info_topic').value, 10
        )
        self.create_subscription(
            Image,
            self.get_parameter('left_image_topic').value,
            self._on_left_image,
            sensor_qos,
        )
        self.create_subscription(
            Image,
            self.get_parameter('right_image_topic').value,
            self._on_right_image,
            sensor_qos,
        )

        self.get_logger().info(
            'Publishing simulated T265 CameraInfo on %s and %s'
            % (
                self.get_parameter('left_camera_info_topic').value,
                self.get_parameter('right_camera_info_topic').value,
            )
        )

    def _publish_from_image(
        self,
        template: CameraInfo,
        publisher,
        image_msg: Image,
    ) -> None:
        info_msg = copy.deepcopy(template)
        info_msg.header = image_msg.header
        if not info_msg.header.frame_id:
            info_msg.header.frame_id = template.header.frame_id
        if image_msg.width > 0:
            info_msg.width = image_msg.width
        if image_msg.height > 0:
            info_msg.height = image_msg.height
        publisher.publish(info_msg)

    def _on_left_image(self, msg: Image) -> None:
        self._publish_from_image(self._left_template, self._left_pub, msg)

    def _on_right_image(self, msg: Image) -> None:
        self._publish_from_image(self._right_template, self._right_pub, msg)


def main() -> None:
    rclpy.init()
    node = T265SimCameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
