import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
import time

class TFListenerPy(Node):
    def __init__(self):
        super().__init__("tf_listener_py_node")
        # 创建buffer
        self.buffer = Buffer()
        # 创建transform_listener
        self.listener = TransformListener(self.buffer, self)
        
        # 参数：父帧、子帧（可通过参数修改）
        self.declare_parameter("parent_frame", "world")
        self.declare_parameter("child_frame", "child_frame")
        self.parent_frame = self.get_parameter("parent_frame").get_parameter_value().string_value
        self.child_frame = self.get_parameter("child_frame").get_parameter_value().string_value

        # 创建定时器
        self.timer = self.create_timer(0.5, self.on_timer)

        self.get_logger().info(f"tf_listener started, parent: '{self.parent_frame}', child: '{self.child_frame}'")

    def on_timer(self):
        try:
            # 使用最新的可用变换
            transform = self.buffer.lookup_transform(self.parent_frame, self.child_frame, rclpy.time.Time())
            
            t = transform.transform.translation
            r = transform.transform.rotation
            
            self.get_logger().info(
                f"Transform {transform.header.frame_id} -> {transform.child_frame_id}: "
                f"translation=({t.x:.3f}, {t.y:.3f}, {t.z:.3f}) "
                f"rotation=({r.x:.3f}, {r.y:.3f}, {r.z:.3f}, {r.w:.3f})"
            )
        except TransformException as ex:
            self.get_logger().warn(f"Could not get transform: {ex}", throttle_duration_sec=2)


def main(args=None):
    rclpy.init(args=args)
    node = TFListenerPy()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()