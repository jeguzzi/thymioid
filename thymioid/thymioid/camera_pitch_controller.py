import rclpy
import rclpy.node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import JointState
from typing import List, Any
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, FloatingPointRange

latching_qos = QoSProfile(depth=1,
                          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class PitchController(rclpy.node.Node):  # type: ignore

    def __init__(self) -> None:
        super(PitchController, self).__init__("camera_pitch_controller")
        pitch_range = FloatingPointRange(from_value=-0.34, to_value=1.3, step=0.01)
        pitch_desc = ParameterDescriptor(
            name='pitch', type=rclpy.Parameter.Type.DOUBLE.value, description='The camera pitch',
            floating_point_range=[pitch_range])
        pitch = self.declare_parameter('pitch', 0.0)
        joint = self.declare_parameter('joint', 'camera_body_support_joint')
        self.msg = JointState()
        self.msg.name = [joint.value]
        self.msg.position = [float(pitch.value)]
        self.clock = rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.ROS_TIME)
        # TODO(J): add  latch=True
        self.pub = self.create_publisher(JointState, "joint_states", latching_qos)
        self.add_on_set_parameters_callback(self.callback)

    def callback(self, params: List[rclpy.Parameter]) -> SetParametersResult:
        params = [p for p in params if p.name == 'pitch']
        if params:
            self.get_logger().info(f'Will set joint {self.msg.name} to angle {params[0].value}')
            self.msg.header.stamp = self.clock.now().to_msg()
            self.msg.position = [float(params[0].value)]
            self.pub.publish(self.msg)
        return SetParametersResult(successful=True)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    pitch_controller = PitchController()
    rclpy.spin(pitch_controller)
    pitch_controller.destroy_node()
    rclpy.shutdown()
