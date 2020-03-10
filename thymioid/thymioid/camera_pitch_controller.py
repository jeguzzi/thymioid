import rclpy
import rclpy.node
from sensor_msgs.msg import JointState
from typing import List, Any
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, FloatingPointRange
# def update(pub, msg):
#     def f(event):
#         msg.header.stamp = rospy.Time.now()
#         pub.publish(msg)
#     return f


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
        self.msg.position = [pitch.value]
        self.clock = rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.ROS_TIME)
        # TODO(J): add  latch=True
        self.pub = self.create_publisher(JointState, "joint_states", 1)
        self.set_parameters_callback(self.callback)
        # TODO(J): should I still use a timer?
        # rospy.Timer(rospy.Duration(10), update(pub, msg))

    def callback(self, params: List[rclpy.Parameter]) -> SetParametersResult:
        params = [p for p in params if p.name == 'pitch']
        if params:
            self.msg.header.stamp = self.clock.now().to_msg()
            self.msg.position = [params[0].value]
            self.pub.publish(self.msg)
        return SetParametersResult(successful=True)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    pitch_controller = PitchController()
    rclpy.spin(pitch_controller)
    pitch_controller.destroy_node()
    rclpy.shutdown()
