import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Pose, PoseStamped
from .flutterComms import FlutterComms

class GoalPoser(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.previous_pose = PoseStamped()
        self.firstTick = True
        self.prevId = ""
        
        # Declare parameters
        self.declare_parameter('robot_id', '')
        self.declare_parameter('key_path', '')

        # Get parameters
        robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        key_path = self.get_parameter('key_path').get_parameter_value().string_value

        if not robot_id or not key_path:
            self.get_logger().error('robot_id and key_path must be provided as parameters.')
            return

        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        timer_period = 3.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.poseGetter = FlutterComms(robot_id, key_path)
        self.current_pose = Pose()

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == 'base_link':  # Extract pose for base_link frame
                self.current_pose.position.x = transform.transform.translation.x
                self.current_pose.position.y = transform.transform.translation.y
                self.current_pose.position.z = transform.transform.translation.z
                self.current_pose.orientation.x = transform.transform.rotation.x
                self.current_pose.orientation.y = transform.transform.rotation.y
                self.current_pose.orientation.z = transform.transform.rotation.z
                self.current_pose.orientation.w = transform.transform.rotation.w

                self.poseGetter.sendCurrentPose({
                    "position": {
                        "x": self.current_pose.position.x,
                        "y": self.current_pose.position.y,
                        "z": self.current_pose.position.z
                    },
                    "orientation": {
                        "x": self.current_pose.orientation.x,
                        "y": self.current_pose.orientation.y,
                        "z": self.current_pose.orientation.z,
                        "w": self.current_pose.orientation.w
                    }
                })
                #self.get_logger().info('Received current pose: "%s"' % transform)

    def timer_callback(self):
        msg = PoseStamped()
        flutterMsg = self.poseGetter.getGoalPose()
        
        msg.pose.position.x = flutterMsg["position"]["x"]
        msg.pose.position.y = flutterMsg["position"]["y"]
        msg.pose.position.z = flutterMsg["position"]["z"]
        msg.pose.orientation.x = flutterMsg["orientation"]["x"]
        msg.pose.orientation.y = flutterMsg["orientation"]["y"]
        msg.pose.orientation.z = flutterMsg["orientation"]["z"]
        msg.pose.orientation.w = flutterMsg["orientation"]["w"]

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'  # Set appropriate frame_id

        if flutterMsg["id"] == self.prevId:
            return
        
        if self.firstTick:
            self.prevId = flutterMsg["id"]
            self.firstTick = False
            return

        self.publisher_.publish(msg)
        self.prevId = flutterMsg["id"]
        
        #self.get_logger().info('Publishing goal pose: "%s"' % msg)
    
    def checkPoseFreshness(self, current_pose):
        if self.previous_pose.pose.position.x == current_pose.position.x and self.previous_pose.pose.position.y == current_pose.position.y and not self.firstTick:
            return False
        else:
            return True

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoser()
    if not node.get_parameter('robot_id').get_parameter_value().string_value or not node.get_parameter('key_path').get_parameter_value().string_value:
        rclpy.shutdown()
        return

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(sys.argv)