import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from .flutterComms import FlutterComms

class GoalPublisher(Node):
    def __init__(self, robot_id):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(Pose, 'goal_pose', 10)
        self.subscription = self.create_subscription(
            Pose,
            'current_pose',
            self.current_pose_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        timer_period = 3.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.poseGetter = FlutterComms(robot_id)
        self.current_pose = Pose()

    def current_pose_callback(self, msg):
        self.current_pose = msg
        self.poseGetter.sendCurrentPose({
            "position": {
                "x": msg.position.x,
                "y": msg.position.y,
                "z": msg.position.z
            },
            "orientation": {
                "x": msg.orientation.x,
                "y": msg.orientation.y,
                "z": msg.orientation.z,
                "w": msg.orientation.w
            }
        })
        self.get_logger().info('Received current pose: "%s"' % msg)

    def timer_callback(self):
        msg = Pose()
        flutterMsg = self.poseGetter.getGoalPose()
        # Set your goal pose here
        msg.position.x = flutterMsg["position"]["x"]
        msg.position.y = flutterMsg["position"]["y"]
        msg.position.z = flutterMsg["position"]["z"]
        msg.orientation.x = flutterMsg["orientation"]["x"]
        msg.orientation.y = flutterMsg["orientation"]["y"]
        msg.orientation.z = flutterMsg["orientation"]["z"]
        msg.orientation.w = flutterMsg["orientation"]["w"]

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing goal pose: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    if args and len(args) > 1:
        robot_id = args[1]
    else:
        print("[goalposer] Error: robot_id not provided. Node will not start.")
        rclpy.shutdown()
        return

    node = GoalPublisher(robot_id)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(sys.argv)