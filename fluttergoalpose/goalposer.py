import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from .flutterComms import FlutterComms

class GoalPoser(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        
        # Declare parameters
        self.declare_parameter('robot_id', '')
        self.declare_parameter('key_path', '')

        # Get parameters
        robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        key_path = self.get_parameter('key_path').get_parameter_value().string_value

        if not robot_id or not key_path:
            self.get_logger().error('robot_id and key_path must be provided as parameters.')
            return

        self.publisher_ = self.create_publisher(Pose, 'goal_pose', 10)
        self.subscription = self.create_subscription(
            Pose,
            'current_pose',
            self.current_pose_callback,
            10)
        self.subscription  # prevent unused variable warning

        timer_period = 3.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.poseGetter = FlutterComms(robot_id, key_path)
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
        #self.get_logger().info('Received current pose: "%s"' % msg)

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
        #self.get_logger().info('Publishing goal pose: "%s"' % msg)

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

#Testing:

# ros2 run fluttergoalpose goalposer --ros-args -p robot_id:=artro1212 -p key_path:=/root/src/astrom-f99b0-firebase-adminsdk-yjjdd-a50a8d1a33.json

# ros2 topic echo /goal_pose

# ros2 topic pub /current_pose geometry_msgs/msg/Pose "{
#   position: {x: $(python3 -c 'import random; print(random.uniform(-10.0, 10.0))'), 
#              y: $(python3 -c 'import random; print(random.uniform(-10.0, 10.0))'), 
#              z: $(python3 -c 'import random; print(random.uniform(-10.0, 10.0))')}, 
#   orientation: {x: $(python3 -c 'import random; print(random.uniform(-1.0, 1.0))'), 
#                 y: $(python3 -c 'import random; print(random.uniform(-1.0, 1.0))'), 
#                 z: $(python3 -c 'import random; print(random.uniform(-1.0, 1.0))'), 
#                 w: $(python3 -c 'import random; print(random.uniform(-1.0, 1.0))')}
# }"