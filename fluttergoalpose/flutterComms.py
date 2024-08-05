import firebase_admin
from firebase_admin import credentials, firestore
import json

class FlutterComms():
    def __init__(self, robot_id, key_path):
        # Use a service account
        cred = credentials.Certificate(key_path)
        firebase_admin.initialize_app(cred)
        
        self.db = firestore.client()
        self.doc_ref = self.db.collection('robots').document(robot_id)
    
    def getGoalPose(self):
        doc = self.doc_ref.get()
        if doc.exists:
            goal_pose = doc.to_dict().get('goal_pose', {})
            return {
                "position": {
                    "x": float(goal_pose['position']['x']),
                    "y": float(goal_pose['position']['y']),
                    "z": float(goal_pose['position']['z'])
                },
                "orientation": {
                    "x": float(goal_pose['orientation']['x']),
                    "y": float(goal_pose['orientation']['y']),
                    "z": float(goal_pose['orientation']['z']),
                    "w": float(goal_pose['orientation']['w'])
                },
                "active": bool(goal_pose['active'])
            }
        else:
            print("No such document!")
            return None
    
    def sendCurrentPose(self, pose):
        current_pose_data = {
            "current_pose": {
                "position": {
                    "x": str(pose['position']['x']),
                    "y": str(pose['position']['y']),
                    "z": str(pose['position']['z'])
                },
                "orientation": {
                    "x": str(pose['orientation']['x']),
                    "y": str(pose['orientation']['y']),
                    "z": str(pose['orientation']['z']),
                    "w": str(pose['orientation']['w'])
                }
            }
        }
        self.doc_ref.set(current_pose_data, merge=True)
        return "[goalposer] Current pose updated"

    def sendGoalPose(self, pose):
        goal_pose_data = {
            "goal_pose": {
                "position": {
                    "x": str(pose['position']['x']),
                    "y": str(pose['position']['y']),
                    "z": str(pose['position']['z'])
                },
                "orientation": {
                    "x": str(pose['orientation']['x']),
                    "y": str(pose['orientation']['y']),
                    "z": str(pose['orientation']['z']),
                    "w": str(pose['orientation']['w'])
                }
            }
        }
        self.doc_ref.set(goal_pose_data, merge=True)
        return "[goalposer] Goal pose updated"

if __name__ == '__main__':
    import sys
    # python3 flutterComms.py artro1212 /Users/oorischubert/turtlebot_ros2/astrom-f99b0-firebase-adminsdk-yjjdd-a50a8d1a33.json
    # Ensure you pass the robot_id as the first argument when running this script
    if len(sys.argv) != 3:
        print("[goalposer] Error: robot_id and key_path not provided. Script will not run.")
        sys.exit(1)

    robot_id = sys.argv[1]
    key_path = sys.argv[2]
    comms = FlutterComms(robot_id, key_path)

    # Test getting the goal pose
    try:
        goal_pose = comms.getGoalPose()
        print("Goal Pose:")
        print(goal_pose)
    except Exception as e:
        print(f"[goalposer] Failed to get goal pose: {e}")

    # Test sending the current pose
    current_pose = {
        "position": {
            "x": 1.0,
            "y": 2.0,
            "z": 3.0
        },
        "orientation": {
            "x": 0.0,
            "y": 1.1232,
            "z": 0.0,
            "w": 1.0
        }
    }
    try:
        response = comms.sendCurrentPose(current_pose)
        print("[goalposer] Response from sending current pose:")
        print(response)
    except Exception as e:
        print(f"[goalposer] Failed to send current pose: {e}")