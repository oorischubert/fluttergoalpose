This node connects to the Astrom Hub, sending and receiving goalpose data. The Hive manager sends out goal poses to the robots and users can also view / send poses.

- Install requirments:
pip install firebase-admin

- Test goalposer node:
ros2 run fluttergoalpose goalposer --ros-args -p robot_id:=robotID -p key_path:=keyPath

- Check current_pose uplink:
ros2 topic pub /current_pose geometry_msgs/msg/Pose "{
  position: {x: $(python3 -c 'import random; print(random.uniform(-10.0, 10.0))'), 
             y: $(python3 -c 'import random; print(random.uniform(-10.0, 10.0))'), 
             z: $(python3 -c 'import random; print(random.uniform(-10.0, 10.0))')}, 
  orientation: {x: $(python3 -c 'import random; print(random.uniform(-1.0, 1.0))'), 
                y: $(python3 -c 'import random; print(random.uniform(-1.0, 1.0))'), 
                z: $(python3 -c 'import random; print(random.uniform(-1.0, 1.0))'), 
                w: $(python3 -c 'import random; print(random.uniform(-1.0, 1.0))')}
}"

- Check goal_pose downlink:
ros2 topic echo /goal_pose