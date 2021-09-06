# config_robot
ROS package for robot configuration

Added omniverse mecanum wheel robot model. urdf/lib/isaac_sim/soap_odom.usd

Only robot_event_cam.urdf.xacro is the omniverse mecanum wheel robt model
Generate the urdf:
rosrun xacro xacro -o robot_event_cam.urdf config_robot/urdf/robot_event_cam.urdf.xacro
