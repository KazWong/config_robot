# config_robot
ROS package store different robot description for omniverse isaac sim, gazebo,and real robot.

## Use of ROS
There are three main ROS workspaces in a robot.
1. ROS main distro workspace (/opt/ros/[distro])
2. Common workspace (general packages)
3. Project workspace (hardware depended / tasks)

| Distro Workspace        |      | Common Workspace        |      | Project Workspace  |
| -----------             |      | -----------             |      | -------            |
|                         | <->  | ghost                   | <->  | launcher           |
|                         |      |                         |      | config_robot       |
|                         |      |                         |      | config_env         |
|                         |      |                         |      |                    |
|                         |      | navigation              |      | tasks              |

ROS distro workspace is a workspace for original packages from the internet.  
Common workspace is a prebuild workspace which install to the robot.  
Project workspace is the development workspace for each project or robot.

Common workspace source distro workspace to build, project workspace source common workspace to build.

## Develop Notes

Added omniverse mecanum wheel robot model. urdf/lib/isaac_sim/soap_odom.usd

Only robot_event_cam.urdf.xacro is the omniverse mecanum wheel robt model
Generate the urdf:
rosrun xacro xacro -o robot_event_cam.urdf config_robot/urdf/robot_event_cam.urdf.xacro


Joint Note:

In Gazebo:  
Tag in <gazebo> is the joint contact coefficient:  
```
<joint name="joint2" type="continuous">
  ...
  <dynamics damping="0.7" friction="0.0"/>
</joint>
```

[Gazebosim joint](http://gazebosim.org/tutorials/?tut=ros_urdf#Joints)  
Notice the dynamics element with a viscous damping coefficient of 0.7 N*m*s/rad (Torque per angular velocity), damping is simply the amount of opposing force to any joint velocity (in this case torque per angular velocity) that is used to "slow" a moving joint towards rest.

The value of 0.7 N*m*s/rad was decided on by testing different amounts of damping and watching how "realistic" the swinging pendulum appeared. We encourage you to play with this value now (increase/decrease it) to get a feel for how it affects the physics engine.

[urdf tutorial](http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model)
friction - The physical static friction. For prismatic joints, the units are Newtons. For revolving joints, the units are Newton meters.

damping - The physical damping value. For prismatic joints, the units are Newton seconds per meter. For revolving joints, Newton meter secons per radian.


Tag in <gazebo> is the link contact coefficient:
```
<gazebo reference="<link name>">
  ...
  <kp>1000000.0</kp>
  <kd>1.0</kd>
</gazebo>
```
[ODE usergide](http://www.ode.org/ode-latest-userguide.html#sec_3_7_0)

In Omniverse:  

https://graphics.pixar.com/usd/docs/Rigid-Body-Physics-in-USD-Proposal.html#RigidBodyPhysicsinUSDProposal-Joints
The drive API allows joints to be motorized along degrees of freedom.  It may specify either a force or acceleration drive (The strength of force drives is impacted by the mass of the bodies attached to the joint, an acceleration drive is not).  It also has a target value to reach, and one can specify if the target is a goal position or velocity.  One can limit the maximum force the drive can apply, and one can specify a spring and damping coefficient.
The resulting drive force or acceleration is proportional to

stiffness × (targetPosition - p) + damping × (targetVelocity - v)

where p is the relative pose space motion of the joint (the axial rotation of a revolute joint, or axial translation for a prismatic joint) and v is the rate of change of this motion.

[Isaac Gym: High Performance GPU-Based Physics Simulation For Robot Learning](https://arxiv.org/pdf/2108.10470.pdf)  
Drive stiffness Positional error correction coefficient of a PD controller  
Drive damping Velocity error correction coefficient of a PD controller

```
float physics:damping = 0.0 (
  customData = {
    string apiName = "damping"
  }
  doc = """Damping of the drive. Units:
  if linear drive: mass/time
  If angular drive: mass*DIST_UNITS*DIST_UNITS/time/time/degrees."""
)

float physics:stiffness = 0.0 (
  customData = {
    string apiName = "stiffness"
  }
  doc = """Stiffness of the drive. Units:
  if linear drive: mass/time/time
  if angular drive: mass*DIST_UNITS*DIST_UNITS/degree/time/time."""
)
```
