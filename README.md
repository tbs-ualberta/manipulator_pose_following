This package implements methods for teleoperation of manipulators (e.g., articulated arms) and utility interfaces to move the manipulator to a desired end-effector pose.

### Nodes:

#### `teleop`:
The main node facilitating teleoperation.
- *Subscribed topics*:
  - `teleop/delta_pose_rpy` - (manipulator_teleop/DeltaPoseRPY)  
    The pose derivative in Euler angles (RPY).
  - `teleop/pose` - (geometry_msgs/PoseStamped)  
    The current pose of the device used for teleoperation. The pose derivative is calculated internally.
  - `joint_states` - (sensor_msgs/JointState)  
    The current joint states of the manipulator.
- *Published topics*:
  - `joint_command` - (trajectory_msgs/JointTrajectory)  
    See http://wiki.ros.org/Industrial/Industrial_Robot_Driver_Spec
- *Services*:
  - `/teleop/start`  
    Enables the node to accept pose or pose derivative messages.
  - `/teleop/stop`  
    Sets the node to an idle state where pose or pose derivative messages are ignored.
- *Parameters*:
  - `rate` - (`int`, default: 100) [Hz]  
    The node's sampling rate
  - `group` - (`string`, default: 'manipulator')  
    The name of the manipulator group.
  - `w0` - (`double`, default: 0.1)  
    Factor determining the proximity to a singularity (see [Nakamura *et al.*](http://dynamicsystems.asmedigitalcollection.asme.org/article.aspx?articleid=1403812)).
  - `k0` - (`double`, default: 0.001)  
    Adjustment factor to inverse of Jacobian in proximity of a simgularity (see [Nakamura *et al.*](http://dynamicsystems.asmedigitalcollection.asme.org/article.aspx?articleid=1403812)).
  - `theta_d_lim` - (`double`, default: 3.14) [rad/s]  
    Joint velocity limit threshold. If exceeded, the node's state changes to idle.

#### `moveit_interface`:
Interface node for planning and executing trajectories using MoveIt!, and for moving the manipulator's end-effector to desired poses and pre-definded configurations (e.g., home position) from which to begin teleoperation.
- *Services*:
  - `/moveit_interface/move_home`  
    Moves the manipulator to the configuration where all joint angles are zero.
      - Parameters:
        - `max_vel_fact` (`double`): Velocity factor of the maximum velocity [0,1]
        - `max_acc_fact` (`double`): Acceleration factor of the maximum acceleration [0,1]
  - `/moveit_interface/move_to_rpy`  
    Moves the end-effector to a given pose where the orientation is given as Euler angles (RPY).
      - Parameters:
        - `pose_rpy` (`double[6]`): Desired end-effector pose as Euler angles (RPY).
        - `max_vel_fact` (`double`): Velocity factor of the maximum velocity [0,1]
        - `max_acc_fact` (`double`): Acceleration factor of the maximum acceleration [0,1]
  - `/moveit_interface/move_to_quat`  
    Moves the end-effector to a given pose where the orientation is given as quaternions.
      - Parameters:
        - `pose_rpy` (`double[7]`): Desired end-effector pose as quaternions.
        - `max_vel_fact` (`double`): Velocity factor of the maximum velocity [0,1]
        - `max_acc_fact` (`double`): Acceleration factor of the maximum acceleration [0,1]
- *Parameters*:
  - `rate` - (`int`, default: 10) [Hz]

#### `kb_jogging`:
Node for jogging the manipulator's end-effector pose via keyboard.
- *Button assignments*:
  - Translation: ±x: "1"/"q", ±y: "2"/"w", ±z: "3"/"e"
  - Rotation: ±x: "4"/"r", ±y: "5"/"t", ±z: "6"/"y"
  - Stop: "x"
- *Published topics*:
  - `teleop/delta_pose_rpy` - (manipulator_teleop/DeltaPoseRPY)  
    The pose derivative in Euler angles (RPY).
- *Services*:
  - `/keyboard_teleop/set_velocity`  
    Sets the jogging velocity factor where the maximum jogging velocity is set to 0.5 m/s.
    - Parameters:
      - `velocity` (`double`): Velocity factor [0,1]