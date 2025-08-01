geometry_msgs/Pose target_pose
int32 gripper_action_type
float64 gripper_width  # [m]
float64 gripper_speed  # [m/s]
float64 gripper_force  # [N]

# Action types
int32 NONE = 0
int32 GRASP = 1
int32 HOMING = 2
int32 MOVE = 3