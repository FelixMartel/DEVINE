''' IRL-1 Constant: controllers name with joints name and joints limites'''

ROBOT_NAME = 'jn0'
ROBOT_CONTROLLER = {
    'neck_controller':  {
        'joints_name': ['neck_pan_joint', 'neck_tilt_joint'],
        'joints_limit' : [[-1.57, 1.57], [-0.17, 0.79]]
    },
    'left_arm_controller': {
        'joints_name': ['L_shoulder_pan_joint', 'L_shoulder_tilt_joint', 'L_shoulder_roll_joint', 'L_elbow_tilt_joint'],
        'joints_limit' : [[-1.57, 1.57], [-3.14, 1.57], [-1.57, 1.57], [-0.75, 1.22]]
    },
    'right_arm_controller': {
        'joints_name': ['R_shoulder_pan_joint', 'R_shoulder_tilt_joint', 'R_shoulder_roll_joint', 'R_elbow_tilt_joint'],
        'joints_limit' : [[-1.57, 1.57], [-3.14, 1.57], [-1.57, 1.57], [-0.75, 1.22]]
    },
    'left_gripper_up_controller': {
        'joints_limit': [-1.7, 0.4] # 0.4 is closed
    },
    'left_gripper_down_controller': {
        'joints_limit': [-1.7, 0.4] # 0.4 is closed
    },
    'right_gripper_down_controller': {
        'joints_limit': [-1.7, 0.4] # 0.4 is closed
    },
    'right_gripper_up_controller': {
        'joints_limit': [-1.7, 0.4] # 0.4 is closed
    }
}
