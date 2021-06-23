import os
import time

import numpy as np
import rospy

from yumipy import YuMiRobot, YuMiMotionPlanner
from autolab_core import RigidTransform
from scipy.spatial.transform import Rotation

# from hardware.robot import Robot
from utils.transforms import get_pose, rotate_pose_msg_by_euler_angles


class Calibration:
    def __init__(self):
        self.robot = YuMiRobot()

        homedir = os.path.join(os.path.expanduser('~'), "handover_ws/src/baxter-pnp/grasp_common")
        # homedir = os.path.join(os.path.expanduser('~'), "workplace/robotic-grasping/saved_calib")
        
        self.move_completed = os.path.join(homedir, "move_completed.npy")
        self.tool_position = os.path.join(homedir, "tool_position.npy")

    def rpy_to_wxyz(self, r, p, y):  # Change euler angle to quaternion
        rot = Rotation.from_euler('xyz', [r, p, y], degrees=True)
        return rot.as_quat()
        
    def run(self):

        # Move robot to home pose
        print('Moving to start position...')
        self.robot.reset_home()

        # Calibrate gripper
        self.robot.calibrate_grippers()

        right_side_rotation = self.rpy_to_wxyz(0, 0, 90)
        self.robot.right.goto_pose(RigidTransform(translation=[0.1 , -0.1 , 0.15 ], rotation = right_side_rotation, from_frame='gripper', to_frame='world'))
        # Allow user to install the checker board
        self.robot.right.open_gripper()
        raw_input('Press enter to close gripper..')
        self.robot.right.close_gripper()
        raw_input('Press enter to continue..')

              
        
        
        np.save(self.move_completed, 0)
        # Move robot to each calibration point in workspace
        print('Collecting data...')
        while not rospy.is_shutdown():
            if not np.load(self.move_completed):
                tool_position = np.load(self.tool_position)
                print('Moving to tool position: ', tool_position)
                # pose = get_pose(position=tool_position) ny
                # pose_rot = rotate_pose_msg_by_euler_angles(pose, 0, np.pi/2, -np.pi/2)
                
                # print('tool_position', tool_position, type(tool_position))

                self.robot.right.goto_pose(RigidTransform(translation=tool_position, rotation = right_side_rotation, from_frame='gripper', to_frame='world'))
                # self.robot.move_to(pose_rot)
                # print('robot move')
                # pose = self.robot.right.get_pose().as_frames('gripper', 'world')
                # for i in range(len(tool_position)):
                #     pose.translation[i] = tool_position[i]

                np.save(self.move_completed, 1)

            else:
                time.sleep(0.1)
