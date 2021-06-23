import os
import time

import numpy as np
import rospy

from hardware.robot import Robot
from utils.transforms import get_pose, rotate_pose_msg_by_euler_angles


class Calibration:
    def __init__(self):
        self.robot = Robot()

        homedir = os.path.join(os.path.expanduser('~'), "catkin_build_ws/src/baxter-pnp/grasp_common")
        self.move_completed = os.path.join(homedir, "move_completed.npy")
        self.tool_position = os.path.join(homedir, "tool_position.npy")
        
    def run(self):
        # Connect to robot
        self.robot.connect()

        # Move robot to home pose
        print('Moving to start position...')
        self.robot.go_home()

        # Calibrate gripper
        self.robot.calibrate_gripper()

        # Allow user to install the checker board
        self.robot.open_gripper()
        raw_input('Press enter to close gripper..')
        self.robot.close_gripper()
        raw_input('Press enter to continue..')

        # Move robot to each calibration point in workspace
        print('Collecting data...')
        while not rospy.is_shutdown():
            if not np.load(self.move_completed):
                tool_position = np.load(self.tool_position)
                print('Moving to tool position: ', tool_position)
                pose = get_pose(position=tool_position)
                pose_rot = rotate_pose_msg_by_euler_angles(pose, 0, np.pi/2, -np.pi/2)
                self.robot.move_to(pose_rot)
                np.save(self.move_completed, 1)
            else:
                time.sleep(0.1)
