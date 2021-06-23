import copy
import os
import time

import numpy as np
import rospy
from geometry_msgs.msg import Pose

from yumipy import YuMiRobot, YuMiMotionPlanner
from autolab_core import RigidTransform

# from hardware.robot import Robot
from utils.transforms import rotate_pose_msg_by_euler_angles, get_pose
from scipy.spatial.transform import Rotation

class PickAndPlace:
    def __init__(
            self,
            place_position_l,
            place_position_r,
            force_threshold=-8,
            hover_distance=0.07,
            step_size=0.05,
    ):
        """

        @param place_position: Place position as [x, y, z]
        @param force_threshold: Z force threshold in Newtons
        @param hover_distance: Distance above the pose in meters
        @param step_size: Step size for approaching the pose
        """
        self.place_position_l = place_position_l
        self.place_position_r = place_position_r
        # self.force_threshold = force_threshold
        self._hover_distance = hover_distance
        self.step_size = step_size

        # self.first = True
        # self.ref = [0.0] * 3
        # self.right_point = [0.4, -0.22, 0.09]

        homedir = os.path.join(os.path.expanduser('~'), "handover_ws/src/baxter-pnp/grasp_common")
        self.grasp_request = os.path.join(homedir, "grasp_request.npy")
        self.grasp_available = os.path.join(homedir, "grasp_available.npy")
        self.grasp_pose = os.path.join(homedir, "grasp_pose.npy")


    def rpy_to_wxyz(self, r, p, y):  # Change euler angle to quaternion
        rot = Rotation.from_euler('xyz', [r, p, y], degrees=True)
        return rot.as_quat()

    def _approach(self, pose, arm):
        """
        Move to a pose with a hover-distance above the requested pose and
        then move to the pose incrementally while monitoring the z force
        """
        print('approaching...')
        approach = copy.deepcopy(pose)

        # approach.position.z = approach.position.z + self._hover_distance
        approach[2] = approach[2] + self._hover_distance
        


        # self.robot.move_to(approach)

        check =arm.is_pose_reachable(RigidTransform(translation=pose[:3], rotation = self.rpy_to_wxyz(pose[3]*180/3.14 +90, 0, 180)))
        
        print('check :', check)

        arm.goto_pose(RigidTransform(translation=pose[:3], rotation = self.rpy_to_wxyz(pose[3]*180/3.14 +90, 0, 180)))
        # arm.goto_pose_shortest_path(pose=approach, plan_timeout=10)


        while approach[2] >= pose[2]:
            approach[2] = approach[2] - self.step_size
            arm.goto_pose(RigidTransform(translation=pose[:3], rotation = self.rpy_to_wxyz(pose[3]*180/3.14 + 90, 0, 180)))
        
            # arm.goto_pose_shortest_path(pose=approach, plan_timeout=10)




        # while approach.position.z >= pose.position.z:
        #     approach.position.z = approach.position.z - self.step_size
        #     arm.goto_pose_shortest_path(pose=approach, plan_timeout=10)
            #try use 

            # force = self.robot.get_force()
            # print('force: ', self.robot.get_force())

            # if force.z < self.force_threshold:
            #     print(("End Effector Force is: " + str([force.x, force.y, force.z])))
            #     print("Max z force reached before reaching the pose")
            #     break

    def _retract(self, arm):
        """
        Retract up from current pose
        """
        # retrieve current pose from endpoint

        current_pose = arm.get_pose()

        # pose = Pose()
        # pose.position.x = current_pose['position'].x
        # pose.position.y = current_pose['position'].y
        # pose.position.z = current_pose['position'].z + self._hover_distance
        current_pose.translation[2] = current_pose.translation[2] + self._hover_distance
        # pose.orientation.x = current_pose['orientation'].x
        # pose.orientation.y = current_pose['orientation'].y
        # pose.orientation.z = current_pose['orientation'].z
        # pose.orientation.w = current_pose['orientation'].w


        # servo up from current pose
        # arm.goto_pose_shortest_path(pose=pose, plan_timeout=10)

        check =arm.is_pose_reachable(current_pose)
        print('retract :', check)

        arm.goto_pose(current_pose)
        

    def pick(self, grasp_pose , arm):
        """
        Pick from given pose
        """
        # Calculate grasp pose
        # pose = get_pose(position=grasp_pose[:3])

        # Apply grasp angle from model output
        # pose = rotate_pose_msg_by_euler_angles(pose, 0.0, 0.0, grasp_pose[3])

        # open the gripper
        arm.open_gripper()
        # approach to the pose
        # self._approach(pose , arm)
        self._approach(grasp_pose , arm)
        
        # close gripper
        arm.close_gripper(force= 10, width=0.01)
        # retract to clear object
        self._retract(arm)

    def place(self, place_position , arm):
        """
        Place to given pose
        """
        # Calculate pose from place position
        # pose = get_pose(position=place_position)

        # approach to the pose
        # self._approach(pose, arm)
        self._approach(place_position, arm)
        

        # open the gripper
        arm.open_gripper()
        # Get the next grasp pose
        np.save(self.grasp_request, 1)
        # retract to clear object
        self._retract(arm)

    # def pose_check(msg):
    #     global ref
    #     global first
    #     if first:
    #         first = False
    #         ref[0] = right_point[0] - msg.poses[0].position.x
    #         ref[1] = right_point[1] - msg.poses[0].position.y
    #         ref[2] = right_point[2] - msg.poses[0].position.z
    #         print(ref)
    #         return
    #     if msg.poses[0].position.y < 0:
    #         return self.robot.right
    #         # right = True
    #     else:
    #         return self.robot.left
            # right = False
        # set_scene(arm._motion_planner, msg.poses)
        # pick_and_put(arm, arm._motion_planner, msg.poses[0].position, right)
        # goto_pose_delta(arm, msg)

    def pose_check(self, pose):
        # global ref
        # global first
        # if self.first:
        #     self.first = False
        #     self.ref[0] = self.right_point[0] - msg.poses[0].position.x
        #     self.ref[1] = self.right_point[1] - msg.poses[0].position.y
        #     self.ref[2] = self.right_point[2] - msg.poses[0].position.z
        #     print(ref)
        #     return
        if  pose < 0:
            return self.robot.right
            # right = True
        else:
            return self.robot.left

    def set_scene(planner, object_list):
        planner.remove_all_object(True)
        for i in range(len(object_list)):
            pose = [object_list[i].position.x, object_list[i].position.y, 0.045]
            planner.add_object(pose, size, "stuff"+str(i))

    def goto_pose_delta(arm, position):
        pose = arm.get_pose().as_frames('gripper', 'world')
        for i in range(3):
            pose.translation[i] += position[i]
        arm.goto_pose_shortest_path(pose=pose, plan_timeout=10)


    def run(self):

        rospy.init_node('yumi_robot')
        self.robot = YuMiRobot()
        self.robot.set_v(1000)

        # Calibrate gripper
        self.robot.calibrate_grippers()

        
        # planner_r = YuMiMotionPlanner(arm='right', planner="RRTConnect")
        # planner_l = YuMiMotionPlanner(arm='left', planner="RRTConnect")
        # self.robot.right.set_motion_planner(planner_r)
        # self.robot.left.set_motion_planner(planner_l)

        # Initialize grasp request and grasp available
        np.save(self.grasp_request, 0)
        np.save(self.grasp_available, 0)




        # Move robot to home pose
        print('Moving to start position...')
        self.robot.reset_home()
        # self.robot.open_grippers()
        self.robot.right.open_gripper()
        self.robot.left.open_gripper()

        self.robot.right.close_gripper(force= 10, width=0.01)
        self.robot.left.close_gripper(force= 10, width=0.01)

        # Get the first grasp pose
        np.save(self.grasp_request, 1)

        print('finish')
        while not rospy.is_shutdown():
            print('Waiting for grasp pose...')
            while not np.load(self.grasp_available) and not rospy.is_shutdown():
                time.sleep(0.1)

            # call grasp pose            
            grasp_pose = np.load(self.grasp_pose)
            np.save(self.grasp_available, 0)

            #check right and left
            # grasp_pose[2] = 0.93 - grasp_pose[2]
            
            grasp_pose[2] = 0.05
            # print(33, grasp_pose)
            arm = self.pose_check(grasp_pose[1])

            # Perform pick
            print('Picking from ', grasp_pose)
            self.pick(grasp_pose, arm)

            # Perform place
            # print('Placing to ', self.place_position)
            if arm == self.robot.right:
                print('Placing to ', self.place_position_r)
                self.place(self.place_position_r , arm)
            else:
                print('Placing to ', self.place_position_l)
                self.place(self.place_position_l , arm)