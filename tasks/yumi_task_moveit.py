import copy
import os
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped

from yumipy import YuMiRobot, YuMiMotionPlanner
from autolab_core import RigidTransform
import moveit_commander

from utils.transforms import rotate_pose_msg_by_euler_angles, get_pose
from scipy.spatial.transform import Rotation
from math import pi


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


        homedir = os.path.join(os.path.expanduser('~'), "handover_ws/src/baxter-pnp/grasp_common")
        self.grasp_request = os.path.join(homedir, "grasp_request.npy")
        self.grasp_available = os.path.join(homedir, "grasp_available.npy")
        self.grasp_pose = os.path.join(homedir, "grasp_pose.npy")
        self.home_r = RigidTransform.load("/home/nmail/yumipy/src/yumipy/tools/paths/home_r.tf")
        self.home_l = RigidTransform.load("/home/nmail/yumipy/src/yumipy/tools/paths/home_l.tf")


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

        approach[2] = approach[2] + self._hover_distance

        if isinstance(pose[3], list):
            rot = pose[3]
        else:
            rot = self.rpy_to_wxyz(pose[3]*180/pi +90, 0, 180)

        check = arm.is_pose_reachable(RigidTransform(translation=approach[:3], rotation = rot, from_frame='gripper', to_frame='world'))
        
        print('check :', check)
        if not check:
            return False

        # arm.goto_pose(RigidTransform(translation=approach[:3], rotation = self.rpy_to_wxyz(pose[3]*180/3.14 +90, 0, 180), from_frame='gripper', to_frame='world'))
        arm.goto_pose_shortest_path(pose=RigidTransform(translation=approach[:3], rotation = rot, from_frame='gripper', to_frame='world'), plan_timeout=10, shorten=True)


        while approach[2] >= pose[2]:
            # approach[2] = approach[2] - self.step_size
            arm.goto_pose_delta((0, 0, -self.step_size))
            # arm.goto_pose(RigidTransform(translation=approach[:3], rotation = rot, from_frame='gripper', to_frame='world'))
            tmp = arm.get_pose().as_frames('gripper', 'world')
            for i in range(3):
                approach[i] = tmp.translation[i]

        return True

    def _retract(self, arm):
        """
        Retract up from current pose
        """
        # retrieve current pose from endpoint

        current_pose = arm.get_pose().as_frames('gripper', 'world')

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
        # arm.goto_pose_shortest_path(pose=pose, plan_timeout=5)

        check = arm.is_pose_reachable(current_pose)
        print('retract :', check)
        if not check:
            return False

        arm.goto_pose_delta((0, 0, self._hover_distance))
        

    def pick(self, grasp_pose , arm):
        """
        Pick from given pose
        """
        
        # open the gripper
        arm.open_gripper()
        # approach to the pose
        self._approach(grasp_pose , arm)
        
        # close gripper
        arm.close_gripper(force= 10, width=0.01)
        time.sleep(0.5)
        # retract to clear object
        self._retract(arm)

    def place(self, place_position , arm):
        """
        Place to given pose
        """
        
        # approach to the pose
        self._approach(place_position, arm)
        
        # open the gripper
        arm.open_gripper()

        # retract to clear object
        self._retract(arm)
        if arm == self.robot.right:
            arm.goto_pose_shortest_path(pose=self.home_r, plan_timeout=2, shorten=True)
        else:
            arm.goto_pose_shortest_path(pose=self.home_l, plan_timeout=2, shorten=True)
        self.robot.reset_home()


    def pose_check(self, pose):
        if  pose < 0:
            return self.robot.right
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
        arm.goto_pose_shortest_path(pose=pose, plan_timeout=5)

    def init(self, arg):
        moveit_commander.roscpp_initialize(arg)
        self.robot = YuMiRobot()
        self.robot.set_v(80)
        planner_r = YuMiMotionPlanner(arm='right', planner="TRRT") #TRRT, RRTstar, RRTConnect, KPIECE
        planner_l = YuMiMotionPlanner(arm='left', planner="TRRT")
        self.robot.right.set_motion_planner(planner_r)
        self.robot.left.set_motion_planner(planner_l)

        # Move robot to home pose
        print('Moving to start position...')

        self.robot.reset_home()
        self.robot.calibrate_grippers()
        self.robot.right.close_gripper()
        self.robot.left.close_gripper()

        # Initialize grasp request and grasp available
        np.save(self.grasp_request, 0)
        np.save(self.grasp_available, 0)


    def end(self):
        self.robot.right._motion_planner.end()
        self.robot.left._motion_planner.end()
        self.robot.stop()
        print "####################################     Program finished     ####################################"

    def run(self, msg):
        # Get the grasp pose
        np.save(self.grasp_request, 1)

        print('Waiting for grasp pose...')
        while not np.load(self.grasp_available) and not rospy.is_shutdown():
            time.sleep(0.1)

        # call grasp pose            
        grasp_pose = np.load(self.grasp_pose)
        np.save(self.grasp_available, 0)

        grasp_pose[2] = 0.01
        grasp_pose[0] += 0.005
        grasp_pose[1] -= 0.025
        
        # grasp_pose[0] -= 0.12
        # grasp_pose[1] -= 0.09

        arm = self.pose_check(grasp_pose[1])

        # Perform pick
        print('Picking from ', grasp_pose)
        self.pick(grasp_pose, arm)
        print('grasp_pose', grasp_pose)
        # raw_input('Press enter to continue..')

        # Perform place
        if msg != None:
            dest = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]]
            print('Placing to ', dest)
            self.place(dest, arm)
        
        elif arm == self.robot.right:
            print('Placing to ', self.place_position_r)
            self.place(self.place_position_r , arm)
            
        else:
            print('Placing to ', self.place_position_l)
            self.place(self.place_position_l , arm)

