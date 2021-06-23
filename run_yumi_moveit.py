#!/usr/bin/env python

from tasks.yumi_task_moveit import PickAndPlace
import moveit_commander
from geometry_msgs.msg import PoseStamped
import rospy
import sys


if __name__ == '__main__':

    try:
        rospy.init_node('yumi_robot')
        pnp = PickAndPlace(
        place_position_l=[0.3, 0.4, 0.15, 0],
        place_position_r=[0.3, -0.4, 0.15, 0],
        step_size=0.01
        )
        pnp.init(sys.argv)
        print('initialization finished')
        rospy.Subscriber('/hand_points', PoseStamped, pnp.run, queue_size = 1)
        rospy.spin()
        rospy.on_shutdown(pnp.end)
    except rospy.ROSInterruptException:
        pass

    
