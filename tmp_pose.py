import rospy
from geometry_msgs.msg import Pose

def publish_pose(ans, object_pub):
    out = Pose()
    out.position.x = 0.3
    if ans == 'l':
        out.position.y = 0.3
    else:
        out.position.y = -0.3
    out.position.z = 0.13
    out.orientation.x = 0
    out.orientation.y = -0.707
    out.orientation.z = 0.707
    out.orientation.w = 0
    object_pub.publish(out)



def main():
    object_pub = rospy.Publisher('/hand_points', Pose, queue_size = 10)
    rospy.init_node('hand_detect')
    rate =  rospy.Rate(2)
    while not rospy.is_shutdown():
        ans = raw_input('Press l or r: ')
        if ans == 'c':
            return
        publish_pose(ans, object_pub)
        rate.sleep()

if __name__ == '__main__':
    main()
