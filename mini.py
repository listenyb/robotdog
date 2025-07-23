import rospy
from nav_msgs.msg import Odometry

def odom_callback(msg):
    # 输出位置信息
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    rospy.loginfo("Position: x=%.2f y=%.2f z=%.2f", position.x, position.y, position.z)
    rospy.loginfo("Orientation: x=%.2f y=%.2f z=%.2f w=%.2f",
                  orientation.x, orientation.y, orientation.z, orientation.w)

def main():
    rospy.init_node('odom_listener', anonymous=True)
    rospy.Subscriber("/base_link/odom", Odometry, odom_callback)
    rospy.loginfo("Subscribed to /base_link/odom")
    rospy.spin()

if __name__ == '__main__':
    main()
