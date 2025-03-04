import rospy
from nav_msgs.msg import Odometry

def odom_callback(msg):
    position = msg.pose.pose.position
    x = position.x
    y = position.y
    z = position.z
    
    rospy.loginfo("position = (%f, %f, %f)" % (x, y, z))

def listener():
    rospy.init_node("get_position", anonymous=True)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.spin()

if __name__ == "__main__":
    listener()
