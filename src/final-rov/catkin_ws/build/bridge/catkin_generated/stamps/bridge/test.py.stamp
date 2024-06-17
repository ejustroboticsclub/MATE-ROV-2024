import rospy
from std_msgs.msg import Empty

flag = True

def callback(msg):
    flag = False


rospy.init_node('subscriber_node', anonymous=True)

rospy.Subscriber('test', Empty, callback)

rospy.spin()

while flag:
    rospy.loginfo('flag is True')
