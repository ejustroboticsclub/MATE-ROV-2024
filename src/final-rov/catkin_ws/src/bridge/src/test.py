import rospy
from std_msgs.msg import Empty
import multiprocessing

flag = True

def callback(msg):
    rospy.loginfo('flag is False')
    global flag
    flag = False


rospy.init_node('subscriber_node', anonymous=True)

rospy.Subscriber('test', Empty, callback)
rospy.loginfo('flag is True')


def f():
    global flag
    while flag:
        rospy.loginfo('flag is True')

def spin():
    rospy.spin()

p = multiprocessing.Process(target=f)
p2 = multiprocessing.Process(target=spin)
p.start()
p2.start()
p.join()
p2.join()


