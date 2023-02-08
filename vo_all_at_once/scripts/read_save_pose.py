import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped


def save_groundtruth():
  print ("saving groundtruth")


def gt_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "Tartan cam pose %s", data.pose)
    
def listener():

    rospy.init_node('gt_listener', anonymous=True)
    rospy.on_shutdown(save_groundtruth)

    gt_topic = rospy.get_param('~pose_topic')

    rospy.Subscriber(gt_topic, PoseStamped, gt_callback)
    
    rospy.spin()

if __name__ == '__main__':
    listener()