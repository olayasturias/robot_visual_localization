import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry


def save_groundtruth():
  print ("saving groundtruth")

def gt_PS_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "pose %s", data.pose)
def gt_Odom_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "pose %s", data.pose)
    
def listener():

    rospy.init_node('gt_listener', anonymous=True)
    rospy.on_shutdown(save_groundtruth)

    gt_topic = rospy.get_param('~pose_topic')
    msg_type = rospy.get_param('~msg_type')
    if msg_type == 'PoseStamped':
        rospy.Subscriber(gt_topic, PoseStamped, gt_PS_callback)
    # elif msg_type == 'TransformStamped':
    #     rospy.Subscriber(gt_topic, TransformStamped, gt_callback)
    elif msg_type == "Odometry":
        rospy.Subscriber(gt_topic, Odometry, gt_Odom_callback)
    
    rospy.spin()

if __name__ == '__main__':
    listener()