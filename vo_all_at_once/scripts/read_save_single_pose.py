import rospy
import pandas as pd
import os
from datetime import datetime
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry

class Pose_msg_to_file():
    def __init__(self):
        rospy.init_node('gt_listener', anonymous=True)
        rospy.on_shutdown(self.save_groundtruth)

        pose_topic = rospy.get_param('~pose_topic')
        msg_type = rospy.get_param('~msg_type')

        self.ts = []
        self.tx = []
        self.ty = []
        self.tz = []
        self.qx = []
        self.qy = []
        self.qz = []
        self.qw = []

        if msg_type == 'PoseStamped':
            rospy.Subscriber(pose_topic, PoseStamped, self.pose_callback)
        # elif msg_type == 'TransformStamped':
        #     rospy.Subscriber(pose_topic, TransformStamped, gt_callback)
        elif msg_type == "Odometry":
            rospy.Subscriber(pose_topic, Odometry, self.pose_callback)
        
        
    def save_groundtruth(self):
        file_dir = rospy.get_param('~file_dir')
        file_name = rospy.get_param('~file_name')

        if rospy.get_param('~db_format') == "tum-rgbd":
            now = datetime.now()
            dt_string = now.strftime("%d-%m-%Y_%H-%M-%S")
            abspath = os.path.join(file_dir, dt_string)
            if not os.path.exists(abspath):
                os.makedirs(abspath)

            filepath = os.path.join(abspath, file_name)

            df = pd.DataFrame({'#timestamp':self.ts, 'tx':self.tx, 'ty':self.ty, 'tz':self.tz,
                             'qx':self.qx, 'qy':self.qy, 'qz':self.qz, 'qw':self.qw})
            df.to_csv(filepath, header = True, index = False, sep=' ',
                     columns = ['#timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])

    def pose_callback(self,data):
        timestamp = data.header.stamp.nsec
        position_msg = data.pose.position
        x, y, z = position_msg.x, position_msg.y, position_msg.z
        orientation_msg = data.pose.orientation
        qx, qy, qz, qw = orientation_msg.x, orientation_msg.y, orientation_msg.z, orientation_msg.w

        self.ts.append(timestamp)
        self.tx.append(x)
        self.ty.append(y)
        self.tz.append(z)
        self.qx.append(qx)
        self.qy.append(qy)
        self.qz.append(qz)
        self.qw.append(qw)


    


if __name__ == '__main__':
    pose_listener = Pose_msg_to_file()
    rospy.spin()