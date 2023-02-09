import rospy
import pandas as pd
import os
from datetime import datetime
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from evo.tools import file_interface
from evo.core import metrics, sync
from evo.core.metrics import PoseRelation

class Pose_msg_to_file():
    def __init__(self):
        rospy.init_node('gt_listener', anonymous=True)
        rospy.on_shutdown(self.save_groundtruth)

        gt_pose_topic = rospy.get_param('~gt_pose_topic')
        gt_msg_type = rospy.get_param('~gt_msg_type')

        orb_pose_topic = rospy.get_param('~orb_pose_topic')
        orb_msg_type = rospy.get_param('~orb_msg_type')

        tartan_pose_topic = rospy.get_param('~tartan_pose_topic')
        tartan_msg_type = rospy.get_param('~tartan_msg_type')

        self.gt_ts = [], self.gt_tx = [], self.gt_ty = [], self.gt_tz = []
        self.gt_qx = [], self.gt_qy = [], self.gt_qz = [], self.gt_qw = []

        self.orb_ts = [], self.orb_tx = [], self.orb_ty = [], self.orb_tz = []
        self.orb_qx = [], self.orb_qy = [], self.orb_qz = [], self.orb_qw = []

        self.tartan_ts = [], self.tartan_tx = [], self.tartan_ty = [], self.tartan_tz = []
        self.tartan_qx = [], self.tartan_qy = [], self.tartan_qz = [], self.tartan_qw = []

        if gt_msg_type == 'PoseStamped':
            rospy.Subscriber(gt_pose_topic, PoseStamped, self.gt_callback)
        elif gt_msg_type == "Odometry":
            rospy.Subscriber(gt_pose_topic, Odometry, self.gt_callback)

        if orb_msg_type == 'PoseStamped':
            rospy.Subscriber(orb_pose_topic, PoseStamped, self.orb_callback)
        elif orb_msg_type == "Odometry":
            rospy.Subscriber(orb_pose_topic, Odometry, self.orb_callback)
        
        if tartan_msg_type == 'PoseStamped':
            rospy.Subscriber(tartan_pose_topic, PoseStamped, self.tartan_callback)
        elif tartan_msg_type == "Odometry":
            rospy.Subscriber(tartan_pose_topic, Odometry, self.tartan_callback)

    def gt_callback(self,data): 
        self.read_pose(data, self.gt_ts, self.gt_tx, self.gt_ty, self.gt_tz,
                             self.gt_qx, self.gt_qy, self.gt_qz, self.gt_qw)
    def orb_callback(self,data): 
        self.read_pose(data, self.orb_ts, self.orb_tx, self.orb_ty, self.orb_tz,
                             self.orb_qx, self.orb_qy, self.orb_qz, self.orb_qw)
    def tartan_callback(self,data): 
        self.read_pose(data, self.tartan_ts, self.tartan_tx, self.tartan_ty, self.tartan_tz,
                             self.tartan_qx, self.tartan_qy, self.tartan_qz, self.tartan_qw)

    def read_pose(self,data, ts_list, tx_list, ty_list, tz_list, 
                        qx_list, qy_list, qz_list, qw_list):
        timestamp = data.header.stamp.nsec
        position_msg = data.pose.position
        x, y, z = position_msg.x, position_msg.y, position_msg.z
        orientation_msg = data.pose.orientation
        qx, qy, qz, qw = orientation_msg.x, orientation_msg.y, orientation_msg.z, orientation_msg.w

        ts_list.append(timestamp)
        tx_list.append(x)
        ty_list.append(y)
        tz_list.append(z)
        qx_list.append(qx)
        qy_list.append(qy)
        qz_list.append(qz)
        qw_list.append(qw)
       
    def save_pose_tum(self,filepath, ts, tx, ty, tz, qx, qy, qz, qw,):
        df = pd.DataFrame({'#timestamp':ts, 'tx':tx, 'ty':ty, 'tz':tz, 
                           'qx':qx, 'qy':qy, 'qz':qz, 'qw':qw})
        df.to_csv(filepath, header = True, index = False, sep=' ',
                     columns = ['#timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])
    
    def evo_evaluate(self, groundtruth, orb, tartan, outfile):
        check_opts_ape = {"align_origin": True,"align": False, "correct_scale": True, "show_plot": True}
        check_opts_rpe = {"align_origin": True,"align": False, "correct_scale": True, "all_pairs": True, "show_plot": True}
        
        result_dict = {}

        evo_gt     = file_interface.read_tum_trajectory_file(groundtruth)
        evo_orb    = file_interface.read_tum_trajectory_file(orb)
        evo_tartan = file_interface.read_tum_trajectory_file(tartan)

        result_dict.update(self.create_evo_dict('orb_slam3', evo_gt, evo_orb))
        result_dict.update(self.create_evo_dict('tartan', evo_gt, evo_tartan))

        print(result_dict)
        with open(outfile,'w+') as data: 
            data.write(str(result_dict))


    def create_evo_dict(self, algorithm_name, gt, estimate):
        result_dict = {}
        gt, estimate = sync.associate_trajectories(gt, estimate)
        estimate.align(gt, correct_scale=True, correct_only_scale=False)
        # APE translation metrics for orb
        pose_relation = metrics.PoseRelation.translation_part
        ape_metric = metrics.APE(pose_relation)
        ape_metric.process_data(gt, estimate)
        ape_stats = ape_metric.get_all_statistics()
        result_dict.update({(algorithm_name,'ape','translation'):ape_stats})
        # APE rotation metrics for orb
        pose_relation = metrics.PoseRelation.rotation_angle_rad
        ape_metric = metrics.APE(pose_relation)
        ape_metric.process_data(gt, estimate)
        ape_stats = ape_metric.get_all_statistics()
        result_dict.update({(algorithm_name,'ape','rotation'):ape_stats})

        # RPE translation metrics for orb
        pose_relation = metrics.PoseRelation.translation_part
        rpe_metric = metrics.RPE(pose_relation)
        rpe_metric.process_data(gt, estimate)
        rpe_stats = rpe_metric.get_all_statistics()
        result_dict.update({(algorithm_name,'rpe','translation'):rpe_stats})
        # RPE rotation metrics for orb
        pose_relation = metrics.PoseRelation.rotation_angle_rad
        rpe_metric = metrics.RPE(pose_relation)
        rpe_metric.process_data(gt, estimate)
        rpe_stats = rpe_metric.get_all_statistics()
        result_dict.update({(algorithm_name,'rpe','rotation'):rpe_stats})

        return result_dict

   
    def save_groundtruth(self):
        file_dir = rospy.get_param('~file_dir')

        if rospy.get_param('~db_format') == "tum-rgbd":
            now = datetime.now()
            dt_string = now.strftime("%d-%m-%Y_%H-%M-%S")
            abspath = os.path.join(file_dir, dt_string)
            if not os.path.exists(abspath):
                os.makedirs(abspath)

            gt_filepath = os.path.join(abspath, "groundtruth.txt")
            orb_filepath = os.path.join(abspath, "orb.txt")
            tartan_filepath = os.path.join(abspath, "tartan.txt")

            self.save_pose_tum(gt_filepath, self.gt_ts, self.gt_tx, self.gt_ty, self.gt_tz,
                                            self.gt_qx, self.gt_qy, self.gt_qz, self.gt_qw)
            self.save_pose_tum(orb_filepath, self.orb_ts, self.orb_tx, self.orb_ty, self.orb_tz,
                                            self.orb_qx, self.orb_qy, self.orb_qz, self.orb_qw)
            self.save_pose_tum(tartan_filepath, self.tartan_ts, self.tartan_tx, self.tartan_ty, self.tartan_tz,
                                            self.tartan_qx, self.tartan_qy, self.tartan_qz, self.tartan_qw)
            self.evo_evaluate(gt_filepath, orb_filepath, tartan_filepath, os.path.join(abspath, "evaluation.txt"))
        else:
            print("format other than TUM not implemented yet")


if __name__ == '__main__':
    pose_listener = Pose_msg_to_file()
    rospy.spin()