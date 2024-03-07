# robot_visual_localization
ROS packages for robot camera-based localization


![](https://raw.githubusercontent.com/olayasturias/robot_visual_localization/ros1/media/diagram.png?token=GHSAT0AAAAAAB5OL65OODOZX7K7ZTNRBOQKY7TM2ZA)

Clone this repository with its submodules:

```bash
git clone --recurse-submodules -j8 https://github.com/olayasturias/robot_visual_localization.git
```
If you forgot to clone the submodules, you can download them afterwards as

```bash
git submodule update --init --recursive
```

- [DSO](https://github.com/JakobEngel/dso_ros)
>Direct Sparse Odometry, J. Engel, V. Koltun, D. Cremers, In arXiv:1607.02565, 2016
>A Photometrically Calibrated Benchmark For Monocular Visual Odometry, J. Engel, V. Usenko, D. Cremers, In arXiv:1607.02555, 2016

<!-- - [Kimera-VIO-ROS](https://github.com/MIT-SPARK/Kimera-VIO-ROS)
> A. Rosinol, M. Abate, Y. Chang, L. Carlone. Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping. arXiv preprint arXiv:1910.02490.

kimera-rpgo
fix with PR as git checkout fix/gtsam_boost_removal
mesh_rviz_plugins to compile with c++14
modified function declarations of gtsam (fork?)
modified kimera_VIO to include boost. fork?
Undistort.h #include <boost/optional.hpp>
Featuredetector.h #include <boost/optional.hpp>
Replace boost::make_shared with std::make_shared
replace cam1_R_cam2.quaternion()[0]) with cam1_R_cam2.Quaternion()[0]) -->
- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
> Campos C, Elvira R, Rodríguez JJ, Montiel JM, Tardós JD. Orb-slam3: An accurate open-source library for visual, visual–inertial, and multimap slam. IEEE Transactions on Robotics. 2021 May 25;37(6):1874-90.

<!-- - [PTAM](https://github.com/ethz-asl/ethzasl_ptam)
> [1] Georg Klein and David Murray, "Parallel Tracking and Mapping for Small AR Workspaces", Proc. ISMAR 2007
> [2] Georg Klein and David Murray, "Improving the Agility of Keyframe-based SLAM", Proc. ECCV 2008 -->

<!-- - [SVO](https://github.com/uzh-rpg/rpg_svo)
> Christian Forster, Matia Pizzoli, Davide Scaramuzza, "SVO: Fast Semi-direct Monocular Visual Odometry," IEEE International Conference on Robotics and Automation, 2014. -->

- [TartanVO](https://github.com/castacks/tartanvo)
> Wang W, Hu Y, Scherer S. Tartanvo: A generalizable learning-based vo. InConference on Robot Learning 2021 Oct 4 (pp. 1761-1772). PMLR.

From your ros workspace directory:
```bash
mkdir -p src/robot_visual_localization/tartan_vo_ros/models && wget https://cmu.box.com/shared/static/t1a5u4x6dxohl89104dyrsiz42mvq2sz.pkl -O src/robot_visual_localization/tartan_vo_ros/models/1914.pkl
```