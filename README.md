# Awesome-LiDAR-Camera-Calibration

[![Awesome](https://cdn.rawgit.com/sindresorhus/awesome/d7305f38d29fed78fa85652e3a63e154dd8e8829/media/badge.svg)](https://github.com/sindresorhus/awesome)

A Collection of LiDAR-Camera-Calibration Papers, Toolboxes and Notes. 

**Outline**
- [Awesome-LiDAR-Camera-Calibration](#awesome-lidar-camera-calibration)
  - [0. Introduction](#0-introduction)
  - [1. Target-based methods](#1-target-based-methods)
  - [2. Targetless methods](#2-targetless-methods)
    - [2.1. Motion-based methods](#21-motion-based-methods)
    - [2.2. Scene-based methods](#22-scene-based-methods)
      - [2.2.1. Traditional feature-based methods](#221-traditional-feature-based-methods)
      - [2.2.2 Deep feature-based methods](#222-deep-feature-based-methods)
      - [2.2.3 E2E learning-based methods](#223-e2e-learning-based-methods)

## 0. Introduction
For applications such as autonomous driving, robotics, navigation systems, and 3-D scene reconstruction, data of the same scene is often captured using both lidar and camera sensors. To accurately interpret the objects in a scene, it is necessary to fuse the lidar and the camera outputs together. Lidar camera calibration estimates a **rigid transformation matrix (extrinsics, rotation+translation, 6 DoF)** that establishes the correspondences between the points in the 3-D lidar plane and the pixels in the image plane. 

![Example](https://github.com/Deephome/Awesome-LiDAR-Camera-Calibration/blob/main/projection.png)

## 1. Target-based methods
|Paper|Target|Feature|Optimization|Toolbox|Note|
| --- | --- | --- | --- | --- | --- |
| [Extrinsic Calibration of a Camera and Laser Range Finder (improves camera calibration), 2004](http://lars.mec.ua.pt/public/LAR%20Projects/Modelling/2011_MiguelAlmeida/Documentos%20leitura/Extrinsic%20Calibration%20of%20a%20Camera%20and%20Laser.pdf) | checkerboard |C:Plane (a), L: pts in plane (m)| point-to-plane | [CamLaserCalibraTool](https://github.com/MegviiRobot/CamLaserCalibraTool) | [CN](https://zhuanlan.zhihu.com/p/137501892)
| [Fast Extrinsic Calibration of a Laser Rangefinder to a Camera, 2005](http://www.cs.cmu.edu/~ranjith/lcct.html) | checkerboard| C: Plane (a), L: Plane (m)| plane(n/d) correspondence, point-to-plane | [LCCT](http://www.cs.cmu.edu/~ranjith/lcct.html) | * |
|[Extrinsic calibration of a 3D laser scanner and an omnidirectional camera, 2010]()|checkerboard|C: plane (a), L: pts in plane (m)|point-to-plane|[cam_lidar_calib](https://github.com/SubMishMar/cam_lidar_calib)|*|
|[LiDAR-Camera Calibration using 3D-3D Point correspondences, 2017](https://arxiv.org/abs/1705.09785)| cardboard + ArUco | C: 3D corners (a), L: 3D corners (m) | ICP | [lidar_camera_calibration](https://github.com/ankitdhall/lidar_camera_calibration) | * |
|[Reflectance Intensity Assisted Automatic and Accurate Extrinsic Calibration of 3D LiDAR and Panoramic Camera Using a Printed Chessboard, 2017](http://www.mdpi.com/2072-4292/9/8/851/htm)| checkerboard | C: 2D corners (a), L: 3D corners (a) |PnP, angle difference|[ILCC](https://github.com/mfxox/ILCC)|* |
|[Extrinsic Calibration of Lidar and Camera with Polygon, 2018](https://ram-lab.com/papers/2018/liao_robio2018.pdf) | regular cardboard |C: 2D edge, corners (a), L: 3D edge, pts in plane (a) | point-to-line, point-inside-polygon | [ram-lab/plycal](https://github.com/ram-lab/plycal) | * |
|[Automatic Extrinsic Calibration of a Camera and a 3D LiDAR using Line and Plane Correspondences, 2018](http://www.cs.cmu.edu/~kaess/pub/Zhou18iros.pdf)| checkerboard | C: 3D edge, plane(a), L: 3D edge, pts in plane (a) | direcion/normal, point-to-line, point-to-plane | [Matlab LiDAR Toolbox](https://ww2.mathworks.cn/help/lidar/ug/lidar-and-camera-calibration.html)  |*| 
|[Improvements to Target-Based 3D LiDAR to Camera Calibration, 2020](https://arxiv.org/pdf/1910.03126v3.pdf)|cardboard with ArUco|C: 2d corners (a), L: 3D corners (a) | PnP, IOU | [github](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration)|*|
|[ACSC: Automatic Calibration for Non-repetitive Scanning Solid-State LiDAR and Camera Systems, 2020](https://arxiv.org/pdf/2011.08516.pdf)|checkerboard | C: 2D corners (a), L: 3D corners (a) | PnP | [ACSC](https://github.com/HViktorTsoi/ACSC) | * |
|[Automatic Extrinsic Calibration Method for LiDAR and Camera Sensor Setups, 2021](https://arxiv.org/abs/2101.04431)|cardboard with circle & Aruco| C: 3D points (a), L: 3D points (a) | ICP | [velo2cam_ calibration](https://github.com/beltransen/velo2cam_calibration)|*|
|[Single-Shot is Enough: Panoramic Infrastructure Based Calibration of Multiple Cameras and 3D LiDARs,2021 IROS](https://arxiv.org/pdf/2103.12941.pdf)|panoramic infrastructure|C: CCTag(a), L: corner points and vectors(a)|PnP, ICP|[multiple-cameras-and-3D-LiDARs-extrinsic-calibration](https://github.com/alibaba/multiple-cameras-and-3D-LiDARs-extrinsic-calibration)|*|
|[Joint Camera Intrinsic and LiDAR-Camera Extrinsic Calibration, 2023, ICRA](https://arxiv.org/pdf/2202.13708.pdf)|chessboard + circles | C: chessboard corners, circle centers, L: circle centers | PnP, intrinsic constraints (corners, cirlce center)| [OpenCalib/JointCalib](https://github.com/OpenCalib/JointCalib) | [CN](https://mp.weixin.qq.com/s/AcygRdkTlb_X9Rn8jSYXAQ) |
|[LCE-Calib: Automatic LiDAR-Frame/Event Camera Extrinsic Calibration With a Globally Optimal Solution](https://arxiv.org/pdf/2303.09825.pdf) | checkboard | 3D point-to-3D plan, 3D point-to-3D edge | point-to-plane, point-to-edge | LCECalib | *|

> C: camera, L: LiDAR, a: automaic, m: manual

## 2. Targetless methods  

### 2.1. Motion-based methods
|Paper|Feature|Optimization|Code|
| --- | --- | --- | --- |
|[LiDAR and Camera Calibration Using Motions Estimated by Sensor Fusion Odometry, 2018](https://arxiv.org/pdf/1804.05178.pdf)|C: motion (ICP),  L: motion (VO) | hand-eye calibration | N/A |

### 2.2. Scene-based methods
#### 2.2.1. Traditional feature-based methods
|Paper|Feature|Optimization|Code|
| --- | --- | --- | --- |
|[Automatic Targetless Extrinsic Calibration of a 3D Lidar and Camera by Maximizing Mutual Information, 2012](http://robots.engin.umich.edu/publications/gpandey-2012a.pdf)|C：grayscale, L: reflectivity| mutual information, BB steepest gradient ascent | [Extrinsic Calib](http://robots.engin.umich.edu/SoftwareData/ExtrinsicCalib)|
|[Automatic Calibration of Lidar and Camera Images using Normalized Mutual Information, 2013](http://www-personal.acfr.usyd.edu.au/jnieto/Publications_files/TaylorICRA2013.pdf)|C:grayscale, L: reflectivity, noraml | normalized MI, particle swarm | N/A |
|[Automatic Online Calibration of Cameras and Lasers, 2013](http://www.roboticsproceedings.org/rss09/p29.pdf)|C: Canny edge, L: depth-discontinuous edge| correlation, grid search | N/A |
|[SOIC: Semantic Online Initialization and Calibration for LiDAR and Camera, 2020](https://arxiv.org/pdf/2003.04260)|semantic centroid| PnP|N/A|
|[A Low-cost and Accurate Lidar-assisted Visual SLAM System, 2021](http://www.zywok.com:20441/index.php/s/xEqCafR6dEp4REZ)|C: edge(grayscale), L: edge (reflectivity, depth projection) | ICP,  coordinate descent algorithms | [CamVox](https://github.com/ISEE-Technology/CamVox) |
|[Pixel-level Extrinsic Self Calibration of High Resolution LiDAR and Camera in Targetless Environments,2021](http://arxiv.org/abs/2103.01627)|C:Canny edge(grayscale), L: depth-continuous edge|point-to-line, Gaussian-Newton|[livox_camera_calib](https://github.com/hku-mars/livox_camera_calib)|
|[CRLF: Automatic Calibration and Refinement based on Line Feature for LiDAR and Camera in Road Scenes, 2021](https://arxiv.org/pdf/2103.04558v1.pdf)|C:straight line, L: straight line | perspective3-lines (P3L) | N/A |
|[Fast and Accurate Extrinsic Calibration for Multiple LiDARs and Cameras,2021](https://arxiv.org/pdf/2109.06550.pdf)|C:Canny edge(grayscale), L: depth-continuous edge|point-to-plane, point-to-edge|[mlcc](https://github.com/hku-mars/mlcc)|
|[General, Single-shot, Target-less, and Automatic LiDAR-Camera Extrinsic Calibration Toolbox,ICRA 2023](https://arxiv.org/pdf/2302.05094.pdf)|C:keypoints, L: keypoints on intensity images (by SuperGlue)|reprojection error minimization, RANSAC|[direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration)|
| | [Robust Calibration of Vehicle Solid-State Lidar-Camera Perception System Using Line-Weighted Correspondences in Natural Environments](https://ieeexplore.ieee.org/abstract/document/10315057) | TITS'24 |
| P2O-Calib | [P2O-Calib: Camera-LiDAR Calibration Using
Point-Pair Spatial Occlusion Relationship](https://arxiv.org/abs/2311.02413) | IROS'23 |
| | [Online Camera–LiDAR Calibration Monitoring and Rotational Drift Tracking](https://ieeexplore.ieee.org/document/10374278) | TRO'23 | [Code](https://github.com/moravecj/OCaMo)
| | [Online Calibration Between Camera and LiDAR With Spatial-Temporal Photometric Consistency](https://ieeexplore.ieee.org/abstract/document/10354343/) | RA-L'24 | [Code](https://github.com/NERanger/online_lidar_cam_calib)

#### 2.2.2 Deep feature-based methods
|Shortname |Paper|Source| Code|
| --- | --- | --- | --- |
| Se-Calib | [SE-Calib: Semantic Edge-Based LiDAR–Camera Boresight Online Calibration in Urban Scenes](https://ieeexplore.ieee.org/document/10129871) | TGRS'23 | |
| Calib-Depth| [CalibDepth: Unifying Depth Map Representation for Iterative LiDAR-Camera Online Calibration](https://ieeexplore.ieee.org/document/10161575) | ICRA'23 | [Code](https://github.com/Brickzhuantou/CalibDepth)
| DEDgeNet | [DEdgeNet: Extrinsic Calibration of Camera and LiDAR with Depth-discontinuous Edges](https://ieeexplore.ieee.org/document/10160910) | IROS'23 | 
| | [Automatic Targetless Calibration for LiDAR and Camera Based on Instance Segmentation](https://ieeexplore.ieee.org/document/9830826)| RA-L'23|
| Edge-Calib | [EdgeCalib: Multi-Frame Weighted Edge Features for Automatic Targetless LiDAR-Camera Calibration](https://ieeexplore.ieee.org/document/10669166) | RA-L'24 | 
| Calib-Anything |[Calib-Anything: Zero-training LiDAR-Camera Extrinsic Calibration Method Using Segment Anything](https://arxiv.org/pdf/2306.02656.pdf)|ICRA'24| [Code](https://github.com/OpenCalib/CalibAnything)|
| SG-Calib | [SGCalib: A Two-stage Camera-LiDAR Calibration Method Using Semantic Information and Geometric Features](https://ieeexplore.ieee.org/abstract/document/10610560) | ICRA'24 |
| OL-Reg | [OL-Reg: Registration of Image and Sparse LiDAR Point Cloud With Object-Level Dense Correspondences](https://ieeexplore.ieee.org/abstract/document/10463050) | TCSVT'24 | [Code](https://github.com/anpei96/ol-reg-demo)
| SOAC | [SOAC: Spatio-Temporal Overlap-Aware Multi-Sensor Calibration using Neural Radiance Fields](https://arxiv.org/abs/2311.15803) | CVPR'24 | 
| | [From Chaos to Calibration: A Geometric Mutual Information Approach to Target-Free Camera LiDAR Extrinsic Calibration](https://openaccess.thecvf.com/content/WACV2024/papers/Borer_From_Chaos_to_Calibration_A_Geometric_Mutual_Information_Approach_To_WACV_2024_paper.pdf) | WACV'24 |
| Unical | [UniCal: Unified Neural Sensor Calibration](https://waabi.ai/unical/) | ECCV'24 | 


#### 2.2.3 E2E learning-based methods

|Shortname|Paper|Source|Code|
| --- | --- | --- | --- |
| Sem-Calib | [Online Camera-LiDAR Calibration with Sensor Semantic Information](https://ieeexplore.ieee.org/abstract/document/9196627) | ICRA'20 | 
| RegNet |[RegNet: Multimodal sensor registration using deep neural networks, 2017,IV](https://arxiv.org/pdf/1707.03167.pdf)|IV'2017|[Code](https://github.com/aaronlws95/regnet)|
| CalibNet | [CalibNet: Geometrically supervised extrinsic calibration using 3d spatial transformer networks](https://arxiv.org/pdf/1803.08181.pdf)| IROS'18 |[Code](https://github.com/epiception/CalibNet)|
| LCCNet | [LCCNet: Lidar and Camera Self-Calibration using CostVolume Network](https://openaccess.thecvf.com/content/CVPR2021W/WAD/papers/Lv_LCCNet_LiDAR_and_Camera_Self-Calibration_Using_Cost_Volume_Network_CVPRW_2021_paper.pdf)| CVPR-W'21|[LCCNet](https://github.com/IIPCVLAB/LCCNet)|
| RGGNet | [RGGNet: Tolerance Aware LiDAR-Camera Online Calibration With Geometric Deep Learning and Generative Model](https://ieeexplore.ieee.org/abstract/document/9206138) | RA-L'21 | [Code](https://github.com/KleinYuan/RGGNet)
| CalibFormer | [CalibFormer: A Transformer-based Automatic LiDAR-Camera Calibration Network](https://arxiv.org/pdf/2306.02656.pdf)| ICRA'24 | |
| RLCA | [Robust LiDAR-Camera Alignment With Modality Adapted Local-to-Global Representation](https://ieeexplore.ieee.org/abstract/document/9852266) | TCSVT'23|
| | [A Robust LiDAR-Camera Self-Calibration Via Rotation-Based Alignment and Multi-Level Cost Volume](https://ieeexplore.ieee.org/document/10328060) | RA-L'24 |
| RobustCalib | [RobustCalib: Robust Lidar-Camera Extrinsic Calibration with Consistency Learning](https://arxiv.org/abs/2312.01085) | CoRR'24 ? |



