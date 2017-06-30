# ORB-YGZ-SLAM
在ORB2基础上添加IMU和直接法

# TODO
- 调节纯视觉的双目模式
- 调节带IMU的双目模式
- 对比实验
- Tracking中加重定位
- Tracking的poseopt中不优化IMU bias？
- LoopClosing还没仔细考虑
- 接口（ROS和非ROS）
- 带IMU时，回环成功之后,Track with sparse alignment会给出一个不正常的数值
- 处理视觉丢失但IMU仍有效的场合
- TrackReferenceFrame返回的匹配数量特别少？是存在什么问题吗？

# DONE 
- done: Reset之后程序会挂，要debug（Merge到Master之后好像没这问题了。。。？）
- done: 为什么有些明显还在被优化的点，没有显示成localmap中的？（不是红色，而是黑色）
- 精度较差，待调：把Cache阈值从100改到300，精度明显变好。 - 现在cache命中的点数可以在配置文件中Tracking.CacheFeatures中修改
- 原版ORB在EUROC上基本可以在每个数据集都较好的运行，但YGZ在运行过程中可能出现明显的累计误差和漂移的情况。尝试寻找原因并解决之。 - 可能是直接法本身的锅，＋加速－鲁棒
- 直接法在配准2D点时会有謎之飘移 - 似乎是因为关键帧Track过来的那些点不再是角点导致的
- 初始化没有原来那个好（前端增加多点特征点，现在跟踪的特征点分布有时候太集中于某一块了？） - 现在特征点用的单层变网格fast

# 想法
- 利用未完全三角化特征点的信息（类似于depth filter的方式）
- 丢失之后的重定位or新建地图（和轮式里程计+ORB的方式类似）
- OKVIS类型的sliding window odometry
- odometry里，初始化改进
- 把地图点参数化成单个逆深度的形式

# 日志
## 2017.4.23
- 照着DSO那边的做法处理了一下SparseImageAlgin，但是对效果还是不大满意

## 2017.4.18
- change the sophus lib into the templated version
- 将double类型的数据改成float，因为用不到那么高的精度。但g2o里内部用的double，没法改，所以只能cast一下。

## 2017.4.7
- fix: Track with Reference frame会崩溃的问题

## 2017.4.4
- 将align中的SSE替换成普通的align，对光照效果更好一些。SSE实现的有点问题。
- 将search local map direct中的cache点数量设置成可配置的参数，默认150
- 修复了UpdateLastFrame中的问题。闭环后应该不会直接丢失了。
- 增加了VIO中，视觉inlier太少时，用IMU处理Pose的机制。

## 2017.3.29
- 做了一些ORB与YGZ的对比实验。
- 增加了多层级的DSO like特征提取方式
- 修复之前提取特征点未算旋转的bug
- 结论：直接法的框架鲁棒性确实不如原版ORB，一大原因是，Track Local Map Direct是直接依赖上一步估计的位姿的，假设地图点投影误差在几个像素以内，这件事情不是总能够保证的。然而，如果有特征点，那么总可以通过计算BoW匹配，从地图点中找到合适的匹配，而无需假设位姿已知。
- 修复一个bug，在VI初始化之后，计算所有的P/V/R时，跳过那些isBad的关键帧（它的prev/next关键帧是NULL，程序会挂）

## 2017.3.28
- 添加了generate.sh，可以用这个脚本直接编译整个工程
- 修掉了原版ORB在初始化创建地图点后，未更新map point的min distance, max distance，导致Frame::InFrustum判定一直失败的问题。

## 2017.3.23
- （前两天）VI初始化所用数据拷贝一份专门计算，防止KFCull或者插入新关键帧有影响。
- 在VI初始化计算前，先做一次全局纯视觉BA，初始化完成之后（15s时间到了），再进行一次带IMU的全局BA进行优化（和LoopClosing的全局BA类似）
- 和之前的纯ORB的初始化比较，在V101上效果差不多？（不同时间点的初始化，原始ORB的收敛性好一些。这个版本的波动大一些。可能和关键帧策略有关。）

## 2017.3.22
- 修正了keyframe culling时无法删除的问题（纯视觉中）。这是由于loop closing在detectloop的时候，将keyframe的mbNotErase设置为真导致的。
- 特征提取部分增加了变网格/变阈值的FAST，现在提取更加稳定，且不容易有强行提的特征点

## 2017.3.21
- 将IMU和pixel selector合并到master
- 把图像等待加上（处理时间间隔和时间戳一致），把跟踪失败阈值调小（30->10），V101可以运行
- 问题：
	debug完。V101最后时刻仍然会丢
	TODO 初始化没有原来那个好（前端增加多点特征点，现在跟踪的特征点分布有时候太集中于某一块了？）
	KeyFrameCulling好像还有点问题，有好多没有cull成功的？

- debug完。初始化完成后，cntSuccess=0，也就是FindDirectProjection()都失败了。 
	mvpDirectMapPointsCache.size是正常的。
	但inFrustum的MapPoint点数变少
	原因：mfMinDistance/mfMaxDistance没有按照尺度更新。。。
- Cache的cntSuccess阈值从100改到300，精度明显变好些（MH01）

## 2017.3.20
- 改几个小bug
- V1_01运行OK，尺度1.0x(1.02~1.05)，RMSE~0.09m

## 2017.3.19
- 加入IMU并测试（非实时，等LocalMapping能AcceptNewKF时，才喂新的数据）
- 现存问题：
	1. 优化过程中会有stereo的观测值？（ 不满足 pFrame->mvuRight[i]<0 ）
	2. 初始化时要再仔细考虑（KF cull和使用时的关系。Cull时会改变前后帧关系，初始化应当考虑到）
- 增加了DSO like的pixel selector

## 2017.3.16-18
- jingpang
- Tracking中加入IMU：
  Tracking中有两步，第一步是SVO-align计算位姿（原来ORBSLAM中TrackWithMotionModel），这里直接预测出位姿，不处理IMU数据；第二步是TrackWithLocalMap，这里会调用PoseOptimization，在这里把IMU的信息加入位姿优化。
- Tracking中暂时没考虑重定位（TODO）。
- 使用IMU方式的一些不同：
	1. 创建新KF的策略不同。
	2. 允许localwindow中进行KFCull。
	3. Tracking第一步进行纯图像定位，第二步才利用IMU进行优化
	4. updateLocalKF中没考虑prev/nextKF

- LocalMapping中加入IMU
- VINS初始化中把cv::Mat都换成Eigen
- 调整了双目模式，现在能在EUROC上跑双目了
- 增加了intel realsense的接口，在realsense上进行了单目实验
-- 注意到的地方：
1. 特征点提取不够均匀，容易大范围空白。此时已有的特征点被挡住就会丢失。
2. 回环检测过于严格。
3. 相机原地不动时，会不断试图插入keyframe，然后被culling掉。此时地图点有些跳动，导致追踪不稳定。
4. 对旋转容易丢失。
5. 实时性较好，能达到60帧。


## 2017.3.13
- 增加了RGBD模式
- 修改了一些RGBD模式下可能带来的问题。
- 修正了系统重置后，cache中出现指针错误的问题。

## 2017.3.12
- 增加了畸变处理，现在可以较好地运行EUROC数据集了。
- 增加了一些TUM数据集和EUROC数据集上的比较结果。
- 处理了畸变带来的一些溢出bug.

## 2017.3.10 
- 修正了一个溢出的bug，在特征点位于边缘时，计算角度和描述时会导致图像数据溢出。
- 讨论了特征提取当中网格大小对结果的影响，见ORBextractor.h中网格部分的注释。
- 增加了匹配点缓存机制，见Tracking.h中的mvDirectMapPointCache，现在会优先匹配缓存区中的地图点。逻辑见search map point direct函数。

## 2017.3.9
- 引进了原生fast匹配，加速至每帧20ms左右。
- 修改了特征点提取算法，保留直接法追踪的特征点之余再加入新的特征点，同时计算它们的描述

## 2017.3.8 
- 将svo的 local map部分添加进来，见TrackLocalMapDirect 
- track过程中，在Frame.mbExtractedFeatures中区分此帧是不是已经提取了特征点。如果已经提了，则用特征点法的配准，否则使用直接法配准。

## 2017.3.5
- 将SVO的 Track Ref Frame 移植到ORB中。

## 2017.3.3
- 将位姿有关的计算替换成Sophus::SE3，修改了Thirdparty/g2o中的内容
- 全面Vector3d化，现在只有loop closing部分仍使用cv::Mat,但那一部分计算量不大，于是先保留不动了。

## 2017.3.3
- TODO(jingpang): IMU相关文件和数据结构重新组织
- g2o中加入Cholmod相关（之后优化中marginalization需要）
- 把IMU文件夹加入
- 把System/Frame加入IMU，Frame加入了部分，Tracking中加入部分使编译能通过，但Tracking中还没加入IMU相关的逻辑
- 修改了下.gitignore，加了些二进制文件和.kdev4

## 2017.3.2 
- 添加本代码至git.oschina.net 
- 将Frame, MapPoint, KeyFrame中有关代数部分，从cv::Mat修改成Sophus::SE3和eigen::Vector
- 将字典替换成ORBvoc.bin，加载更快速
- 其余部分和原版ORB-SLAM一致。安装时需要添加非模板类的Sophus库。见Thirdparty/Sophus.tar.gz
