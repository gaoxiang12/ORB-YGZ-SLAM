#ORB-YGZ-SLAM
This is YGZ SLAM, a faster version folked from ORB-SLAM2 (see https://github.com/raulmur/ORB_SLAM2 and the README-ORB-SLAM2.md in this repo). We put the direct tracking in SVO to accelerate the feature matching in ORB-SLAM2. We can get an average 3x speed up and keep almost same accuracy. In addition we also support monocular Visual-Inertial SLAM (VI-SLAM), following idea proposed in Raul's paper.

#Dependency
If you are using ubuntu, just type "./install_dependency.sh" to install all the dependencies except pangolin.

- Pangolin (for visualization): https://github.com/stevenlovegrove/Pangolin 
- Eigen3: sudo apt-get install libeigen3-dev
- g2o: sudo apt-get install libcxsparse-dev libqt4-dev libcholmod3.0.6 libsuitesparse-dev qt4-qmake 
- OpenCV: sudo apt-get install libopencv-dev
- glog (for logging): sudo apt-get install libgoogle-glog-dev

#Compile
run "./generate.sh" to compile all the things, or follow the steps in generate.sh

#Examples
We support all the examples in the original ORB-SLAM2, and also the monocular-inertial examples. You can try the EUROC dataset (http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) and run the monocular/mono-inertial examples by typing:

```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.bin Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE_FOLDER/mav0/cam0/data Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt 
```

just like in ORB-SLAM2. For VIO examples, try: 
```
 ./Examples/Monocular/mono_euroc_vins ./Vocabulary/ORBvoc.bin ./Examples/Monocular/EuRoC.yaml /var/dataset/euroc/V1_01_easy/cam0/data ./Examples/Monocular/EuRoC_TimeStamps/V101.txt /var/dataset/euroc/V1_01_easy/imu0/data.csv
```

to run the VIO case.

#Other things
We follow SVO when writing direct tracking, whose speed is very fast but robustness is not very good. In EUROC it can pass the test of MH01, MH02, MH03 and V101, V201. For difficult cases it can still fail. We are still improving it and writing a better solution for stereo-inertial case.

YGZ stands for Yi-Guo-Zhou (a port of porridge, a group of mess) because it contains feature method, direct method and imu things.

The Note.md is a file of develop records.

Contact me (gaoxiang12@mails.tsinghua.edu.cn) or Wang Jing (https://github.com/jingpang) for commercial use.

Thanks the following companies/people for finantial support:
- Usens凌感科技
- Hyperception 远形时空
- Qfeeltech 速感科技
- 天之博特
- 视辰信息科技
