cd Thirdparty/DBoW2/
mkdir build
cd build
cmake ..
make -j4

cd ../../sophus
mkdir build
cd build
cmake ..
make -j2 

cd ../../g2o
mkdir build
cd build
cmake ..
make -j4

cd ../../fast
mkdir build
cd build
cmake ..
make -j2

cd ../../..
mkdir build
cd build
cmake ..
make -j4
