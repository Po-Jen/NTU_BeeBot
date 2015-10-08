#! /bin/bash

clear
ros_workspace=~/indigo_ws  # Modify to the directory of your ROS workspace
ros_version=indigo  # Modify to your ROS version
echo "================================================================="
rosdep install --from-paths src --ignore-src --rosdistro ${ros_version} -y
echo "Start the installation of OpenCV 2.4.9"
sudo apt-get update
sudo apt-get install build-essential libgtk2.0-dev libjpeg-dev libtiff4-dev libjasper-dev libopenexr-dev cmake python-dev python-numpy python-tk libtbb-dev libeigen3-dev yasm libfaac-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev libqt4-dev libqt4-opengl-dev sphinx-common texlive-latex-extra libv4l-dev libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev default-jdk ant libvtk5-qt4-dev
cd ~
wget http://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.4.9/opencv-2.4.9.zip
unzip opencv-2.4.9.zip
cd opencv-2.4.9
mkdir build
cd build
cmake -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_VTK=ON ..
make -j4
sudo make install
sudo sh -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig
sudo sh -c 'echo "PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig" > /etc/bash.bashrc'
sudo sh -c 'echo "export PKG_CONFIG_PATH" > /etc/bash.bashrc'
echo "OpenCV 2.4.9 ready to be used"
echo "==================================================================="
cd ~
echo "Start the installation of libserial"
sudo apt-get install libserial-dev
echo "Libserial ready to be used"
echo "===================================================================="
echo "Start the installation of Gstreamer"
sudo add-apt-repository ppa:gstreamer-developers/ppa
sudo apt-get update
sudo apt-get install gstreamer*
echo "Gstreamer ready to be used"
echo "===================================================================="
echo "Start the installation of g2o"
sudo apt-get install build-essential cmake libcholmod2.1.2 libsuitesparse-metis-3.1.0 libsuitesparse-metis-dbg libsuitesparse-metis-dev freeglut3 freeglut3-dev freeglut3-dbg qt4-qmake libqglviewer2 libqglviewer-dev libqglviewer-doc libeigen3-dev libeigen3-doc
cmake ../g2o
make
sudo make install
echo "g2o ready to be used"
echo "===================================================================="
echo "Start installing necessary ROS packages from remote repositories"
echo "===================================================================="
echo "Start installing RTABMap"
cd ~
echo "installing dependencies"
sudo apt-get install ros-${ros_version}-navigation
sudo apt-get install build-essential
sudo apt-get install libeigen3-dev libsvm-dev python-numpy python-scipy ros-${ros_version}-openni-launch ros-${ros_version}-openni2-launch ros-${ros_version}-cmake-modules ros-${ros_version}-eigen-conversions
echo "installing the package"
git clone https://github.com/introlab/rtabmap.git rtabmap
cd rtabmap/build
cmake -DCMAKE_INSTALL_PREFIX=${ros_workspace}/devel ..
make -j4
make install
git clone https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
catkin_make
echo "RTABMap ready to be used"
echo "===================================================================="
echo "Start installing Spencer_people_tracking"
git clone https://github.com/Megacephalo/spencer_people_tracking.git src/spencer_people_tracking
catkin_make
echo "Spencer_people_tracking ready to be used"
echo "===================================================================="
echo "updating again"
sudo apt-get update
echo "Finish system update. Reboot to finish configuration."
echo "===================================================================="




