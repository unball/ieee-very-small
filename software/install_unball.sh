echo "Starting ROS-indigo installation"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-indigo-desktop-full
sudo rosdep init
rosdep update
echo "# Sourcing ROS environment variables" >> /root/.bashrc
echo "source /opt/ros/indigo/setup.bash" >> /root/.bashrc
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src && catkin_init_workspace
echo "# Sourcing catkin environment variables" >> root/.bashrc
echo "source ~/catkin_ws/devel/setup.sh" >> root/.
echo "Finished"

echo "Starting opencv installation"
sudo apt-get install build-essential 
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
#Change opencv version on next line, need to verify in github
OPENCV=2.4.13
wget https://github.com/Itseez/opencv/archive/$OPENCV.zip
unzip $OPENCV
cd opencv-$OPENCV
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j $nproc
sudo make install
echo "Finished"

echo "Starting Freenect Installation"
sudo apt-get install git-core cmake freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libusb-1.0-0-dev
git clone git://github.com/OpenKinect/libfreenect.git
cd libfreenect
mkdir build
cd build
cmake ..
make -j $nproc
sudo make install
sudo ldconfig
sudo adduser $USER video
echo "# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666"
# ATTR{product}=="Xbox NUI Audio"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666"
# ATTR{product}=="Xbox NUI Camera"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666"
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c2", MODE="0666"
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02be", MODE="0666"
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02bf", MODE="0666"" >> /etc/udev/rules.d/51-kinect.rules
sudo apt-get install ros-indigo-freenect-camera ros-indigo-freenect-launch
echo "Finished"


