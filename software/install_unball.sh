#!/bin/bash
function program_is_installed {
  # set to 1 initially
  local return_=1
  # set to 0 if not found
  type $1 >/dev/null 2>&1 || { local return_=0; }
  # return value
  echo "$return_"
}

function verify_opencv {
    if pkg-config --cflags opencv; then
        echo "opencv $(echo_pass)"
    else
        printf "\e[93mLight\e[1m[WARNING]\e[21m Package not found"
        printf "\e[0m\n"
        printf "\e[1m Installing"
        printf "\e[0m\n"
        OPENCV=2.4.13
        cd ~/
        wget https://github.com/Itseez/opencv/archive/$OPENCV.zip
        unzip $OPENCV
        cd opencv-$OPENCV
        mkdir build
        cd build
        cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
        make -j $nproc
        sudo make install
        echo "Finished"
    fi
}
# display a message in red with a cross by it
# example
# echo echo_fail "No"
function echo_fail {
  # echo first argument in red
  printf "\e[31m✘ ${1}"
  # reset colours back to normal
  printf "\033[0m"
}

# display a message in green with a tick by it
# example
# echo echo_fail "Yes"
function echo_pass {
  # echo first argument in green
  printf "\e[32m✔ ${1}"
  # reset colours back to normal
  printf "\033[0m"
}

function echo_if {
  if [ $1 == 1 ]; then
    echo_pass $2
  else
    echo_fail $2
  fi
}

install_dependency() {
  declare -a argAry=("${!2}")

  for i in "${argAry[@]}"
  do
    if dpkg-query -W "$i"; then
      printf "\e[32m----------------Package $i already installed----------------"
      printf "\033[0m \n"
    else
      printf "\e[91m\e[1m[WARNING]\e[21m Package not found"
      printf "\e[0m\n"
      printf "\e[1m Installing"
      printf "\e[0m\n"
      sudo apt-get -y --force-yes install "$i"
      echo "Done installing $i!"
    fi
  done
}

devtools=(
  "build-essential"
  "cmake"
  "pkg-config"
  "git-core"
)

gtk=(
  "libgtk2.0-dev"
)

video_iopack=(
  "libavcodec-dev"
  "libavformat-dev"
  "libswscale-dev"
  "libv4l-dev"
)

python_dev=(
    "python-pip"
    "python3-pip"
    "python2.7-dev"
    "python-numpy"
    "python-dev"
    "python-opencv"
    "python-qt4"
    "python-qt4-gl"
)

freenect_libs=(
    "freeglut3-dev"
    "libxmu-dev"
    "libxi-dev"
    "libusb-1.0-0-dev"
)

freenect_ros=(
    "ros-indigo-freenect-camera"
    "ros-indigo-freenect-launch"
)

install_dependency "Developer tools and packages" devtools[@]
install_dependency "GTK development library" gtk[@]
install_dependency "Video I/O packages" video_iopack[@]
install_dependency "Python 2.7 dev tools" python_dev[@]

echo "ros $(echo_if $(program_is_installed roscore))"
rosinstall_=$(program_is_installed roscore)


if [ $rosinstall_ == 0 ]; then
    echo "Starting ROS-indigo installation"
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
    wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

    sudo apt-get update
    sudo apt-get install ros-indigo-desktop-full
    sudo rosdep init
    rosdep update
    echo "# Sourcing ROS environment variables" >> ~/.bashrc
    echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
    mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src && catkin_init_workspace
    echo "# Sourcing catkin environment variables" >> ~/.bashrc
    echo "source ~/catkin_ws/devel/setup.sh" >> ~/.bashrc
    echo "Finished"
fi

echo "$(verify_opencv)"

git clone https://github.com/OpenKinect/libfreenect.git ~/libfreenect
cd ~/libfreenect
mkdir build
cd build
cmake CMAKE_INSTALL_PREFIX=/usr/local ..
make -j $(nproc)
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
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02bf", MODE="0666"" >> 51-kinect.rules
sudo mv 51-kinect.rules /etc/udev/rules.d/51-kinect.rules

install_dependency "Freenect ROS packages" freenect_ros[@]

printf "\e[94m\e[1mFinished\033[0m\n"
install_dependency "Freenect Libs" freenect_libs[@]
