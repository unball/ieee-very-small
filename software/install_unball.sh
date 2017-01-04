#!/bin/bash

ROS_FOLDER=/opt/ros/kinetic/

configld(){
    sudo echo "$ROS_FOLDER"/lib > /etc/ld.so.conf.d/ros.conf
    sudo ldconfig -v
}

program_is_installed() {
  # set to 1 initially
  local return_=1
  # set to 0 if not found
  type $1 >/dev/null 2>&1 || { local return_=0; }
  # return value
  echo "$return_"
}

# display a message in red with a cross by it
# example
# echo echo_fail "No"
echo_fail(){
  # echo first argument in red
  printf "\e[31m✘ ${1}"
  # reset colours back to normal
  printf "\033[0m"
}

# display a message in green with a tick by it
# example
# echo echo_fail "Yes"
echo_pass(){
  # echo first argument in green
  printf "\e[32m✔ ${1}"
  # reset colours back to normal
  printf "\033[0m"
}

echo_if(){
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

install_ros(){
  echo "Starting ROS-kinetic installation"
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

  sudo apt-get update
  sudo apt-get install ros-kinetic-desktop-full
  sudo rosdep init
  rosdep update
  echo "# Sourcing ROS environment variables" >> ~/.bashrc
  echo "source /opt/ros/ kinetic/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  mkdir -p ~/catkin_ws/src; cd ~/catkin_ws/src; catkin_init_workspace
  echo "# Sourcing catkin environment variables" >> ~/.bashrc
  echo "source ~/catkin_ws/devel/setup.sh" >> ~/.bashrc
  source ~/.bashrc
  echo "Finished"
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

[ `whoami` = root ] || { sudo "$0" "$@"; exit $?; }

install_dependency "Developer tools and packages" devtools[@]
install_dependency "GTK development library" gtk[@]
install_dependency "Video I/O packages" video_iopack[@]
install_dependency "Python 2.7 dev tools" python_dev[@]

echo "ros $(echo_if $(program_is_installed roscore))"
rosinstall_=$(program_is_installed roscore)


if [ $rosinstall_ == 0 ]; then
  install_ros
  configld
fi

