#!/bin/bash

##Only use this script if you are facing troubles on compilation

OPENCV=opencv
CONTRIB=opencv_contrib
INSTALL_DIR=installdir
OPENCV_VERSION=3.1.0
NORMALCOLOR="\e[0m\n"
ROS_FOLDER=/opt/ros/kinetic
VISION_ROOT=""

isprograminstalled() {
  # set to 1 initially
  local return_=1
  # set to 0 if not found
  type $1 >/dev/null 2>&1 || { local return_=0; }
  # return value
  echo "$return_"
}

downloadandextract(){
    printf "\e[1m\e[34m Downloading Necessary Files"
    printf $NORMALCOLOR
    OPENCV_REPO=https://github.com/opencv/opencv/
    CONTRIB_REPO=https://github.com/opencv/opencv_contrib/ 
    SOURCE_TAR=source_code.tar.gz
    CONTRIB_TAR=contrib.tar.gz
    wget $OPENCV_REPO"archive/"$OPENCV_VERSION.tar.gz -O $SOURCE_TAR 
    wget $CONTRIB_REPO"/archive/"$OPENCV_VERSION.tar.gz -O $CONTRIB_TAR
    printf "\e[1m\e[34m Downloaded"
    printf $NORMALCOLOR
    tar -xzf $SOURCE_TAR; tar -xzf $CONTRIB_TAR
    rm $SOURCE_TAR; rm $CONTRIB_TAR
}

installopencv(){ 
      
    BUILD_DIR=build

    cd $OPENCV-$OPENCV_VERSION
    mkdir $BUILD_DIR
    cd $BUILD_DIR
    cmake -D -DOPENCV_EXTRA_MODULES_PATH=../../$CONTRIB-$OPENCV_VERSION/modules CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$ROS_FOLDER/$OPENCV-$OPENCV_VERSION/ ..
    sudo make -j$(nproc); sudo make install
}

configld(){
    sudo echo "$ROS_FOLDER"/lib > /etc/ld.so.conf.d/$OPENCV.conf
    sudo ldconfig -v
}

[ `whoami` = root ] || { sudo "$0" "$@"; exit $?; }

VISION_ROOT=$(pwd)

if ! isprograminstalled roscore; then
    printf "\e[91m\e[1m[ERROR]ROS not installed yet, exiting\n"
        printf $NORMALCOLOR
        printf "Check ROS installation on https://github.com/unball/ieee-very-small/wiki/Installation\n"
        exit
fi

if [[ -d $INSTALL_DIR ]]; then
    rm -R $INSTALL_DIR
fi

printf "\e[1m\e[34mCreating Directory for Installation"
printf $NORMALCOLOR
mkdir -p $INSTALL_DIR
cd $INSTALL_DIR
downloadandextract
installopencv
configld
mv $ROS_FOLDER/lib/pkgconfig/opencv-3.1.0-dev.pc $ROS_FOLDER/lib/pkgconfig/opencv.pc
cd $VISION_ROOT
rm -R $INSTALL_DIR

echo "Finished"

