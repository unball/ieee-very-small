.. _Ubuntu: http://ubuntu.com/
.. _official website: http://www.ubuntu.com/download/desktop
.. _installation:

Installation
=============

On this subsection will be shown all necessary steps to get Vision Module's enviroment set in order to run/develop code.

Installing Ubuntu
""""""""""""""""""""""

First of all, we need to install `Ubuntu`_ [#ubuntunote]_:
        #. Download Ubuntu from `official website`_
        #. Burn the `.iso` image either in a CD-ROM, DVD-ROM or flash drive. If you decide to use the flash drive approach, take a look in `this tutorial on how to create a bootable USB stick on Windows. <http://www.ubuntu.com/download/desktop/create-a-usb-stick-on-windows>`_ 
        #. Put the media in your computer and check whether the device comes first in the BIOS boot order. Take a look in `this tutorial to learn how to change the boot order. <http://www.howtogeek.com/129815/beginner-geek-how-to-change-the-boot-order-in-your-computers-bios/>`_
        #. The Ubuntu installation script must have initialized. Choose the preferred language, mark to download updates while installing and install third-party software and to erase the current disk. You may also manually create partitions and allocate space in disk according to your preferences.
        #. The installation will also require you geographic location and keyboard layout. The keyboard layout need not be the same as the selected language. For instance, if you bought your laptop in the United States, the preferred language may be Brazilian Portuguese and the keyboard layout English (US, alternative international) to allow typing accents.
        #. Afterwards, the installation will ask your name, the computer's name (for network identification), an username and password. Do not forget this password, since it will also be the superuser password (unless changed).
        #. Finally, Ubuntu will be installed. This step requires to reboot the machine when finished.
        #. Update all packages to its last version. You can either use the Update Manager or the commands `sudo apt-get update` and `sudo apt-get upgrade.`

Installing ROS [#rosreference]_
""""""""""""""""""""""""""""""""

Add the ROS Debian packages source for Ubuntu 16.04.


.. code-block:: bash

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

Setup keys

.. code-block:: bash
    
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

Update `apt-get`

.. code-block:: bash

    sudo apt-get update

Install ROS Kinetic full

.. code-block:: bash

    sudo apt-get install ros-kinetic-desktop-full

Initialize rosdep

.. code-block:: bash
    
    sudo rosdep init
    rosdep update

In order to setup environment variables, append the following to `/.bashrc`

.. code-block:: bash
    
    # Sourcing ROS environment variables
    source /opt/ros/kinetic/setup.bash

Configuring catkin workspace
"""""""""""""""""""""""""""""
ROS recommends the usage of catkin for building it's packages. In order to achieve this, we'll create a new directory `catkin_ws`, with an inner directory `src` and a `CMakeLists.txt` indicating it is a catkin workspace.

.. code-block:: bash

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace

Also, in order to setup environment variables, append the following to `~/.bashrc`. Make sure to adapt to your usage if you don't create the workspace in the `~/catkin_ws` directory.

.. code-block:: bash

    # Sourcing catkin environment variables
    source ~/catkin_ws/devel/setup.sh

Install OpenCV
"""""""""""""""

On ROS Kinetic the OpenCV library its already installed, all you need to do is set `PKG_CONFIG_PATH` to point to ROS libs [#versionnote]_:


.. code-block:: bash

    export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/opt/ros/kinetic/lib/pkgconfig
    sudo cp /opt/ros/kinetic/lib/pkgconfig/opencv-3.1.0-dev.pc /opt/ros/kinetic/lib/pkgconfig/opencv.pc



.. rubric:: Notes
.. [#ubuntunote] A VM can be used but depending on your machine, some algorithms may behave slower than in case that the system is really installed on HDD
.. [#rosreference] `ROS installation guide <http://wiki.ros.org/kinetic/Installation/Ubuntu>`_
.. [#versionnote] ROS has keep opencv version sync with opencv repository, so the file `opencv-3.1.0-dev.pc` may be another opencv version, in case of error, just substitute `opencv-3.1.0-dev.pc` for existent `opencv-version.pc` in the folder.