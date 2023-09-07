# radiation
“# radiation”


2023.02 How to use

Checked: Ubuntu 20.04.5 LTS (ROS1 only can run on up to Ubuntu 20.04 ROS1 can't run on Ubuntu 22.04) /Preperation // Install libusb-dev sudo apt-get install libusb-dev

// Create catkin work space and source folder mkdir -p catkin_ws/src/

// Initialize work space cd catkin_ws catkin_make

//Create ROS package cd src/ catkin_create_pkg radiation roscpp rospy std_msgs

//Build the package in catkin work space cd ~/catkin_ws catkin_make . ~/catkin_ws/devel/setup.bash

//libusb-dev needs sudo permission, but rosrun can't be executed sudo //So we need alternative solution //Using udev, we can specify the usb to be used without sudo permission cd /etc/udev/rules.d sudo gedit 50-my-rules.rules

//and type as following and save SUBSYSTEM=="usb", ATTRS{idVendor}=="0661", ATTRS{idProduct}=="2917", MODE="0666", GROUP="plugdev"

//You can check idVendor and idProduct with lsusb command lsusb

//The following is an example of the output of lsusb command Bus 004 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub Bus 001 Device 002: ID 0cf3:e007 Qualcomm Atheros Communications Bus 001 Device 017: ID 045e:0084 Microsoft Corp. Basic Optical Mouse Bus 001 Device 018: ID 0661:2917 Hamamatsu Photonics K.K. Bus 001 Device 022: ID 04ca:0027 Lite-On Technology Corp. USB Multimedia Keyboard Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub

//The ID is {idVendor}:{idProduct} //In this case, the idVendor of Hamamatsu Photonics K.K. is 0661, and the idProduct is 2917

// After writing the file, reload the udev rules sudo udevadm control --reload-rules

// And then unplug the device and replug it

/ Run // ex. Get sievert(update cycle:0.1s(10 Hz), total time:20s) Go to the directory

rosrun radiation radiation --g_sievert 0.1 60

//radiation.cpp : executable file //kev_sub.cpp : subscriber for KeV //sv_sub.cpp : subscriber for Sv
