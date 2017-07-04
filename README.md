# PX4 Firmware for ICSL Multirotor Team #

## Contributors ##
* #### *Hoseong Seo* (hosung9009@gmail.com) ####
* #### *Clark Youngdong Son* (clark.y.d.son@gmail.com) ####

## Installation Guide ##
* #### QtCreator ####
$ sudo apt-get install qtcreator
* #### ros-qt dependancies ####
$ sudo apt-get install ros-indigo-qt-build ros-indigo-qt-create

## Make ##
* #### package ####
In "px4_gcs" folder,

$ catkin_make

## Push button function ##
* #### add a push button ####
In Qtcreator with 'icsl_gcs.ui' file, add an additional push button.
* #### define a slot function ####
Add an additional slot function in main_window.hpp file with the name 'on_NAMEOFPUSHBUTTON_clicked()' (e.g. on_pushButton_connect_ros_clicked()).
* #### fill content of the slot function ####
Write content in main_window.cpp file.