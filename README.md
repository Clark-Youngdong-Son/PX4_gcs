# PX4 Firmware for ICSL Multirotor Team #

## Contributors ##
* #### *Hoseong Seo* (hosung9009@gmail.com)
* #### *Clark Youngdong Son* (clark.y.d.son@gmail.com)

## Installation Guide ##
* #### QtCreator
$ sudo apt-get install qtcreator
* #### ros-qt dependancies
$ sudo apt-get install ros-indigo-qt-build ros-indigo-qt-create

## Make ##
In "px4_gcs" folder,

$ catkin_make

## Push button function ##
In Qtcreator with 'icsl_gcs.ui' file, add an additional push button.

Add an additional slot function in main_window.hpp file with the name 'on_NAMEOFPUSHBUTTON_clicked()' (e.g. on_pushButton_connect_ros_clicked()).

Write content in main_window.cpp file.