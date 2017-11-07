cmake_minimum_required(VERSION 2.8.0)
project(px4_gcs)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -std=gnu++11")

find_package(catkin REQUIRED COMPONENTS qt_build roscpp mavros_msgs geometry_msgs sensor_msgs nav_msgs)
find_package(LibXml2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread timer)
include_directories( include ${catkin_INCLUDE_DIRS} ${LIBXML2_INCLUDE_DIR} ${Boost_INCLUDE_DIR})
catkin_package()

rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here


file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp)
file(GLOB_RECURSE QT_MOC2 RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.h)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
QT4_WRAP_CPP(QT_MOC_H ${QT_MOC2})


file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

add_executable(px4_gcs ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP} ${QT_MOC_H})
target_link_libraries(px4_gcs ${QT_LIBRARIES} ${catkin_LIBRARIES} ${LIBXML2_LIBRARIES} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE} ${Boost_TIMER_RELEASE})

add_dependencies(px4_gcs optical_flow_generate_messages_cpp)
add_dependencies(px4_gcs ublox_msgs_generate_messages_cpp)
add_dependencies(px4_gcs dji_sdk_generate_messages_cpp)

install(TARGETS px4_gcs RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
