cmake_minimum_required(VERSION 3.5)
project(turtlebot_gui)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Find QT packages
find_package(Qt5 REQUIRED COMPONENTS Core Widgets DataVisualization)

## Enable QT
# Make the uic, moc, and rcc Qt components be run automatically
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

## Declare ROS messages and services
# add_message_files(FILES Num.msg)
# add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()
## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(display_ui 
    src/main.cpp
    src/include/MainWindow.h
    src/include/MainWindow.cpp
    src/include/PCViewerWidget.h
    src/include/PCViewerWidget.cpp
    )
target_link_libraries(display_ui 
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::DataVisualization
    )
add_dependencies(display_ui turtlebot_gui_generate_messages_cpp)