cmake_minimum_required(VERSION 3.5)

project(exampleQtProject LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Make the uic, moc, and rcc Qt components be run automatically
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

add_executable(mainGUIwindow
    mainGUIwindow.cpp
)
target_link_libraries(mainGUIwindow
    Qt5::Core
    Qt5::Widgets
)
