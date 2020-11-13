cmake_minimum_required(VERSION 2.8.3)
project(snake_control)

## catkin构建时需要的组件包。
## 是依赖包，是message_generation、 std_msgs和roscpp。
## 如果这些功能包不存在，在构建过程中会发生错误。
find_package(catkin REQUIRED COMPONENTS
  joy_handler_hori
  snake_msgs
  roscpp
  gazebo_ros
)

## catkin功能包选项，描述了库、catkin构建依赖项和系统依赖的功能包。
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES snake_control
  CATKIN_DEPENDS joy_handler_hori roscpp
#  DEPENDS system_lib
)


## 设置包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## topic_publisher节点的构建选项。
## 配置可执行文件、目标链接库和其他依赖项
add_executable(snake_control_node
                  src/main.cpp
                  src/shift_control_method.cpp
                  src/snake_control_request.cpp
                  src/snake_control.cpp
                  src/winding_gait.cpp
                  src/inchworm_gait.cpp
                  src/helical_wave_propagate_motion.cpp
                  src/pedal_wave_motion.cpp
                  src/sinus_lifting.cpp
                  src/sidewinding.cpp
                  src/s_shaped_longitudnal_wave.cpp
)
#add_dependencies(snake_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(snake_control_node ${catkin_LIBRARIES})


