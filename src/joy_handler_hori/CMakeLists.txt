cmake_minimum_required(VERSION 2.8.3)
project(joy_handler_hori)

## catkin构建时需要的组件包。
## 是依赖包，是message_generation、 std_msgs和roscpp。
## 如果这些功能包不存在，在构建过程中会发生错误。
find_package(catkin REQUIRED COMPONENTS
  joy
  message_generation
  roscpp
  std_msgs
)

## 消息声明：JoySelectedData.msg
## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   JoySelectedData.msg
#   Message2.msg
 )


## 这是设置依赖性消息的选项。
## 如果未安装std_msgs，则在构建过程中会发生错误。
generate_messages(
   DEPENDENCIES
   std_msgs
 )

## catkin功能包选项，描述了库、catkin构建依赖项和系统依赖的功能包。
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES joy_handler_HORI
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

## 设置包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

### topic_publisher节点的构建选项。
## 配置可执行文件、目标链接库和其他依赖项。
##add_dependencies(joy_handler_hori ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

### topic_publisher节点的构建选项。
## 配置可执行文件、目标链接库和其他依赖项。
add_executable(joy_handler_hori 
                  src/joy_handler_hori.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
##add_dependencies(joy_handler_hori_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joy_handler_hori ${catkin_LIBRARIES})
