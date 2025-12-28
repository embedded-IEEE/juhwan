cmake_minimum_required(VERSION 3.8)
project(rc_nav2_bringup)

find_package(ament_cmake REQUIRED)

# Python 패키지지만 launch/config 설치를 위해 CMakeLists가 필요합니다.
install(

  DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)
install(

  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(

  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(

  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
