cmake_minimum_required (VERSION 2.6)
project (rgbd_kinect)

find_package( OpenCV REQUIRED )

add_executable (rgbd_kinect src/main.c)
target_link_libraries (rgbd_kinect tiobj ${OpenCV_LIBS} pthread m freenect)

install(TARGETS rgbd_kinect DESTINATION bin)
