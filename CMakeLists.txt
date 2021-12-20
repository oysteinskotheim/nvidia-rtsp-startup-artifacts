cmake_minimum_required(VERSION 3.10)
project(rtsp_server)

find_package(PkgConfig REQUIRED)

pkg_check_modules(
  GSTREAMER
  REQUIRED
  gstreamer-1.0
  gstreamer-rtsp-1.0
  gstreamer-rtsp-server-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
  glib-2.0)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads)

# RTSP server (to reproduce startup artifacts)
set(RTSP_SERVER_SOURCES src/NvArgusCamera.cpp src/rtsp_server.cpp)

add_executable(rtsp_server ${RTSP_SERVER_SOURCES})
target_compile_features(rtsp_server PRIVATE cxx_std_17)
target_include_directories(rtsp_server PUBLIC ${GSTREAMER_INCLUDE_DIRS})
target_link_libraries(rtsp_server PUBLIC ${GSTREAMER_LIBRARIES}
                                         ${CMAKE_THREAD_LIBS_INIT})
install(TARGETS rtsp_server DESTINATION bin)
install(FILES service/rtsp-server.service DESTINATION /etc/systemd/system)
