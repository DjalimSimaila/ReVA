cmake_minimum_required(VERSION 3.5)
project(CoursOpenCv)

file(GLOB_RECURSE SRC src/*)
file(GLOB_RECURSE HEADERS include/*)

add_executable(CoursOpenCv ${LIB_HEADERS} ${HEADERS} ${SRC})

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(OpenGL_GL_PREFERENCE GLVND)  
find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)
pkg_check_modules(GLEW REQUIRED glew)

target_link_directories(CoursOpenCv PUBLIC .)
target_link_libraries(CoursOpenCv ${OpenCV_LIBS} ${GLFW_LINK_LIBRARIES} ${GLEW_LINK_LIBRARIES} OpenGL::GL)
target_include_directories(CoursOpenCv PUBLIC include ${OpenCV_INCLUDE_DIRS} ${GLFW_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS})
