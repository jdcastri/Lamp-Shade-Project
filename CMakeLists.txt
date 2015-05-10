CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

project(voxelizer)

file(GLOB_RECURSE HEADER_CODE ${voxelizer_SOURCE_DIR}/include/*.h)
file(GLOB_RECURSE VOXELIZER_SRC_CODE ${voxelizer_SOURCE_DIR}/vox_source/*.cpp)
file(GLOB_RECURSE LAMP_SRC_CODE ${voxelizer_SOURCE_DIR}/lamp_source/*.cpp)

set(CMAKE_CXX_FLAGS                "-pthread -std=c++11")

find_package( OpenCV REQUIRED )
find_package( Threads REQUIRED )
ADD_EXECUTABLE(lampshade ${LAMP_SRC_CODE} ${HEADER_CODE})
ADD_EXECUTABLE(voxelizer ${VOXELIZER_SRC_CODE} ${HEADER_CODE})

target_link_libraries( lampshade ${OpenCV_LIBS} )
target_link_libraries( voxelizer ${CMAKE_THREAD_LIBS_INIT} )
