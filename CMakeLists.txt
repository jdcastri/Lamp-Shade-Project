
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

project(voxelizer)

file(GLOB_RECURSE HEADER_CODE ${voxelizer_SOURCE_DIR}/include/*.h)
file(GLOB_RECURSE SRC_CODE ${voxelizer_SOURCE_DIR}/source/*.cpp)

find_package( OpenCV REQUIRED )
ADD_EXECUTABLE(voxelizer ${SRC_CODE} ${HEADER_CODE})
target_link_libraries( voxelizer ${OpenCV_LIBS} )

