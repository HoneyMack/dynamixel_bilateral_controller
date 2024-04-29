CMAKE_MINIMUM_REQUIRED(VERSION 3.14)
set(PROJECT_VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

set(dynamixelSDK_LIBS /usr/local/lib/libdxl_x64_cpp.so)
set(dynamixelSDK_HEADERS /usr/local/include/dynamixel_sdk)


# dynamixelのテスト用プログラムのコンパイル
add_executable(read_write read_write.cpp)
target_include_directories(read_write PUBLIC ${dynamixelSDK_HEADERS})
target_link_libraries(read_write ${dynamixelSDK_LIBS})
