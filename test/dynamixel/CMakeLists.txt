CMAKE_MINIMUM_REQUIRED(VERSION 3.14)
set(PROJECT_VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

# dynamixelのテスト用プログラムのコンパイル
add_executable(read_write read_write.cpp)
add_executable(control_regular_interval control_regular_interval.cpp ${my_dynamixel_SRC})
add_executable(identify_system identify_system.cpp ${my_dynamixel_SRC})
add_executable(measure_bulk_rw measure_bulk_rw.cpp ${my_dynamixel_SRC})
add_executable(cranex7_get_states cranex7_get_states.cpp ${my_dynamixel_SRC})