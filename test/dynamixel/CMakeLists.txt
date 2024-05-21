CMAKE_MINIMUM_REQUIRED(VERSION 3.14)
set(PROJECT_VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

# dynamixelのテスト用プログラムのコンパイル
add_executable(read_write read_write.cpp)
target_include_directories(read_write PUBLIC ${dynamixelSDK_HEADERS})
target_link_libraries(read_write ${dynamixelSDK_LIBS})

add_executable(control_regular_interval control_regular_interval.cpp ${my_dynamixel_SRC} ${my_dynamixel_HEADERS})
target_include_directories(control_regular_interval PUBLIC ${PROJECT_SOURCE_DIR}/include ${dynamixelSDK_HEADERS} )
target_link_libraries(control_regular_interval ${dynamixelSDK_LIBS} )

add_executable(identify_system identify_system.cpp ${my_dynamixel_SRC} ${my_dynamixel_HEADERS})
target_include_directories(identify_system PUBLIC ${PROJECT_SOURCE_DIR}/include ${dynamixelSDK_HEADERS} )
target_link_libraries(identify_system ${dynamixelSDK_LIBS} )

add_executable(measure_bulk_rw measure_bulk_rw.cpp ${my_dynamixel_SRC} ${my_dynamixel_HEADERS})
target_include_directories(measure_bulk_rw PUBLIC ${PROJECT_SOURCE_DIR}/include ${dynamixelSDK_HEADERS} )
target_link_libraries(measure_bulk_rw ${dynamixelSDK_LIBS} )


add_executable(cranex7_get_states cranex7_get_states.cpp ${my_dynamixel_SRC} ${my_dynamixel_HEADERS})
target_include_directories(cranex7_get_states PUBLIC ${PROJECT_SOURCE_DIR}/include ${dynamixelSDK_HEADERS} )
target_link_libraries(cranex7_get_states ${dynamixelSDK_LIBS} )