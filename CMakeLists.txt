CMAKE_MINIMUM_REQUIRED(VERSION 3.14)
project(dxl_bilateral_controller CXX)
message("project name is automaically set as: ${PROJECT_NAME}")

set(PROJECT_VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin) # 実行ファイルの出力先
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread") # pthreadを使うためのオプション
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g")

option(BUILD_TESTS "Build tests" ON)

set(dynamixelSDK_LIBS /usr/local/lib/libdxl_x64_cpp.so)
set(dynamixelSDK_HEADERS /usr/local/include/dynamixel_sdk)


find_package(Boost REQUIRED
  COMPONENTS
    context
    filesystem
    program_options
    regex
    system
    thread
)
if(Boost_FOUND)
  message (STATUS "Found Boost")
else(Boost_FOUND)
  message(WARNING "Boost not found")
endif()

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
  message (STATUS "Found OpenCV")
else(OpenCV_FOUND)
  message(WARNING "OpenCV not found")
endif()

set(my_dynamixel_SRC
  ${PROJECT_SOURCE_DIR}/src/dxl_handler.cpp
  ${PROJECT_SOURCE_DIR}/src/cranex7_observer.cpp
)

set(my_dynamixel_HEADERS
  ${PROJECT_SOURCE_DIR}/include/dxl_handler.hpp
  ${PROJECT_SOURCE_DIR}/include/dxl_const.hpp
  ${PROJECT_SOURCE_DIR}/include/filters.hpp
  ${PROJECT_SOURCE_DIR}/include/observers.hpp
  ${PROJECT_SOURCE_DIR}/include/cranex7_observer.hpp
)


add_executable(bilateral_pdt src/bilateral_pdt.cpp ${my_dynamixel_SRC} ${my_dynamixel_HEADERS})
target_include_directories(bilateral_pdt PUBLIC include ${dynamixelSDK_HEADERS})
target_link_libraries(bilateral_pdt ${dynamixelSDK_LIBS} ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(bilateral_pd src/bilateral_pd.cpp ${my_dynamixel_SRC} ${my_dynamixel_HEADERS})
target_include_directories(bilateral_pd PUBLIC include ${dynamixelSDK_HEADERS})
target_link_libraries(bilateral_pd ${dynamixelSDK_LIBS} ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(cranex7_bilateral src/cranex7_bilateral.cpp ${my_dynamixel_SRC} ${my_dynamixel_HEADERS})
target_include_directories(cranex7_bilateral PUBLIC include ${dynamixelSDK_HEADERS})
target_link_libraries(cranex7_bilateral ${dynamixelSDK_LIBS} ${OpenCV_LIBS} ${Boost_LIBRARIES})


# test用のプログラムのコンパイル
if(BUILD_TESTS)
    add_subdirectory(test)
endif()

MESSAGE(STATUS "--------------------------------------------")
MESSAGE(STATUS "Info :")
MESSAGE(STATUS "  Project Name = ${PROJECT_NAME}")
MESSAGE(STATUS "  Version =     ${PROJECT_VERSION}")
MESSAGE(STATUS "  CMAKE_VERSION =        ${CMAKE_VERSION}")
MESSAGE(STATUS "  CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}")
MESSAGE(STATUS "  CMAKE_BUILD_TYPE =     ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "  CMAKE_INSTALL_LIBDIR = ${CMAKE_INSTALL_LIBDIR}")
message(STATUS "  PCL_VERSION =          ${PCL_VERSION}")
message(STATUS "  OpenCV_VERSION =       ${OpenCV_VERSION}")
