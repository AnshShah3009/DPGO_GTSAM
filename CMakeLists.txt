cmake_minimum_required(VERSION 3.15)

set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_GTSAM_USE_SYSTEM_EIGEN TRUE)

project(DPGO_GTSAM)

find_package(GTSAM REQUIRED)

find_package(OpenCV REQUIRED)

find_package(Ceres REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(Boost REQUIRED)

include_directories(${GTSAM_LIBRARY_DIRS} ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIR} )

add_executable(Main src/main.cpp)
add_executable(Trial src/trial.cpp)
add_executable(Main2 src/main2.cpp)
add_executable(GNC src/Gnc.cpp)
add_executable(GNC2 src/GNC.cpp)
add_executable(DS src/Dynamic_Static.cpp)
message(Eigen3 : ${EIGEN3_LIBRARY_DIRS})
message(Opencv : ${OpenCV_LIBS})
target_link_libraries(Main gtsam ${OpenCV_LIBS})
target_link_libraries(Trial gtsam ${OpenCV_LIBS})
target_link_libraries(Main2 gtsam ${OpenCV_LIBS})
target_link_libraries(GNC gtsam ${OpenCV_LIBS})
target_link_libraries(GNC2 gtsam ${OpenCV_LIBS})
target_link_libraries(DS gtsam ${OpenCV_LIBS})