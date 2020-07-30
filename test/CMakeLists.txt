cmake_minimum_required(VERSION 3.10)

# Note: add_subdirectory(googletest ...) is called in basalt-headers

include_directories(include)


# add_executable(test_spline_opt src/test_spline_opt.cpp)
# target_link_libraries(test_spline_opt gtest gtest_main basalt)

add_executable(test_vio src/test_vio.cpp)
target_link_libraries(test_vio gtest gtest_main basalt)

add_executable(test_nfr src/test_nfr.cpp)
target_link_libraries(test_nfr gtest gtest_main basalt)

add_executable(test_image src/test_image.cpp)
target_link_libraries(test_image gtest gtest_main)

add_executable(test_spline src/test_spline.cpp)
target_link_libraries(test_spline gtest gtest_main)

add_executable(test_spline_se3 src/test_spline_se3.cpp)
target_link_libraries(test_spline_se3 gtest gtest_main)

add_executable(test_camera src/test_camera.cpp)
target_link_libraries(test_camera gtest gtest_main)

add_executable(test_sophus src/test_sophus.cpp)
target_link_libraries(test_sophus gtest gtest_main)

add_executable(test_preintegration src/test_preintegration.cpp)
target_link_libraries(test_preintegration gtest gtest_main ${TBB_LIBRARIES})

add_executable(test_ceres_spline_helper src/test_ceres_spline_helper.cpp)
target_link_libraries(test_ceres_spline_helper gtest gtest_main ${TBB_LIBRARIES})


enable_testing()

include(GoogleTest)

#gtest_discover_tests(test_spline_opt DISCOVERY_TIMEOUT 60)
#gtest_discover_tests(test_vio DISCOVERY_TIMEOUT 60)
#gtest_discover_tests(test_nfr DISCOVERY_TIMEOUT 60)

# gtest_add_tests(TARGET test_spline_opt AUTO)
gtest_add_tests(TARGET test_vio AUTO)
gtest_add_tests(TARGET test_nfr AUTO)

gtest_discover_tests(test_image)
gtest_discover_tests(test_spline)
gtest_discover_tests(test_camera)
gtest_discover_tests(test_sophus)
gtest_discover_tests(test_preintegration)

