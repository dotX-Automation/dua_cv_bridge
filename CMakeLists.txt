cmake_minimum_required(VERSION 3.12)
set(BUILD_TESTING OFF CACHE BOOL "Build tests")
project(dua_cv_bridge)

# Keep FindCUDA available to CUDA-enabled OpenCV builds
cmake_policy(SET CMP0146 OLD)

# Default to RelWithDebInfo, but let colcon override it
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fvisibility=hidden)
endif()

# Enable coverage instrumentation when testing (must precede every target definition)
if(BUILD_TESTING)
  add_compile_options(--coverage -O0 -g)
  add_link_options(--coverage)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV 4.11.0 REQUIRED)
find_package(sensor_msgs REQUIRED)

# dua_cv_bridge library configuration
add_library(dua_cv_bridge SHARED src/dua_cv_bridge.cpp)
add_library(dua_cv_bridge::dua_cv_bridge ALIAS dua_cv_bridge)
target_compile_features(dua_cv_bridge PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
target_compile_definitions(dua_cv_bridge PRIVATE "DUA_CV_BRIDGE_BUILDING_LIBRARY")
target_include_directories(dua_cv_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(dua_cv_bridge
  ${OpenCV_LIBS})
ament_target_dependencies(dua_cv_bridge sensor_msgs)

# Library installation
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})
install(
  TARGETS dua_cv_bridge
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME})

# Static analysis
if(BUILD_TESTING)
  find_program(CPPCHECK cppcheck)
  if(CPPCHECK)
    add_test(
      NAME cppcheck
      COMMAND ${CPPCHECK}
        --enable=warning,style,performance,portability
        --suppress=missingIncludeSystem
        --inline-suppr
        --error-exitcode=1
        -I ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  endif()
endif()

# Export all dependencies and library targets for this package
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  OpenCV
  sensor_msgs)

ament_package()
