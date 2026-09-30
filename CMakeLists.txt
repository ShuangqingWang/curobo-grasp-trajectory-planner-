cmake_minimum_required(VERSION 3.16)
project(OpenmindTrajectoryPlan LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---- 第三方库：优先系统，缺失时回退 /opt/openmind hybrid 打包 ----
set(THIRDPARTY_ROOT "/opt/openmind/hybrid_packages/third_party-develop-51d86920-x86_64-20260522023109")

find_library(GLOG_LIBRARY NAMES glog
    PATHS "${THIRDPARTY_ROOT}/glog_v0.5.0/lib"
    NO_DEFAULT_PATH)
find_path(GLOG_INCLUDE_DIR NAMES glog/logging.h
    PATHS "${THIRDPARTY_ROOT}/glog_v0.5.0/include"
    NO_DEFAULT_PATH)
if(NOT GLOG_LIBRARY)
    find_library(GLOG_LIBRARY NAMES glog)
endif()
if(NOT GLOG_INCLUDE_DIR)
    find_path(GLOG_INCLUDE_DIR NAMES glog/logging.h)
endif()
if(NOT GLOG_LIBRARY OR NOT GLOG_INCLUDE_DIR)
    message(FATAL_ERROR "glog not found; expected under ${THIRDPARTY_ROOT}/glog_v0.5.0")
endif()

find_library(GFLAGS_LIBRARY NAMES gflags
    PATHS "${THIRDPARTY_ROOT}/gflags-2.2.1/lib"
    NO_DEFAULT_PATH)
find_path(GFLAGS_INCLUDE_DIR NAMES gflags/gflags.h
    PATHS "${THIRDPARTY_ROOT}/gflags-2.2.1/include"
    NO_DEFAULT_PATH)
if(NOT GFLAGS_LIBRARY)
    find_library(GFLAGS_LIBRARY NAMES gflags)
endif()
if(NOT GFLAGS_INCLUDE_DIR)
    find_path(GFLAGS_INCLUDE_DIR NAMES gflags/gflags.h)
endif()
if(NOT GFLAGS_LIBRARY OR NOT GFLAGS_INCLUDE_DIR)
    message(FATAL_ERROR "gflags not found; expected under ${THIRDPARTY_ROOT}/gflags-2.2.1")
endif()

find_package(yaml-cpp REQUIRED)
find_path(NLOHMANN_JSON_INCLUDE_DIR NAMES nlohmann/json.hpp)
if(NOT NLOHMANN_JSON_INCLUDE_DIR)
    message(FATAL_ERROR "nlohmann/json.hpp not found")
endif()
find_package(Eigen3 REQUIRED)
find_package(ZLIB REQUIRED)
find_package(PNG REQUIRED)

add_executable(OpenmindTrajectoryPlan
    src/main.cpp
    src/openmind_trajectory_plan_app.cpp
    src/camera/camera_manager.cpp
    src/camera/tiff_depth_camera.cpp
    src/device/device_manager.cpp
    src/device/aubo_i12h_model.cpp
    src/pipeline/pipeline.cpp
    src/service/tcp_server.cpp
    src/service/trigger_router.cpp
    src/service/response_builder.cpp
    src/service/session_archive.cpp
    src/service/output_publisher.cpp
    src/config/config_service.cpp
    src/config/config_types.cpp
    src/tools/logger_init.cpp
    src/tools/json_util.cpp
    src/tools/log.cpp
    src/tools/interrupt.cpp
    src/algorithm/pose.cpp
    src/algorithm/robot_kinematics_interface.cpp
    src/algorithm/grasp_label_load_algorithm.cpp
    src/algorithm/grasp_pose_compose_algorithm.cpp
    src/algorithm/grasp_output_format_algorithm.cpp
    src/algorithm/depth_cloud_fuse_algorithm.cpp
    src/algorithm/collision_world_build_algorithm.cpp
    src/algorithm/curobo_plan_client.cpp
    src/algorithm/trajectory_collision_check_algorithm.cpp
    src/algorithm/trajectory_quality_grade_algorithm.cpp
    src/algorithm/trajectory_output_format_algorithm.cpp
    src/algorithm/joint_trajectory.cpp)

add_library(stage_grasp OBJECT
    src/pipeline/stage/grasp/load_request_stage.cpp
    src/pipeline/stage/grasp/load_grasp_labels_stage.cpp
    src/pipeline/stage/grasp/compose_grasp_poses_stage.cpp
    src/pipeline/stage/grasp/write_grasp_output_stage.cpp)
target_sources(OpenmindTrajectoryPlan PRIVATE $<TARGET_OBJECTS:stage_grasp>)

add_library(stage_trajectory OBJECT
    src/pipeline/stage/trajectory/load_grasp_candidates_stage.cpp
    src/pipeline/stage/trajectory/fuse_base_cloud_stage.cpp
    src/pipeline/stage/trajectory/build_collision_world_stage.cpp
    src/pipeline/stage/trajectory/plan_full_cycle_stage.cpp
    src/pipeline/stage/trajectory/write_trajectory_output_stage.cpp)
target_sources(OpenmindTrajectoryPlan PRIVATE $<TARGET_OBJECTS:stage_trajectory>)

set(OBJECT_TARGETS stage_grasp stage_trajectory)
foreach(object_target IN LISTS OBJECT_TARGETS)
    target_include_directories(${object_target} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${NLOHMANN_JSON_INCLUDE_DIR}
        ${GLOG_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIRS})
endforeach()

target_include_directories(OpenmindTrajectoryPlan PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${NLOHMANN_JSON_INCLUDE_DIR}
    ${GLOG_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS})

target_compile_definitions(OpenmindTrajectoryPlan PRIVATE
    GLOG_NO_ABBREVIATED_SEVERITIES)

target_link_libraries(OpenmindTrajectoryPlan PRIVATE
    ${GLOG_LIBRARY}
    ${GFLAGS_LIBRARY}
    yaml-cpp
    ZLIB::ZLIB
    PNG::PNG
    pthread)

set_target_properties(OpenmindTrajectoryPlan PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    BUILD_RPATH "${THIRDPARTY_ROOT}/glog_v0.5.0/lib;${THIRDPARTY_ROOT}/gflags-2.2.1/lib"
    INSTALL_RPATH "\$ORIGIN/../lib;\$ORIGIN")

add_executable(OpenmindTrajectoryTrigger
    src/trigger_tcp_main.cpp
    src/tools/json_util.cpp
    src/algorithm/pose.cpp)
target_include_directories(OpenmindTrajectoryTrigger PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${NLOHMANN_JSON_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(OpenmindTrajectoryTrigger PRIVATE yaml-cpp)
set_target_properties(OpenmindTrajectoryTrigger PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_executable(OpenmindVisualizerServer src/visualizer_server_main.cpp)
set_target_properties(OpenmindVisualizerServer PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

install(TARGETS OpenmindTrajectoryPlan OpenmindTrajectoryTrigger OpenmindVisualizerServer
    RUNTIME DESTINATION .)
install(FILES config/grasp_planner.json DESTINATION config)
install(DIRECTORY config/base DESTINATION config)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/simulation)
    install(DIRECTORY scripts/simulation DESTINATION scripts)
endif()
install(DIRECTORY scripts/轨迹运行 DESTINATION scripts)
set(CPACK_GENERATOR "TGZ")
set(CPACK_PACKAGE_NAME "openmind-trajectory-plan")
set(CPACK_PACKAGE_VERSION "1.0.0")
set(CPACK_PACKAGE_FILE_NAME "openmind-trajectory-plan")
include(CPack)
