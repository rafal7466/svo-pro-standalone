cmake_minimum_required(VERSION 3.16)

# --- Windows: find vcpkg automatically if no toolchain file was given ---------
if(WIN32 AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  # VCPKG_ROOT first, then the private copy made by scripts/setup_vcpkg.ps1,
  # then common install locations.
  set(_vcpkg_candidates "$ENV{VCPKG_ROOT}" "${CMAKE_CURRENT_SOURCE_DIR}/tools/vcpkg"
                        "C:/vcpkg" "C:/src/vcpkg" "C:/tools/vcpkg" "$ENV{USERPROFILE}/vcpkg")
  foreach(_c ${_vcpkg_candidates})
    if(_c AND EXISTS "${_c}/scripts/buildsystems/vcpkg.cmake")
      set(CMAKE_TOOLCHAIN_FILE "${_c}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "vcpkg toolchain")
      message(STATUS "Using vcpkg toolchain: ${CMAKE_TOOLCHAIN_FILE}")
      break()
    endif()
  endforeach()
endif()
if(WIN32 AND NOT DEFINED VCPKG_TARGET_TRIPLET)
  set(VCPKG_TARGET_TRIPLET "x64-windows" CACHE STRING "vcpkg triplet")
endif()

project(svo_pro_standalone LANGUAGES C CXX)

# -----------------------------------------------------------------------------
# SVO Pro (rpg_svo_pro_open) - standalone build, no ROS / catkin required.
# Works with MSVC (Visual Studio 2019/2022) + vcpkg on Windows and gcc/clang on
# Linux.  Dependencies: Eigen3, OpenCV, Ceres (>= 2.1), glog, gflags, yaml-cpp.
# -----------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(SVO_BUILD_TOOLS "Build the EuRoC runner application" ON)
# MSVC: number of parallel cl.exe processes per project (/MP). Eigen/Ceres heavy
# files need ~1-2 GB RAM each, so do not use all logical cores by default.
set(SVO_MSVC_MP_JOBS 8 CACHE STRING "Parallel compile jobs for MSVC (/MP), 0 = off")
if(SVO_MSVC_MP_JOBS GREATER 1)
  set(SVO_MSVC_MP_FLAG "/MP${SVO_MSVC_MP_JOBS}")
else()
  set(SVO_MSVC_MP_FLAG "")
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# ----------------------------------------------------------------------------
# Dependencies
# ----------------------------------------------------------------------------
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui video features2d calib3d)
find_package(gflags CONFIG QUIET)
if(NOT gflags_FOUND)
  find_package(gflags REQUIRED)
endif()
find_package(glog CONFIG QUIET)
if(NOT glog_FOUND)
  find_package(glog REQUIRED)
endif()
find_package(Ceres REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(YAML_CPP_TARGET yaml-cpp)
endif()
if(TARGET gflags::gflags)
  set(GFLAGS_TARGET gflags::gflags)
elseif(TARGET gflags)
  set(GFLAGS_TARGET gflags)
else()
  set(GFLAGS_TARGET ${GFLAGS_LIBRARIES})
endif()

message(STATUS "Eigen3  : ${Eigen3_VERSION}")
message(STATUS "OpenCV  : ${OpenCV_VERSION}")
message(STATUS "Ceres   : ${Ceres_VERSION}")

# ----------------------------------------------------------------------------
# Global compile settings
# ----------------------------------------------------------------------------
set(SVO_COMMON_DEFS
  SVO_USE_OPENGV
  SVO_DEPTHFILTER_IN_REPROJECTOR
  USE_OPENMP   # only makes the Ceres backend use ceres_num_threads
)

if(MSVC)
  # x64 MSVC always has SSE2 but never defines the GCC macros used by SVO/FAST.
  list(APPEND SVO_COMMON_DEFS
    __SSE2__ __SSE3__ __SSSE3__
    _USE_MATH_DEFINES NOMINMAX WIN32_LEAN_AND_MEAN
    _CRT_SECURE_NO_WARNINGS _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS
    GLOG_NO_ABBREVIATED_SEVERITIES GLOG_USE_GLOG_EXPORT
    _ENABLE_EXTENDED_ALIGNED_STORAGE)
  set(SVO_COMMON_OPTS /bigobj ${SVO_MSVC_MP_FLAG} /permissive- /Zc:__cplusplus /utf-8 /FIcassert /FIcmath
    /wd4244 /wd4267 /wd4305 /wd4996 /wd4018 /wd4800 /wd4251 /wd4275 /wd4127
    /wd4068 /wd4100 /wd4189 /wd4146 /wd4834 /wd4005 /wd4661 /wd5033)
  string(REPLACE "/Ob1" "/Ob2" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
  set(CMAKE_CXX_FLAGS_RELEASE "/O2 /Ob2 /Oi /Ot /DNDEBUG")
else()
  set(SVO_COMMON_OPTS -Wall -Wno-unused-variable -Wno-unused-but-set-variable
    -Wno-unknown-pragmas -Wno-unused-function -Wno-int-in-bool-context
    -Wno-maybe-uninitialized -Wno-sign-compare -Wno-deprecated-declarations
    -Wno-misleading-indentation -Wno-reorder -Wno-unused-local-typedefs
    -Wno-class-memaccess -Wno-deprecated-copy -Wno-catch-value
    -msse -msse2 -msse3 -mssse3)
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -fno-math-errno")
endif()

# ----------------------------------------------------------------------------
# third_party: FAST corner detector (uzh-rpg/fast_neon, SSE2 path)
# ----------------------------------------------------------------------------
add_library(svo_fast STATIC
  third_party/fast/src/fast_9.cpp
  third_party/fast/src/fast_10.cpp
  third_party/fast/src/fast_10_score.cpp
  third_party/fast/src/nonmax_3x3.cpp
  third_party/fast/src/faster_corner_10_sse.cpp)
target_include_directories(svo_fast PUBLIC third_party/fast/include)
target_compile_definitions(svo_fast PRIVATE ${SVO_COMMON_DEFS})
target_compile_options(svo_fast PRIVATE ${SVO_COMMON_OPTS})

# ----------------------------------------------------------------------------
# third_party: OpenGV (only the parts SVO needs)
# ----------------------------------------------------------------------------
set(OPENGV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/opengv)
add_library(svo_opengv STATIC
  ${OPENGV_DIR}/src/relative_pose/modules/main.cpp
  ${OPENGV_DIR}/src/relative_pose/modules/fivept_nister/modules.cpp
  ${OPENGV_DIR}/src/relative_pose/modules/fivept_stewenius/modules.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/opengv_stubs/kneip_sixpt_stubs.cpp
  ${OPENGV_DIR}/src/relative_pose/modules/eigensolver/modules.cpp
  ${OPENGV_DIR}/src/math/cayley.cpp
  ${OPENGV_DIR}/src/math/quaternion.cpp
  ${OPENGV_DIR}/src/math/arun.cpp
  ${OPENGV_DIR}/src/math/Sturm.cpp
  ${OPENGV_DIR}/src/math/roots.cpp
  ${OPENGV_DIR}/src/math/gauss_jordan.cpp
  ${OPENGV_DIR}/src/relative_pose/methods.cpp
  ${OPENGV_DIR}/src/relative_pose/CentralRelativeAdapter.cpp
  ${OPENGV_DIR}/src/relative_pose/NoncentralRelativeAdapter.cpp
  ${OPENGV_DIR}/src/triangulation/methods.cpp
  ${OPENGV_DIR}/src/point_cloud/methods.cpp
  ${OPENGV_DIR}/src/point_cloud/PointCloudAdapter.cpp
  ${OPENGV_DIR}/src/sac_problems/relative_pose/CentralRelativePoseSacProblem.cpp
  ${OPENGV_DIR}/src/sac_problems/relative_pose/NoncentralRelativePoseSacProblem.cpp
  ${OPENGV_DIR}/src/sac_problems/relative_pose/RotationOnlySacProblem.cpp
  ${OPENGV_DIR}/src/sac_problems/relative_pose/TranslationOnlySacProblem.cpp
  ${OPENGV_DIR}/src/sac_problems/point_cloud/PointCloudSacProblem.cpp)
target_include_directories(svo_opengv PUBLIC ${OPENGV_DIR}/include)
target_link_libraries(svo_opengv PUBLIC Eigen3::Eigen)
# OpenGV needs Eigen's "unsupported" modules (NonLinearOptimization).
get_target_property(_eigen_incs Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
foreach(_inc ${_eigen_incs})
  if(EXISTS "${_inc}/unsupported/Eigen/NonLinearOptimization")
    target_include_directories(svo_opengv PRIVATE "${_inc}/unsupported")
  endif()
endforeach()
if(MSVC)
  target_compile_options(svo_opengv PRIVATE /bigobj ${SVO_MSVC_MP_FLAG} /utf-8 /FIcassert /wd4514 /wd4267 /wd4244 /wd4305 /wd4996)
  target_compile_definitions(svo_opengv PRIVATE _USE_MATH_DEFINES NOMINMAX)
else()
  target_compile_options(svo_opengv PRIVATE -w)
endif()

# ----------------------------------------------------------------------------
# SVO core library (frontend + Ceres VIO backend)
# ----------------------------------------------------------------------------
set(M ${CMAKE_CURRENT_SOURCE_DIR}/modules)
file(GLOB SVO_SOURCES
  ${M}/rpg_common/src/*.cpp
  ${M}/vikit_common/src/*.cpp
  ${M}/vikit_cameras/src/*.cpp
  ${M}/vikit_solver/src/*.cpp
  ${M}/svo_common/src/*.cpp
  ${M}/svo_direct/src/*.cpp
  ${M}/svo_img_align/src/*.cpp
  ${M}/svo_tracker/src/*.cpp
  ${M}/svo/src/*.cpp
  ${M}/svo_vio_common/src/*.cpp
  ${M}/svo_ceres_backend/src/*.cpp)

add_library(svo_core STATIC ${SVO_SOURCES})
target_include_directories(svo_core PUBLIC
  ${M}/rpg_common/include
  ${M}/vikit_common/include
  ${M}/vikit_cameras/include
  ${M}/vikit_solver/include
  ${M}/svo_common/include
  ${M}/svo_direct/include
  ${M}/svo_img_align/include
  ${M}/svo_tracker/include
  ${M}/svo/include
  ${M}/svo_vio_common/include
  ${M}/svo_ceres_backend/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/minkindr/include)
target_compile_definitions(svo_core PUBLIC ${SVO_COMMON_DEFS})
target_compile_options(svo_core PRIVATE ${SVO_COMMON_OPTS})
target_link_libraries(svo_core PUBLIC
  svo_fast svo_opengv
  Eigen3::Eigen ${OpenCV_LIBS} Ceres::ceres glog::glog ${GFLAGS_TARGET}
  ${YAML_CPP_TARGET} Threads::Threads)
target_include_directories(svo_core PUBLIC ${OpenCV_INCLUDE_DIRS})

# ----------------------------------------------------------------------------
# Applications
# ----------------------------------------------------------------------------
if(SVO_BUILD_TOOLS)
  add_executable(svo_euroc
    app/svo_euroc.cpp
    app/param_server.cpp
    app/svo_factory.cpp
    app/euroc_dataset.cpp
    app/visualizer_cv.cpp
    app/trajectory_eval.cpp)
  target_include_directories(svo_euroc PRIVATE app)
  target_compile_options(svo_euroc PRIVATE ${SVO_COMMON_OPTS})
  target_link_libraries(svo_euroc PRIVATE svo_core)
endif()
