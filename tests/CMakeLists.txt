# Fork tests. Enabled with -DSPACECAL_BUILD_TESTS=ON; never part of the default build so the
# upstream configuration is untouched.
cmake_minimum_required(VERSION 3.24)
project(SpaceCalibratorTests CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(spacecal_blackbox_test
    ${CMAKE_SOURCE_DIR}/tests/blackbox_test.cpp
    ${CMAKE_SOURCE_DIR}/src/overlay/recorder/black_box.cpp)
target_include_directories(spacecal_blackbox_test
    PRIVATE ${CMAKE_SOURCE_DIR}/src/common
    PRIVATE ${CMAKE_SOURCE_DIR}/src/overlay/guard
    PRIVATE ${CMAKE_SOURCE_DIR}/src/overlay/recorder
    PRIVATE ${CMAKE_SOURCE_DIR}/src/driver
    PRIVATE ${CMAKE_SOURCE_DIR}/tests
    PRIVATE ${CMAKE_SOURCE_DIR}/vendor/openvr/headers)
if (NOT MSVC)
    target_link_libraries(spacecal_blackbox_test PRIVATE pthread)
endif()

add_executable(spacecal_trust_test
    ${CMAKE_SOURCE_DIR}/tests/trust_test.cpp
    ${CMAKE_SOURCE_DIR}/src/overlay/trust/plausibility.cpp
    ${CMAKE_SOURCE_DIR}/src/overlay/trust/consensus.cpp)
target_include_directories(spacecal_trust_test
    PRIVATE ${CMAKE_SOURCE_DIR}/src/common
    PRIVATE ${CMAKE_SOURCE_DIR}/src/overlay/trust
    PRIVATE ${CMAKE_SOURCE_DIR}/tests
    PRIVATE ${CMAKE_SOURCE_DIR}/vendor/eigen)
if (MSVC)
    target_compile_definitions(spacecal_trust_test PRIVATE _USE_MATH_DEFINES NOMINMAX)
endif()

# Live tab data (src/overlay/guard/live_stats.h)
add_executable(spacecal_live_test
    ${CMAKE_SOURCE_DIR}/tests/live_test.cpp
    ${CMAKE_SOURCE_DIR}/src/overlay/guard/live_stats.cpp)
target_include_directories(spacecal_live_test
    PRIVATE ${CMAKE_SOURCE_DIR}/src/overlay/guard
    PRIVATE ${CMAKE_SOURCE_DIR}/tests
    PRIVATE ${CMAKE_SOURCE_DIR}/vendor/eigen)
if (MSVC)
    target_compile_definitions(spacecal_live_test PRIVATE _USE_MATH_DEFINES NOMINMAX)
endif()

# SteamVR universe shifts (src/overlay/trust/universe_watch.h)
add_executable(spacecal_universe_test
    ${CMAKE_SOURCE_DIR}/tests/universe_test.cpp
    ${CMAKE_SOURCE_DIR}/src/overlay/trust/universe_watch.cpp)
target_include_directories(spacecal_universe_test
    PRIVATE ${CMAKE_SOURCE_DIR}/src/overlay/trust
    PRIVATE ${CMAKE_SOURCE_DIR}/tests
    PRIVATE ${CMAKE_SOURCE_DIR}/vendor/eigen)
if (MSVC)
    target_compile_definitions(spacecal_universe_test PRIVATE _USE_MATH_DEFINES NOMINMAX)
endif()

# SteamVR moving base stations, per device (src/overlay/trust/frame_corrector.h)
add_executable(spacecal_frames_test
    ${CMAKE_SOURCE_DIR}/tests/frames_test.cpp
    ${CMAKE_SOURCE_DIR}/src/overlay/trust/frame_corrector.cpp)
target_include_directories(spacecal_frames_test
    PRIVATE ${CMAKE_SOURCE_DIR}/src/overlay/trust
    PRIVATE ${CMAKE_SOURCE_DIR}/tests
    PRIVATE ${CMAKE_SOURCE_DIR}/vendor/eigen)
if (MSVC)
    target_compile_definitions(spacecal_frames_test PRIVATE _USE_MATH_DEFINES NOMINMAX)
endif()

# replay tool (same detector code as the overlay)
add_library(spacecal_replay_core STATIC
    ${CMAKE_SOURCE_DIR}/tools/replay/replay_core.cpp
    ${CMAKE_SOURCE_DIR}/src/overlay/guard/live_stats.cpp
    ${CMAKE_SOURCE_DIR}/src/overlay/trust/universe_watch.cpp
    ${CMAKE_SOURCE_DIR}/src/overlay/trust/frame_corrector.cpp
    ${CMAKE_SOURCE_DIR}/src/overlay/trust/plausibility.cpp
    ${CMAKE_SOURCE_DIR}/src/overlay/trust/consensus.cpp)
target_include_directories(spacecal_replay_core
    PUBLIC ${CMAKE_SOURCE_DIR}/src/common
    PUBLIC ${CMAKE_SOURCE_DIR}/src/overlay
    PUBLIC ${CMAKE_SOURCE_DIR}/src/overlay/trust
    PUBLIC ${CMAKE_SOURCE_DIR}/src/overlay/guard
    PUBLIC ${CMAKE_SOURCE_DIR}/tools/replay
    PUBLIC ${CMAKE_SOURCE_DIR}/vendor/eigen
    PUBLIC ${CMAKE_SOURCE_DIR}/vendor/glaze/include)
if (MSVC)
    target_compile_definitions(spacecal_replay_core PUBLIC _USE_MATH_DEFINES NOMINMAX)
endif()

add_executable(spacecal-replay ${CMAKE_SOURCE_DIR}/tools/replay/main.cpp)
target_link_libraries(spacecal-replay PRIVATE spacecal_replay_core)

add_executable(spacecal_replay_test
    ${CMAKE_SOURCE_DIR}/tests/replay_test.cpp
    ${CMAKE_SOURCE_DIR}/src/overlay/recorder/black_box.cpp)
target_include_directories(spacecal_replay_test
    PRIVATE ${CMAKE_SOURCE_DIR}/src/overlay/recorder
    PRIVATE ${CMAKE_SOURCE_DIR}/src/driver
    PRIVATE ${CMAKE_SOURCE_DIR}/tests
    PRIVATE ${CMAKE_SOURCE_DIR}/vendor/openvr/headers)
target_link_libraries(spacecal_replay_test PRIVATE spacecal_replay_core)
if (NOT MSVC)
    target_link_libraries(spacecal_replay_test PRIVATE pthread)
endif()

# pose ring (driver -> overlay shared memory) and the recorder reading it
add_executable(spacecal_pose_ring_test
    ${CMAKE_SOURCE_DIR}/tests/pose_ring_test.cpp
    ${CMAKE_SOURCE_DIR}/src/common/pose_ring.cpp
    ${CMAKE_SOURCE_DIR}/src/overlay/recorder/black_box.cpp)
target_include_directories(spacecal_pose_ring_test
    PRIVATE ${CMAKE_SOURCE_DIR}/src/common
    PRIVATE ${CMAKE_SOURCE_DIR}/src/overlay/recorder
    PRIVATE ${CMAKE_SOURCE_DIR}/src/driver
    PRIVATE ${CMAKE_SOURCE_DIR}/tests
    PRIVATE ${CMAKE_SOURCE_DIR}/vendor/openvr/headers)
if (NOT MSVC)
    target_link_libraries(spacecal_pose_ring_test PRIVATE pthread)
endif()

add_custom_target(spacecal_tests DEPENDS spacecal_blackbox_test spacecal_trust_test spacecal_replay_test spacecal_pose_ring_test spacecal_live_test spacecal_universe_test spacecal_frames_test spacecal-replay)
add_test(NAME replay COMMAND spacecal_replay_test)

enable_testing()
add_test(NAME blackbox COMMAND spacecal_blackbox_test)
add_test(NAME trust COMMAND spacecal_trust_test)
add_test(NAME pose_ring COMMAND spacecal_pose_ring_test)
add_test(NAME live COMMAND spacecal_live_test)
add_test(NAME universe COMMAND spacecal_universe_test)
add_test(NAME frames COMMAND spacecal_frames_test)
