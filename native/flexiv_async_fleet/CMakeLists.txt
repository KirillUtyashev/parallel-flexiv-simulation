cmake_minimum_required(VERSION 3.16.3)
project(flexiv_async_fleet LANGUAGES CXX)

get_filename_component(ISAAC_WS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(FLEXIV_VENDOR_ROOT "${ISAAC_WS_ROOT}/vendor/flexiv_sim_plugin")
set(FLEXIV_THIRDPARTY_ROOT "${FLEXIV_VENDOR_ROOT}/thirdparty")

find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 REQUIRED CONFIG)
find_package(Threads REQUIRED)

set(FLEXIV_SIM_PLUGIN_ARCHIVE
  "${FLEXIV_VENDOR_ROOT}/lib/libflexiv_sim_plugin.x86_64-linux-gnu.a"
  CACHE FILEPATH "Sim Plugin static archive used by the async fleet module")
option(FLEXIV_ASYNC_DDS_SHARED_PARTICIPANT
  "Route async UserNodes through one reference-counted DDS participant" OFF)
set(SIM_PLUGIN_ARCHIVE "${FLEXIV_SIM_PLUGIN_ARCHIVE}")
set(FLEXIV_DDS_DUMMY_PEER_ARCHIVE
  "${FLEXIV_VENDOR_ROOT}/lib/libflexiv_sim_plugin.x86_64-linux-gnu.a"
  CACHE FILEPATH "Private Sim Plugin archive used by the complementary DDS probe")
set(FLEXIV_DDS_VENDOR_PEER_ARCHIVE
  "${FLEXIV_VENDOR_ROOT}/lib/libflexiv_sim_plugin.x86_64-linux-gnu.a"
  CACHE FILEPATH "Unmodified Sim Plugin archive used by the DDS control probe")
set(FAST_DDS_ARCHIVE "${FLEXIV_THIRDPARTY_ROOT}/lib/libfastrtps.a")
set(FAST_CDR_ARCHIVE "${FLEXIV_THIRDPARTY_ROOT}/lib/libfastcdr.a")
set(FOONATHAN_ARCHIVE
  "${FLEXIV_THIRDPARTY_ROOT}/lib/libfoonathan_memory-0.7.3.a")
set(SPDLOG_ARCHIVE "${FLEXIV_THIRDPARTY_ROOT}/lib/libspdlog.a")
set(TINYXML2_ARCHIVE "${FLEXIV_THIRDPARTY_ROOT}/lib/libtinyxml2.a")

foreach(archive IN ITEMS SIM_PLUGIN_ARCHIVE FLEXIV_DDS_DUMMY_PEER_ARCHIVE
    FLEXIV_DDS_VENDOR_PEER_ARCHIVE FAST_DDS_ARCHIVE FAST_CDR_ARCHIVE
    FOONATHAN_ARCHIVE SPDLOG_ARCHIVE TINYXML2_ARCHIVE)
  if(NOT EXISTS "${${archive}}")
    message(FATAL_ERROR "Required static archive is missing: ${${archive}}")
  endif()
endforeach()

set(FLEXIV_INCLUDE_DIRS
  "${CMAKE_CURRENT_LIST_DIR}/include"
  "${FLEXIV_VENDOR_ROOT}/include"
  "${FLEXIV_THIRDPARTY_ROOT}/fastdds/generated_include"
  "${FLEXIV_THIRDPARTY_ROOT}/fastdds/include"
  "${FLEXIV_THIRDPARTY_ROOT}/asio/include"
  "${FLEXIV_THIRDPARTY_ROOT}/fastcdr/include")
pybind11_add_module(_flexiv_async_fleet MODULE
  python/async_fleet_pybind.cpp
  src/async_fleet.cpp
  src/shared_dds_participant.cpp)
target_compile_features(_flexiv_async_fleet PRIVATE cxx_std_17)
target_compile_options(_flexiv_async_fleet PRIVATE -Wall -Wextra -Wpedantic)
if(FLEXIV_ASYNC_DDS_SHARED_PARTICIPANT)
  target_compile_definitions(_flexiv_async_fleet PRIVATE
    FLEXIV_ASYNC_DDS_SHARED_PARTICIPANT=1)
endif()
target_link_options(_flexiv_async_fleet PRIVATE "LINKER:--exclude-libs,ALL")
target_include_directories(_flexiv_async_fleet PRIVATE ${FLEXIV_INCLUDE_DIRS})
target_link_libraries(_flexiv_async_fleet PRIVATE
  "-Wl,--start-group"
  "${SIM_PLUGIN_ARCHIVE}"
  "${FAST_DDS_ARCHIVE}"
  "${FAST_CDR_ARCHIVE}"
  "${FOONATHAN_ARCHIVE}"
  "${SPDLOG_ARCHIVE}"
  "${TINYXML2_ARCHIVE}"
  "-Wl,--end-group"
  Threads::Threads
  ${CMAKE_DL_LIBS}
  rt)

add_executable(flexiv_dds_dummy_peers src/flexiv_dds_dummy_peers.cpp)
add_executable(flexiv_dds_dummy_peers_vendor src/flexiv_dds_dummy_peers.cpp)
foreach(target IN ITEMS flexiv_dds_dummy_peers flexiv_dds_dummy_peers_vendor)
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(${target} PRIVATE ${FLEXIV_INCLUDE_DIRS})
  target_link_options(${target} PRIVATE "LINKER:--exclude-libs,ALL")
endforeach()
target_link_libraries(flexiv_dds_dummy_peers PRIVATE
  "-Wl,--start-group"
  "${FLEXIV_DDS_DUMMY_PEER_ARCHIVE}"
  "${FAST_DDS_ARCHIVE}"
  "${FAST_CDR_ARCHIVE}"
  "${FOONATHAN_ARCHIVE}"
  "${SPDLOG_ARCHIVE}"
  "${TINYXML2_ARCHIVE}"
  "-Wl,--end-group"
  Threads::Threads
  ${CMAKE_DL_LIBS}
  rt)
target_link_libraries(flexiv_dds_dummy_peers_vendor PRIVATE
  "-Wl,--start-group"
  "${FLEXIV_DDS_VENDOR_PEER_ARCHIVE}"
  "${FAST_DDS_ARCHIVE}"
  "${FAST_CDR_ARCHIVE}"
  "${FOONATHAN_ARCHIVE}"
  "${SPDLOG_ARCHIVE}"
  "${TINYXML2_ARCHIVE}"
  "-Wl,--end-group"
  Threads::Threads
  ${CMAKE_DL_LIBS}
  rt)
