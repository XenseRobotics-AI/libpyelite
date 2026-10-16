cmake_minimum_required(VERSION 3.18)
project(libpyelite LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)
find_package(Threads REQUIRED)
execute_process(COMMAND "${Python_EXECUTABLE}" -c
  "import sys; from pathlib import Path; print(';'.join(str(Path(p)/'cmeel.prefix') for p in sys.path if (Path(p)/'cmeel.prefix').is_dir()))"
  OUTPUT_VARIABLE PIN_PREFIX OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ERROR_IS_FATAL ANY)
list(PREPEND CMAKE_PREFIX_PATH "${PIN_PREFIX}")
find_package(pinocchio 4.1 REQUIRED)
find_package(coal REQUIRED)
set(ELITE_COMPILE_KIN_PLUGIN OFF CACHE BOOL "" FORCE)
set(ELITE_COMPILE_TESTS OFF CACHE BOOL "" FORCE)
set(ELITE_COMPILE_EXAMPLES OFF CACHE BOOL "" FORCE)
set(ELITE_INSTALL OFF CACHE BOOL "" FORCE)
# libpyelite is a standalone Python/C++ wheel.  The parent conda environment
# may contain ROS 2 for other hardware SDKs, but this package must never enter
# the vendor SDK's optional ament/ROS build path.
set(ELITE_ROS2_BUILD OFF CACHE BOOL "" FORCE)
# Prevent the vendor CMakeLists.txt from auto-enabling ELITE_ROS2_BUILD merely
# because ament_cmake happens to be discoverable in the active environment.
set(CMAKE_DISABLE_FIND_PACKAGE_ament_cmake TRUE CACHE BOOL "" FORCE)
set(CMAKE_DISABLE_FIND_PACKAGE_libssh TRUE)
add_subdirectory(vendor/elite_cs_sdk EXCLUDE_FROM_ALL)
target_link_libraries(elite_cs_series_sdk_STATIC PRIVATE Boost::boost Boost::filesystem)
# Locked SDK uses ::round without including cmath; keep the vendor tree unmodified.
target_compile_options(elite_cs_series_sdk_STATIC PRIVATE -include cmath)
file(GLOB RUCKIG_SOURCES CONFIGURE_DEPENDS vendor/ruckig/src/ruckig/*.cpp)
list(FILTER RUCKIG_SOURCES EXCLUDE REGEX "cloud_client|python")
add_library(elite_ruckig STATIC ${RUCKIG_SOURCES})
target_include_directories(elite_ruckig PUBLIC vendor/ruckig/include)
pybind11_add_module(_native NO_EXTRAS src/pink_ik.cpp src/planner.cpp
  src/compliance.cpp src/station_collision.cpp src/projection_qp.cpp src/station_guard.cpp
  src/native_controller.cpp src/station_coordinator.cpp src/bindings.cpp)
target_include_directories(_native PRIVATE src)
target_link_libraries(_native PRIVATE elite_ruckig pinocchio::pinocchio coal::coal
  elite-cs-series-sdk::static Threads::Threads ${CMAKE_DL_LIBS})
target_compile_options(_native PRIVATE -Wall -Wextra)
set_target_properties(_native PROPERTIES
  INSTALL_RPATH "$ORIGIN/../cmeel.prefix/lib"
  INSTALL_RPATH_USE_LINK_PATH FALSE)
install(TARGETS _native LIBRARY DESTINATION libpyelite)
install(FILES vendor/elite_cs_sdk/source/resources/external_control.script DESTINATION libpyelite)
install(FILES vendor/elite_cs_sdk/LICENSE DESTINATION libpyelite/licenses RENAME elite-sdk-LICENSE)

install(FILES NOTICE.md DESTINATION libpyelite/licenses)

install(FILES vendor/ruckig/LICENSE DESTINATION libpyelite/licenses RENAME ruckig-LICENSE)
