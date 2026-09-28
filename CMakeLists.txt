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
set(ELITE_COMPILE_KIN_PLUGIN OFF CACHE BOOL "" FORCE)
set(ELITE_COMPILE_TESTS OFF CACHE BOOL "" FORCE)
set(ELITE_COMPILE_EXAMPLES OFF CACHE BOOL "" FORCE)
set(ELITE_INSTALL OFF CACHE BOOL "" FORCE)
set(ELITE_ROS2_BUILD OFF CACHE BOOL "" FORCE)
set(CMAKE_DISABLE_FIND_PACKAGE_libssh TRUE)
add_subdirectory(vendor/elite_cs_sdk EXCLUDE_FROM_ALL)
target_link_libraries(elite_cs_series_sdk_STATIC PRIVATE Boost::boost Boost::filesystem)
# Locked SDK uses ::round without including cmath; keep the vendor tree unmodified.
target_compile_options(elite_cs_series_sdk_STATIC PRIVATE -include cmath)
pybind11_add_module(_native NO_EXTRAS src/pink_ik.cpp src/planner.cpp
  src/native_controller.cpp src/bindings.cpp)
target_include_directories(_native PRIVATE src)
target_link_libraries(_native PRIVATE pinocchio::pinocchio
  elite-cs-series-sdk::static Threads::Threads ${CMAKE_DL_LIBS})
target_compile_options(_native PRIVATE -Wall -Wextra)
set_target_properties(_native PROPERTIES
  INSTALL_RPATH "$ORIGIN/../cmeel.prefix/lib"
  INSTALL_RPATH_USE_LINK_PATH FALSE)
install(TARGETS _native LIBRARY DESTINATION libpyelite)
install(FILES vendor/elite_cs_sdk/source/resources/external_control.script DESTINATION libpyelite)
install(FILES vendor/elite_cs_sdk/LICENSE DESTINATION libpyelite/licenses RENAME elite-sdk-LICENSE)

install(FILES NOTICE.md DESTINATION libpyelite/licenses)
