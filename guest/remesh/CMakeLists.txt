# remesh.elf: meshoptimizer's voxel remesher and simplifier as a godot-sandbox
# guest, the remesh stage of RFD 2284. Standalone until dress-on's host builds
# the stage guests from sibling checkouts; then its add_stage_elf takes over.
cmake_minimum_required(VERSION 3.16)
project(remesh_guest C CXX)
set(SANDBOX_API "" CACHE PATH "godot-sandbox guest API checkout (contract-guest-runtime's sandbox-api)")
set(MESHOPTIMIZER_DIR "" CACHE PATH "meshoptimizer checkout at the manifest pin 9e1f07b1")
set(DOWNLOAD_RUNTIME_API OFF CACHE BOOL "" FORCE)
set(SANDBOX_RISCV_EXT_V OFF CACHE BOOL "" FORCE)
add_subdirectory(${SANDBOX_API}/cmake sandbox_api)
file(GLOB MESHOPT_SOURCES ${MESHOPTIMIZER_DIR}/src/*.cpp)
add_sandbox_program(remesh main.cpp lod.cpp transfer.cpp ${MESHOPT_SOURCES})
target_include_directories(remesh PRIVATE ${MESHOPTIMIZER_DIR}/src)
target_link_libraries(remesh PRIVATE -lm -lstdc++)
