cmake_minimum_required(VERSION 2.8)
project(naocrawl)

set(BOOST_DIR "/usr/include/")
find_package(qibuild)

# Set project include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Bring Robotlib into the fold
set(ROBOTLIB_DIR /home/crrl-user1/code_projects/robotlib)
include_directories("${ROBOTLIB_DIR}/include")
link_directories("${ROBOTLIB_DIR}/lib")

# Get Boost
set(ROBOTLIB_BOOSTLIBS boost_system boost_thread pthread)

include_directories( ${Boost_INCLUDE_DIR} )

# Set variable for the sources to compile
set(_srcs
	main.cpp
	src/naopp.cpp
)

# Create a executable named naocrawl
# with the source file: main.cpp
qi_create_bin(naocrawl ${_srcs})

# Add libraries
target_link_libraries(naocrawl ROBOTLIB ${ROBOTLIB_BOOSTLIBS})

# Qibuild Dependencies
qi_use_lib(naocrawl ALCOMMON ALPROXIES ALMATH)

# Add a simple test:
enable_testing()
qi_create_test(test_naocrawl "test.cpp")