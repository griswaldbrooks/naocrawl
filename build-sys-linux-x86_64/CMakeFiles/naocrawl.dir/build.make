# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/crrl-user1/code_projects/nao_worktree/naocrawl

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/crrl-user1/code_projects/nao_worktree/naocrawl/build-sys-linux-x86_64

# Include any dependencies generated for this target.
include CMakeFiles/naocrawl.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/naocrawl.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/naocrawl.dir/flags.make

CMakeFiles/naocrawl.dir/main.cpp.o: CMakeFiles/naocrawl.dir/flags.make
CMakeFiles/naocrawl.dir/main.cpp.o: ../main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/crrl-user1/code_projects/nao_worktree/naocrawl/build-sys-linux-x86_64/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/naocrawl.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/naocrawl.dir/main.cpp.o -c /home/crrl-user1/code_projects/nao_worktree/naocrawl/main.cpp

CMakeFiles/naocrawl.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/naocrawl.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/crrl-user1/code_projects/nao_worktree/naocrawl/main.cpp > CMakeFiles/naocrawl.dir/main.cpp.i

CMakeFiles/naocrawl.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/naocrawl.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/crrl-user1/code_projects/nao_worktree/naocrawl/main.cpp -o CMakeFiles/naocrawl.dir/main.cpp.s

CMakeFiles/naocrawl.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/naocrawl.dir/main.cpp.o.requires

CMakeFiles/naocrawl.dir/main.cpp.o.provides: CMakeFiles/naocrawl.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/naocrawl.dir/build.make CMakeFiles/naocrawl.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/naocrawl.dir/main.cpp.o.provides

CMakeFiles/naocrawl.dir/main.cpp.o.provides.build: CMakeFiles/naocrawl.dir/main.cpp.o

# Object files for target naocrawl
naocrawl_OBJECTS = \
"CMakeFiles/naocrawl.dir/main.cpp.o"

# External object files for target naocrawl
naocrawl_EXTERNAL_OBJECTS =

sdk/bin/naocrawl: CMakeFiles/naocrawl.dir/main.cpp.o
sdk/bin/naocrawl: CMakeFiles/naocrawl.dir/build.make
sdk/bin/naocrawl: CMakeFiles/naocrawl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable sdk/bin/naocrawl"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/naocrawl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/naocrawl.dir/build: sdk/bin/naocrawl
.PHONY : CMakeFiles/naocrawl.dir/build

CMakeFiles/naocrawl.dir/requires: CMakeFiles/naocrawl.dir/main.cpp.o.requires
.PHONY : CMakeFiles/naocrawl.dir/requires

CMakeFiles/naocrawl.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/naocrawl.dir/cmake_clean.cmake
.PHONY : CMakeFiles/naocrawl.dir/clean

CMakeFiles/naocrawl.dir/depend:
	cd /home/crrl-user1/code_projects/nao_worktree/naocrawl/build-sys-linux-x86_64 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/crrl-user1/code_projects/nao_worktree/naocrawl /home/crrl-user1/code_projects/nao_worktree/naocrawl /home/crrl-user1/code_projects/nao_worktree/naocrawl/build-sys-linux-x86_64 /home/crrl-user1/code_projects/nao_worktree/naocrawl/build-sys-linux-x86_64 /home/crrl-user1/code_projects/nao_worktree/naocrawl/build-sys-linux-x86_64/CMakeFiles/naocrawl.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/naocrawl.dir/depend

