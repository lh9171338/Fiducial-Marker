# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_COMMAND = /home/lihao/software/clion-2018.3.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/lihao/software/clion-2018.3.4/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lihao/workspace/opencv/AprilTag_location

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lihao/workspace/opencv/AprilTag_location/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/AprilTag_location.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/AprilTag_location.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/AprilTag_location.dir/flags.make

CMakeFiles/AprilTag_location.dir/main.cpp.o: CMakeFiles/AprilTag_location.dir/flags.make
CMakeFiles/AprilTag_location.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lihao/workspace/opencv/AprilTag_location/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/AprilTag_location.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AprilTag_location.dir/main.cpp.o -c /home/lihao/workspace/opencv/AprilTag_location/main.cpp

CMakeFiles/AprilTag_location.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AprilTag_location.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lihao/workspace/opencv/AprilTag_location/main.cpp > CMakeFiles/AprilTag_location.dir/main.cpp.i

CMakeFiles/AprilTag_location.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AprilTag_location.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lihao/workspace/opencv/AprilTag_location/main.cpp -o CMakeFiles/AprilTag_location.dir/main.cpp.s

# Object files for target AprilTag_location
AprilTag_location_OBJECTS = \
"CMakeFiles/AprilTag_location.dir/main.cpp.o"

# External object files for target AprilTag_location
AprilTag_location_EXTERNAL_OBJECTS =

AprilTag_location: CMakeFiles/AprilTag_location.dir/main.cpp.o
AprilTag_location: CMakeFiles/AprilTag_location.dir/build.make
AprilTag_location: CMakeFiles/AprilTag_location.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lihao/workspace/opencv/AprilTag_location/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable AprilTag_location"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/AprilTag_location.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/AprilTag_location.dir/build: AprilTag_location

.PHONY : CMakeFiles/AprilTag_location.dir/build

CMakeFiles/AprilTag_location.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/AprilTag_location.dir/cmake_clean.cmake
.PHONY : CMakeFiles/AprilTag_location.dir/clean

CMakeFiles/AprilTag_location.dir/depend:
	cd /home/lihao/workspace/opencv/AprilTag_location/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lihao/workspace/opencv/AprilTag_location /home/lihao/workspace/opencv/AprilTag_location /home/lihao/workspace/opencv/AprilTag_location/cmake-build-debug /home/lihao/workspace/opencv/AprilTag_location/cmake-build-debug /home/lihao/workspace/opencv/AprilTag_location/cmake-build-debug/CMakeFiles/AprilTag_location.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/AprilTag_location.dir/depend

