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
CMAKE_SOURCE_DIR = /home/lihao/workspace/opencv/ChromaTag_location

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug

# Include any dependencies generated for this target.
include Utilities/Common/CMakeFiles/Timer.dir/depend.make

# Include the progress variables for this target.
include Utilities/Common/CMakeFiles/Timer.dir/progress.make

# Include the compile flags for this target's objects.
include Utilities/Common/CMakeFiles/Timer.dir/flags.make

Utilities/Common/CMakeFiles/Timer.dir/JMD_Utils_Timer.cpp.o: Utilities/Common/CMakeFiles/Timer.dir/flags.make
Utilities/Common/CMakeFiles/Timer.dir/JMD_Utils_Timer.cpp.o: ../Utilities/Common/JMD_Utils_Timer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Utilities/Common/CMakeFiles/Timer.dir/JMD_Utils_Timer.cpp.o"
	cd /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug/Utilities/Common && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Timer.dir/JMD_Utils_Timer.cpp.o -c /home/lihao/workspace/opencv/ChromaTag_location/Utilities/Common/JMD_Utils_Timer.cpp

Utilities/Common/CMakeFiles/Timer.dir/JMD_Utils_Timer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Timer.dir/JMD_Utils_Timer.cpp.i"
	cd /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug/Utilities/Common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lihao/workspace/opencv/ChromaTag_location/Utilities/Common/JMD_Utils_Timer.cpp > CMakeFiles/Timer.dir/JMD_Utils_Timer.cpp.i

Utilities/Common/CMakeFiles/Timer.dir/JMD_Utils_Timer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Timer.dir/JMD_Utils_Timer.cpp.s"
	cd /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug/Utilities/Common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lihao/workspace/opencv/ChromaTag_location/Utilities/Common/JMD_Utils_Timer.cpp -o CMakeFiles/Timer.dir/JMD_Utils_Timer.cpp.s

# Object files for target Timer
Timer_OBJECTS = \
"CMakeFiles/Timer.dir/JMD_Utils_Timer.cpp.o"

# External object files for target Timer
Timer_EXTERNAL_OBJECTS =

Utilities/Common/libTimer.a: Utilities/Common/CMakeFiles/Timer.dir/JMD_Utils_Timer.cpp.o
Utilities/Common/libTimer.a: Utilities/Common/CMakeFiles/Timer.dir/build.make
Utilities/Common/libTimer.a: Utilities/Common/CMakeFiles/Timer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libTimer.a"
	cd /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug/Utilities/Common && $(CMAKE_COMMAND) -P CMakeFiles/Timer.dir/cmake_clean_target.cmake
	cd /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug/Utilities/Common && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Timer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Utilities/Common/CMakeFiles/Timer.dir/build: Utilities/Common/libTimer.a

.PHONY : Utilities/Common/CMakeFiles/Timer.dir/build

Utilities/Common/CMakeFiles/Timer.dir/clean:
	cd /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug/Utilities/Common && $(CMAKE_COMMAND) -P CMakeFiles/Timer.dir/cmake_clean.cmake
.PHONY : Utilities/Common/CMakeFiles/Timer.dir/clean

Utilities/Common/CMakeFiles/Timer.dir/depend:
	cd /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lihao/workspace/opencv/ChromaTag_location /home/lihao/workspace/opencv/ChromaTag_location/Utilities/Common /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug/Utilities/Common /home/lihao/workspace/opencv/ChromaTag_location/cmake-build-debug/Utilities/Common/CMakeFiles/Timer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Utilities/Common/CMakeFiles/Timer.dir/depend

