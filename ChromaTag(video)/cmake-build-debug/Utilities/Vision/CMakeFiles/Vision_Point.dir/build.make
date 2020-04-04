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
CMAKE_SOURCE_DIR = "/home/lihao/workspace/opencv/ChromaTag(video)"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug"

# Include any dependencies generated for this target.
include Utilities/Vision/CMakeFiles/Vision_Point.dir/depend.make

# Include the progress variables for this target.
include Utilities/Vision/CMakeFiles/Vision_Point.dir/progress.make

# Include the compile flags for this target's objects.
include Utilities/Vision/CMakeFiles/Vision_Point.dir/flags.make

Utilities/Vision/CMakeFiles/Vision_Point.dir/JMD_Vision_Point.cpp.o: Utilities/Vision/CMakeFiles/Vision_Point.dir/flags.make
Utilities/Vision/CMakeFiles/Vision_Point.dir/JMD_Vision_Point.cpp.o: ../Utilities/Vision/JMD_Vision_Point.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Utilities/Vision/CMakeFiles/Vision_Point.dir/JMD_Vision_Point.cpp.o"
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Vision" && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Vision_Point.dir/JMD_Vision_Point.cpp.o -c "/home/lihao/workspace/opencv/ChromaTag(video)/Utilities/Vision/JMD_Vision_Point.cpp"

Utilities/Vision/CMakeFiles/Vision_Point.dir/JMD_Vision_Point.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Vision_Point.dir/JMD_Vision_Point.cpp.i"
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Vision" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/lihao/workspace/opencv/ChromaTag(video)/Utilities/Vision/JMD_Vision_Point.cpp" > CMakeFiles/Vision_Point.dir/JMD_Vision_Point.cpp.i

Utilities/Vision/CMakeFiles/Vision_Point.dir/JMD_Vision_Point.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Vision_Point.dir/JMD_Vision_Point.cpp.s"
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Vision" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/lihao/workspace/opencv/ChromaTag(video)/Utilities/Vision/JMD_Vision_Point.cpp" -o CMakeFiles/Vision_Point.dir/JMD_Vision_Point.cpp.s

# Object files for target Vision_Point
Vision_Point_OBJECTS = \
"CMakeFiles/Vision_Point.dir/JMD_Vision_Point.cpp.o"

# External object files for target Vision_Point
Vision_Point_EXTERNAL_OBJECTS =

Utilities/Vision/libVision_Point.a: Utilities/Vision/CMakeFiles/Vision_Point.dir/JMD_Vision_Point.cpp.o
Utilities/Vision/libVision_Point.a: Utilities/Vision/CMakeFiles/Vision_Point.dir/build.make
Utilities/Vision/libVision_Point.a: Utilities/Vision/CMakeFiles/Vision_Point.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libVision_Point.a"
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Vision" && $(CMAKE_COMMAND) -P CMakeFiles/Vision_Point.dir/cmake_clean_target.cmake
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Vision" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Vision_Point.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Utilities/Vision/CMakeFiles/Vision_Point.dir/build: Utilities/Vision/libVision_Point.a

.PHONY : Utilities/Vision/CMakeFiles/Vision_Point.dir/build

Utilities/Vision/CMakeFiles/Vision_Point.dir/clean:
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Vision" && $(CMAKE_COMMAND) -P CMakeFiles/Vision_Point.dir/cmake_clean.cmake
.PHONY : Utilities/Vision/CMakeFiles/Vision_Point.dir/clean

Utilities/Vision/CMakeFiles/Vision_Point.dir/depend:
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/lihao/workspace/opencv/ChromaTag(video)" "/home/lihao/workspace/opencv/ChromaTag(video)/Utilities/Vision" "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug" "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Vision" "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Utilities/Vision/CMakeFiles/Vision_Point.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : Utilities/Vision/CMakeFiles/Vision_Point.dir/depend

