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
include Detector/CMakeFiles/JMD_ChromaTag.dir/depend.make

# Include the progress variables for this target.
include Detector/CMakeFiles/JMD_ChromaTag.dir/progress.make

# Include the compile flags for this target's objects.
include Detector/CMakeFiles/JMD_ChromaTag.dir/flags.make

Detector/CMakeFiles/JMD_ChromaTag.dir/JMD_ChromaTag.cpp.o: Detector/CMakeFiles/JMD_ChromaTag.dir/flags.make
Detector/CMakeFiles/JMD_ChromaTag.dir/JMD_ChromaTag.cpp.o: ../Detector/JMD_ChromaTag.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Detector/CMakeFiles/JMD_ChromaTag.dir/JMD_ChromaTag.cpp.o"
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Detector" && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/JMD_ChromaTag.dir/JMD_ChromaTag.cpp.o -c "/home/lihao/workspace/opencv/ChromaTag(video)/Detector/JMD_ChromaTag.cpp"

Detector/CMakeFiles/JMD_ChromaTag.dir/JMD_ChromaTag.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/JMD_ChromaTag.dir/JMD_ChromaTag.cpp.i"
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Detector" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/lihao/workspace/opencv/ChromaTag(video)/Detector/JMD_ChromaTag.cpp" > CMakeFiles/JMD_ChromaTag.dir/JMD_ChromaTag.cpp.i

Detector/CMakeFiles/JMD_ChromaTag.dir/JMD_ChromaTag.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/JMD_ChromaTag.dir/JMD_ChromaTag.cpp.s"
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Detector" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/lihao/workspace/opencv/ChromaTag(video)/Detector/JMD_ChromaTag.cpp" -o CMakeFiles/JMD_ChromaTag.dir/JMD_ChromaTag.cpp.s

# Object files for target JMD_ChromaTag
JMD_ChromaTag_OBJECTS = \
"CMakeFiles/JMD_ChromaTag.dir/JMD_ChromaTag.cpp.o"

# External object files for target JMD_ChromaTag
JMD_ChromaTag_EXTERNAL_OBJECTS =

Detector/libJMD_ChromaTag.a: Detector/CMakeFiles/JMD_ChromaTag.dir/JMD_ChromaTag.cpp.o
Detector/libJMD_ChromaTag.a: Detector/CMakeFiles/JMD_ChromaTag.dir/build.make
Detector/libJMD_ChromaTag.a: Detector/CMakeFiles/JMD_ChromaTag.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libJMD_ChromaTag.a"
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Detector" && $(CMAKE_COMMAND) -P CMakeFiles/JMD_ChromaTag.dir/cmake_clean_target.cmake
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Detector" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/JMD_ChromaTag.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Detector/CMakeFiles/JMD_ChromaTag.dir/build: Detector/libJMD_ChromaTag.a

.PHONY : Detector/CMakeFiles/JMD_ChromaTag.dir/build

Detector/CMakeFiles/JMD_ChromaTag.dir/clean:
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Detector" && $(CMAKE_COMMAND) -P CMakeFiles/JMD_ChromaTag.dir/cmake_clean.cmake
.PHONY : Detector/CMakeFiles/JMD_ChromaTag.dir/clean

Detector/CMakeFiles/JMD_ChromaTag.dir/depend:
	cd "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/lihao/workspace/opencv/ChromaTag(video)" "/home/lihao/workspace/opencv/ChromaTag(video)/Detector" "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug" "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Detector" "/home/lihao/workspace/opencv/ChromaTag(video)/cmake-build-debug/Detector/CMakeFiles/JMD_ChromaTag.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : Detector/CMakeFiles/JMD_ChromaTag.dir/depend

